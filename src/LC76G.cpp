/*
 * ============================================================================
 *  LC76G.cpp  -  Core driver for the Quectel LC76G GNSS module
 * ============================================================================
 *  Part of the Quectel_LCx6G_GNSS Arduino library. Author: Ugo Silato. License: MIT.
 * ============================================================================
 */

#include "LC76G.h"
#include <string.h>   // strncmp(), strchr(), strlen(), strncpy()
#include <stdlib.h>   // atoi(), atol(), strtol()

// Baud rates tried when the module does not answer at the target speed.
// The PAIR864 command is sent blindly at each of them: at a wrong speed the
// module only sees garbage with a bad checksum and ignores it.
// Speeds above 115200 are left out: SoftwareSerial cannot transmit them.
static const uint32_t PROBE_BAUDS[] = { 115200, 9600, 19200, 38400, 57600 };


// ===========================================================================
//  CONSTRUCTION
// ===========================================================================

LC76G::LC76G()
  : _port(nullptr),
    _setHostBaud(nullptr),
    _baud(LC76G_DEFAULT_BAUD),
    _pwrPin(-1),
    _pwrMode(LC76G_DRIVE_PUSH_PULL_HIGH),
    _rstPin(-1),
    _rstMode(LC76G_DRIVE_OPEN_DRAIN_LOW),
    _lineLen(0),
    _inLine(false),
    _lastSentenceMs(0),
    _waitId(-1),
    _waitResult(LC76G_TIMEOUT),
    _respBuf(nullptr),
    _respSize(0),
    _sentenceCb(nullptr),
    _retries(1),
    _jamStatus(LC76G_JAM_UNKNOWN),
    _jamStatusMs(0),
    _jamPin(-1),
    _epeMs(0),
    _waitPqtm(nullptr),
    _geoStatusMs(0),
    _geoPin(-1),
    _locusActive(false),
    _locusDumping(false),
    _locusCb(nullptr),
    _locusRecCb(nullptr),
    _locusDumpDone(false),
    _locusRecords(0),
    _locusReceived(0),
    _locusRejected(0),
    _locusLastMs(0) {
  memset(_geoState, 0, sizeof(_geoState));
  memset(&_epe, 0, sizeof(_epe));
  memset(_hooks, 0, sizeof(_hooks));           // No output message decoded yet
}


// ===========================================================================
//  CONTROL PINS
// ===========================================================================

void LC76G::drivePin(int8_t pin, LC76G_DriveMode mode, bool asserted) {
  if (pin < 0) {
    return;                                    // Pin not configured
  }

  switch (mode) {
    case LC76G_DRIVE_PUSH_PULL_HIGH:
      pinMode(pin, OUTPUT);
      digitalWrite(pin, asserted ? HIGH : LOW);
      break;

    case LC76G_DRIVE_WEAK_PULLUP_HIGH:
      if (asserted) {
        // Internal pull-up = current-limited HIGH, safe for a bare NPN base
        pinMode(pin, INPUT_PULLUP);
      } else {
        // ORDER MATTERS on AVR: in INPUT_PULLUP mode the output latch is
        // HIGH. Writing LOW first clears it, so switching to OUTPUT never
        // produces a HIGH pulse into the base. The second write makes the
        // level explicit on cores (e.g. Renesas) where pinMode() may not
        // preserve the previous latch value.
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
      }
      break;

    case LC76G_DRIVE_OPEN_DRAIN_LOW:
      if (asserted) {
        digitalWrite(pin, LOW);                // Latch LOW (and pull-up off on AVR)
        pinMode(pin, OUTPUT);                  // Pull the line LOW
        digitalWrite(pin, LOW);
      } else {
        pinMode(pin, INPUT);                   // Release: external pull-up wins
      }
      break;
  }
}

void LC76G::disableRxPullup(uint8_t rxPin) {
#if defined(ARDUINO_ARCH_AVR)
  // AVR: on an input pin, writing LOW turns the pull-up off; the pin stays
  // an input, so SoftwareSerial (pin-change interrupt) keeps working
  digitalWrite(rxPin, LOW);
#elif defined(ARDUINO_ARCH_RENESAS)
  // Renesas (Uno R4): pinMode() would also drop the IRQ function used by
  // SoftwareSerial, so the pin is reconfigured directly: input + IRQ,
  // without pull-up
  R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[rxPin].pin,
                  (uint32_t)(IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_IRQ_ENABLE));
#else
  (void)rxPin;                                 // Other cores: nothing to do
#endif
}

void LC76G::setPowerPin(int8_t pin, LC76G_DriveMode mode) {
  _pwrPin  = pin;
  _pwrMode = mode;
}

void LC76G::setResetPin(int8_t pin, LC76G_DriveMode mode) {
  _rstPin  = pin;
  _rstMode = mode;
  drivePin(_rstPin, _rstMode, false);          // Make sure the module is NOT held in reset
}

void LC76G::powerOn() {
  drivePin(_pwrPin, _pwrMode, true);
}

void LC76G::powerOff() {
  if (_pwrPin >= 0) {
    pauseLocus();                              // Otherwise the LOCUS data would be lost
  }
  drivePin(_pwrPin, _pwrMode, false);
}

void LC76G::resetPulse() {
  if (_rstPin < 0) {
    return;
  }
  drivePin(_rstPin, _rstMode, true);           // Hold in reset
  delay(LC76G_RESET_PULSE_MS);
  drivePin(_rstPin, _rstMode, false);          // Release
}

bool LC76G::reboot() {
  if (_rstPin < 0 && _pwrPin < 0) {
    return false;                              // No way to reboot from software
  }
  bool resumeLocus = pauseLocus();             // Stop LOCUS or its data would be lost

  if (_rstPin >= 0) {                          // Preferred: reset keeps VCC stable
    resetPulse();
  } else if (_pwrPin >= 0) {                   // Fallback: power cycle
    powerOff();
    delay(LC76G_POWER_OFF_MS);
    powerOn();
  }
  delay(LC76G_BOOT_MS);
  _inLine = false;                             // Discard any half-received sentence

  if (resumeLocus && sendPair(900, "1") == LC76G_OK) {
    _locusActive = true;                       // Recording restarted
  }
  return true;
}


// ===========================================================================
//  SENTENCE ASSEMBLY
// ===========================================================================

bool LC76G::checksumOk(const char *body) {
  const char *star = strchr(body, '*');
  if (star == nullptr || strlen(star) < 3) {   // Need '*' + 2 hex digits
    return false;
  }
  uint8_t calc = 0;
  for (const char *p = body; p < star; p++) {  // XOR of everything between '$' and '*'
    calc ^= (uint8_t)*p;
  }
  char hex[3] = { star[1], star[2], '\0' };
  return calc == (uint8_t)strtol(hex, nullptr, 16);
}

bool LC76G::assemble(char c) {
  if (c == '$') {                              // Start of a sentence
    _inLine  = true;
    _lineLen = 0;
    return false;
  }
  if (!_inLine) {                              // Ignore bytes before '$'
    return false;
  }
  if (c == '\r' || c == '\n') {                // End of sentence
    _line[_lineLen] = '\0';
    _inLine = false;
    return checksumOk(_line);
  }
  if (_lineLen < LC76G_LINE_MAX - 1) {
    _line[_lineLen++] = c;
  } else {                                     // Too long: not a valid sentence
    _inLine = false;
  }
  return false;
}

void LC76G::handleSentence(const char *body) {
  _lastSentenceMs = millis();

  // The library first, the user hook last: 'body' points into _line, and
  // the library must be done with it before any user code runs
  routeSentence(body);

  if (_sentenceCb != nullptr) {                // User hook sees every sentence
    _sentenceCb(body);
  }
}

void LC76G::routeSentence(const char *body) {
  // PQTM output messages registered with onPvt(), onDop(), ...
  if (body[0] == 'P' && body[1] == 'Q') {
    for (uint8_t i = 0; i < LC76G_OUTPUT_HOOKS; i++) {
      if (_hooks[i].decode != nullptr && _hooks[i].decode(body, _hooks[i].cb)) {
        return;                                // Decoded: nothing else to do
      }
    }
  }

  // LOCUS read-back: $LOGGA / $LORMC records must NEVER reach the live
  // decoder (they are old positions); $PAIR908,<n>,... frames the dump.
  if (strncmp(body, "LOGGA,", 6) == 0 || strncmp(body, "LORMC,", 6) == 0) {
    if (_locusDumping) {
      _locusLastMs = millis();
      // Field count check: the XOR checksum cannot detect the loss of two
      // identical characters (e.g. ",,"), the number of commas can.
      uint8_t commas = 0;
      for (const char *p = body; *p != '\0' && *p != '*'; p++) {
        if (*p == ',') commas++;
      }
      uint8_t expected = (body[2] == 'G') ? 14 : 13;  // LOGGA : LORMC
      if (commas != expected) {
        _locusRejected++;
      } else {
        _locusReceived++;
        if (_locusCb != nullptr) {
          _locusCb(body);
        }
      }
    }
    return;
  }
  if (_locusDumping && strncmp(body, "PAIR908,", 8) == 0) {
    _locusLastMs = millis();
    char f[12];
    LC76G_Nmea::getField(body, 1, f, sizeof(f));
    if (f[0] == '1') {                         // $PAIR908,1,<Record_Num>,<Record_Size>
      LC76G_Nmea::getField(body, 2, f, sizeof(f));
      _locusRecords = (uint32_t)atol(f);
    } else if (f[0] == '2') {                  // $PAIR908,2,... = one compact record
      LC76G_LocusRecord rec;
      if (decodeLocusRecord(body, rec)) {
        _locusReceived++;
        if (_locusRecCb != nullptr) {
          _locusRecCb(rec);
        }
      } else {
        _locusRejected++;
      }
    } else if (f[0] == '3') {                  // $PAIR908,3 = end of data
      _locusDumpDone = true;
    }
    return;
  }

  // Jamming status messages (sent periodically when detection is enabled):
  //   $PAIRSPF,<Status>               (PAIR391)
  //   $PQTMJAMMINGSTATUS,1,<Status>   (PQTM output message)
  if (strncmp(body, "PAIRSPF,", 8) == 0 || strncmp(body, "PQTMJAMMINGSTATUS,", 18) == 0) {
    char f[4];
    uint8_t field = (body[1] == 'A') ? 1 : 2;  // PAIRSPF: field 1, PQTM...: field 2
    LC76G_Nmea::getField(body, field, f, sizeof(f));
    uint8_t s = (uint8_t)atoi(f);
    if (s <= LC76G_JAM_CRITICAL) {
      _jamStatus   = (LC76G_JamStatus)s;
      _jamStatusMs = millis();
    }
    return;
  }

  // Estimated error: $PQTMEPE,2,<North>,<East>,<Down>,<2D>,<3D> (metres)
  if (strncmp(body, "PQTMEPE,", 8) == 0) {
    char f[16];
    uint32_t *dst[5] = { &_epe.northMm, &_epe.eastMm, &_epe.downMm,
                         &_epe.horizontalMm, &_epe.total3dMm };
    for (uint8_t i = 0; i < 5; i++) {
      LC76G_Nmea::getField(body, 2 + i, f, sizeof(f));
      *dst[i] = (uint32_t)LC76G_Nmea::parseFixed(f, 3);   // metres -> mm (0 if empty)
    }
    _epeMs = millis();
    return;
  }

  // Geofence status: $PQTMGEOFENCESTATUS,1,<Time>,<State0>,...,<State3>
  if (strncmp(body, "PQTMGEOFENCESTATUS,", 19) == 0) {
    char f[4];
    for (uint8_t i = 0; i < LC76G_GEOFENCE_COUNT; i++) {
      LC76G_Nmea::getField(body, 3 + i, f, sizeof(f));
      uint8_t s = (uint8_t)atoi(f);
      _geoState[i] = (s <= LC76G_GEO_OUTSIDE) ? s : (uint8_t)LC76G_GEO_UNKNOWN;
    }
    _geoStatusMs = millis();
    return;
  }

  // Reply to a pending PQTM command: $<name>,OK[,...] or $<name>,ERROR,<code>
  if (_waitPqtm != nullptr) {
    size_t n = strlen(_waitPqtm);
    if (strncmp(body, _waitPqtm, n) == 0 && body[n] == ',') {
      const char *rest = body + n + 1;
      if (strncmp(rest, "ERROR", 5) == 0) {
        char f[4];
        LC76G_Nmea::getField(body, 2, f, sizeof(f));
        switch (atoi(f)) {
          case 1:  _waitResult = LC76G_PARAM_ERROR;   break;  // Invalid parameters
          case 3:  _waitResult = LC76G_NOT_SUPPORTED; break;  // Unsupported command
          default: _waitResult = LC76G_FAILED;        break;  // 2 = failed execution
        }
      } else {
        // "OK[,...]" or, for a few commands such as PQTMVERNO, the data
        // directly without "OK": both mean success
        if (_respBuf != nullptr && _respSize > 0) {
          strncpy(_respBuf, body, _respSize - 1);
          _respBuf[_respSize - 1] = '\0';
        }
        _waitResult = LC76G_OK;
      }
      return;
    }
  }

  // Proprietary PAIR sentences: acknowledgements and query results
  if (strncmp(body, "PAIR", 4) == 0) {
    if (_waitId < 0) {
      return;                                  // No command pending
    }
    char f[8];
    if (strncmp(body, "PAIR001,", 8) == 0) {
      // $PAIR001,<CommandID>,<Result>
      LC76G_Nmea::getField(body, 1, f, sizeof(f));
      if (atoi(f) == _waitId) {
        LC76G_Nmea::getField(body, 2, f, sizeof(f));
        _waitResult = (int8_t)atoi(f);
      }
    } else if (_respBuf != nullptr && _respSize > 0) {
      // Query result "PAIRnnn,..." for the pending command
      if (atoi(body + 4) == _waitId && body[7] == ',') {
        strncpy(_respBuf, body, _respSize - 1);
        _respBuf[_respSize - 1] = '\0';
      }
    }
    return;
  }

  // Everything else goes to the NMEA decoder
  nmea.process(body);
}

void LC76G::update() {
  if (_port == nullptr) {
    return;
  }
  while (_port->available()) {
    if (assemble((char)_port->read())) {
      handleSentence(_line);
    }
  }
}


// ===========================================================================
//  COMMANDS
// ===========================================================================

bool LC76G::sendRaw(const char *body) {
  if (_port == nullptr) {
    return false;
  }
  uint8_t cs = 0;
  for (const char *p = body; *p != '\0'; p++) {
    cs ^= (uint8_t)*p;
  }
  char tail[6];                                // "*XX\r\n" + terminator
  snprintf(tail, sizeof(tail), "*%02X\r\n", cs);
  _port->print('$');
  _port->print(body);
  _port->print(tail);
  return true;
}

bool LC76G::sendRaw2(const char *a, const char *b) {
  if (_port == nullptr) {
    return false;
  }
  bool hasB = (b != nullptr && b[0] != '\0');
  uint8_t cs = 0;                              // Checksum over "a,b" without building it
  for (const char *p = a; *p != '\0'; p++) cs ^= (uint8_t)*p;
  if (hasB) {
    cs ^= (uint8_t)',';
    for (const char *p = b; *p != '\0'; p++) cs ^= (uint8_t)*p;
  }
  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X\r\n", cs);
  _port->print('$');
  _port->print(a);
  if (hasB) {
    _port->print(',');
    _port->print(b);
  }
  _port->print(tail);
  return true;
}

LC76G_Result LC76G::sendPqtm(const char *name, const char *params, uint16_t timeoutMs,
                             char *resp, uint8_t respSize) {
  // Retry only when the module did not answer at all
  LC76G_Result r = LC76G_TIMEOUT;
  for (uint8_t attempt = 0; attempt <= _retries; attempt++) {
    r = sendPqtmOnce(name, params, timeoutMs, resp, respSize);
    if (r != LC76G_TIMEOUT) {
      break;
    }
  }
  return r;
}

LC76G_Result LC76G::sendPqtmOnce(const char *name, const char *params, uint16_t timeoutMs,
                                 char *resp, uint8_t respSize) {
  if (_port == nullptr) {
    return LC76G_NO_PORT;
  }

  // Arm the reply matcher before sending
  _waitPqtm   = name;
  _waitResult = LC76G_TIMEOUT;
  _respBuf    = (respSize > 0) ? resp : nullptr;
  _respSize   = respSize;
  if (_respBuf != nullptr) {
    _respBuf[0] = '\0';
  }

  sendRaw2(name, params);

  // PQTM commands answer OK or ERROR directly, no intermediate state
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs && _waitResult == LC76G_TIMEOUT) {
    update();
  }

  LC76G_Result result = (LC76G_Result)_waitResult;
  _waitPqtm = nullptr;                         // Disarm
  _respBuf  = nullptr;
  _respSize = 0;
  return result;
}

LC76G_Result LC76G::setPqtmMessageRate(const char *msgName, uint8_t rate, uint8_t msgVer) {
  char params[40];                             // "W,<MsgName>,<Rate>,<MsgVer>"
  snprintf(params, sizeof(params), "W,%s,%u,%u", msgName, (unsigned)rate, (unsigned)msgVer);
  return sendPqtm("PQTMCFGMSGRATE", params);
}

LC76G_Result LC76G::getPqtmMessageRate(const char *msgName, uint8_t msgVer, uint8_t &rate) {
  // Query:  PQTMCFGMSGRATE,R,<MsgName>,<MsgVer>
  // Reply:  PQTMCFGMSGRATE,OK,<MsgName>,<Rate>,<MsgVer>
  //  field:      0         1     2        3       4
  char params[32];
  snprintf(params, sizeof(params), "R,%s,%u", msgName, (unsigned)msgVer);
  char resp[48];
  LC76G_Result r = sendPqtm("PQTMCFGMSGRATE", params, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r != LC76G_OK) {
    return r;
  }
  char f[4];
  LC76G_Nmea::getField(resp, 3, f, sizeof(f));  // Output rate field
  rate = (uint8_t)atoi(f);
  return r;
}

LC76G_Result LC76G::sendPair(uint16_t id, const char *params, uint16_t timeoutMs,
                             char *resp, uint8_t respSize) {
  // Retry only when the module did not answer at all
  LC76G_Result r = LC76G_TIMEOUT;
  for (uint8_t attempt = 0; attempt <= _retries; attempt++) {
    r = sendPairOnce(id, params, timeoutMs, resp, respSize);
    if (r != LC76G_TIMEOUT) {
      break;
    }
  }
  return r;
}

LC76G_Result LC76G::sendPairOnce(uint16_t id, const char *params, uint16_t timeoutMs,
                                 char *resp, uint8_t respSize) {
  if (_port == nullptr) {
    return LC76G_NO_PORT;
  }

  // Build "PAIRnnn" or "PAIRnnn,<params>"
  char body[LC76G_LINE_MAX];
  if (params != nullptr && params[0] != '\0') {
    snprintf(body, sizeof(body), "PAIR%03u,%s", (unsigned)id, params);
  } else {
    snprintf(body, sizeof(body), "PAIR%03u", (unsigned)id);
  }

  // Arm the reply matcher BEFORE sending, so a fast reply is not missed
  _waitId     = (int16_t)id;
  _waitResult = LC76G_TIMEOUT;
  _respBuf    = (respSize > 0) ? resp : nullptr;
  _respSize   = respSize;
  if (_respBuf != nullptr) {
    _respBuf[0] = '\0';
  }

  sendRaw(body);

  // Keep processing incoming data (NMEA included) while waiting
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    update();
    bool finalAck = (_waitResult != LC76G_TIMEOUT && _waitResult != LC76G_PROCESSING);
    if (!finalAck) {
      continue;                                // No ack yet, or "processing"
    }
    if (_waitResult != LC76G_OK) {
      break;                                   // Error: no query result will follow
    }
    if (_respBuf == nullptr || _respBuf[0] != '\0') {
      break;                                   // Done (no data expected, or data received)
    }
  }

  // Work out the final result
  LC76G_Result result = (LC76G_Result)_waitResult;
  if (result == LC76G_PROCESSING) {
    result = LC76G_TIMEOUT;                    // Never reached a final state
  }
  if (result == LC76G_OK && _respBuf != nullptr && _respBuf[0] == '\0') {
    result = LC76G_TIMEOUT;                    // Acknowledged but query data missing
  }

  // Disarm the matcher
  _waitId  = -1;
  _respBuf = nullptr;
  _respSize = 0;
  return result;
}

// ===========================================================================
//  QUERY HELPER (shared by several feature files)
// ===========================================================================

LC76G_Result LC76G::queryPairInt(uint16_t id, uint8_t field, int32_t &value) {
  char resp[48];
  LC76G_Result r = sendPair(id, nullptr, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[12];
    LC76G_Nmea::getField(resp, field, f, sizeof(f));
    value = atol(f);
  }
  return r;
}


LC76G_Result LC76G::queryBaudRate(uint32_t &baud) {
  char resp[24];                               // "PAIR865,115200*xx"
  LC76G_Result r = sendPair(865, "0,0", LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[12];
    LC76G_Nmea::getField(resp, 1, f, sizeof(f));
    baud = (uint32_t)atol(f);
  }
  return r;
}


// ===========================================================================
//  BAUD RATE HANDLING
// ===========================================================================

void LC76G::setHostBaud(uint32_t baud) {
  if (_setHostBaud != nullptr) {
    _setHostBaud(baud);
  }
  _baud   = baud;
  _inLine = false;                             // Old partial data is meaningless now
}

bool LC76G::detectAt(uint32_t baud, uint16_t timeoutMs) {
  setHostBaud(baud);
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (_port->available() && assemble((char)_port->read())) {
      handleSentence(_line);
      return true;                             // Valid checksum = correct baud rate
    }
  }
  return false;
}

LC76G_Result LC76G::setBaudRate(uint32_t baud) {
  if (_port == nullptr) {
    return LC76G_NO_PORT;
  }

  // Send PAIR864 at the current speed. The acknowledgement is not required:
  // on SoftwareSerial it may be lost if the current speed is 115200.
  char params[20];
  snprintf(params, sizeof(params), "0,0,%lu", (unsigned long)baud);   // Port 0 = UART
  sendPair(864, params, 500);

  // The new speed is applied only after a reboot
  if (!reboot()) {
    return LC76G_NO_REBOOT;                    // User must power-cycle the module
  }
  return detectAt(baud, LC76G_DETECT_MS) ? LC76G_OK : LC76G_TIMEOUT;
}


// ===========================================================================
//  START-UP
// ===========================================================================

bool LC76G::begin(Stream &port, LC76G_BaudFn setHostBaud, uint32_t targetBaud) {
  _port        = &port;
  _setHostBaud = setHostBaud;

  // Safe initial state, then power the module and let it boot
  drivePin(_rstPin, _rstMode, false);          // Not in reset
  powerOn();
  delay(LC76G_BOOT_MS);

  // 1) Already at the target speed?
  if (detectAt(targetBaud, LC76G_DETECT_MS)) {
    waitCommandReady(LC76G_READY_MS);                 // NMEA flows: wait until commands work too
    return true;
  }

  // 2) Without a way to change the host speed nothing else can be tried
  if (_setHostBaud == nullptr) {
    return false;
  }

  // 3) Send the baud change blindly at every common speed (the module
  //    ignores the copies sent at a wrong speed), then reboot once.
  char params[20];
  snprintf(params, sizeof(params), "0,0,%lu", (unsigned long)targetBaud);
  for (uint8_t i = 0; i < sizeof(PROBE_BAUDS) / sizeof(PROBE_BAUDS[0]); i++) {
    if (PROBE_BAUDS[i] == targetBaud) {
      continue;                                // Already known not to answer there
    }
    setHostBaud(PROBE_BAUDS[i]);
    delay(20);
    sendPair(864, params, 200);                // Short timeout: ack not required
  }

  if (!reboot()) {
    setHostBaud(targetBaud);
    return false;                              // Manual power cycle needed
  }

  // 4) Verify
  if (!detectAt(targetBaud, LC76G_DETECT_MS)) {
    return false;
  }
  waitCommandReady(LC76G_READY_MS);
  return true;
}

bool LC76G::waitCommandReady(uint32_t timeoutMs) {
  // A PAIR query is used on purpose: on the reference shield the module
  // answered PQTM commands (e.g. PQTMVERNO) a few seconds before PAIR
  // commands. PAIR063,0 (output rate of GGA) is harmless and supported by
  // all the modules of the family.
  char resp[24];
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (sendPairOnce(63, "0", LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp)) == LC76G_OK) {
      return true;
    }
  }
  return false;
}


// ===========================================================================
//  STATUS
// ===========================================================================

bool LC76G::isTalking(uint32_t silenceMs) const {
  return _lastSentenceMs != 0 && (millis() - _lastSentenceMs) < silenceMs;
}
