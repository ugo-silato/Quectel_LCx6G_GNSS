/*
 * ============================================================================
 *  LC76G_Locus.cpp  -  LOCUS built-in logger of the LC76G
 * ============================================================================
 *  Part of the Quectel_LC76G Arduino library. Author: Ugo Silato. License: MIT.
 *
 *  Commands used (Quectel GNSS Protocol Specification):
 *    PAIR900 / PAIR901  Enable / status
 *    PAIR902 / PAIR903  Save mode
 *    PAIR904 / PAIR905  Thresholds (time, speed, distance)
 *    PAIR906            Clear data and/or settings
 *    PAIR907            Record now
 *    PAIR908            Read the records
 *    PAIR909            Number of records
 * ============================================================================
 */

#include "LC76G.h"
#include <stdlib.h>   // atol()


// ===========================================================================
//  INTERNAL: data protection
// ===========================================================================

bool LC76G::pauseLocus() {
  if (_port == nullptr) {
    return false;
  }
  // After a host reset the library does not know whether the module is
  // still recording: ask it, unless we already know it is.
  bool active = _locusActive;
  if (!active) {
    bool en = false;
    if (getLocusEnabled(en) == LC76G_OK) {
      active = en;
    }
  }
  if (active && sendPair(900, "0") == LC76G_OK) {
    _locusActive = false;
  }
  return active;
}


// ===========================================================================
//  ENABLE / STATUS
// ===========================================================================

LC76G_Result LC76G::setLocusEnabled(bool enable) {
  // The protocol does not allow setting the same state twice in a row
  // (PAIR900 note 2): check first and do nothing if already there.
  bool current = false;
  if (getLocusEnabled(current) == LC76G_OK && current == enable) {
    return LC76G_OK;
  }
  LC76G_Result r = sendPair(900, enable ? "1" : "0");
  if (r == LC76G_OK) {
    _locusActive = enable;
  }
  return r;
}

LC76G_Result LC76G::getLocusEnabled(bool &enabled) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(901, 1, v);    // $PAIR901,<Enable>
  if (r == LC76G_OK) {
    enabled = (v == 1);
    _locusActive = enabled;                    // Keep the cached state in sync
  }
  return r;
}


// ===========================================================================
//  MODE AND THRESHOLDS
// ===========================================================================

LC76G_Result LC76G::setLocusMode(uint8_t modeBits, bool require3dFix) {
  if (modeBits == 0 || modeBits > 0x3F) {
    return LC76G_PARAM_ERROR;
  }
  // Recording must be disabled while the mode changes (PAIR902 note 1)
  bool wasActive = pauseLocus();

  char p[8];
  snprintf(p, sizeof(p), "%u,%u", (unsigned)modeBits, require3dFix ? 1u : 0u);
  LC76G_Result r = sendPair(902, p);

  if (wasActive && sendPair(900, "1") == LC76G_OK) {
    _locusActive = true;                       // Restore the previous state
  }
  return r;
}

LC76G_Result LC76G::getLocusMode(uint8_t &modeBits, bool &require3dFix) {
  // $PAIR903,<Mode>,<Check_3D_Fix>
  char resp[24];
  LC76G_Result r = sendPair(903, nullptr, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[6];
    LC76G_Nmea::getField(resp, 1, f, sizeof(f));
    modeBits = (uint8_t)atol(f);
    LC76G_Nmea::getField(resp, 2, f, sizeof(f));
    require3dFix = (f[0] == '1');
  }
  return r;
}

LC76G_Result LC76G::setLocusThreshold(LC76G_LocusTrigger trigger, uint16_t value) {
  // Ranges from the protocol specification
  uint16_t maxValue;
  switch (trigger) {
    case LC76G_LOCUS_BY_TIME:     maxValue = 43200; break;  // seconds
    case LC76G_LOCUS_BY_SPEED:    maxValue = 100;   break;  // m/s
    case LC76G_LOCUS_BY_DISTANCE: maxValue = 50000; break;  // metres
    default:                      return LC76G_PARAM_ERROR;
  }
  if (value < 1 || value > maxValue) {
    return LC76G_PARAM_ERROR;
  }
  char p[12];
  snprintf(p, sizeof(p), "%u,%u", (unsigned)trigger, (unsigned)value);
  return sendPair(904, p);
}

LC76G_Result LC76G::getLocusThreshold(LC76G_LocusTrigger trigger, uint16_t &value) {
  char p[4];
  snprintf(p, sizeof(p), "%u", (unsigned)trigger);
  // $PAIR905,<Threshold>
  char resp[24];
  LC76G_Result r = sendPair(905, p, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[8];
    LC76G_Nmea::getField(resp, 1, f, sizeof(f));
    value = (uint16_t)atol(f);
  }
  return r;
}


// ===========================================================================
//  DATA
// ===========================================================================

LC76G_Result LC76G::clearLocus(LC76G_LocusClear what) {
  char p[4];
  snprintf(p, sizeof(p), "%u", (unsigned)what);
  return sendPair(906, p, 3000);               // Flash erase: allow more time
}

LC76G_Result LC76G::logLocusNow() {
  return sendPair(907);
}

LC76G_Result LC76G::getLocusRecordCount(uint32_t &count) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(909, 1, v);    // $PAIR909,<Record_Num>
  if (r == LC76G_OK) {
    count = (uint32_t)v;
  }
  return r;
}

// ===========================================================================
//  READ-BACK
// ===========================================================================

// Parses 'len' hexadecimal digits; returns false on a non-hex character
static bool parseHex(const char *s, uint8_t len, uint32_t &value) {
  value = 0;
  for (uint8_t i = 0; i < len; i++) {
    char c = s[i];
    uint8_t v;
    if (c >= '0' && c <= '9')      v = c - '0';
    else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
    else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
    else                           return false;
    value = (value << 4) | v;
  }
  return true;
}

bool LC76G::decodeLocusRecord(const char *body, LC76G_LocusRecord &rec) {
  // $PAIR908,2,<UTC>,<Fix>,<Lat>,<Lon>,<Height>,<Speed>,<Heading>,<HDOP>,<SatNo>
  // field:  0 1   2     3     4     5      6       7        8       9     10
  // Every field has a FIXED number of hex digits (bytes * 2): checking the
  // length of each field detects lost characters that the checksum misses.
  static const uint8_t DIGITS[9] = { 8, 2, 8, 8, 4, 4, 4, 4, 2 };
  uint32_t v[9];
  char f[12];
  for (uint8_t i = 0; i < 9; i++) {
    if (!LC76G_Nmea::getField(body, 2 + i, f, sizeof(f)) ||
        strlen(f) != DIGITS[i] || !parseHex(f, DIGITS[i], v[i])) {
      return false;                            // Missing, short or invalid field
    }
  }
  // An 11th data field would mean a malformed line
  if (LC76G_Nmea::getField(body, 11, f, sizeof(f))) {
    return false;
  }
  rec.utc        = v[0];
  rec.fixType    = (uint8_t)v[1];
  rec.latE7      = (int32_t)v[2];              // Signed 32-bit, degrees * 1e7
  rec.lonE7      = (int32_t)v[3];
  rec.heightRaw  = (int16_t)(uint16_t)v[4];    // Signed 16-bit (altitude can be negative)
  rec.speedRaw   = (uint16_t)v[5];
  rec.headingRaw = (uint16_t)v[6];
  rec.hdopRaw    = (uint16_t)v[7];
  rec.sats       = (uint8_t)v[8];
  return true;
}

LC76G_Result LC76G::dumpLocusInternal(uint8_t type, LC76G_LocusDumpStats &stats,
                                      uint32_t idleTimeoutMs) {
  stats.announced = 0;
  stats.received  = 0;
  stats.rejected  = 0;
  if (_port == nullptr) {
    return LC76G_NO_PORT;
  }

  // 1) Recording must be off before reading (PAIR908 note 1)
  LC76G_Result r = setLocusEnabled(false);
  if (r != LC76G_OK) {
    return r;
  }

  // 2) The UART speed is NOT changed for the read-back: changing it needs
  //    module reboots, and at 9600 / 19200 baud the module was observed
  //    to drop its own replies (the protocol warns that speeds < 115200
  //    may lose messages). On SoftwareSerial the read-back may therefore
  //    be incomplete: 'stats' reports it. Use a hardware UART for
  //    reliable read-backs.

  // 3) Silence the live NMEA output, so the UART carries only LOCUS data
  uint8_t saved[LC76G_NMEA_TYPE_COUNT];
  bool    restore[LC76G_NMEA_TYPE_COUNT];
  for (uint8_t t = 0; t < LC76G_NMEA_TYPE_COUNT; t++) {
    restore[t] = (getNmeaRate((LC76G_NmeaType)t, saved[t]) == LC76G_OK && saved[t] != 0);
    if (restore[t]) {
      setNmeaRate((LC76G_NmeaType)t, 0);
    }
  }

  // 4) Arm the read-back: handleSentence() now routes the LOCUS data
  _locusDumping  = true;
  _locusDumpDone = false;
  _locusRecords  = 0;
  _locusReceived = 0;
  _locusRejected = 0;
  _locusLastMs   = millis();
  _waitId        = 908;                        // The $PAIR001 arrives AFTER the data
  _waitResult    = LC76G_TIMEOUT;
  _respBuf       = nullptr;
  _respSize      = 0;

  // 5) Request the records and read until the end marker. Right after a
  //    reboot the module may answer "busy" or "failed" while its storage
  //    is still starting: retry a few times before giving up.
  const char *cmd = (type == 1) ? "PAIR908,1" : "PAIR908,0";
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    _waitResult  = LC76G_TIMEOUT;
    _locusLastMs = millis();
    sendRaw(cmd);

    unsigned long doneAt = 0;
    while (true) {
      update();
      if (_locusDumpDone) {
        if (doneAt == 0) {
          doneAt = millis();
        }
        // Wait briefly for the final acknowledgement after the end marker
        if (_waitResult != LC76G_TIMEOUT || millis() - doneAt > LC76G_CMD_TIMEOUT_MS) {
          break;
        }
      } else if (_waitResult != LC76G_TIMEOUT && _waitResult != LC76G_OK &&
                 _waitResult != LC76G_PROCESSING) {
        break;                                 // The module refused the command
      } else if (millis() - _locusLastMs > idleTimeoutMs) {
        break;                                 // No LOCUS data for too long
      }
    }
    // Retry only if the module refused without sending any data
    bool refused = !_locusDumpDone && _locusRecords == 0 &&
                   (_waitResult == LC76G_BUSY || _waitResult == LC76G_FAILED);
    if (!refused) {
      break;
    }
    delay(1000);
  }

  // 6) Result: report the module error code when there is one
  if (!_locusDumpDone) {
    r = (_waitResult != LC76G_TIMEOUT && _waitResult != LC76G_OK &&
         _waitResult != LC76G_PROCESSING) ? (LC76G_Result)_waitResult : LC76G_TIMEOUT;
  } else if (_waitResult != LC76G_TIMEOUT && _waitResult != LC76G_OK) {
    r = (LC76G_Result)_waitResult;
  } else {
    r = LC76G_OK;
  }
  stats.announced = _locusRecords;
  stats.received  = _locusReceived;
  stats.rejected  = _locusRejected;

  // 7) Disarm, restore the live NMEA output, then the original speed
  //    (in this order: the speed change waits for an NMEA sentence)
  _locusDumping = false;
  _locusCb      = nullptr;
  _locusRecCb   = nullptr;
  _waitId       = -1;
  for (uint8_t t = 0; t < LC76G_NMEA_TYPE_COUNT; t++) {
    if (restore[t]) {
      setNmeaRate((LC76G_NmeaType)t, saved[t]);
    }
  }
  return r;
}

LC76G_Result LC76G::dumpLocus(LC76G_LocusRecordFn cb, LC76G_LocusDumpStats &stats,
                              uint32_t idleTimeoutMs) {
  _locusRecCb = cb;
  _locusCb    = nullptr;
  return dumpLocusInternal(1, stats, idleTimeoutMs);
}

LC76G_Result LC76G::dumpLocusNmea(LC76G_SentenceFn cb, LC76G_LocusDumpStats &stats,
                                  uint32_t idleTimeoutMs) {
  _locusCb    = cb;
  _locusRecCb = nullptr;
  return dumpLocusInternal(0, stats, idleTimeoutMs);
}


// ===========================================================================
//  TIME CONVERSION
// ===========================================================================

void LC76G::unixToUtc(uint32_t t, uint16_t &year, uint8_t &month, uint8_t &day,
                      uint8_t &hour, uint8_t &minute, uint8_t &second) {
  second = (uint8_t)(t % 60);  t /= 60;
  minute = (uint8_t)(t % 60);  t /= 60;
  hour   = (uint8_t)(t % 24);  t /= 24;      // t is now days since 1970-01-01

  // Civil date from day count (H. Hinnant's algorithm), 32-bit integers
  // only: the Uno 'int' is 16-bit, so every variable is explicit.
  int32_t  z   = (int32_t)t + 719468L;
  int32_t  era = z / 146097L;
  uint32_t doe = (uint32_t)(z - era * 146097L);                      // [0, 146096]
  uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;  // [0, 399]
  int32_t  y   = (int32_t)yoe + era * 400;
  uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);            // [0, 365]
  uint32_t mp  = (5 * doy + 2) / 153;                                // [0, 11]
  day   = (uint8_t)(doy - (153 * mp + 2) / 5 + 1);
  month = (uint8_t)(mp < 10 ? mp + 3 : mp - 9);
  year  = (uint16_t)(y + (month <= 2 ? 1 : 0));
}
