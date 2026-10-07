/*
 * ============================================================================
 *  ShieldTest.ino  -  Development test sketch for the reference shield
 * ============================================================================
 *  One sketch for every test of the library. Choose what to test in the
 *  "TEST SELECTION" block below: remove the "//" in front of a #define to
 *  enable a test, put it back to disable it. The options of each test are
 *  right under its #define.
 *
 *  Disabled tests are NOT compiled (#ifdef), so they use no memory.
 *
 *  !!! MEMORY NOTE FOR THE ARDUINO UNO R3 !!!
 *  The Uno R3 has only 32 KB of flash and 2 KB of RAM.
 *  Measured with library 0.17.4 (values reported by the IDE):
 *    - one test enabled      : flash 43-64 %, global variables 41-62 %
 *    - all tests except TEST_PVT and TEST_EXTRA: flash 82 %, RAM 60 %
 *    - ALL tests enabled     : does NOT fit (flash overflow)
 *  Recommended: enable one or two tests at a time, so the output is
 *  also easier to read.
 *  If you add your own code, keep "Global variables use ..." below ~75 %:
 *  above that the R3 may reset randomly. If the IDE reports
 *  "Sketch too big" or "Not enough memory", disable some tests
 *  (nothing is damaged, the upload simply does not start).
 *  The Uno R4 Minima (256 KB flash, 32 KB RAM) has no such limits.
 *
 *  Always active: start-up, minimal NMEA output, status block every
 *  STATUS_EVERY_EPOCHS seconds, status LED on D13
 *  (fast blink = no data, slow blink = searching, steady = fix).
 *
 *  Hardware: Arduino Uno R3 or Uno R4 Minima + LC76G reference shield.
 *  Serial Monitor: 115200 baud.
 *
 *  !!! UNO R4 MINIMA: WIRING CHANGE NEEDED !!!
 *  On the R4 the sketch talks to the module through the hardware port
 *  Serial1 (D0 = RX, D1 = TX), because D4 cannot be used as SoftwareSerial
 *  RX there (it is not an interrupt-capable pin on the RA4M1).
 *  On the rev. 1 shield solder two short wires on the header pins:
 *      D4 -> D0   (module TXD to Arduino RX)
 *      D5 -> D1   (Arduino TX to module RXD, through the level shifter)
 *  Remove them before using the shield on an Uno R3 again: there D0/D1
 *  belong to the USB link and the upload would fail.
 * ============================================================================
 */

#if !defined(ARDUINO_ARCH_RENESAS)
#include <SoftwareSerial.h>          // Uno R3 only: software port on D4/D5
#endif
#include <Quectel_LC76G.h>

// ===========================================================================
//  TEST SELECTION  -  remove "//" to enable, add "//" to disable
//  (alphabetical order; the tests always run in a fixed order at start-up,
//  with TEST_FACTORY_RESET first)
// ===========================================================================

//#define TEST_AGNSS
// Firmware version, EASY / EPOC detection, prediction progress every minute.
const uint8_t EPOC_CONSTELLATIONS = LC76G_EPOC_GPS | LC76G_EPOC_GALILEO;

//#define TEST_CONSTELLATIONS
// Satellite systems to search (only LC76G-supported combinations).
const uint8_t CONSTELLATIONS = LC76G_GNSS_ALL;
                                           // e.g. LC76G_GNSS_GPS | LC76G_GNSS_GALILEO

#define TEST_EPE
// Estimated positioning error (PQTMEPE), printed in the status block.

//#define TEST_EXTRA
// Odometer (PQTMODO, reset at start), GPS time (PQTMTIMEGPS), protection
// level (PQTMPL) and the three ECEF messages (PQTMPOSECEF, PQTMVELECEF,
// PQTMPVTECEF), printed in the status block. Uses all 6 decoder slots
// (LC76G_OUTPUT_HOOKS): do not enable it together with TEST_PVT.

//#define TEST_FACTORY_RESET
// Restores ALL module parameters to the factory defaults at start-up
// (except baud rate and LOCUS settings). Runs before the other tests.

//#define TEST_GEOFENCE
// Circle created around the position of the first fix, GEOFENCE pin (D7).
const uint32_t GEOFENCE_RADIUS_M  = 30;    // Radius in metres
const bool     LED_SHOWS_GEOFENCE = false; // true = LED: steady inside, slow blink outside

//#define TEST_I2C
// I2C link (SDA/SCL), read-only check of the hardware: the module must
// answer at 0x50 / 0x54 / 0x58, send NMEA over I2C and answer a
// $PQTMVERNO written over I2C. The Arduino internal pull-ups are
// switched off: the bus is pulled up to 3.3 V by the shield only.
const uint32_t I2C_CLOCK_HZ = 100000;       // 100 kHz standard mode (400 kHz also allowed)
const uint8_t  I2C_READ_SIZE = 32;          // Bytes per read, max 32 on the R3 (try 8, 32, 128)

//#define TEST_JAMMING
// Jamming detection: status message and JAM_IND pin (D8).

//#define TEST_LOCUS
// LOCUS logger. Choose ONE value for LOCUS_ACTION:
#define LOCUS_STATUS 0                     // Only print the status
#define LOCUS_START  1                     // Start recording every LOCUS_PERIOD_S s; type d to read back
#define LOCUS_DUMP   2                     // Read all records, print them as CSV
#define LOCUS_CLEAR  3                     // Erase all records
const uint8_t  LOCUS_ACTION         = LOCUS_STATUS;
const uint16_t LOCUS_PERIOD_S       = 5;
const bool     LOCUS_REQUIRE_3D_FIX = false;   // false = record also without fix

//#define TEST_LOW_POWER
// Low power mode. Choose ONE value for LOW_POWER_MODE:
#define LP_CONTINUOUS 0                    // All low power modes off
#define LP_ALP1       1                    // Adaptive Low Power mode 1 (lowest power)
#define LP_ALP2       2                    // Adaptive Low Power mode 2 (performance)
#define LP_PERIODIC   3                    // Run / sleep cycles (PERIODIC_CFG)
#define LP_BACKUP     4                    // Backup after BACKUP_AFTER_S, wake-up after BACKUP_FOR_S
const uint8_t  LOW_POWER_MODE = LP_ALP1;
const LC76G_PeriodicConfig PERIODIC_CFG = { LC76G_PERIODIC_SMART, 10, 30, 30, 60 };
const uint32_t BACKUP_AFTER_S = 30;
const uint32_t BACKUP_FOR_S   = 30;

//#define TEST_NMEA_OUTPUT
// Prints the output rate of every NMEA sentence.
const bool NMEA_FACTORY_SET = false;       // true = factory set (GGA, GLL, GSA, GSV, RMC, VTG)
                                           // false = minimal set (GGA, GSV, RMC)

//#define TEST_PPS
// 1PPS configuration and pulse measurement on D2 (interrupt).
const uint16_t      PPS_DURATION_MS = 100; // Pulse length, 1..999 ms
const LC76G_PpsMode PPS_MODE        = LC76G_PPS_ALWAYS;   // Also without fix

//#define TEST_PVT
// Position, velocity and time (PQTMPVT), velocity with accuracy estimates
// (PQTMVEL) and all dilution of precision values (PQTMDOP), printed in the
// status block. Best with a fix (outdoors or near a window).

//#define TEST_RECEPTION
// Reception settings applied at start-up (defaults in brackets).
const int8_t           ELEVATION_MASK_DEG = 10;              // [5] degrees
const bool             AIC_ON             = true;            // Interference cancellation
const bool             SBAS_ON            = true;            // EGNOS satellites search
const LC76G_DgpsSource DGPS_SOURCE        = LC76G_DGPS_SBAS; // Apply SBAS corrections
const bool             FIX_2D_ON          = true;            // Fix with 3 satellites
const bool             IMMEDIATE_SPEED_ON = false;           // Less filtered speed
const bool             BDS_B1C_ON         = true;            // BDS B1C band
const uint8_t          STATIC_DMS         = 0;               // [0] static threshold, dm/s

//#define TEST_SETTINGS
// Output settings: minimum SNR, NMEA decimal places, NMEA Talker ID, RTCM
// output, Galileo RLM, 1PPS/NMEA sync. Each one is changed, read back and
// put back to the factory value (no RTCM data is left enabled).

//#define TEST_SUPPORT
// Read-only check: which PQTM messages and settings the module firmware
// supports (nothing is changed in the module).

// Status block period, in epochs (1 epoch = 1 second)
const uint8_t STATUS_EVERY_EPOCHS = 5;

#ifdef TEST_I2C
#include <Wire.h>                     // I2C master, only when the test is enabled
#endif


// ===========================================================================
//  PINS OF THE REFERENCE SHIELD
// ===========================================================================
const uint8_t  PIN_GNSS_TXD   = 4;   // Arduino RX <- module TXD
const uint8_t  PIN_GNSS_RXD   = 5;   // Arduino TX -> module RXD (level shifted)
const uint8_t  PIN_PWR_EN     = 6;   // Supply switch (NPN base, no resistor on rev. 1)
const uint8_t  PIN_RESET_DRV  = 3;   // Reset driver  (NPN base, no resistor on rev. 1)
const uint8_t  PIN_PPS        = 2;   // <- module 1PPS (interrupt pin)
const uint8_t  PIN_GEOFENCE   = 7;   // <- module GEOFENCE (HIGH = inside), plain input
const uint8_t  PIN_JAM_IND    = 8;   // <- module JAM_IND (LOW = jamming), plain input
const uint8_t  PIN_STATUS_LED = LED_BUILTIN;
const uint32_t GNSS_BAUD      = 38400;

// ---------------------------------------------------------------------------
//  Serial port towards the module (chosen automatically by board type)
// ---------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_RENESAS)
// Uno R4: hardware UART Serial1 on D0/D1 (USB uses a separate port).
// No RX pull-up is enabled by Serial1, so nothing else is needed.
#define gnssSerial Serial1
void openGnssPort(uint32_t baud) {
  Serial1.begin(baud);                                 // (Re)open the UART at this speed
}
#else
// Uno R3: SoftwareSerial on D4/D5 (D0/D1 are used by the USB link).
SoftwareSerial gnssSerial(PIN_GNSS_TXD, PIN_GNSS_RXD);   // (rx, tx)
void openGnssPort(uint32_t baud) {
  gnssSerial.begin(baud);                              // (Re)open the software port
  LC76G::disableRxPullup(PIN_GNSS_TXD);                // No 5 V pull-up into the 3.3 V TXD
}
#endif

LC76G gnss;

uint8_t epochCounter = 0;            // Counts epochs between status blocks
bool    epochPending = false;        // Set by onEpoch(), handled in loop()


// ===========================================================================
//  COMMON PRINT HELPERS
// ===========================================================================

// Prints "<name>... <result>" for a command
void printResult(const __FlashStringHelper *name, LC76G_Result r) {
  Serial.print(name);
  Serial.print(F("... "));
  Serial.println(LC76G::resultText(r));
}

// Prints "on" / "off", or the error text if the query failed
void printFlag(LC76G_Result r, bool on) {
  if (r == LC76G_OK) Serial.println(on ? F("on") : F("off"));
  else               Serial.println(LC76G::resultText(r));
}

// Prints a value scaled by 100 with 2 decimals, e.g. 233 -> 2.33
void printX100(uint32_t v) {
  Serial.print(v / 100);
  Serial.print('.');
  uint8_t frac = v % 100;
  if (frac < 10) Serial.print('0');
  Serial.print(frac);
}

// Prints degrees * 1e7 with 7 decimals, e.g. 446220417 -> 44.6220417
void printE7(int32_t v) {
  if (v < 0) { Serial.print('-'); v = -v; }
  Serial.print(v / 10000000L);
  Serial.print('.');
  int32_t frac = v % 10000000L;
  for (int32_t d = 1000000L; d >= 1 && frac < d; d /= 10) {
    Serial.print('0');                          // Leading zeros of the fraction
  }
  if (frac != 0) Serial.print(frac);
}

// Prints a two-digit number with leading zero, followed by a separator
void print2Sep(uint8_t v, char sep) {
  if (v < 10) Serial.print('0');
  Serial.print(v);
  Serial.print(sep);
}

// Prints a value scaled by 1000 with 3 decimals and sign,
// e.g. -1234 -> -1.234 (millimetres as metres, mm/s as m/s)
void printMilli(int32_t v) {
  if (v < 0) { Serial.print('-'); v = -v; }
  Serial.print(v / 1000);
  Serial.print('.');
  int16_t frac = (int16_t)(v % 1000);
  if (frac < 100) Serial.print('0');
  if (frac < 10)  Serial.print('0');
  Serial.print(frac);
}


// ===========================================================================
//  TEST HELPERS (compiled only when the test is enabled)
// ===========================================================================

#if defined(TEST_NMEA_OUTPUT) || defined(TEST_FACTORY_RESET)
// Output rate of every NMEA sentence, one per line with the full name
void printNmeaRates() {
  for (uint8_t t = 0; t < LC76G_NMEA_TYPE_COUNT; t++) {
    uint8_t rate = 0;
    LC76G_Result r = gnss.getNmeaRate((LC76G_NmeaType)t, rate);
    Serial.print(F("  "));
    Serial.print(LC76G::nmeaTypeName((LC76G_NmeaType)t));
    Serial.print(F(" = "));
    if (r != LC76G_OK)  Serial.println(LC76G::resultText(r));
    else if (rate == 0) Serial.println(F("off"));
    else if (rate == 1) Serial.println(F("every fix"));
    else { Serial.print(F("every ")); Serial.print(rate); Serial.println(F(" fixes")); }
  }
}
#endif

#ifdef TEST_CONSTELLATIONS
// Which constellations are searched, one per line with the full name
void printConstellations() {
  uint8_t mask = 0;
  LC76G_Result r = gnss.getConstellations(mask);
  if (r != LC76G_OK) {
    Serial.print(F("  query failed: "));
    Serial.println(LC76G::resultText(r));
    return;
  }
  for (uint8_t i = 0; i < 5; i++) {             // Bits 0..4: GPS..QZSS
    uint8_t bit = (uint8_t)(1u << i);
    Serial.print(F("  "));
    Serial.print(LC76G::constellationName(bit));
    Serial.println((mask & bit) ? F(" = enabled") : F(" = disabled"));
  }
}
#endif

#ifdef TEST_PPS
// 1PPS configuration in readable form
void printPpsConfig() {
  LC76G_PpsConfig cfg;
  LC76G_Result r = gnss.getPps(cfg);
  if (r != LC76G_OK) {
    Serial.print(F("  query failed: "));
    Serial.println(LC76G::resultText(r));
    return;
  }
  Serial.print(F("  Output: "));
  Serial.println(cfg.enabled ? F("enabled") : F("disabled"));
  if (cfg.enabled) {
    Serial.print(F("  Pulse duration: "));
    Serial.print(cfg.durationMs);
    Serial.print(F(" ms, mode: "));
    Serial.println(LC76G::ppsModeText(cfg.mode));
  }
}
#endif

#ifdef TEST_LOW_POWER
// Low power related settings of the module, in readable form
void printLowPowerConfig() {
  LC76G_NavMode nav;
  Serial.print(F("  Navigation mode: "));
  LC76G_Result r = gnss.getNavigationMode(nav);
  if (r == LC76G_OK) Serial.println(LC76G::navModeText(nav));
  else               Serial.println(LC76G::resultText(r));

  uint8_t alp;
  Serial.print(F("  Adaptive Low Power (ALP): "));
  r = gnss.getAlpMode(alp);
  if (r != LC76G_OK) Serial.println(LC76G::resultText(r));
  else if (alp == 0) Serial.println(F("off"));
  else { Serial.print(F("mode ")); Serial.println(alp); }

  LC76G_PeriodicConfig pc;
  Serial.print(F("  Periodic power saving: "));
  r = gnss.getPeriodicMode(pc);
  if (r == LC76G_OK) Serial.println(LC76G::periodicModeText(pc.mode));
  else               Serial.println(LC76G::resultText(r));
}

// Backup test, non-blocking state machine (no delay(): gnss.update() must
// keep running): RUNNING -> IN_BACKUP -> WAIT_FIX -> DONE
enum BackupStep : uint8_t { BK_RUNNING, BK_IN_BACKUP, BK_WAIT_FIX, BK_DONE };
BackupStep    backupStep = BK_RUNNING;
unsigned long backupT0   = 0;

void handleBackupTest() {
  unsigned long now = millis();
  if (backupStep == BK_RUNNING && now - backupT0 >= BACKUP_AFTER_S * 1000UL) {
    printResult(F(">>> Entering Backup mode (VCC off)"), gnss.enterBackup(0));
    backupStep = BK_IN_BACKUP;
    backupT0 = now;
  } else if (backupStep == BK_IN_BACKUP && now - backupT0 >= BACKUP_FOR_S * 1000UL) {
    printResult(F(">>> Leaving Backup mode"), gnss.exitBackup());
    backupStep = BK_WAIT_FIX;
    backupT0 = millis();
  } else if (backupStep == BK_WAIT_FIX && gnss.nmea.fix().valid) {
    Serial.print(F(">>> Fix after Backup in "));
    Serial.print((millis() - backupT0) / 1000);
    Serial.println(F(" s"));
    backupStep = BK_DONE;
  }
}
#endif

#ifdef TEST_AGNSS
bool epocAvailable = false;          // true if the firmware has EPOC instead of EASY

// Firmware version and prediction method
void printAgnssStatus() {
  char ver[32];
  Serial.print(F("  Firmware version: "));
  Serial.println(gnss.getFirmwareVersion(ver, sizeof(ver)) == LC76G_OK ? ver : "unknown");

  bool on = false;
  uint8_t mask = 0;
  epocAvailable = (gnss.getEpoc(on, mask) == LC76G_OK);
  if (epocAvailable) {
    Serial.print(F("  Enhanced Prediction Orbit on Chip (EPOC): "));
    Serial.print(on ? F("enabled") : F("disabled"));
    Serial.print(F(", constellations mask 0x"));
    Serial.println(mask, HEX);
  } else {
    uint8_t days = 0;
    Serial.print(F("  Embedded Assist System (EASY): "));
    printFlag(gnss.getEasy(on, days), on);
  }
}

// Prediction progress, every minute
unsigned long lastAgnssReport = 0;
void handleAgnssReport() {
  if (millis() - lastAgnssReport < 60000UL || !gnss.isTalking()) {
    return;
  }
  lastAgnssReport = millis();
  if (epocAvailable) {
    for (uint8_t bit = LC76G_EPOC_GPS; bit <= LC76G_EPOC_BDS; bit <<= 1) {
      if (!(EPOC_CONSTELLATIONS & bit)) continue;
      int32_t status = 0;
      uint8_t ready = 0;
      if (gnss.getEpocPredictionStatus(bit, status, ready) == LC76G_OK) {
        Serial.print(F(">>> EPOC mask 0x"));
        Serial.print(bit, HEX);
        Serial.print(F(": "));
        Serial.print(LC76G::epocStatusText(status));
        Serial.print(F(", satellites ready: "));
        Serial.println(ready);
      }
    }
  } else {
    bool on;
    uint8_t days = 0;
    if (gnss.getEasy(on, days) == LC76G_OK) {
      Serial.print(F(">>> EASY prediction days ready: "));
      Serial.println(days);
    }
  }
}
#endif

#ifdef TEST_I2C
// ---------------------------------------------------------------------------
// LC76G I2C protocol (I2C Application Note v1.2):
//   0x50 = configuration address, 0x54 = read address, 0x58 = write address
//   Read : 1) write {0xAA510008, 4} to 0x50, read 4 bytes (length) at 0x54
//          2) write {0xAA512000, n} to 0x50, read n bytes at 0x54
//   Write: 1) write {0xAA510004, 4} to 0x50, read 4 bytes (free space) at 0x54
//          2) write {0xAA531000, n} to 0x50, write n bytes at 0x58
//   Words are sent little-endian. About 10 ms between the steps.
// ---------------------------------------------------------------------------
const uint8_t I2C_ADDR_CFG   = 0x50;
const uint8_t I2C_ADDR_READ  = 0x54;
const uint8_t I2C_ADDR_WRITE = 0x58;
const uint8_t I2C_STEP_MS    = 10;          // Pause required by the module
#if defined(ARDUINO_ARCH_RENESAS)
const uint8_t I2C_CHUNK      = 128;         // Wire buffer is 255 bytes on the R4
#else
const uint8_t I2C_CHUNK      = 32;          // Wire buffer is 32 bytes on the R3
#endif

// Switches off the Arduino internal pull-ups on SDA/SCL (Wire.begin()
// enables them, towards 5 V). The shield pull-ups (2.2 k to 3.3 V) remain.
void i2cDisableInternalPullups() {
#if defined(ARDUINO_ARCH_RENESAS)
  // RA4M1: clear the PCR (pull-up control) bit of the two pin registers
  R_BSP_PinAccessEnable();
  bsp_io_port_pin_t sda = g_pin_cfg[SDA].pin;
  bsp_io_port_pin_t scl = g_pin_cfg[SCL].pin;
  R_PFS->PORT[sda >> 8].PIN[sda & 0xFF].PmnPFS_b.PCR = 0;
  R_PFS->PORT[scl >> 8].PIN[scl & 0xFF].PmnPFS_b.PCR = 0;
  R_BSP_PinAccessDisable();
#else
  // AVR: with the TWI enabled, writing LOW to the port bit removes the pull-up
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
#endif
}

// Every transaction follows the Quectel sample code: a 10 ms pause BEFORE
// it (the module needs it after the previous transfer, otherwise it does
// not acknowledge its address), and up to I2C_MAX_TRIES attempts.
const uint8_t I2C_MAX_TRIES = 20;
uint16_t i2cRetries = 0;                    // Extra attempts needed (statistics)

// Writes 'len' bytes to 'addr'; true when every byte was acknowledged
bool i2cTx(uint8_t addr, const uint8_t *data, uint8_t len) {
  for (uint8_t t = 0; t < I2C_MAX_TRIES; t++) {
    delay(I2C_STEP_MS);
    Wire.beginTransmission(addr);
    Wire.write(data, len);
    if (Wire.endTransmission() == 0) return true;   // 0 = all acknowledged
    i2cRetries++;
  }
  return false;
}

// Reads exactly 'n' bytes from 'addr' into 'buf'
bool i2cRx(uint8_t addr, uint8_t *buf, uint8_t n) {
  for (uint8_t t = 0; t < I2C_MAX_TRIES; t++) {
    delay(I2C_STEP_MS);
    if (Wire.requestFrom(addr, n) == n) {
      for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
      return true;
    }
    while (Wire.available()) Wire.read();   // Drop a partial answer
    i2cRetries++;
  }
  return false;
}

// Writes a configuration command (two 32-bit words, little-endian) to 0x50
bool i2cConfig(uint32_t cmd, uint32_t len) {
  uint8_t b[8];
  for (uint8_t i = 0; i < 4; i++) {
    b[i]     = (uint8_t)(cmd >> (8 * i));
    b[4 + i] = (uint8_t)(len >> (8 * i));
  }
  return i2cTx(I2C_ADDR_CFG, b, 8);
}

// Reads a 32-bit little-endian value from the read address 0x54
bool i2cReadWord(uint32_t &value) {
  uint8_t b[4];
  if (!i2cRx(I2C_ADDR_READ, b, 4)) return false;
  value = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
  return true;
}

// Read step 1: bytes waiting in the module transmit buffer
bool i2cAvailable(uint32_t &len) {
  return i2cConfig(0xAA510008UL, 4) && i2cReadWord(len);
}

// Read step 2: reads 'n' bytes (n <= I2C_CHUNK) into 'buf'.
// Returns 0 = OK, 1 = configuration command refused, 2 = data not received
uint8_t i2cReadData(uint8_t *buf, uint8_t n) {
  if (!i2cConfig(0xAA512000UL, n)) return 1;
  if (!i2cRx(I2C_ADDR_READ, buf, n)) return 2;
  return 0;
}

// Explains a failure code of i2cReadData()
void printReadError(uint8_t code) {
  Serial.print(code == 1 ? F("configuration command refused") : F("data not received"));
}

// Writes a whole text (e.g. "$PQTMVERNO*58\r\n") in pieces that fit both
// the free space of the module and the Wire buffer. Prints every step.
bool i2cWriteText(const char *text) {
  size_t total = strlen(text), done = 0;
  while (done < total) {
    uint32_t freeLen = 0;
    if (!i2cConfig(0xAA510004UL, 4)) {                 // Write step 1-a
      Serial.println(F("  write step 1: configuration command refused"));
      return false;
    }
    if (!i2cReadWord(freeLen)) {                       // Write step 1-b
      Serial.println(F("  write step 1: free length not received"));
      return false;
    }
    Serial.print(F("  write step 1: free space in the module "));
    Serial.print(freeLen);
    Serial.println(F(" bytes"));
    if (freeLen == 0) continue;                        // Full: ask again
    size_t n = total - done;
    if (n > freeLen)        n = freeLen;
    if (n > I2C_CHUNK - 1)  n = I2C_CHUNK - 1;        // Wire buffer limit
    if (!i2cConfig(0xAA531000UL, (uint32_t)n)) {       // Write step 2-a
      Serial.println(F("  write step 2: configuration command refused"));
      return false;
    }
    if (!i2cTx(I2C_ADDR_WRITE, (const uint8_t *)text + done, (uint8_t)n)) {   // Write step 2-b
      Serial.println(F("  write step 2: data refused at 0x58"));
      return false;
    }
    done += n;
  }
  return true;
}

// Small sentence assembler for the I2C stream, separate from the library
char     i2cLine[LC76G_LINE_MAX];
uint8_t  i2cLen = 0;
bool     i2cInLine = false;
uint16_t i2cGood = 0, i2cBad = 0;           // Sentences with right / wrong checksum
uint8_t  i2cShown = 0;                      // Sentences already printed
bool     i2cVersionSeen = false;            // $PQTMVERNO reply received over I2C

void i2cFeed(char c) {
  if (c == '$') { i2cInLine = true; i2cLen = 0; return; }
  if (!i2cInLine) return;
  if (c == '\r' || c == '\n') {
    i2cLine[i2cLen] = '\0';
    i2cInLine = false;
    if (LC76G::checksumOk(i2cLine)) {
      i2cGood++;
      if (strncmp(i2cLine, "PQTMVERNO,", 10) == 0) {
        i2cVersionSeen = true;
        Serial.print(F("  >>> reply over I2C: $"));
        Serial.println(i2cLine);
      } else if (i2cShown < 5) {            // Only the first few, as a sample
        i2cShown++;
        Serial.print(F("  NMEA over I2C: $"));
        Serial.println(i2cLine);
      }
    } else {
      i2cBad++;
    }
    return;
  }
  if (i2cLen < LC76G_LINE_MAX - 1) i2cLine[i2cLen++] = c;
  else i2cInLine = false;                   // Too long: drop it
}

// Reads the I2C stream for 'ms' milliseconds; returns the bytes read.
// The first 3 failures are explained, the others only counted.
uint32_t i2cReadFor(uint32_t ms) {
  uint8_t buf[I2C_CHUNK];
  uint32_t bytes = 0;
  uint16_t fail1 = 0, fail2 = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    uint32_t len = 0;
    if (!i2cAvailable(len)) {
      if (++fail1 <= 3) Serial.println(F("  read step 1: length not received"));
      delay(50);
      continue;
    }
    if (len == 0) { delay(I2C_STEP_MS); continue; }  // Nothing waiting yet
    uint8_t chunk = (I2C_READ_SIZE > I2C_CHUNK) ? I2C_CHUNK : I2C_READ_SIZE;
    uint8_t n = (len > chunk) ? chunk : (uint8_t)len;
    uint8_t code = i2cReadData(buf, n);
    if (code != 0) {
      if (++fail2 <= 3) {
        Serial.print(F("  read step 2 failed: waiting in the module "));
        Serial.print(len);
        Serial.print(F(" bytes, asked "));
        Serial.print(n);
        Serial.print(F(", "));
        printReadError(code);
        Serial.println();
      }
      delay(50);
      continue;
    }
    for (uint8_t i = 0; i < n; i++) i2cFeed((char)buf[i]);
    bytes += n;
  }
  Serial.print(F("  failures: step 1 "));
  Serial.print(fail1);
  Serial.print(F(", step 2 "));
  Serial.print(fail2);
  Serial.print(F(" | transactions repeated (module not ready) "));
  Serial.println(i2cRetries);
  return bytes;
}

void runI2cTest() {
  Serial.println(F("I2C link:"));
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  i2cDisableInternalPullups();
  Serial.println(F("  internal pull-ups off (bus pulled up to 3.3 V by the shield)"));

  // 1) Does the module answer at its configuration address? (0x54 and
  //    0x58 answer only inside a transfer started at 0x50: probing them
  //    alone gives NO answer, which is normal)
  Wire.beginTransmission(I2C_ADDR_CFG);
  Serial.print(F("  address 0x50: "));
  Serial.println(Wire.endTransmission() == 0 ? F("acknowledged") : F("NO answer"));
  Serial.print(F("  read size "));
  Serial.print(I2C_READ_SIZE);
  Serial.print(F(" bytes, clock "));
  Serial.print(I2C_CLOCK_HZ / 1000);
  Serial.println(F(" kHz"));

  // 2) Command written over I2C (this also wakes up the I2C transmitter
  //    if its buffer filled up while nobody was reading)
  printResult(F("  writing $PQTMVERNO over I2C"),
              i2cWriteText("$PQTMVERNO*58\r\n") ? LC76G_OK : LC76G_FAILED);

  // 3) Read the I2C stream for 5 seconds
  Serial.println(F("  reading for 5 s..."));
  uint32_t bytes = i2cReadFor(5000);
  Serial.print(F("  bytes read "));
  Serial.print(bytes);
  Serial.print(F(", sentences OK "));
  Serial.print(i2cGood);
  Serial.print(F(", damaged "));
  Serial.print(i2cBad);
  Serial.print(F(", firmware version reply "));
  Serial.println(i2cVersionSeen ? F("received") : F("NOT received"));
}
#endif


#ifdef TEST_LOCUS
// LOCUS status in readable form
void printLocusStatus() {
  bool on = false;
  Serial.print(F("  Recording: "));
  printFlag(gnss.getLocusEnabled(on), on);
  uint32_t count = 0;
  Serial.print(F("  Records stored: "));
  LC76G_Result r = gnss.getLocusRecordCount(count);
  if (r == LC76G_OK) Serial.println(count);
  else               Serial.println(LC76G::resultText(r));
}

// One CSV line per record
void onLocusRecord(const LC76G_LocusRecord &rec) {
  uint16_t y; uint8_t mo, d, h, mi, se;
  LC76G::unixToUtc(rec.utc, y, mo, d, h, mi, se);
  Serial.print(y);  Serial.print('-');
  print2Sep(mo, '-');  print2Sep(d, ',');       // Date yyyy-mm-dd
  print2Sep(h, ':');   print2Sep(mi, ':');  print2Sep(se, ',');   // Time hh:mm:ss
  Serial.print(rec.fixType);    Serial.print(',');
  printE7(rec.latE7);           Serial.print(',');
  printE7(rec.lonE7);           Serial.print(',');
  Serial.print(rec.heightRaw);  Serial.print(',');
  Serial.println(rec.sats);
}

// Reads all stored records (recording is stopped first by the library)
// and prints them as CSV, followed by the read-back counters
void doLocusDump() {
  LC76G_LocusDumpStats st;
  Serial.println(F("--- LOCUS DUMP BEGIN ---"));
  Serial.println(F("utc_date,utc_time,fix,latitude,longitude,height_m,sats"));
  LC76G_Result r = gnss.dumpLocus(onLocusRecord, st);
  Serial.println(F("--- LOCUS DUMP END ---"));
  Serial.print(F("Announced "));
  Serial.print(st.announced);
  Serial.print(F(", received "));
  Serial.print(st.received);
  Serial.print(F(", damaged "));
  Serial.print(st.rejected);
  Serial.print(F(", result "));
  Serial.println(LC76G::resultText(r));
}

// While recording: the character 'd' typed in the Serial Monitor stops
// LOCUS and reads it back WITHOUT resetting the Arduino. A reset (or a
// new upload) switches the module off and the recording would be lost.
// Note: on the Uno R3 opening the Serial Monitor resets the board, so
// keep the Monitor open for the whole test. The Uno R4 does not reset.
bool locusDumped = false;
void handleLocusCommand() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if ((c == 'd' || c == 'D') && !locusDumped) {
      locusDumped = true;                 // Only once: recording is now off
      doLocusDump();
      Serial.println(F("LOCUS after:"));
      printLocusStatus();
    }
  }
}

// Number of records every minute while recording
unsigned long lastLocusReport = 0;
void handleLocusReport() {
  if (locusDumped) {
    return;                               // Recording stopped by the dump
  }
  if (millis() - lastLocusReport < 60000UL || !gnss.isTalking()) {
    return;
  }
  lastLocusReport = millis();
  uint32_t count = 0;
  if (gnss.getLocusRecordCount(count) == LC76G_OK) {
    Serial.print(F(">>> LOCUS records stored: "));
    Serial.println(count);
  }
}
#endif

#ifdef TEST_RECEPTION
// All reception settings, one per line with the full name
void printReceptionSettings() {
  LC76G_Result r;
  bool on = false;
  int8_t deg = 0;
  Serial.print(F("  Elevation mask: "));
  r = gnss.getElevationMask(deg);
  if (r == LC76G_OK) { Serial.print(deg); Serial.println(F(" degrees")); }
  else               Serial.println(LC76G::resultText(r));
  Serial.print(F("  Active Interference Cancellation (AIC): "));
  r = gnss.getAic(on);               printFlag(r, on);
  Serial.print(F("  Satellite-Based Augmentation System (SBAS) search: "));
  r = gnss.getSbas(on);              printFlag(r, on);
  LC76G_DgpsSource src = LC76G_DGPS_NONE;
  Serial.print(F("  Differential GPS (DGPS) correction source: "));
  r = gnss.getDgpsSource(src);
  if (r == LC76G_OK) Serial.println(LC76G::dgpsSourceText(src));
  else               Serial.println(LC76G::resultText(r));
  Serial.print(F("  2D fix allowed: "));
  r = gnss.get2dFix(on);             printFlag(r, on);
  Serial.print(F("  Real-time speed response: "));
  r = gnss.getImmediateSpeed(on);    printFlag(r, on);
  Serial.print(F("  BeiDou (BDS) B1C band tracking: "));
  r = gnss.getBdsB1c(on);            printFlag(r, on);
}
#endif

#ifdef TEST_SETTINGS
// Prints the six NMEA decimal places on one line
void printDecimals(const __FlashStringHelper *label) {
  LC76G_NmeaDecimals d;
  Serial.print(label);
  LC76G_Result r = gnss.getNmeaDecimals(d);
  if (r != LC76G_OK) { Serial.println(LC76G::resultText(r)); return; }
  Serial.print(F("UTC ")); Serial.print(d.utc);
  Serial.print(F(", position ")); Serial.print(d.position);
  Serial.print(F(", altitude ")); Serial.print(d.altitude);
  Serial.print(F(", DOP ")); Serial.print(d.dop);
  Serial.print(F(", speed ")); Serial.print(d.speed);
  Serial.print(F(", course ")); Serial.println(d.course);
}

// Prints the Talker ID setting
void printTalker(const __FlashStringHelper *label) {
  char id[3];
  bool gsvSame = false;
  Serial.print(label);
  LC76G_Result r = gnss.getNmeaTalkerId(id, gsvSame);
  if (r != LC76G_OK) { Serial.println(LC76G::resultText(r)); return; }
  Serial.print(F("main "));
  Serial.print(id);
  if (strcmp(id, "00") == 0) Serial.print(F(" (automatic)"));
  Serial.print(F(", GSV "));
  Serial.println(gsvSame ? F("same as main") : F("by constellation"));
}

// Prints the three RTCM settings
void printRtcm(const __FlashStringHelper *label) {
  LC76G_RtcmMode mode = LC76G_RTCM_OFF;
  bool ant = false, eph = false;
  Serial.print(label);
  LC76G_Result r = gnss.getRtcmMode(mode);
  if (r != LC76G_OK) { Serial.println(LC76G::resultText(r)); return; }
  Serial.print(F("mode "));
  Serial.print(LC76G::rtcmModeText(mode));
  Serial.print(F(", antenna reference point (1005) "));
  r = gnss.getRtcmAntennaPoint(ant);
  Serial.print(r == LC76G_OK ? (ant ? F("on") : F("off")) : LC76G::resultText(r));
  Serial.print(F(", ephemeris "));
  r = gnss.getRtcmEphemeris(eph);
  Serial.println(r == LC76G_OK ? (eph ? F("on") : F("off")) : LC76G::resultText(r));
}

// Prints the minimum SNR
void printMinSnr(const __FlashStringHelper *label) {
  uint8_t snr = 0;
  Serial.print(label);
  LC76G_Result r = gnss.getMinSnr(snr);
  if (r != LC76G_OK) { Serial.println(LC76G::resultText(r)); return; }
  Serial.print(snr);
  Serial.println(F(" dB-Hz"));
}

// Prints the Galileo Return Link Message output state
void printRlm(const __FlashStringHelper *label) {
  bool on = false;
  Serial.print(label);
  printFlag(gnss.getRlmOutput(on), on);
}

void runSettingsTest() {
  // Minimum Signal-to-Noise Ratio: 15, then back to 9
  Serial.println(F("Minimum Signal-to-Noise Ratio (SNR):"));
  printMinSnr(F("  before: "));
  printResult(F("  set 15 dB-Hz"), gnss.setMinSnr(15));
  printMinSnr(F("  read back: "));
  printResult(F("  restore 9 dB-Hz"), gnss.setMinSnr(9));

  // NMEA decimal places: example of the specification, then the factory set
  Serial.println(F("NMEA decimal places:"));
  printDecimals(F("  before: "));
  const LC76G_NmeaDecimals testDp    = { 3, 8, 1, 2, 3, 2 };
  const LC76G_NmeaDecimals factoryDp = { 3, 6, 3, 2, 2, 2 };
  printResult(F("  set 3, 8, 1, 2, 3, 2"), gnss.setNmeaDecimals(testDp));
  printDecimals(F("  read back: "));
  printResult(F("  restore 3, 6, 3, 2, 2, 2"), gnss.setNmeaDecimals(factoryDp));

  // Talker ID: "GP", then automatic
  Serial.println(F("NMEA Talker ID:"));
  printTalker(F("  before: "));
  printResult(F("  set GP"), gnss.setNmeaTalkerId("GP"));
  printTalker(F("  read back: "));
  printResult(F("  restore automatic"), gnss.setNmeaTalkerId("00"));

  // RTCM: MSM4 + ephemeris for one read-back only, then all off again
  Serial.println(F("RTCM output:"));
  printRtcm(F("  before: "));
  printResult(F("  set MSM4"), gnss.setRtcmMode(LC76G_RTCM_MSM4));
  printResult(F("  set ephemeris on"), gnss.setRtcmEphemeris(true));
  printRtcm(F("  read back: "));
  printResult(F("  restore off"), gnss.setRtcmMode(LC76G_RTCM_OFF));
  printResult(F("  restore ephemeris off"), gnss.setRtcmEphemeris(false));
  printRtcm(F("  after: "));

  // Galileo Return Link Message
  Serial.println(F("Galileo Return Link Message (RLM):"));
  printRlm(F("  before: "));
  printResult(F("  set on"), gnss.setRlmOutput(true));
  printRlm(F("  read back: "));
  printResult(F("  restore off"), gnss.setRlmOutput(false));

  // 1PPS / NMEA synchronisation: no read-back command, only the answer
  Serial.println(F("1PPS / NMEA synchronisation:"));
  printResult(F("  set on"), gnss.setPpsNmeaSync(true));
  printResult(F("  set off"), gnss.setPpsNmeaSync(false));
}
#endif

#ifdef TEST_SUPPORT
// Copies a command name stored in flash (PSTR) into a RAM buffer.
// On the Uno R3 this keeps the many names out of the 2 KB of RAM.
void copyName(char *buf, uint8_t size, PGM_P name) {
  strncpy_P(buf, name, size - 1);
  buf[size - 1] = '\0';
}

// Prints "  <label>: " at the start of every line of this test
void printLabel(const __FlashStringHelper *label) {
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(": "));
}

// Prints the outcome of a command that failed
void printProbeError(LC76G_Result r) {
  if (r == LC76G_NOT_SUPPORTED || r == LC76G_PARAM_ERROR) {
    Serial.print(F("not supported ("));        // The firmware rejected it
    Serial.print(LC76G::resultText(r));
    Serial.println(')');
  } else {
    Serial.println(LC76G::resultText(r));      // e.g. timeout: no answer at all
  }
}

// Asks the output rate of one PQTM message: if the firmware answers,
// the message exists on this module.
void probeMessage(const __FlashStringHelper *label, PGM_P name, uint8_t msgVer) {
  char buf[24];
  copyName(buf, sizeof(buf), name);
  uint8_t rate = 0;
  LC76G_Result r = gnss.getPqtmMessageRate(buf, msgVer, rate);
  printLabel(label);
  if (r != LC76G_OK) {
    printProbeError(r);
    return;
  }
  Serial.print(F("supported, "));
  if (rate == 0) {
    Serial.println(F("off"));
  } else {
    Serial.print(F("every "));
    Serial.print(rate);
    Serial.println(F(" fix(es)"));
  }
}

// Sends the read form ("R") of a PQTM setting command and prints the
// raw reply, so the current values can be checked against the manual.
void probePqtmRead(const __FlashStringHelper *label, PGM_P name) {
  char buf[24];
  copyName(buf, sizeof(buf), name);
  char resp[64];
  LC76G_Result r = gnss.sendPqtm(buf, "R", LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  printLabel(label);
  if (r != LC76G_OK) {
    printProbeError(r);
    return;
  }
  Serial.print(F("supported, reply "));
  Serial.println(resp);
}

// Sends a PAIR query (no parameters) and prints the raw query result
void probePairQuery(const __FlashStringHelper *label, uint16_t id) {
  char resp[48];
  LC76G_Result r = gnss.sendPair(id, nullptr, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  printLabel(label);
  if (r != LC76G_OK) {
    printProbeError(r);
    return;
  }
  Serial.print(F("supported, reply "));
  Serial.println(resp);
}

// The whole check, grouped as in the protocol specification
void runSupportCheck() {
  Serial.println(F("PQTM output messages (PQTMCFGMSGRATE):"));
  probeMessage(F("Position, Velocity and Time (PQTMPVT)"),        PSTR("PQTMPVT"), 1);
  probeMessage(F("Velocity (PQTMVEL)"),                           PSTR("PQTMVEL"), 1);
  probeMessage(F("Dilution of Precision (PQTMDOP)"),              PSTR("PQTMDOP"), 1);
  probeMessage(F("Protection Level (PQTMPL)"),                    PSTR("PQTMPL"), 1);
  probeMessage(F("Odometer (PQTMODO)"),                           PSTR("PQTMODO"), 1);
  probeMessage(F("GPS Time (PQTMTIMEGPS)"),                       PSTR("PQTMTIMEGPS"), 1);
  probeMessage(F("ECEF Position (PQTMPOSECEF)"),                  PSTR("PQTMPOSECEF"), 1);
  probeMessage(F("ECEF Velocity (PQTMVELECEF)"),                  PSTR("PQTMVELECEF"), 1);
  probeMessage(F("ECEF Position, Velocity, Time (PQTMPVTECEF)"),  PSTR("PQTMPVTECEF"), 1);
  probeMessage(F("Antenna Status (PQTMANTENNASTATUS)"),           PSTR("PQTMANTENNASTATUS"), 1);
  probeMessage(F("Leap Second (PQTMLS)"),                         PSTR("PQTMLS"), 1);
  probeMessage(F("UTC Time (PQTMUTC)"),                           PSTR("PQTMUTC"), 1);
  probeMessage(F("Jamming Status (PQTMJAMMINGSTATUS)"),           PSTR("PQTMJAMMINGSTATUS"), 1);

  Serial.println(F("PQTM settings (read only):"));
  probePqtmRead(F("Odometer configuration (PQTMCFGODO)"),         PSTR("PQTMCFGODO"));
  probePqtmRead(F("NMEA decimal places (PQTMCFGNMEADP)"),         PSTR("PQTMCFGNMEADP"));
  probePqtmRead(F("NMEA Talker ID (PQTMCFGNMEATID)"),             PSTR("PQTMCFGNMEATID"));
  probePqtmRead(F("Constellations (PQTMCFGCNST)"),                PSTR("PQTMCFGCNST"));
  probePqtmRead(F("Antenna mode (PQTMCFGANTENNA)"),               PSTR("PQTMCFGANTENNA"));
  probePqtmRead(F("Sub-meter Level Augmentation Service (PQTMCFGSLAS)"), PSTR("PQTMCFGSLAS"));

  Serial.println(F("PAIR settings (read only):"));
  probePairQuery(F("Minimum Signal-to-Noise Ratio (PAIR059)"),    59);
  probePairQuery(F("Return Link Message output (PAIR155)"),       155);
  probePairQuery(F("Sub-meter Level Augmentation Service (PAIR421)"), 421);
  probePairQuery(F("RTCM output mode (PAIR433)"),                 433);
  probePairQuery(F("RTCM antenna reference point (PAIR435)"),     435);
  probePairQuery(F("RTCM ephemeris output (PAIR437)"),            437);
  probePairQuery(F("Position fix interval (PAIR051)"),            51);
  probePairQuery(F("PPS configuration (PAIR763)"),                763);
}
#endif

#ifdef TEST_PVT
// Last messages received. The callbacks run inside gnss.update(): they
// only copy the data, printing happens in the status block.
LC76G_Pvt      lastPvt;
LC76G_Velocity lastVel;
LC76G_Dop      lastDop;
unsigned long  pvtMs = 0, velMs = 0, dopMs = 0;   // millis() of arrival, 0 = never

void onPvtMessage(const LC76G_Pvt &p)           { lastPvt = p; pvtMs = millis(); }
void onVelocityMessage(const LC76G_Velocity &v) { lastVel = v; velMs = millis(); }
void onDopMessage(const LC76G_Dop &d)           { lastDop = d; dopMs = millis(); }

// true if a message arrived in the last 3 seconds
bool isRecent(unsigned long t) {
  return t != 0 && millis() - t < 3000UL;
}

// Prints the three messages, one block each
void printPvtBlock() {
  Serial.print(F("    Position, Velocity and Time (PQTMPVT): "));
  if (!isRecent(pvtMs)) {
    Serial.println(F("no recent message"));
  } else {
    const LC76G_Pvt &p = lastPvt;
    Serial.print(p.year);  Serial.print('-');
    print2Sep(p.month, '-'); print2Sep(p.day, ' ');
    print2Sep(p.hour, ':');  print2Sep(p.minute, ':'); print2Sep(p.second, ' ');
    Serial.print(F("UTC, time of week "));
    Serial.print(p.towMs);
    Serial.print(F(" ms, fix mode "));
    Serial.print(p.fixMode == 3 ? F("3D") : (p.fixMode == 2 ? F("2D") : F("none")));
    Serial.print(F(", satellites used "));
    Serial.print(p.satsUsed);
    Serial.print(F(", leap seconds "));
    if (p.leapSeconds == LC76G_LEAP_UNKNOWN) Serial.println(F("unknown"));
    else                                     Serial.println(p.leapSeconds);

    Serial.print(F("      position: "));
    if (!p.positionValid) {
      Serial.println(F("not valid"));
    } else {
      Serial.print(F("latitude "));      printE7(p.latE7);
      Serial.print(F(", longitude "));   printE7(p.lonE7);
      Serial.print(F(", altitude "));    printMilli(p.altitudeMm);
      Serial.print(F(" m, geoid separation ")); printMilli(p.geoidSepMm);
      Serial.println(F(" m"));
    }
    Serial.print(F("      velocity: "));
    if (!p.velocityValid) {
      Serial.println(F("not valid"));
    } else {
      Serial.print(F("north "));  printMilli(p.velNorthMmS);
      Serial.print(F(", east ")); printMilli(p.velEastMmS);
      Serial.print(F(", down ")); printMilli(p.velDownMmS);
      Serial.print(F(" m/s, ground speed ")); printMilli((int32_t)p.speedMmS);
      Serial.print(F(" m/s, heading ")); printX100(p.headingX100);
      Serial.println(F(" degrees"));
    }
    Serial.print(F("      Horizontal Dilution of Precision (HDOP) ")); printX100(p.hdopX100);
    Serial.print(F(", Position Dilution of Precision (PDOP) "));        printX100(p.pdopX100);
    Serial.println();
  }

  Serial.print(F("    Velocity (PQTMVEL): "));
  if (!isRecent(velMs)) {
    Serial.println(F("no recent message"));
  } else if (!lastVel.valid) {
    Serial.println(F("not valid"));
  } else {
    const LC76G_Velocity &v = lastVel;
    Serial.print(F("ground speed "));  printMilli((int32_t)v.groundSpeedMmS);
    Serial.print(F(" +/- "));          printMilli((int32_t)v.groundSpeedAccMmS);
    Serial.print(F(" m/s, 3D speed ")); printMilli((int32_t)v.speed3dMmS);
    Serial.print(F(" +/- "));          printMilli((int32_t)v.speedAccMmS);
    Serial.print(F(" m/s, heading ")); printX100(v.headingX100);
    Serial.print(F(" +/- "));          printX100(v.headingAccX100);
    Serial.println(F(" degrees"));
  }

  Serial.print(F("    Dilution of Precision (PQTMDOP): "));
  if (!isRecent(dopMs)) {
    Serial.println(F("no recent message"));
  } else {
    const LC76G_Dop &d = lastDop;
    Serial.print(F("geometric (GDOP) "));   printX100(d.gdopX100);
    Serial.print(F(", position (PDOP) "));  printX100(d.pdopX100);
    Serial.print(F(", time (TDOP) "));      printX100(d.tdopX100);
    Serial.print(F(", vertical (VDOP) "));  printX100(d.vdopX100);
    Serial.print(F(", horizontal (HDOP) ")); printX100(d.hdopX100);
    Serial.println();
  }
}
#endif

#ifdef TEST_EXTRA
// Last messages received (copied by the callbacks, printed later)
LC76G_Odometer        lastOdo;
LC76G_GpsTime         lastTime;
LC76G_ProtectionLevel lastPl;
LC76G_EcefPosition    lastEcefPos;
LC76G_EcefVelocity    lastEcefVel;
LC76G_EcefPvt         lastEcefPvt;
unsigned long odoMs = 0, timeMs = 0, plMs = 0, ecefPosMs = 0, ecefVelMs = 0, ecefPvtMs = 0;

void onOdometerMessage(const LC76G_Odometer &o)           { lastOdo = o;     odoMs = millis(); }
void onGpsTimeMessage(const LC76G_GpsTime &t)             { lastTime = t;    timeMs = millis(); }
void onProtectionLevelMessage(const LC76G_ProtectionLevel &p) { lastPl = p;  plMs = millis(); }
void onEcefPositionMessage(const LC76G_EcefPosition &e)   { lastEcefPos = e; ecefPosMs = millis(); }
void onEcefVelocityMessage(const LC76G_EcefVelocity &e)   { lastEcefVel = e; ecefVelMs = millis(); }
void onEcefPvtMessage(const LC76G_EcefPvt &e)             { lastEcefPvt = e; ecefPvtMs = millis(); }

// true if a message arrived in the last 3 seconds
bool isRecentExtra(unsigned long t) {
  return t != 0 && millis() - t < 3000UL;
}

// Prints centimetres as metres with 2 decimals and sign, e.g. -12345 -> -123.45
void printCenti(int32_t cm) {
  if (cm < 0) { Serial.print('-'); cm = -cm; }
  printX100((uint32_t)cm);
}

// Prints "x X, y Y, z Z" from centimetres
void printEcefCm(int32_t x, int32_t y, int32_t z) {
  Serial.print(F("x "));    printCenti(x);
  Serial.print(F(", y "));  printCenti(y);
  Serial.print(F(", z "));  printCenti(z);
  Serial.print(F(" m"));
}

// Prints "x X, y Y, z Z" from mm/s
void printEcefMmS(int32_t x, int32_t y, int32_t z) {
  Serial.print(F("x "));    printMilli(x);
  Serial.print(F(", y "));  printMilli(y);
  Serial.print(F(", z "));  printMilli(z);
  Serial.print(F(" m/s"));
}

void printExtraBlock() {
  Serial.print(F("    Odometer (PQTMODO): "));
  if (!isRecentExtra(odoMs)) {
    Serial.println(F("no recent message"));
  } else {
    Serial.print(lastOdo.enabled ? F("enabled") : F("disabled"));
    Serial.print(F(", distance "));
    Serial.print(lastOdo.distanceDm / 10);
    Serial.print('.');
    Serial.print(lastOdo.distanceDm % 10);
    Serial.println(F(" m"));
  }

  Serial.print(F("    GPS Time (PQTMTIMEGPS): "));
  if (!isRecentExtra(timeMs)) {
    Serial.println(F("no recent message"));
  } else if (!lastTime.valid) {
    Serial.println(F("not valid"));
  } else {
    Serial.print(F("week "));
    Serial.print(lastTime.week);
    Serial.print(F(", time of week "));
    Serial.print(lastTime.towMs);
    Serial.print(F(" ms + "));
    Serial.print(lastTime.towFracNs);
    Serial.print(F(" ns, leap seconds "));
    if (lastTime.leapSeconds == LC76G_LEAP_UNKNOWN) Serial.print(F("unknown"));
    else                                            Serial.print(lastTime.leapSeconds);
    Serial.print(F(", accuracy "));
    if (lastTime.accuracyNs == 0xFFFFFFFFUL) Serial.println(F("unknown"));
    else { Serial.print(lastTime.accuracyNs); Serial.println(F(" ns")); }
  }

  Serial.print(F("    Protection Level (PQTMPL): "));
  if (!isRecentExtra(plMs)) {
    Serial.println(F("no recent message"));
  } else if (!lastPl.valid) {
    Serial.println(F("not valid"));
  } else {
    Serial.print(F("probability "));
    printX100(lastPl.probabilityX100);
    Serial.print(F(" %, position north "));
    printMilli((int32_t)lastPl.northMm);
    Serial.print(F(", east "));
    printMilli((int32_t)lastPl.eastMm);
    Serial.print(F(", down "));
    printMilli((int32_t)lastPl.downMm);
    Serial.print(F(" m, time "));
    Serial.print(lastPl.timeNs);
    Serial.println(F(" ns"));
  }

  Serial.print(F("    ECEF Position (PQTMPOSECEF): "));
  if (!isRecentExtra(ecefPosMs))   Serial.println(F("no recent message"));
  else if (!lastEcefPos.valid)     Serial.println(F("not valid"));
  else {
    printEcefCm(lastEcefPos.xCm, lastEcefPos.yCm, lastEcefPos.zCm);
    Serial.print(F(", accuracy "));
    printX100(lastEcefPos.accuracyCm);
    Serial.println(F(" m"));
  }

  Serial.print(F("    ECEF Velocity (PQTMVELECEF): "));
  if (!isRecentExtra(ecefVelMs))   Serial.println(F("no recent message"));
  else if (!lastEcefVel.valid)     Serial.println(F("not valid"));
  else {
    printEcefMmS(lastEcefVel.vxMmS, lastEcefVel.vyMmS, lastEcefVel.vzMmS);
    Serial.print(F(", accuracy "));
    printMilli((int32_t)lastEcefVel.accuracyMmS);
    Serial.println(F(" m/s"));
  }

  Serial.print(F("    ECEF Position, Velocity, Time (PQTMPVTECEF): "));
  if (!isRecentExtra(ecefPvtMs))   Serial.println(F("no recent message"));
  else if (!lastEcefPvt.valid)     Serial.println(F("not valid"));
  else {
    print2Sep(lastEcefPvt.hour, ':'); print2Sep(lastEcefPvt.minute, ':');
    print2Sep(lastEcefPvt.second, ' ');
    Serial.print(F("UTC, "));
    printEcefCm(lastEcefPvt.xCm, lastEcefPvt.yCm, lastEcefPvt.zCm);
    Serial.print(F(", "));
    printEcefMmS(lastEcefPvt.vxMmS, lastEcefPvt.vyMmS, lastEcefPvt.vzMmS);
    Serial.println();
  }
}
#endif

#ifdef TEST_EPE
// Prints a value in millimetres as metres with 2 decimals, e.g. 1414 -> 1.41 m
void printMetres(uint32_t mm) {
  printX100((mm + 5) / 10);                     // mm -> cm, rounded
  Serial.print(F(" m"));
}
#endif


// ===========================================================================
//  STATUS BLOCK (every STATUS_EVERY_EPOCHS seconds)
//  Called by the library after each epoch: it only prints. Commands must
//  NOT be sent from here (it runs inside gnss.update()): loop() does that.
// ===========================================================================
void onEpoch() {
  epochPending = true;
  if (++epochCounter < STATUS_EVERY_EPOCHS) {
    return;
  }
  epochCounter = 0;

  // Common line: fix, satellites, geometry, signal
  const LC76G_Fix &f = gnss.nmea.fix();
  Serial.print(F("--- Fix quality: "));
  Serial.print(LC76G::fixQualityText(f.quality));
  Serial.print(F(" | satellites used "));
  Serial.print(f.satsUsed);
  Serial.print(F(", in view "));
  Serial.print(gnss.nmea.satsInViewTotal());
  Serial.print(F(" | Horizontal Dilution of Precision (HDOP) "));
  printX100(f.hdopX100);
  Serial.print(F(" | strongest Signal-to-Noise Ratio (SNR) "));
  Serial.print(gnss.nmea.maxSnr());
  Serial.println(F(" dB-Hz"));

#ifdef TEST_CONSTELLATIONS
  // Satellites in view per constellation (LC76G_System i <-> bit 1 << i)
  for (uint8_t i = 0; i < LC76G_NUM_SYSTEMS; i++) {
    Serial.print(F("    "));
    Serial.print(LC76G::constellationName((uint8_t)(1u << i)));
    Serial.print(F(": "));
    Serial.print(gnss.nmea.satsInView((LC76G_System)i));
    Serial.println(F(" in view"));
  }
#endif

#ifdef TEST_JAMMING
  Serial.print(F("    Jamming status message: "));
  Serial.print(LC76G::jamStatusText(gnss.jammingStatus()));
  Serial.print(F(" | JAM_IND pin (D8): "));
  Serial.println(gnss.jamIndicatorActive() ? F("LOW = jamming") : F("HIGH = no jamming"));
#endif

#ifdef TEST_GEOFENCE
  Serial.print(F("    Geofence 0: "));
  Serial.print(LC76G::geofenceStateText(gnss.geofenceState(0)));
  Serial.print(F(" | GEOFENCE pin (D7): "));
  Serial.println(gnss.geofencePinInside() ? F("HIGH = inside") : F("LOW = outside"));
#endif

#ifdef TEST_PPS
  Serial.print(F("    1PPS on D2: "));
  if (gnss.ppsCount() == 0 || gnss.ppsAgeMs() > 2000) {
    Serial.println(F("no pulses"));
  } else {
    uint32_t period = gnss.ppsPeriodUs();
    Serial.print(gnss.ppsCount());
    Serial.print(F(" pulses, period "));
    Serial.print(period);
    Serial.print(F(" us, width "));
    Serial.print(gnss.ppsWidthUs());
    Serial.print(F(" us, host clock error "));
    Serial.print((long)period - 1000000L);      // 1 us over 1 s = 1 ppm
    Serial.println(F(" ppm"));
  }
#endif

#ifdef TEST_PVT
  printPvtBlock();
#endif

#ifdef TEST_EXTRA
  printExtraBlock();
#endif

#ifdef TEST_EPE
  Serial.print(F("    Estimated Positioning Error (EPE): "));
  if (gnss.epeAgeMs() > 3000) {
    Serial.println(F("no recent message"));
  } else {
    const LC76G_Epe &e = gnss.epe();
    Serial.print(F("horizontal "));
    printMetres(e.horizontalMm);
    Serial.print(F(", vertical "));
    printMetres(e.downMm);
    Serial.print(F(", 3D "));
    printMetres(e.total3dMm);
    Serial.println();
  }
#endif
}


// ===========================================================================
//  STATUS LED (non-blocking)
// ===========================================================================
void updateStatusLed() {
  unsigned long now = millis();
  bool on;
#ifdef TEST_GEOFENCE
  if (LED_SHOWS_GEOFENCE) {
    LC76G_GeofenceState st = gnss.geofenceState(0);
    if (st == LC76G_GEO_UNKNOWN)     on = (now / 100) % 2;   // Fast: no geofence / no fix
    else if (st == LC76G_GEO_INSIDE) on = true;              // Steady: inside
    else                             on = (now / 500) % 2;   // Slow: outside
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
    return;
  }
#endif
  if (!gnss.isTalking())          on = (now / 100) % 2;      // Fast blink: no data
  else if (gnss.nmea.fix().valid) on = true;                 // Steady: fix
  else                            on = (now / 500) % 2;      // Slow blink: searching
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}


// ===========================================================================
//  SETUP
// ===========================================================================
void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);              // Never leave D13 floating (Uno R3 op-amp)
  Serial.begin(115200);
  // Uno R4: the USB port is native, wait (max 3 s) for the Serial Monitor
  // so the first lines are not lost. On the Uno R3 this returns at once.
  while (!Serial && millis() < 3000) {
  }
  Serial.println();
  Serial.print(F("=== Quectel_LC76G ShieldTest v"));
  Serial.print(F(QUECTEL_LC76G_VERSION));
  Serial.println(F(" ==="));

  // Rev. 1 shield: control pins go straight to NPN bases -> weak pull-up mode
  gnss.setPowerPin(PIN_PWR_EN, LC76G_DRIVE_WEAK_PULLUP_HIGH);
  gnss.setResetPin(PIN_RESET_DRV, LC76G_DRIVE_WEAK_PULLUP_HIGH);

#if defined(ARDUINO_ARCH_RENESAS)
  // Uno R4: D4/D5 are wired to D0/D1 on the shield, so they must stay
  // high-impedance inputs and never drive the lines used by Serial1
  pinMode(PIN_GNSS_TXD, INPUT);
  pinMode(PIN_GNSS_RXD, INPUT);
  Serial.println(F("Board: Uno R4, module on Serial1 (D0/D1)"));
#else
  Serial.println(F("Board: Uno R3, module on SoftwareSerial (D4/D5)"));
#endif

  // Start-up; openGnssPort() reopens the port at the speed the library needs
  if (!gnss.begin(gnssSerial, openGnssPort, GNSS_BAUD)) {
    Serial.println(F("ERROR: module not responding."));
    return;
  }
  Serial.print(F("Module OK at "));
  Serial.print(gnss.baud());
  Serial.println(F(" baud"));

#ifdef TEST_FACTORY_RESET
  // ---- Factory reset: first, so the other tests start from the defaults ----
  Serial.println(F("NMEA output before the factory reset:"));
  printNmeaRates();
  printResult(F("Restoring factory defaults (the module reboots)"), gnss.restoreDefaults());
  Serial.println(F("NMEA output after the factory reset:"));
  printNmeaRates();
#endif

  // ---- NMEA output: minimal set by default (less SoftwareSerial traffic) ----
#ifdef TEST_NMEA_OUTPUT
  if (NMEA_FACTORY_SET) {
    LC76G_Result r = LC76G_OK;
    for (uint8_t t = LC76G_NMEA_GGA; t <= LC76G_NMEA_VTG && r == LC76G_OK; t++) {
      r = gnss.setNmeaRate((LC76G_NmeaType)t, 1);
    }
    printResult(F("Factory NMEA output set"), r);
  } else
#endif
  {
    printResult(F("Minimal NMEA output"), gnss.setMinimalNmeaOutput(1));
  }
#ifdef TEST_NMEA_OUTPUT
  Serial.println(F("NMEA output:"));
  printNmeaRates();
#endif

#ifdef TEST_CONSTELLATIONS
  Serial.println(F("Constellations before:"));
  printConstellations();
  printResult(F("Setting constellations (the module may reboot)"), gnss.setConstellations(CONSTELLATIONS));
  Serial.println(F("Constellations after:"));
  printConstellations();
#endif

#ifdef TEST_JAMMING
  gnss.setJamIndicatorPin(PIN_JAM_IND);
  printResult(F("Enabling jamming detection"), gnss.setJammingDetection(true));
#endif

#ifdef TEST_GEOFENCE
  gnss.setGeofencePin(PIN_GEOFENCE);
  printResult(F("Disabling old geofence 0"), gnss.disableGeofence(0));
  printResult(F("Enabling geofence status message"), gnss.setGeofenceStatusOutput(1));
  Serial.println(F("Geofence 0 will be created at the first fix"));
#endif

#ifdef TEST_PPS
  Serial.println(F("1PPS before:"));
  printPpsConfig();
  printResult(F("Configuring 1PPS"), gnss.setPps(true, PPS_DURATION_MS, PPS_MODE, true));
  Serial.println(F("1PPS after:"));
  printPpsConfig();
  if (!gnss.beginPpsCapture(PIN_PPS)) {
    Serial.println(F("ERROR: the 1PPS pin cannot generate interrupts"));
  }
#endif

#ifdef TEST_LOW_POWER
  Serial.println(F("Low power before:"));
  printLowPowerConfig();
  printResult(F("Leaving periodic mode if active (up to 60 s)"), gnss.disablePeriodicMode(60000UL));
  printResult(F("Navigation mode normal"), gnss.setNavigationMode(LC76G_NAV_NORMAL));
  switch (LOW_POWER_MODE) {
    case LP_ALP1:     printResult(F("ALP mode 1"), gnss.setAlpMode(1)); break;
    case LP_ALP2:     printResult(F("ALP mode 2"), gnss.setAlpMode(2)); break;
    case LP_PERIODIC: gnss.setAlpMode(0);
                      printResult(F("Periodic mode"), gnss.setPeriodicMode(PERIODIC_CFG)); break;
    default:          printResult(F("ALP off (continuous)"), gnss.setAlpMode(0)); break;
  }
  Serial.println(F("Low power after:"));
  printLowPowerConfig();
  backupT0 = millis();
#endif

#ifdef TEST_AGNSS
  Serial.println(F("Assisted GNSS (AGNSS):"));
  printAgnssStatus();
  if (epocAvailable) {
    printResult(F("Enabling EPOC"), gnss.setEpoc(true));
    printResult(F("Setting EPOC constellations"), gnss.setEpocConstellations(EPOC_CONSTELLATIONS));
  }
#endif

#ifdef TEST_LOCUS
  Serial.println(F("LOCUS before:"));
  printLocusStatus();
  if (LOCUS_ACTION == LOCUS_START) {
    printResult(F("LOCUS save mode"), gnss.setLocusMode(LC76G_LOCUS_TIME, LOCUS_REQUIRE_3D_FIX));
    printResult(F("LOCUS time threshold"), gnss.setLocusThreshold(LC76G_LOCUS_BY_TIME, LOCUS_PERIOD_S));
    printResult(F("Starting LOCUS recording"), gnss.setLocusEnabled(true));
    Serial.println(F(">>> Type d + Enter in the Serial Monitor to stop and read back"));
  } else if (LOCUS_ACTION == LOCUS_DUMP) {
    doLocusDump();
  } else if (LOCUS_ACTION == LOCUS_CLEAR) {
    printResult(F("Erasing LOCUS data"), gnss.clearLocus(LC76G_LOCUS_CLEAR_DATA));
  }
  if (LOCUS_ACTION != LOCUS_STATUS) {
    Serial.println(F("LOCUS after:"));
    printLocusStatus();
  }
#endif

#ifdef TEST_RECEPTION
  Serial.println(F("Reception settings before:"));
  printReceptionSettings();
  printResult(F("Elevation mask"),           gnss.setElevationMask(ELEVATION_MASK_DEG));
  printResult(F("AIC"),                      gnss.setAic(AIC_ON));
  printResult(F("SBAS search"),              gnss.setSbas(SBAS_ON));
  printResult(F("DGPS source"),              gnss.setDgpsSource(DGPS_SOURCE));
  printResult(F("2D fix"),                   gnss.set2dFix(FIX_2D_ON));
  printResult(F("Real-time speed response"), gnss.setImmediateSpeed(IMMEDIATE_SPEED_ON));
  printResult(F("BDS B1C band"),             gnss.setBdsB1c(BDS_B1C_ON));
  printResult(F("Static threshold"),         gnss.setStaticThreshold(STATIC_DMS));
  Serial.println(F("Reception settings after:"));
  printReceptionSettings();
#endif

#ifdef TEST_PVT
  // Callbacks first, then the outputs (every fix)
  gnss.onPvt(onPvtMessage);
  gnss.onVelocity(onVelocityMessage);
  gnss.onDop(onDopMessage);
  printResult(F("Enabling Position, Velocity and Time message (PQTMPVT)"), gnss.setPvtOutput(1));
  printResult(F("Enabling Velocity message (PQTMVEL)"), gnss.setVelocityOutput(1));
  printResult(F("Enabling Dilution of Precision message (PQTMDOP)"), gnss.setDopOutput(1));
#endif

#ifdef TEST_EXTRA
  // Callbacks (6 decoder slots), then odometer and message outputs
  bool hooksOk = gnss.onOdometer(onOdometerMessage) && gnss.onGpsTime(onGpsTimeMessage) &&
                 gnss.onProtectionLevel(onProtectionLevelMessage) &&
                 gnss.onEcefPosition(onEcefPositionMessage) &&
                 gnss.onEcefVelocity(onEcefVelocityMessage) && gnss.onEcefPvt(onEcefPvtMessage);
  if (!hooksOk) {
    Serial.println(F("WARNING: decoder table full (disable TEST_PVT)"));
  }
  {
    bool odoOn = false;
    uint32_t odoInitDm = 0;
    Serial.print(F("Odometer before: "));
    LC76G_Result r = gnss.getOdometer(odoOn, odoInitDm);
    if (r == LC76G_OK) {
      Serial.print(odoOn ? F("enabled") : F("disabled"));
      Serial.print(F(", initial distance "));
      Serial.print(odoInitDm / 10); Serial.print('.'); Serial.print(odoInitDm % 10);
      Serial.println(F(" m"));
    } else {
      Serial.println(LC76G::resultText(r));
    }
  }
  printResult(F("Enabling the odometer"), gnss.setOdometer(true, 0));
  printResult(F("Resetting the odometer"), gnss.resetOdometer());
  printResult(F("Enabling Odometer message (PQTMODO)"), gnss.setOdometerOutput(1));
  printResult(F("Enabling GPS Time message (PQTMTIMEGPS)"), gnss.setGpsTimeOutput(1));
  printResult(F("Enabling Protection Level message (PQTMPL)"), gnss.setProtectionLevelOutput(1));
  printResult(F("Enabling ECEF Position message (PQTMPOSECEF)"), gnss.setEcefPositionOutput(1));
  printResult(F("Enabling ECEF Velocity message (PQTMVELECEF)"), gnss.setEcefVelocityOutput(1));
  printResult(F("Enabling ECEF Position, Velocity, Time message (PQTMPVTECEF)"), gnss.setEcefPvtOutput(1));
#endif

#ifdef TEST_I2C
  runI2cTest();                                 // I2C hardware and protocol check
#endif

#ifdef TEST_SETTINGS
  runSettingsTest();                            // Change, read back, restore
#endif

#ifdef TEST_SUPPORT
  runSupportCheck();                            // Read-only firmware check
#endif

#ifdef TEST_EPE
  printResult(F("Enabling Estimated Positioning Error message (PQTMEPE)"), gnss.setEpeOutput(1));
#endif

  gnss.nmea.onEpoch(onEpoch);
  Serial.println(F("--- Monitoring ---"));
}


// ===========================================================================
//  LOOP
// ===========================================================================
void loop() {
  gnss.update();                                // Read and decode everything

  // Work that sends commands runs here, once per epoch or on its own timer
  if (epochPending) {
    epochPending = false;
#ifdef TEST_GEOFENCE
    // First fix: create the circle around the current position
    static bool fenceCreated = false;
    const LC76G_Fix &f = gnss.nmea.fix();
    if (!fenceCreated && f.valid && f.positionValid) {
      LC76G_Result r = gnss.setGeofenceCircle(0, f.latE7, f.lonE7, GEOFENCE_RADIUS_M);
      printResult(F(">>> First fix: creating geofence 0"), r);
      fenceCreated = (r == LC76G_OK);
    }
    // Print every change of state
    static LC76G_GeofenceState lastState = LC76G_GEO_UNKNOWN;
    LC76G_GeofenceState st = gnss.geofenceState(0);
    if (fenceCreated && st != lastState) {
      Serial.print(F(">>> Geofence 0: "));
      Serial.print(LC76G::geofenceStateText(lastState));
      Serial.print(F(" -> "));
      Serial.println(LC76G::geofenceStateText(st));
      lastState = st;
    }
#endif
  }

#ifdef TEST_LOW_POWER
  if (LOW_POWER_MODE == LP_BACKUP) {
    handleBackupTest();
  }
#endif
#ifdef TEST_AGNSS
  handleAgnssReport();
#endif
#ifdef TEST_LOCUS
  if (LOCUS_ACTION == LOCUS_START) {
    handleLocusReport();                        // Record count every minute
    handleLocusCommand();                       // 'd' = stop and read back
  }
#endif

  updateStatusLed();
}
