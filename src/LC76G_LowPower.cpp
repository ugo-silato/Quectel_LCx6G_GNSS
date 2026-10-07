/*
 * ============================================================================
 *  LC76G_LowPower.cpp  -  Low power modes of the LC76G
 * ============================================================================
 *  Part of the Quectel_LC76G Arduino library. Author: Ugo Silato. License: MIT.
 *
 *  Commands used (Quectel GNSS Protocol Specification and Low Power Mode
 *  Application Note):
 *    PAIR080 / PAIR081  Set / get navigation mode
 *    PAIR650            Enter Backup mode
 *    PAIR680 / PAIR681  GPS Low Power (GLP) mode
 *    PAIR690 / PAIR691  Periodic power saving mode
 *    PAIR730 / PAIR731  Fitness Low Power (FLP) mode
 *    PAIR732 / PAIR733  Adaptive Low Power (ALP) mode
 * ============================================================================
 */

#include "LC76G.h"
#include <stdlib.h>   // atol()

// Limits from the protocol specification
static const uint32_t PERIODIC_MIN_S = 3;
static const uint32_t PERIODIC_MAX_S = 518400UL;     // 6 days
static const uint32_t BACKUP_MIN_S   = 10;
static const uint32_t BACKUP_MAX_S   = 62208000UL;   // 2 years


// ===========================================================================
//  NAVIGATION MODE
// ===========================================================================

LC76G_Result LC76G::setNavigationMode(LC76G_NavMode mode) {
  char p[4];
  snprintf(p, sizeof(p), "%u", (unsigned)mode);
  return sendPair(80, p);
}

LC76G_Result LC76G::getNavigationMode(LC76G_NavMode &mode) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(81, 1, v);     // $PAIR081,<NavMode>
  if (r == LC76G_OK) {
    mode = (LC76G_NavMode)v;
  }
  return r;
}


// ===========================================================================
//  ALP / GLP / FLP
// ===========================================================================

LC76G_Result LC76G::setAlpMode(uint8_t alpMode) {
  if (alpMode > 2) {
    return LC76G_PARAM_ERROR;
  }
  char p[4];
  snprintf(p, sizeof(p), "%u", (unsigned)alpMode);
  return sendPair(732, p);
}

LC76G_Result LC76G::getAlpMode(uint8_t &alpMode) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(733, 1, v);    // $PAIR733,<Enabled>
  if (r == LC76G_OK) {
    alpMode = (uint8_t)v;
  }
  return r;
}

LC76G_Result LC76G::setGlpMode(bool enable) {
  return sendPair(680, enable ? "1" : "0");
}

LC76G_Result LC76G::getGlpMode(bool &enabled) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(681, 1, v);    // $PAIR681,<Enabled>
  if (r == LC76G_OK) {
    enabled = (v == 1);
  }
  return r;
}

LC76G_Result LC76G::setFlpMode(bool enable) {
  return sendPair(730, enable ? "1" : "0");
}

LC76G_Result LC76G::getFlpMode(bool &enabled) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(731, 1, v);    // $PAIR731,<Enabled>
  if (r == LC76G_OK) {
    enabled = (v == 1);
  }
  return r;
}


// ===========================================================================
//  PERIODIC MODE
// ===========================================================================

// true if 't' is a valid time; 'zeroAllowed' for the "no signal" pair
static bool validPeriodicTime(uint32_t t, bool zeroAllowed) {
  if (t == 0) {
    return zeroAllowed;
  }
  return t >= PERIODIC_MIN_S && t <= PERIODIC_MAX_S;
}

LC76G_Result LC76G::setPeriodicMode(const LC76G_PeriodicConfig &cfg) {
  char p[48];
  if (cfg.mode == LC76G_PERIODIC_OFF) {
    // Times are still required by the command: use the minimum valid values
    snprintf(p, sizeof(p), "0,%lu,%lu,0,0",
             (unsigned long)PERIODIC_MIN_S, (unsigned long)PERIODIC_MIN_S);
  } else {
    // The "no signal" times must be both zero or both valid (notes 3 and 4)
    bool noSignalPairOk = (cfg.runNoSignalS == 0) == (cfg.sleepNoSignalS == 0);
    if (cfg.mode > LC76G_PERIODIC_STRICT ||
        !validPeriodicTime(cfg.runS, false) || !validPeriodicTime(cfg.sleepS, false) ||
        !validPeriodicTime(cfg.runNoSignalS, true) || !validPeriodicTime(cfg.sleepNoSignalS, true) ||
        !noSignalPairOk) {
      return LC76G_PARAM_ERROR;
    }
    snprintf(p, sizeof(p), "%u,%lu,%lu,%lu,%lu", (unsigned)cfg.mode,
             (unsigned long)cfg.runS, (unsigned long)cfg.sleepS,
             (unsigned long)cfg.runNoSignalS, (unsigned long)cfg.sleepNoSignalS);
  }
  return sendPair(690, p);
}

LC76G_Result LC76G::getPeriodicMode(LC76G_PeriodicConfig &cfg) {
  // $PAIR691,<Mode>,<FirstRun>,<FirstSleep>,<SecondRun>,<SecondSleep>
  char resp[64];
  LC76G_Result r = sendPair(691, nullptr, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[12];
    LC76G_Nmea::getField(resp, 1, f, sizeof(f)); cfg.mode           = (LC76G_PeriodicMode)atol(f);
    LC76G_Nmea::getField(resp, 2, f, sizeof(f)); cfg.runS           = (uint32_t)atol(f);
    LC76G_Nmea::getField(resp, 3, f, sizeof(f)); cfg.sleepS         = (uint32_t)atol(f);
    LC76G_Nmea::getField(resp, 4, f, sizeof(f)); cfg.runNoSignalS   = (uint32_t)atol(f);
    LC76G_Nmea::getField(resp, 5, f, sizeof(f)); cfg.sleepNoSignalS = (uint32_t)atol(f);
  }
  return r;
}

LC76G_Result LC76G::disablePeriodicMode(uint32_t timeoutMs) {
  LC76G_PeriodicConfig off = { LC76G_PERIODIC_OFF, 0, 0, 0, 0 };
  unsigned long t0 = millis();
  LC76G_Result r = LC76G_TIMEOUT;
  // While the module sleeps the command is lost: repeat it every second
  // (each attempt waits up to 1 s for the acknowledgement)
  while (millis() - t0 < timeoutMs) {
    r = setPeriodicMode(off);
    if (r == LC76G_OK) {
      break;
    }
  }
  return r;
}


// ===========================================================================
//  BACKUP MODE
// ===========================================================================

LC76G_Result LC76G::enterBackup(uint32_t seconds) {
  if (seconds != 0 && (seconds < BACKUP_MIN_S || seconds > BACKUP_MAX_S)) {
    return LC76G_PARAM_ERROR;
  }
  pauseLocus();                                // LOCUS data would be lost otherwise
  char p[12];
  snprintf(p, sizeof(p), "%lu", (unsigned long)seconds);
  LC76G_Result r = sendPair(650, p);

  // Without timer, the datasheet procedure is: send PAIR650, then cut VCC
  // keeping V_BCKP powered. Done automatically if a power pin is set.
  if (r == LC76G_OK && seconds == 0 && _pwrPin >= 0) {
    delay(100);                                // Let the module complete the entry
    powerOff();
  }
  return r;
}

LC76G_Result LC76G::exitBackup() {
  if (_pwrPin < 0 && _rstPin < 0) {
    return LC76G_NO_REBOOT;                    // Cannot wake the module from software
  }
  powerOn();                                   // Restore VCC (no-op without power pin)
  delay(100);
  resetPulse();                                // RESET_N low for 100 ms (Application Note)
  delay(LC76G_BOOT_MS);

  // Wait for the first valid sentence
  _inLine = false;
  unsigned long before = _lastSentenceMs;
  unsigned long t0 = millis();
  while (millis() - t0 < LC76G_DETECT_MS) {
    update();
    if (_lastSentenceMs != before) {
      return LC76G_OK;
    }
  }
  return LC76G_TIMEOUT;
}
