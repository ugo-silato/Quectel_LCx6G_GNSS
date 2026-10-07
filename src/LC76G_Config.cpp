/*
 * ============================================================================
 *  LC76G_Config.cpp  -  NMEA output configuration for the LC76G
 * ============================================================================
 *  Part of the Quectel_LCx6G_GNSS Arduino library. Author: Ugo Silato. License: MIT.
 *
 *  Kept in a separate file so that sketches which do not call these
 *  functions do not pay for them in flash (unused code is removed by the
 *  linker).
 *
 *  Commands used (Quectel GNSS Protocol Specification):
 *    PAIR062  Set output rate of a standard NMEA sentence
 *    PAIR063  Get output rate of a standard NMEA sentence
 *    PAIR513  Save current configuration from RTC RAM to flash (NVM)
 * ============================================================================
 */

#include "LC76G.h"
#include <stdlib.h>   // atoi()

// Maximum output rate accepted by the module (once every 20 fixes)
static const uint8_t NMEA_RATE_MAX = 20;


LC76G_Result LC76G::setNmeaRate(LC76G_NmeaType type, uint8_t rate) {
  if (type >= LC76G_NMEA_TYPE_COUNT || rate > NMEA_RATE_MAX) {
    return LC76G_PARAM_ERROR;                  // Refuse locally, do not bother the module
  }
  char params[8];                              // "<type>,<rate>", e.g. "3,5"
  snprintf(params, sizeof(params), "%u,%u", (unsigned)type, (unsigned)rate);
  return sendPair(62, params);
}

LC76G_Result LC76G::getNmeaRate(LC76G_NmeaType type, uint8_t &rate) {
  if (type >= LC76G_NMEA_TYPE_COUNT) {
    return LC76G_PARAM_ERROR;
  }
  char params[4];
  snprintf(params, sizeof(params), "%u", (unsigned)type);

  // Query result format: $PAIR063,<Type>,<OutputRate>*CS
  char resp[24];
  LC76G_Result r = sendPair(63, params, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[6];
    LC76G_Nmea::getField(resp, 2, f, sizeof(f));
    rate = (uint8_t)atoi(f);
  }
  return r;
}

LC76G_Result LC76G::setMinimalNmeaOutput(uint8_t gsvRate) {
  // Each entry: sentence type and the rate to apply
  struct Entry {
    LC76G_NmeaType type;
    uint8_t        rate;
  };
  const Entry table[] = {
    { LC76G_NMEA_GGA, 1 },        // Needed: satellites used, altitude, HDOP
    { LC76G_NMEA_RMC, 1 },        // Needed: time, date, position, end of epoch
    { LC76G_NMEA_GSV, gsvRate },  // Optional: satellites in view
    { LC76G_NMEA_GLL, 0 },        // Duplicate of RMC
    { LC76G_NMEA_GSA, 0 },        // Not decoded
    { LC76G_NMEA_VTG, 0 },        // Duplicate of RMC
  };

  // Apply every entry; stop at the first error and report it
  for (uint8_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
    LC76G_Result r = setNmeaRate(table[i].type, table[i].rate);
    if (r != LC76G_OK) {
      return r;
    }
  }
  return LC76G_OK;
}

LC76G_Result LC76G::saveSettings() {
  // Writing flash takes longer than a normal command: allow 2 s
  return sendPair(513, nullptr, 2000);
}


// ===========================================================================
//  CONSTELLATION SELECTION
// ===========================================================================

// Combinations supported by the LC76G series (Protocol Specification,
// PAIR066 note 1). Bit order: QZSS BDS Galileo GLONASS GPS.
static const uint8_t SUPPORTED_SETS[] = {
  LC76G_GNSS_GPS,                                                           // GPS only
  LC76G_GNSS_BDS,                                                           // BDS only
  LC76G_GNSS_GPS | LC76G_GNSS_QZSS,
  LC76G_GNSS_GPS | LC76G_GNSS_GLONASS,
  LC76G_GNSS_GPS | LC76G_GNSS_GLONASS | LC76G_GNSS_QZSS,
  LC76G_GNSS_GPS | LC76G_GNSS_GALILEO,
  LC76G_GNSS_GPS | LC76G_GNSS_GALILEO | LC76G_GNSS_QZSS,
  LC76G_GNSS_GPS | LC76G_GNSS_BDS,
  LC76G_GNSS_GPS | LC76G_GNSS_BDS | LC76G_GNSS_QZSS,
  LC76G_GNSS_GPS | LC76G_GNSS_GALILEO | LC76G_GNSS_BDS,
  LC76G_GNSS_GPS | LC76G_GNSS_GALILEO | LC76G_GNSS_BDS | LC76G_GNSS_QZSS,
  LC76G_GNSS_GPS | LC76G_GNSS_GLONASS | LC76G_GNSS_GALILEO | LC76G_GNSS_BDS,
  LC76G_GNSS_ALL                                                            // All five
};

bool LC76G::isSupportedConstellationSet(uint8_t mask) {
  for (uint8_t i = 0; i < sizeof(SUPPORTED_SETS); i++) {
    if (SUPPORTED_SETS[i] == mask) {
      return true;
    }
  }
  return false;
}

LC76G_Result LC76G::setConstellations(uint8_t mask) {
  if (!isSupportedConstellationSet(mask)) {
    return LC76G_PARAM_ERROR;                  // Refuse locally
  }

  // Skip the command if nothing changes: avoids an unnecessary reboot check
  uint8_t current = 0;
  if (getConstellations(current) == LC76G_OK && current == mask) {
    return LC76G_OK;
  }

  // $PAIR066,<GPS>,<GLONASS>,<Galileo>,<BDS>,<QZSS>,<Reserved=0>
  char params[16];
  snprintf(params, sizeof(params), "%u,%u,%u,%u,%u,0",
           (mask & LC76G_GNSS_GPS)     ? 1u : 0u,
           (mask & LC76G_GNSS_GLONASS) ? 1u : 0u,
           (mask & LC76G_GNSS_GALILEO) ? 1u : 0u,
           (mask & LC76G_GNSS_BDS)     ? 1u : 0u,
           (mask & LC76G_GNSS_QZSS)    ? 1u : 0u);
  bool resumeLocus = pauseLocus();             // The module may reboot: protect LOCUS data
  LC76G_Result r = sendPair(66, params);

  if (r == LC76G_OK) {
    // The module reboots by itself when the set changes: wait for it to
    // come back (any valid sentence), so the next command is not lost.
    delay(LC76G_BOOT_MS);
    unsigned long t0 = millis();
    unsigned long before = _lastSentenceMs;
    while (millis() - t0 < LC76G_DETECT_MS && _lastSentenceMs == before) {
      update();
    }
  }
  if (resumeLocus && sendPair(900, "1") == LC76G_OK) {
    _locusActive = true;
  }
  return r;
}

LC76G_Result LC76G::getConstellations(uint8_t &mask) {
  // Query result: $PAIR067,<GPS>,<GLONASS>,<Galileo>,<BDS>,<QZSS>,<Res>
  char resp[32];
  LC76G_Result r = sendPair(67, nullptr, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    mask = 0;
    char f[4];
    for (uint8_t i = 0; i < 5; i++) {          // Fields 1..5 -> bits 0..4
      LC76G_Nmea::getField(resp, i + 1, f, sizeof(f));
      if (f[0] == '1') {
        mask |= (uint8_t)(1u << i);
      }
    }
  }
  return r;
}
