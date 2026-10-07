/*
 * ============================================================================
 *  LC76G_Agnss.cpp  -  AGNSS (Assisted GNSS) for the LC76G
 * ============================================================================
 *  Part of the Quectel_LCx6G_GNSS Arduino library. Author: Ugo Silato. License: MIT.
 *
 *  Commands used (Quectel AGNSS Application Note, Protocol Specification):
 *    PQTMVERNO          Firmware version
 *    PAIR490 / PAIR491  EASY enable / status
 *    PAIR496            EPOC enable
 *    PAIR498 / PAIR508  EPOC constellations / status
 *    PAIR507            Clear EPOC data
 *    PAIR509            EPOC prediction status
 *    PAIR511 / PAIR512  Save / clear navigation data
 *    PAIR590            Reference UTC time
 *    PAIR600            Reference position
 * ============================================================================
 */

#include "LC76G.h"
#include <stdlib.h>   // atol()


// ===========================================================================
//  FIRMWARE VERSION
// ===========================================================================

LC76G_Result LC76G::getFirmwareVersion(char *buf, uint8_t size) {
  // Reply: $PQTMVERNO,<VerStr>,<BuildDate>,<BuildTime>  (no "OK" field)
  char resp[64];
  LC76G_Result r = sendPqtm("PQTMVERNO", nullptr, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    LC76G_Nmea::getField(resp, 1, buf, size);
  } else if (size > 0) {
    buf[0] = '\0';
  }
  return r;
}


// ===========================================================================
//  EASY
// ===========================================================================

LC76G_Result LC76G::setEasy(bool enable) {
  return sendPair(490, enable ? "1" : "0");
}

LC76G_Result LC76G::getEasy(bool &enabled, uint8_t &extensionDays) {
  // Reply: $PAIR491,<Enabled>[,<Status>]
  char resp[32];
  LC76G_Result r = sendPair(491, nullptr, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[4];
    LC76G_Nmea::getField(resp, 1, f, sizeof(f));
    enabled = (f[0] == '1');
    extensionDays = 0;                         // Field missing when EASY is off
    if (LC76G_Nmea::getField(resp, 2, f, sizeof(f)) && f[0] != '\0') {
      extensionDays = (uint8_t)atol(f);
    }
  }
  return r;
}


// ===========================================================================
//  EPOC
// ===========================================================================

LC76G_Result LC76G::setEpoc(bool enable) {
  return sendPair(496, enable ? "1" : "0");
}

LC76G_Result LC76G::setEpocConstellations(uint8_t mask) {
  // Only GPS, GPS + Galileo, GPS + BDS are available (Application Note)
  if (mask != LC76G_EPOC_GPS &&
      mask != (LC76G_EPOC_GPS | LC76G_EPOC_GALILEO) &&
      mask != (LC76G_EPOC_GPS | LC76G_EPOC_BDS)) {
    return LC76G_PARAM_ERROR;
  }
  char p[4];
  snprintf(p, sizeof(p), "%u", (unsigned)mask);  // Decimal value of the bit mask
  return sendPair(498, p);
}

LC76G_Result LC76G::getEpoc(bool &enabled, uint8_t &mask) {
  // Reply: $PAIR508,<Enabled>,<Constellation>
  char resp[32];
  LC76G_Result r = sendPair(508, nullptr, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[6];
    LC76G_Nmea::getField(resp, 1, f, sizeof(f));
    enabled = (f[0] == '1');
    LC76G_Nmea::getField(resp, 2, f, sizeof(f));
    mask = (uint8_t)strtol(f, nullptr, 16);    // Hexadecimal in the reply
  }
  return r;
}

LC76G_Result LC76G::clearEpocData() {
  return sendPair(507, nullptr, 2000);         // Flash erase: allow more time
}

LC76G_Result LC76G::getEpocPredictionStatus(uint8_t constellationBit, int32_t &status,
                                            uint8_t &satsReady) {
  // Only one constellation per query
  if (constellationBit != LC76G_EPOC_GPS && constellationBit != LC76G_EPOC_GALILEO &&
      constellationBit != LC76G_EPOC_BDS) {
    return LC76G_PARAM_ERROR;
  }
  char p[4];
  snprintf(p, sizeof(p), "%X", (unsigned)constellationBit);  // Hexadecimal parameter

  // Reply: $PAIR509,<Status>,<Constellation>,<SatelliteStatus>
  //   <SatelliteStatus> = 16 hex digits, one bit per satellite whose
  //   prediction is complete (e.g. 00000000000D2D2C -> 10 satellites)
  char resp[64];
  LC76G_Result r = sendPair(509, p, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[20];
    LC76G_Nmea::getField(resp, 1, f, sizeof(f));
    status = atol(f);

    // Count the '1' bits hex digit by hex digit: avoids 64-bit maths on AVR
    LC76G_Nmea::getField(resp, 3, f, sizeof(f));
    satsReady = 0;
    for (const char *c = f; *c != '\0'; c++) {
      uint8_t v;
      if (*c >= '0' && *c <= '9')      v = *c - '0';
      else if (*c >= 'A' && *c <= 'F') v = *c - 'A' + 10;
      else if (*c >= 'a' && *c <= 'f') v = *c - 'a' + 10;
      else                             continue;
      for (; v != 0; v >>= 1) {
        satsReady += (v & 1);
      }
    }
  }
  return r;
}


// ===========================================================================
//  NAVIGATION DATA
// ===========================================================================

LC76G_Result LC76G::saveNavigationData() {
  return sendPair(511, nullptr, 2000);         // Flash write: allow more time
}

LC76G_Result LC76G::clearNavigationData() {
  return sendPair(512, nullptr, 2000);
}


// ===========================================================================
//  HOST ASSISTANCE
// ===========================================================================

LC76G_Result LC76G::setReferenceTime(uint16_t year, uint8_t month, uint8_t day,
                                     uint8_t hour, uint8_t minute, uint8_t second) {
  if (year < 1980 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hour > 23 || minute > 59 || second > 59) {
    return LC76G_PARAM_ERROR;
  }
  // $PAIR590,<YYYY>,<MM>,<DD>,<hh>,<mm>,<ss>
  char p[24];
  snprintf(p, sizeof(p), "%u,%u,%u,%u,%u,%u", (unsigned)year, (unsigned)month,
           (unsigned)day, (unsigned)hour, (unsigned)minute, (unsigned)second);
  return sendPair(590, p);
}

LC76G_Result LC76G::setReferencePosition(int32_t latE7, int32_t lonE7, int32_t heightM,
                                         uint32_t accMajorM, uint32_t accMinorM,
                                         uint16_t bearingDeg, uint32_t accVertM) {
  if (latE7 < -900000000L || latE7 > 900000000L ||
      lonE7 < -1800000000L || lonE7 > 1800000000L || bearingDeg > 359) {
    return LC76G_PARAM_ERROR;
  }
  // $PAIR600,<Lat>,<Lon>,<Height>,<AccMaj>,<AccMin>,<Bear>,<AccVert>
  char lat[14];
  char lon[14];
  LC76G_Nmea::formatE7(lat, sizeof(lat), latE7);
  LC76G_Nmea::formatE7(lon, sizeof(lon), lonE7);
  char p[80];
  snprintf(p, sizeof(p), "%s,%s,%ld,%lu,%lu,%u,%lu", lat, lon, (long)heightM,
           (unsigned long)accMajorM, (unsigned long)accMinorM, (unsigned)bearingDeg,
           (unsigned long)accVertM);
  return sendPair(600, p);
}
