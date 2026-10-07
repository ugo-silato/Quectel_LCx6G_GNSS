/*
 * ============================================================================
 *  LC76G_Reception.cpp  -  Reception settings of the LC76G
 * ============================================================================
 *  Part of the Quectel_LCx6G_GNSS Arduino library. Author: Ugo Silato. License: MIT.
 *
 *  Commands used (Quectel GNSS Protocol Specification):
 *    PAIR070 / PAIR071  Static navigation threshold
 *    PAIR072 / PAIR073  Elevation mask
 *    PAIR074 / PAIR075  AIC (Active Interference Cancellation)
 *    PAIR158 / PAIR159  BDS B1C band tracking
 *    PAIR160 / PAIR161  Real-time speed response
 *    PAIR162 / PAIR163  2D fix
 *    PAIR400 / PAIR401  DGPS correction source
 *    PAIR410 / PAIR411  SBAS satellite search
 * ============================================================================
 */

#include "LC76G.h"


// ===========================================================================
//  SHARED HELPERS FOR ON/OFF SETTINGS
// ===========================================================================

LC76G_Result LC76G::setPairFlag(uint16_t id, bool enable) {
  return sendPair(id, enable ? "1" : "0");
}

LC76G_Result LC76G::getPairFlag(uint16_t id, bool &enabled) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(id, 1, v);     // $PAIRnnn,<0|1>
  if (r == LC76G_OK) {
    enabled = (v == 1);
  }
  return r;
}

// One-line wrappers: set command id, query command id = set id + 1
LC76G_Result LC76G::setAic(bool e)            { return setPairFlag(74, e); }
LC76G_Result LC76G::getAic(bool &e)           { return getPairFlag(75, e); }
LC76G_Result LC76G::setSbas(bool e)           { return setPairFlag(410, e); }
LC76G_Result LC76G::getSbas(bool &e)          { return getPairFlag(411, e); }
LC76G_Result LC76G::set2dFix(bool e)          { return setPairFlag(162, e); }
LC76G_Result LC76G::get2dFix(bool &e)         { return getPairFlag(163, e); }
LC76G_Result LC76G::setImmediateSpeed(bool e) { return setPairFlag(160, e); }
LC76G_Result LC76G::getImmediateSpeed(bool &e){ return getPairFlag(161, e); }
LC76G_Result LC76G::setBdsB1c(bool e)         { return setPairFlag(158, e); }
LC76G_Result LC76G::getBdsB1c(bool &e)        { return getPairFlag(159, e); }


// ===========================================================================
//  ELEVATION MASK
// ===========================================================================

LC76G_Result LC76G::setElevationMask(int8_t degrees) {
  if (degrees < -90 || degrees > 90) {
    return LC76G_PARAM_ERROR;
  }
  char p[6];
  snprintf(p, sizeof(p), "%d", (int)degrees);
  return sendPair(72, p);
}

LC76G_Result LC76G::getElevationMask(int8_t &degrees) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(73, 1, v);     // $PAIR073,<Degree>
  if (r == LC76G_OK) {
    degrees = (int8_t)v;
  }
  return r;
}


// ===========================================================================
//  DGPS SOURCE
// ===========================================================================

LC76G_Result LC76G::setDgpsSource(LC76G_DgpsSource source) {
  if (source != LC76G_DGPS_NONE && source != LC76G_DGPS_SBAS && source != LC76G_DGPS_SLAS) {
    return LC76G_PARAM_ERROR;
  }
  char p[4];
  snprintf(p, sizeof(p), "%u", (unsigned)source);
  return sendPair(400, p);
}

LC76G_Result LC76G::getDgpsSource(LC76G_DgpsSource &source) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(401, 1, v);    // $PAIR401,<Mode>
  if (r == LC76G_OK) {
    source = (LC76G_DgpsSource)v;
  }
  return r;
}


// ===========================================================================
//  STATIC NAVIGATION THRESHOLD
// ===========================================================================

LC76G_Result LC76G::setStaticThreshold(uint8_t dmPerS) {
  if (dmPerS > 20) {
    return LC76G_PARAM_ERROR;
  }
  char p[4];
  snprintf(p, sizeof(p), "%u", (unsigned)dmPerS);  // Sent in dm/s (protocol example: 4)
  return sendPair(70, p);
}

LC76G_Result LC76G::getStaticThreshold(uint8_t &dmPerS) {
  // The query answers in m/s with one decimal (protocol example: 0.4)
  char resp[24];
  LC76G_Result r = sendPair(71, nullptr, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[8];
    LC76G_Nmea::getField(resp, 1, f, sizeof(f));
    dmPerS = (uint8_t)LC76G_Nmea::parseFixed(f, 1);   // "0.4" m/s -> 4 dm/s
  }
  return r;
}
