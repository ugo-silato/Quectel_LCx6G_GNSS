/*
 * LC76G_Settings.cpp - Output settings: minimum SNR, NMEA decimal places,
 *                      NMEA Talker ID, RTCM output, Galileo RLM output,
 *                      1PPS / NMEA synchronisation
 *
 * Commands (LC26G/LC76G/LC86G protocol specification v1.5):
 *   $PAIR058,<MIN_SNR>                 / $PAIR059 -> $PAIR059,<MIN_SNR>
 *   $PQTMCFGNMEADP,W,<UTC>,<POS>,<ALT>,<DOP>,<SPD>,<COG>  / ,R
 *   $PQTMCFGNMEATID,W,<Main_TalkerID>,<GSV_TalkerID>      / ,R
 *   $PAIR432,<Mode>  / $PAIR433   RTCM output mode
 *   $PAIR434,<0|1>   / $PAIR435   RTCM antenna reference point (1005)
 *   $PAIR436,<0|1>   / $PAIR437   RTCM ephemeris
 *   $PAIR154,<0|1>   / $PAIR155   Galileo Return Link Message
 *   $PAIR751,<0|1>                1PPS synchronised with NMEA
 *
 * Author: Ugo Silato
 * License: MIT (see LICENSE file)
 */

#include "LC76G.h"
#include <stdio.h>    // snprintf()
#include <string.h>   // strlen(), strncpy()
#include <stdlib.h>   // atoi()


// ===========================================================================
//  MINIMUM SNR
// ===========================================================================

LC76G_Result LC76G::setMinSnr(uint8_t dbHz) {
  if (dbHz < 9 || dbHz > 37) {
    return LC76G_PARAM_ERROR;                  // Range of the specification
  }
  char params[4];
  snprintf(params, sizeof(params), "%u", (unsigned)dbHz);
  return sendPair(58, params);
}

LC76G_Result LC76G::getMinSnr(uint8_t &dbHz) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(59, 1, v);     // $PAIR059,<MIN_SNR>
  if (r == LC76G_OK) {
    dbHz = (uint8_t)v;
  }
  return r;
}


// ===========================================================================
//  NMEA DECIMAL PLACES
// ===========================================================================

LC76G_Result LC76G::setNmeaDecimals(const LC76G_NmeaDecimals &d) {
  // Ranges of the specification; refuse before sending anything
  if (d.utc > 3 || d.position > 8 || d.altitude > 3 ||
      d.dop > 3 || d.speed > 3 || d.course > 3) {
    return LC76G_PARAM_ERROR;
  }
  char params[24];
  snprintf(params, sizeof(params), "W,%u,%u,%u,%u,%u,%u",
           (unsigned)d.utc, (unsigned)d.position, (unsigned)d.altitude,
           (unsigned)d.dop, (unsigned)d.speed, (unsigned)d.course);
  return sendPqtm("PQTMCFGNMEADP", params);
}

LC76G_Result LC76G::getNmeaDecimals(LC76G_NmeaDecimals &d) {
  // Reply: PQTMCFGNMEADP,OK,<UTC>,<POS>,<ALT>,<DOP>,<SPD>,<COG>
  //  field:      0       1   2     3     4     5     6     7
  char resp[48];
  LC76G_Result r = sendPqtm("PQTMCFGNMEADP", "R", LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r != LC76G_OK) {
    return r;
  }
  uint8_t *dst[6] = { &d.utc, &d.position, &d.altitude, &d.dop, &d.speed, &d.course };
  char f[4];
  for (uint8_t i = 0; i < 6; i++) {
    LC76G_Nmea::getField(resp, 2 + i, f, sizeof(f));
    *dst[i] = (uint8_t)atoi(f);
  }
  return r;
}


// ===========================================================================
//  NMEA TALKER ID
// ===========================================================================

LC76G_Result LC76G::setNmeaTalkerId(const char *mainId, bool gsvSameAsMain) {
  // Exactly two characters, printable ASCII 0x20..0x7E. The specification
  // excludes '*'; ',' and '$' are refused too, they would break NMEA.
  // A first character 'P' is refused as well: sentences starting with 'P'
  // are proprietary for NMEA, so the decoder would ignore RMC/GGA/GSV.
  if (mainId == nullptr || strlen(mainId) != 2 || mainId[0] == 'P') {
    return LC76G_PARAM_ERROR;
  }
  for (uint8_t i = 0; i < 2; i++) {
    char c = mainId[i];
    if (c < 0x20 || c > 0x7E || c == '*' || c == ',' || c == '$') {
      return LC76G_PARAM_ERROR;
    }
  }
  char params[12];
  snprintf(params, sizeof(params), "W,%s,%u", mainId, gsvSameAsMain ? 1u : 0u);
  return sendPqtm("PQTMCFGNMEATID", params);
}

LC76G_Result LC76G::getNmeaTalkerId(char *mainId, bool &gsvSameAsMain) {
  // Reply: PQTMCFGNMEATID,OK,<Main_TalkerID>,<GSV_TalkerID>
  //  field:      0        1        2              3
  char resp[40];
  LC76G_Result r = sendPqtm("PQTMCFGNMEATID", "R", LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r != LC76G_OK) {
    return r;
  }
  LC76G_Nmea::getField(resp, 2, mainId, 3);    // Two characters + terminator
  char f[4];
  LC76G_Nmea::getField(resp, 3, f, sizeof(f));
  gsvSameAsMain = (atoi(f) == 1);
  return r;
}


// ===========================================================================
//  RTCM OUTPUT
// ===========================================================================

LC76G_Result LC76G::setRtcmMode(LC76G_RtcmMode mode) {
  char params[4];
  snprintf(params, sizeof(params), "%d", (int)mode);   // "-1", "0" or "1"
  return sendPair(432, params);
}

LC76G_Result LC76G::getRtcmMode(LC76G_RtcmMode &mode) {
  int32_t v = 0;
  LC76G_Result r = queryPairInt(433, 1, v);    // $PAIR433,<Mode>
  if (r == LC76G_OK) {
    mode = (LC76G_RtcmMode)v;
  }
  return r;
}

// On/off settings share the "set id / query id = set id + 1" pattern
LC76G_Result LC76G::setRtcmAntennaPoint(bool e)  { return setPairFlag(434, e); }
LC76G_Result LC76G::getRtcmAntennaPoint(bool &e) { return getPairFlag(435, e); }
LC76G_Result LC76G::setRtcmEphemeris(bool e)     { return setPairFlag(436, e); }
LC76G_Result LC76G::getRtcmEphemeris(bool &e)    { return getPairFlag(437, e); }


// ===========================================================================
//  GALILEO RETURN LINK MESSAGE, 1PPS / NMEA SYNCHRONISATION
// ===========================================================================

LC76G_Result LC76G::setRlmOutput(bool e)   { return setPairFlag(154, e); }
LC76G_Result LC76G::getRlmOutput(bool &e)  { return getPairFlag(155, e); }
LC76G_Result LC76G::setPpsNmeaSync(bool e) { return setPairFlag(751, e); }
