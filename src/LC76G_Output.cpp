/*
 * LC76G_Output.cpp - PQTM output messages: $PQTMPVT, $PQTMVEL, $PQTMDOP,
 *                    $PQTMODO, $PQTMTIMEGPS, $PQTMPL, $PQTMPOSECEF,
 *                    $PQTMVELECEF, $PQTMPVTECEF, and the odometer commands
 *
 * How it works:
 *   - set...Output(rate) asks the module to send the message
 *     (PQTMCFGMSGRATE, message version 1).
 *   - on...(callback) registers a small decoder in the library's hook
 *     table. When the message arrives, the decoder fills a temporary
 *     structure on the stack and calls the sketch's callback.
 *   Each decoder is a separate function referenced only by its on...()
 *   method: if a sketch never calls onPvt(), the PVT decoder is removed
 *   by the linker and costs no flash.
 *
 * Message formats (LC26G/LC76G/LC86G protocol specification v1.5):
 *   $PQTMPVT,1,<TOW>,<Date>,<Time>,<Quality>,<FixMode>,<NumSatUsed>,
 *            <LeapS>,<Lat>,<Lon>,<Alt>,<Sep>,<VelN>,<VelE>,<VelD>,
 *            <Spd>,<Heading>,<HDOP>,<PDOP>
 *   $PQTMVEL,1,<Time>,<VelN>,<VelE>,<VelD>,<GrdSpd>,<Spd>,<Heading>,
 *            <GrdSpdAcc>,<SpdAcc>,<HeadingAcc>
 *   $PQTMDOP,1,<TOW>,<GDOP>,<PDOP>,<TDOP>,<VDOP>,<HDOP>,<NDOP>,<EDOP>
 *   $PQTMODO,1,<Time>,<State>,<Dist>
 *   $PQTMTIMEGPS,1,<TOW>,<FTOW>,<WN>,<Date>,<Time>,<LeapS>,<Time_Acc>
 *   $PQTMPL,1,<TOW>,<PUL>,<Res1>,<Res2>,<PL_PosN>,<PL_PosE>,<PL_PosD>,
 *           <PL_VelN>,<PL_VelE>,<PL_VelD>,<Res3>,<Res4>,<PL_Time>
 *   $PQTMPOSECEF,1,<Time>,<PosX>,<PosY>,<PosZ>,<PosAcc>
 *   $PQTMVELECEF,1,<Time>,<VelX>,<VelY>,<VelZ>,<VelAcc>
 *   $PQTMPVTECEF,1,<TOW>,<Date>,<Time>,<LeapS>,<PosX>,<PosY>,<PosZ>,
 *                <PosAcc>,<VelX>,<VelY>,<VelZ>,<VelAcc>
 *
 * Author: Ugo Silato
 * License: MIT (see LICENSE file)
 */

#include "LC76G.h"
#include <string.h>   // strncmp(), memset()
#include <stdlib.h>   // atoi(), atol()
#include <stdio.h>    // snprintf()


// ===========================================================================
//  SMALL PARSING HELPERS
// ===========================================================================

// Reads field 'index' of 'body' as a fixed-point number with 'decimals'
// decimals (e.g. "1.234" with 3 decimals -> 1234). 'present' tells
// whether the field was non-empty.
static int32_t fieldFixed(const char *body, uint8_t index, uint8_t decimals, bool *present = nullptr) {
  char f[20];
  LC76G_Nmea::getField(body, index, f, sizeof(f));
  if (present != nullptr) {
    *present = (f[0] != '\0');
  }
  return LC76G_Nmea::parseFixed(f, decimals);
}

// Same, limited to 0..65535 (DOP values, headings, accuracies)
static uint16_t fieldU16(const char *body, uint8_t index, uint8_t decimals) {
  int32_t v = fieldFixed(body, index, decimals);
  if (v < 0)      return 0;
  if (v > 65535L) return 65535U;
  return (uint16_t)v;
}

// Parses a UTC time "hhmmss.sss" (field 'index') into its parts
static void fieldTime(const char *body, uint8_t index,
                      uint8_t &h, uint8_t &m, uint8_t &s, uint16_t &ms) {
  char f[12];
  LC76G_Nmea::getField(body, index, f, sizeof(f));
  if (strlen(f) < 6) {                         // Empty or malformed
    h = m = s = 0;
    ms = 0;
    return;
  }
  h  = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
  m  = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
  s  = (uint8_t)((f[4] - '0') * 10 + (f[5] - '0'));
  ms = (f[6] == '.') ? (uint16_t)LC76G_Nmea::parseFixed(f + 6, 3) : 0;   // ".sss"
}


// Parses a UTC date "YYYYMMDD" (field 'index'); 0 when empty
static void fieldDate(const char *body, uint8_t index,
                      uint16_t &year, uint8_t &month, uint8_t &day) {
  char f[12];
  LC76G_Nmea::getField(body, index, f, sizeof(f));
  if (strlen(f) != 8) {
    year = 0; month = 0; day = 0;
    return;
  }
  long ymd = atol(f);                          // YYYYMMDD as one number (32 bit)
  year  = (uint16_t)(ymd / 10000);
  month = (uint8_t)((ymd / 100) % 100);
  day   = (uint8_t)(ymd % 100);
}

// Reads a non-negative value (negative or empty -> 0)
static uint32_t fieldU32(const char *body, uint8_t index, uint8_t decimals) {
  int32_t v = fieldFixed(body, index, decimals);
  return (v > 0) ? (uint32_t)v : 0;
}

// Reads the leap seconds field (LC76G_LEAP_UNKNOWN when empty)
static int8_t fieldLeap(const char *body, uint8_t index) {
  bool present = false;
  int32_t v = fieldFixed(body, index, 0, &present);
  return present ? (int8_t)v : (int8_t)LC76G_LEAP_UNKNOWN;
}

// ===========================================================================
//  DECODERS (one per message, called from LC76G::handleSentence)
// ===========================================================================

// $PQTMPVT,1,...
static bool decodePvt(const char *body, LC76G::GenericFn cb) {
  if (strncmp(body, "PQTMPVT,", 8) != 0) {
    return false;                              // Not this message
  }
  LC76G_Pvt p;
  memset(&p, 0, sizeof(p));

  p.towMs = fieldU32(body, 2, 0);              // Time of week, ms
  fieldDate(body, 3, p.year, p.month, p.day);
  fieldTime(body, 4, p.hour, p.minute, p.second, p.millisecond);

  p.quality     = (uint8_t)fieldFixed(body, 5, 0);
  p.fixMode     = (uint8_t)fieldFixed(body, 6, 0);
  p.satsUsed    = (uint8_t)fieldFixed(body, 7, 0);
  p.leapSeconds = fieldLeap(body, 8);

  // Position: decimal degrees (e.g. 31.82176510) -> 1e-7 degrees
  p.latE7 = fieldFixed(body, 9, 7, &p.positionValid);
  p.lonE7 = fieldFixed(body, 10, 7);
  p.altitudeMm = fieldFixed(body, 11, 3);
  p.geoidSepMm = fieldFixed(body, 12, 3);

  // Velocity: m/s -> mm/s
  p.velNorthMmS = fieldFixed(body, 13, 3, &p.velocityValid);
  p.velEastMmS  = fieldFixed(body, 14, 3);
  p.velDownMmS  = fieldFixed(body, 15, 3);
  int32_t spd   = fieldFixed(body, 16, 3);
  p.speedMmS    = (spd > 0) ? (uint32_t)spd : 0;
  p.headingX100 = fieldU16(body, 17, 2);

  p.hdopX100 = fieldU16(body, 18, 2);
  p.pdopX100 = fieldU16(body, 19, 2);

  if (cb != nullptr) {
    ((LC76G_PvtFn)cb)(p);                      // Back to the real type
  }
  return true;
}

// $PQTMVEL,1,...
static bool decodeVelocity(const char *body, LC76G::GenericFn cb) {
  if (strncmp(body, "PQTMVEL,", 8) != 0) {
    return false;
  }
  LC76G_Velocity v;
  memset(&v, 0, sizeof(v));

  fieldTime(body, 2, v.hour, v.minute, v.second, v.millisecond);
  v.velNorthMmS = fieldFixed(body, 3, 3, &v.valid);
  v.velEastMmS  = fieldFixed(body, 4, 3);
  v.velDownMmS  = fieldFixed(body, 5, 3);
  int32_t g = fieldFixed(body, 6, 3);
  int32_t s = fieldFixed(body, 7, 3);
  v.groundSpeedMmS = (g > 0) ? (uint32_t)g : 0;
  v.speed3dMmS     = (s > 0) ? (uint32_t)s : 0;
  v.headingX100    = fieldU16(body, 8, 2);
  int32_t ga = fieldFixed(body, 9, 3);
  int32_t sa = fieldFixed(body, 10, 3);
  v.groundSpeedAccMmS = (ga > 0) ? (uint32_t)ga : 0;
  v.speedAccMmS       = (sa > 0) ? (uint32_t)sa : 0;
  v.headingAccX100    = fieldU16(body, 11, 2);

  if (cb != nullptr) {
    ((LC76G_VelocityFn)cb)(v);
  }
  return true;
}

// $PQTMDOP,1,...
static bool decodeDop(const char *body, LC76G::GenericFn cb) {
  if (strncmp(body, "PQTMDOP,", 8) != 0) {
    return false;
  }
  LC76G_Dop d;
  d.towMs    = fieldU32(body, 2, 0);
  d.gdopX100 = fieldU16(body, 3, 2);
  d.pdopX100 = fieldU16(body, 4, 2);
  d.tdopX100 = fieldU16(body, 5, 2);
  d.vdopX100 = fieldU16(body, 6, 2);
  d.hdopX100 = fieldU16(body, 7, 2);
  // Fields 8 and 9 (NDOP, EDOP) are "not supported, always null"

  if (cb != nullptr) {
    ((LC76G_DopFn)cb)(d);
  }
  return true;
}

// $PQTMODO,1,<Time>,<State>,<Dist>
static bool decodeOdometer(const char *body, LC76G::GenericFn cb) {
  if (strncmp(body, "PQTMODO,", 8) != 0) {
    return false;
  }
  LC76G_Odometer o;
  fieldTime(body, 2, o.hour, o.minute, o.second, o.millisecond);
  o.enabled    = (fieldFixed(body, 3, 0) == 1);
  o.distanceDm = fieldU32(body, 4, 1);         // metres with 1 decimal -> dm
  if (cb != nullptr) {
    ((LC76G_OdometerFn)cb)(o);
  }
  return true;
}

// $PQTMTIMEGPS,1,<TOW>,<FTOW>,<WN>,<Date>,<Time>,<LeapS>,<Time_Acc>
static bool decodeGpsTime(const char *body, LC76G::GenericFn cb) {
  if (strncmp(body, "PQTMTIMEGPS,", 12) != 0) {
    return false;
  }
  LC76G_GpsTime t;
  memset(&t, 0, sizeof(t));
  t.towMs     = (uint32_t)fieldFixed(body, 2, 0, &t.valid);
  t.towFracNs = fieldU32(body, 3, 0);
  t.week      = (uint16_t)fieldU32(body, 4, 0);
  fieldDate(body, 5, t.year, t.month, t.day);
  fieldTime(body, 6, t.hour, t.minute, t.second, t.millisecond);
  t.leapSeconds = fieldLeap(body, 7);
  bool present = false;
  int32_t acc = fieldFixed(body, 8, 0, &present);
  t.accuracyNs = (present && acc >= 0) ? (uint32_t)acc : 0xFFFFFFFFUL;
  if (cb != nullptr) {
    ((LC76G_GpsTimeFn)cb)(t);
  }
  return true;
}

// $PQTMPL,1,<TOW>,<PUL>,1,1,<PosN>,<PosE>,<PosD>,<VelN>,<VelE>,<VelD>,,,<Time>
// The protection levels are already in mm, mm/s and ns (integers).
static bool decodeProtectionLevel(const char *body, LC76G::GenericFn cb) {
  if (strncmp(body, "PQTMPL,", 7) != 0) {
    return false;
  }
  LC76G_ProtectionLevel pl;
  memset(&pl, 0, sizeof(pl));
  pl.towMs           = fieldU32(body, 2, 0);
  pl.probabilityX100 = fieldU16(body, 3, 2);
  pl.northMm         = (uint32_t)fieldFixed(body, 6, 0, &pl.valid);
  pl.eastMm          = fieldU32(body, 7, 0);
  pl.downMm          = fieldU32(body, 8, 0);
  pl.velNorthMmS     = fieldU32(body, 9, 0);
  pl.velEastMmS      = fieldU32(body, 10, 0);
  pl.velDownMmS      = fieldU32(body, 11, 0);
  pl.timeNs          = fieldU32(body, 14, 0);
  if (cb != nullptr) {
    ((LC76G_ProtectionLevelFn)cb)(pl);
  }
  return true;
}

// $PQTMPOSECEF,1,<Time>,<PosX>,<PosY>,<PosZ>,<PosAcc>   (metres -> cm)
static bool decodeEcefPosition(const char *body, LC76G::GenericFn cb) {
  if (strncmp(body, "PQTMPOSECEF,", 12) != 0) {
    return false;
  }
  LC76G_EcefPosition e;
  memset(&e, 0, sizeof(e));
  fieldTime(body, 2, e.hour, e.minute, e.second, e.millisecond);
  e.xCm        = fieldFixed(body, 3, 2, &e.valid);
  e.yCm        = fieldFixed(body, 4, 2);
  e.zCm        = fieldFixed(body, 5, 2);
  e.accuracyCm = fieldU32(body, 6, 2);
  if (cb != nullptr) {
    ((LC76G_EcefPositionFn)cb)(e);
  }
  return true;
}

// $PQTMVELECEF,1,<Time>,<VelX>,<VelY>,<VelZ>,<VelAcc>   (m/s -> mm/s)
static bool decodeEcefVelocity(const char *body, LC76G::GenericFn cb) {
  if (strncmp(body, "PQTMVELECEF,", 12) != 0) {
    return false;
  }
  LC76G_EcefVelocity e;
  memset(&e, 0, sizeof(e));
  fieldTime(body, 2, e.hour, e.minute, e.second, e.millisecond);
  e.vxMmS       = fieldFixed(body, 3, 3, &e.valid);
  e.vyMmS       = fieldFixed(body, 4, 3);
  e.vzMmS       = fieldFixed(body, 5, 3);
  e.accuracyMmS = fieldU32(body, 6, 3);
  if (cb != nullptr) {
    ((LC76G_EcefVelocityFn)cb)(e);
  }
  return true;
}

// $PQTMPVTECEF,1,<TOW>,<Date>,<Time>,<LeapS>,<PosX>,<PosY>,<PosZ>,<PosAcc>,
//              <VelX>,<VelY>,<VelZ>,<VelAcc>
static bool decodeEcefPvt(const char *body, LC76G::GenericFn cb) {
  if (strncmp(body, "PQTMPVTECEF,", 12) != 0) {
    return false;
  }
  LC76G_EcefPvt e;
  memset(&e, 0, sizeof(e));
  e.towMs = fieldU32(body, 2, 0);
  fieldDate(body, 3, e.year, e.month, e.day);
  fieldTime(body, 4, e.hour, e.minute, e.second, e.millisecond);
  e.leapSeconds    = fieldLeap(body, 5);
  e.xCm            = fieldFixed(body, 6, 2, &e.valid);
  e.yCm            = fieldFixed(body, 7, 2);
  e.zCm            = fieldFixed(body, 8, 2);
  e.posAccuracyCm  = fieldU32(body, 9, 2);
  e.vxMmS          = fieldFixed(body, 10, 3);
  e.vyMmS          = fieldFixed(body, 11, 3);
  e.vzMmS          = fieldFixed(body, 12, 3);
  e.velAccuracyMmS = fieldU32(body, 13, 3);
  if (cb != nullptr) {
    ((LC76G_EcefPvtFn)cb)(e);
  }
  return true;
}


// ===========================================================================
//  HOOK TABLE
// ===========================================================================

bool LC76G::setHook(HookDecoder decode, GenericFn cb) {
  // 1) Same decoder already registered: replace or remove it
  for (uint8_t i = 0; i < LC76G_OUTPUT_HOOKS; i++) {
    if (_hooks[i].decode == decode) {
      if (cb == nullptr) {
        _hooks[i].decode = nullptr;            // Free the slot
      }
      _hooks[i].cb = cb;
      return true;
    }
  }
  if (cb == nullptr) {
    return true;                               // Nothing to remove
  }
  // 2) New decoder: first free slot
  for (uint8_t i = 0; i < LC76G_OUTPUT_HOOKS; i++) {
    if (_hooks[i].decode == nullptr) {
      _hooks[i].decode = decode;
      _hooks[i].cb     = cb;
      return true;
    }
  }
  return false;                                // Table full (see LC76G_OUTPUT_HOOKS)
}


// ===========================================================================
//  PUBLIC API
// ===========================================================================

LC76G_Result LC76G::setPvtOutput(uint8_t rate)      { return setPqtmMessageRate("PQTMPVT", rate, 1); }
LC76G_Result LC76G::setVelocityOutput(uint8_t rate) { return setPqtmMessageRate("PQTMVEL", rate, 1); }
LC76G_Result LC76G::setDopOutput(uint8_t rate)      { return setPqtmMessageRate("PQTMDOP", rate, 1); }

bool LC76G::onPvt(LC76G_PvtFn cb)           { return setHook(decodePvt,      (GenericFn)cb); }
bool LC76G::onVelocity(LC76G_VelocityFn cb) { return setHook(decodeVelocity, (GenericFn)cb); }
bool LC76G::onDop(LC76G_DopFn cb)           { return setHook(decodeDop,      (GenericFn)cb); }
LC76G_Result LC76G::setOdometerOutput(uint8_t rate)        { return setPqtmMessageRate("PQTMODO", rate, 1); }
LC76G_Result LC76G::setGpsTimeOutput(uint8_t rate)         { return setPqtmMessageRate("PQTMTIMEGPS", rate, 1); }
LC76G_Result LC76G::setProtectionLevelOutput(uint8_t rate) { return setPqtmMessageRate("PQTMPL", rate, 1); }
LC76G_Result LC76G::setEcefPositionOutput(uint8_t rate)    { return setPqtmMessageRate("PQTMPOSECEF", rate, 1); }
LC76G_Result LC76G::setEcefVelocityOutput(uint8_t rate)    { return setPqtmMessageRate("PQTMVELECEF", rate, 1); }
LC76G_Result LC76G::setEcefPvtOutput(uint8_t rate)         { return setPqtmMessageRate("PQTMPVTECEF", rate, 1); }

bool LC76G::onOdometer(LC76G_OdometerFn cb)               { return setHook(decodeOdometer,        (GenericFn)cb); }
bool LC76G::onGpsTime(LC76G_GpsTimeFn cb)                 { return setHook(decodeGpsTime,         (GenericFn)cb); }
bool LC76G::onProtectionLevel(LC76G_ProtectionLevelFn cb) { return setHook(decodeProtectionLevel, (GenericFn)cb); }
bool LC76G::onEcefPosition(LC76G_EcefPositionFn cb)       { return setHook(decodeEcefPosition,    (GenericFn)cb); }
bool LC76G::onEcefVelocity(LC76G_EcefVelocityFn cb)       { return setHook(decodeEcefVelocity,    (GenericFn)cb); }
bool LC76G::onEcefPvt(LC76G_EcefPvtFn cb)                 { return setHook(decodeEcefPvt,         (GenericFn)cb); }


// ===========================================================================
//  ODOMETER COMMANDS
// ===========================================================================

// Prints decimetres as metres with one decimal, e.g. 105 -> "10.5"
static void formatDm(char *out, uint8_t size, uint32_t dm) {
  snprintf(out, size, "%lu.%u", (unsigned long)(dm / 10), (unsigned)(dm % 10));
}

LC76G_Result LC76G::setOdometer(bool enable, uint32_t initialDm) {
  // $PQTMCFGODO,W,<State>,<InitDist>   (InitDist in metres)
  char dist[16];
  formatDm(dist, sizeof(dist), initialDm);
  char params[24];
  snprintf(params, sizeof(params), "W,%u,%s", enable ? 1u : 0u, dist);
  return sendPqtm("PQTMCFGODO", params);
}

LC76G_Result LC76G::getOdometer(bool &enabled, uint32_t &initialDm) {
  // Reply: PQTMCFGODO,OK,<State>,<InitDist>
  //  field:     0      1    2        3
  char resp[40];
  LC76G_Result r = sendPqtm("PQTMCFGODO", "R", LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r != LC76G_OK) {
    return r;
  }
  enabled   = (fieldFixed(resp, 2, 0) == 1);
  initialDm = fieldU32(resp, 3, 1);
  return r;
}

LC76G_Result LC76G::resetOdometer() {
  return sendPqtm("PQTMRESETODO");             // Accumulated distance back to 0
}
