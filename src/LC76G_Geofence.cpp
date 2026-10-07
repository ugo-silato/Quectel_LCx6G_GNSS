/*
 * ============================================================================
 *  LC76G_Geofence.cpp  -  Geofence (virtual fence) for the LC76G
 * ============================================================================
 *  Part of the Quectel_LCx6G_GNSS Arduino library. Author: Ugo Silato. License: MIT.
 *
 *  A geofence is an area defined by coordinates. The module compares its
 *  position with up to 4 areas and reports, for each one, whether the
 *  receiver is inside or outside:
 *    - $PQTMGEOFENCESTATUS messages (enabled with PQTMCFGMSGRATE)
 *    - the GEOFENCE output pin: HIGH = inside the active area(s)
 *
 *  Commands used (Quectel GNSS Protocol Specification):
 *    PQTMCFGGEOFENCE  Set / get a geofence
 *    PQTMCFGMSGRATE   Enable the PQTMGEOFENCESTATUS output
 * ============================================================================
 */

#include "LC76G.h"
#include <stdlib.h>   // atoi()

// Shapes accepted by PQTMCFGGEOFENCE
static const uint8_t SHAPE_CIRCLE_RADIUS = 0;  // Centre + radius
static const uint8_t SHAPE_TRIANGLE      = 2;  // 3 corners
static const uint8_t SHAPE_QUADRANGLE    = 3;  // 4 corners

// Appends ",<lat>,<lon>" to 'buf' (of total size 'size')
static void appendPoint(char *buf, uint8_t size, int32_t latE7, int32_t lonE7) {
  char lat[14];
  char lon[14];
  LC76G_Nmea::formatE7(lat, sizeof(lat), latE7);
  LC76G_Nmea::formatE7(lon, sizeof(lon), lonE7);
  uint8_t len = (uint8_t)strlen(buf);
  snprintf(buf + len, size - len, ",%s,%s", lat, lon);
}

// Basic validity check of a coordinate pair (degrees * 1e7)
static bool validPoint(int32_t latE7, int32_t lonE7) {
  return latE7 >= -900000000L && latE7 <= 900000000L &&
         lonE7 >= -1800000000L && lonE7 <= 1800000000L;
}


// ===========================================================================
//  CONFIGURATION
// ===========================================================================

LC76G_Result LC76G::setGeofenceCircle(uint8_t index, int32_t latE7, int32_t lonE7,
                                      uint32_t radiusM) {
  if (index >= LC76G_GEOFENCE_COUNT || radiusM == 0 || !validPoint(latE7, lonE7)) {
    return LC76G_PARAM_ERROR;
  }
  // W,<Index>,<Status=1>,<Res=0>,<Shape=0>,<Lat0>,<Lon0>,<Radius>
  char params[64];
  snprintf(params, sizeof(params), "W,%u,1,0,%u", (unsigned)index, (unsigned)SHAPE_CIRCLE_RADIUS);
  appendPoint(params, sizeof(params), latE7, lonE7);
  uint8_t len = (uint8_t)strlen(params);
  snprintf(params + len, sizeof(params) - len, ",%lu", (unsigned long)radiusM);
  return sendPqtm("PQTMCFGGEOFENCE", params);
}

LC76G_Result LC76G::setGeofencePolygon(uint8_t index, const int32_t latE7[],
                                       const int32_t lonE7[], uint8_t count) {
  if (index >= LC76G_GEOFENCE_COUNT || (count != 3 && count != 4)) {
    return LC76G_PARAM_ERROR;
  }
  for (uint8_t i = 0; i < count; i++) {
    if (!validPoint(latE7[i], lonE7[i])) {
      return LC76G_PARAM_ERROR;
    }
  }
  // W,<Index>,<Status=1>,<Res=0>,<Shape>,<Lat0>,<Lon0>,...,<Lat3>,<Lon3>
  char params[120];                            // 4 points x ~24 chars + header
  snprintf(params, sizeof(params), "W,%u,1,0,%u", (unsigned)index,
           (unsigned)(count == 3 ? SHAPE_TRIANGLE : SHAPE_QUADRANGLE));
  for (uint8_t i = 0; i < count; i++) {
    appendPoint(params, sizeof(params), latE7[i], lonE7[i]);
  }
  return sendPqtm("PQTMCFGGEOFENCE", params);
}

LC76G_Result LC76G::disableGeofence(uint8_t index) {
  if (index >= LC76G_GEOFENCE_COUNT) {
    return LC76G_PARAM_ERROR;
  }
  // The shape fields can be omitted when disabling (protocol note 5)
  char params[8];
  snprintf(params, sizeof(params), "W,%u,0", (unsigned)index);
  LC76G_Result r = sendPqtm("PQTMCFGGEOFENCE", params);
  if (r == LC76G_OK) {
    _geoState[index] = LC76G_GEO_UNKNOWN;
  }
  return r;
}

LC76G_Result LC76G::getGeofenceEnabled(uint8_t index, bool &enabled) {
  if (index >= LC76G_GEOFENCE_COUNT) {
    return LC76G_PARAM_ERROR;
  }
  // Reply: PQTMCFGGEOFENCE,OK,<Index>,<Status>[,...]  -> field 3 = status
  char params[6];
  snprintf(params, sizeof(params), "R,%u", (unsigned)index);
  char resp[LC76G_LINE_MAX];
  LC76G_Result r = sendPqtm("PQTMCFGGEOFENCE", params, LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r == LC76G_OK) {
    char f[4];
    LC76G_Nmea::getField(resp, 3, f, sizeof(f));
    enabled = (atoi(f) == 1);
  }
  return r;
}

LC76G_Result LC76G::setGeofenceStatusOutput(uint8_t rate) {
  // PQTMGEOFENCESTATUS is message version 1
  return setPqtmMessageRate("PQTMGEOFENCESTATUS", rate, 1);
}


// ===========================================================================
//  STATUS
// ===========================================================================

LC76G_GeofenceState LC76G::geofenceState(uint8_t index) const {
  return (index < LC76G_GEOFENCE_COUNT) ? (LC76G_GeofenceState)_geoState[index]
                                        : LC76G_GEO_UNKNOWN;
}

void LC76G::setGeofencePin(int8_t pin) {
  _geoPin = pin;
  if (_geoPin >= 0) {
    pinMode(_geoPin, INPUT);                   // Plain input: never pull it up
  }
}

bool LC76G::geofencePinInside() const {
  return (_geoPin >= 0) && digitalRead(_geoPin) == HIGH;
}
