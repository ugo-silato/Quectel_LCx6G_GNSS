/*
 * ============================================================================
 *  LC76G_Nmea.cpp  -  NMEA 0183 decoder for the Quectel LC76G GNSS module
 * ============================================================================
 *  Part of the Quectel_LC76G Arduino library. Author: Ugo Silato. License: MIT.
 * ============================================================================
 */

#include "LC76G_Nmea.h"
#include <string.h>   // strncmp(), strchr(), memset()
#include <stdlib.h>   // atoi()

// Talker IDs of the GSV sentences, in the same order as LC76G_System
static const char SYSTEM_TALKER[LC76G_NUM_SYSTEMS][3] = { "GP", "GL", "GA", "GB", "GQ" };

// Buffer size for a single field (longest field we read is a coordinate)
static const uint8_t FIELD_SIZE = 16;


// ===========================================================================
//  CONSTRUCTION / RESET
// ===========================================================================

LC76G_Nmea::LC76G_Nmea() : _epochCb(nullptr) {
  reset();
}

void LC76G_Nmea::reset() {
  memset(&_fix, 0, sizeof(_fix));       // All fields to zero / false
  _fix.hdopX100 = 9999;                 // 99.99 = "no solution", as the module reports
  memset(_inView, 0, sizeof(_inView));
  memset(_inViewWork, 0, sizeof(_inViewWork));
  _maxSnr = 0;
  _maxSnrWork = 0;
  _gsvSeen = false;
}


// ===========================================================================
//  LOW-LEVEL HELPERS
// ===========================================================================

bool LC76G_Nmea::getField(const char *body, uint8_t index, char *out, uint8_t outSize) {
  const char *p = body;
  uint8_t current = 0;

  // Skip 'index' commas to reach the start of the requested field
  while (current < index) {
    if (*p == '\0' || *p == '*') {     // Sentence ended before the field
      out[0] = '\0';
      return false;
    }
    if (*p == ',') {
      current++;
    }
    p++;
  }

  // Copy characters up to the next delimiter (',' or '*' or end)
  uint8_t i = 0;
  while (*p != '\0' && *p != ',' && *p != '*' && i < outSize - 1) {
    out[i++] = *p++;
  }
  out[i] = '\0';
  return true;
}

int32_t LC76G_Nmea::parseFixed(const char *s, uint8_t decimals, bool *ok) {
  bool    negative = false;
  bool    anyDigit = false;
  bool    inFraction = false;
  uint8_t fracDigits = 0;
  int32_t value = 0;

  if (*s == '-') {                      // Optional sign
    negative = true;
    s++;
  } else if (*s == '+') {
    s++;
  }

  for (; *s != '\0'; s++) {
    if (*s == '.') {                    // Decimal point: start of fraction
      if (inFraction) {
        break;                          // Second '.' = malformed, stop here
      }
      inFraction = true;
      continue;
    }
    if (*s < '0' || *s > '9') {         // Any other character ends the number
      break;
    }
    if (inFraction) {
      if (fracDigits >= decimals) {     // Ignore (truncate) extra decimals
        continue;
      }
      fracDigits++;
    }
    value = value * 10 + (*s - '0');
    anyDigit = true;
  }

  while (fracDigits < decimals) {       // Pad missing decimals: "1.2" -> 120 for 2 decimals
    value *= 10;
    fracDigits++;
  }

  if (ok != nullptr) {
    *ok = anyDigit;
  }
  return negative ? -value : value;
}

bool LC76G_Nmea::parseCoordinate(const char *raw, char hemi, int32_t &e7) {
  if (raw[0] == '\0') {
    return false;
  }

  // Integer part "dddmm": degrees are all digits except the last two
  int32_t intPart = 0;
  const char *p = raw;
  while (*p >= '0' && *p <= '9') {
    intPart = intPart * 10 + (*p - '0');
    p++;
  }
  int32_t degrees = intPart / 100;
  int32_t minutes = intPart % 100;

  // Fraction of minute, converted to millionths of a minute (6 digits)
  int32_t fracMicro = 0;
  if (*p == '.') {
    int32_t scale = 100000L;            // First decimal digit = 100000 millionths
    for (p++; *p >= '0' && *p <= '9' && scale > 0; p++) {
      fracMicro += (int32_t)(*p - '0') * scale;
      scale /= 10;
    }
  }

  // degrees*1e7 + minutes/60*1e7, computed as (minutes in 1e-6) * 10 / 60.
  // Worst case 59.999999 min -> 599999990, still inside int32 range.
  int32_t minutesE6 = minutes * 1000000L + fracMicro;
  e7 = degrees * 10000000L + (minutesE6 * 10L) / 60L;

  if (hemi == 'S' || hemi == 'W') {     // South and West are negative
    e7 = -e7;
  }
  return true;
}


void LC76G_Nmea::formatE7(char *out, uint8_t size, int32_t e7) {
  int32_t e6 = (e7 >= 0) ? (e7 + 5) / 10 : (e7 - 5) / 10;   // Round to 1e-6 degree
  bool negative = (e6 < 0);
  uint32_t a = negative ? (uint32_t)(-e6) : (uint32_t)e6;
  snprintf(out, size, "%s%lu.%06lu", negative ? "-" : "",
           (unsigned long)(a / 1000000UL), (unsigned long)(a % 1000000UL));
}


// ===========================================================================
//  SENTENCE DISPATCH
// ===========================================================================

void LC76G_Nmea::process(const char *body) {
  // Standard sentences: 2-char talker ID + 3-char type, e.g. "GNRMC".
  // Proprietary sentences start with 'P' ("PAIR...", "PQTM...") and are skipped.
  if (body[0] == 'P' || strlen(body) < 5) {
    return;
  }
  const char *type = body + 2;          // Skip the talker ID

  if (strncmp(type, "RMC", 3) == 0) {
    parseRmc(body);
  } else if (strncmp(type, "GGA", 3) == 0) {
    parseGga(body);
  } else if (strncmp(type, "GSV", 3) == 0) {
    parseGsv(body);
  }
}


// ===========================================================================
//  SENTENCE PARSERS
// ===========================================================================

// $xxRMC,hhmmss.sss,A,ddmm.mmmm,N,dddmm.mmmm,E,speed,course,ddmmyy,...
//  field:     1     2     3     4     5      6   7     8      9
void LC76G_Nmea::parseRmc(const char *body) {
  char f[FIELD_SIZE];
  char hemi[2];

  // --- Time "hhmmss.sss" ---
  getField(body, 1, f, sizeof(f));
  _fix.timeValid = (strlen(f) >= 6);
  if (_fix.timeValid) {
    _fix.hour   = (f[0] - '0') * 10 + (f[1] - '0');
    _fix.minute = (f[2] - '0') * 10 + (f[3] - '0');
    _fix.second = (f[4] - '0') * 10 + (f[5] - '0');
    _fix.millisecond = (f[6] == '.') ? (uint16_t)parseFixed(&f[6], 3) : 0;
  }

  // --- Fix status ---
  getField(body, 2, f, sizeof(f));
  _fix.valid = (f[0] == 'A');

  // --- Latitude / longitude ---
  char lat[FIELD_SIZE];
  char lon[FIELD_SIZE];
  getField(body, 3, lat, sizeof(lat));
  getField(body, 4, hemi, sizeof(hemi));
  bool latOk = parseCoordinate(lat, hemi[0], _fix.latE7);
  getField(body, 5, lon, sizeof(lon));
  getField(body, 6, hemi, sizeof(hemi));
  bool lonOk = parseCoordinate(lon, hemi[0], _fix.lonE7);
  _fix.positionValid = latOk && lonOk;

  // --- Speed (knots) and course (degrees) ---
  getField(body, 7, f, sizeof(f));
  _fix.speedKnX100 = (uint32_t)parseFixed(f, 2);
  getField(body, 8, f, sizeof(f));
  _fix.courseX100 = (uint16_t)parseFixed(f, 2);

  // --- Date "ddmmyy" ---
  getField(body, 9, f, sizeof(f));
  _fix.dateValid = (strlen(f) >= 6);
  if (_fix.dateValid) {
    _fix.day   = (f[0] - '0') * 10 + (f[1] - '0');
    _fix.month = (f[2] - '0') * 10 + (f[3] - '0');
    uint8_t yy = (f[4] - '0') * 10 + (f[5] - '0');
    _fix.year  = (yy >= 80) ? 1900 + yy : 2000 + yy;  // Module default date is 1980
  }

  // RMC closes the epoch on the LC76G
  endOfEpoch();
}

// $xxGGA,time,lat,N,lon,E,quality,numSV,HDOP,altitude,M,...
//  field:  1   2  3  4  5    6      7     8      9
void LC76G_Nmea::parseGga(const char *body) {
  char f[FIELD_SIZE];
  bool ok;

  getField(body, 6, f, sizeof(f));
  _fix.quality = (uint8_t)atoi(f);

  getField(body, 7, f, sizeof(f));
  _fix.satsUsed = (uint8_t)atoi(f);

  getField(body, 8, f, sizeof(f));
  int32_t hdop = parseFixed(f, 2, &ok);
  _fix.hdopX100 = ok ? (uint16_t)hdop : 9999;

  getField(body, 9, f, sizeof(f));
  _fix.altitudeCm = parseFixed(f, 2, &ok);
  _fix.altitudeValid = ok;
}

// $xxGSV,numMsg,msgNum,numSV,{PRN,elev,azim,SNR} x 1..4,signalID
//  field:   1      2     3     4    5    6    7   (+4 per satellite)
void LC76G_Nmea::parseGsv(const char *body) {
  char f[FIELD_SIZE];
  _gsvSeen = true;                        // This epoch carries satellite data

  // Which constellation does this GSV belong to?
  uint8_t sys = LC76G_NUM_SYSTEMS;
  for (uint8_t i = 0; i < LC76G_NUM_SYSTEMS; i++) {
    if (strncmp(body, SYSTEM_TALKER[i], 2) == 0) {
      sys = i;
      break;
    }
  }

  getField(body, 2, f, sizeof(f));
  uint8_t msgNum = (uint8_t)atoi(f);
  getField(body, 3, f, sizeof(f));
  uint8_t inView = (uint8_t)atoi(f);

  // The "satellites in view" count is repeated in every GSV message of the
  // group: take it from the first one only.
  if (sys < LC76G_NUM_SYSTEMS && msgNum == 1) {
    _inViewWork[sys] = inView;
  }

  // SNR of up to 4 satellites per message: fields 7, 11, 15, 19.
  // An empty SNR means the satellite is predicted but not tracked.
  for (uint8_t k = 0; k < 4; k++) {
    if (!getField(body, 7 + 4 * k, f, sizeof(f))) {
      break;                              // Fewer satellites in this message
    }
    if (f[0] != '\0') {
      uint8_t snr = (uint8_t)atoi(f);
      if (snr > _maxSnrWork) {
        _maxSnrWork = snr;
      }
    }
  }
}

// Publishes the statistics collected during the epoch and calls the user.
// If no GSV arrived (GSV output every N fixes), the previous values are
// kept instead of dropping to zero.
void LC76G_Nmea::endOfEpoch() {
  if (_gsvSeen) {
    memcpy(_inView, _inViewWork, sizeof(_inView));
    _maxSnr = _maxSnrWork;
  }
  memset(_inViewWork, 0, sizeof(_inViewWork));
  _maxSnrWork = 0;
  _gsvSeen = false;

  if (_epochCb != nullptr) {
    _epochCb();
  }
}


// ===========================================================================
//  GETTERS
// ===========================================================================

uint8_t LC76G_Nmea::satsInView(LC76G_System system) const {
  return (system < LC76G_NUM_SYSTEMS) ? _inView[system] : 0;
}

uint8_t LC76G_Nmea::satsInViewTotal() const {
  uint8_t total = 0;
  for (uint8_t i = 0; i < LC76G_NUM_SYSTEMS; i++) {
    total += _inView[i];
  }
  return total;
}
