/*
 * ============================================================================
 *  LC76G_Nmea.h  -  NMEA 0183 decoder for the Quectel LC76G GNSS module
 * ============================================================================
 *  Part of the Quectel_LCx6G_GNSS Arduino library.
 *
 *  Decodes the standard sentences needed for navigation:
 *    RMC : time, date, fix status, position, speed, course
 *    GGA : fix quality, satellites used, HDOP, altitude
 *    GSV : satellites in view per constellation, signal strength
 *
 *  Design notes:
 *    - No floating point in the decoder: every value is stored as a scaled
 *      integer (e.g. latitude in degrees * 1e7). A 32-bit AVR float has
 *      only ~7 significant digits, which would cost about 1 m of
 *      resolution on coordinates.
 *    - No dynamic memory and no String objects (the Uno R3 has 2 KB RAM).
 *    - The LC76G outputs RMC after GGA and GSV in every 1 Hz epoch, so RMC
 *      is used as the "end of epoch" marker: GSV statistics are published
 *      and the user epoch callback is called at that moment.
 *
 *  Author: Ugo Silato
 *  License: MIT (see LICENSE file)
 * ============================================================================
 */

#ifndef LC76G_NMEA_H
#define LC76G_NMEA_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Constellations, identified by the NMEA talker ID of their GSV sentences
// ---------------------------------------------------------------------------
enum LC76G_System : uint8_t {
  LC76G_GPS = 0,      // Talker "GP"
  LC76G_GLONASS,      // Talker "GL"
  LC76G_GALILEO,      // Talker "GA"
  LC76G_BEIDOU,       // Talker "GB"
  LC76G_QZSS,         // Talker "GQ"
  LC76G_NUM_SYSTEMS   // Number of tracked constellations (keep last)
};

// ---------------------------------------------------------------------------
// Decoded navigation data. All "...Valid" flags tell whether the related
// field was present (non-empty) in the last sentence that carries it.
// ---------------------------------------------------------------------------
struct LC76G_Fix {
  // Fix status
  bool     valid;          // RMC status 'A' = valid fix, 'V' = no fix
  uint8_t  quality;        // GGA fix quality: 0 none, 1 GPS, 2 DGPS, 6 estimated
  uint8_t  satsUsed;       // GGA: satellites used in the solution

  // Position (from RMC)
  bool     positionValid;  // true if latitude/longitude fields were present
  int32_t  latE7;          // Latitude  in degrees * 1e7, positive = North
  int32_t  lonE7;          // Longitude in degrees * 1e7, positive = East

  // Altitude and accuracy (from GGA)
  bool     altitudeValid;  // true if the altitude field was present
  int32_t  altitudeCm;     // Altitude above mean sea level, centimetres
  uint16_t hdopX100;       // Horizontal dilution of precision * 100 (9999 = invalid)

  // Motion (from RMC)
  uint32_t speedKnX100;    // Speed over ground, knots * 100
  uint16_t courseX100;     // Course over ground, degrees * 100

  // UTC time and date (from RMC)
  bool     timeValid;      // true if the time field was present
  uint8_t  hour;           // 0..23
  uint8_t  minute;         // 0..59
  uint8_t  second;         // 0..60 (60 = leap second)
  uint16_t millisecond;    // 0..999
  bool     dateValid;      // true if the date field was present
  uint8_t  day;            // 1..31
  uint8_t  month;          // 1..12
  uint16_t year;           // Full year, e.g. 2026 (module default before fix: 1980)
};

// ---------------------------------------------------------------------------
// NMEA decoder
// ---------------------------------------------------------------------------
class LC76G_Nmea {
public:
  // Function called once per epoch (after each RMC sentence)
  typedef void (*EpochCallback)();

  LC76G_Nmea();

  // Clears all decoded data
  void reset();

  // Decodes one sentence. 'body' is the text after '$' and without CR/LF,
  // checksum already validated (e.g. "GNRMC,123519.000,A,...*6A").
  // Unknown or proprietary sentences are silently ignored.
  void process(const char *body);

  // Last decoded navigation data
  const LC76G_Fix &fix() const { return _fix; }

  // Satellites in view during the last epoch that contained GSV sentences.
  // If GSV is output only every N fixes, the values are kept in between.
  uint8_t satsInView(LC76G_System system) const;
  uint8_t satsInViewTotal() const;

  // Strongest signal (C/N0, dB-Hz) seen during the last complete epoch
  uint8_t maxSnr() const { return _maxSnr; }

  // Registers the end-of-epoch callback (nullptr to disable)
  void onEpoch(EpochCallback cb) { _epochCb = cb; }

  // Convenience conversions (float, for printing or simple maths only)
  float latitudeDeg()  const { return _fix.latE7 / 1.0e7f; }
  float longitudeDeg() const { return _fix.lonE7 / 1.0e7f; }
  float altitudeM()    const { return _fix.altitudeCm / 100.0f; }
  float speedKmh()     const { return _fix.speedKnX100 * 0.01852f; } // 1 kn = 1.852 km/h

  // -------------------------------------------------------------------------
  // Low-level helpers, public because the core class and user code reuse them
  // -------------------------------------------------------------------------

  // Copies field 'index' of a sentence body into 'out' (field 0 = sentence
  // ID such as "GNRMC"). Returns false if the sentence has fewer fields.
  static bool getField(const char *body, uint8_t index, char *out, uint8_t outSize);

  // Parses a decimal string into an integer scaled by 10^decimals
  // ("12.345", 2 -> 1234). Extra decimals are truncated.
  // *ok (optional) is set to false if the string contains no digits.
  static int32_t parseFixed(const char *s, uint8_t decimals, bool *ok = nullptr);

  // Converts an NMEA coordinate ("ddmm.mmmm" / "dddmm.mmmm") plus hemisphere
  // ('N','S','E','W') into degrees * 1e7. Returns false if empty.
  static bool parseCoordinate(const char *raw, char hemi, int32_t &e7);

  // Formats degrees * 1e7 as decimal degrees with 6 decimals, rounded,
  // e.g. 446020566 -> "44.602057", -335000004 -> "-33.500000".
  // Integer arithmetic only (AVR snprintf() has no %f).
  static void formatE7(char *out, uint8_t size, int32_t e7);

private:
  void parseRmc(const char *body);
  void parseGga(const char *body);
  void parseGsv(const char *body);
  void endOfEpoch();

  LC76G_Fix     _fix;                          // Decoded data
  uint8_t       _inView[LC76G_NUM_SYSTEMS];    // Published per-system counts
  uint8_t       _inViewWork[LC76G_NUM_SYSTEMS];// Counts being collected this epoch
  uint8_t       _maxSnr;                       // Published strongest SNR
  uint8_t       _maxSnrWork;                   // Strongest SNR being collected
  bool          _gsvSeen;                      // true if any GSV arrived in this epoch
  EpochCallback _epochCb;                      // User callback
};

#endif // LC76G_NMEA_H
