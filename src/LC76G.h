/*
 * ============================================================================
 *  LC76G.h  -  Core driver for the Quectel LC76G GNSS module
 * ============================================================================
 *  Part of the Quectel_LC76G Arduino library.
 *
 *  Responsibilities of the core class:
 *    - Owns the link to the module: any Arduino Stream (HardwareSerial,
 *      SoftwareSerial, or the I2C port LC76G_I2C from LC76G_I2C.h)
 *    - Assembles incoming bytes into sentences and validates checksums
 *    - Feeds standard NMEA sentences to the decoder (member 'nmea')
 *    - Sends PAIR commands with automatic checksum and waits for the
 *      $PAIR001 acknowledgement (and for the query result, if any)
 *    - Optionally controls module power and reset through GPIO pins,
 *      with drive modes that are safe for different hardware designs
 *    - Auto-configures the module UART baud rate (e.g. 115200 -> 38400)
 *
 *    - Sends PQTM commands, decodes PQTM output messages through callbacks
 *
 *  Supported boards: Arduino Uno R3 (AVR) and Uno R4 (Renesas RA4M1).
 *
 *  Author: Ugo Silato
 *  License: MIT (see LICENSE file)
 * ============================================================================
 */

#ifndef LC76G_H
#define LC76G_H

#include <Arduino.h>
#include "LC76G_Nmea.h"

// ---------------------------------------------------------------------------
// Library limits and defaults
// ---------------------------------------------------------------------------
#define LC76G_LINE_MAX          160     // Max sentence length incl. terminator. Standard NMEA
                                        // is max 82, but some PQTM messages are longer: a
                                        // $PQTMPVT can reach ~155 characters (negative
                                        // coordinates and velocities, high altitude)
#define LC76G_DEFAULT_BAUD      115200  // Factory default UART baud rate
#define LC76G_BOOT_MS           1500    // Time the module needs after power-on / reset
#define LC76G_POWER_OFF_MS      1000    // Off time used when power-cycling
#define LC76G_RESET_PULSE_MS    100     // Reset pulse length
#define LC76G_DETECT_MS         3000    // Max wait for a valid sentence when probing a baud rate
#define LC76G_CMD_TIMEOUT_MS    1000    // Default timeout for PAIR commands
#define LC76G_READY_MS          5000    // Max wait in begin() for the module to accept commands
                                        // (it ends as soon as the module answers; tested on the
                                        // reference shield, where the module needed up to ~6 s,
                                        // the missing second being covered by the command retry)

// ---------------------------------------------------------------------------
// Result of a command. Values 0..5 are the <Result> field of $PAIR001.
// Negative values are generated locally by the library.
// ---------------------------------------------------------------------------
enum LC76G_Result : int8_t {
  LC76G_OK            =  0,  // Command executed successfully
  LC76G_PROCESSING    =  1,  // Module is still processing (intermediate state)
  LC76G_FAILED        =  2,  // Command sending failed
  LC76G_NOT_SUPPORTED =  3,  // Command ID not supported by this module/firmware
  LC76G_PARAM_ERROR   =  4,  // Parameter out of range, missing, or checksum error
  LC76G_BUSY          =  5,  // Module service busy, retry later
  LC76G_TIMEOUT       = -1,  // No acknowledgement (or no query result) in time
  LC76G_NO_PORT       = -2,  // begin() not called
  LC76G_NO_REBOOT     = -3   // A reboot is required but no power/reset pin is configured
};

// ---------------------------------------------------------------------------
// How a control pin (power enable / reset) is driven.
//
//  LC76G_DRIVE_PUSH_PULL_HIGH
//      Asserted = OUTPUT HIGH, released = OUTPUT LOW.
//      For logic inputs with their own protection (e.g. the EN pin of a
//      load switch or regulator at the host voltage).
//
//  LC76G_DRIVE_WEAK_PULLUP_HIGH
//      Asserted = INPUT_PULLUP, released = OUTPUT LOW.
//      For a pin wired DIRECTLY to an NPN transistor base without a series
//      resistor: the internal pull-up (20-50 kOhm) limits the base current
//      to a safe ~0.1 mA. Used by the reference shield, revision 1.
//
//  LC76G_DRIVE_OPEN_DRAIN_LOW
//      Asserted = OUTPUT LOW, released = INPUT (high impedance).
//      For active-low inputs pulled up to 3.3 V on the module side
//      (e.g. RESET_N wired directly): the 5 V host never drives the line HIGH.
// ---------------------------------------------------------------------------
enum LC76G_DriveMode : uint8_t {
  LC76G_DRIVE_PUSH_PULL_HIGH = 0,
  LC76G_DRIVE_WEAK_PULLUP_HIGH,
  LC76G_DRIVE_OPEN_DRAIN_LOW
};

// ---------------------------------------------------------------------------
// Standard NMEA sentence types, numbered as in PAIR062 / PAIR063.
// Default on the LC76G: GGA, GLL, GSA, GSV, RMC, VTG every fix; the others off.
// ---------------------------------------------------------------------------
enum LC76G_NmeaType : uint8_t {
  LC76G_NMEA_GGA = 0,   // Fix data: quality, satellites used, HDOP, altitude
  LC76G_NMEA_GLL,       // Geographic position (duplicate of RMC data)
  LC76G_NMEA_GSA,       // DOP and IDs of the satellites used
  LC76G_NMEA_GSV,       // Satellites in view and signal strength (long!)
  LC76G_NMEA_RMC,       // Recommended minimum: time, date, position, speed
  LC76G_NMEA_VTG,       // Course and speed (duplicate of RMC data)
  LC76G_NMEA_ZDA,       // UTC time and date
  LC76G_NMEA_GRS,       // Range residuals
  LC76G_NMEA_GST,       // Position error statistics
  LC76G_NMEA_GNS,       // Fix data (multi-constellation variant of GGA)
  LC76G_NMEA_TYPE_COUNT // Number of types (keep last)
};

// ---------------------------------------------------------------------------
// Constellation bit mask for setConstellations() / getConstellations().
// Combine with '|', e.g. LC76G_GNSS_GPS | LC76G_GNSS_GALILEO.
// ---------------------------------------------------------------------------
#define LC76G_GNSS_GPS      0x01   // Global Positioning System (USA)
#define LC76G_GNSS_GLONASS  0x02   // GLObal NAvigation Satellite System (Russia)
#define LC76G_GNSS_GALILEO  0x04   // Galileo (European Union)
#define LC76G_GNSS_BDS      0x08   // BeiDou Navigation Satellite System (China)
#define LC76G_GNSS_QZSS     0x10   // Quasi-Zenith Satellite System (Japan)
#define LC76G_GNSS_ALL      0x1F   // All five systems

// ---------------------------------------------------------------------------
// Jamming detection status, as reported by $PAIRSPF / $PQTMJAMMINGSTATUS
// ---------------------------------------------------------------------------
enum LC76G_JamStatus : uint8_t {
  LC76G_JAM_UNKNOWN  = 0,   // Detection off, or not evaluated yet
  LC76G_JAM_GOOD     = 1,   // No jamming, healthy status
  LC76G_JAM_WARNING  = 2,   // Interference detected
  LC76G_JAM_CRITICAL = 3    // Strong interference, anti-jamming repair failed
};

// ---------------------------------------------------------------------------
// Geofence
// ---------------------------------------------------------------------------
#define LC76G_GEOFENCE_COUNT 4     // Geofences 0..3

enum LC76G_GeofenceState : uint8_t {
  LC76G_GEO_UNKNOWN = 0,           // Geofence disabled, or no position fix
  LC76G_GEO_INSIDE  = 1,           // Receiver inside the area
  LC76G_GEO_OUTSIDE = 2            // Receiver outside the area
};

// ---------------------------------------------------------------------------
// 1PPS (One Pulse Per Second)
// ---------------------------------------------------------------------------
enum LC76G_PpsMode : uint8_t {
  LC76G_PPS_ALWAYS          = 1,   // Pulse always, even without a fix (not synchronised)
  LC76G_PPS_2D_FIX          = 2,   // Pulse only with a 2D fix
  LC76G_PPS_3D_FIX          = 3,   // Pulse only with a 3D fix
  LC76G_PPS_AFTER_FIRST_FIX = 4    // Pulse after the first fix, then always
};

struct LC76G_PpsConfig {
  bool          enabled;           // Pulse output on/off
  uint16_t      durationMs;        // Pulse length, 1..999 ms
  LC76G_PpsMode mode;              // When the pulse is generated
  bool          activeHigh;        // true = positive pulse, false = negative pulse
};

// ---------------------------------------------------------------------------
// Navigation mode (PAIR080): tunes the position filter to the application
// ---------------------------------------------------------------------------
enum LC76G_NavMode : uint8_t {
  LC76G_NAV_NORMAL     = 0,   // General purpose (required by ALP mode)
  LC76G_NAV_FITNESS    = 1,   // Walking/running, < 5 m/s (required by GLP and FLP)
  LC76G_NAV_BALLOON    = 3,   // High-altitude balloon
  LC76G_NAV_STATIONARY = 4,   // Not moving (zero dynamics assumed)
  LC76G_NAV_DRONE      = 5,   // Drones: hovering and cruising
  LC76G_NAV_SWIMMING   = 7    // Swimming: smoothed track
};

// ---------------------------------------------------------------------------
// Periodic power saving mode (PAIR690): run / sleep cycles
// ---------------------------------------------------------------------------
enum LC76G_PeriodicMode : uint8_t {
  LC76G_PERIODIC_OFF    = 0,  // Disabled (continuous operation)
  LC76G_PERIODIC_SMART  = 1,  // Smart periodic mode
  LC76G_PERIODIC_STRICT = 2   // Strict periodic mode
};

struct LC76G_PeriodicConfig {
  LC76G_PeriodicMode mode;
  uint32_t runS;              // Running time after wake-up and fix, 3..518400 s
  uint32_t sleepS;            // Sleeping time, 3..518400 s
  uint32_t runNoSignalS;      // Running time used when there is no signal (0 or 3..518400)
  uint32_t sleepNoSignalS;    // Sleeping time used when there is no signal (0 or 3..518400)
};

// ---------------------------------------------------------------------------
// EPOC constellation bits (PAIR498 / PAIR508 / PAIR509).
// Only GPS, GPS + Galileo and GPS + BDS are accepted.
// ---------------------------------------------------------------------------
#define LC76G_EPOC_GPS      0x01
#define LC76G_EPOC_GALILEO  0x02
#define LC76G_EPOC_BDS      0x04

// ---------------------------------------------------------------------------
// LOCUS (built-in logger, 128 KB) - save mode bits for setLocusMode()
// ---------------------------------------------------------------------------
#define LC76G_LOCUS_EVERY_FIX     0x01  // Record every fix
#define LC76G_LOCUS_TIME          0x02  // Record when the time threshold is met
#define LC76G_LOCUS_SPEED         0x04  // Record when the speed threshold is met
#define LC76G_LOCUS_DISTANCE      0x08  // Record when the distance threshold is met
#define LC76G_LOCUS_BEFORE_SLEEP  0x10  // Record before entering sleep
#define LC76G_LOCUS_USER          0x20  // Record on request (logLocusNow())

// Threshold types for setLocusThreshold() / getLocusThreshold()
enum LC76G_LocusTrigger : uint8_t {
  LC76G_LOCUS_BY_TIME     = 0,          // Seconds, 1..43200
  LC76G_LOCUS_BY_SPEED    = 1,          // m/s, 1..100
  LC76G_LOCUS_BY_DISTANCE = 2           // Metres, 1..50000
};

// What clearLocus() erases
enum LC76G_LocusClear : uint8_t {
  LC76G_LOCUS_CLEAR_ALL      = 0,       // Recorded data and settings (back to default)
  LC76G_LOCUS_CLEAR_DATA     = 1,       // Recorded data only
  LC76G_LOCUS_CLEAR_SETTINGS = 2        // Settings only (back to default)
};

// One LOCUS record decoded from the compact format ($PAIR908,2,...).
// Time, position, fix type and satellites are verified against the
// protocol examples; altitude, heading and HDOP against an NMEA read-back
// of the same records. Values are given as stored by the module.
struct LC76G_LocusRecord {
  uint32_t utc;          // UTC, seconds since 1970-01-01 (Unix time)
  uint8_t  fixType;      // Fix quality, as GGA: 0 none, 1 GPS, 2 DGPS, 6 estimated
  int32_t  latE7;        // Latitude,  degrees * 1e7 (same unit as LC76G_Fix)
  int32_t  lonE7;        // Longitude, degrees * 1e7
  int16_t  heightRaw;    // Altitude above mean sea level, metres (verified)
  uint16_t speedRaw;     // Ground speed (protocol unit: m/s, scale NOT verified yet)
  uint16_t headingRaw;   // Heading, degrees * 100 (verified)
  uint16_t hdopRaw;      // HDOP * 100 (verified)
  uint8_t  sats;         // Satellites used
};

// Counters of a LOCUS read-back
struct LC76G_LocusDumpStats {
  uint32_t announced;    // Records announced by the module ($PAIR908,1)
  uint32_t received;     // Valid items received: records (compact) or sentences (NMEA)
  uint32_t rejected;     // Damaged items discarded (wrong format or length)
};

// Function called for every LOCUS record of a compact read-back
typedef void (*LC76G_LocusRecordFn)(const LC76G_LocusRecord &rec);

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Output settings (LC76G_Settings.cpp)
// ---------------------------------------------------------------------------

// Decimal places used by the module in the standard NMEA sentences
// (PQTMCFGNMEADP). Factory values: 3, 6, 3, 2, 2, 2.
struct LC76G_NmeaDecimals {
  uint8_t utc;             // UTC seconds, 0..3
  uint8_t position;        // Latitude / longitude minutes, 0..8
  uint8_t altitude;        // Altitude and geoid separation, 0..3
  uint8_t dop;             // DOP values, 0..3
  uint8_t speed;           // Speed, 0..3
  uint8_t course;          // Course over ground, 0..3
};

// RTCM 3 output (PAIR432 / PAIR433). RTCM is BINARY data sent on the
// same UART as NMEA: the library does not decode it.
enum LC76G_RtcmMode : int8_t {
  LC76G_RTCM_OFF  = -1,    // No RTCM output (factory value)
  LC76G_RTCM_MSM4 =  0,    // MSM4 observations (1074, 1084, 1094, 1114, 1124)
  LC76G_RTCM_MSM7 =  1     // MSM7 observations (1077, 1087, 1097, 1117, 1127)
};

// DGPS correction source (PAIR400 / PAIR401)
// ---------------------------------------------------------------------------
enum LC76G_DgpsSource : uint8_t {
  LC76G_DGPS_NONE = 0,   // No correction
  LC76G_DGPS_SBAS = 2,   // SBAS: WAAS (USA), EGNOS (Europe), GAGAN (India), MSAS (Japan)
  LC76G_DGPS_SLAS = 3    // QZSS SLAS (Japan only)
};

// ---------------------------------------------------------------------------
// Estimated positioning error, from $PQTMEPE (message version 2).
// Values in millimetres; a field is 0 when the module left it empty.
// ---------------------------------------------------------------------------
struct LC76G_Epe {
  uint32_t northMm;      // Estimated north error
  uint32_t eastMm;       // Estimated east error
  uint32_t downMm;       // Estimated vertical (down) error
  uint32_t horizontalMm; // Estimated 2D (horizontal) error
  uint32_t total3dMm;    // Estimated 3D error
};

// ---------------------------------------------------------------------------
// PQTM output messages (LC76G_Output.cpp)
// All values are scaled integers (no float on the Uno R3):
//   angles in 1e-7 degrees, lengths in mm, speeds in mm/s,
//   headings in 0.01 degrees, DOP values x100 (9999 = invalid).
// The structures are NOT stored in the library: each message is decoded
// into a temporary structure and passed to the sketch's callback, so a
// sketch that does not use a message spends no RAM on it.
// ---------------------------------------------------------------------------

// Leap seconds field left empty by the module
#define LC76G_LEAP_UNKNOWN  (-128)

// $PQTMPVT (version 1): position, velocity and time in one message
struct LC76G_Pvt {
  uint32_t towMs;          // GPS time of week, milliseconds
  uint16_t year;           // UTC date
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;           // UTC time
  uint8_t  minute;
  uint8_t  second;
  uint16_t millisecond;
  uint8_t  quality;        // Same as GGA: 0 none, 1 GPS, 2 DGPS/SBAS, 6 estimated
  uint8_t  fixMode;        // 0 no fix, 2 = 2D, 3 = 3D
  uint8_t  satsUsed;       // Satellites used in the solution
  int8_t   leapSeconds;    // GPS-UTC leap seconds, LC76G_LEAP_UNKNOWN if empty
  bool     positionValid;  // false: the position fields below are 0
  int32_t  latE7;          // Latitude, 1e-7 degrees (+ = north)
  int32_t  lonE7;          // Longitude, 1e-7 degrees (+ = east)
  int32_t  altitudeMm;     // Altitude above mean sea level
  int32_t  geoidSepMm;     // Geoid separation
  bool     velocityValid;  // false: the velocity fields below are 0
  int32_t  velNorthMmS;    // North velocity
  int32_t  velEastMmS;     // East velocity
  int32_t  velDownMmS;     // Down velocity (+ = descending)
  uint32_t speedMmS;       // Ground speed
  uint16_t headingX100;    // Heading, 0..36000
  uint16_t hdopX100;       // Horizontal dilution of precision
  uint16_t pdopX100;       // Position (3D) dilution of precision
};

// $PQTMVEL (version 1): velocity with accuracy estimates
struct LC76G_Velocity {
  uint8_t  hour;           // UTC time of the solution
  uint8_t  minute;
  uint8_t  second;
  uint16_t millisecond;
  bool     valid;          // false: the module left the fields empty (no fix)
  int32_t  velNorthMmS;    // North velocity
  int32_t  velEastMmS;     // East velocity
  int32_t  velDownMmS;     // Down velocity (+ = descending)
  uint32_t groundSpeedMmS; // 2D speed
  uint32_t speed3dMmS;     // 3D speed
  uint16_t headingX100;    // Heading, 0..36000
  uint32_t groundSpeedAccMmS; // Estimated 2D speed accuracy
  uint32_t speedAccMmS;    // Estimated 3D speed accuracy
  uint16_t headingAccX100; // Estimated heading accuracy (65535 = 655.35 or more)
};

// $PQTMDOP (version 1): all dilution of precision values
struct LC76G_Dop {
  uint32_t towMs;          // GPS time of week, ms (0 if empty)
  uint16_t gdopX100;       // Geometric
  uint16_t pdopX100;       // Position (3D)
  uint16_t tdopX100;       // Time
  uint16_t vdopX100;       // Vertical
  uint16_t hdopX100;       // Horizontal
};

// $PQTMODO (version 1): odometer, distance since the last reset
struct LC76G_Odometer {
  uint8_t  hour;           // UTC time of the message
  uint8_t  minute;
  uint8_t  second;
  uint16_t millisecond;
  bool     enabled;        // Odometer state
  uint32_t distanceDm;     // Distance in decimetres (initial distance included)
};

// $PQTMTIMEGPS (version 1): GPS time with its accuracy
struct LC76G_GpsTime {
  bool     valid;          // false: time of week / week left empty by the module
  uint32_t towMs;          // Time of week, integer milliseconds
  uint32_t towFracNs;      // Fraction of the millisecond, nanoseconds
  uint16_t week;           // GPS week number (since 6 Jan 1980, not rolled over)
  uint16_t year;           // UTC date and time
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
  uint16_t millisecond;
  int8_t   leapSeconds;    // LC76G_LEAP_UNKNOWN if empty
  uint32_t accuracyNs;     // Time accuracy estimate (0xFFFFFFFF if empty)
};

// $PQTMPL (version 1): protection levels (bounds of the position error
// with the given probability)
struct LC76G_ProtectionLevel {
  bool     valid;          // false: the module left the levels empty (no fix)
  uint32_t towMs;          // Time of week, ms
  uint16_t probabilityX100;// Probability of uncertainty level per epoch, % x100
  uint32_t northMm;        // Position protection levels
  uint32_t eastMm;
  uint32_t downMm;
  uint32_t velNorthMmS;    // Velocity protection levels
  uint32_t velEastMmS;
  uint32_t velDownMmS;
  uint32_t timeNs;         // Time protection level
};

// ECEF = Earth-Centered, Earth-Fixed cartesian coordinates (WGS84).
// Positions in CENTIMETRES (the Earth radius in mm does not fit 32 bit).
// $PQTMPOSECEF (version 1)
struct LC76G_EcefPosition {
  uint8_t  hour;           // UTC time
  uint8_t  minute;
  uint8_t  second;
  uint16_t millisecond;
  bool     valid;          // false: fields left empty by the module
  int32_t  xCm;
  int32_t  yCm;
  int32_t  zCm;
  uint32_t accuracyCm;     // 3D position accuracy (1 sigma)
};

// $PQTMVELECEF (version 1)
struct LC76G_EcefVelocity {
  uint8_t  hour;           // UTC time
  uint8_t  minute;
  uint8_t  second;
  uint16_t millisecond;
  bool     valid;
  int32_t  vxMmS;
  int32_t  vyMmS;
  int32_t  vzMmS;
  uint32_t accuracyMmS;    // Speed accuracy (1 sigma)
};

// $PQTMPVTECEF (version 1): position, velocity and time in ECEF
struct LC76G_EcefPvt {
  uint32_t towMs;          // GPS time of week, ms
  uint16_t year;           // UTC date and time
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
  uint16_t millisecond;
  int8_t   leapSeconds;    // LC76G_LEAP_UNKNOWN if empty
  bool     valid;          // false: position / velocity left empty
  int32_t  xCm;            // Position, cm
  int32_t  yCm;
  int32_t  zCm;
  uint32_t posAccuracyCm;
  int32_t  vxMmS;          // Velocity, mm/s
  int32_t  vyMmS;
  int32_t  vzMmS;
  uint32_t velAccuracyMmS;
};

// Callbacks for the messages above
typedef void (*LC76G_PvtFn)(const LC76G_Pvt &pvt);
typedef void (*LC76G_VelocityFn)(const LC76G_Velocity &vel);
typedef void (*LC76G_DopFn)(const LC76G_Dop &dop);
typedef void (*LC76G_OdometerFn)(const LC76G_Odometer &odo);
typedef void (*LC76G_GpsTimeFn)(const LC76G_GpsTime &t);
typedef void (*LC76G_ProtectionLevelFn)(const LC76G_ProtectionLevel &pl);
typedef void (*LC76G_EcefPositionFn)(const LC76G_EcefPosition &pos);
typedef void (*LC76G_EcefVelocityFn)(const LC76G_EcefVelocity &vel);
typedef void (*LC76G_EcefPvtFn)(const LC76G_EcefPvt &pvt);

// Maximum number of output messages decoded at the same time
// (each slot costs 4 bytes of RAM on the Uno R3)
#ifndef LC76G_OUTPUT_HOOKS
#define LC76G_OUTPUT_HOOKS 6
#endif

// Function that changes the HOST serial port baud rate.
// The library cannot do this through a generic Stream, so the sketch
// provides it, typically as a lambda:
//     [](uint32_t b) { gnssSerial.begin(b); }
typedef void (*LC76G_BaudFn)(uint32_t baud);

// Function called for every valid sentence (body without '$' and CR/LF)
typedef void (*LC76G_SentenceFn)(const char *body);


class LC76G {
public:
  LC76G();

  // -------------------------------------------------------------------------
  // Decoded navigation data (position, time, satellites...)
  // -------------------------------------------------------------------------
  LC76G_Nmea nmea;

  // -------------------------------------------------------------------------
  // Hardware configuration (call BEFORE begin())
  // -------------------------------------------------------------------------

  // Pin that switches the module supply (VCC). 'asserted' = module ON.
  void setPowerPin(int8_t pin, LC76G_DriveMode mode);

  // Pin that controls the module reset. 'asserted' = module IN RESET.
  void setResetPin(int8_t pin, LC76G_DriveMode mode);

  // -------------------------------------------------------------------------
  // Start-up
  // -------------------------------------------------------------------------

  // Powers the module (if a power pin is set), waits for boot, then makes
  // sure the module UART runs at 'targetBaud', reconfiguring it from the
  // factory default if needed. 'setHostBaud' may be nullptr if the port is
  // already open at 'targetBaud' and no reconfiguration is wanted.
  // Returns true when valid NMEA is received at 'targetBaud'.
  bool begin(Stream &port, LC76G_BaudFn setHostBaud, uint32_t targetBaud = 38400);

  // Must be called as often as possible from loop(): reads the UART,
  // validates sentences, updates 'nmea' and handles command replies.
  void update();

  // -------------------------------------------------------------------------
  // Power and reset
  // -------------------------------------------------------------------------
  void powerOn();                 // Assert the power pin (no-op if not set)
  void powerOff();                // Release the power pin (no-op if not set)
  void resetPulse();              // Short reset pulse (no-op if not set)

  // Reboots the module using the reset pin if available, otherwise by
  // power-cycling. Returns false if neither pin is configured.
  bool reboot();

  // -------------------------------------------------------------------------
  // Commands
  // -------------------------------------------------------------------------

  // Sends "$<body>*<checksum>\r\n". 'body' excludes '$' and checksum.
  bool sendRaw(const char *body);

  // Sends $PAIR<id>[,<params>] and waits for $PAIR001,<id>,<result>.
  // If 'resp' is given, also waits for the query result sentence
  // ($PAIR<id>,...) and copies its body into 'resp'.
  LC76G_Result sendPair(uint16_t id,
                        const char *params = nullptr,
                        uint16_t timeoutMs = LC76G_CMD_TIMEOUT_MS,
                        char *resp = nullptr,
                        uint8_t respSize = 0);

  // Sends $<name>[,<params>] (PQTM proprietary command) and waits for
  // $<name>,OK or $<name>,ERROR,<code>. Error codes are mapped to
  // LC76G_PARAM_ERROR (1), LC76G_FAILED (2), LC76G_NOT_SUPPORTED (3).
  // If 'resp' is given, the body of the OK reply is copied into it
  // (used by the "R" = read variants of the commands).
  LC76G_Result sendPqtm(const char *name,
                        const char *params = nullptr,
                        uint16_t timeoutMs = LC76G_CMD_TIMEOUT_MS,
                        char *resp = nullptr,
                        uint8_t respSize = 0);

  // Output rate of a PQTM message (PQTMCFGMSGRATE), e.g.
  // setPqtmMessageRate("PQTMEPE", 1, 2). 'msgVer' is the message version
  // listed in the protocol specification for that message.
  LC76G_Result setPqtmMessageRate(const char *msgName, uint8_t rate, uint8_t msgVer);

  // Reads the output rate of a PQTM message (PQTMCFGMSGRATE,R). 'rate'
  // is 0 when the message is off. LC76G_NOT_SUPPORTED (or
  // LC76G_PARAM_ERROR) means the firmware does not know the message:
  // useful to check which PQTM messages a module supports.
  LC76G_Result getPqtmMessageRate(const char *msgName, uint8_t msgVer, uint8_t &rate);

  // Removes the internal pull-up that SoftwareSerial enables on its RX pin
  // (both the AVR and the Renesas versions do it). With a 3.3 V module
  // wired directly to a 5 V host, that pull-up pushes current from the
  // host 5 V rail into the module TXD output, and back-powers the module
  // while it is off. Call it right after every gnssSerial.begin(), e.g.:
  //   [](uint32_t b) { gnssSerial.begin(b); LC76G::disableRxPullup(4); }
  static void disableRxPullup(uint8_t rxPin);

  // Number of automatic retries when a command gets no answer (default 1).
  // Right after start-up the module can ignore commands for a few seconds
  // while it is busy; a retry makes the library robust to it.
  void setCommandRetries(uint8_t retries) { _retries = retries; }

  // Ready-made commands
  LC76G_Result hotStart()  { return sendPair(4); }  // PAIR004: use all stored data
  LC76G_Result warmStart() { return sendPair(5); }  // PAIR005: discard ephemeris
  LC76G_Result coldStart() { return sendPair(6); }  // PAIR006: discard everything
  LC76G_Result queryBaudRate(uint32_t &baud);       // PAIR865

  // Changes the module UART baud rate (PAIR864), reboots the module and
  // reopens the host port at the new speed.
  LC76G_Result setBaudRate(uint32_t baud);

  // -------------------------------------------------------------------------
  // NMEA output configuration (implemented in LC76G_Config.cpp)
  // -------------------------------------------------------------------------

  // Sets how often a sentence type is output (PAIR062):
  //   0 = disabled, N = once every N position fixes (1..20).
  // WARNING: the decoder needs RMC (end-of-epoch marker, position, time)
  // and GGA (satellites used, altitude, HDOP). GSV is needed only for the
  // satellites-in-view statistics.
  LC76G_Result setNmeaRate(LC76G_NmeaType type, uint8_t rate);

  // Reads the output rate of a sentence type (PAIR063)
  LC76G_Result getNmeaRate(LC76G_NmeaType type, uint8_t &rate);

  // Keeps only the sentences used by the decoder (GGA, GSV, RMC) and
  // disables GLL, GSA, VTG: about 30% less UART traffic with a fix.
  // 'gsvRate' lets GSV, the longest sentence group, be sent less often.
  LC76G_Result setMinimalNmeaOutput(uint8_t gsvRate = 1);

  // Saves the current configuration to the module flash (PAIR513).
  // Needed only if the backup supply (V_BCKP) is NOT kept powered: with
  // V_BCKP alive, settings already survive power cycles in RTC RAM.
  // Note: the baud rate (PAIR864) is stored separately and needs no save.
  LC76G_Result saveSettings();

  // -------------------------------------------------------------------------
  // Constellation selection (implemented in LC76G_Config.cpp)
  // -------------------------------------------------------------------------

  // Selects the satellite systems to search (PAIR066, Set GNSS Search Mode).
  // 'mask' is a combination of LC76G_GNSS_xxx. Only the combinations listed
  // by Quectel for the LC76G are accepted (see isSupportedConstellationSet);
  // others return LC76G_PARAM_ERROR without contacting the module.
  // If the set changes, the module reboots by itself: this function waits
  // for the reboot to complete before returning.
  LC76G_Result setConstellations(uint8_t mask);

  // Reads the systems currently searched (PAIR067, Get GNSS Search Mode)
  LC76G_Result getConstellations(uint8_t &mask);

  // true if 'mask' is one of the combinations supported by the LC76G
  static bool isSupportedConstellationSet(uint8_t mask);

  // -------------------------------------------------------------------------
  // Readable names, stored in flash (implemented in LC76G_Text.cpp)
  // -------------------------------------------------------------------------

  // e.g. LC76G_TIMEOUT -> "timeout"
  static const __FlashStringHelper *resultText(LC76G_Result r);

  // e.g. LC76G_NMEA_ZDA -> "Time and Date (ZDA)"
  static const __FlashStringHelper *nmeaTypeName(LC76G_NmeaType type);

  // e.g. LC76G_GNSS_BDS -> "BeiDou Navigation Satellite System (BDS)".
  // 'bit' must be a single LC76G_GNSS_xxx flag.
  static const __FlashStringHelper *constellationName(uint8_t bit);

  // e.g. LC76G_JAM_WARNING -> "warning (interference detected)"
  static const __FlashStringHelper *jamStatusText(LC76G_JamStatus s);

  // -------------------------------------------------------------------------
  // Jamming detection (implemented in LC76G_Jamming.cpp)
  // -------------------------------------------------------------------------

  // Enables/disables jamming detection (PAIR391, Test Jamming Detect).
  // When enabled the module periodically sends $PAIRSPF,<status>, decoded
  // automatically into jammingStatus(). The protocol does not state that
  // this setting is stored: send it again after every power-up or reboot.
  LC76G_Result setJammingDetection(bool enable);

  // Last status received ($PAIRSPF or $PQTMJAMMINGSTATUS)
  LC76G_JamStatus jammingStatus() const { return _jamStatus; }

  // millis() timestamp of the last status message (0 = none yet)
  unsigned long jammingStatusMs() const { return _jamStatusMs; }

  // Optional: Arduino pin wired to the module JAM_IND output.
  // The pin is configured as a plain INPUT: NEVER enable the internal
  // pull-up, it would push 5 V into a 3.3 V module output.
  void setJamIndicatorPin(int8_t pin);

  // Reads JAM_IND: the module drives it LOW while jamming is detected.
  // Returns false if no pin is configured.
  bool jamIndicatorActive() const;

  // -------------------------------------------------------------------------
  // Geofence (implemented in LC76G_Geofence.cpp)
  //   Up to 4 areas (index 0..3): circles or polygons (3 or 4 corners).
  //   Coordinates in degrees * 1e7, the same unit as LC76G_Fix, so the
  //   current position can be used directly as a centre.
  //   The module keeps the geofences in RTC RAM (while V_BCKP is powered).
  //   To store them in flash, send sendPqtm("PQTMSAVEPAR").
  // -------------------------------------------------------------------------

  // Circle with centre and radius in metres (PQTMCFGGEOFENCE, shape 0)
  LC76G_Result setGeofenceCircle(uint8_t index, int32_t latE7, int32_t lonE7, uint32_t radiusM);

  // Triangle (count = 3) or quadrangle (count = 4). Corners must be given in
  // clockwise or counter-clockwise order (shape 2 / 3).
  LC76G_Result setGeofencePolygon(uint8_t index, const int32_t latE7[], const int32_t lonE7[],
                                  uint8_t count);

  // Disables one geofence (its shape is kept in the module)
  LC76G_Result disableGeofence(uint8_t index);

  // Reads whether a geofence is enabled
  LC76G_Result getGeofenceEnabled(uint8_t index, bool &enabled);

  // Enables (rate 1 = every fix) or disables (0) the $PQTMGEOFENCESTATUS
  // message, decoded automatically into geofenceState()
  LC76G_Result setGeofenceStatusOutput(uint8_t rate);

  // Last state of a geofence from $PQTMGEOFENCESTATUS
  LC76G_GeofenceState geofenceState(uint8_t index) const;

  // millis() timestamp of the last status message (0 = none yet)
  unsigned long geofenceStatusMs() const { return _geoStatusMs; }

  // Optional: Arduino pin wired to the module GEOFENCE output
  // (HIGH = inside the active area(s)). Configured as a plain INPUT:
  // the module must not see a pull-up in the first 50 ms after power-on.
  void setGeofencePin(int8_t pin);

  // true if the GEOFENCE pin is HIGH (false if no pin is configured)
  bool geofencePinInside() const;

  // e.g. LC76G_GEO_OUTSIDE -> "outside"
  static const __FlashStringHelper *geofenceStateText(LC76G_GeofenceState s);

  // -------------------------------------------------------------------------
  // 1PPS (implemented in LC76G_Pps.cpp)
  //   The pulse rising edge (or falling edge with negative polarity) is
  //   aligned to the start of each UTC second once the module has a fix.
  // -------------------------------------------------------------------------

  // Configures the 1PPS output (PQTMCFGPPS, Configure PPS)
  LC76G_Result setPps(bool enable,
                      uint16_t durationMs = 100,
                      LC76G_PpsMode mode = LC76G_PPS_3D_FIX,
                      bool activeHigh = true);

  // Reads the 1PPS configuration
  LC76G_Result getPps(LC76G_PpsConfig &cfg);

  // e.g. LC76G_PPS_ALWAYS -> "always (even without fix)"
  static const __FlashStringHelper *ppsModeText(LC76G_PpsMode m);

  // Optional measurement of the pulses on an Arduino interrupt pin
  // (D2 or D3 on the Uno R3). Returns false if the pin cannot generate
  // interrupts. The pin is a plain INPUT (3.3 V module output).
  // Only one LC76G object can use the capture at a time.
  // Timing resolution: 4 us on the Uno R3. The edge time can be delayed by
  // up to ~0.3 ms while SoftwareSerial is receiving a byte, because
  // SoftwareSerial blocks interrupts during each byte.
  bool beginPpsCapture(int8_t pin);

  uint32_t ppsCount() const;       // Pulses counted since beginPpsCapture()
  uint32_t ppsPeriodUs() const;    // Time between the last two pulses (us)
  uint32_t ppsWidthUs() const;     // Length of the last pulse (us)
  uint32_t ppsAgeMs() const;       // Time since the last pulse (ms), 0xFFFFFFFF = none

  // -------------------------------------------------------------------------
  // Low power (implemented in LC76G_LowPower.cpp)
  //
  //   Typical current of the LC76G (PA), all constellations (datasheet):
  //     Continuous (acquisition / tracking)  ~9.7 mA
  //     ALP mode 2 (performance)             ~7.5 mA
  //     ALP mode 1 (power)                   ~3.8 mA
  //     Backup                               ~13 uA (V_BCKP only)
  //   Periodic, GLP and FLP modes reduce the average current by cycling or
  //   by duty-cycling the receiver; the saving depends on the settings.
  //
  //   Requirements (protocol specification):
  //     ALP     : navigation mode Normal, 1 Hz fix rate
  //     GLP     : navigation mode Fitness, GPS only or GPS + QZSS
  //     FLP     : navigation mode Fitness, see protocol for constellations
  //     Enabling one of ALP / GLP / FLP / Periodic disables the others.
  // -------------------------------------------------------------------------

  // Navigation mode (PAIR080 / PAIR081)
  LC76G_Result setNavigationMode(LC76G_NavMode mode);
  LC76G_Result getNavigationMode(LC76G_NavMode &mode);

  // ALP, Adaptive Low Power (PAIR732 / PAIR733): 0 = off, 1 = mode 1
  // (lowest power), 2 = mode 2 (better performance)
  LC76G_Result setAlpMode(uint8_t alpMode);
  LC76G_Result getAlpMode(uint8_t &alpMode);

  // GLP, GPS Low Power (PAIR680 / PAIR681)
  LC76G_Result setGlpMode(bool enable);
  LC76G_Result getGlpMode(bool &enabled);

  // FLP, Fitness Low Power (PAIR730 / PAIR731)
  LC76G_Result setFlpMode(bool enable);
  LC76G_Result getFlpMode(bool &enabled);

  // Periodic power saving (PAIR690 / PAIR691).
  // WARNING: during the sleep stage the module does not receive commands
  // and sends no NMEA. Use disablePeriodicMode() to leave it reliably.
  LC76G_Result setPeriodicMode(const LC76G_PeriodicConfig &cfg);
  LC76G_Result getPeriodicMode(LC76G_PeriodicConfig &cfg);

  // Sends "periodic off" repeatedly until the module, once awake,
  // acknowledges it, or until 'timeoutMs' expires.
  LC76G_Result disablePeriodicMode(uint32_t timeoutMs = 120000UL);

  // Backup mode (PAIR650): only the backup domain (RTC, satellite data)
  // stays alive on V_BCKP.
  //   seconds = 0 : no timer. If a power pin is configured, VCC is cut
  //                 right after the command (procedure in the datasheet).
  //   seconds = 10..62208000 : the module wakes up by itself after that
  //                 time (VCC is left on).
  // The module cannot receive commands while in Backup mode.
  LC76G_Result enterBackup(uint32_t seconds = 0);

  // Leaves Backup mode: restores VCC (power pin), pulses RESET_N for
  // 100 ms (reset pin) and waits for the first NMEA sentence.
  LC76G_Result exitBackup();

  // -------------------------------------------------------------------------
  // AGNSS, Assisted GNSS (implemented in LC76G_Agnss.cpp)
  //
  //   Self-based assistance (no internet needed): the module predicts the
  //   satellite orbits for up to 3 days from the broadcast ephemeris, so
  //   later starts are faster. Two alternatives, never both:
  //     EASY (Embedded Assist System)       - enabled by default, GPS only
  //     EPOC (Enhanced Prediction Orbit on Chip) - GPS, GPS+Galileo, GPS+BDS
  //   A given firmware supports only one of them: the other one answers
  //   LC76G_NOT_SUPPORTED or LC76G_FAILED.
  //   Predictions live in RTC RAM (kept by V_BCKP); saveNavigationData()
  //   copies them to flash for boards without backup supply.
  //
  //   Host assistance: reference time (within 3 s) and position (within
  //   30 km) sent by the host after each reboot.
  //
  //   Assistance shortens the time to first fix but CANNOT compensate for
  //   a signal that is too weak (Quectel AGNSS Application Note).
  //   Network EPO files (download from the internet) are out of scope.
  // -------------------------------------------------------------------------

  // Firmware version string (PQTMVERNO), e.g. "LC76GPANR12A01S"
  LC76G_Result getFirmwareVersion(char *buf, uint8_t size);

  // EASY (PAIR490 / PAIR491). 'extensionDays' = days of prediction already
  // computed: 0 = not finished, 1..3 = days ready.
  LC76G_Result setEasy(bool enable);
  LC76G_Result getEasy(bool &enabled, uint8_t &extensionDays);

  // EPOC (PAIR496 / PAIR498 / PAIR508 / PAIR507 / PAIR509)
  LC76G_Result setEpoc(bool enable);
  LC76G_Result setEpocConstellations(uint8_t mask);          // LC76G_EPOC_xxx
  LC76G_Result getEpoc(bool &enabled, uint8_t &mask);
  LC76G_Result clearEpocData();
  // Prediction status of ONE constellation bit (PAIR509):
  //   status 1 = prediction in progress
  //          2 = all received ephemeris data have been used for prediction
  //          3 = no valid ephemeris data
  //   satsReady = satellites whose 3-day prediction is complete
  LC76G_Result getEpocPredictionStatus(uint8_t constellationBit, int32_t &status,
                                       uint8_t &satsReady);

  // e.g. 1 -> "prediction in progress"
  static const __FlashStringHelper *epocStatusText(int32_t status);

  // Navigation data (ephemeris, predictions) RTC RAM <-> flash
  LC76G_Result saveNavigationData();                         // PAIR511
  LC76G_Result clearNavigationData();                        // PAIR512: forces a cold start

  // Reference UTC time (PAIR590): accurate within 3 s, UTC not local time.
  // Must be sent again after every module reboot.
  LC76G_Result setReferenceTime(uint16_t year, uint8_t month, uint8_t day,
                                uint8_t hour, uint8_t minute, uint8_t second);

  // Reference position (PAIR600): within 30 km of the real position.
  // Coordinates in degrees * 1e7, accuracies in metres.
  LC76G_Result setReferencePosition(int32_t latE7, int32_t lonE7, int32_t heightM,
                                    uint32_t accMajorM = 30000, uint32_t accMinorM = 30000,
                                    uint16_t bearingDeg = 0, uint32_t accVertM = 1000);

  // -------------------------------------------------------------------------
  // LOCUS logger (implemented in LC76G_Locus.cpp)
  //
  //   The module stores UTC time, fix status, latitude, longitude,
  //   altitude, speed, heading, HDOP and satellites used in 128 KB of
  //   internal flash, even while the host is off. Settings are stored in
  //   flash automatically.
  //
  //   WARNING (protocol specification): LOCUS must be stopped before the
  //   module is powered off or restarted, otherwise the data recorded
  //   before is LOST. The library therefore stops LOCUS automatically in
  //   reboot(), powerOff(), enterBackup() and setConstellations();
  //   reboot() and setConstellations() restart it afterwards.
  //   An uncontrolled loss of the module supply (e.g. a board that cuts
  //   it when the HOST resets) cannot be handled: the recording is lost.
  // -------------------------------------------------------------------------

  // Starts / stops recording (PAIR900 / PAIR901)
  LC76G_Result setLocusEnabled(bool enable);
  LC76G_Result getLocusEnabled(bool &enabled);

  // Save mode: combination of LC76G_LOCUS_xxx bits; 'require3dFix' = record
  // only positions with a 3D fix (PAIR902 / PAIR903). Recording is paused
  // automatically while the mode is changed, as the protocol requires.
  LC76G_Result setLocusMode(uint8_t modeBits, bool require3dFix);
  LC76G_Result getLocusMode(uint8_t &modeBits, bool &require3dFix);

  // Threshold of the time / speed / distance modes (PAIR904 / PAIR905).
  // Set the matching mode bit with setLocusMode() first.
  LC76G_Result setLocusThreshold(LC76G_LocusTrigger trigger, uint16_t value);
  LC76G_Result getLocusThreshold(LC76G_LocusTrigger trigger, uint16_t &value);

  LC76G_Result clearLocus(LC76G_LocusClear what);      // PAIR906
  LC76G_Result logLocusNow();                          // PAIR907 (needs LC76G_LOCUS_USER)
  LC76G_Result getLocusRecordCount(uint32_t &count);   // PAIR909

  // Reads all records in the COMPACT format (PAIR908 type 1): one line per
  // record, half the data of the NMEA format. During the read-back the
  // library:
  //   - stops recording (required by the protocol),
  //   - silences the live NMEA output and restores it at the end, so the
  //     UART carries only LOCUS data,
  //   - checks the format and length of every record and discards
  //     damaged ones ('stats.rejected').
  // Complete read-back: stats.received == stats.announced.
  // Records are never fed to the live decoder.
  // LIMITATION: the module streams the records without pauses. With
  // SoftwareSerial (Uno R3) the host cannot keep up and part of the
  // records is lost; a hardware UART is required for a reliable read-back.
  LC76G_Result dumpLocus(LC76G_LocusRecordFn cb, LC76G_LocusDumpStats &stats,
                         uint32_t idleTimeoutMs = 5000);

  // Same, in NMEA format (PAIR908 type 0): two sentences per record,
  // $LOGGA (GGA format) and $LORMC (RMC format), passed to 'cb' as bodies
  // without '$' and with "*checksum". Complete read-back:
  // stats.received == 2 * stats.announced.
  LC76G_Result dumpLocusNmea(LC76G_SentenceFn cb, LC76G_LocusDumpStats &stats,
                             uint32_t idleTimeoutMs = 5000);

  // Converts Unix time (as in LC76G_LocusRecord::utc) to a UTC date/time
  static void unixToUtc(uint32_t t, uint16_t &year, uint8_t &month, uint8_t &day,
                        uint8_t &hour, uint8_t &minute, uint8_t &second);

  // -------------------------------------------------------------------------
  // Reception settings (implemented in LC76G_Reception.cpp)
  // -------------------------------------------------------------------------

  // Elevation mask (PAIR072 / PAIR073): satellites lower than this angle
  // above the horizon are not used. -90..90 degrees, default 5. A higher
  // mask rejects low satellites, whose signals cross more atmosphere and
  // are more often reflected.
  LC76G_Result setElevationMask(int8_t degrees);
  LC76G_Result getElevationMask(int8_t &degrees);

  // AIC, Active Interference Cancellation (PAIR074 / PAIR075): filters
  // narrow-band interference out of the GNSS band
  LC76G_Result setAic(bool enable);
  LC76G_Result getAic(bool &enabled);

  // SBAS, Satellite-Based Augmentation System (PAIR410 / PAIR411):
  // search the geostationary correction satellites (EGNOS in Europe).
  // Not available in fitness / swimming navigation and in GLP, FLP and
  // ALP mode 1. To APPLY the corrections also select LC76G_DGPS_SBAS with
  // setDgpsSource(). A corrected fix shows quality 2 (DGPS) in GGA.
  LC76G_Result setSbas(bool enable);
  LC76G_Result getSbas(bool &enabled);

  // DGPS correction source (PAIR400 / PAIR401)
  LC76G_Result setDgpsSource(LC76G_DgpsSource source);
  LC76G_Result getDgpsSource(LC76G_DgpsSource &source);

  // 2D fix (PAIR162 / PAIR163): allow a position with only 3 satellites
  // (latitude/longitude without reliable altitude)
  LC76G_Result set2dFix(bool enable);
  LC76G_Result get2dFix(bool &enabled);

  // Real-time speed response (PAIR160 / PAIR161): speed follows changes
  // immediately, with less filtering
  LC76G_Result setImmediateSpeed(bool enable);
  LC76G_Result getImmediateSpeed(bool &enabled);

  // Tracking of the BDS B1C signal band (PAIR158 / PAIR159)
  LC76G_Result setBdsB1c(bool enable);
  LC76G_Result getBdsB1c(bool &enabled);

  // Static navigation threshold (PAIR070 / PAIR071): below this speed the
  // position is frozen and the speed reported as 0. Unit: decimetres per
  // second (dm/s), 0..20, 0 = disabled.
  LC76G_Result setStaticThreshold(uint8_t dmPerS);
  LC76G_Result getStaticThreshold(uint8_t &dmPerS);

  // -------------------------------------------------------------------------
  // Output settings (implemented in LC76G_Settings.cpp)
  // Like the other settings they live in the module RAM: they are lost at
  // the next power-on unless saved with saveParameters().
  // -------------------------------------------------------------------------

  // Minimum signal-to-noise ratio of the satellites used in the position
  // (PAIR058 / PAIR059): weaker satellites are ignored. 9..37 dB-Hz,
  // factory value 9. Higher values reject reflected / weak signals, but
  // need a good sky view.
  LC76G_Result setMinSnr(uint8_t dbHz);
  LC76G_Result getMinSnr(uint8_t &dbHz);

  // Decimal places of the standard NMEA sentences (PQTMCFGNMEADP).
  // The library decoder accepts any number of decimals.
  LC76G_Result setNmeaDecimals(const LC76G_NmeaDecimals &d);
  LC76G_Result getNmeaDecimals(LC76G_NmeaDecimals &d);

  // Talker ID of the standard NMEA sentences (PQTMCFGNMEATID):
  //   mainId: "00" = automatic (factory: GN, GP, ...), or any two printable
  //           characters, e.g. "GP" (not starting with 'P': reserved to
  //           proprietary sentences, the decoder would ignore them)
  //   gsvSameAsMain: true = GSV uses mainId too. Leave it false: the
  //           library tells the constellations apart by the GSV talker ID
  //           (GP, GL, GA, GB), so satsInView() per system needs it.
  LC76G_Result setNmeaTalkerId(const char *mainId, bool gsvSameAsMain = false);
  // 'mainId' must have room for 3 characters ("GP" + terminator)
  LC76G_Result getNmeaTalkerId(char *mainId, bool &gsvSameAsMain);

  // RTCM 3 output configuration (PAIR432...PAIR437). The data is binary,
  // on the same UART: use a hardware UART and a high baud rate (MSM7 with
  // many satellites can exceed what 38400 baud carries). The library
  // skips it and does not decode it; use onSentence() for NMEA only.
  LC76G_Result setRtcmMode(LC76G_RtcmMode mode);       // PAIR432
  LC76G_Result getRtcmMode(LC76G_RtcmMode &mode);      // PAIR433
  LC76G_Result setRtcmAntennaPoint(bool enable);       // PAIR434: message 1005
  LC76G_Result getRtcmAntennaPoint(bool &enabled);     // PAIR435
  LC76G_Result setRtcmEphemeris(bool enable);          // PAIR436: 1019, 1020, 1042, 1044, 1046
  LC76G_Result getRtcmEphemeris(bool &enabled);        // PAIR437

  // Galileo Return Link Message output, $GARLM (PAIR154 / PAIR155): the
  // acknowledgement of a Galileo search-and-rescue beacon. The sentence
  // is not decoded by the library (use onSentence()).
  LC76G_Result setRlmOutput(bool enable);
  LC76G_Result getRlmOutput(bool &enabled);

  // Fixes the delay between the 1PPS pulse and the NMEA sentences of the
  // same second to 350 ms (PAIR751). No read-back command exists.
  LC76G_Result setPpsNmeaSync(bool enable);

  // -------------------------------------------------------------------------
  // Estimated positioning error and parameter management
  // (implemented in LC76G_Params.cpp)
  // -------------------------------------------------------------------------

  // Enables (rate N = every N fixes, 1..20) or disables (0) the $PQTMEPE
  // message: the module estimates the error of its own position, in
  // metres, north / east / down / horizontal / 3D. Easier to read than
  // HDOP, which only describes the satellite geometry.
  LC76G_Result setEpeOutput(uint8_t rate);

  // Last estimate received (see epeAgeMs() to know how recent it is)
  const LC76G_Epe &epe() const { return _epe; }

  // Milliseconds since the last $PQTMEPE (0xFFFFFFFF = none received yet)
  uint32_t epeAgeMs() const;

  // -------------------------------------------------------------------------
  // PQTM output messages (implemented in LC76G_Output.cpp)
  //   Two steps for each message:
  //   1) set...Output(rate): the module sends it every 'rate' fixes
  //      (1..20, 0 = off). Not saved in flash: call it after every begin().
  //   2) on...(callback): the library decodes it and calls the callback.
  //      Up to LC76G_OUTPUT_HOOKS messages at the same time; nullptr
  //      removes the callback. Returns false if the table is full.
  //   Keep callbacks short (store the values, print them later): long
  //   prints inside a callback can make SoftwareSerial lose characters.
  // -------------------------------------------------------------------------
  LC76G_Result setPvtOutput(uint8_t rate);        // $PQTMPVT
  LC76G_Result setVelocityOutput(uint8_t rate);   // $PQTMVEL
  LC76G_Result setDopOutput(uint8_t rate);        // $PQTMDOP
  LC76G_Result setOdometerOutput(uint8_t rate);   // $PQTMODO
  LC76G_Result setGpsTimeOutput(uint8_t rate);    // $PQTMTIMEGPS
  LC76G_Result setProtectionLevelOutput(uint8_t rate); // $PQTMPL
  LC76G_Result setEcefPositionOutput(uint8_t rate);    // $PQTMPOSECEF
  LC76G_Result setEcefVelocityOutput(uint8_t rate);    // $PQTMVELECEF
  LC76G_Result setEcefPvtOutput(uint8_t rate);         // $PQTMPVTECEF
  bool onPvt(LC76G_PvtFn cb);
  bool onVelocity(LC76G_VelocityFn cb);
  bool onDop(LC76G_DopFn cb);
  bool onOdometer(LC76G_OdometerFn cb);
  bool onGpsTime(LC76G_GpsTimeFn cb);
  bool onProtectionLevel(LC76G_ProtectionLevelFn cb);
  bool onEcefPosition(LC76G_EcefPositionFn cb);
  bool onEcefVelocity(LC76G_EcefVelocityFn cb);
  bool onEcefPvt(LC76G_EcefPvtFn cb);

  // Odometer (PQTMCFGODO / PQTMRESETODO). The module adds up the distance
  // travelled; $PQTMODO reports it (see setOdometerOutput()).
  //   initialDm: starting value in decimetres (distance = travelled + initial)
  // The travelled distance restarts from 0 at every power-off of the
  // module, or with resetOdometer(). Disabling stops counting but does
  // not reset the value.
  LC76G_Result setOdometer(bool enable, uint32_t initialDm = 0);
  LC76G_Result getOdometer(bool &enabled, uint32_t &initialDm);
  LC76G_Result resetOdometer();

  // Restores ALL parameters to the factory defaults (PQTMRESTOREPAR),
  // then reboots the module so the defaults take effect. Without a power
  // or reset pin it returns LC76G_NO_REBOOT: the defaults are stored but
  // apply only after a manual power cycle. NOT restored, by design of the module: UART
  // baud rate (PAIR864) and LOCUS settings (PAIR900 / 902 / 904), so the
  // link keeps working after the restore.
  LC76G_Result restoreDefaults();

  // Saves the current configuration to flash (PQTMSAVEPAR). Not covered:
  // baud rate and LOCUS settings, which are stored automatically.
  // See also saveSettings() (PAIR513).
  LC76G_Result saveParameters();

  // e.g. LC76G_RTCM_MSM4 -> "MSM4"
  static const __FlashStringHelper *rtcmModeText(LC76G_RtcmMode m);

  // e.g. LC76G_DGPS_SBAS -> "SBAS (EGNOS, WAAS, GAGAN, MSAS)"
  static const __FlashStringHelper *dgpsSourceText(LC76G_DgpsSource s);

  // e.g. 2 -> "DGPS (corrected)", from the GGA fix quality
  static const __FlashStringHelper *fixQualityText(uint8_t quality);

  // e.g. LC76G_NAV_FITNESS -> "fitness (walking, running)"
  static const __FlashStringHelper *navModeText(LC76G_NavMode m);

  // e.g. LC76G_PERIODIC_SMART -> "smart"
  static const __FlashStringHelper *periodicModeText(LC76G_PeriodicMode m);

  // -------------------------------------------------------------------------
  // Status
  // -------------------------------------------------------------------------

  // true if a valid sentence arrived within the last 'silenceMs'
  bool isTalking(uint32_t silenceMs = 3000) const;

  // millis() timestamp of the last valid sentence
  unsigned long lastSentenceMs() const { return _lastSentenceMs; }

  // Baud rate currently used on the host side
  uint32_t baud() const { return _baud; }

  // Registers a function called for every valid sentence (raw logging,
  // custom parsing). nullptr to disable.
  // Like every callback of the library (onEpoch, onPvt, ...), it runs
  // inside update(): it must NOT send commands (they would disturb the
  // command being processed). Set a flag and send them from loop().
  void onSentence(LC76G_SentenceFn cb) { _sentenceCb = cb; }

  // true if 'body' (text between '$' and CR/LF, e.g. "GNRMC,...*2C") has
  // a correct XOR checksum
  static bool checksumOk(const char *body);

private:
  // Pin helpers
  static void drivePin(int8_t pin, LC76G_DriveMode mode, bool asserted);

  // Byte -> sentence assembler; returns true when _line holds a valid sentence
  bool assemble(char c);

  // Routes a valid sentence to command handling, NMEA decoder, user callback
  void handleSentence(const char *body);
  void routeSentence(const char *body);     // Library part of handleSentence()

  // Single attempt of sendPair() / sendPqtm() (no retry)
  LC76G_Result sendPairOnce(uint16_t id, const char *params, uint16_t timeoutMs,
                            char *resp, uint8_t respSize);
  LC76G_Result sendPqtmOnce(const char *name, const char *params, uint16_t timeoutMs,
                            char *resp, uint8_t respSize);

  // After start-up: waits until the module answers commands, not only
  // sends NMEA (it may need a few more seconds). Returns false on timeout.
  bool waitCommandReady(uint32_t timeoutMs);

  // Opens the host port at 'baud' and waits for one valid sentence
  bool detectAt(uint32_t baud, uint16_t timeoutMs);
  void setHostBaud(uint32_t baud);

  // Link
  Stream          *_port;
  LC76G_BaudFn     _setHostBaud;
  uint32_t         _baud;

  // Control pins
  int8_t           _pwrPin;
  LC76G_DriveMode  _pwrMode;
  int8_t           _rstPin;
  LC76G_DriveMode  _rstMode;

  // Sentence assembler
  char             _line[LC76G_LINE_MAX];
  uint8_t          _lineLen;
  bool             _inLine;
  unsigned long    _lastSentenceMs;

  // Pending PAIR command
  int16_t          _waitId;       // Command ID awaited, -1 = none
  int8_t           _waitResult;   // Last $PAIR001 result for _waitId
  char            *_respBuf;      // Where to copy the query result
  uint8_t          _respSize;

  LC76G_SentenceFn _sentenceCb;
  uint8_t          _retries;        // Automatic retries on timeout

  // Jamming detection
  LC76G_JamStatus  _jamStatus;
  unsigned long    _jamStatusMs;
  int8_t           _jamPin;

  // Estimated positioning error
  LC76G_Epe        _epe;
  unsigned long    _epeMs;          // millis() of the last $PQTMEPE, 0 = none

  // Pending PQTM command (nullptr = none)
  const char      *_waitPqtm;

public:
  // Output message hooks. Public only so the decoders in
  // LC76G_Output.cpp can be plain functions; not meant for sketches.
  // A decoder returns true if 'body' was its message (and then calls
  // the user callback, stored as a generic function pointer).
  typedef void (*GenericFn)();
  typedef bool (*HookDecoder)(const char *body, GenericFn cb);

private:
  struct OutputHook {
    HookDecoder decode;    // nullptr = free slot
    GenericFn   cb;        // User callback, cast back by the decoder
  };
  OutputHook       _hooks[LC76G_OUTPUT_HOOKS];

  // Adds, replaces (same decoder) or removes (cb == nullptr) a hook.
  // Only the decoders actually registered end up in the program, so
  // unused messages cost no flash on the Uno R3.
  bool setHook(HookDecoder decode, GenericFn cb);

  // Geofence
  uint8_t          _geoState[LC76G_GEOFENCE_COUNT];
  unsigned long    _geoStatusMs;
  int8_t           _geoPin;

  // LOCUS state
  bool             _locusActive;    // Last known recording state
  bool             _locusDumping;   // A read-back is in progress
  LC76G_SentenceFn _locusCb;        // NMEA read-back callback
  LC76G_LocusRecordFn _locusRecCb;  // Compact read-back callback
  bool             _locusDumpDone;  // End marker ($PAIR908,3) received
  uint32_t         _locusRecords;   // Records announced by $PAIR908,1
  uint32_t         _locusReceived;  // Valid items received
  uint32_t         _locusRejected;  // Damaged items discarded
  unsigned long    _locusLastMs;    // Last LOCUS sentence during a dump

  // Common part of dumpLocus() / dumpLocusNmea(); type 0 = NMEA, 1 = compact
  LC76G_Result dumpLocusInternal(uint8_t type, LC76G_LocusDumpStats &stats,
                                 uint32_t idleTimeoutMs);
  // Validates and decodes one "$PAIR908,2,..." record
  bool decodeLocusRecord(const char *body, LC76G_LocusRecord &rec);

  // Stops LOCUS if active; returns true if it was recording
  bool pauseLocus();

  // On/off commands share the same shape: $PAIRnnn,<0|1> and a query
  // returning $PAIRmmm,<0|1>
  LC76G_Result setPairFlag(uint16_t id, bool enable);
  LC76G_Result getPairFlag(uint16_t id, bool &enabled);

  // Sends a PAIR query and returns field 'field' of the result as integer
  LC76G_Result queryPairInt(uint16_t id, uint8_t field, int32_t &value);

  // Sends "$<a>,<b>*CS" (or "$<a>*CS" if b is null/empty) without first
  // copying both parts into one buffer
  bool sendRaw2(const char *a, const char *b);
};

#endif // LC76G_H
