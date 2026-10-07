/*
 * ============================================================================
 *  LC76G_Text.cpp  -  Readable names for results, sentences, constellations
 * ============================================================================
 *  Part of the Quectel_LC76G Arduino library. Author: Ugo Silato. License: MIT.
 *
 *  All strings are returned with F(), so they stay in flash memory and do
 *  not use the very limited RAM of the Uno R3. They can be printed directly:
 *      Serial.print(LC76G::nmeaTypeName(LC76G_NMEA_ZDA));
 * ============================================================================
 */

#include "LC76G.h"

const __FlashStringHelper *LC76G::resultText(LC76G_Result r) {
  switch (r) {
    case LC76G_OK:            return F("OK");
    case LC76G_PROCESSING:    return F("processing");
    case LC76G_FAILED:        return F("failed");
    case LC76G_NOT_SUPPORTED: return F("not supported");
    case LC76G_PARAM_ERROR:   return F("parameter error");
    case LC76G_BUSY:          return F("busy");
    case LC76G_TIMEOUT:       return F("timeout");
    case LC76G_NO_PORT:       return F("no port");
    case LC76G_NO_REBOOT:     return F("reboot needed");
  }
  return F("unknown");
}

const __FlashStringHelper *LC76G::nmeaTypeName(LC76G_NmeaType type) {
  switch (type) {
    case LC76G_NMEA_GGA: return F("Global Positioning System Fix Data (GGA)");
    case LC76G_NMEA_GLL: return F("Geographic Position - Latitude/Longitude (GLL)");
    case LC76G_NMEA_GSA: return F("GNSS DOP and Active Satellites (GSA)");
    case LC76G_NMEA_GSV: return F("GNSS Satellites in View (GSV)");
    case LC76G_NMEA_RMC: return F("Recommended Minimum Specific GNSS Data (RMC)");
    case LC76G_NMEA_VTG: return F("Course Over Ground and Ground Speed (VTG)");
    case LC76G_NMEA_ZDA: return F("Time and Date (ZDA)");
    case LC76G_NMEA_GRS: return F("GNSS Range Residuals (GRS)");
    case LC76G_NMEA_GST: return F("GNSS Pseudorange Error Statistics (GST)");
    case LC76G_NMEA_GNS: return F("GNSS Fix Data (GNS)");
    default:             break;
  }
  return F("unknown sentence");
}

const __FlashStringHelper *LC76G::constellationName(uint8_t bit) {
  switch (bit) {
    case LC76G_GNSS_GPS:     return F("Global Positioning System (GPS)");
    case LC76G_GNSS_GLONASS: return F("GLObal NAvigation Satellite System (GLONASS)");
    case LC76G_GNSS_GALILEO: return F("Galileo");
    case LC76G_GNSS_BDS:     return F("BeiDou Navigation Satellite System (BDS)");
    case LC76G_GNSS_QZSS:    return F("Quasi-Zenith Satellite System (QZSS)");
    default:                 break;
  }
  return F("unknown system");
}

const __FlashStringHelper *LC76G::jamStatusText(LC76G_JamStatus s) {
  switch (s) {
    case LC76G_JAM_UNKNOWN:  return F("unknown (detection off or not evaluated yet)");
    case LC76G_JAM_GOOD:     return F("good (no jamming)");
    case LC76G_JAM_WARNING:  return F("warning (interference detected)");
    case LC76G_JAM_CRITICAL: return F("critical (strong interference)");
  }
  return F("invalid");
}

const __FlashStringHelper *LC76G::geofenceStateText(LC76G_GeofenceState s) {
  switch (s) {
    case LC76G_GEO_UNKNOWN: return F("unknown (disabled or no fix)");
    case LC76G_GEO_INSIDE:  return F("inside");
    case LC76G_GEO_OUTSIDE: return F("outside");
  }
  return F("invalid");
}

const __FlashStringHelper *LC76G::ppsModeText(LC76G_PpsMode m) {
  switch (m) {
    case LC76G_PPS_ALWAYS:          return F("always (even without fix)");
    case LC76G_PPS_2D_FIX:          return F("only with 2D fix");
    case LC76G_PPS_3D_FIX:          return F("only with 3D fix");
    case LC76G_PPS_AFTER_FIRST_FIX: return F("after the first fix");
  }
  return F("invalid");
}

const __FlashStringHelper *LC76G::navModeText(LC76G_NavMode m) {
  switch (m) {
    case LC76G_NAV_NORMAL:     return F("normal (general purpose)");
    case LC76G_NAV_FITNESS:    return F("fitness (walking, running)");
    case LC76G_NAV_BALLOON:    return F("balloon (high altitude)");
    case LC76G_NAV_STATIONARY: return F("stationary (not moving)");
    case LC76G_NAV_DRONE:      return F("drone");
    case LC76G_NAV_SWIMMING:   return F("swimming");
  }
  return F("reserved/unknown");
}

const __FlashStringHelper *LC76G::periodicModeText(LC76G_PeriodicMode m) {
  switch (m) {
    case LC76G_PERIODIC_OFF:    return F("off (continuous)");
    case LC76G_PERIODIC_SMART:  return F("smart");
    case LC76G_PERIODIC_STRICT: return F("strict");
  }
  return F("invalid");
}

const __FlashStringHelper *LC76G::epocStatusText(int32_t status) {
  switch (status) {
    case 1: return F("prediction in progress");
    case 2: return F("all received ephemeris predicted");
    case 3: return F("no valid ephemeris data yet");
  }
  return F("unknown");
}

const __FlashStringHelper *LC76G::rtcmModeText(LC76G_RtcmMode m) {
  switch (m) {
    case LC76G_RTCM_OFF:  return F("off");
    case LC76G_RTCM_MSM4: return F("MSM4");
    case LC76G_RTCM_MSM7: return F("MSM7");
  }
  return F("unknown");
}

const __FlashStringHelper *LC76G::dgpsSourceText(LC76G_DgpsSource s) {
  switch (s) {
    case LC76G_DGPS_NONE: return F("none");
    case LC76G_DGPS_SBAS: return F("SBAS (EGNOS, WAAS, GAGAN, MSAS)");
    case LC76G_DGPS_SLAS: return F("QZSS SLAS (Japan)");
  }
  return F("unknown");
}

const __FlashStringHelper *LC76G::fixQualityText(uint8_t quality) {
  switch (quality) {
    case 0: return F("no fix");
    case 1: return F("GPS (autonomous)");
    case 2: return F("DGPS (corrected, e.g. by SBAS)");
    case 6: return F("estimated (dead reckoning)");
  }
  return F("other");
}
