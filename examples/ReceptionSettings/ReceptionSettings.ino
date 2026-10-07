/*
 * ============================================================================
 *  ReceptionSettings.ino  -  Quectel_LCx6G_GNSS library example
 * ============================================================================
 *  Reception settings of the module:
 *    elevation mask, AIC (Active Interference Cancellation), SBAS
 *    (Satellite-Based Augmentation System, EGNOS in Europe) and DGPS
 *    source, 2D fix, real-time speed response, BDS B1C band, static
 *    navigation threshold.
 *  The sketch reads every setting, applies the values chosen below, reads
 *  them again, then prints a status block every 5 seconds. Outdoors, a fix
 *  corrected by SBAS shows the quality "DGPS".
 *
 *  Status LED on D13: fast blink = no data, slow blink = searching,
 *  steady = fix.
 *
 *  Hardware: Arduino Uno R3 or Uno R4 + LC76G reference shield (rev. 1).
 *  Serial Monitor: 115200 baud.
 * ============================================================================
 */

#if !defined(ARDUINO_ARCH_RENESAS)
#include <SoftwareSerial.h>   // Uno R3 only: module UART on D4/D5
#endif
#include <Quectel_LC76G.h>

// ---------------------------------------------------------------------------
// Values to apply (module defaults in brackets, where documented)
// ---------------------------------------------------------------------------
const int8_t           ELEVATION_MASK_DEG = 10;              // [5] degrees above the horizon
const bool             AIC_ON             = true;            // Interference cancellation
const bool             SBAS_ON            = true;            // Search EGNOS satellites
const LC76G_DgpsSource DGPS_SOURCE        = LC76G_DGPS_SBAS; // Apply the SBAS corrections
const bool             FIX_2D_ON          = true;            // Allow fix with 3 satellites
const bool             IMMEDIATE_SPEED_ON = false;           // Less filtered speed
const bool             BDS_B1C_ON         = true;            // Track the BDS B1C band
const uint8_t          STATIC_DMS         = 0;               // [0] static threshold, dm/s

// ---------------------------------------------------------------------------
// Pins of the reference shield
// ---------------------------------------------------------------------------
const uint8_t  PIN_GNSS_TXD   = 4;   // Arduino RX <- module TXD
const uint8_t  PIN_GNSS_RXD   = 5;   // Arduino TX -> module RXD (level shifted)
const uint8_t  PIN_PWR_EN     = 6;   // Supply switch (NPN base, no resistor on rev. 1)
const uint8_t  PIN_RESET_DRV  = 3;   // Reset driver  (NPN base, no resistor on rev. 1)
const uint8_t  PIN_STATUS_LED = LED_BUILTIN;
const uint32_t GNSS_BAUD      = 38400;

// ---------------------------------------------------------------------------
//  Serial port towards the module (chosen automatically by board type)
//  Uno R3: SoftwareSerial on D4/D5 (D0/D1 belong to the USB link).
//  Uno R4: hardware Serial1 on D0/D1. On the R4 D4 cannot be a
//          SoftwareSerial RX pin (not interrupt-capable), so on the
//          reference shield solder two wires D4 -> D0 and D5 -> D1
//          (remove them before going back to an Uno R3).
// ---------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_RENESAS)
#define gnssSerial Serial1
void openGnssPort(uint32_t baud) {
  Serial1.begin(baud);                        // Hardware UART: no RX pull-up to remove
}
#else
SoftwareSerial gnssSerial(PIN_GNSS_TXD, PIN_GNSS_RXD);  // (rx, tx)
void openGnssPort(uint32_t baud) {
  gnssSerial.begin(baud);                     // (Re)open the software port
  LC76G::disableRxPullup(PIN_GNSS_TXD);       // No 5 V pull-up into the 3.3 V TXD
}
#endif
LC76G gnss;

uint8_t epochCounter = 0;            // Status block every 5 epochs (5 s)


// ===========================================================================
//  PRINT HELPERS
// ===========================================================================

// "on" / "off" or the error text
void printFlag(LC76G_Result r, bool on) {
  if (r == LC76G_OK) Serial.println(on ? F("on") : F("off"));
  else               Serial.println(LC76G::resultText(r));
}

// All reception settings, one per line with the full name
void printReceptionSettings() {
  LC76G_Result r;
  bool on = false;

  int8_t deg = 0;
  Serial.print(F("  Elevation mask: "));
  r = gnss.getElevationMask(deg);
  if (r == LC76G_OK) { Serial.print(deg); Serial.println(F(" degrees")); }
  else               Serial.println(LC76G::resultText(r));

  Serial.print(F("  Active Interference Cancellation (AIC): "));
  r = gnss.getAic(on);
  printFlag(r, on);

  Serial.print(F("  Satellite-Based Augmentation System (SBAS) search: "));
  r = gnss.getSbas(on);
  printFlag(r, on);

  LC76G_DgpsSource src = LC76G_DGPS_NONE;
  Serial.print(F("  Differential GPS (DGPS) correction source: "));
  r = gnss.getDgpsSource(src);
  if (r == LC76G_OK) Serial.println(LC76G::dgpsSourceText(src));
  else               Serial.println(LC76G::resultText(r));

  Serial.print(F("  2D fix allowed: "));
  r = gnss.get2dFix(on);
  printFlag(r, on);

  Serial.print(F("  Real-time speed response: "));
  r = gnss.getImmediateSpeed(on);
  printFlag(r, on);

  Serial.print(F("  BeiDou (BDS) B1C band tracking: "));
  r = gnss.getBdsB1c(on);
  printFlag(r, on);

  uint8_t dms = 0;
  Serial.print(F("  Static navigation threshold: "));
  r = gnss.getStaticThreshold(dms);
  if (r != LC76G_OK)  Serial.println(LC76G::resultText(r));
  else if (dms == 0)  Serial.println(F("off"));
  else { Serial.print(dms); Serial.println(F(" dm/s")); }
}

// Prints "name... result" for one setting command
void apply(const __FlashStringHelper *name, LC76G_Result r) {
  Serial.print(F("  "));
  Serial.print(name);
  Serial.print(F("... "));
  Serial.println(LC76G::resultText(r));
}


// ===========================================================================
//  STATUS BLOCK (epoch callback, prints only)
// ===========================================================================
void onEpoch() {
  if (++epochCounter < 5) {
    return;
  }
  epochCounter = 0;

  const LC76G_Fix &f = gnss.nmea.fix();
  Serial.print(F("--- Fix quality: "));
  Serial.print(LC76G::fixQualityText(f.quality));
  Serial.print(F(" | satellites used "));
  Serial.print(f.satsUsed);
  Serial.print(F(", in view "));
  Serial.print(gnss.nmea.satsInViewTotal());
  Serial.print(F(" | Horizontal Dilution of Precision (HDOP) "));
  Serial.print(f.hdopX100 / 100);
  Serial.print('.');
  uint8_t frac = f.hdopX100 % 100;
  if (frac < 10) Serial.print('0');
  Serial.print(frac);
  Serial.print(F(" | strongest Signal-to-Noise Ratio (SNR) "));
  Serial.print(gnss.nmea.maxSnr());
  Serial.println(F(" dB-Hz"));
}


// ===========================================================================
//  STATUS LED (non-blocking)
// ===========================================================================
void updateStatusLed() {
  unsigned long now = millis();
  bool on;
  if (!gnss.isTalking())          on = (now / 100) % 2;   // Fast blink: no data
  else if (gnss.nmea.fix().valid) on = true;              // Steady: fix
  else                            on = (now / 500) % 2;   // Slow blink: searching
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}


// ===========================================================================
//  SETUP / LOOP
// ===========================================================================
void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);              // Never leave D13 floating (op-amp LED driver)
  Serial.begin(115200);
  // Uno R4: native USB, wait (max 3 s) for the Serial Monitor so the first
  // lines are not lost. On the Uno R3 this returns at once.
  while (!Serial && millis() < 3000) {
  }
#if defined(ARDUINO_ARCH_RENESAS)
  // Uno R4: D4/D5 are wired to D0/D1, keep them as high-impedance inputs
  pinMode(PIN_GNSS_TXD, INPUT);
  pinMode(PIN_GNSS_RXD, INPUT);
#endif
  Serial.println();
  Serial.print(F("=== Quectel_LCx6G_GNSS ReceptionSettings v"));
  Serial.print(F(QUECTEL_LC76G_VERSION));
  Serial.println(F(" ==="));

  gnss.setPowerPin(PIN_PWR_EN, LC76G_DRIVE_WEAK_PULLUP_HIGH);
  gnss.setResetPin(PIN_RESET_DRV, LC76G_DRIVE_WEAK_PULLUP_HIGH);
  if (!gnss.begin(gnssSerial, openGnssPort, GNSS_BAUD)) {
    Serial.println(F("ERROR: module not responding."));
    return;
  }
  Serial.print(F("Module OK at "));
  Serial.print(gnss.baud());
  Serial.println(F(" baud"));

  // Only the sentences the decoder needs: less SoftwareSerial traffic
  Serial.print(F("Minimal NMEA output... "));
  Serial.println(LC76G::resultText(gnss.setMinimalNmeaOutput(1)));

  // ---- Feature under test: reception settings ----
  Serial.println(F("Reception settings before:"));
  printReceptionSettings();

  Serial.println(F("Applying:"));
  apply(F("Elevation mask"),           gnss.setElevationMask(ELEVATION_MASK_DEG));
  apply(F("AIC"),                      gnss.setAic(AIC_ON));
  apply(F("SBAS search"),              gnss.setSbas(SBAS_ON));
  apply(F("DGPS source"),              gnss.setDgpsSource(DGPS_SOURCE));
  apply(F("2D fix"),                   gnss.set2dFix(FIX_2D_ON));
  apply(F("Real-time speed response"), gnss.setImmediateSpeed(IMMEDIATE_SPEED_ON));
  apply(F("BDS B1C band"),             gnss.setBdsB1c(BDS_B1C_ON));
  apply(F("Static threshold"),         gnss.setStaticThreshold(STATIC_DMS));

  Serial.println(F("Reception settings after:"));
  printReceptionSettings();

  gnss.nmea.onEpoch(onEpoch);
  Serial.println(F("--- Monitoring ---"));
}

void loop() {
  gnss.update();
  updateStatusLed();
}
