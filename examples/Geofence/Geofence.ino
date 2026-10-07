/*
 * ============================================================================
 *  Geofence.ino  -  Quectel_LCx6G_GNSS library example
 * ============================================================================
 *  Creates a circular geofence (virtual fence) around the position of the
 *  first fix and reports whether the receiver is inside or outside, both
 *  from the module status message and from the GEOFENCE pin.
 *  Test: wait for the first fix, then walk farther than the radius
 *  (plus some margin for the position error).
 *  NOTE: not field-tested yet (only the "inside" state was verified).
 *
 *  On-board LED: fast blink = no fix / geofence not created yet,
 *                steady = inside, slow blink = outside.
 *
 *  Hardware: Arduino Uno R3 or Uno R4 + LC76G reference shield (rev. 1).
 *  GEOFENCE output of the module on D7 (plain input, never pulled up).
 *  Serial Monitor: 115200 baud.
 * ============================================================================
 */

#if !defined(ARDUINO_ARCH_RENESAS)
#include <SoftwareSerial.h>   // Uno R3 only: module UART on D4/D5
#endif
#include <Quectel_LC76G.h>

// ---------------------------------------------------------------------------
// Pins of the reference shield (change them for other hardware)
// ---------------------------------------------------------------------------
const uint8_t  PIN_GNSS_TXD  = 4;   // Arduino RX <- module TXD
const uint8_t  PIN_GNSS_RXD  = 5;   // Arduino TX -> module RXD (level shifted)
const uint8_t  PIN_PWR_EN    = 6;   // Module supply switch (NPN base)
const uint8_t  PIN_RESET_DRV = 3;   // Module reset driver  (NPN base)
const uint32_t GNSS_BAUD     = 38400; // Reliable speed for SoftwareSerial (Uno R3)

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

const uint8_t  PIN_GEOFENCE = 7;              // <- module GEOFENCE (HIGH = inside)
const uint32_t RADIUS_M     = 30;             // Circle radius in metres

bool fenceCreated = false;                    // Set once the circle has been sent
LC76G_GeofenceState lastState = LC76G_GEO_UNKNOWN;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
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
  Serial.println(F("=== Quectel_LCx6G_GNSS Geofence ==="));
  // Reference shield rev. 1: control pins go straight to NPN bases, so they
  // are never driven HIGH (see LC76G_DriveMode in src/LC76G.h)
  gnss.setPowerPin(PIN_PWR_EN, LC76G_DRIVE_WEAK_PULLUP_HIGH);
  gnss.setResetPin(PIN_RESET_DRV, LC76G_DRIVE_WEAK_PULLUP_HIGH);

  // Start-up; a brand-new module (115200 baud) is reconfigured automatically
  if (!gnss.begin(gnssSerial, openGnssPort, GNSS_BAUD)) {
    Serial.println(F("ERROR: module not responding. Check wiring and supply."));
    while (true) { }                            // Stop here
  }
  Serial.println(F("Module OK"));

  gnss.setGeofencePin(PIN_GEOFENCE);
  gnss.disableGeofence(0);                    // Remove any old area
  gnss.setGeofenceStatusOutput(1);            // $PQTMGEOFENCESTATUS every fix
  Serial.println(F("Waiting for the first fix..."));
}

void loop() {
  gnss.update();
  const LC76G_Fix &f = gnss.nmea.fix();

  // First fix: create the circle around the current position
  if (!fenceCreated && f.valid && f.positionValid) {
    Serial.print(F("Creating geofence 0 around the current position... "));
    LC76G_Result r = gnss.setGeofenceCircle(0, f.latE7, f.lonE7, RADIUS_M);
    Serial.println(LC76G::resultText(r));
    fenceCreated = (r == LC76G_OK);
  }

  // Report every change, comparing message and pin
  LC76G_GeofenceState st = gnss.geofenceState(0);
  if (fenceCreated && st != lastState) {
    Serial.print(F("Geofence 0: "));
    Serial.print(LC76G::geofenceStateText(st));
    Serial.print(F(" | GEOFENCE pin: "));
    Serial.println(gnss.geofencePinInside() ? F("HIGH (inside)") : F("LOW (outside)"));
    lastState = st;
  }

  // LED
  unsigned long now = millis();
  bool on;
  if (!fenceCreated || st == LC76G_GEO_UNKNOWN) on = (now / 100) % 2;   // Fast blink
  else if (st == LC76G_GEO_INSIDE)              on = true;              // Steady
  else                                          on = (now / 500) % 2;   // Slow blink
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}
