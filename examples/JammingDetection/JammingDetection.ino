/*
 * ============================================================================
 *  JammingDetection.ino  -  Quectel_LCx6G_GNSS library example
 * ============================================================================
 *  Enables jamming detection (interference covering the GNSS signals) and
 *  prints every change of status, from the module message and from the
 *  JAM_IND pin (LOW = jamming detected).
 *  Note: deliberately transmitting on GNSS frequencies is illegal; this
 *  example only verifies the "no jamming" condition and real events.
 *
 *  Hardware: Arduino Uno R3 or Uno R4 + LC76G reference shield (rev. 1).
 *  JAM_IND output of the module on D8 (plain input, never pulled up).
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

const uint8_t PIN_JAM_IND = 8;                // <- module JAM_IND (LOW = jamming)

LC76G_JamStatus lastStatus = LC76G_JAM_UNKNOWN;
bool            lastPin    = false;

void setup() {
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
  Serial.println(F("=== Quectel_LCx6G_GNSS JammingDetection ==="));
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

  gnss.setJamIndicatorPin(PIN_JAM_IND);
  // The setting is not stored by the module: send it after every power-up
  Serial.print(F("Enabling jamming detection... "));
  Serial.println(LC76G::resultText(gnss.setJammingDetection(true)));
}

void loop() {
  gnss.update();

  LC76G_JamStatus st = gnss.jammingStatus();
  bool pin = gnss.jamIndicatorActive();
  if (st != lastStatus || pin != lastPin) {     // Print only the changes
    Serial.print(F("Jamming status: "));
    Serial.print(LC76G::jamStatusText(st));
    Serial.print(F(" | JAM_IND pin: "));
    Serial.println(pin ? F("LOW (jamming)") : F("HIGH (no jamming)"));
    lastStatus = st;
    lastPin = pin;
  }
}
