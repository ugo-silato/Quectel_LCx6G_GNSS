/*
 * ============================================================================
 *  Pps.ino  -  Quectel_LC76G library example
 * ============================================================================
 *  Configures the 1PPS (One Pulse Per Second) output and measures the
 *  pulses with an interrupt: count, period, width. Mode "always" pulses
 *  also without fix, so the example works indoors. The difference between
 *  the measured period and 1 000 000 us is almost entirely the error of
 *  the Arduino clock (1 us = 1 ppm).
 *
 *  Hardware: Arduino Uno R3 or Uno R4 + LC76G reference shield (rev. 1).
 *  1PPS output of the module on D2 (interrupt pin, plain input).
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

const uint8_t PIN_PPS = 2;                    // <- module 1PPS (INT0 on the Uno R3)

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
  Serial.println(F("=== Quectel_LC76G Pps ==="));
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

  // 100 ms positive pulse, always (even without fix)
  Serial.print(F("Configuring 1PPS... "));
  Serial.println(LC76G::resultText(gnss.setPps(true, 100, LC76G_PPS_ALWAYS, true)));
  if (!gnss.beginPpsCapture(PIN_PPS)) {
    Serial.println(F("ERROR: this pin cannot generate interrupts"));
  }
}

void loop() {
  gnss.update();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 5000) {
    return;                                   // Print every 5 s
  }
  lastPrint = millis();
  if (gnss.ppsCount() == 0 || gnss.ppsAgeMs() > 2000) {
    Serial.println(F("No 1PPS pulses"));
    return;
  }
  uint32_t period = gnss.ppsPeriodUs();
  Serial.print(gnss.ppsCount());
  Serial.print(F(" pulses, period "));
  Serial.print(period);
  Serial.print(F(" us, width "));
  Serial.print(gnss.ppsWidthUs());
  Serial.print(F(" us, Arduino clock error "));
  Serial.print((long)period - 1000000L);
  Serial.println(F(" ppm"));
}
