/*
 * ============================================================================
 *  Agnss.ino  -  Quectel_LCx6G_GNSS library example
 * ============================================================================
 *  AGNSS (Assisted GNSS) without internet. The firmware has EITHER EASY
 *  (Embedded Assist System) OR EPOC (Enhanced Prediction Orbit on Chip):
 *  the example detects which one, enables it and prints the orbit
 *  prediction progress every minute. Leave the module outdoors, with a
 *  fix, until the prediction is complete: later starts will be faster.
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

// EPOC constellations: GPS alone, or GPS + Galileo, or GPS + BDS
const uint8_t EPOC_SET = LC76G_EPOC_GPS | LC76G_EPOC_GALILEO;

bool epoc = false;                            // true if the firmware has EPOC

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
  Serial.println(F("=== Quectel_LCx6G_GNSS Agnss ==="));
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

  char ver[32];
  gnss.getFirmwareVersion(ver, sizeof(ver));
  Serial.print(F("Firmware: "));
  Serial.println(ver);

  bool on;
  uint8_t mask;
  epoc = (gnss.getEpoc(on, mask) == LC76G_OK);
  if (epoc) {
    Serial.println(F("Prediction method: EPOC"));
    gnss.setEpoc(true);
    gnss.setEpocConstellations(EPOC_SET);
  } else {
    Serial.println(F("Prediction method: EASY"));
    gnss.setEasy(true);
  }
}

void loop() {
  gnss.update();

  static unsigned long last = 0;
  if (millis() - last < 60000UL) {
    return;                                   // Report every minute
  }
  last = millis();

  if (epoc) {
    // One line per configured constellation
    for (uint8_t bit = LC76G_EPOC_GPS; bit <= LC76G_EPOC_BDS; bit <<= 1) {
      if (!(EPOC_SET & bit)) continue;
      int32_t status;
      uint8_t ready;
      if (gnss.getEpocPredictionStatus(bit, status, ready) == LC76G_OK) {
        Serial.print(F("EPOC bit 0x"));
        Serial.print(bit, HEX);
        Serial.print(F(": "));
        Serial.print(LC76G::epocStatusText(status));
        Serial.print(F(", satellites ready: "));
        Serial.println(ready);
      }
    }
  } else {
    bool on;
    uint8_t days;
    if (gnss.getEasy(on, days) == LC76G_OK) {
      Serial.print(F("EASY prediction days ready: "));
      Serial.println(days);
    }
  }
}
