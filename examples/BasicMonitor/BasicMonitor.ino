/*
 * ============================================================================
 *  BasicMonitor.ino  -  Quectel_LCx6G_GNSS library example
 * ============================================================================
 *  Prints a readable summary once per second (date/time, fix, position,
 *  altitude, satellites) and uses the on-board LED as a status indicator:
 *    - fast blink (5 Hz) : no data from the module
 *    - slow blink (1 Hz) : searching for satellites
 *    - steady ON         : valid fix
 *
 *  Hardware: Arduino Uno R3 or Uno R4 + LC76G reference shield (rev. 1)
 *    D4 <- module TXD          D5 -> module RXD (level shifted)
 *    D6 -> PWR_EN_VCC (NPN base, no series resistor on rev. 1)
 *    D3 -> RESET driver (NPN base, no series resistor on rev. 1)
 *
 *  Using another board/module? Change the pins and drive modes below:
 *  see the LC76G_DriveMode description in src/LC76G.h.
 *
 *  Serial Monitor: 115200 baud.
 * ============================================================================
 */

#if !defined(ARDUINO_ARCH_RENESAS)
#include <SoftwareSerial.h>   // Uno R3 only: module UART on D4/D5
#endif
#include <Quectel_LC76G.h>

// ---------------------------------------------------------------------------
// Pins of the reference shield
// ---------------------------------------------------------------------------
const uint8_t PIN_GNSS_TXD   = 4;            // Arduino RX <- module TXD
const uint8_t PIN_GNSS_RXD   = 5;            // Arduino TX -> module RXD
const uint8_t PIN_PWR_EN     = 6;            // Module supply switch
const uint8_t PIN_RESET_DRV  = 3;            // Module reset driver
const uint8_t PIN_STATUS_LED = LED_BUILTIN;  // D13

// Module UART speed: 38400 is reliable with SoftwareSerial.
// A brand-new module (115200) is reconfigured automatically by begin().
const uint32_t GNSS_BAUD = 38400;

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

// ---------------------------------------------------------------------------
// Printing helpers (integer maths only, no float precision loss)
// ---------------------------------------------------------------------------

// Prints a value scaled by 10^decimals, e.g. (12345, 2) -> "123.45"
void printScaled(int32_t value, uint8_t decimals) {
  if (value < 0) {
    Serial.print('-');
    value = -value;
  }
  int32_t divisor = 1;
  for (uint8_t i = 0; i < decimals; i++) {
    divisor *= 10;
  }
  Serial.print(value / divisor);               // Integer part
  if (decimals > 0) {
    Serial.print('.');
    int32_t frac = value % divisor;
    for (int32_t d = divisor / 10; d > 1 && frac < d; d /= 10) {
      Serial.print('0');                       // Leading zeros of the fraction
    }
    Serial.print(frac);
  }
}

// Prints a two-digit number with a leading zero
void print2(uint8_t v) {
  if (v < 10) {
    Serial.print('0');
  }
  Serial.print(v);
}

// ---------------------------------------------------------------------------
// Called by the library once per second, after each complete epoch
// ---------------------------------------------------------------------------
void onEpoch() {
  const LC76G_Fix &f = gnss.nmea.fix();

  // Line 1: UTC (Coordinated Universal Time) date/time and fix status
  Serial.print(F("--- "));
  if (f.dateValid && f.timeValid) {
    print2(f.day);    Serial.print('/');
    print2(f.month);  Serial.print('/');
    Serial.print(f.year);
    Serial.print(' ');
    print2(f.hour);   Serial.print(':');
    print2(f.minute); Serial.print(':');
    print2(f.second);
  } else {
    Serial.print(F("--/--/---- --:--:--"));
  }
  Serial.print(F(" UTC | Fix: "));
  Serial.println(f.valid ? F("YES") : F("NO"));

  // Line 2: position and altitude
  Serial.print(F("    Latitude "));
  if (f.positionValid) printScaled(f.latE7, 7); else Serial.print(F("--"));
  Serial.print(F("  Longitude "));
  if (f.positionValid) printScaled(f.lonE7, 7); else Serial.print(F("--"));
  Serial.print(F("  Altitude "));
  if (f.altitudeValid) printScaled(f.altitudeCm, 2); else Serial.print(F("--"));
  Serial.println(F(" m"));

  // Line 3: accuracy and speed
  Serial.print(F("    Horizontal Dilution of Precision (HDOP) "));
  printScaled(f.hdopX100, 2);
  Serial.print(F("  Speed "));
  Serial.print(gnss.nmea.speedKmh(), 1);
  Serial.println(F(" km/h"));

  // Line 4: satellite totals
  Serial.print(F("    Satellites used "));
  Serial.print(f.satsUsed);
  Serial.print(F(", in view "));
  Serial.print(gnss.nmea.satsInViewTotal());
  Serial.print(F(", strongest Signal-to-Noise Ratio (SNR) "));
  Serial.print(gnss.nmea.maxSnr());
  Serial.println(F(" dB-Hz"));

  // Following lines: satellites in view per constellation, full names.
  // LC76G_System index i corresponds to the LC76G_GNSS_xxx bit (1 << i).
  for (uint8_t i = 0; i < LC76G_NUM_SYSTEMS; i++) {
    Serial.print(F("      "));
    Serial.print(LC76G::constellationName((uint8_t)(1u << i)));
    Serial.print(F(": "));
    Serial.println(gnss.nmea.satsInView((LC76G_System)i));
  }
}

// ---------------------------------------------------------------------------
// Non-blocking status LED
// ---------------------------------------------------------------------------
void updateStatusLed() {
  unsigned long now = millis();
  bool on;
  if (!gnss.isTalking()) {
    on = (now / 100) % 2;                      // Fast blink: no data
  } else if (gnss.nmea.fix().valid) {
    on = true;                                 // Steady: fix
  } else {
    on = (now / 500) % 2;                      // Slow blink: searching
  }
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
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
  Serial.print(F("=== Quectel_LCx6G_GNSS BasicMonitor v"));
  Serial.print(F(QUECTEL_LC76G_VERSION));
  Serial.println(F(" ==="));

  // Reference shield rev. 1: both control pins go straight to NPN bases,
  // so they must never be driven HIGH -> weak pull-up mode.
  gnss.setPowerPin(PIN_PWR_EN, LC76G_DRIVE_WEAK_PULLUP_HIGH);
  gnss.setResetPin(PIN_RESET_DRV, LC76G_DRIVE_WEAK_PULLUP_HIGH);

  // Decoded data is printed from the epoch callback
  gnss.nmea.onEpoch(onEpoch);

  // Start: openGnssPort() lets the library change the port speed
  Serial.println(F("Starting module..."));
  bool ok = gnss.begin(gnssSerial, openGnssPort, GNSS_BAUD);

  if (ok) {
    Serial.print(F("Module OK at "));
    Serial.print(gnss.baud());
    Serial.println(F(" baud"));
  } else {
    Serial.println(F("ERROR: module not responding. Check wiring and supply."));
  }
}

void loop() {
  gnss.update();        // Read and decode everything the module sends
  updateStatusLed();
}
