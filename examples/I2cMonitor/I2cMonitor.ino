/*
 * ============================================================================
 *  I2cMonitor.ino  -  Quectel_LC76G library example
 * ============================================================================
 *  Same output as BasicMonitor, but the module is read and commanded
 *  through I2C (SDA / SCL) instead of the UART.
 *
 *  Hardware: Arduino Uno R3 or Uno R4 + LC76G reference shield (rev. 1)
 *    SDA / SCL <-> module I2C (2.2 k pull-ups to 3.3 V on the shield)
 *    D6 -> PWR_EN_VCC (NPN base, no series resistor on rev. 1)
 *    D3 -> RESET driver (NPN base, no series resistor on rev. 1)
 *  The UART is not used (the module keeps sending NMEA on it as well).
 *
 *  VOLTAGE WARNING: the rev. 1 shield has no I2C level shifter. The bus
 *  high level is 3.3 V, below the guaranteed input threshold of the 5 V
 *  Arduino I2C pins (0.7 x 5 V = 3.5 V). It worked on the bench on the
 *  Uno R4 Minima; a level shifter is the safe solution (rev. 2).
 *  LC76G_I2C::begin() switches off the Arduino internal pull-ups to 5 V.
 *
 *  Speed: every I2C transfer needs a 10 ms pause, so the output is limited
 *  to the sentences the decoder needs (setMinimalNmeaOutput()).
 *
 *  Serial Monitor: 115200 baud.
 * ============================================================================
 */

#include <Wire.h>                     // I2C master
#include <Quectel_LC76G.h>
#include <LC76G_I2C.h>                // I2C port for the library (header-only)

// ---------------------------------------------------------------------------
// Pins of the reference shield
// ---------------------------------------------------------------------------
const uint8_t PIN_GNSS_TXD   = 4;            // Module TXD (not used, kept as input)
const uint8_t PIN_GNSS_RXD   = 5;            // Module RXD (not used, kept as input)
const uint8_t PIN_PWR_EN     = 6;            // Module supply switch
const uint8_t PIN_RESET_DRV  = 3;            // Module reset driver
const uint8_t PIN_STATUS_LED = LED_BUILTIN;  // D13

const uint32_t I2C_CLOCK_HZ = 100000;        // 100 kHz (400 kHz also supported)
// How often the module is asked for new data when nothing is pending.
// Each poll blocks the sketch for about 20 ms; at 1 Hz 200..1000 ms is fine.
const uint16_t I2C_POLL_MS  = 200;

LC76G_I2C gnssI2c(Wire);                     // I2C port, used like a serial port
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

  // Interference seen by the module (useful to judge the I2C activity)
  Serial.print(F("    Jamming status: "));
  Serial.println(LC76G::jamStatusText(gnss.jammingStatus()));

  // I2C transfers that failed after all the attempts (should stay 0)
  if (gnssI2c.errors() > 0) {
    Serial.print(F("    I2C errors: "));
    Serial.println(gnssI2c.errors());
  }

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
  // UART pins of the shield not used: keep them as high-impedance inputs
  pinMode(PIN_GNSS_TXD, INPUT);
  pinMode(PIN_GNSS_RXD, INPUT);
  Serial.println();
  Serial.print(F("=== Quectel_LC76G I2cMonitor v"));
  Serial.print(F(QUECTEL_LC76G_VERSION));
  Serial.println(F(" ==="));

  // Reference shield rev. 1: both control pins go straight to NPN bases,
  // so they must never be driven HIGH -> weak pull-up mode.
  gnss.setPowerPin(PIN_PWR_EN, LC76G_DRIVE_WEAK_PULLUP_HIGH);
  gnss.setResetPin(PIN_RESET_DRV, LC76G_DRIVE_WEAK_PULLUP_HIGH);

  // Decoded data is printed from the epoch callback
  gnss.nmea.onEpoch(onEpoch);

  // 1) Power the module FIRST: on the reference shield the I2C pull-ups
  //    are supplied by the module rail, so with the module off the bus
  //    sits at 0 V and no transfer can work.
  Serial.println(F("1) Powering the module..."));
  Serial.flush();                              // Make sure the line goes out
  gnss.powerOn();
  delay(LC76G_BOOT_MS);                        // Let it boot

  // 2) I2C master, internal pull-ups to 5 V switched off
  Serial.println(F("2) Starting I2C..."));
  Serial.flush();
  gnssI2c.begin(I2C_CLOCK_HZ);
  gnssI2c.setPollInterval(I2C_POLL_MS);
  Serial.print(F("   poll interval "));
  Serial.print(I2C_POLL_MS);
  Serial.println(F(" ms"));

  // 3) Module start-up through I2C (nullptr: no baud rate on I2C)
  Serial.println(F("3) Waiting for the module on I2C..."));
  Serial.flush();
  bool ok = gnss.begin(gnssI2c, nullptr);

  if (ok) {
    Serial.println(F("Module OK on I2C"));
    // Less data to move: only the sentences the decoder needs
    LC76G_Result r = gnss.setMinimalNmeaOutput();
    Serial.print(F("Minimal NMEA output... "));
    Serial.println(LC76G::resultText(r));
    // Jamming detection: tells whether the module sees interference
    r = gnss.setJammingDetection(true);
    Serial.print(F("Jamming detection... "));
    Serial.println(LC76G::resultText(r));
  } else {
    Serial.print(F("ERROR: module not responding on I2C (failed transfers "));
    Serial.print(gnssI2c.errors());
    Serial.println(F("). Check SDA/SCL and supply."));
  }
}

void loop() {
  gnss.update();        // Read and decode everything the module sends
  updateStatusLed();
}
