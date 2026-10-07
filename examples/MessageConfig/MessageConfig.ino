/*
 * ============================================================================
 *  MessageConfig.ino  -  Quectel_LCx6G_GNSS library example
 * ============================================================================
 *  Shows how to choose which NMEA sentences the module outputs.
 *
 *  1. Reads and prints the current output rate of every sentence type.
 *  2. Applies either:
 *       - the MINIMAL set used by the decoder (GGA, GSV, RMC), or
 *       - the factory DEFAULT set (GGA, GLL, GSA, GSV, RMC, VTG),
 *     depending on APPLY_MINIMAL below.
 *  3. Prints the rates again to confirm the change.
 *  4. Optionally saves the configuration to the module flash.
 *  5. Then prints, every second, how many bytes per second arrive from the
 *     module, so the traffic reduction is visible.
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
// User options
// ---------------------------------------------------------------------------
const bool    APPLY_MINIMAL = true;  // true = minimal set, false = factory default set
const uint8_t GSV_RATE      = 1;     // Minimal set only: GSV once every N fixes (1..20)
const bool    SAVE_TO_FLASH = false; // true = also store in flash (PAIR513).
                                     // Not needed while V_BCKP is powered by the battery.

// ---------------------------------------------------------------------------
// Pins of the reference shield (see BasicMonitor for details)
// ---------------------------------------------------------------------------
const uint8_t  PIN_GNSS_TXD  = 4;
const uint8_t  PIN_GNSS_RXD  = 5;
const uint8_t  PIN_PWR_EN    = 6;
const uint8_t  PIN_RESET_DRV = 3;
const uint32_t GNSS_BAUD     = 38400;

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

// Bytes received from the module since the last epoch
volatile uint32_t bytesThisEpoch = 0;

// ---------------------------------------------------------------------------
// Prints the output rate of every sentence type, one per line, with the
// full sentence name, e.g. "  Time and Date (ZDA) = off"
// ---------------------------------------------------------------------------
void printAllRates() {
  for (uint8_t t = 0; t < LC76G_NMEA_TYPE_COUNT; t++) {
    uint8_t rate = 0;
    LC76G_Result r = gnss.getNmeaRate((LC76G_NmeaType)t, rate);
    Serial.print(F("  "));
    Serial.print(LC76G::nmeaTypeName((LC76G_NmeaType)t));
    Serial.print(F(" = "));
    if (r != LC76G_OK) {
      Serial.println(LC76G::resultText(r));    // Query failed: show why
    } else if (rate == 0) {
      Serial.println(F("off"));
    } else if (rate == 1) {
      Serial.println(F("every fix"));
    } else {
      Serial.print(F("every "));
      Serial.print(rate);
      Serial.println(F(" fixes"));
    }
  }
}

// ---------------------------------------------------------------------------
// Counts the bytes of every valid sentence ('$' + body + CR LF)
// ---------------------------------------------------------------------------
void onSentence(const char *body) {
  bytesThisEpoch += strlen(body) + 3;
}

// ---------------------------------------------------------------------------
// Once per second: traffic and short fix status
// ---------------------------------------------------------------------------
void onEpoch() {
  const LC76G_Fix &f = gnss.nmea.fix();
  Serial.print(F("Traffic "));
  Serial.print(bytesThisEpoch);
  Serial.print(F(" bytes/s | "));
  Serial.print(f.valid ? F("FIX") : F("NO FIX"));
  Serial.print(F(" | satellites used "));
  Serial.print(f.satsUsed);
  Serial.print(F(", in view "));
  Serial.println(gnss.nmea.satsInViewTotal());
  bytesThisEpoch = 0;
}

// ---------------------------------------------------------------------------
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
  Serial.println();
  Serial.println(F("=== Quectel_LCx6G_GNSS MessageConfig ==="));

  gnss.setPowerPin(PIN_PWR_EN, LC76G_DRIVE_WEAK_PULLUP_HIGH);
  gnss.setResetPin(PIN_RESET_DRV, LC76G_DRIVE_WEAK_PULLUP_HIGH);

  if (!gnss.begin(gnssSerial, openGnssPort, GNSS_BAUD)) {
    Serial.println(F("ERROR: module not responding."));
    return;
  }

  // 1) Current configuration
  Serial.println(F("NMEA output before:"));
  printAllRates();

  // 2) Apply the chosen configuration
  LC76G_Result r;
  if (APPLY_MINIMAL) {
    Serial.print(F("Applying minimal output... "));
    r = gnss.setMinimalNmeaOutput(GSV_RATE);
  } else {
    // Factory default: the six standard sentences every fix
    Serial.print(F("Restoring default output... "));
    r = LC76G_OK;
    for (uint8_t t = LC76G_NMEA_GGA; t <= LC76G_NMEA_VTG && r == LC76G_OK; t++) {
      r = gnss.setNmeaRate((LC76G_NmeaType)t, 1);
    }
  }
  Serial.println(LC76G::resultText(r));

  // 3) Verify
  Serial.println(F("NMEA output after:"));
  printAllRates();

  // 4) Optional permanent save
  if (SAVE_TO_FLASH) {
    Serial.print(F("Saving to flash... "));
    Serial.println(LC76G::resultText(gnss.saveSettings()));
  }

  // 5) Start the traffic monitor
  gnss.onSentence(onSentence);
  gnss.nmea.onEpoch(onEpoch);
  Serial.println(F("--- Traffic monitor ---"));
}

void loop() {
  gnss.update();
}
