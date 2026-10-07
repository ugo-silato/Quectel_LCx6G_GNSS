/*
 * ============================================================================
 *  Locus.ino  -  Quectel_LCx6G_GNSS library example
 * ============================================================================
 *  LOCUS, the built-in logger of the module (128 KB). Choose ONE ACTION:
 *    ACTION_START : record one position every PERIOD_S seconds
 *    ACTION_DUMP  : read all records and print them as CSV lines
 *    ACTION_CLEAR : erase all records
 *  WARNING: on the reference shield the module is switched off when the
 *  Arduino resets or loses power (on the Uno R3 also when the Serial
 *  Monitor opens): a recording in progress is lost in that case.
 *  LIMITATION: with SoftwareSerial (Uno R3) the read-back loses part of
 *  the records. On the Uno R4 (Serial1) the complete read-back has been
 *  verified (87 records, 0 damaged).
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

#define ACTION_START 1
#define ACTION_DUMP  2
#define ACTION_CLEAR 3
const uint8_t  ACTION   = ACTION_START;
const uint16_t PERIOD_S = 5;                  // Time-triggered recording period

// Prints degrees * 1e7 with 7 decimals
void printE7(int32_t v) {
  if (v < 0) { Serial.print('-'); v = -v; }
  Serial.print(v / 10000000L);
  Serial.print('.');
  int32_t frac = v % 10000000L;
  for (int32_t d = 1000000L; d >= 1 && frac < d; d /= 10) {
    Serial.print('0');                        // Leading zeros of the fraction
  }
  if (frac != 0) Serial.print(frac);
}

// One CSV line per record: unix time, fix, lat, lon, height m, sats
void onRecord(const LC76G_LocusRecord &rec) {
  Serial.print(rec.utc);        Serial.print(',');
  Serial.print(rec.fixType);    Serial.print(',');
  printE7(rec.latE7);           Serial.print(',');
  printE7(rec.lonE7);           Serial.print(',');
  Serial.print(rec.heightRaw);  Serial.print(',');
  Serial.println(rec.sats);
}

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
  Serial.println(F("=== Quectel_LCx6G_GNSS Locus ==="));
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

  uint32_t count = 0;
  gnss.getLocusRecordCount(count);
  Serial.print(F("Records stored: "));
  Serial.println(count);

  if (ACTION == ACTION_START) {
    gnss.setLocusMode(LC76G_LOCUS_TIME, false);           // Also without fix
    gnss.setLocusThreshold(LC76G_LOCUS_BY_TIME, PERIOD_S);
    Serial.print(F("Starting recording... "));
    Serial.println(LC76G::resultText(gnss.setLocusEnabled(true)));
  } else if (ACTION == ACTION_DUMP) {
    LC76G_LocusDumpStats st;
    Serial.println(F("unix_time,fix,latitude,longitude,height_m,sats"));
    LC76G_Result r = gnss.dumpLocus(onRecord, st);
    Serial.print(F("Announced "));
    Serial.print(st.announced);
    Serial.print(F(", received "));
    Serial.print(st.received);
    Serial.print(F(", result "));
    Serial.println(LC76G::resultText(r));
  } else {
    Serial.print(F("Erasing... "));
    Serial.println(LC76G::resultText(gnss.clearLocus(LC76G_LOCUS_CLEAR_DATA)));
  }
}

void loop() {
  gnss.update();
}
