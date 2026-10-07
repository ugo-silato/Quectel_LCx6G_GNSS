/*
 * ============================================================================
 *  LowPower.ino  -  Quectel_LC76G library example
 * ============================================================================
 *  Shows the low power modes of the module. Choose ONE with MODE below:
 *    MODE_ALP1     Adaptive Low Power mode 1 (lowest power)
 *    MODE_ALP2     Adaptive Low Power mode 2 (better performance)
 *    MODE_PERIODIC run / sleep cycles
 *    MODE_BACKUP   30 s normal, 30 s Backup (VCC off, V_BCKP only), wake-up
 *  Typical currents (datasheet, module alone): continuous ~9.7 mA,
 *  ALP2 ~7.5 mA, ALP1 ~3.8 mA, Backup ~13 uA. Real values depend on the
 *  board and must be measured in a laboratory.
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

#define MODE_ALP1     1
#define MODE_ALP2     2
#define MODE_PERIODIC 3
#define MODE_BACKUP   4
const uint8_t MODE = MODE_ALP1;

// Periodic: smart mode, 10 s run / 30 s sleep with signal,
// 30 s run / 60 s sleep without signal
const LC76G_PeriodicConfig PERIODIC_CFG = { LC76G_PERIODIC_SMART, 10, 30, 30, 60 };

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
  Serial.println(F("=== Quectel_LC76G LowPower ==="));
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

  // Known starting point: periodic off, normal navigation, ALP off
  gnss.disablePeriodicMode(60000UL);
  gnss.setNavigationMode(LC76G_NAV_NORMAL);   // Required by ALP
  gnss.setAlpMode(0);

  LC76G_Result r = LC76G_OK;
  switch (MODE) {
    case MODE_ALP1:     Serial.print(F("ALP mode 1... "));   r = gnss.setAlpMode(1); break;
    case MODE_ALP2:     Serial.print(F("ALP mode 2... "));   r = gnss.setAlpMode(2); break;
    case MODE_PERIODIC: Serial.print(F("Periodic mode... ")); r = gnss.setPeriodicMode(PERIODIC_CFG); break;
    default:            Serial.print(F("Continuous, Backup in 30 s... ")); break;
  }
  Serial.println(LC76G::resultText(r));
}

void loop() {
  gnss.update();

  if (MODE != MODE_BACKUP) {
    return;
  }
  // Backup test, run once: enter after 30 s, wake up 30 s later.
  // No delay(): gnss.update() must keep running while waiting.
  static uint8_t step = 0;
  static unsigned long t0 = millis();
  if (step == 0 && millis() - t0 > 30000UL) {
    Serial.print(F("Entering Backup mode (VCC off)... "));
    Serial.println(LC76G::resultText(gnss.enterBackup(0)));
    step = 1;
    t0 = millis();
  } else if (step == 1 && millis() - t0 > 30000UL) {
    Serial.print(F("Leaving Backup mode... "));
    Serial.println(LC76G::resultText(gnss.exitBackup()));
    step = 2;
  }
}
