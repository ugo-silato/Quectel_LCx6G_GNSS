/*
 * ============================================================================
 *  LC76G_Jamming.cpp  -  Jamming detection for the LC76G
 * ============================================================================
 *  Part of the Quectel_LCx6G_GNSS Arduino library. Author: Ugo Silato. License: MIT.
 *
 *  Jamming = a radio signal (intentional or accidental) that covers the very
 *  weak GNSS signals. The LC76G can detect it and report it in two ways:
 *    - $PAIRSPF,<status> messages on the UART, once detection is enabled
 *      with PAIR391 (decoded in LC76G::handleSentence)
 *    - the JAM_IND output pin: HIGH = no jamming, LOW = jamming detected
 *
 *  Commands used (Quectel GNSS Protocol Specification):
 *    PAIR391  Enable/disable jamming detection
 * ============================================================================
 */

#include "LC76G.h"

LC76G_Result LC76G::setJammingDetection(bool enable) {
  LC76G_Result r = sendPair(391, enable ? "1" : "0");
  if (r == LC76G_OK && !enable) {
    _jamStatus   = LC76G_JAM_UNKNOWN;          // No more reports will arrive
    _jamStatusMs = 0;
  }
  return r;
}

void LC76G::setJamIndicatorPin(int8_t pin) {
  _jamPin = pin;
  if (_jamPin >= 0) {
    pinMode(_jamPin, INPUT);                   // Plain input: NO pull-up to 5 V
  }
}

bool LC76G::jamIndicatorActive() const {
  if (_jamPin < 0) {
    return false;
  }
  return digitalRead(_jamPin) == LOW;          // Active low: LOW = jamming
}
