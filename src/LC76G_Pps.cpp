/*
 * ============================================================================
 *  LC76G_Pps.cpp  -  1PPS (One Pulse Per Second) for the LC76G
 * ============================================================================
 *  Part of the Quectel_LC76G Arduino library. Author: Ugo Silato. License: MIT.
 *
 *  1PPS is a pulse output synchronised to the start of each UTC second.
 *  It is used to discipline clocks or to timestamp events precisely.
 *
 *  Commands used (Quectel GNSS Protocol Specification):
 *    PQTMCFGPPS  Set / get the 1PPS configuration
 *      W,1,<Enable>[,<Duration>,<Mode>,<Polarity>,<Interval>]
 *      <Interval> is reserved (always 0) on the LC76G.
 *
 *  Optional pulse capture: an interrupt on CHANGE timestamps both edges
 *  with micros(), giving period and pulse width without an oscilloscope.
 * ============================================================================
 */

#include "LC76G.h"
#include <stdlib.h>   // atoi()

// ---------------------------------------------------------------------------
// Capture state. Static because an interrupt routine cannot be a member
// function: only one LC76G object can use the capture at a time.
// 'volatile' because the values are written inside the interrupt.
// ---------------------------------------------------------------------------
static volatile uint32_t s_count     = 0;     // Pulses counted
static volatile uint32_t s_lastStart = 0;     // micros() of the last active edge
static volatile uint32_t s_period    = 0;     // Last period, us
static volatile uint32_t s_width     = 0;     // Last pulse width, us
static volatile uint32_t s_lastMs    = 0;     // millis() of the last active edge
static volatile bool     s_activeHigh = true; // Polarity used to tell the edges apart
static int8_t            s_pin       = -1;    // Capture pin

// Interrupt routine, called on every edge of the 1PPS pin.
// Kept as short as possible: it runs while the main program is stopped.
static void ppsIsr() {
  uint32_t now = micros();
  bool level = digitalRead(s_pin) == HIGH;
  if (level == s_activeHigh) {                // Start of a pulse (active edge)
    if (s_count > 0) {
      s_period = now - s_lastStart;           // Unsigned maths handles micros() wrap-around
    }
    s_lastStart = now;
    s_lastMs = millis();
    s_count++;
  } else if (s_count > 0) {                   // End of the pulse
    s_width = now - s_lastStart;
  }
}

// Reads a 32-bit volatile value atomically. On the 8-bit AVR a 32-bit read
// takes several instructions and the interrupt could change the value in
// the middle: interrupts are paused for those few instructions.
static uint32_t atomicRead(const volatile uint32_t &v) {
  noInterrupts();
  uint32_t copy = v;
  interrupts();
  return copy;
}


// ===========================================================================
//  CONFIGURATION
// ===========================================================================

LC76G_Result LC76G::setPps(bool enable, uint16_t durationMs, LC76G_PpsMode mode, bool activeHigh) {
  char params[32];
  if (enable) {
    if (durationMs < 1 || durationMs > 999 ||
        mode < LC76G_PPS_ALWAYS || mode > LC76G_PPS_AFTER_FIRST_FIX) {
      return LC76G_PARAM_ERROR;
    }
    // W,<Index=1>,<Enable=1>,<Duration>,<Mode>,<Polarity>,<Interval=0>
    snprintf(params, sizeof(params), "W,1,1,%u,%u,%u,0",
             (unsigned)durationMs, (unsigned)mode, activeHigh ? 1u : 0u);
  } else {
    // When disabling, the fields after <Enable> must be omitted (note 1)
    snprintf(params, sizeof(params), "W,1,0");
  }

  LC76G_Result r = sendPqtm("PQTMCFGPPS", params);
  if (r == LC76G_OK && enable) {
    s_activeHigh = activeHigh;                // Keep the capture in step with the polarity
  }
  return r;
}

LC76G_Result LC76G::getPps(LC76G_PpsConfig &cfg) {
  // Reply: PQTMCFGPPS,OK,1,<Enable>[,<Duration>,<Mode>,<Polarity>,<Interval>]
  //  field:    0     1  2    3          4        5       6          7
  char resp[48];
  LC76G_Result r = sendPqtm("PQTMCFGPPS", "R,1", LC76G_CMD_TIMEOUT_MS, resp, sizeof(resp));
  if (r != LC76G_OK) {
    return r;
  }
  char f[6];
  LC76G_Nmea::getField(resp, 3, f, sizeof(f));
  cfg.enabled = (atoi(f) == 1);

  // The other fields may be missing when the output is disabled
  if (LC76G_Nmea::getField(resp, 4, f, sizeof(f)) && f[0] != '\0') {
    cfg.durationMs = (uint16_t)atoi(f);
    LC76G_Nmea::getField(resp, 5, f, sizeof(f));
    cfg.mode = (LC76G_PpsMode)atoi(f);
    LC76G_Nmea::getField(resp, 6, f, sizeof(f));
    cfg.activeHigh = (atoi(f) == 1);
    s_activeHigh = cfg.activeHigh;
  } else {
    cfg.durationMs = 0;
    cfg.mode = LC76G_PPS_ALWAYS;
    cfg.activeHigh = true;
  }
  return r;
}


// ===========================================================================
//  PULSE CAPTURE
// ===========================================================================

bool LC76G::beginPpsCapture(int8_t pin) {
  if (pin < 0) {
    return false;
  }
  int irq = digitalPinToInterrupt(pin);
#ifdef NOT_AN_INTERRUPT
  // AVR (Uno R3): pins without interrupt are reported as NOT_AN_INTERRUPT
  if (irq == NOT_AN_INTERRUPT) {
    return false;
  }
#endif
#if defined(ARDUINO_ARCH_RENESAS)
  // Renesas (Uno R4): digitalPinToInterrupt() returns the pin itself and
  // there is no NOT_AN_INTERRUPT: ask the pin table of the core whether
  // the pin has an external interrupt (on the R4 Minima: D0-D3, D8, D12,
  // D13, A1-A5)
  if (pin >= (int8_t)PINS_COUNT || getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT)[0] == 0) {
    return false;
  }
#endif
  if (s_pin >= 0) {
    detachInterrupt(digitalPinToInterrupt(s_pin));  // Release a previous capture pin
  }
  s_pin = pin;
  pinMode(pin, INPUT);                        // Plain input: never pull up a module output

  noInterrupts();                             // Reset the counters atomically
  s_count = 0;
  s_period = 0;
  s_width = 0;
  s_lastMs = 0;
  interrupts();

  attachInterrupt(irq, ppsIsr, CHANGE);       // Both edges: start and end of the pulse
  return true;
}

uint32_t LC76G::ppsCount() const    { return atomicRead(s_count); }
uint32_t LC76G::ppsPeriodUs() const { return atomicRead(s_period); }
uint32_t LC76G::ppsWidthUs() const  { return atomicRead(s_width); }

uint32_t LC76G::ppsAgeMs() const {
  if (atomicRead(s_count) == 0) {
    return 0xFFFFFFFFUL;                      // No pulse seen yet
  }
  return millis() - atomicRead(s_lastMs);
}
