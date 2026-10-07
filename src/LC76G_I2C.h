/*
 * LC76G_I2C.h - I2C port for the Quectel LC76G (LC26G (AB) / LC26G-T (AA)
 *               / LC76G series), usable in place of a serial port:
 *
 *     #include <Wire.h>
 *     #include <Quectel_LC76G.h>
 *     #include <LC76G_I2C.h>
 *
 *     LC76G_I2C gnssI2c(Wire);
 *     LC76G     gnss;
 *     ...
 *     gnss.powerOn();                  // FIRST: on the reference shield the
 *     delay(LC76G_BOOT_MS);            // I2C pull-ups need the module supply
 *     gnssI2c.begin();                 // Wire.begin() + 100 kHz
 *     gnss.begin(gnssI2c, nullptr);    // nullptr: no baud rate to set
 *
 * The class is a Stream: the library reads NMEA and sends commands through
 * it exactly as with a serial port.
 *
 * Protocol (Quectel I2C Application Note v1.2):
 *   addresses 0x50 = configuration, 0x54 = read, 0x58 = write
 *   read : 1) write {0xAA510008, 4} to 0x50, read 4 bytes at 0x54 = length
 *          2) write {0xAA512000, n} to 0x50, read n bytes at 0x54
 *             (step 2 can be repeated until the length of step 1 is read)
 *   write: 1) write {0xAA510004, 4} to 0x50, read 4 bytes at 0x54 = free space
 *          2) write {0xAA531000, n} to 0x50, write n bytes to 0x58
 *   32-bit words little-endian; at least 10 ms between two transfers
 *   (verified on the bench: without the pause the module does not
 *   acknowledge its address).
 *
 * Header-only on purpose: the Wire library is compiled only into the
 * sketches that include this file, so UART-only sketches do not pay for it.
 *
 * Timing: every transfer is preceded by the 10 ms pause, so reading costs
 * about 20 ms per chunk (32 bytes on the Uno R3, 128 on the Uno R4) and
 * gnss.update() can take a few hundred ms when much data is waiting.
 * Keep the NMEA output small (setMinimalNmeaOutput()), especially on the R3.
 *
 * Voltage: the module I2C pins work at 3.3 V. begin() switches OFF the
 * Arduino internal pull-ups (towards 5 V): the bus must be pulled up to
 * 3.3 V by the board (2.2 k on the reference shield). With a 5 V Arduino
 * the 3.3 V high level is below the guaranteed input threshold of its I2C
 * pins (0.7 x VCC = 3.5 V): it works on the bench, but an I2C level
 * shifter is the safe solution.
 *
 * Author: Ugo Silato
 * License: MIT (see LICENSE file)
 */

#ifndef LC76G_I2C_H
#define LC76G_I2C_H

#include <Arduino.h>
#include <Wire.h>

// Chunk size: limited by the Wire buffer (32 bytes on AVR)
#if defined(ARDUINO_ARCH_RENESAS)
#define LC76G_I2C_CHUNK     128
#else
#define LC76G_I2C_CHUNK     32
#endif
#define LC76G_I2C_GAP_MS    10      // Pause before every transfer (module requirement)
#define LC76G_I2C_TRIES     20      // Attempts per transfer, as in the Quectel sample code
#define LC76G_I2C_POLL_MS   200     // Default minimum interval between two "how many bytes?" polls
#define LC76G_I2C_WAKE_MS   3000    // No data for this long: send a wake-up write
#define LC76G_I2C_BACKOFF_MS 500    // After a failed transfer, no new poll for this long
#define LC76G_I2C_FAST_MS   1500    // After a command, fast polling for this long (reply expected)
#define LC76G_I2C_FAST_POLL_MS 20   // Poll interval during that time

class LC76G_I2C : public Stream {
public:
  // Module addresses (7 bit)
  static const uint8_t ADDR_CFG   = 0x50;
  static const uint8_t ADDR_READ  = 0x54;
  static const uint8_t ADDR_WRITE = 0x58;

  explicit LC76G_I2C(TwoWire &wire = Wire)
    : _wire(wire), _rxLen(0), _rxPos(0), _pending(0), _txLen(0),
      _lastXfer(0), _lastPoll(0), _lastData(0), _lastError(0), _errors(0),
      _pollMs(LC76G_I2C_POLL_MS), _fastUntil(0) {}

  // Starts the I2C master. 'clockHz': 100000 (standard) or 400000 (fast).
  // 'keepInternalPullups' = true only if the bus has no external pull-ups
  // AND the module tolerates the Arduino voltage (not the case at 5 V).
  void begin(uint32_t clockHz = 100000, bool keepInternalPullups = false) {
    _wire.begin();
    _wire.setClock(clockHz);
    if (!keepInternalPullups) {
      disableInternalPullups();
    }
    _lastData = millis();
  }

  // Number of transfers that failed after all the attempts
  uint16_t errors() const { return _errors; }

  // Minimum interval between two "how many bytes are waiting?" polls when
  // no data is pending (default LC76G_I2C_POLL_MS). Each poll is two
  // transfers with their 10 ms pauses, so it blocks the sketch for about
  // 20 ms: a longer interval leaves more time to the sketch and less
  // activity on the bus. At 1 Hz, 200..1000 ms is enough (the module keeps
  // up to 9216 bytes waiting). Replies to commands are not delayed: after
  // a command the port polls at the fast rate for LC76G_I2C_FAST_MS.
  // Bench test (Uno R4, reference shield): 20 ms and 1000 ms gave the same
  // reception (SNR 27 dB-Hz, fix) as the UART.
  void setPollInterval(uint16_t ms) { _pollMs = ms; }

  // ---- Stream interface (used by the LC76G class) ----

  int available() override {
    if (_rxPos >= _rxLen) {
      refill();                                // Buffer empty: ask the module
    }
    return (int)(_rxLen - _rxPos);
  }

  int read() override {
    if (available() == 0) {
      return -1;
    }
    return _rx[_rxPos++];
  }

  int peek() override {
    if (available() == 0) {
      return -1;
    }
    return _rx[_rxPos];
  }

  // Bytes are collected and sent at the end of each line ('\n') or when
  // the piece is full: a command reaches the module in one or more pieces.
  size_t write(uint8_t c) override {
    _tx[_txLen++] = c;
    if (c == '\n' || _txLen >= sizeof(_tx)) {
      flush();
    }
    return 1;
  }
  using Print::write;                          // Keeps write(buffer, size) etc.

  void flush() override {
    if (_txLen > 0) {
      sendPiece(_tx, _txLen);
      _txLen = 0;
    }
  }

private:
  TwoWire      &_wire;
  uint8_t       _rx[LC76G_I2C_CHUNK];          // Received data
  uint8_t       _rxLen;                        // Valid bytes in _rx
  uint8_t       _rxPos;                        // Next byte to return
  uint32_t      _pending;                      // Bytes still to read (from step 1)
  uint8_t       _tx[LC76G_I2C_CHUNK - 1];      // Outgoing piece (fits one Wire transfer)
  uint8_t       _txLen;
  unsigned long _lastXfer;                     // millis() of the last transfer
  unsigned long _lastPoll;                     // millis() of the last length poll
  unsigned long _lastData;                     // millis() of the last data received
  unsigned long _lastError;                    // millis() of the last failed transfer
  uint16_t      _errors;
  uint16_t      _pollMs;                       // Minimum interval between polls
  unsigned long _fastUntil;                    // Fast polling until this millis() (reply to a command)

  // Waits until LC76G_I2C_GAP_MS have passed since the previous transfer
  void gap() {
    while (millis() - _lastXfer < LC76G_I2C_GAP_MS) {
    }
  }

  // Writes 'len' bytes to 'addr', with pause and retries
  bool tx(uint8_t addr, const uint8_t *data, uint8_t len) {
    for (uint8_t t = 0; t < LC76G_I2C_TRIES; t++) {
      gap();
      _wire.beginTransmission(addr);
      _wire.write(data, len);
      uint8_t e = _wire.endTransmission();
      _lastXfer = millis();
      if (e == 0) {
        return true;                           // Every byte acknowledged
      }
    }
    _errors++;
    _lastError = millis();
    return false;
  }

  // Reads exactly 'n' bytes from 'addr', with pause and retries
  bool rx(uint8_t addr, uint8_t *buf, uint8_t n) {
    for (uint8_t t = 0; t < LC76G_I2C_TRIES; t++) {
      gap();
      uint8_t got = (uint8_t)_wire.requestFrom(addr, n);
      _lastXfer = millis();
      if (got == n) {
        for (uint8_t i = 0; i < n; i++) {
          buf[i] = (uint8_t)_wire.read();
        }
        return true;
      }
      while (_wire.available()) {
        _wire.read();                          // Drop a partial answer
      }
    }
    _errors++;
    _lastError = millis();
    return false;
  }

  // Configuration command: two 32-bit little-endian words to 0x50
  bool config(uint32_t cmd, uint32_t len) {
    uint8_t b[8];
    for (uint8_t i = 0; i < 4; i++) {
      b[i]     = (uint8_t)(cmd >> (8 * i));
      b[4 + i] = (uint8_t)(len >> (8 * i));
    }
    return tx(ADDR_CFG, b, 8);
  }

  // Reads a 32-bit little-endian value from 0x54
  bool readWord(uint32_t &v) {
    uint8_t b[4];
    if (!rx(ADDR_READ, b, 4)) {
      return false;
    }
    v = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    return true;
  }

  // Fills _rx with the next chunk of data, if any
  void refill() {
    _rxLen = 0;
    _rxPos = 0;
    if (_pending == 0) {
      // Read step 1, not more often than every _pollMs, and not soon
      // after a failed transfer (module off or bus problem): otherwise
      // every call would block for all the attempts
      // After a command the reply must arrive within the library timeout
      // (1 s): poll at the fast rate for a while, whatever _pollMs is
      uint16_t interval = ((long)(_fastUntil - millis()) > 0) ? LC76G_I2C_FAST_POLL_MS : _pollMs;
      if (millis() - _lastPoll < interval) {
        return;
      }
      if (_errors > 0 && millis() - _lastError < LC76G_I2C_BACKOFF_MS) {
        return;
      }
      _lastPoll = millis();
      uint32_t len = 0;
      if (!config(0xAA510008UL, 4) || !readWord(len)) {
        return;
      }
      _pending = len;
      if (_pending == 0) {
        // Nothing for a long time: the module stops its I2C transmitter
        // when its buffer overflows; any complete write wakes it up
        if (millis() - _lastData > LC76G_I2C_WAKE_MS) {
          static const uint8_t wake[2] = { '\r', '\n' };
          sendPiece(wake, 2);
          _lastData = millis();
        }
        return;
      }
    }
    // Read step 2: one chunk
    uint8_t n = (_pending > LC76G_I2C_CHUNK) ? LC76G_I2C_CHUNK : (uint8_t)_pending;
    if (!config(0xAA512000UL, n) || !rx(ADDR_READ, _rx, n)) {
      _pending = 0;                            // Start again from step 1
      return;
    }
    _pending -= n;
    _rxLen = n;
    _lastData = millis();
  }

  // Write flow for one piece (len <= LC76G_I2C_CHUNK - 1)
  void sendPiece(const uint8_t *data, uint8_t len) {
    for (uint8_t t = 0; t < LC76G_I2C_TRIES; t++) {
      uint32_t freeLen = 0;
      if (!config(0xAA510004UL, 4) || !readWord(freeLen)) {
        return;                                // Counted in _errors
      }
      if (freeLen >= len) {
        if (config(0xAA531000UL, len)) {
          tx(ADDR_WRITE, data, len);
        }
        _fastUntil = millis() + LC76G_I2C_FAST_MS;   // A reply is coming
        return;
      }
      // Receive buffer of the module almost full: ask again
    }
    _errors++;
    _lastError = millis();                     // Start the back-off of refill()
  }

  // Switches off the Arduino internal pull-ups on SDA / SCL
  static void disableInternalPullups() {
#if defined(ARDUINO_ARCH_RENESAS)
    // RA4M1: clear the PCR (pull-up control) bit of the two pins
    R_BSP_PinAccessEnable();
    bsp_io_port_pin_t sda = g_pin_cfg[SDA].pin;
    bsp_io_port_pin_t scl = g_pin_cfg[SCL].pin;
    R_PFS->PORT[sda >> 8].PIN[sda & 0xFF].PmnPFS_b.PCR = 0;
    R_PFS->PORT[scl >> 8].PIN[scl & 0xFF].PmnPFS_b.PCR = 0;
    R_BSP_PinAccessDisable();
#elif defined(ARDUINO_ARCH_AVR)
    // AVR: with the TWI enabled, writing LOW to the port removes the pull-up
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
#endif
  }
};

#endif  // LC76G_I2C_H
