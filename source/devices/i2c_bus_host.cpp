// i2c_bus_host.cpp — HOST_BUILD scripted-fake implementation of
// Devices::I2CBus. Ported from source/com/i2c_bus_host.cpp; see i2c_bus.h's
// file header for the port/re-casing notes.
//
// Compiled ONLY when HOST_BUILD is defined, and NEVER linked alongside the
// real source/devices/i2c_bus.cpp (both files define the same
// Devices::I2CBus:: symbols; linking both into one binary is a build-
// configuration error, not a supported dual-build). This gives every
// source/devices/ host harness a dependency-free I2CBus to script against —
// no MicroBitI2C, no CODAL, no wall clock.
//
// Scripting model: a test pre-loads expected (address, status) tuples for
// writes and (address, bytes, status) tuples for reads via scriptWrite()/
// scriptRead(), in the exact order production code is expected to call
// write()/read(). Each call pops the next scripted entry (FIFO); an
// unscripted call, or one whose address doesn't match, returns a distinct
// mismatch status instead of crashing the test process (see kScriptMismatch
// below) — this fails the specific assertion the test cares about (a
// status/errCount() check) rather than aborting the whole harness.
//
// Clock: HOST_BUILD has no wall clock, so the exact lastEnd/readyAt
// clearance-timer bookkeeping in write()/read() (identical logic to the
// real fork — see i2c_bus.cpp) runs against a static, test-settable
// counter (I2CBus::setClock()/advanceClock()/clock()). A live entry-spin
// self-advances the counter by 1us per iteration so a scripted preClear/
// postClear deadline always terminates even if a test forgets to advance
// the clock itself — a safety net for callers that never trigger a live
// spin (every scripted scenario either uses the defaults, or advances the
// clock explicitly before the next call).
#include "devices/i2c_bus.h"
#include <cstdio>

namespace Devices {
namespace {
// Distinct from any real CODAL status — a scripted call with no queued
// script, or a wrong-address script, returns this rather than silently
// returning "OK" or crashing.
constexpr int kScriptMismatch = -100;

// HOST_BUILD fake clock — see the "Clock" section above. A single counter
// shared by every I2CBus instance in the process; starts at 0.
uint64_t g_fakeClockUs = 0;  // [us]
}  // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

I2CBus::I2CBus()
    : inUse_(false),
      inFlightAddr_(0),
      reentryViolations_(0),
      reentryInFlightAddr_(0),
      reentryNewAddr_(0),
      deviceCount_(0),
      logHead_(0),
      logTotal_(0),
      logOn_(false),
      irqGuard_(true) {
  for (int i = 0; i < kMaxDevices; ++i) {
    devices_[i].addr = 0;
    devices_[i].txnCount = 0;
    devices_[i].errCount = 0;
    devices_[i].lastErr = 0;
    devices_[i].lastEnd = 0;
    devices_[i].readyAt = 0;
  }
  for (int i = 0; i < kLogSize; ++i) {
    log_[i] = TxnLog{0, 0, 0, 0, 0, 0, 0};
  }
}

// ---------------------------------------------------------------------------
// Clock
// ---------------------------------------------------------------------------

uint64_t I2CBus::clockUs() { return g_fakeClockUs; }

void I2CBus::setClock(uint64_t us) { g_fakeClockUs = us; }

void I2CBus::advanceClock(uint64_t us) { g_fakeClockUs += us; }

uint64_t I2CBus::clock() { return g_fakeClockUs; }

// ---------------------------------------------------------------------------
// I2C forwarding — scripted fake, no MicroBitI2C
// ---------------------------------------------------------------------------

int I2CBus::write(uint16_t address, uint8_t* data, int len, bool repeated,
                   uint32_t preClear, uint32_t postClear) {
  (void)repeated;
  (void)data;
  (void)len;

  // address is the 8-bit wire address (7-bit addr << 1) — same convention
  // as the real fork.
  uint16_t addr7 = static_cast<uint16_t>(address >> 1);

  // Same lazy-clearance entry spin as the real fork (i2c_bus.cpp), against
  // the fake clock instead of system_timer_current_time_us(). See the file
  // header for why this self-advances rather than truly blocking.
  int idx = findOrAdd(addr7);
  uint64_t entryDeadline = devices_[idx].readyAt;
  uint64_t preDeadline = devices_[idx].lastEnd + static_cast<uint64_t>(preClear);
  if (preDeadline > entryDeadline) entryDeadline = preDeadline;
  while (clockUs() < entryDeadline) {
    advanceClock(1);
  }

  int status = kScriptMismatch;
  if (!scriptedWrites_.empty()) {
    ScriptedWrite expected = scriptedWrites_.front();
    scriptedWrites_.pop_front();
    status = (expected.addr == address) ? expected.status : kScriptMismatch;
  }

  record(addr7, status);
  logTxn(addr7, 0, len, data, status);

  devices_[idx].lastEnd = clockUs();
  devices_[idx].readyAt = devices_[idx].lastEnd + static_cast<uint64_t>(postClear);

  return status;
}

int I2CBus::read(uint16_t address, uint8_t* data, int len, bool repeated,
                  uint32_t preClear, uint32_t postClear) {
  (void)repeated;

  uint16_t addr7 = static_cast<uint16_t>(address >> 1);

  int idx = findOrAdd(addr7);
  uint64_t entryDeadline = devices_[idx].readyAt;
  uint64_t preDeadline = devices_[idx].lastEnd + static_cast<uint64_t>(preClear);
  if (preDeadline > entryDeadline) entryDeadline = preDeadline;
  while (clockUs() < entryDeadline) {
    advanceClock(1);
  }

  int status = kScriptMismatch;
  if (!scriptedReads_.empty()) {
    ScriptedRead expected = scriptedReads_.front();
    scriptedReads_.pop_front();
    if (expected.addr == address) {
      status = expected.status;
      int copyLen = (len < static_cast<int>(expected.data.size()))
                        ? len
                        : static_cast<int>(expected.data.size());
      for (int i = 0; i < copyLen; ++i) {
        data[static_cast<size_t>(i)] = expected.data[static_cast<size_t>(i)];
      }
    }
  }

  record(addr7, status);
  logTxn(addr7, 1, len, data, status);

  devices_[idx].lastEnd = clockUs();
  devices_[idx].readyAt = devices_[idx].lastEnd + static_cast<uint64_t>(postClear);

  return status;
}

// ---------------------------------------------------------------------------
// Scripting
// ---------------------------------------------------------------------------

void I2CBus::scriptWrite(uint16_t address, int status) {
  scriptedWrites_.push_back(ScriptedWrite{address, status});
}

void I2CBus::scriptRead(uint16_t address, const uint8_t* data, int len,
                         int status) {
  ScriptedRead entry;
  entry.addr = address;
  entry.status = status;
  if (data && len > 0) {
    entry.data.assign(data, data + len);
  }
  scriptedReads_.push_back(entry);
}

// ---------------------------------------------------------------------------
// Transaction log (diagnostic ring buffer) — identical logic to the real
// fork (i2c_bus.cpp), against the fake clock.
// ---------------------------------------------------------------------------

void I2CBus::logTxn(uint16_t addr7, uint8_t rw, int len, const uint8_t* data,
                     int status) {
  if (!logOn_) return;
  TxnLog& e = log_[logHead_];
  e.t = static_cast<uint32_t>(clockUs());  // [us]
  e.addr = addr7;
  e.rw = rw;
  e.len = static_cast<uint8_t>(len > 255 ? 255 : (len < 0 ? 0 : len));
  e.b0 = (rw == 0 && len > 4 && data) ? data[4] : (data && len > 0) ? data[0] : 0;
  e.b1 = (data && len > 1) ? data[1] : 0;
  e.status = static_cast<int16_t>(status);
  logHead_ = (logHead_ + 1) % kLogSize;
  ++logTotal_;
}

void I2CBus::dumpRecent(void (*fn)(const char*, void*), void* ctx) const {
  if (!fn || !ctx) return;
  int count = (logTotal_ < static_cast<uint32_t>(kLogSize)) ? static_cast<int>(logTotal_) : kLogSize;
  int start = (logTotal_ < static_cast<uint32_t>(kLogSize)) ? 0 : logHead_;
  char line[256];
  uint32_t prevTime = 0;  // [us]
  int pos = snprintf(line, sizeof(line), "I2CLOG ");
  for (int i = 0; i < count; ++i) {
    const TxnLog& e = log_[(start + i) % kLogSize];
    uint32_t dt = (i == 0) ? 0 : (e.t - prevTime);
    prevTime = e.t;
    int w = snprintf(line + pos, sizeof(line) - pos, "%02X%c%02X.%lu ",
                      static_cast<unsigned>(e.addr), e.rw ? 'R' : 'W',
                      static_cast<unsigned>(e.b0), static_cast<unsigned long>(dt));
    if (w <= 0 || w >= static_cast<int>(sizeof(line)) - pos) break;  // out of room
    pos += w;
  }
  snprintf(line + pos, sizeof(line) - pos, "\r\n");
  fn(line, ctx);
}

// ---------------------------------------------------------------------------
// Per-device statistics — identical logic to the real fork.
// ---------------------------------------------------------------------------

uint32_t I2CBus::txnCount(uint16_t addr) const {
  for (int i = 0; i < deviceCount_; ++i) {
    if (devices_[i].addr == addr) return devices_[i].txnCount;
  }
  return 0;
}

uint32_t I2CBus::errCount(uint16_t addr) const {
  for (int i = 0; i < deviceCount_; ++i) {
    if (devices_[i].addr == addr) return devices_[i].errCount;
  }
  return 0;
}

int I2CBus::lastErr(uint16_t addr) const {
  for (int i = 0; i < deviceCount_; ++i) {
    if (devices_[i].addr == addr) return devices_[i].lastErr;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Lazy per-device clearance timers — non-spinning peek
// ---------------------------------------------------------------------------

bool I2CBus::clear(uint16_t addr7) const {
  for (int i = 0; i < deviceCount_; ++i) {
    if (devices_[i].addr == addr7) {
      return clockUs() >= devices_[i].readyAt;
    }
  }
  return true;  // never transacted with — nothing to wait for
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

void I2CBus::resetStats() {
  reentryViolations_ = 0;
  reentryInFlightAddr_ = 0;
  reentryNewAddr_ = 0;
  inUse_ = false;
  inFlightAddr_ = 0;

  for (int i = 0; i < kMaxDevices; ++i) {
    devices_[i].addr = 0;
    devices_[i].txnCount = 0;
    devices_[i].errCount = 0;
    devices_[i].lastErr = 0;
    devices_[i].lastEnd = 0;
    devices_[i].readyAt = 0;
  }
  deviceCount_ = 0;

  logHead_ = 0;
  logTotal_ = 0;
}

// ---------------------------------------------------------------------------
// Private helpers — identical logic to the real fork.
// ---------------------------------------------------------------------------

int I2CBus::findOrAdd(uint16_t addr7) {
  for (int i = 0; i < deviceCount_; ++i) {
    if (devices_[i].addr == addr7) return i;
  }

  if (deviceCount_ < kMaxDevices - 1) {
    int idx = deviceCount_++;
    devices_[idx].addr = addr7;
    devices_[idx].txnCount = 0;
    devices_[idx].errCount = 0;
    devices_[idx].lastErr = 0;
    devices_[idx].lastEnd = 0;
    devices_[idx].readyAt = 0;
    return idx;
  }

  int overflow = kMaxDevices - 1;
  if (devices_[overflow].addr == 0) {
    devices_[overflow].addr = 0xFFFF;
    deviceCount_ = kMaxDevices;
  }
  return overflow;
}

void I2CBus::record(uint16_t addr7, int status) {
  int idx = findOrAdd(addr7);
  ++devices_[idx].txnCount;
  if (status != 0) {  // 0 == OK; HOST_BUILD has no MicroBit.h MICROBIT_OK,
                       // but CODAL's own convention is MICROBIT_OK == 0.
    ++devices_[idx].errCount;
    devices_[idx].lastErr = status;
  }
}

}  // namespace Devices
