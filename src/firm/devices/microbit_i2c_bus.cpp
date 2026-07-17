// microbit_i2c_bus.cpp — Devices::MicroBitI2CBus real implementation.
// Design/rationale: DESIGN.md.
#include "devices/microbit_i2c_bus.h"
#include "codal_target_hal.h"  // target_disable_irq() / target_enable_irq()
#include "MicroBit.h"          // system_timer_current_time_us()
#include <cstdio>

namespace Devices {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

MicroBitI2CBus::MicroBitI2CBus(MicroBitI2C& bus)
    : bus_(bus),
      inUse_(false),
      inFlightAddr_(0),
      reentryViolations_(0),
      reentryInFlightAddr_(0),
      reentryNewAddr_(0),
      clearanceSafetyNetCount_(0),
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

uint64_t MicroBitI2CBus::clockUs() { return system_timer_current_time_us(); }

// ---------------------------------------------------------------------------
// I2C forwarding
// ---------------------------------------------------------------------------

int MicroBitI2CBus::write(uint16_t address, uint8_t* data, int len,
                           bool repeated, uint32_t preClear,
                           uint32_t postClear) {
  // address is the 8-bit wire address (7-bit addr << 1).
  uint16_t addr7 = static_cast<uint16_t>(address >> 1);

  // Lazy per-device clearance wait — BEFORE the re-entrancy guard's
  // target_disable_irq() critical section starts (never mask interrupts for
  // a multi-ms wait). Defaults (preClear=postClear=0) collapse
  // entryDeadline to lastEnd, already in the past by the time the NEXT call
  // happens, so every existing 4-argument call site waits zero time.
  int idx = findOrAdd(addr7);
  uint64_t entryDeadline = devices_[idx].readyAt;
  uint64_t preDeadline = devices_[idx].lastEnd + static_cast<uint64_t>(preClear);
  if (preDeadline > entryDeadline) entryDeadline = preDeadline;
  waitForClearance(entryDeadline);

  const bool guard = irqGuard_;

  // Always mask IRQs for the flag check-and-set. When irqGuard_ is on we
  // KEEP them masked through the whole bus_ transaction (nRF52 TWIM errata
  // fix — see microbit_i2c_bus.h / NRF52I2C::waitForStop); when off, we
  // re-enable before the transaction (original narrow-guard behaviour).
  target_disable_irq();
  bool alreadyInUse = inUse_;
  if (alreadyInUse) {
    ++reentryViolations_;
    reentryInFlightAddr_ = inFlightAddr_;
    reentryNewAddr_ = static_cast<uint16_t>(address);
  } else {
    inUse_ = true;
    inFlightAddr_ = static_cast<uint16_t>(address);
  }
  if (!guard) target_enable_irq();

  int status = bus_.write(address, data, len, repeated);

  if (!alreadyInUse) {
    inUse_ = false;
  }
  if (guard) target_enable_irq();

  record(addr7, status);
  logTxn(addr7, 0, len, data, status);

  devices_[idx].lastEnd = clockUs();
  devices_[idx].readyAt = devices_[idx].lastEnd + static_cast<uint64_t>(postClear);

  return status;
}

int MicroBitI2CBus::read(uint16_t address, uint8_t* data, int len,
                          bool repeated, uint32_t preClear,
                          uint32_t postClear) {
  uint16_t addr7 = static_cast<uint16_t>(address >> 1);

  int idx = findOrAdd(addr7);
  uint64_t entryDeadline = devices_[idx].readyAt;
  uint64_t preDeadline = devices_[idx].lastEnd + static_cast<uint64_t>(preClear);
  if (preDeadline > entryDeadline) entryDeadline = preDeadline;
  waitForClearance(entryDeadline);

  const bool guard = irqGuard_;

  target_disable_irq();
  bool alreadyInUse = inUse_;
  if (alreadyInUse) {
    ++reentryViolations_;
    reentryInFlightAddr_ = inFlightAddr_;
    reentryNewAddr_ = static_cast<uint16_t>(address);
  } else {
    inUse_ = true;
    inFlightAddr_ = static_cast<uint16_t>(address);
  }
  if (!guard) target_enable_irq();

  int status = bus_.read(address, data, len, repeated);

  if (!alreadyInUse) {
    inUse_ = false;
  }
  if (guard) target_enable_irq();

  record(addr7, status);
  logTxn(addr7, 1, len, data, status);

  devices_[idx].lastEnd = clockUs();
  devices_[idx].readyAt = devices_[idx].lastEnd + static_cast<uint64_t>(postClear);

  return status;
}

// ---------------------------------------------------------------------------
// Clearance safety-net wait
// ---------------------------------------------------------------------------

void MicroBitI2CBus::waitForClearance(uint64_t entryDeadline) {
  uint64_t now = clockUs();
  if (now >= entryDeadline) return;

  // Entered before the clearance deadline -- the loop was supposed to own
  // this gap (runAndWait/sleepUntil); count the trip (feeds
  // Telemetry.fault_bits bit 0 -- see microbit_i2c_bus.h's accessor
  // comment). NEVER spin: yield the shortfall via fiber_sleep() -- the same
  // cooperative primitive clock.h's Sleeper wraps -- rounded UP to whole
  // milliseconds. fiber_sleep() reliably sleeps AT LEAST the requested
  // duration, so rounding up never shortchanges the real vendor clearance
  // requirement (docs/knowledge/2026-07-04-encoder-wedge.md) -- it only
  // ever waits slightly longer than strictly necessary.
  ++clearanceSafetyNetCount_;
  uint64_t shortfallUs = entryDeadline - now;
  uint32_t shortfallMs = static_cast<uint32_t>((shortfallUs + 999) / 1000);
  if (shortfallMs > 0) fiber_sleep(shortfallMs);
}

// ---------------------------------------------------------------------------
// Transaction log (diagnostic ring buffer)
// ---------------------------------------------------------------------------

void MicroBitI2CBus::logTxn(uint16_t addr7, uint8_t rw, int len,
                             const uint8_t* data, int status) {
  if (!logOn_) return;
  TxnLog& e = log_[logHead_];
  e.t = static_cast<uint32_t>(clockUs());  // [us]
  e.addr = addr7;
  e.rw = rw;
  e.len = static_cast<uint8_t>(len > 255 ? 255 : (len < 0 ? 0 : len));
  // For a Nezha WRITE the meaningful byte is the command at frame byte[4]
  // ("FF F9 id dir <CMD> ..." — 0x46=read-angle-request, 0x60=move, 0x47=read-
  // speed). The header byte[0] is always 0xFF, useless. For a READ, byte[0] is
  // the low data byte. So log byte[4] on writes, byte[0] on reads.
  e.b0 = (rw == 0 && len > 4 && data) ? data[4] : (data && len > 0) ? data[0] : 0;
  e.b1 = (data && len > 1) ? data[1] : 0;
  e.status = static_cast<int16_t>(status);
  logHead_ = (logHead_ + 1) % kLogSize;
  ++logTotal_;
}

void MicroBitI2CBus::dumpRecent(void (*fn)(const char*, void*),
                                 void* ctx) const {
  if (!fn || !ctx) return;
  // Walk the ring oldest->newest. If we've wrapped, oldest is at logHead_;
  // otherwise the buffer filled 0..logHead_-1.
  int count = (logTotal_ < static_cast<uint32_t>(kLogSize)) ? static_cast<int>(logTotal_) : kLogSize;
  int start = (logTotal_ < static_cast<uint32_t>(kLogSize)) ? 0 : logHead_;
  // Emit the WHOLE ring as ONE line — multiple lines overflow the async serial
  // TX buffer (~255 B) and garble. One <=255-char line is safe.
  // Token: <addr><R/W><b0>.<dt_us>  e.g. "10W60.0 10R46.4012 43RA6.250"
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
// Per-device statistics
// ---------------------------------------------------------------------------

uint32_t MicroBitI2CBus::txnCount(uint16_t addr) const {
  for (int i = 0; i < deviceCount_; ++i) {
    if (devices_[i].addr == addr) return devices_[i].txnCount;
  }
  return 0;
}

uint32_t MicroBitI2CBus::errCount(uint16_t addr) const {
  for (int i = 0; i < deviceCount_; ++i) {
    if (devices_[i].addr == addr) return devices_[i].errCount;
  }
  return 0;
}

int MicroBitI2CBus::lastErr(uint16_t addr) const {
  for (int i = 0; i < deviceCount_; ++i) {
    if (devices_[i].addr == addr) return devices_[i].lastErr;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Lazy per-device clearance timers — non-spinning peek
// ---------------------------------------------------------------------------

bool MicroBitI2CBus::clear(uint16_t addr7) const {
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

void MicroBitI2CBus::resetStats() {
  reentryViolations_ = 0;
  reentryInFlightAddr_ = 0;
  reentryNewAddr_ = 0;
  clearanceSafetyNetCount_ = 0;
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
// Private helpers
// ---------------------------------------------------------------------------

int MicroBitI2CBus::findOrAdd(uint16_t addr7) {
  // Linear scan: return existing slot if found.
  for (int i = 0; i < deviceCount_; ++i) {
    if (devices_[i].addr == addr7) return i;
  }

  // Allocate a new slot if there is room (leave the last slot as
  // the "other" bucket so we never exceed the array bounds).
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

  // Table full: use the last slot as the "other" overflow bucket.
  // Mark it addr=0xFFFF so queries for unknown addresses don't
  // collide with real devices (real 7-bit addrs are 0x00..0x7F).
  int overflow = kMaxDevices - 1;
  if (devices_[overflow].addr == 0) {
    devices_[overflow].addr = 0xFFFF;
    deviceCount_ = kMaxDevices;
  }
  return overflow;
}

void MicroBitI2CBus::record(uint16_t addr7, int status) {
  int idx = findOrAdd(addr7);
  ++devices_[idx].txnCount;
  if (status != MICROBIT_OK) {
    ++devices_[idx].errCount;
    devices_[idx].lastErr = status;
  }
}

}  // namespace Devices
