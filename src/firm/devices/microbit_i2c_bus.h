// microbit_i2c_bus.h — Devices::MicroBitI2CBus: the real ARM implementation
// of Devices::I2CBus, wrapping MicroBitI2C.
//
// Sprint 108 ticket 001 split. This class holds the machinery that used to
// live directly in `Devices::I2CBus` before that header was reduced to a
// pure interface (source/devices/i2c_bus.h): the MicroBitI2C& member,
// re-entrancy guard, lazy preClear/postClear clearance timers, per-device
// stats, the transaction ring log, and the IRQ guard. Moved verbatim from
// the old i2c_bus.cpp/.h (ticket DB-003, device-bus-tickets.md) — no
// behavior change, only the class name and file split.
//
// Behavior is otherwise ported verbatim, in particular the IRQ guard
// default-ON (non-negotiable — `.claude/rules/naming-and-style.md`'s "Armor
// stays intact" precedent, and the nRF52 TWIM errata NRF52I2C::waitForStop
// documents) and the lazy preClear/postClear clearance timers.
//
// All four device leaves this subsystem owns (Motor, Otos, LineSensorLeaf,
// ColorSensorLeaf) route their bus traffic through this wrapper (held via
// an `I2CBus&` reference) so every transaction is counted, errors are
// tracked, and potential re-entrancy violations are captured — without
// changing the semantics of any transaction.
//
// Re-entrancy guard (diagnostic, NOT a lock):
//   inUse_ is checked and set atomically via target_disable_irq() /
//   target_enable_irq() around the flag access only (NOT around the full
//   I2C transaction unless irqGuard_ is on). If a second call enters while
//   the first is still in flight (e.g. an ISR-context call or a spurious
//   re-entry), the violation counter is incremented and the address pair is
//   captured. The transaction proceeds normally — the guard records the
//   concurrency violation but does NOT block or skip any I2C traffic.
//
// Per-device counters:
//   Keyed by the 7-bit device address (the address before the caller
//   left-shifts it). Known devices:
//     0x10 — Nezha V2 motor controller (Motor)
//     0x17 — OTOS odometry sensor (Odometer)
//     0x1A — line sensor (LineSensorLeaf)
//     0x39 — APDS9960 color sensor (ColorSensorLeaf, fallback variant)
//     0x43 — alt color sensor (ColorSensorLeaf, primary variant)
//   Unrecognised addresses are accumulated in an "other" bucket (index
//   kMaxDevices-1) so the table never overflows.
//
// Usage: one MicroBitI2CBus instance, owned by main(), constructed before
// any device leaf and passed to each by reference (as an `I2CBus&`) —
// mirrors the ported source's own `static I2CBus bus(uBit.i2c);` usage
// note, now `static Devices::MicroBitI2CBus bus(uBit.i2c);` (source/
// main.cpp).
//
// Thread safety:
//   The inUse_ flag window is intentionally narrow (3-4 instructions) so the
//   critical section is safe on Cortex-M4. All counter increments happen
//   outside IRQ-disabled sections and are therefore NOT ISR-safe themselves —
//   they are meant for cooperative-loop diagnostics only (concurrency
//   contract rule 3: "Nothing here is ISR-safe, by design").
#pragma once
#include "MicroBit.h"
#include "devices/i2c_bus.h"
#include <cstdint>

namespace Devices {

class MicroBitI2CBus : public I2CBus {
 public:
  // ---------------------------------------------------------------------
  // Construction
  // ---------------------------------------------------------------------

  explicit MicroBitI2CBus(MicroBitI2C& bus);

  // ---------------------------------------------------------------------
  // I2CBus interface
  // ---------------------------------------------------------------------

  int write(uint16_t address, uint8_t* data, int len, bool repeated = false,
             uint32_t preClear = 0, uint32_t postClear = 0) override;
  int read(uint16_t address, uint8_t* data, int len, bool repeated = false,
            uint32_t preClear = 0, uint32_t postClear = 0) override;

  uint32_t clearanceSafetyNetCount() const override {
    return clearanceSafetyNetCount_;
  }

  // ---------------------------------------------------------------------
  // Per-device statistics (keyed by 7-bit address)
  // ---------------------------------------------------------------------

  // Total transactions (write + read) for the device at addr.
  uint32_t txnCount(uint16_t addr) const;

  // Total error transactions (status != MICROBIT_OK) for the device.
  uint32_t errCount(uint16_t addr) const;

  // Last non-OK CODAL status returned for the device. 0 if no errors.
  int lastErr(uint16_t addr) const;

  // ---------------------------------------------------------------------
  // Lazy per-device clearance timers — non-spinning peek.
  // ---------------------------------------------------------------------

  // True if the device at the bare 7-bit addr7 is past its clearance
  // deadline (write()/read()'s postClear-derived readyAt) — the
  // non-spinning counterpart to the entry-side spin inside write()/
  // read(). Callers poll this before a would-be-blocking collect to
  // decide whether the wait has already elapsed. Takes the SAME 7-bit
  // convention as txnCount()/errCount()/lastErr() — NOT the 8-bit wire
  // address write()/read() take (an easy off-by-one-bit trap). A device
  // never transacted with (no slot yet) is always clear.
  bool clear(uint16_t addr7) const;

  // ---------------------------------------------------------------------
  // Re-entrancy diagnostics
  // ---------------------------------------------------------------------

  // Total number of re-entrancy violations detected since construction
  // (or last resetStats()).
  uint32_t reentryViolations() const { return reentryViolations_; }

  // 8-bit wire address that was in flight when the most recent violation
  // was detected.
  uint16_t reentryInFlightAddr() const { return reentryInFlightAddr_; }

  // 8-bit wire address of the new caller that triggered the most recent
  // violation.
  uint16_t reentryNewAddr() const { return reentryNewAddr_; }

  // ---------------------------------------------------------------------
  // Utility
  // ---------------------------------------------------------------------

  // Reset all counters and violation state to zero.
  void resetStats();

  // ---------------------------------------------------------------------
  // Transaction log (diagnostic) — a ring buffer of the most recent
  // transactions (addr, R/W, len, first 2 bytes, status, timestamp). OFF by
  // default (zero overhead); armed/dumped on demand to inspect exactly what
  // was on the bus and in what order (addr/RW/byte/dt).
  // ---------------------------------------------------------------------

  // Dump the recent transaction ring (chronological) via fn/ctx.
  void dumpRecent(void (*fn)(const char*, void*), void* ctx) const;

  // Enable/disable transaction logging (off by default).
  void setLogging(bool on) { logOn_ = on; }

  // IRQ-guard the FULL transaction (not just the inUse_ flag). The nRF52
  // TWIM has a silicon errata (see NRF52I2C::waitForStop) that strikes
  // "under higher levels of background interrupt load"; masking interrupts
  // for the duration of each transaction removes that load. Default ON —
  // non-negotiable (issue "Armor stays intact"). Toggle live for bench A/B
  // against the wedge.
  void setIrqGuard(bool on) { irqGuard_ = on; }
  bool irqGuard() const { return irqGuard_; }

 private:
  // Maximum number of distinct devices tracked (including the "other"
  // bucket at index kMaxDevices-1).
  static constexpr int kMaxDevices = 8;

  struct DeviceSlot {
    uint16_t addr;     // 7-bit address (0 = empty slot)
    uint32_t txnCount;
    uint32_t errCount;
    int lastErr;
    uint64_t lastEnd;  // [us] end time of the most recent transaction to this device
    uint64_t readyAt;  // [us] max(lastEnd, previous readyAt) + that transaction's postClear
  };

  MicroBitI2C& bus_;

  // Re-entrancy guard state.
  volatile bool inUse_;
  uint16_t inFlightAddr_;  // wire address currently in flight (for guard)

  // Violation capture.
  uint32_t reentryViolations_;
  uint16_t reentryInFlightAddr_;
  uint16_t reentryNewAddr_;

  // Clearance safety-net trip count — see the public accessor's own comment
  // above for what this feeds.
  uint32_t clearanceSafetyNetCount_;

  // Per-device slot table.
  DeviceSlot devices_[kMaxDevices];
  int deviceCount_;

  // Transaction log ring buffer.
  struct TxnLog {
    uint32_t t;      // [us] timestamp
    uint16_t addr;   // 7-bit device address
    uint8_t rw;      // 0 = write, 1 = read
    uint8_t len;     // transfer length
    uint8_t b0, b1;  // first two data bytes (command/result)
    int16_t status;  // CODAL status
  };
  static constexpr int kLogSize = 24;
  TxnLog log_[kLogSize];
  int logHead_;         // next slot to write
  uint32_t logTotal_;   // total logged (for chronological ordering)
  bool logOn_;
  bool irqGuard_;        // mask IRQs for the full transaction (TWIM errata fix)

  // Append one transaction to the ring (no-op if logging off).
  void logTxn(uint16_t addr7, uint8_t rw, int len, const uint8_t* data,
              int status);

  // ---------------------------------------------------------------------
  // Private helpers
  // ---------------------------------------------------------------------

  // clockUs — current time [us], wraps system_timer_current_time_us().
  // Every "now" read in this class (entry-spin check, lastEnd/readyAt
  // stamping, clear()'s peek, the transaction log timestamp) goes through
  // this single point.
  static uint64_t clockUs();

  // waitForClearance — write()/read()'s entry-side clearance wait (103-002,
  // M1 fix). If entryDeadline is still in the future, bumps
  // clearanceSafetyNetCount_ and waits out the shortfall WITHOUT spinning:
  // sleeps via fiber_sleep() — the same cooperative primitive clock.h's
  // Sleeper wraps, rounded up to whole milliseconds (fiber_sleep() reliably
  // sleeps at least the requested duration, so rounding up never
  // shortchanges the real vendor clearance requirement
  // docs/knowledge/2026-07-04-encoder-wedge.md documents). A no-op (no
  // counter bump, no wait) if entryDeadline has already elapsed.
  void waitForClearance(uint64_t entryDeadline);

  // findOrAdd — return the slot index for the given 7-bit address.
  //
  // If the address already has a slot, that index is returned. If not, a
  // new slot is allocated (up to kMaxDevices-1). If the table is full, the
  // last slot (the "other" bucket) is returned so counters never overflow.
  int findOrAdd(uint16_t addr7);

  // record — update per-device counters after a transaction completes.
  //
  // addr7:   7-bit device address.
  // status:  CODAL status int from the underlying MicroBitI2C call.
  void record(uint16_t addr7, int status);
};

}  // namespace Devices
