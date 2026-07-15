// i2c_bus.h — Devices::I2CBus: thin diagnostic wrapper around MicroBitI2C.
//
// Ticket DB-003 (device-bus-tickets.md). Ported from source/com/i2c_bus.h
// into the greenfield `source/devices/` subsystem (namespace `Devices`) per
// clasi/issues/device-bus-fiber-owned-self-contained-device-subsystem.md's
// "Shape" / "Sole-ownership rule": after cutover this is the ONLY class in
// the firmware that performs an I2C transaction. Re-cased to the project's
// CamelCase rule (naming-and-style.md) on the way in — private data members
// take the trailing-underscore form (`bus_`, not `_bus`); the class name and
// public method names are unchanged (`I2CBus` keeps its all-caps acronym;
// every method was already lowerCamelCase). Behavior is otherwise ported
// verbatim, in particular the IRQ guard default-ON (non-negotiable —
// `.claude/rules/naming-and-style.md`'s "Armor stays intact" precedent, and
// the nRF52 TWIM errata NRF52I2C::waitForStop documents) and the lazy
// preClear/postClear clearance timers.
//
// All four device leaves this subsystem owns (Motor, Otos, LineSensorLeaf,
// ColorSensorLeaf — DB-004..DB-006) route their bus traffic through this
// wrapper so every transaction is counted, errors are tracked, and
// potential re-entrancy violations are captured — without changing the
// semantics of any transaction.
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
// Usage: one I2CBus instance, owned by the loop, constructed before any
// device leaf and passed to each by reference — mirrors the ported source's
// own `static I2CBus bus(uBit.i2c);` usage note.
//
// Thread safety:
//   The inUse_ flag window is intentionally narrow (3-4 instructions) so the
//   critical section is safe on Cortex-M4. All counter increments happen
//   outside IRQ-disabled sections and are therefore NOT ISR-safe themselves —
//   they are meant for cooperative-loop diagnostics only (concurrency
//   contract rule 3: "Nothing here is ISR-safe, by design").
#pragma once
#ifndef HOST_BUILD
#include "MicroBit.h"
#else
#include <deque>
#include <vector>
#endif
#include <cstdint>

namespace Devices {

class I2CBus {
 public:
  // ---------------------------------------------------------------------
  // Construction
  // ---------------------------------------------------------------------

#ifndef HOST_BUILD
  explicit I2CBus(MicroBitI2C& bus);
#else
  // HOST_BUILD scripted fake — no MicroBitI2C to wire. See the
  // "HOST_BUILD scripted-fake surface" section below (source/devices/
  // i2c_bus_host.cpp) for the script/clock API this fork adds.
  I2CBus();
#endif

  // ---------------------------------------------------------------------
  // I2C forwarding — mirror MicroBitI2C signatures exactly.
  //
  // address: 8-bit wire address (7-bit addr << 1), as the callers pass it.
  // Returns: CODAL status int (MICROBIT_OK == 0 on success).
  //
  // preClear/postClear (// [us], default 0): lazy per-device clearance
  // timers. At entry, BEFORE the re-entrancy guard's critical section, the
  // call spins until max(slot.readyAt, slot.lastEnd + preClear); after the
  // transaction, slot.lastEnd is stamped to now and slot.readyAt to
  // lastEnd + postClear. Defaults collapse the entry deadline to lastEnd,
  // already in the past by the next call, so every 4-argument call site is
  // byte-identical to before this parameter pair existed.
  // ---------------------------------------------------------------------

  int write(uint16_t address, uint8_t* data, int len, bool repeated = false,
            uint32_t preClear = 0, uint32_t postClear = 0);
  int read(uint16_t address, uint8_t* data, int len, bool repeated = false,
           uint32_t preClear = 0, uint32_t postClear = 0);

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
  // Clearance safety-net diagnostics (103-002, M1 fix — 2026-07-13 code
  // review). write()/read()'s entry-side clearance wait used to be a hard
  // `while(clockUs()<deadline){}` spin with no yield — up to ~4ms of
  // scheduler-blocking spin nearly every cycle (the motor1-duty-write ->
  // motor2-request hot site the review flagged; both motors share address
  // 0x10, so motor1's postClear stamp is almost always still in motor2's
  // future). In the single-loop design the LOOP is meant to own this gap
  // (explicit runAndWait/sleepUntil calls, ticket 008's own scope) — this
  // per-device readyAt stamp is the backstop for a caller that still
  // arrives early. It never spins: see i2c_bus.cpp's (real fork,
  // fiber_sleep()) and i2c_bus_host.cpp's (HOST_BUILD fork, a direct fake-
  // clock jump) own write()/read() for the exact non-spinning mechanism.
  // ---------------------------------------------------------------------

  // Total number of times write()/read() found itself called BEFORE a
  // device's clearance deadline (readyAt/preClear) had elapsed. This is the
  // narrow signal ticket 001 already numbered as Telemetry.fault_bits bit 0
  // ("I2CBus readyAt clearance safety-net trip (ticket 002/005)") — it
  // "should never fire if the loop schedule is right" (the issue's own
  // words). Wiring the actual fault_bits write is ticket 005's job:
  // source/app/'s telemetry-population code doesn't exist yet as of this
  // ticket (002), only source/devices/ does.
  uint32_t clearanceSafetyNetCount() const { return clearanceSafetyNetCount_; }

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

#ifdef HOST_BUILD
  // ---------------------------------------------------------------------
  // HOST_BUILD scripted-fake surface (source/devices/i2c_bus_host.cpp) —
  // never compiled alongside the real i2c_bus.cpp. Real MicroBitI2C traffic
  // is replaced by a scripted, in-order FIFO of expected transactions; the
  // fake still runs the exact lastEnd/readyAt clearance bookkeeping in
  // write()/read() above against an injectable, steppable clock — no
  // wall-clock reads, no real sleeps — so a host test can assert
  // clearance-timer behavior deterministically.
  // ---------------------------------------------------------------------

  // Script the next write() call (FIFO order): returns `status`
  // (0 == OK). A write() whose address doesn't match what was scripted,
  // or one with no script queued, returns a distinct mismatch status
  // rather than crashing the test process.
  void scriptWrite(uint16_t address, int status = 0);

  // Script the next read() call (FIFO order): returns `status` and
  // copies up to `len` bytes of `data` into the caller's buffer.
  void scriptRead(uint16_t address, const uint8_t* data, int len,
                   int status = 0);

  // Test-only fake clock — HOST_BUILD has no wall clock to read. Starts
  // at 0; set or advance it explicitly to script clearance deadlines
  // deterministically. A single counter shared by every I2CBus instance
  // in the process — a live entry-spin inside write()/read() also
  // self-advances this by 1us per iteration so a scripted preClear/
  // postClear deadline always terminates even if a test forgets to
  // advance the clock itself.
  static void setClock(uint64_t us);      // [us]
  static void advanceClock(uint64_t us);  // [us]
  static uint64_t clock();                // [us]
#endif

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

#ifndef HOST_BUILD
  MicroBitI2C& bus_;
#else
  // HOST_BUILD scripted-fake queues (FIFO) — see scriptWrite()/scriptRead().
  struct ScriptedWrite {
    uint16_t addr;    // expected 8-bit wire address
    int status;       // status to return
  };
  struct ScriptedRead {
    uint16_t addr;              // expected 8-bit wire address
    std::vector<uint8_t> data;  // canned response bytes
    int status;                 // status to return
  };
  std::deque<ScriptedWrite> scriptedWrites_;
  std::deque<ScriptedRead> scriptedReads_;
#endif

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

  // clockUs — current time [us]. The real fork wraps
  // system_timer_current_time_us(); the HOST_BUILD fork returns a
  // test-settable fake counter (source/devices/i2c_bus_host.cpp) instead of
  // a wall clock. Every "now" read in this class (entry-spin check,
  // lastEnd/readyAt stamping, clear()'s peek, the transaction log
  // timestamp) goes through this single point so the two forks stay
  // structurally parallel.
  static uint64_t clockUs();

  // waitForClearance — write()/read()'s entry-side clearance wait (103-002,
  // M1 fix). If entryDeadline is still in the future, bumps
  // clearanceSafetyNetCount_ and waits out the shortfall WITHOUT spinning:
  // the real fork (i2c_bus.cpp) sleeps via fiber_sleep() — the same
  // cooperative primitive clock.h's Sleeper wraps, rounded up to whole
  // milliseconds (fiber_sleep() reliably sleeps at least the requested
  // duration, so rounding up never shortchanges the real vendor clearance
  // requirement docs/knowledge/2026-07-04-encoder-wedge.md documents); the
  // HOST_BUILD fork (i2c_bus_host.cpp) jumps its fake clock directly to
  // entryDeadline in one step (no wall clock, no fiber scheduler to sleep
  // against). A no-op (no counter bump, no wait) if entryDeadline has
  // already elapsed.
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
