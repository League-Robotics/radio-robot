---
status: pending
filed: 2026-07-23
filed_by: programmer (120-003 diagnosis)
related:
- restore-the-interleaved-request-settle-tick-loop-schedule.md
---

# I2C safety-net fault bit conflates Otos's own settle-wait with loop-schedule health

## Background

120-003 (`clasi/sprints/120-bench-tour-bring-up-with-fake-otos/tickets/done/003-i2c-safety-net-fault-bit-diagnose-whether-bit-6-reflects-live-bus-health-or-a-latched-boot-artifact.md`)
traced `flags` bit 6 (`kFlagFaultI2CSafetyNet`) on real hardware via
pyOCD/DBG and conclusively determined the root cause. Full evidence is in
that ticket's own record and in `src/firm/app/DESIGN.md`'s
`kFlagFaultI2CSafetyNet` entry (§4). Summary:

- The bit is derived from `MicroBitI2CBus::clearanceSafetyNetCount() > 0`
  — a monotonically non-decreasing counter, never reset (`resetStats()`
  exists, is never called in production).
- Hardware trace (2026-07-23, robot "tovez", `/dev/cu.usbmodem2121102`):
  the counter is NOT a boot-time one-shot latch (a prior doc claim, now
  corrected) — it climbs continuously for as long as the robot runs,
  idle or driving.
- Root cause, confirmed by an exact 1:1 accounting (two independent
  windows: Δ243 over ~14s, Δ148 over ~8.6s, each exactly matching half of
  `Devices::Otos`'s own transaction-count delta):
  `Devices::Otos::readPositionVelocity()`/`readReg8()`/`readXYH()` issue
  a register-select `write()` immediately followed by a `read()` on the
  SAME device with NO intervening loop-scheduled gap — so
  `MicroBitI2CBus::waitForClearance()` trips on every single Otos burst
  read, unconditionally, at Otos's own ~20ms cadence, regardless of drive
  state (`Otos::tick()` runs every cycle unconditionally).
  `NezhaMotor`'s own split-phase `requestEncoder()`/`collectEncoder()`
  DOES cross a real loop-scheduled gap (118-001's `kSettle`/`kClear`
  restore) and contributes ZERO measured trips in either an idle or a
  driving window.

## The problem this leaves

The fault bit currently conflates two unrelated situations under one
counter:

1. A genuine loop-schedule violation (e.g. a future motor-path
   regression) — the bit's actual intended purpose ("the loop was
   supposed to own this gap" per `microbit_i2c_bus.cpp`'s own doc
   comment).
2. `Devices::Otos`'s own expected, necessary, self-contained bus-settle
   wait — not a defect, but it saturates the counter to a nonzero value
   within about a second of boot (any boot with the real OTOS present)
   and never clears, so the bit can never again show a genuine NEW
   motor-side regression — the signal is permanently swamped.

120-003 deliberately shipped NO fix, because every candidate requires a
real design decision.

## Candidate fixes for a future ticket/stakeholder decision

- **(a) Redesign Otos's read pattern** to split the register-select
  write and the data read across a real loop-scheduled gap (mirroring
  the motor's `requestEncoder()`/`collectEncoder()` split-phase shape),
  so it no longer trips the shared safety net at all. Real
  hardware-timing risk to a currently-working, bench-proven sensor path
  — needs its own bench-verified ticket, not a guess.
- **(b) Per-device trip accounting.** Extend `MicroBitI2CBus`'s existing
  per-device `DeviceSlot` (already keyed by 7-bit address, already
  tracks `txnCount`/`errCount`) with a per-device trip counter, and
  derive the fault bit from non-OTOS devices only (or expose both an
  aggregate and a "excluding known-self-contained devices" count).
  Confines the change to `microbit_i2c_bus.{h,cpp}` + `robot_loop.cpp`'s
  derivation, no OTOS timing change — but requires a policy decision
  about which device(s) are "expected" to self-trip.
- **(c) Caller-intent flag.** Add an optional parameter to
  `I2CBus::write()`/`read()` (e.g. `selfContainedWait`) that a caller
  sets to declare "this clearance wait, if any, is my own expected
  requirement, not a loop-schedule dependency" — `Otos`'s helpers would
  pass it true. Requires touching `otos.cpp`'s call sites (arguments
  only, zero timing/behavior change) plus the interface (`i2c_bus.h`)
  and implementation.
- **(d) Windowed/reset-per-emit counter** (the option 120-003 evaluated
  and rejected as insufficient alone): resetting the counter each
  telemetry window does NOT achieve "bit clears during driving" on its
  own, since Otos's own trips recur far faster than the telemetry period
  — it would only stop a truly one-shot boot trip from latching forever,
  which is not the actual failure mode measured here.

Whichever direction, verify on real hardware afterward (idle AND
driving) per `.claude/rules/debugging.md`/
`.claude/rules/hardware-bench-testing.md`, the same way 120-003 did.

## Related

- `clasi/sprints/120-bench-tour-bring-up-with-fake-otos/tickets/done/003-i2c-safety-net-fault-bit-diagnose-whether-bit-6-reflects-live-bus-health-or-a-latched-boot-artifact.md`
  (the diagnosis ticket)
- `clasi/sprints/done/118-loop-schedule-truth-firmware-loop-reorder-sim-cadence-parity/tickets/done/001-restore-the-interleaved-request-settle-tick-loop-schedule.md`
  (corrected acceptance record)
- `src/firm/app/telemetry.h`'s `kFlagFaultI2CSafetyNet` doc comment and
  `src/firm/app/DESIGN.md`'s §4 entry (corrected characterization)
