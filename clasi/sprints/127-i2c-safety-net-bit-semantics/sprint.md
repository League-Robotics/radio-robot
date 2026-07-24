---
id: '127'
title: I2C safety-net bit semantics
status: roadmap
branch: sprint/127-i2c-safety-net-bit-semantics
worktree: false
use-cases: []
issues:
- i2c-safety-net-bit-conflates-otos-settle-wait-with-loop-schedule-health.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 127: I2C safety-net bit semantics

> Roadmap-level plan (Phase 1). Architecture, use cases, and tickets are
> filled in at detail-planning time. NOTE: needs a stakeholder DESIGN DECISION
> at detail time (which of four candidate fixes) plus hardware bench verify.

## Goals

Make `flags` bit 6 (`kFlagFaultI2CSafetyNet`) mean what it was meant to mean —
"the loop was supposed to own this gap" (a genuine loop-schedule violation) —
instead of conflating that with `Devices::Otos`'s own expected, self-contained
bus-settle wait.

## Problem

120-003's on-chip trace (pyOCD/DBG) conclusively determined the root cause:
`Devices::Otos::readPositionVelocity()`/`readReg8()`/`readXYH()` issue a
register-select `write()` immediately followed by a `read()` on the SAME device
with NO intervening loop-scheduled gap, so `MicroBitI2CBus::waitForClearance()`
trips on every Otos burst read, unconditionally, at Otos's ~20ms cadence,
regardless of drive state. `NezhaMotor`'s split-phase
`requestEncoder()`/`collectEncoder()` DOES cross a real loop-scheduled gap and
contributes ZERO trips. The counter is monotonically non-decreasing and never
reset, so the bit derived from `count() > 0` saturates within ~1s of boot and
can never again surface a genuine NEW motor-side regression — the signal is
permanently swamped. 120-003 deliberately shipped NO fix because every
candidate requires a real design decision.

## Candidate fixes (STAKEHOLDER DECISION required at detail time)

From the issue — pick one at detail planning:

- **(a) Redesign Otos's read pattern** to split register-select write and data
  read across a real loop-scheduled gap (mirror the motor's split-phase shape),
  so it no longer trips the shared safety net. Real hardware-timing risk to a
  currently-working, bench-proven sensor path.
- **(b) Per-device trip accounting.** Extend `MicroBitI2CBus`'s per-device
  `DeviceSlot` with a per-device trip counter; derive the fault bit from
  non-OTOS devices only (or expose both aggregate and "excluding known
  self-contained devices"). Confined to `microbit_i2c_bus.{h,cpp}` +
  `robot_loop.cpp`'s derivation; needs a policy call on which devices are
  "expected" to self-trip.
- **(c) Caller-intent flag.** Add an optional `selfContainedWait` parameter to
  `I2CBus::write()`/`read()`; `Otos`'s helpers pass it true. Touches
  `otos.cpp`'s call sites (arguments only, zero timing change) + the interface
  (`i2c_bus.h`) + implementation.
- **(d) Windowed/reset-per-emit counter** — already evaluated and REJECTED by
  120-003 as insufficient alone (Otos's trips recur faster than the telemetry
  period). Recorded here only so it is not re-proposed.

## Success Criteria

- A stakeholder decision selects one candidate (b/c strongly favored as
  lower-risk than (a); (d) rejected), recorded in the detail plan.
- After the fix: on real hardware, `flags` bit 6 is clear during normal idle
  AND driving operation, and provably re-asserts on an injected/simulated
  genuine loop-schedule violation (or an equivalent stated test of "the bit now
  tracks motor-path schedule health, not Otos's self-contained wait").
- Verified on real hardware idle AND driving per
  `.claude/rules/debugging.md` / `.claude/rules/hardware-bench-testing.md`,
  the same way 120-003 traced it.
- `src/firm/app/DESIGN.md` sec 4 and `src/firm/app/telemetry.h`'s
  `kFlagFaultI2CSafetyNet` doc comment updated to the corrected semantics.

## Scope

### In Scope

- The chosen candidate's implementation: `devices/microbit_i2c_bus.{h,cpp}`
  and/or `devices/otos.{h,cpp}` and/or `devices/i2c_bus.h` +
  `app/robot_loop.cpp`'s bit-6 derivation, per the decision.
- Doc corrections (DESIGN.md sec 4, telemetry.h comment).
- Hardware bench verify (idle + driving) via pyOCD/DBG.

### Out of Scope

- Any change to Otos's actual pose/velocity ACCURACY or the motor split-phase
  schedule (untouched unless candidate (a) is chosen).
- Reviving the never-called `resetStats()` as the sole fix (option (d),
  rejected).

## Dependencies / Sequencing

- **Independent** of 121/122/123/124/125/126. Builds directly on 120-003's
  diagnosis (same robot, same DBG method). Can run any time.

## Architecture

Deferred to detail planning. Expected tier: compact-to-substantial depending on
the chosen candidate (b/c are confined to devices + one derivation site; (a)
would touch the sensor timing path and is riskier). Architecture section sized
to the decision once made.

## Use Cases

Deferred to detail planning (internal diagnostic-correctness property; likely a
single SUC in the shape of 120's SUC-071).

## Tickets

Deferred to detail planning.
