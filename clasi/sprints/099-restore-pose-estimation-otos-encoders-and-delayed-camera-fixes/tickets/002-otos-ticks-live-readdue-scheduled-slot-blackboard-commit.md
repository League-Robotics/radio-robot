---
id: '002'
title: 'OTOS ticks live: readDue() scheduled slot + Blackboard commit'
status: open
use-cases: [SUC-001]
depends-on: ['001']
github-issue: ''
issue: restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS ticks live: readDue() scheduled slot + Blackboard commit

## Description

`Hal::OtosOdometer::begin()` runs at boot but `tick()` is never called by
the live loop — `bb.otos`/`bb.otosConnected`/`bb.otosPresent` sit at their
zero defaults forever. A prior attempt to tick it unconditionally,
per-pass, directly from the loop (ticket 098-004, mirroring the pre-093
`dev_loop.cpp` pattern) caused a real bus-hang-class regression (a live
turn over-rotated `-90 -> -192deg`) because OTOS (I2C 0x17) traffic could
start while an outstanding Nezha motor (I2C 0x10) 0x46 request was still
in its settle window — see `.clasi/knowledge/
otos-per-pass-i2c-tick-wrecks-motion-timing.md` and
architecture-update.md's Decision 2.

This ticket implements D2: the OTOS tick becomes a **scheduled slot
inside `NezhaHardware::tick()`** — structurally, not probabilistically,
ruled out from ever landing inside a 0x10 request/collect window — plus
commits the resulting raw OTOS state onto the Blackboard every pass.
`Telemetry::tick()`/`buildTelemetryMessage()` (`source/telemetry/
tlm_frame.cpp`) already read `bb.otos`/`bb.otosConnected` (gated on
`bb.otosPresent`) — this ticket needs **zero changes** to those files;
`otos=`/`otosconn=` light up on TLM automatically once these cells are
committed.

This ticket does **not** touch `PoseEstimator` or `bb.otosValid` (the
fusion-gating flag `Hal::Odometer::fusableThisPass()` produces) — that is
ticket 004/007's job, once `PoseEstimator` is actually consuming an OTOS
observation. This ticket is purely: "the chip gets read safely, and its
raw reading + connection state are visible."

## Acceptance Criteria

- [ ] `Hal::OtosOdometer` gains a new public `bool readDue(uint32_t now)
      const` query (`!hasRead_ || (now - lastReadMs_) >= kReadPeriod`,
      signed-cast rollover-safe, matching the project's established
      uint32-ms-subtraction convention).
- [ ] `NezhaHardware::tick()` gains one new branch at its top: when
      `phase_ == Phase::REQUEST_DUE && otosOdometer_.readDue(now)`, this
      call services the OTOS (`otosOdometer_.tick(now)`) and returns
      immediately — never entering the existing flip-flop switch this
      call. The existing flip-flop switch is otherwise byte-identical.
- [ ] `Rt::MainLoop::commit()` gains: `bb.otos =
      hardware_.odometer()->pose();` and `bb.otosConnected =
      hardware_.odometer()->connected();`, every pass.
- [ ] `bb.otosPresent` is seeded exactly once, at boot, immediately after
      `hardware.begin()` (both `main.cpp` and `tests/_infra/sim/
      sim_api.cpp`'s `SimHandle` constructor): `bb.otosPresent =
      hardware.odometer()->connected();`.
- [ ] `SimHardware`/`SimOdometer` need no matching change (already ticks
      internally, per `nezha_hardware.h`'s own D2 rationale note) — verify
      by reading, not assuming.
- [ ] New/extended `otos_odometer_harness.cpp` case(s) for `readDue()`:
      false immediately after a real read, true once `kReadPeriod` has
      elapsed, true before any read has ever happened.
- [ ] New/extended `nezha_flipflop_harness.cpp` case(s): the OTOS slot
      never fires while `phase_ == COLLECT_DUE`; at most one OTOS slot
      services per `kReadPeriod` window; the Nezha flip-flop's own
      request/collect cadence is otherwise unchanged by the new branch.
- [ ] **BENCH MANDATORY**: sustained (>=10 minute) bench session with
      0x17 (OTOS) and 0x10 (Nezha motor) traffic interleaved — zero bus
      hangs, verified via `robot_radio`'s `NezhaProtocol` (never lock-step
      pyserial, per prior bench-session lessons).
- [ ] The SAME session, with motion commands running throughout (binary
      `drive`/`segment`), shows no motion-timing regression versus a
      pre-ticket baseline — the 098-004 hazard class does not reproduce.
- [ ] `TLM`/binary `stream` shows `otosconn=`/`otos=` live-updating (or a
      truthful `false`/omitted if no chip is detected) on the bench.

## Implementation Plan

**Approach**: (1) add the `readDue()` query to `OtosOdometer`
(`source/hal/otos/otos_odometer.{h,cpp}`) — a pure function of its
existing private `hasRead_`/`lastReadMs_` fields, no new state. (2) Add
the scheduled-slot branch to `NezhaHardware::tick()`
(`source/subsystems/nezha_hardware.cpp`) exactly as specified in
architecture-update.md's D2 code block. (3) Extend `Rt::MainLoop::commit()`
(`source/runtime/main_loop.cpp`) with the two new `bb.otos*` assignments.
(4) Seed `bb.otosPresent` once in `source/main.cpp` and `tests/_infra/
sim/sim_api.cpp`'s `SimHandle::SimHandle()`, immediately after
`hardware.begin()`.

**Files to modify**:
- `source/hal/otos/otos_odometer.h` — declare `readDue()`.
- `source/hal/otos/otos_odometer.cpp` — implement `readDue()`.
- `source/subsystems/nezha_hardware.cpp` — new top-of-`tick()` branch.
- `source/runtime/main_loop.cpp` — `commit()` gains `bb.otos`/
  `bb.otosConnected`.
- `source/main.cpp` — seed `bb.otosPresent` once, post-`begin()`.
- `tests/_infra/sim/sim_api.cpp` — seed `bb.otosPresent` once,
  post-`hardware.begin()`, mirroring `main.cpp`.

**Files NOT to modify**: `source/telemetry/tlm_frame.{h,cpp}`,
`source/telemetry/telemetry_tick.cpp`, `protos/telemetry.proto` — already
correct (verified by reading, D9 in architecture-update.md).

**Testing plan**:
- Extend `tests/sim/unit/otos_odometer_harness.cpp` for `readDue()`.
- Extend `tests/sim/unit/nezha_flipflop_harness.cpp` for the
  never-fires-during-COLLECT_DUE and at-most-one-per-`kReadPeriod`
  invariants.
- Bench session per the acceptance criteria above — this is the sprint's
  first mandatory bench gate; do not skip or shorten the >=10 minute
  window (the hazard this closes is specifically a low-probability,
  long-session bus-timing failure).

**Documentation updates**: none required this ticket (the TLM field
semantics are unchanged, only their liveness).
