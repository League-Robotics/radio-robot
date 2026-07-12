---
id: '003'
title: 'Per-motor acceleration: Hal::Motor base policy + MotorState.acceleration'
status: open
use-cases: [SUC-004]
depends-on: []
github-issue: ''
issue: restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Per-motor acceleration: Hal::Motor base policy + MotorState.acceleration

## Description

`Hal::Motor` leaves (`NezhaMotor`, `SimMotor`) already sample position and
compute filtered velocity every `tick()`, but nothing computes or
publishes acceleration per-port. `Subsystems::Drivetrain` separately
computes its OWN acceleration EMA (`updateAccelEma()`) for the bound
drive pair only, feeding `DrivetrainState.acc_[]` — already live on TLM's
`acc_left`/`acc_right`.

This ticket adds a **second, deliberately separate, generic** surface
(architecture-update.md's D3/Decision 3): `Hal::Motor::trackAcceleration
(velocity, dtUs)`, a concrete base-class method (EMA, alpha=0.25 —
matching `Drivetrain`'s own proven value), called once by each leaf's own
`tick()` right after it refreshes its filtered velocity. This is
per-motor policy in the base class per project convention
(`.clasi/knowledge/motor-armor-policy-lives-in-base.md`) — available for
all 4 ports, not only the bound drive pair (unlike `DrivetrainState.
acc_[]`, which structurally cannot be). `DrivetrainState.acc_[]`/TLM's
`acc_left`/`acc_right` are **not** touched or rewired by this ticket —
see architecture-update.md's Decision 3 for the explicit rationale (two
parallel, currently-identical-formula sources, by design, flagged as Open
Question 2 for a possible future consolidation, not this sprint's job).

No wire schema change beyond one new `MotorState` field — this is a pure
diagnostic addition, independent of every other ticket in this sprint
(no dependency on `MainLoop`, `PoseEstimator`, or OTOS work).

## Acceptance Criteria

- [ ] `Hal::Motor` gains a protected `float acceleration_ = 0.0f;` field,
      a concrete `void trackAcceleration(float velocity, uint32_t dtUs)`
      (EMA, alpha=0.25), and a public `float acceleration() const`.
- [ ] `Hal::NezhaMotor::tick()` calls `trackAcceleration(velocity(),
      dtUs)` immediately after its own filtered-velocity update (the line
      that updates `filteredVelocity_`, inside the `sampleOk &&
      haveElapsed` branch), using the SAME raw microsecond delta
      (`nowUs - lastTickUs_`) it already computes there for its own
      elapsed-time math — before `updateWedgeDetector()`.
- [ ] `Hal::SimMotor::tick()` calls `trackAcceleration(velocity(), dtUs)`
      immediately after its own filtered-velocity update (the `elapsedTime
      > 0.0f` branch), converting its existing millisecond `elapsedMs` to
      microseconds (`elapsedMs * 1000`) — before the mode-dispatch switch.
- [ ] `Hal::Motor::state()` gains `s.acceleration.has = true;
      s.acceleration.val = acceleration();` inside the existing
      `caps.has_encoder` block, alongside `position`/`velocity`/`wedged`.
- [ ] `protos/motor.proto`'s `MotorState` gains `optional float
      acceleration = 12; // [mm/s^2] EMA-filtered (Hal::Motor::
      trackAcceleration(), base policy, alpha=0.25)`; regenerate
      `source/messages/motor.h` (`scripts/gen_messages.py`) — never
      hand-edited.
- [ ] `bb.drivetrain.acc_left`/`acc_right` (TLM) are byte-identical to a
      pre-ticket build — no regression (verify via existing TLM-reading
      sim tests, if any exercise these fields, or a bench spot-check).
- [ ] Extended `motor_policy_harness.cpp`: acceleration EMA responds
      plausibly (correct sign, settles toward zero) to a velocity ramp
      up/down/hold sequence, for both `NezhaMotor` and `SimMotor` leaves.
- [ ] Bench light: on a duty-step ramp (0 -> nonzero), `bb.motors[i].
      acceleration` (via `GET`/binary `config`-adjacent read, or a
      dedicated bench probe) rises then settles — plausible values, not a
      full regression sweep.

## Implementation Plan

**Approach**: add the base-class EMA method (mirrors
`Drivetrain::updateAccelEma()`'s own formula, ported not re-derived), wire
one call site per leaf at the exact insertion points confirmed by reading
each leaf's current `tick()` (see below), extend the message plane
(`state()`) and the generated schema.

**Files to modify**:
- `source/hal/capability/motor.h` — `acceleration_` field,
  `trackAcceleration()`, `acceleration()` getter, `state()` extension.
- `source/hal/nezha/nezha_motor.cpp` — one `trackAcceleration(velocity(),
  nowUs - lastTickUs_)` call, inserted right after the filtered-velocity
  update (confirmed insertion point: after the line that sets
  `filteredVelocity_` inside `if (sampleOk && haveElapsed)`, before
  `updateWedgeDetector()`).
- `source/hal/sim/sim_motor.cpp` — one `trackAcceleration(velocity(),
  elapsedMs * 1000)` call, inserted right after the filtered-velocity
  update (confirmed insertion point: after the line that sets
  `filteredVelocity_` inside `if (elapsedTime > 0.0f)`, before the
  mode-dispatch switch).
- `protos/motor.proto` — `MotorState.acceleration = 12`.
- Regenerate `source/messages/motor.h` via `scripts/gen_messages.py` —
  never hand-edit.

**Files NOT to modify**: `source/subsystems/drivetrain.{h,cpp}` (its own
`acc_[]`/`updateAccelEma()` stay exactly as they are — Decision 3).

**Testing plan**:
- Extend `tests/sim/unit/motor_policy_harness.cpp` with the EMA
  acceleration cases described above.
- Run the full sim suite — confirm zero regressions in any existing
  `DrivetrainState.acc_[]`/TLM-reading test.
- Bench light verification per the acceptance criteria.

**Documentation updates**: none beyond the generated schema/`.proto`
comment (self-documenting field).
