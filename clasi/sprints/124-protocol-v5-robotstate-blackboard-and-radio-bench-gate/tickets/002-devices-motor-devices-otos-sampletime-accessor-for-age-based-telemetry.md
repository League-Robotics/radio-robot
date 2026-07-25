---
id: '002'
title: 'Devices::Motor / Devices::Otos: sampleTime() accessor for age-based telemetry'
status: done
use-cases:
- SUC-006
depends-on: []
github-issue: ''
issue: protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Devices::Motor / Devices::Otos: sampleTime() accessor for age-based telemetry

## Description

Independent prerequisite ticket — "do this first," per the protocol-v5
issue's own B2 section. Today neither `Devices::Motor` nor
`Devices::Otos` exposes when its current reading was actually taken;
telemetry stamps `enc_left.time`/`enc_right.time` with the SAME `now`
for both wheels even though the brick's bus choreography samples them
8-12ms apart (`robot_loop.cpp`'s request/settle/collect sequence). This
ticket adds the one accessor honest `age` fields need.

Add `sampleTime()` [us] to `Devices::Motor` and `Devices::Otos` (and
their sim/fake counterparts, plus `MotorArmor`'s passthrough), returning
the `nowUs` of the tick that produced the currently-cached reading — not
"now" at call time.

## Acceptance Criteria

- [x] `Motor::sampleTime()` and `Otos::sampleTime()` declared (pure
      virtual, matching `position()`/`velocity()`'s existing shape) and
      implemented in `NezhaMotor`, the sim/fake motor, and both the real
      and fake `Otos` implementations.
- [x] `MotorArmor::sampleTime()` forwards to `inner_.sampleTime()`, same
      as its other passthrough accessors.
- [x] The returned value reflects the actual tick timestamp of the last
      ACCEPTED fresh sample (matching `lastFreshUs_`'s existing role in
      `NezhaMotor`), not the current cycle's `now`.
- [x] Existing device unit tests pass unmodified; new tests cover the
      accessor directly.

## Testing

- **Existing tests to run**:
  `src/tests/sim/unit/devices_motor_harness.cpp`,
  `src/tests/sim/unit/app_fake_otos_harness.cpp`.
- **New tests to write**: assert `sampleTime()` differs from the
  cycle's `now` by a realistic skew after a fresh sample is collected,
  and that it stays fixed between fresh samples.
- **Verification command**: `uv run pytest` and the sim-tests C++ build
  (`just build-tests` or equivalent per `src/tests/sim/`).
