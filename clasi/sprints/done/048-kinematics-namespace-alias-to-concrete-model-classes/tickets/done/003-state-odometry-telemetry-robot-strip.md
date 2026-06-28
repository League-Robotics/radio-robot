---
id: '003'
title: 'State/odometry/telemetry/robot wiring strip: Odometry, PhysicalStateEstimate,
  Robot, RobotTelemetry, Config'
status: done
use-cases:
- SUC-048-002
depends-on:
- '001'
- '002'
issue: eliminate-ifdef-robot-drivetrain-mecanum-everywhere.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

## Description

Strip `#ifdef ROBOT_DRIVETRAIN_MECANUM` from the state, odometry, telemetry, and
robot-wiring layer: `Odometry.h`, `PhysicalStateEstimate.h/.cpp`, `Robot.cpp`,
`RobotTelemetry.cpp`, and `Config.h`.

Specific removals:
- `Odometry.h`: `setOtosAlphaVy`, `fusedVy`, `_fusedVy`, `_otosAlphaVy` — the
  lateral complementary filter (mecanum-only OTOS `vy` fusion).
- `PhysicalStateEstimate.h/.cpp`: `setOtosAlphaVy` forwarder.
- `Robot.cpp`: `bindRearMotors`, `setOtosAlphaVy` init, OTOS 3-DOF `vy` read path.
- `RobotTelemetry.cpp`: mecanum telemetry field emissions.
- `Config.h`: update comments that reference the macro; leave all mecanum config
  fields (`halfTrackMm`, `halfWheelbaseMm`, `vyBodyMax`, etc.) in place.

**Sequencing rationale:** Depends on tickets 001 and 002. The Odometry/Robot
changes depend on MotorController (ticket 002) having no `bindRearMotors` to call.
Config.h is independent but grouped here for completeness.

## Acceptance Criteria

- [x] `source/control/Odometry.h` (lines 222–228, 277–284):
  - [x] No `setOtosAlphaVy()` declaration.
  - [x] No `fusedVy()` accessor.
  - [x] No `_fusedVy` or `_otosAlphaVy` private members.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
- [x] `source/control/Odometry.cpp`:
  - [x] No `_fusedVy` complementary filter code path.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
  - [x] `actual.fused.twist.vy` (Sprint 047 `BodyTwist3` field) is written as
    `0.0f` unconditionally or left at its zero-initialized value — NOT deleted.
- [x] `source/state/PhysicalStateEstimate.h`:
  - [x] No `setOtosAlphaVy` declaration.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
- [x] `source/state/PhysicalStateEstimate.cpp`:
  - [x] No `setOtosAlphaVy` implementation.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
- [x] `source/robot/Robot.cpp` (lines 98–103, 124–127, ~284–296):
  - [x] No `bindRearMotors()` call.
  - [x] No `setOtosAlphaVy()` init call.
  - [x] OTOS read uses 2-DOF path (vx + omega) only; no mecanum `vy` read.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
- [x] `source/robot/RobotTelemetry.cpp` (lines 53, 85, 105):
  - [x] Rear-motor PWM/velocity telemetry fields removed.
  - [x] `vy` fusion telemetry field removed.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
- [x] `source/types/Config.h` (~line 48):
  - [x] Comments updated: no reference to "set by CMake from `drivetrain_type`"
    or "if ROBOT_DRIVETRAIN_MECANUM".
  - [x] All mecanum config fields (`halfTrackMm`, `halfWheelbaseMm`, `vyBodyMax`,
    `aMaxY`, `jMaxY`, `fwdSignFR`, `fwdSignFL`, `fwdSignBR`, `fwdSignBL`,
    `otosAlphaVy`) remain in the struct, unchanged.
- [x] `uv run pytest` passes.
- [x] `grep -rn ROBOT_DRIVETRAIN_MECANUM source/control source/state source/robot source/types`
  returns zero matches.

## Implementation Plan

### Approach

File-by-file. Start with `Odometry.h` (defines what `PhysicalStateEstimate` and
`Robot.cpp` can call), then `PhysicalStateEstimate`, then `Robot.cpp`, then
`RobotTelemetry.cpp`, then `Config.h`.

Key invariant: the `vy` field in `actual.fused.twist` (a `BodyTwist3` from Sprint
047) must NOT be removed — it is an unconditional field that is zero on differential
builds. Only the `_fusedVy` complementary filter that was writing to it under the
mecanum `#ifdef` is removed.

### Files to Modify

**`source/control/Odometry.h`**
- Remove the `setOtosAlphaVy(float alpha)` declaration (lines ~222–228).
- Remove the `float fusedVy() const` accessor.
- Remove `float _fusedVy` and `float _otosAlphaVy` private member declarations
  (lines ~277–284).
- Remove any `#ifdef ROBOT_DRIVETRAIN_MECANUM` guards around these.

**`source/control/Odometry.cpp`**
- Remove the `_fusedVy` complementary filter update in `correctEKF()` (or wherever
  the `vy` fusion was computed: `_fusedVy = alpha * otos_vy + (1-alpha) * _fusedVy`).
- Remove the write of `_fusedVy` to `actual.fused.twist.vy` under `#ifdef`. If the
  Sprint 047 architecture established that `actual.fused.twist.vy = 0.0f` on
  differential, ensure that invariant is preserved (it will be via zero-init).
- Remove `setOtosAlphaVy()` implementation.
- Remove all `#ifdef ROBOT_DRIVETRAIN_MECANUM` blocks.

**`source/state/PhysicalStateEstimate.h`**
- Remove `setOtosAlphaVy(float alpha)` forward declaration.
- Remove any `#ifdef ROBOT_DRIVETRAIN_MECANUM`.

**`source/state/PhysicalStateEstimate.cpp`**
- Remove `setOtosAlphaVy(float alpha)` implementation (it forwarded to `Odometry`).
- Remove any `#ifdef ROBOT_DRIVETRAIN_MECANUM`.

**`source/robot/Robot.cpp`**
- Lines 98–103: Remove `bindRearMotors(...)` call block.
- Lines 124–127: Remove `setOtosAlphaVy(...)` call (was reading `cfg.otosAlphaVy`
  and forwarding to `PhysicalStateEstimate`).
- Lines ~284–296: In the OTOS read loop, remove the mecanum 3-DOF `vy` read path.
  Keep only the 2-DOF `vx` + `omega` read (what the differential robot uses).
- Remove all `#ifdef ROBOT_DRIVETRAIN_MECANUM` blocks.

**`source/robot/RobotTelemetry.cpp`**
- Line 53: Remove rear-motor (BR/BL) PWM or velocity telemetry emission.
- Line 85: Remove mecanum-related telemetry field (likely `vy` fusion or rear
  encoder telemetry).
- Line 105: Remove corresponding mecanum field.
- Remove all `#ifdef ROBOT_DRIVETRAIN_MECANUM` blocks.

**`source/types/Config.h`**
- Update the comment near line 48 that references `ROBOT_DRIVETRAIN_MECANUM`
  (e.g. "populated from JSON only when ROBOT_DRIVETRAIN_MECANUM is defined" → 
  "retained for future mecanum use; not wired into firmware in differential build").
- Update the `drivetrain` field comment similarly.
- Do NOT remove any struct fields.

### Testing Plan

- `uv run pytest` after completing all files.
- `grep -rn ROBOT_DRIVETRAIN_MECANUM source/control source/state source/robot source/types`
  returns zero.
- Verify `actual.fused.twist.vy` is still accessible (the field exists; it just
  always reads 0 on differential — confirmed by a quick inspection of `BodyTwist3`).

### Documentation Updates

No new documentation files. In-code comment updates are part of the acceptance
criteria (Config.h). Existing architecture docs (`architecture-update-047.md`)
reference the mecanum vy path; those will be updated in the consolidate-architecture
sprint after 048 closes.
