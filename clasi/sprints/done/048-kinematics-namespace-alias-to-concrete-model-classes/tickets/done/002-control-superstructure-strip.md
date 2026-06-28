---
id: '002'
title: 'Control/superstructure strip: BVC, MotorController, Superstructure, MotionCommands'
status: done
use-cases:
- SUC-048-002
depends-on:
- '001'
issue: eliminate-ifdef-robot-drivetrain-mecanum-everywhere.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

## Description

Strip all `#ifdef ROBOT_DRIVETRAIN_MECANUM` branches from the control and
superstructure layer: `BodyVelocityController`, `MotorController`,
`MotionController`, `Superstructure`, and `MotionCommands`. At each site, keep
the `#else` (differential) body and delete the mecanum body plus the three
preprocessor lines. Delete mecanum-only `#ifdef…#endif` blocks wholesale.

This is the largest single ticket by line count (~17 blocks in MotorController
alone) but all changes are deletions — no new logic is introduced.

**Sequencing rationale:** Depends on ticket 001 (IKinematics.h is clean). The
CMake macro is still defined during this ticket. All `#ifdef` branches in these
files will be stripped, so the macro value no longer matters for compilation.
Ticket 004 (CMake removal) can then safely drop the macro definition.

## Acceptance Criteria

- [x] `source/control/BodyVelocityController.h`:
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
  - [x] No `setTarget(v, omega, vy)` overload — only `setTarget(float v_mms, float omega_rads)`.
  - [x] No `currentVy()` or `targetVy()` accessors.
  - [x] No `_vy`, `_vyTgt`, `_vyALive`, `_geom` private members.
  - [x] No `#include "io/capability/Pose2D.h"` inside the mecanum guard.
- [x] `source/control/BodyVelocityController.cpp`:
  - [x] `advance()` contains no mecanum `vy`-ramp branch.
  - [x] `advance()` publishes `DesiredState` as `{_v, 0.0f, _omega}` unconditionally.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM` anywhere.
- [x] `source/control/MotorController.h`:
  - [x] No `_motorBR`, `_motorBL`, `_vcBR`, `_vcBL` declarations.
  - [x] No `bindRearMotors()` declaration.
  - [x] No 4-wheel `setTarget` overload.
  - [x] No `getEncoderPositions(float[4])` — only `getEncoderPositions(float[2])`.
  - [x] No BR/BL encoder-state fields.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
- [x] `source/control/MotorController.cpp`:
  - [x] All ~17 mecanum blocks deleted.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
- [x] `source/superstructure/MotionController.h`:
  - [x] `GoalRequest` struct has no `vy_mms` field.
  - [x] No 8-argument `beginVelocity` overload.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
- [x] `source/superstructure/Superstructure.h` / `.cpp`:
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
  - [x] No calls to the 8-arg `beginVelocity` overload.
- [x] `source/commands/MotionCommands.cpp`:
  - [x] Mecanum blocks at ~808–873, 885, 1120, 1151 deleted.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM`.
- [x] `source/commands/MotionCommand.h` (residual site, second pass):
  - [x] 3-DOF `setTarget(float v_mms, float omega_rads, float vy_mms)` overload declaration removed.
  - [x] `#ifdef ROBOT_DRIVETRAIN_MECANUM` / `#endif` guards removed.
- [x] `source/commands/MotionCommand.cpp` (residual site, second pass):
  - [x] 3-DOF `setTarget` definition removed.
  - [x] `#ifdef ROBOT_DRIVETRAIN_MECANUM` / `#endif` guards removed.
- [x] `source/control/MotionControllerBegin.cpp` (residual site, second pass):
  - [x] Mecanum `beginVelocity` overload signature guards removed; only 7-arg differential form remains.
  - [x] `#ifdef ROBOT_DRIVETRAIN_MECANUM` post-start block applying 3-DOF `vy` removed.
  - [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM` anywhere.
- [x] `source/robot/DefaultConfig.cpp` (residual site, second pass):
  - [x] Comment reworded to not reference the macro literal.
- [x] `source/io/real/MecanumHAL.cpp` (residual site, second pass):
  - [x] TODO comment reworded to drop `ROBOT_DRIVETRAIN_MECANUM` token.
- [x] `uv run pytest` passes (2230 passed, 2 known pre-existing failures only).
- [x] `grep -rln ROBOT_DRIVETRAIN_MECANUM source/` returns zero matches (full tree clean).
- [x] `grep -rn ROBOT_DRIVETRAIN_MECANUM source/control source/superstructure source/commands`
  returns zero matches.

## Implementation Plan

### Approach

Work file-by-file. For each `#ifdef ROBOT_DRIVETRAIN_MECANUM … #else … #endif`:
keep the `#else` body, delete the mecanum body and the three preprocessor lines.
For `#ifdef … #endif` (no `#else`): delete the entire block.

Suggested order within this ticket: `.h` files first (they define the API surface),
then `.cpp` files. This lets you verify that no declaration uses a removed member
before you try to remove it from the implementation.

### Files to Modify

**`source/control/BodyVelocityController.h`**
- Remove `#ifdef ROBOT_DRIVETRAIN_MECANUM` include guard (line 4–6) for `Pose2D.h`.
- Replace the dual `setTarget` declaration block (lines 60–64) with the single
  differential form: `void setTarget(float v_mms, float omega_rads);`
- Delete the `currentVy()` / `targetVy()` accessor block (lines 105–111).
- Delete the mecanum private member block (lines 159–165): `_vy`, `_vyTgt`,
  `_vyALive`, `_geom`.
- Update the `setTarget` doc comment to remove the `vy_mms` parameter description.

**`source/control/BodyVelocityController.cpp`**
- In `advance()`: remove the mecanum `_vy` ramp code path.
- In the `DesiredState` publish block: replace `{_v, _vy, _omega}` with
  `{_v, 0.0f, _omega}` unconditionally.
- Remove the `setTarget(v, omega, vy)` 3-arg implementation if it exists.
- Remove `currentVy()` / `targetVy()` implementations if they are not inline.

**`source/control/MotorController.h`**
- Remove `bindRearMotors()` declaration.
- Remove 4-wheel `setTarget` overload.
- Remove `getEncoderPositions(float[4])` overload (keep `float[2]` form).
- Remove `_motorBR`, `_motorBL`, `_vcBR`, `_vcBL`, BR/BL encoder fields.
- Remove all `#ifdef ROBOT_DRIVETRAIN_MECANUM` guards.

**`source/control/MotorController.cpp`**
- Remove all ~17 mecanum `#ifdef ROBOT_DRIVETRAIN_MECANUM` blocks. Each follows
  the standard pattern: locate the block, keep differential body, delete mecanum
  body + 3 preprocessor lines, or delete wholesale if mecanum-only.
- Remove `bindRearMotors()` implementation.
- Remove 4-wheel `setTarget` implementation.
- Remove `getEncoderPositions(float[4])` implementation.

**`source/superstructure/MotionController.h`**
- Remove `vy_mms` from `GoalRequest`.
- Remove 8-arg `beginVelocity` overload declaration.
- Remove any `#ifdef ROBOT_DRIVETRAIN_MECANUM` guards.

**`source/superstructure/Superstructure.h`**
- Remove `#ifdef ROBOT_DRIVETRAIN_MECANUM` guards.
- Remove any `vy_mms`-related declarations.

**`source/superstructure/Superstructure.cpp`**
- Remove calls to 8-arg `beginVelocity`; update to 7-arg where needed.
- Remove `#ifdef ROBOT_DRIVETRAIN_MECANUM` blocks.

**`source/commands/MotionCommands.cpp`**
- Delete mecanum blocks at approximately: lines 808–873 (mecanum velocity command
  variant), line 885 (mecanum-only `vy` argument), line 1120, line 1151.
- For each, keep the differential body.

### Testing Plan

- `uv run pytest` after completing all files in this ticket.
- `grep -rn ROBOT_DRIVETRAIN_MECANUM source/control source/superstructure source/commands`
  must return zero.
- Firmware compile (macro still defined in CMake but no `#ifdef` in these files —
  should compile cleanly).

### Documentation Updates

Update doc comments in `BodyVelocityController.h` to remove the `vy_mms`
parameter note. Update `MotorController.h` class-level doc to reflect
2-wheel-only interface.
