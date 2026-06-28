---
id: 048
title: Eliminate ROBOT_DRIVETRAIN_MECANUM ifdef (differential-only)
status: done
branch: sprint/048-kinematics-namespace-alias-to-concrete-model-classes
issues:
- eliminate-ifdef-robot-drivetrain-mecanum-everywhere.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->
<!-- NOTE: directory name is legacy. This sprint was re-scoped from the old
     "kinematics namespace alias to concrete model classes" plan, which is now
     SUPERSEDED by the eliminate-ifdef issue (see issues/superseded/). -->

# Sprint 048: Eliminate `#ifdef ROBOT_DRIVETRAIN_MECANUM` (differential-only)

## Goals

Remove the `ROBOT_DRIVETRAIN_MECANUM` compile-time switch from the codebase
**entirely** (81+ sites across ~15 files). Compile a single, unconditional
**differential** code path; keep the standalone mecanum math classes
(`MecanumKinematics`, `MecanumHAL`) in-tree but unwired; remove the build-flag
plumbing. Robot-model selection collapses to one documented top-level point
(`main.cpp` HAL line + `IKinematics.h`). Producing a mecanum robot becomes a
deliberate future edit — git history preserves the deleted integration.

This supersedes the original 048 ("namespace alias → concrete classes"), which only
partially removed the macro.

## Issues addressed

- `eliminate-ifdef-robot-drivetrain-mecanum-everywhere.md`

## Rationale / grouping

Single-issue sprint. Sequenced **FIRST** in the roadmap because it is a
deletion/simplification (lowest risk) and it shrinks the conflict surface for every
later sprint: it removes mecanum `vy` from `Odometry`/`PhysicalStateEstimate` (which
sprint 050's EKF work touches) and strips mecanum branches from `MotionCommands` /
`Superstructure` (which sprints 051–053 touch).

## Scope sketch (detail-planning will produce tickets)

- Build plumbing: drop the flag in `CMakeLists.txt`, `tests/_infra/sim/CMakeLists.txt`,
  `build.py`.
- Model-selection consolidation: `main.cpp` (NezhaHAL only), `IKinematics.h`
  (differential only, no `#ifdef`).
- Strip mecanum branches keeping differential: `BodyVelocityController`,
  `MotorController`, `Superstructure`/`MotionController`, `MotionCommands`,
  `Odometry`, `PhysicalStateEstimate`, `Robot.cpp`, `RobotTelemetry`, `Config.h`
  comments.
- Retain (unwired): `MecanumKinematics.{h,cpp}`, `MecanumHAL.cpp`.
- Tests: keep `test_mecanum_kinematics.py`; remove integrated-path mecanum tests
  and the dual-config `build_mecanum` sim build.

## Dependencies

None upstream. Should land before sprints 050, 051, 052, 053 (shared files).

## Success gate

`grep -rn ROBOT_DRIVETRAIN_MECANUM source tests CMakeLists.txt build.py` → zero;
clean firmware build; `uv run pytest` green.

## Tickets

Tickets execute serially in dependency order. Build safety: source `#ifdef` branches
are stripped (001–003) BEFORE the CMake macro definition is removed (004), so each
ticket boundary leaves the codebase in a compilable state.

| # | Title | Depends On |
|---|-------|-----------|
| 001 | Model-selection consolidation: IKinematics.h and main.cpp unconditional differential | — |
| 002 | Control/superstructure strip: BVC, MotorController, Superstructure, MotionCommands | 001 |
| 003 | State/odometry/telemetry/robot wiring strip: Odometry, PhysicalStateEstimate, Robot, RobotTelemetry, Config | 001, 002 |
| 004 | Build-plumbing removal: CMakeLists.txt, tests sim CMakeLists, build.py | 001, 002, 003 |
| 005 | Tests cleanup and final verification: delete mecanum integration tests, verify green | 001, 002, 003, 004 |
