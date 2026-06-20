---
id: '046'
title: 'Mecanum drivetrain: swappable differential/mecanum kinematics (omnidirectional)'
status: planning-docs
branch: mecanum
use-cases:
  - SUC-001
  - SUC-002
  - SUC-003
  - SUC-004
  - SUC-005
issues:
  - mecanum-drivetrain-swappable-differential-mecanum-kinematics-full-omnidirectional.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 046: Mecanum drivetrain: swappable differential/mecanum kinematics (omnidirectional)

## Goals

Deliver compile-time swappable drivetrain support so the same source tree
builds either the existing 2-wheel differential robot (tovez, byte-identical)
or a new 4-wheel mecanum robot with full omnidirectional motion (forward +
strafe + turn). Work stays on the `mecanum` branch throughout.

## Problem

The firmware hard-assumes a 2-wheel differential drivetrain throughout the
kinematics, control, HAL, and command layers. A second robot (4-wheel mecanum,
Nezha motors, OTOS) is now available and requires omnidirectional control
(strafing). The two drivetrains must coexist in the same repo — cloning and
pointing at a robot config selects the variant at build time.

## Solution

A compile-time macro (`ROBOT_DRIVETRAIN_MECANUM`, set from `drivetrain_type`
in the robot JSON via build.py/CMake) gates every mecanum-specific addition.
The differential path is completely untouched in the non-mecanum build. The
mecanum path adds: `MecanumKinematics` (4-wheel inverse/forward/saturate),
`MecanumHAL` (4-motor sibling of NezhaHAL), a 3-channel BodyVelocityController
(`vy` added), N-wheel MotorController, `OMNI`/`STRAFE` command verbs,
OTOS-led lateral odometry, and telemetry extensions. Robot config schema grows
optional mecanum-only fields (schema-additive; differential JSON unchanged).

## Success Criteria

- `tovez` differential build: regenerated `DefaultConfig.cpp` diff is
  additive-constant lines only; sim suite stays 2093-passed; golden-TLM oracle
  unchanged.
- Mecanum build: robot moves forward, turns, and strafes on the bench;
  `SNAP` reports non-zero `vx/vy/omega`; playfield camera verifies commanded
  direction matches observed motion.

## Scope

### In Scope

- Compile-time drivetrain select: schema, RobotConfig enum, gen_default_config,
  CMakeLists.txt (firmware + sim), build.py.
- `BodyTwist3` / `RobotGeometry` types; `IKinematics.h` namespace alias;
  `MecanumKinematics.{h,cpp}`; array-form BodyKinematics overloads.
- `MecanumHAL.{h,cpp}` (4-motor); `NoopDevices.h` refactor; Hardware.h
  additive Noop accessors; `main.cpp` `#ifdef` select.
- Mecanum robot JSON (scaffold with known values, MEASURE/CALIBRATE
  placeholders); first-flash bring-up to read the 5-char name.
- N-wheel MotorController/MotorCommands arrays; 3-channel
  BodyVelocityController; `VW vy=` extension; `OMNI`/`STRAFE` verbs.
- OTOS-led odometry: surface `vy` from OTOS velocity read; carry lateral
  velocity in fused twist; simple OTOS-trusting lateral channel (not a new EKF
  state).
- Telemetry: `vy` in `twist=`; per-wheel `vel=` for mecanum build.
- Bench + playfield bring-up and calibration; strafe leg in
  `tests/bench/playfield_camera_run.py`.
- Host `robot_config.py`: add `drivetrain_type` field.

### Out of Scope

- Runtime drivetrain switching (no live motor-port reconfiguration).
- Differential EKF changes (5-state EKF stays untouched).
- Go-to / DISTANCE / G-command lateral extensions (deferred to a future sprint).
- Line-sensor support on the mecanum robot (no line sensor; graceful absence
  already handled).

## Test Strategy

- **Differential regression gate (every firmware ticket):** regenerate
  `DefaultConfig.cpp`, `git diff` must be additive-constant lines only; run
  `uv run --with pytest python -m pytest tests/simulation -q` (2093 passed);
  golden-TLM oracle unchanged.
- **Kinematics unit tests (T2):** host pytest for MecanumKinematics
  inverse/forward round-trip, known-vector strafe/rotate/forward, saturation.
- **Sim build (T1, T3, T5, T6):** mecanum sim CMake must compile cleanly;
  new verbs must parse in sim.
- **Hardware-in-the-loop (T4, T8):** team-lead runs over radio relay + overhead
  camera; programmer delivers code, team-lead executes.

## Architecture Notes

- Macro polarity: `#ifdef ROBOT_DRIVETRAIN_MECANUM` (not `#ifndef`), per
  the in-repo convention at CMakeLists.txt:293–298.
- Kinematics: compile-time namespace alias (no vtable); Cortex-M4 safe.
- Wheel order canonical: [0]=FR, [1]=FL, [2]=BR, [3]=BL (ports 1,2,3,4).
- Sync-coupling disabled in mecanum build (no clean 4-wheel analog).
- OTOS-led lateral: OTOS `vy` is a direct observation; no new EKF state.
- Differential EKF (5-state, non-holonomic) is completely untouched.
- `BodyTwist3` (3-DOF: vx, vy, omega) is mecanum-only; `BodyTwist` (2-DOF)
  remains the interface type for differential path and IOdometer.

## GitHub Issues

None linked yet.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Config/build compile-time drivetrain select | — |
| 002 | Kinematics math: BodyTwist3, RobotGeometry, MecanumKinematics, IKinematics | 001 |
| 003 | HAL: MecanumHAL, NoopDevices refactor, Hardware Noop additions, main.cpp select | 002 |
| 004 | Mecanum robot JSON scaffold + first-flash bring-up (HITL) | 003 |
| 005 | N-wheel control: MotorController arrays, BVC vy channel, VW/OMNI/STRAFE verbs | 004 |
| 006 | OTOS-led odometry + lateral velocity (vy) | 005 |
| 007 | Telemetry: vy in twist=, per-wheel vel= for mecanum build | 006 |
| 008 | Bench + playfield bring-up and calibration (HITL) | 007 |

Tickets execute serially in the order listed.
