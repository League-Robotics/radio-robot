---
id: '057'
title: Phase 2 - Subsystem contract, Drive and Sensors with sim tests
status: done
branch: sprint/057-phase-2-subsystem-contract-drive-and-sensors-with-sim-tests
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- message-based-subsystem-architecture.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 057: Phase 2 - Subsystem contract, Drive and Sensors with sim tests

## Goals

Realize the message-based subsystem contract in C++11 firmware code on top of the
Phase 1 (sprint 056) generated message headers. Deliverables:

1. Namespace generated `Pose2D`/`BodyTwist3` types to `msg::` — resolve Phase 1's
   known name collision with HAL types so generated messages and HAL types coexist
   in any TU without redefinition errors.
2. Subsystem contract documented and realized as a callable pattern: 3-message /
   4-verb convention, fluent builders, `newCommand()` / `newConfig()`.
3. `subsystems::Sensors` facade aggregating existing `LineSensor` and `ColorSensor`
   wrappers behind `tick(now)` / `state()` / `configure()` using generated message types.
4. New `subsystems::Drive2` class composing `MotorController`, `BodyVelocityController`,
   `PhysicalStateEstimate` / `Odometry`, and OTOS into the full 4-verb contract with
   two-phase `tickUpdate`/`tickAction`. Does NOT delete `Drive::periodic()` — the live
   loop swap is Phase 3.
5. Projection functions `toDriveConfig()` / `toSensorsConfig()` from `RobotConfig`.
6. Sim ground-truth / sensor-error model extension in `SimOdometer` and `PhysicsWorld`.
7. Subsystem-isolation tests under `tests/simulation/unit/` exercising both subsystems
   through the sim seam, including EKF fusion beating raw sensor noise.

## Problem

Phase 1 generated C++11 POD message types. Phase 2 wraps the existing control
components behind the new subsystem API so they are testable in isolation (via the
sim seam) without first rewiring the live loop (Phase 3). The existing `periodic()`
calls in `Drive`, `LineSensor`, `ColorSensor` remain; new subsystem classes compose
on top.

## Solution

Additive strategy: new `subsystems::Drive2` and `subsystems::Sensors` compose the
existing control components. Generated messages gain a `msg::` namespace prefix to
resolve the `Pose2D`/`BodyTwist3` collision flagged in `bridges.h`. The sim seam
gains an explicit ground-truth / error-injected dual-track model so subsystem-level
EKF tests can assert fusion beats raw sensors. All work is in `source/subsystems/`,
`source/messages/`, `source/hal/sim/`, and `tests/simulation/unit/`.

## Success Criteria

- `uv run python -m pytest` green: baseline 2367 passed / 2 pre-existing failures
  PLUS all new subsystem-isolation tests pass.
- `python build.py --clean` zero errors — proves new C++11 code compiles under
  `-std=c++11 -fno-rtti -fno-exceptions`.
- `Drive2::apply(DrivetrainCommand{twist})` followed by `tickUpdate(now)` +
  `tickAction(now)` produces byte-plausible parity walk vs today's path.
- `Sensors::tick(now)` populates `state().line` and `state().color` from sim devices.
- `Drive2` rejects `vy != 0` on differential builds (capability check).
- `Drive2::apply(DrivetrainCommand{SetPose})` re-anchors the fused estimate.
- Fused pose from `Drive2` tracks sim ground truth within 20 mm / 0.05 rad after
  50 ticks with OTOS + encoder noise injected.

## Scope

### In Scope

- Namespace migration of generated messages to `msg::` (update `gen_messages.py`,
  all `source/messages/*.h`, `bridges.h`, `tests/_infra/sim/message_test_api.cpp`,
  and `test_messages.py`).
- Subsystem contract documentation as a header comment block in
  `source/subsystems/SubsystemContract.h`.
- Fluent-builder pattern via `newCommand()` / `newConfig()` on `Drive2` and `Sensors`.
- `subsystems::Drive2` — full 4-verb + capabilities contract composing `MotorController`,
  `BodyVelocityController`, `PhysicalStateEstimate`, `Odometry`, `IVelocityMotor`,
  `IOdometer`.
- `subsystems::Sensors` — facade aggregating `subsystems::LineSensor` and
  `subsystems::ColorSensor` behind `tick(now)` / `state()` / `configure()`.
- `toDriveConfig(const RobotConfig&) -> msg::DrivetrainConfig` and
  `toSensorsConfig(const RobotConfig&) -> pair<msg::LineSensorConfig, msg::ColorSensorConfig>`.
- `SimOdometer` extended with drift-per-tick and scale-error knobs.
- `PhysicsWorld` / `SimHardware`: expose `groundTruthX/Y/H()` and `idealX/Y/H()`
  accessors for test assertions.
- New test files: `tests/simulation/unit/test_drive2_subsystem.py` and
  `tests/simulation/unit/test_sensors_subsystem.py`.
- C-ABI shim files `drive2_api.cpp` / `sensors_api.cpp` in `tests/_infra/sim/`
  plus CMakeLists additions.

### Out of Scope

- Phase 3: `loopTickOnce` rewire, Planner as subsystem, command-bus integration,
  live SET routing (sprint 058).
- Deleting or modifying `Drive::periodic()`, `LineSensor::periodic()`, or
  `ColorSensor::periodic()`.
- `superstructure/` Planner implementation.
- Binary wire / serialize-deserialize.
- Bench tests for sensors (explicitly out per issue).
- Swerve / mecanum `vy` axis.

## Test Strategy

All tests: `uv run python -m pytest` (NOT `uv run pytest`).

- Regression: existing `tests/simulation/` stays green (2367 + 2 pre-existing).
- Namespace test: `test_messages.py` extended with `msg::` namespace round-trip.
- Sensors isolation: `test_sensors_subsystem.py` — construct `Sensors` on `SimHardware`
  sim devices; `tick(now)` N times; assert `state().line.connected` and
  `state().color.connected`; assert values in range.
- Drive2 isolation: `test_drive2_subsystem.py` — twist / `vy`-reject / SetPose /
  neutral / EKF-fusion-beats-noise.
- Device compile: `python build.py --clean` zero errors on every C++-touching ticket.

## Architecture Notes

- Generated types go in `msg::` namespace. HAL types stay at global scope unchanged.
- `Drive2` is a new class; `Drive` (with `periodic()`) is untouched this sprint.
- No virtual dispatch in `Drive2` control path.
- No C++20 `concept` — structural convention only.
- `Opt<T>` from `msg::` (formerly `source/messages/common.h`), not `std::optional`.
- Fixed-capacity arrays only; no heap allocation.
- `CommandBatch` returned by `tickAction()` is stack-allocated (K=8).

## GitHub Issues

None.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Namespace generated messages to `msg::` | — |
| 002 | Subsystem contract scaffold and fluent-builder pattern | 001 |
| 003 | Sensors subsystem facade and sim isolation tests | 001, 002 |
| 004 | Drive2 subsystem: compose and contract | 001, 002 |
| 005 | Sim ground-truth/error model extension and Drive2 EKF isolation tests | 001, 004 |

Tickets execute serially in the order listed.
