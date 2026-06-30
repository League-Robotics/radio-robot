---
sprint: '057'
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 057 Use Cases

Sprint 057 (Phase 2) realizes the subsystem contract in firmware code. The use
cases below are scoped to the deliverables of this sprint only. Planner-as-subsystem
and full loop integration are Phase 3 (sprint 058).

---

## SUC-001: Namespace-clean message types usable from any TU

- **Actor**: Firmware developer / compiler
- **Preconditions**: Phase 1 generated headers exist in `source/messages/`. HAL
  types `::Pose2D` and `::BodyTwist3` exist at global scope.
- **Main Flow**:
  1. Developer writes a firmware TU that includes both `messages/drivetrain.h`
     and `hal/capability/Pose2D.h`.
  2. Compiler resolves `msg::DrivetrainCommand` and `::Pose2D` as distinct names.
  3. Developer uses both in the same file with no collision.
  4. `static_assert` layout bridges confirm byte-compatibility.
- **Postconditions**: Zero compiler errors. `test_messages.py` round-trips
  `msg::DrivetrainCommand` fluent builder.
- **Acceptance Criteria**:
  - [ ] All `source/messages/*.h` compile with both `msg::` types and HAL `::Pose2D` included.
  - [ ] `test_messages.py` passes with `msg::` prefix on all generated types.
  - [ ] `python build.py --clean` zero errors.

---

## SUC-002: Subsystem contract callable pattern available to implementers

- **Actor**: Firmware developer
- **Preconditions**: `SubsystemContract.h` documents the convention. At least
  one subsystem implements it.
- **Main Flow**:
  1. Developer reads `SubsystemContract.h` for the `apply` / `tick` / `state` /
     `configure` / `capabilities` pattern and fluent-builder form.
  2. Developer calls `drive2.newCommand().setTwist(vx, vy, omega).apply()`.
  3. Developer calls `drive2.tickUpdate(now)` then `drive2.tickAction(now)`.
  4. Developer reads `drive2.state().get_fused()` for the updated pose.
  5. Developer calls `drive2.configure(cfg)` to change PID gains.
- **Postconditions**: Pattern compiles under `-std=c++11 -fno-rtti -fno-exceptions`.
  No heap, no virtual dispatch in control path.
- **Acceptance Criteria**:
  - [ ] `SubsystemContract.h` documents the 3-message / 4-verb convention.
  - [ ] `newCommand()` returns a `&` to reused internal instance (no allocation).
  - [ ] Fluent setters chain correctly; terminal `.apply()` commits to internal state.
  - [ ] `python build.py --clean` zero errors.

---

## SUC-003: Sensors subsystem read via message state in simulation

- **Actor**: Test suite / firmware developer
- **Preconditions**: `SimHardware` provides `ILineSensor&` and `IColorSensor&`.
  `subsystems::Sensors` wraps `LineSensor` and `ColorSensor` wrappers.
- **Main Flow**:
  1. Test constructs `SimHardware` and `Sensors` from sim device references.
  2. Test calls `sensors.configure(toSensorsConfig(cfg))`.
  3. Test calls `sensors.tick(now)` in a loop.
  4. Test reads `sensors.state().line.get_connected()` and
     `sensors.state().color.get_connected()`.
  5. Test asserts both sensors appear connected and values in expected range.
- **Postconditions**: `test_sensors_subsystem.py` passes. `state()` returns
  const ref — no copy, no heap.
- **Acceptance Criteria**:
  - [ ] `Sensors::tick(now)` drives both `LineSensor` and `ColorSensor` periodic.
  - [ ] `state()` returns a const ref to `SensorsState` holding both sensor states.
  - [ ] `toSensorsConfig()` projection maps `RobotConfig` fields to typed configs.
  - [ ] `test_sensors_subsystem.py` passes in CI.

---

## SUC-004: Drive subsystem apply/tick/state contract in simulation

- **Actor**: Test suite / firmware developer
- **Preconditions**: `SimHardware` provides motors and odometer. Existing
  `MotorController`, `BodyVelocityController`, `PhysicalStateEstimate`, `Odometry`
  are reused by reference inside `Drive2`.
- **Main Flow**:
  1. Test constructs `SimHardware` and `Drive2`.
  2. Test applies `drive2.newCommand().setTwist({200, 0, 0}).apply()`.
  3. Test calls `tickUpdate(now)` + `tickAction(now)` 50 times.
  4. Test reads `state().get_fused().get_pose()` — x has advanced.
  5. Test applies `SetPose{50,50,0.5}`, ticks once, reads pose == (50,50,0.5).
  6. Test applies twist with `vy=50` on differential; asserts capability false /
     command rejected.
  7. Test applies `Neutral{BRAKE}`, ticks once, asserts wheel outputs ~0.
- **Postconditions**: `test_drive2_subsystem.py` passes. No virtual dispatch.
  `tickAction()` returns a `msg::CommandBatch` (stack-allocated).
- **Acceptance Criteria**:
  - [ ] `Drive2::apply()` stages; `tickUpdate`/`tickAction` execute.
  - [ ] `state().get_fused()` reflects updated pose after tick.
  - [ ] `vy`-reject on differential build: capability check false, command rejected.
  - [ ] `SetPose` re-anchor: after apply + tick, fused pose == requested pose.
  - [ ] Neutral/brake: wheel outputs ~0 after tick.
  - [ ] `toDriveConfig()` projection maps `RobotConfig` to `msg::DrivetrainConfig`.
  - [ ] `python build.py --clean` zero errors.

---

## SUC-005: EKF fusion beats raw sensor noise (simulation end-to-end)

- **Actor**: Test suite
- **Preconditions**: `SimOdometer` supports error knobs. `PhysicsWorld` exposes
  `groundTruthX/Y/H()`. `Drive2::tickUpdate()` runs EKF predict/correct.
- **Main Flow**:
  1. Test enables `SimOdometer` sim model with `linearNoiseSigma=5.0` mm and
     `yawNoiseSigma=0.02` rad and drift knobs.
  2. Test applies forward twist; ticks `Drive2` 50 times.
  3. Test reads `groundTruthX/Y/H()` from `PhysicsWorld`.
  4. Test reads `state().get_fused()`, `state().get_encoder()`, `state().get_optical()`.
  5. Test asserts fused position error < 20 mm from ground truth.
  6. Test asserts raw encoder or optical position error > 10 mm (proving fusion matters).
- **Postconditions**: `test_drive2_subsystem.py::test_ekf_fusion_beats_noise` passes.
- **Acceptance Criteria**:
  - [ ] `SimOdometer` extended with `setDriftPerTickMm()` and `setScaleError()` knobs.
  - [ ] `PhysicsWorld` / `SimHardware` exposes `groundTruthX/Y/H()` accessors.
  - [ ] EKF fused error < 20 mm while raw sensor error > 10 mm after 50 ticks.
  - [ ] Test passes deterministically (fixed RNG seed 43u already in `SimOdometer`).

---
