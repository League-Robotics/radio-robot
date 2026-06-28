---
sprint: '047'
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Use Cases — Sprint 047

## SUC-047-001: Observe three pose estimates side by side

- **Actor**: Developer / test operator
- **Preconditions**: Robot is driving; OTOS is available; EST telemetry is enabled.
- **Main Flow**:
  1. Robot drives a trajectory with OTOS enabled.
  2. Firmware emits `EST enc ...`, `EST otos ...`, `EST fuse ...` telemetry lines
     each containing pose (x, y, h), twist (vx, vy, omega), age, and validity flag.
  3. Developer compares the three rows: encoder pose integrates wheel deltas only;
     optical pose reflects raw OTOS before fusion; fused pose is the EKF output.
  4. When OTOS offset is injected, encoder and fused diverge; removing the offset
     causes them to converge again.
- **Postconditions**: All three `EST` rows are emitted each telemetry period.
- **Acceptance Criteria**:
  - [ ] Encoder pose is never overwritten by EKF fusion.
  - [ ] Optical pose reflects raw OTOS position/twist before correction.
  - [ ] All three EST rows appear in telemetry with age and validity columns.

---

## SUC-047-002: Access robot state via clean actual/desired/outputs grouping

- **Actor**: Firmware developer maintaining or extending the robot.
- **Preconditions**: Source tree compiles; developer is reading or writing state.
- **Main Flow**:
  1. Developer opens `RobotStateContainer`.
  2. `actual.*` contains sensor-derived beliefs: encoder, optical, fused pose estimates;
     raw per-wheel odometry (encMm[], velMms[]); line/color/port readings.
  3. `desired.*` contains profiled body twist, target wheel speeds, drive mode, world
     target, port output commands, and async reply plumbing.
  4. `outputs.*` contains PWM values and dirty flags being driven to actuators.
- **Postconditions**: No state field appears in more than one group.
- **Acceptance Criteria**:
  - [ ] `RobotStateContainer` has exactly three top-level fields: `actual`, `desired`, `outputs`.
  - [ ] All fields from old `HardwareState`, `MotorCommands`, and `TargetState` are reachable
        through accessor shims or direct new-path names during the migration window.
  - [ ] PWM and dirty flags are in `outputs`, not `desired`.

---

## SUC-047-003: Both drivetrain variants compile clean with no #ifdef in state structs

- **Actor**: CI / build system.
- **Preconditions**: CMake toolchain available; both `active_robot.json` variants configured.
- **Main Flow**:
  1. `python build.py --clean` builds the differential firmware.
  2. Mecanum build is triggered with `ROBOT_DRIVETRAIN_MECANUM` defined.
  3. Both build without error or warning.
- **Postconditions**: No `#ifdef` inside `ActualState`, `DesiredState`, or `OutputState`
  struct bodies; array sizing is via `Kinematics::kWheelCount`.
- **Acceptance Criteria**:
  - [ ] Zero compile errors in the differential build.
  - [ ] Zero compile errors in the mecanum build.
  - [ ] `BodyTwist3` is used uniformly; `vy` is always present (0 on differential).

---

## SUC-047-004: Simulation unit suite stays green with no Python test edits

- **Actor**: CI / automated test runner.
- **Preconditions**: `tests/_infra/sim/` builds cleanly; Python sim wrapper loads.
- **Main Flow**:
  1. `sim_api.cpp` C-ABI bodies are updated to reference new struct paths
     (e.g. `actual.fused.pose.x`, `actual.encMm[1]`, `outputs.pwm[1]`).
  2. Function signatures remain identical.
  3. `uv run --with pytest python -m pytest tests/simulation/ -q` is run.
  4. All tests pass green.
- **Postconditions**: No Python test file is edited.
- **Acceptance Criteria**:
  - [ ] `tests/simulation/` passes green after `sim_api.cpp` body updates.
  - [ ] No Python test file is modified.
  - [ ] New `sim_get_enc_pose_*` and `sim_get_otos_pose_*` ABI functions are added
        for the fusion-validation test without breaking existing ABI.

---

## SUC-047-005: Fusion-validation test asserts encoder/fused diverge under OTOS offset

- **Actor**: Automated test (new pytest added this sprint).
- **Preconditions**: Sim builds; OTOS injection available; fusion enabled.
- **Main Flow**:
  1. Sim drives straight for N ticks with no OTOS.
  2. OTOS offset (+500 mm lateral or similar) is injected.
  3. Fusion is enabled for K ticks.
  4. `sim_get_enc_pose_x/y/h()` and `sim_get_fused_pose_x/y/h()` are read.
  5. Values differ by measurable amount, confirming encoder is not overwritten by fusion.
  6. `sim_get_otos_pose_x/y/h()` reflects the injected offset.
- **Postconditions**: Test is added to `tests/simulation/unit/`.
- **Acceptance Criteria**:
  - [ ] New test passes green.
  - [ ] `actual.encoder.pose` never equals `actual.fused.pose` after offset injection.
  - [ ] All three `sim_get_*_pose_*` groups return distinct values after the offset.

---

## SUC-047-006: BodyVelocityController publishes desired body twist each advance()

- **Actor**: Telemetry / diagnostic consumer; test that reads `desired.bodyTwist`.
- **Preconditions**: BVC is running; motion command is active.
- **Main Flow**:
  1. Motion command sets a body twist target via `BVC::setTarget()`.
  2. Each `advance()` call ramps toward target and writes `desired.bodyTwist` (profiled
     live values) and `desired.bodyTwistRaw` (commanded pre-clamp values).
  3. Telemetry reads `desired.bodyTwist` without accessing BVC internals.
- **Postconditions**: `desired.bodyTwist` always reflects BVC's current profiled state.
- **Acceptance Criteria**:
  - [ ] `desired.bodyTwist.vx/vy/omega` equals BVC `currentV()/currentVy()/currentOmega()` after each tick.
  - [ ] `desired.bodyTwistRaw.vx/vy/omega` equals BVC `targetV()/targetVy()/targetOmega()`.
  - [ ] BVC internals (`_v`, `_omega`, etc.) remain private.
  - [ ] BVC `currentV()`/`currentOmega()` accessors are not removed (back-compat).
