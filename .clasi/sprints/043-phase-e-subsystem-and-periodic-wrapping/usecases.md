---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 043 Use Cases

## SUC-001: Sensor subsystem wraps per-sensor timed read cycle

- **Actor**: Firmware cooperative loop (`loopTickOnce`)
- **Preconditions**: `LineSensor`, `ColorSensor`, and `Ports` subsystem objects exist
  under `source/subsystems/sensors/`; each holds a reference to its HAL device interface
  and to the relevant `HardwareState` inputs slice.
- **Main Flow**:
  1. `loopTickOnce` reaches the LINE timed block; instead of calling `robot.lineRead()`
     directly, it calls `robot.lineSensor.periodic()`.
  2. `LineSensor::periodic()` checks `lagLineMs` and the timer, then calls
     `updateInputs()` which executes the former `robot.lineRead()` body verbatim.
  3. Same for `ColorSensor::periodic()` and `Ports::periodic()`.
  4. Each subsystem writes its data into `HardwareState` (same fields, same conditions).
- **Postconditions**: `HardwareState.line*`, `colorR/G/B/C`, and `digital/analogIn[]`
  are populated identically to the pre-Phase-E behavior. Golden-TLM frame is byte-exact.
- **Acceptance Criteria**:
  - [ ] `LineSensor`, `ColorSensor`, `Ports` classes compile under `source/subsystems/sensors/`.
  - [ ] Each has `updateInputs()` and `periodic()` methods.
  - [ ] Golden-TLM canary passes byte-exact.
  - [ ] Behavior-preservation fences (`test_incident_scenarios.py`, `test_goto_bounds.py`) green.

---

## SUC-002: Drive subsystem wraps per-wheel velocity control

- **Actor**: Firmware cooperative loop (`loopTickOnce`)
- **Preconditions**: `Drive` subsystem object exists under `source/subsystems/drive/`;
  it holds references to `MotorController`, the encoder-filter streak state members on
  `Robot`, and `HardwareState`.
- **Main Flow**:
  1. `loopTickOnce` reaches the CONTROL COLLECT block; instead of an inline block, it
     calls `robot.drive.periodic()`.
  2. `Drive::periodic()` runs the outlier filter → `motorController.controlTick()` →
     wedge push into `Odometry` verbatim (same order, same code).
  3. `Drive::updateInputs()` writes `encLMm`, `encRMm`, and velocity fields into
     `HardwareState.inputs` (called internally at the start of `periodic()` or from
     within the outlier-filter block, in the same position the writes occur today).
- **Postconditions**: Encoder inputs and motor commands in `HardwareState` are
  identical to the pre-Phase-E values. `MotorController::controlTick` output unchanged.
- **Acceptance Criteria**:
  - [ ] `Drive` class compiles under `source/subsystems/drive/`.
  - [ ] `Drive::periodic()` produces byte-identical encoder + velocity updates.
  - [ ] Wedge-hardening fence (`test_033_005_wedge_hardening.py`) green.
  - [ ] Simulation tier ≥ 2001 passed.

---

## SUC-003: Gripper subsystem wraps optional servo actuator

- **Actor**: `ServoController` (command handler) and `loopTickOnce`
- **Preconditions**: `Gripper` subsystem exists under `source/subsystems/gripper/`.
  `GripperIONull` is the null-object used when `has_gripper = false`.
- **Main Flow**:
  1. When `has_gripper = true`, a `Gripper` instance wraps the `IPositionMotor&` and
     calls the underlying servo each `periodic()`.
  2. When `has_gripper = false`, a `GripperIONull` is constructed; its `periodic()` and
     `updateInputs()` are no-ops.
  3. `loopTickOnce` does not change — gripper's `periodic()` is not called from the
     loop (the gripper is command-driven, not polled). `Gripper` is a structural seam
     for Phase F, not yet wired into `loopTickOnce`'s poll sequence.
- **Postconditions**: `Gripper` compiles. `GripperIONull` satisfies the gripper
  interface. No behavior change to servo actuation.
- **Acceptance Criteria**:
  - [ ] `Gripper.{h,cpp}` and `GripperIONull` compile under `source/subsystems/gripper/`.
  - [ ] `has_gripper = false` path uses `GripperIONull` with no-op `periodic()`.
  - [ ] Existing servo command behavior unchanged; simulation tier green.

---

## SUC-004: loopTickOnce calls subsystem periodic() in original order

- **Actor**: `loopTickOnce` orchestrator
- **Preconditions**: All subsystem objects (`Drive`, `LineSensor`, `ColorSensor`,
  `Ports`) are value members of `Robot`, constructed before `loopTickOnce` is called.
- **Main Flow**:
  1. `loopTickOnce` calls `robot.drive.periodic()` at the position the CONTROL COLLECT
     block occupied.
  2. All other existing calls (`cmd.dequeueOne`, `evaluateSafety`, `driveAdvance`,
     `estimate.addOdometryObservation`, `hal.tick`, `otosCorrect`, line/color/ports
     periodic, `telemetryEmit`) run in the SAME ORDER as today.
  3. The line/color/ports periodic calls replace the former `robot.lineRead()` /
     `robot.colorRead()` / `robot.portsRead()` direct calls, preserving timed-block
     logic (lag gates, `ts.lastLine`, etc.) inside each subsystem's `periodic()`.
- **Postconditions**: The cooperative loop execution order is identical to pre-Phase-E.
  Golden-TLM and all behavior-preservation fences pass.
- **Acceptance Criteria**:
  - [ ] `loopTickOnce` calls subsystems in documented order (no reordering).
  - [ ] Golden-TLM canary byte-exact.
  - [ ] All simulation tier fences green.
  - [ ] ARM firmware build gate: `python3 build.py --fw-only` → 0 errors.
