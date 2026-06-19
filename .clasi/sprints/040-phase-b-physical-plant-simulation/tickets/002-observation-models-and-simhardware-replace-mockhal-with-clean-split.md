---
id: "002"
title: "Observation models and SimHardware — replace MockHAL with clean split"
status: open
use-cases: [SUC-002, SUC-004]
depends-on: ["040-001"]
github-issue: ""
issue: "migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md"
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Observation models and SimHardware — replace MockHAL with clean split

## Description

Create the observation model classes (`SimMotor`, `SimOdometer`, `SimLineSensor`,
`SimColorSensor`, `SimPortIO`) and `SimHardware`, then swap `SimHandle::hal` in
`sim_api.cpp` from `MockHAL` to `SimHardware`. All existing `sim_*` ABI entry points
must be re-pointed to the new plant so existing tests pass without modification.

This is the high-risk ticket: it changes `SimHandle::hal` type, which touches every
test that uses the sim. The golden-TLM byte-exact canary is the primary correctness gate.

### Observation models to create

**`SimMotor`** (`source/io/sim/SimMotor.{h,cpp}`)
- Implements `IVelocityMotor`.
- Holds `const PhysicsWorld&` and a `MotorSide` enum (LEFT or RIGHT).
- `setOutput(int8_t pct)` — stores PWM only; plant is driven by `SimHardware::tick(now,cmds)`.
- `tick(uint32_t now_ms)` — promotes `plant.trueEncL/RMm()` into `_lastPositionMm` cache;
  computes velocity from elapsed time.
- `positionMm()` / `velocityMmps()` — return cached (optionally errored) values.
- `asPositionMotor()` returns `nullptr`.
- `begin()`, `resetPosition()`, `setNeutralMode()` — no-ops.
- Error setters (all default no-op): `setFrozen(bool)`, `setNoiseSigma(float)`.

**`SimOdometer`** (`source/io/sim/SimOdometer.{h,cpp}`)
- Implements `IOdometer`.
- Holds `const PhysicsWorld&`.
- `readTransformed(Pose2D& out, float headingRad)` — returns `plant.truePose*()` (plus
  optional noise / drift when configured); returns `false` when `_readFailure` is set.
- `readVelocityTransformed(BodyTwist& out, float headingRad)` — from plant velocity.
- `readStatus(uint8_t& out)` — 0 (OK) unless LIFT is set.
- `lastReadOk()` — false when `_readFailure` is set.
- `setInjectedPose(float x, float y, float h)` — overrides plant read (back-compat
  for `sim_set_otos_pose`).
- `setReadFailure(bool)` — deterministic read failure (back-compat for
  `sim_set_otos_read_failure`).
- `enableSimModel(bool)` / `setLinearNoise` / `setYawNoise` — wired to internal
  noise fields (forward-compat; noise magnitude defaults to 0 so behavior unchanged).
- `odomX/Y/H()` — return `plant.truePoseX/Y/H()` (back-compat for `sim_get_otos_x/y/h`).
- All calibration/raw-register methods — stored fields or no-ops.
- `begin()` — sets `_initialized = true`.

**`SimLineSensor`** / **`SimColorSensor`** / **`SimPortIO`**
- Implement their respective capability interfaces.
- `begin()`, `setFrozen(bool)` — match `Mock*` semantics so ABI re-points work.

**`SimHardware`** (`source/io/sim/SimHardware.{h,cpp}`)
- Inherits from `Hardware`. Value-member ownership (zero heap):
  ```
  PhysicsWorld  _plant
  SimMotor      _motorL, _motorR
  SimOdometer   _odom
  SimLineSensor _line
  SimColorSensor _color
  SimPortIO     _portIO
  MockServo     _servo   // retained as-is
  ```
- Constructor: `SimHardware(const RobotConfig& cfg)` — initializes plant params.
- `tick(uint32_t now_ms)` — sensor tick: `_motorR.tick(now_ms)`, `_motorL.tick(now_ms)`.
- `tick(uint32_t now_ms, const MotorCommands& cmds)` — plant tick:
  `_plant.setActuators(cmds.pwmL, cmds.pwmR)`, `_plant.update(dt_ms)`.
  Uses signed dt with `dt <= 0` guard (same as `MockHAL::advance`).
- `setOtosBench(bool)` / `isBenchMode()` — no-ops / false.
- Test accessors: `plant()`, `simMotorL()`, `simMotorR()`, `simOdometer()`,
  `simLineSensor()`, `simColorSensor()`, `servoMock()`, etc.

### sim_api.cpp changes

In `SimHandle`: replace `MockHAL hal;` with `SimHardware hal;`. Then re-point:
- `sim_set_motor_slip` → `hal.plant().setSlip(straight, turnExtra)`
- `sim_set_motor_offset` → `hal.plant().setOffsetFactor(side, factor)`
- `sim_enable_otos_model` + `sim_set_otos_fusion` → `hal.simOdometer().enableSimModel()` /
  `.begin()` (fusion flag in `_ts.fuseOtos` unchanged)
- `sim_set_otos_pose` → `hal.simOdometer().setInjectedPose(x, y, h)`
- `sim_set_otos_read_failure` → `hal.simOdometer().setReadFailure(fail)`
- `sim_get_otos_x/y/h` → `hal.simOdometer().odomX/Y/H()`
- `sim_init_line_sensor` / `sim_set_line_frozen` → `hal.simLineSensor()`
- `sim_init_color_sensor` / `sim_set_color_frozen` → `hal.simColorSensor()`
- `sim_get_exact_pose_x/y/h` → `hal.plant().truePoseX/Y/H()` (temporary alias; T3 adds formal `sim_get_true_pose_*`)
- `sim_set_enc_l/r` — leave at the `state.inputs` patch for now; T3 fixes properly.
- **Do NOT touch** `sim_bench_otos_*` — they use `SimHandle::benchOtos` (standalone member).

### CRITICAL: Slip model numerical validation (OQ-1)

After wiring `SimHardware`, run the field-profile fence tests before declaring done:
```
uv run --with pytest python -m pytest \
    tests/simulation/unit/test_rt_slip.py \
    tests/simulation/system/test_incident_scenarios.py \
    tests/simulation/system/test_goto_bounds.py -v
```

If these fail due to slip-model behavior changes, apply **Option A**: add
`_reportedEncLMm` / `_reportedEncRMm` fields to `PhysicsWorld` that apply the
encoder-step slip (old `MockMotor` model). `SimMotor::positionMm()` returns the
reported value. `setSlip(straight, turnExtra)` configures the reported-encoder path.
Escalate to stakeholder only if Option A also fails.

## Acceptance Criteria

- [ ] `SimMotor`, `SimOdometer`, `SimLineSensor`, `SimColorSensor`, `SimPortIO`,
      `SimHardware` exist in `source/io/sim/` and compile cleanly (HOST_BUILD).
- [ ] `SimHandle::hal` is `SimHardware` (not `MockHAL`).
- [ ] All existing `sim_*` ABI back-compat entry points re-pointed to plant/observation.
- [ ] **`test_golden_tlm.py` passes byte-exactly.** This is the primary correctness gate.
- [ ] Behavior-fence tests pass: `test_rt_slip.py`, `test_incident_scenarios.py`,
      `test_goto_bounds.py`, `test_033_005_wedge_hardening.py`,
      `test_watchdog_exemption.py`, `test_ekf.py`, `test_otos_fusion.py`.
- [ ] `test_bench_otos.py` passes unchanged.
- [ ] `uv run --with pytest python -m pytest -q` ≥ 1957 passed, 0 errors.
- [ ] `defaultRobotConfig()` field-pin diff empty.
- [ ] Vendor-confinement grep gate passes.
- [ ] No heap allocation introduced.

## Implementation Plan

### Approach

Create the six new sim source files, then atomically update `sim_api.cpp` to swap
`SimHandle::hal`. Build and run the full suite immediately; apply OQ-1 Option A if
needed.

### Files to Create

- `source/io/sim/SimMotor.h` / `SimMotor.cpp`
- `source/io/sim/SimOdometer.h` / `SimOdometer.cpp`
- `source/io/sim/SimLineSensor.h` / `SimLineSensor.cpp`
- `source/io/sim/SimColorSensor.h` / `SimColorSensor.cpp`
- `source/io/sim/SimPortIO.h` / `SimPortIO.cpp`
- `source/io/sim/SimHardware.h` / `SimHardware.cpp`

### Files to Modify

- `tests/_infra/sim/sim_api.cpp` — swap `SimHandle::hal`; re-point all ABI entry points.

### Testing Plan

After the swap, run the full fence suite:
```
uv run --with pytest python -m pytest \
    tests/simulation/unit/test_golden_tlm.py \
    tests/simulation/unit/test_rt_slip.py \
    tests/simulation/unit/test_ekf.py \
    tests/simulation/unit/test_otos_fusion.py \
    tests/simulation/unit/test_watchdog_exemption.py \
    tests/simulation/unit/test_bench_otos.py \
    tests/simulation/system/test_incident_scenarios.py \
    tests/simulation/system/test_goto_bounds.py \
    tests/simulation/system/test_033_005_wedge_hardening.py \
    -v
uv run --with pytest python -m pytest -q
```

### Documentation Updates

None required — internal sim component.
