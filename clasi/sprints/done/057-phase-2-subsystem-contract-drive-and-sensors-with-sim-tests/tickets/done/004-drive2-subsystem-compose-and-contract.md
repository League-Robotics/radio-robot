---
id: '004'
title: 'Drive2 subsystem: compose and contract'
status: done
use-cases:
- SUC-004
depends-on:
- '001'
- '002'
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Drive2 subsystem: compose and contract

## Description

Implement `subsystems::Drive2` — the Drive subsystem class realizing the full
message-contract API on top of the existing control components. This is the largest
ticket in the sprint: it introduces `apply(msg::DrivetrainCommand)`, two-phase
`tickUpdate(now)` / `tickAction(now)`, `state() -> const msg::DrivetrainState&`,
`configure(msg::DrivetrainConfig)`, and `capabilities() -> msg::DrivetrainCapabilities`.

The existing `subsystems::Drive` class (with `periodic()`) is NOT modified. `Drive2`
is a new class that holds the same component references and drives them via the
message-contract API instead of the old inline block pattern.

Also implements `toDriveConfig(const RobotConfig&) -> msg::DrivetrainConfig` projection.
Also implements C-ABI shims (`drive2_api.cpp`) and isolation tests
(`test_drive2_subsystem.py`) covering the apply/tick/state/configure contract:
twist, `vy`-reject, `SetPose`, and neutral/brake. (EKF fusion test is ticket 005.)

## Approach

### 1. `source/subsystems/drive/Drive2.h`

Class layout (no virtual dispatch, no heap, all refs):

```cpp
#pragma once
#include "messages/drivetrain.h"   // msg::DrivetrainCommand/State/Config/Capabilities
#include "hal/capability/IVelocityMotor.h"
#include "hal/capability/IOdometer.h"
namespace subsystems { class Drive2 {
public:
    Drive2(IMotor& motorL, IMotor& motorR,
           MotorController& mc, BodyVelocityController& bvc,
           PhysicalStateEstimate& est, Odometry& odo,
           IOdometer& otos, const RobotConfig& cfg);

    void                       apply(const msg::DrivetrainCommand& cmd);
    void                       tickUpdate(uint32_t now);   // SENSE phase
    msg::CommandBatch          tickAction(uint32_t now);   // ACT phase; returns batch
    const msg::DrivetrainState& state() const { return _state; }
    void                       configure(const msg::DrivetrainConfig& cfg);
    msg::DrivetrainCapabilities capabilities() const;

    // Fluent-builder helpers (per SubsystemContract.h)
    msg::DrivetrainCommand& newCommand() { _cmd = {}; return _cmd; }
    msg::DrivetrainConfig&  newConfig()  { _cfg2 = {}; return _cfg2; }

private:
    IMotor&                 _motorL;
    IMotor&                 _motorR;
    MotorController&        _mc;
    BodyVelocityController& _bvc;
    PhysicalStateEstimate&  _est;
    Odometry&               _odo;
    IOdometer&              _otos;
    const RobotConfig&      _robCfg;
    msg::DrivetrainConfig   _drvCfg  = {};   // live config slice
    msg::DrivetrainState    _state   = {};   // owned state snapshot
    msg::DrivetrainCommand  _cmd     = {};   // staged command (apply → tick)
    msg::DrivetrainCommand  _pendCmd = {};   // copy taken at tickUpdate start
    msg::DrivetrainConfig   _cfg2    = {};   // fluent builder scratch
    bool _cmdPending = false;
}; }
```

### 2. `source/subsystems/drive/Drive2.cpp`

**`apply(cmd)`**: copies `cmd` into `_cmd`, sets `_cmdPending = true`. Returns
nothing — no hardware, no emission (per contract).

**`tickUpdate(now)`** — SENSE phase (mirrors the existing `Drive::periodic()` and
`PhysicalStateEstimate::driveAdvance()` sense half):
- Capture `_pendCmd = _cmd; _cmdPending = false`.
- Call the existing `_mc.controlTick()` path for encoder reads: collect per-wheel
  position from `_motorL.positionMm()` / `_motorR.positionMm()` (via the existing
  outlier filter — can reuse `Drive::_filterRejectStreakL/R` state if `Drive` stores
  them, or replicate them in `Drive2` since they are 5 `uint8_t`/`uint32_t` members).
- Call `_est.driveAdvance(encLMm, encRMm, now)` (or equivalent) to run the EKF
  predict step.
- Call `_otos.readTransformed(otosOut)` if the OTOS lag timer has elapsed; call
  `_est.updateOtos(otosOut, now)` for the EKF correction step.
- Copy `_est` fields into `_state.fused`, `_state.encoder`, `_state.optical`,
  `_state.enc_mm_`, `_state.vel_mms_`, `_state.enc`, `_state.otos`,
  `_state.wheel_wedged_`, `_state.connected`.

**`tickAction(now)`** — ACT phase:
- Read `_pendCmd`. Switch on `_pendCmd.control_kind`:
  - `TWIST`: if `!capabilities().get_holonomic()` and `vy != 0`, reject (log,
    return empty batch). Else call `_bvc.setTwist(vx, omega)` (or equivalent) and
    let `_mc.controlTick()` run the wheel PIDs → motor outputs.
  - `WHEELS`: call per-wheel `_mc.setWheelSpeed(i, target)` for each populated
    `WheelTarget`.
  - `NEUTRAL(BRAKE)`: call `_motorL.setSpeed(0)` + `_motorR.setSpeed(0)` (or brake
    mode if `IVelocityMotor` has one).
  - `NEUTRAL(COAST)`: call coast equivalent.
  - `POSE` (SetPose): call `_est.resetPose(x, y, h)` — re-anchors the fused estimate.
    The old `handleSI` / `estimate.resetPose` path.
  - `NONE`: no-op (command not staged).
- Return empty `msg::CommandBatch{}` (Drive is a leaf actuator; no outbound commands
  in this sprint).

**`configure(cfg)`**: stores `_drvCfg = cfg`. Next `tick*()` reads updated gains,
lag, etc. (Generalizes `MotorController::updateVelGains`.)

**`capabilities()`**: returns `DrivetrainCapabilities{holonomic=false, onboard_position=false,
wheel_count=2}` for the differential (Tovez) build. The mecanum build sets
`holonomic=true`. Use `_drvCfg.get_drivetrain_type()` == `MECANUM` (or the
`RobotConfig::drivetrainType` field) to set `holonomic`.

### 3. `source/subsystems/drive/DriveConfig.cpp`

`toDriveConfig(const RobotConfig& rc) -> msg::DrivetrainConfig`:
Map each `RobotConfig` field to the generated `DrivetrainConfig` setter:
`rc.trackwidthMm` → `setTrackwidthMm()`, `rc.velKp` → `setVelGains(Gains{...})`,
`rc.alphaPos` → `setAlphaPos()`, `rc.ekfQXy` → `setEkfQXy()`, etc.
Trace against `source/types/Config.h` for exact member names.
Motion limits (`aMax`, `vBodyMax`, `yawRateMax`) are NOT mapped here (PlannerConfig scope).

### 4. C-ABI shims `tests/_infra/sim/drive2_api.cpp`

Functions:
- `drive2_api_create(RobotConfig*)` → `void*` — constructs `SimHardware`, all
  control components, and `Drive2`. Returns opaque handle to a heap-allocated fixture
  struct (in test code, heap is fine).
- `drive2_api_apply_twist(handle, vx, vy, omega)` — calls `apply(cmd{TWIST})`.
- `drive2_api_apply_neutral_brake(handle)` — calls `apply(cmd{NEUTRAL{BRAKE}})`.
- `drive2_api_apply_setpose(handle, x, y, h)` — calls `apply(cmd{POSE{x,y,h}})`.
- `drive2_api_tick_update(handle, now_ms)` — calls `tickUpdate(now)`.
- `drive2_api_tick_action(handle, now_ms)` — calls `tickAction(now)`.
- `drive2_api_get_fused_x/y/h(handle)` → `float`.
- `drive2_api_get_connected(handle)` → `int`.
- `drive2_api_capabilities_holonomic(handle)` → `int`.
- `drive2_api_destroy(handle)`.

Update `tests/_infra/sim/CMakeLists.txt` to add `drive2_api.cpp`.

### 5. `tests/simulation/unit/test_drive2_subsystem.py`

Tests (no EKF noise — that's ticket 005):
- `test_twist_advances_pose`: apply twist (vx=200, vy=0, omega=0), tick 20 times
  (tickUpdate+tickAction each), assert fused x > 0 (robot moved forward).
- `test_vy_reject_on_differential`: apply twist (vx=0, vy=50, omega=0), check
  `capabilities_holonomic == 0`, confirm `tickAction` does not crash and motor output
  is zero / command rejected.
- `test_setpose_reanchor`: apply SetPose(50.0, 50.0, 0.5), tickUpdate+tickAction once,
  assert fused x≈50, y≈50, h≈0.5.
- `test_neutral_brake`: apply neutral(BRAKE), tick once, assert motor outputs ~0
  (if accessible via sim shim) or verify state.connected still true.

## Files to Create/Modify

- `source/subsystems/drive/Drive2.h` — NEW
- `source/subsystems/drive/Drive2.cpp` — NEW
- `source/subsystems/drive/DriveConfig.cpp` — NEW (toDriveConfig projection)
- `tests/_infra/sim/drive2_api.cpp` — NEW
- `tests/_infra/sim/CMakeLists.txt` — add `drive2_api.cpp`
- `tests/simulation/unit/test_drive2_subsystem.py` — NEW (4 tests, no EKF noise)

## Acceptance Criteria

- [x] `Drive2` constructor takes `(IMotor& motorL, IMotor& motorR, MotorController&,
      BodyVelocityController&, PhysicalStateEstimate&, Odometry&, IOdometer&, const RobotConfig&)`.
- [x] `apply(DrivetrainCommand)` stages only — no hardware I/O, no return value.
- [x] `tickUpdate(now)` runs encoder collect + OTOS read + EKF predict/correct; updates `_state`.
- [x] `tickAction(now)` runs kinematics → wheel PID → motor outputs; returns `msg::CommandBatch`.
- [x] `state()` returns `const msg::DrivetrainState&` — no copy.
- [x] `configure(DrivetrainConfig)` stores config; next tick picks it up.
- [x] `capabilities()` returns `holonomic=false` for differential, `holonomic=true` for mecanum.
- [x] `vy`-reject: on differential build, `apply` with `vy!=0` results in no-op actuation.
- [x] `SetPose` re-anchor: after `apply(SetPose{x,y,h})` + `tickUpdate` + `tickAction`,
      `state().get_fused().get_pose().get_x_mm()` ≈ x.
- [x] Neutral/brake: after `apply(Neutral{BRAKE})` + tick, wheel outputs ~0.
- [x] `toDriveConfig()` maps at least: `trackwidthMm`, `velKp/Ki/Kff/IMax/Kaw`, `alphaPos`,
      `alphaYaw`, `otosGate`, `ekfQXy`, `ekfQTheta`, `lagOtosMs`, `drivetrainType`.
- [x] No virtual dispatch in `Drive2` control path. No heap in `Drive2`.
- [x] `test_drive2_subsystem.py` 4 tests pass.
- [x] `python build.py --clean` zero errors.
- [x] `uv run python -m pytest` green at baseline + new tests.

## Testing Plan

- **New tests**: `tests/simulation/unit/test_drive2_subsystem.py` — 4 tests (twist,
  vy-reject, setpose, neutral).
- **Regression**: `uv run python -m pytest` full suite.
- **Device compile**: `python build.py --clean` zero errors.
- **Parity smoke**: after applying a VW-equivalent twist command and ticking 10 times,
  the fused x position should be within 10% of what `Drive::periodic()` + odometry
  would produce for the same command (manual check / assert comment in test).

## Verification Command

`uv run python -m pytest tests/simulation/unit/test_drive2_subsystem.py -v && python build.py --clean`
