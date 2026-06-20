---
id: "005"
title: "N-wheel control: MotorController arrays, BVC vy channel, VW/OMNI/STRAFE verbs"
status: done
use-cases:
  - SUC-001
  - SUC-003
  - SUC-004
depends-on:
  - "046-004"
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 046-005: N-wheel control: MotorController arrays, BVC vy channel, VW/OMNI/STRAFE verbs

## Description

This is the largest firmware ticket. It wires the kinematics (T2) through the
full control stack to the command grammar. Specifically:

1. Widen `MotorCommands` / `HardwareState` to 4-element arrays in the mecanum
   build (with L/R accessors for shared code).
2. Generalize `MotorController` to hold 4 motors/VelocityControllers in the
   mecanum build; disable sync-coupling.
3. Add a `vy` profiled channel to `BodyVelocityController`.
4. **Extend the `VW` command to 3-DOF: `VW vx vy omega`** — parse an optional second velocity component; the existing 2-token form `VW v omega` remains valid and sets `vy=0` (back-compatible). The body twist `vy` flows through to `BodyVelocityController`'s new lateral channel and `MecanumKinematics::inverse`.

The differential build must remain byte-identical throughout.

## Approach

### 1. source/types/Inputs.h — widen MotorCommands / HardwareState

Under `#ifdef ROBOT_DRIVETRAIN_MECANUM`, replace the scalar L/R fields with arrays.
The safest approach is **accessor functions** rather than raw reference members
(architecture note: references in `#ifdef` blocks are fragile):

```cpp
#ifdef ROBOT_DRIVETRAIN_MECANUM
struct MotorCommands {
    float    tgtMms[4]  = {};    // [0]=FR [1]=FL [2]=BR [3]=BL
    int16_t  pwm[4]     = {};
    bool     digitalOut[4]  = {};
    int16_t  analogOut[4]   = {};
    bool     digitalDirty[4] = {};
    bool     analogDirty[4] = {};

    // Accessors for shared code that still uses L/R names.
    // FL = index 1 (semantic "left"), FR = index 0 (semantic "right").
    float& tgtLMms() { return tgtMms[1]; }
    float& tgtRMms() { return tgtMms[0]; }
    float  tgtLMms() const { return tgtMms[1]; }
    float  tgtRMms() const { return tgtMms[0]; }
    int16_t& pwmL() { return pwm[1]; }
    int16_t& pwmR() { return pwm[0]; }
};
struct HardwareState {
    float    encMm[4]  = {};     // [FR,FL,BR,BL]
    float    velMms[4] = {};
    // Accessors
    float encLMm() const { return encMm[1]; }
    float encRMm() const { return encMm[0]; }
    float velLMms() const { return velMms[1]; }
    float velRMms() const { return velMms[0]; }
    // ... plus all non-motor fields unchanged ...
    float fusedVy = 0.0f;   // lateral velocity (OTOS-led)
    // (all other fields: poseX/Y/H, otos*, line*, color*, ports*, etc.)
};
#else
// Existing scalar structs — byte-identical.
struct MotorCommands { /* unchanged */ };
struct HardwareState { /* unchanged */ };
#endif
```

**Implementation note**: changing `tgtLMms` from a data member to a method
changes the call syntax at every call site (`cmds.tgtLMms` → `cmds.tgtLMms()`).
Audit all callers in the mecanum build path; in the differential build the
existing scalar members remain. Alternatively, use a thin wrapper struct or
keep reference members if accessor functions cause too many call-site changes.
Pick the approach that minimizes diff noise; document the decision in a comment.

### 2. source/control/MotorController.{h,cpp} — 4-wheel generalization

Under `#ifdef ROBOT_DRIVETRAIN_MECANUM`:
- Constructor takes `IMotor* motors[4]` (or 4 separate refs from
  `MecanumHAL`).
- Members: `IMotor* _motor[4]`, `VelocityController _vc[4]`.
- `setTarget(const float* wheels, int n)`: sets `tgtMms[i]` for all N wheels.
- `controlTick`: iterates over N wheels for velocity update and PID. The
  `refreshedWheel` parameter now encodes which wheel index (0–3) was just
  collected (or 0xFF for "both/all" in a WedgeTest-style path). Update the
  split-phase encoder read to handle 4 wheels.
- **Sync-coupling block** (`if (_cal.syncGain > 0.0f ...)`) is excluded in the
  mecanum build via `#ifndef ROBOT_DRIVETRAIN_MECANUM` around the coupling block.
  Each wheel gets independent PID.
- 2-arg shim `setTarget(float leftMms, float rightMms)` maps to
  `tgtMms[1]=leftMms, tgtMms[0]=rightMms` (FL=left, FR=right).

`startDriveClean`, `startDrive`, `stop` updated to clear all N targets.
`getEncoderPositions` updated for 4 wheels (add `getEncoderPositions(int32_t[4])`
overload; keep the 2-arg form for shared code).

### 3. source/control/BodyVelocityController.{h,cpp} — vy profiled channel

Under `#ifdef ROBOT_DRIVETRAIN_MECANUM`:
- Add `_vy`, `_vyTgt`, `_vyALive` members.
- `setTarget(float v_mms, float omega_rads, float vy_mms = 0.0f)`:
  sets `_vyTgt`. Default `vy_mms = 0.0f` keeps all existing callers unchanged.
- In `advance()`: add a `vy` trapezoid/S-curve channel mirroring the `_v`
  channel (`vyBodyMax`, `aMaxY`, `jMaxY` from config).
- Replace the `BodyKinematics::inverse / saturate` scalar calls with:
  ```cpp
  float _wheels[kWheelCount];
  const int8_t signs[4] = { _cfg.fwdSignFR, _cfg.fwdSignFL,
                              _cfg.fwdSignBR, _cfg.fwdSignBL };
  Kinematics::inverse(BodyTwist3{_v, _vy, _omega}, _geom, signs, _wheels);
  Kinematics::saturate(_wheels, _cfg.vWheelMax, _satWheels);
  // Anti-windup: if saturated, back-calc effective body twist.
  if (saturated) {
      BodyTwist3 backCalc;
      Kinematics::forward(_satWheels, _geom, signs, backCalc);
      _v = backCalc.vx_mmps;
      _vy = backCalc.vy_mmps;
      _omega = backCalc.omega_rads;
  }
  _mc.setTarget(_satWheels, kWheelCount);
  ```
- `_geom` member (`RobotGeometry`) built once from config in the constructor:
  `_geom = { _cfg.halfTrackMm, _cfg.halfWheelbaseMm }`.
- `reset()`: clear `_vy`/`_vyTgt`/`_vyALive`.
- `atTarget()`: add `vy` convergence check.

### 4. source/app/MotionCommandHandlers.cpp — VW extended to 3-DOF

`VW` is extended to accept an optional third argument:

```
VW <vx> <omega>           — existing 2-token form; vy=0 (back-compatible)
VW <vx> <vy> <omega>      — new 3-token form (mecanum only; parsed under #ifdef)
```

Under `#ifdef ROBOT_DRIVETRAIN_MECANUM`, the parser checks for a third token
after `omega`; if present it is treated as `vy` (mm/s). If absent, `vy` defaults
to `0.0f`. Both forms call `bvc.setTarget(vx, omega_rad, vy)` — no new command
verb is introduced.

`VW` remains the single unified body-twist primitive; for mecanum its velocity
simply becomes 2-D (the `vy` component was always zero in the differential build).

## Files to Modify

- `source/types/Inputs.h`
- `source/control/MotorController.h`
- `source/control/MotorController.cpp`
- `source/control/BodyVelocityController.h`
- `source/control/BodyVelocityController.cpp`
- `source/app/MotionCommandHandlers.cpp` (and any associated parse headers)

## Acceptance Criteria

- [x] `uv run --with pytest python -m pytest tests/simulation -q` reports `2137 passed` (differential sim unchanged; regression gate). 2181 passed (2137 original + 44 new 046-005 tests).
- [x] Differential build: `MotorController`, `BodyVelocityController`, `MotorCommands`, `HardwareState` are byte-identical to pre-sprint (verify with `git diff` — all changes behind `#ifdef ROBOT_DRIVETRAIN_MECANUM`).
- [x] Mecanum sim build compiles cleanly with `ROBOT_DRIVETRAIN_MECANUM` defined.
- [x] `VW 200 30` (2-token form) continues to work in both builds (backward-compat: `vy=0` default). Verified by `test_mecanum_vw_bvc.py::TestVW2TokenBackwardCompat`.
- [x] `VW 200 80 30` (3-token mecanum form) accepted; `vx=200 mm/s, vy=80 mm/s, omega=30 deg/s` threaded through to BVC in mecanum build. Verified by `test_mecanum_vw_bvc.py::TestVW3TokenMecanum`.
- [x] No `OMNI` or `STRAFE` verbs are added; `VW` is the sole body-twist primitive.
- [x] Mecanum build: `controlTick` runs PID for all 4 wheels; 4 `Motor::setSpeed` calls per tick.
- [x] Sync-coupling is disabled in the mecanum build (verified: `syncGain` path is compiled out).
- [x] Anti-windup back-calculation uses `MecanumKinematics::forward` in the mecanum build.

## Testing

- **Regression gate**: `uv run --with pytest python -m pytest tests/simulation -q` — must report 2137 passed (differential sim golden).
- **Sim smoke-test**: mecanum sim build compiles and the robot initializes without crash.
- **New tests**: sim-level parse test for `VW vx vy omega` 3-token form if harness supports it; confirm `VW v omega` (2-token) still passes.
- **HITL gate (T8)**: bench verification that forward + lateral + turn work on hardware via `VW`.
- **Verification command**: `uv run --with pytest python -m pytest tests/simulation -q`

## Implementation Notes

- Read `source/app/MotionCommandHandlers.cpp` carefully before writing — understand
  how the existing `VW` verb structures its parse/handle pair and how it calls
  `BodyVelocityController::setTarget`. The 3-DOF extension is additive: parse a
  third optional token under `#ifdef ROBOT_DRIVETRAIN_MECANUM` before dispatching.
- The `refreshedWheel` encoding in `controlTick` for 4 wheels: use values 0–3 for
  "only wheel N was refreshed", or use a bitmask. Check how `MecanumHAL::tick()`
  signals the collected wheel before implementing the 4-wheel path.
- `MotorCommands` accessor function vs reference member: this is the "minor issue"
  from the architecture review. Document whichever approach is chosen and why.
