---
id: "005"
title: "N-wheel control: MotorController arrays, BVC vy channel, VW/OMNI/STRAFE verbs"
status: open
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
4. Add `VW vy=<val>` optional argument, `OMNI vx vy omega`, and `STRAFE vy [t=|dist=]` command verbs.

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

### 4. source/app/MotionCommandHandlers.cpp — VW/OMNI/STRAFE verbs

`VW` parser: add optional `vy=<float>` argument (parsed after omega; absent → 0).
Pass `vy` to `BodyVelocityController::setTarget(v, omega, vy)`.

Under `#ifdef ROBOT_DRIVETRAIN_MECANUM`:
```
OMNI <vx> <vy> <omega>   — 3-DOF velocity command (mm/s, mm/s, deg/s)
STRAFE <vy> [t=<s>|dist=<mm>]  — pure lateral (vx=0, omega=0)
```

`OMNI` parse: read 3 floats; convert omega from deg/s to rad/s; call
`bvc.setTarget(vx, omega_rad, vy)`.

`STRAFE`: read `vy`; optional `t=` (time-bounded, same as existing `T`-command
machinery) or `dist=` (distance-bounded using OTOS y-pose delta — see OQ-4
in architecture-update.md; implement using OTOS y-pose delta, document the OTOS
dependency). Set `bvc.setTarget(0.0f, 0.0f, vy)` then start the bounded timer
or OTOS-pose watcher.

`STRAFE dist=` stop condition: capture `s.inputs.otosY` at command start;
stop when `|s.inputs.otosY - startY| >= dist`. This requires OTOS to be healthy.
If OTOS is invalid (status non-zero), fall back to time-bounded 5-second safety
stop and emit `EVT strafe_no_otos`.

## Files to Modify

- `source/types/Inputs.h`
- `source/control/MotorController.h`
- `source/control/MotorController.cpp`
- `source/control/BodyVelocityController.h`
- `source/control/BodyVelocityController.cpp`
- `source/app/MotionCommandHandlers.cpp` (and any associated parse headers)

## Acceptance Criteria

- [ ] `uv run --with pytest python -m pytest tests/simulation -q` reports `2093 passed` (differential sim unchanged).
- [ ] Differential build: `MotorController`, `BodyVelocityController`, `MotorCommands`, `HardwareState` are byte-identical to pre-sprint (verify with `git diff` — all changes behind `#ifdef ROBOT_DRIVETRAIN_MECANUM`).
- [ ] Mecanum sim build compiles cleanly with `ROBOT_DRIVETRAIN_MECANUM` defined.
- [ ] `OMNI` and `STRAFE` verbs parse correctly in mecanum sim (add sim parse smoke-test if the test harness supports it).
- [ ] `VW 200 30` (no `vy=`) continues to work in both builds (backward-compat default `vy=0`).
- [ ] `VW 200 30 vy=50` accepted and `vy` threaded through to BVC in mecanum build.
- [ ] `OMNI 200 80 30` accepted; `vx=200 mm/s, vy=80 mm/s, omega=30 deg/s` reached by BVC.
- [ ] `STRAFE 150` accepted; robot translates laterally on the bench (HITL verification in T8).
- [ ] `STRAFE 150 dist=500` accepted; stops when OTOS y-delta reaches 500mm (HITL in T8).
- [ ] Mecanum build: `controlTick` runs PID for all 4 wheels; 4 `Motor::setSpeed` calls per tick.
- [ ] Sync-coupling is disabled in the mecanum build (verified: `syncGain` path is compiled out).
- [ ] Anti-windup back-calculation uses `MecanumKinematics::forward` in the mecanum build.

## Testing

- **Regression gate**: `uv run --with pytest python -m pytest tests/simulation -q`
- **Sim smoke-test**: mecanum sim build compiles and the robot initializes without crash.
- **New tests**: sim-level parse tests for `OMNI`/`STRAFE` verbs if harness supports it.
- **HITL gate (T8)**: bench verification that forward + strafe + turn work on hardware.
- **Verification command**: `uv run --with pytest python -m pytest tests/simulation -q`

## Implementation Notes

- Read `source/app/MotionCommandHandlers.cpp` carefully before writing — understand
  how existing verbs (`VW`, `T`, `D`) structure their parse/handle pairs, bounded
  command state, and the `LoopScheduler` integration. Model `STRAFE` on `T`.
- The `refreshedWheel` encoding in `controlTick` for 4 wheels: use values 0–3 for
  "only wheel N was refreshed", or use a bitmask. Check how `MecanumHAL::tick()`
  signals the collected wheel before implementing the 4-wheel path.
- `MotorCommands` accessor function vs reference member: this is the "minor issue"
  from the architecture review. Document whichever approach is chosen and why.
