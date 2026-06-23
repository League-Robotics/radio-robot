---
id: "006"
title: "OTOS-led odometry + lateral velocity (vy)"
status: done
use-cases:
  - SUC-001
  - SUC-003
  - SUC-004
depends-on:
  - "046-005"
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 046-006: OTOS-led odometry + lateral velocity (vy)

## Description

Extend the OTOS velocity read to surface `vy` (the lateral body velocity
currently discarded) and carry it through `Odometry` as a directly observed
quantity — a simple complementary filter, not a new EKF state. The
differential 5-state EKF path is completely untouched.

After this ticket `SNAP` will report a non-zero `vy` during strafe commands
and the `fusedVy` field in `HardwareState` will contain the OTOS-observed
lateral speed.

## Approach

### 1. source/io/real/OtosSensor.cpp — readVelocityTransformed3

The existing `readVelocityTransformed(BodyTwist& velOut, float headingRad)`
reads the OTOS velocity registers, applies the mount-offset transform, and
returns `{v_mmps, omega_rads}` — discarding `vy`.

Under `#ifdef ROBOT_DRIVETRAIN_MECANUM`, add a new method:

```cpp
// source/io/real/OtosSensor.h (and IOdometer.h — see below)
virtual bool readVelocityTransformed3(BodyTwist3& velOut,
                                      float headingRad = 0.0f) const;
```

Implementation: read the same OTOS velocity registers as `readVelocityTransformed`,
but return all three components:

```cpp
bool OtosSensor::readVelocityTransformed3(BodyTwist3& velOut,
                                           float headingRad) const {
    // Read 6 bytes from VELOCITY registers (vx, vy, omega raw — same burst
    // as readVelocityTransformed but keep the y component).
    // Apply the same mounting-offset transform (odomYawDeg rotation) that
    // readVelocityTransformed already does for vx/omega.
    // The in-chip lever-arm compensation (REG_OFFSET set at begin()) already
    // handles the -51.5 mm offset, so no additional lever-arm needed here.
    ...
    velOut = BodyTwist3{ vx_transformed, vy_transformed, omega_transformed };
    return true; // or false on I2C error
}
```

Check the existing `readVelocityTransformed` implementation in
`source/io/real/OtosSensor.cpp` for the register addresses and transform code.
Copy and extend rather than refactor — the differential path must be unchanged.

Add the declaration to `source/io/capability/IOdometer.h` under
`#ifdef ROBOT_DRIVETRAIN_MECANUM` with a default no-op:

```cpp
#ifdef ROBOT_DRIVETRAIN_MECANUM
virtual bool readVelocityTransformed3(BodyTwist3& velOut,
                                       float headingRad = 0.0f) const {
    (void)velOut; (void)headingRad; return false;
}
#endif
```

### 2. source/control/Odometry.{h,cpp} — fusedVy + complementary filter

Under `#ifdef ROBOT_DRIVETRAIN_MECANUM` in `Odometry`:
- Add `float _fusedVy` member (initialized to 0.0f).
- `correctEKF(...)` is extended with a `vy_otos` parameter in the mecanum
  build. After the existing EKF updates:
  ```cpp
  // Simple complementary filter for lateral velocity (mecanum build only).
  // OTOS directly observes vy; no new EKF state needed.
  _fusedVy = _cfg.otosAlphaVy * vy_otos + (1.0f - _cfg.otosAlphaVy) * _fusedVy;
  s.fusedVy = _fusedVy;
  ```
  `otosAlphaVy` is a new config field (default 0.8 — heavily OTOS-trusting).
  Add it to `RobotConfig` and `gen_default_config.py` in T1 scope, or add here
  with a hardcoded default fallback if T1 is already done.

- `predict()` is unchanged (differential EKF path stays untouched; `_fusedVy`
  is not updated in predict since encoder forward-kinematics don't observe vy
  reliably for a mecanum wheel).

### 3. Robot.cpp — wire readVelocityTransformed3 into otosCorrect

The call site in `Robot::otosCorrect()` (or equivalent) that calls
`_otos.readVelocityTransformed(twist, heading)` should be extended under the
mecanum guard to instead call `readVelocityTransformed3(twist3, heading)` and
pass `twist3.vy_mmps` into `_odometry.correctEKF(...)`.

Locate the exact call site in `source/robot/Robot.cpp` (or wherever
`otosCorrect` is implemented) and add the `#ifdef` branch.

### 4. HardwareState — fusedVy field

`fusedVy` is added to `HardwareState` under `#ifdef ROBOT_DRIVETRAIN_MECANUM`
in `source/types/Inputs.h` (this may already be done as part of T5; coordinate
with T5 implementer).

## Files to Modify

- `source/io/capability/IOdometer.h` (add `readVelocityTransformed3` declaration, gated)
- `source/io/real/OtosSensor.h` (add `readVelocityTransformed3` override, gated)
- `source/io/real/OtosSensor.cpp` (add `readVelocityTransformed3` implementation)
- `source/control/Odometry.h` (add `_fusedVy`, extend `correctEKF` signature, gated)
- `source/control/Odometry.cpp` (complementary filter, gated)
- `source/robot/Robot.cpp` (wire `readVelocityTransformed3` into `otosCorrect`, gated)
- `source/types/Config.h` (add `otosAlphaVy` if not already done in T1)
- `scripts/gen_default_config.py` (emit `otosAlphaVy = 0.8f` if not in T1)
- `source/types/Inputs.h` (add `fusedVy` to `HardwareState` if not done in T5)

## Acceptance Criteria

- [x] `uv run --with pytest python -m pytest tests/simulation -q` reports `2181 passed` → `2207 passed` (2181 differential + 26 new unit tests; differential sim unchanged; `readVelocityTransformed` untouched).
- [x] `readVelocityTransformed` (existing 2-DOF) is byte-identical in the differential build.
- [x] Mecanum sim build (`-DROBOT_DRIVETRAIN=mecanum HOST_BUILD`) compiles and links cleanly with the new `readVelocityTransformed3` method.
- [ ] On hardware (bench HITL gate in T8): `SNAP` after `STRAFE 150` reports `vy=` non-zero (approx ±150 mm/s).
- [ ] `fusedVy` in `HardwareState` is non-zero during strafe commands and near-zero during pure forward/turn (HITL in T8).
- [x] `correctEKF` produces a stable, non-diverging `fusedVy` over a 5-second strafe run (verified by complementary filter convergence + non-divergence unit tests).
- [x] `SimOdometer` (sim) base class `IOdometer::readVelocityTransformed3` default no-op returns `false` — sim tests do not require OTOS lateral velocity.

## Testing

- **Regression gate**: `uv run --with pytest python -m pytest tests/simulation -q`
- **HITL gate (T8)**: `SNAP` `vy=` non-zero during strafe; camera verifies lateral motion.
- **New sim tests**: if the sim harness supports OTOS velocity injection, add a test that
  confirms `fusedVy` tracks the injected OTOS vy through the complementary filter.
- **Verification command**: `uv run --with pytest python -m pytest tests/simulation -q`

## Implementation Notes

- Read `source/io/real/OtosSensor.cpp` `readVelocityTransformed` carefully to
  understand the OTOS register layout and the transform code before writing
  `readVelocityTransformed3`. The OTOS velocity registers are a burst read of 6
  bytes (vx, vy, omega as int16 pairs in order); the existing code discards bytes
  3–4 (the vy pair). Simply include those bytes in the output.
- The OTOS chip's internal mount-offset compensation (written to REG_OFFSET in
  `begin()`) correctly handles the -51.5 mm lever arm, so no extra lever-arm
  code is needed in the transform.
- The `correctEKF` signature change under the mecanum guard must not break
  the differential callers. Add an optional parameter with a default, or use
  a separate `#ifdef` overload.
