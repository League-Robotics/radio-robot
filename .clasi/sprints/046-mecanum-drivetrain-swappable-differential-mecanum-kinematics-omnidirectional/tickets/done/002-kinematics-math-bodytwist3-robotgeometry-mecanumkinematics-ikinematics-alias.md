---
id: '002'
title: 'Kinematics math: BodyTwist3, RobotGeometry, MecanumKinematics, IKinematics
  alias'
status: done
use-cases:
- SUC-001
- SUC-003
- SUC-004
depends-on:
- 046-001
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 046-002: Kinematics math: BodyTwist3, RobotGeometry, MecanumKinematics, IKinematics alias

## Description

Implement the full kinematics layer: new POD types (`BodyTwist3`,
`RobotGeometry`), compile-time namespace alias (`IKinematics.h`), array-form
overloads on `BodyKinematics`, and the new `MecanumKinematics.{h,cpp}` module
with inverse/forward/saturate for a 4-wheel X-roller mecanum drivetrain.

This ticket is purely math and types — no firmware integration yet. The output
is tested exclusively with host unit tests.

## Approach

### 1. source/io/capability/Pose2D.h — add BodyTwist3 and RobotGeometry

```cpp
struct BodyTwist3    { float vx_mmps, vy_mmps, omega_rads; };
struct RobotGeometry { float halfTrackMm, halfWheelbaseMm; };
```

These are additive; existing `BodyTwist` and `Pose2D` are unchanged.

### 2. source/control/BodyKinematics.{h,cpp} — add array-form overloads

Add alongside the existing scalar forms (which are kept verbatim):

```cpp
// Array overloads — differential adapter (vy always 0; wheels[2] = [L, R]).
void inverse(BodyTwist3 t, float b, float wheels[2]);
void forward(const float wheels[2], float b, BodyTwist3& t_out);
void saturate(float wheels[2], int n, float vWheelMax, float steerHeadroom,
              float out[2]);
```

Implementation: `inverse` calls through to the scalar form with `vy` ignored;
`forward` calls through to the scalar form and sets `t_out.vy_mmps = 0.0f`;
`saturate` mirrors the existing scalar logic over an array.

### 3. source/control/MecanumKinematics.h + MecanumKinematics.cpp

Canonical wheel order: `[0]=FR, [1]=FL, [2]=BR, [3]=BL` (Nezha ports 1,2,3,4).
Combined geometry constant: `k = halfTrackMm + halfWheelbaseMm`.
The `fwd_sign_*` values from `RobotConfig` are passed to `inverse` and applied
to each wheel output so signs are encapsulated here.

```cpp
namespace MecanumKinematics {

// inverse: body twist → 4 wheel speeds (mm/s, after fwd_sign application).
// geom: RobotGeometry; signs: {signFR, signFL, signBR, signBL} from RobotConfig.
void inverse(BodyTwist3 t, const RobotGeometry& geom,
             const int8_t signs[4], float wheels[4]);

// forward: 4 wheel speeds → body twist (mm/s, rad/s).
// signs applied in reverse (divide out).
void forward(const float wheels[4], const RobotGeometry& geom,
             const int8_t signs[4], BodyTwist3& t_out);

// saturate: uniform scale when any |wheel| > vWheelMax.
// Preserves twist direction (no per-wheel clipping).
void saturate(float wheels[4], float vWheelMax, float out[4]);

} // namespace MecanumKinematics
```

Equations (from architecture-update.md §A.B):
```
inverse:
  FR_raw = vx - vy - k*omega;  wheels[0] = FR_raw * signs[0]
  FL_raw = vx + vy + k*omega;  wheels[1] = FL_raw * signs[1]
  BR_raw = vx + vy - k*omega;  wheels[2] = BR_raw * signs[2]
  BL_raw = vx - vy + k*omega;  wheels[3] = BL_raw * signs[3]

forward (signs divide out — divide wheel speed by sign before summing):
  w[i] = wheels[i] / signs[i]  (or * signs[i] since signs are ±1)
  vx    = (w[0] + w[1] + w[2] + w[3]) / 4
  vy    = (-w[0] + w[1] + w[2] - w[3]) / 4
  omega = (-w[0] - w[1] + w[2] + w[3]) / (4 * k)
```

### 4. source/control/IKinematics.h — compile-time namespace alias

```cpp
#pragma once
#ifdef ROBOT_DRIVETRAIN_MECANUM
  #include "MecanumKinematics.h"
  namespace Kinematics = MecanumKinematics;
  constexpr int kWheelCount = 4;
#else
  #include "BodyKinematics.h"
  namespace Kinematics = BodyKinematics;
  constexpr int kWheelCount = 2;
#endif
```

### 5. Host unit tests

Create `tests/unit/test_mecanum_kinematics.py` (or `.cpp` host test):

- **Round-trip**: `inverse` then `forward` recovers the original twist (within 1e-4).
- **Pure forward** (`vx=200, vy=0, omega=0`): all wheels equal `200 * sign_i`.
- **Pure strafe** (`vx=0, vy=150, omega=0`): FR/BL = `-150 * sign`, FL/BR = `+150 * sign`.
- **Pure rotate** (`vx=0, vy=0, omega=1.0`): FR/BR negative, FL/BL positive (or
  reverse depending on sign convention; verify against the formula).
- **Saturation**: oversized twist → all wheels scaled by `vWheelMax / max(|wi|)`;
  twist direction preserved (ratio check).
- **Identity geometry** (`k=1`): manual computation sanity check.
- **Array BodyKinematics overloads**: round-trip with `vy=0`; result matches
  scalar form within 1e-6.

## Files to Create

- `source/control/MecanumKinematics.h`
- `source/control/MecanumKinematics.cpp`
- `source/control/IKinematics.h`
- `tests/unit/test_mecanum_kinematics.py` (host unit tests)

## Files to Modify

- `source/io/capability/Pose2D.h` (add `BodyTwist3`, `RobotGeometry`)
- `source/control/BodyKinematics.h` (add array-form overload declarations)
- `source/control/BodyKinematics.cpp` (add array-form overload implementations)

## Acceptance Criteria

- [x] Host unit tests pass: `uv run --with pytest python -m pytest tests/simulation/unit/test_mecanum_kinematics.py -v` (44 passed).
- [x] Inverse → forward round-trip error < 1e-4 for pure forward, pure strafe, pure rotate, and combined twist.
- [x] Pure strafe (`vy=150, vx=0, omega=0`) produces expected wheel pattern (FR/BL=-150, FL/BR=+150).
- [x] Saturation preserves twist direction (verified by round-trip through `forward` after saturate).
- [x] `BodyKinematics` array overloads produce identical results to scalar form for `vy=0` inputs.
- [x] Differential sim build (`ROBOT_RUN_MODE=SIM`, no `ROBOT_DRIVETRAIN_MECANUM`) still compiles cleanly — `MecanumKinematics.cpp` included in sim (pure math, no HAL deps), `BodyKinematics.cpp` unchanged.
- [x] `uv run --with pytest python -m pytest tests/simulation -q` reports `2137 passed` (2093 + 44 new).
- [x] No new compiler warnings on embedded target (cmake build clean; embedded build gated via firmware CMakeLists.txt filter on differential).

## Testing

- **New unit tests**: `tests/unit/test_mecanum_kinematics.py` (cases above).
- **Regression gate**: `uv run --with pytest python -m pytest tests/simulation -q`
- **Verification command**: `uv run --with pytest python -m pytest tests/unit/test_mecanum_kinematics.py tests/simulation -q`
