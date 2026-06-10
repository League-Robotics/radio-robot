---
id: '001'
title: EKF class (EKF.h / EKF.cpp)
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on: []
issue: plan-ekf-pose-fusion-in-firmware.md
completes_issue: false
---

# T001: EKF class (EKF.h / EKF.cpp)

## Description

Create the self-contained 3-state Extended Kalman Filter class. This is the
mathematical core of sprint 022 — it has no dependencies on HAL, OTOS, or any
external math library. Once it compiles and passes T005's Python-mirrored unit
tests, T003 can integrate it into Odometry.

State vector: `x = [x_mm, y_mm, theta_rad]`. All matrix operations (3x3, 3x2)
are unrolled as plain float arithmetic. No heap allocation.

## Acceptance Criteria

- [x] `source/control/EKF.h` exists with exactly this public API:
  ```cpp
  class EKF {
  public:
      EKF();
      void init(float q_xy, float q_theta, float r_otos_xy);
      void setPose(float x, float y, float theta);
      void predict(float dCenter, float dTheta, float theta_before);
      void update(float x_otos, float y_otos);
      float x() const;
      float y() const;
      float theta() const;
  private:
      float _x[3];
      float _P[3][3];
      float _Q[3][3];
      float _r;
  };
  ```
- [x] `source/control/EKF.cpp` exists implementing all methods.
- [x] `EKF()` default constructor zeroes `_x` and `_P` (safe before `init()` is
  called on embedded target).
- [x] `init(q_xy, q_theta, r_otos_xy)` sets `_Q` diagonal to
  `{q_xy, q_xy, q_theta}` and `_r = r_otos_xy`; resets `_x` and `_P` to zero.
- [x] `setPose(x, y, theta)` sets `_x[0..2]` and resets `_P` to zero.
- [x] `predict()` implements the arc-segment motion model exactly:
  - `theta_mid = theta_before + dTheta/2`
  - `dx = dCenter * cosf(theta_mid)`, `dy = dCenter * sinf(theta_mid)`
  - `_x[0] += dx`, `_x[1] += dy`, `_x[2] = wrapPi(_x[2] + dTheta)`
  - Jacobian F is 3x3 identity with `F[0][2] = -dCenter*sinf(theta_mid)` and
    `F[1][2] = dCenter*cosf(theta_mid)`
  - `P = F*P*F^T + Q` fully unrolled (all 9 entries, exploiting symmetry allowed)
- [x] `update()` implements the 2D position-only EKF update:
  - H = [[1,0,0],[0,1,0]] (position-only observation; heading not observed)
  - innovation: `y_inn[0] = x_otos - _x[0]`, `y_inn[1] = y_otos - _x[1]`
  - S (2x2): `S[i][j] = (H*P*H^T)[i][j] + _r*(i==j)`
  - S inverted analytically: `det = S[0][0]*S[1][1] - S[0][1]*S[1][0]`
  - K (3x2 Kalman gain): `K = P*H^T*S_inv`
  - `_x += K * y_inn`; `P = (I - K*H)*P`; `_x[2] = wrapPi(_x[2])`
- [x] `wrapPi` implemented as `atan2f(sinf(theta), cosf(theta))`.
- [x] No `<random>`, no `new`/`malloc`, no STL containers.
- [x] File compiles cleanly in both embedded and host builds (`python3 build.py`
  produces no errors related to EKF.h or EKF.cpp).

## Implementation Plan

### Approach

Create `source/control/EKF.h` and `source/control/EKF.cpp`. Follow the coding
style of `Odometry.h/.cpp`: section comment headers, `static constexpr float`
for pi if needed.

The build system auto-discovers all `.cpp` files in `source/control/`, so no
build script changes are needed.

**P = F*P*F^T + Q — unrolled procedure:**

Let `a = -dCenter*sinf(theta_mid)` and `b = dCenter*cosf(theta_mid)`.
F is identity except F[0][2]=a, F[1][2]=b.

Computing F*P: the product only differs from P in column 2 and in row 0/1 due
to the non-identity entries. Work it out symbolically before coding to get the
minimal set of multiplications.

**2x2 analytic inverse:**

```
det = S00*S11 - S01*S10
S_inv = [[S11/det, -S01/det], [-S10/det, S00/det]]
```

Implement the full formula (not just diagonal shortcut) so it remains correct
when P acquires off-diagonal terms after non-axis-aligned motion.

### Files to create

- `source/control/EKF.h`
- `source/control/EKF.cpp`

### Files to modify

None.

### Testing plan

Run `python3 build.py` to confirm the new files compile. Full mathematical
correctness is verified by the Python-mirror tests in T005 (which can be written
and run independently of the C++ firmware build).

```
python3 build.py
uv run --with pytest python -m pytest tests/dev/test_ekf.py -v
```

### Documentation updates

None. The API is self-documenting via the header file comment.
