---
id: '003'
title: Implement EKFTiny thin layer over ekf_t keeping all robustness layers
status: open
use-cases: ["SUC-002"]
depends-on: ["050-002"]
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: false
---

# Implement EKFTiny thin layer over ekf_t keeping all robustness layers

## Description

Create `source/state/EKFTiny.h` and `source/state/EKFTiny.cpp` implementing the
same public API as `EKF` but backed by `ekf_t` from TinyEKF for the predict/update
linear-algebra core.

The old `source/state/EKF.{h,cpp}` remain untouched. No production call-sites are
changed in this ticket. Both classes coexist in `source/state/` until ticket 005.

### What to keep (robustness layers — do not delete)

Every item below from `EKF.cpp` must be present in `EKFTiny.cpp`:

1. **Arc-segment motion model** (`predict`): `theta_mid`, `dCenter*cos/sin`, `wrapPi`,
   Jacobian `a = -dCenter*sin(theta_mid)`, `b = dCenter*cos(theta_mid)`, dt_s scaling of Q.
   Use `ekf_predict(ekf, fx, F, Q_scaled)` for the 5x5 F*P*Fᵀ+Q computation; compute
   `fx` and `F` arrays yourself before the call.

2. **updatePosition** (M=2): compute `y0 = x_otos - ekf.x[0]`, `y1 = y_otos - ekf.x[1]`,
   S = H*P*Hᵀ + R (2x2, analytic: `s00 = P[0][0]+r`, `s01 = P[0][1]`, etc.), analytic
   2x2 inversion, Mahalanobis distance `d2 <= 5.99` gate, D3 P-inflation re-baseline
   after 10 consecutive position rejections (`_rejPos_streak`), then call `ekf_update`
   with the M=2 H, z, hx, R arrays.
   Note on Cholesky vs analytic: `ekf_update` internally calls `invert()` which uses
   Cholesky. To guarantee numerical parity with the Python oracle (which uses analytic
   2x2 inverse), prefer to compute S⁻¹ analytically in `EKFTiny` and bypass
   `ekf_update`'s inversion for this channel — or verify parity in ticket 004 first and
   adjust if needed.

3. **updateHeading** (M=1, scalar): `y = wrapPi(theta_meas - ekf.x[2])`,
   `s = P[2][2] + r_theta`, gate `y²/s <= 3.84`, D3 P-inflation on `_rejHead_streak >= 10`
   (set `ekf.P[2*5+2] = kRebaselinePTheta = 1e5f`, zero cross-terms), scalar K = `P[i][2]/s`,
   full 5-element state update, full 5x5 P update. Do NOT call `ekf_update` for this scalar
   channel — apply the scalar update manually (identical to current `updateHeading` code,
   reading/writing `ekf.P[]` and `ekf.x[]` directly).

4. **updateVelocity** (M=1, two sequential scalar updates): identical to current
   `updateVelocity` — two independent scalar Kalman updates (state index 3 for v,
   state index 4 for omega), each gated at `y²/s <= 3.84`. Apply manually by reading/writing
   `ekf.x[]` and `ekf.P[]` directly. Do NOT call `ekf_update` for scalar channels.

5. **wrapPi**: `atan2f(sinf(theta), cosf(theta))` — static method, same as current `EKF::wrapPi`.

6. **setPose**: set `ekf.x[0..4]`, set diagonal P-prior (kPriorXY, kPriorTheta, kPriorV,
   kPriorOmega), zero off-diagonal P. Use `ekf_initialize(ekf, pdiag)` if convenient, or
   write `ekf.P[]` directly — same result.

7. **Rejected counter / streak counters**: `_rejected` (uint32_t), `_rejHead_streak` (int),
   `_rejPos_streak` (int) — same semantics as `EKF`.

### API contract (must be identical to EKF's public interface)

```cpp
class EKFTiny {
public:
    EKFTiny();
    void init(float q_xy, float q_theta, float q_v, float q_omega,
              float r_otos_xy, float r_otos_v, float r_enc_v);
    void setPose(float x, float y, float theta);
    void predict(float dCenter, float dTheta, float theta_before, float dt_s);
    void updatePosition(float x_otos, float y_otos);
    void updateVelocity(float v_meas, float omega_meas, float r_v, float r_omega);
    void updateHeading(float theta_meas, float r_theta);
    float x() const;
    float y() const;
    float theta() const;
    float v() const;
    float omega() const;
    uint32_t rejectedCount() const;
    int getRejectCount() const;
    int rejHeadStreak() const;
    int rejPosStreak() const;
    float pDiag(int idx) const;
private:
    ekf_t _ekf;
    float _Q[5][5];
    float _rOtosXy, _rOtosV, _rEncV;
    uint32_t _rejected;
    int _rejHead_streak, _rejPos_streak;
    static float wrapPi(float theta);
    static constexpr float kPriorXY = 100.0f;
    static constexpr float kPriorTheta = 0.00762f;
    static constexpr float kPriorV = 100.0f;
    static constexpr float kPriorOmega = 0.01f;
};
```

### tinyekf.h usage in EKFTiny.cpp

At the top of `EKFTiny.cpp`, before the include:
```cpp
#define EKF_N 5
#define EKF_M 2
#include <tinyekf.h>
```

Use `ekf_predict(ekf_ptr, fx, F, Q_scaled)` for the predict step.
Use `ekf_update(ekf_ptr, z, hx, H, R)` only for the M=2 position update channel.
Apply heading and velocity updates as manual scalar updates (write `_ekf.x[]` and `_ekf.P[]` directly).
Use `ekf_initialize(&_ekf, pdiag)` in `init()` and optionally in `setPose()`.

### Hard constraints (all must hold)

- No `#include <vector>`, `<string>`, or any STL header in EKFTiny.{h,cpp}.
- No `new`, `delete`, `throw`, `typeid`, `dynamic_cast`.
- No CODAL headers (`MicroBit.h`, `I2CBus.h`, `microbit_random.h`).
- All arrays on the stack (static sizes from EKF_N/EKF_M constants).
- `EKFTiny.cpp` must compile in both the firmware and host-sim builds without changes
  (dual-build; it is globbed by the `STATE_SOURCES` glob automatically).

## Acceptance Criteria

- [ ] `source/state/EKFTiny.h` exists with the public API matching `EKF` exactly.
- [ ] `source/state/EKFTiny.cpp` exists with `#define EKF_N 5` and `#define EKF_M 2` before `#include <tinyekf.h>`.
- [ ] `ekf_predict` is called in `EKFTiny::predict()` for the F*P*Fᵀ+Q step.
- [ ] `ekf_update` is called (or bypassed with analytic equivalent) in `EKFTiny::updatePosition()` for the M=2 update.
- [ ] `updateHeading` and `updateVelocity` apply scalar updates manually (no `ekf_update` call) — same numerical path as `EKF.cpp`.
- [ ] All robustness layers present: arc-segment model, Mahalanobis gating (all three channels), D3 P-inflation recovery for position and heading, wedge-aware omega suppression is supported via the `updateVelocity` omega_obs=0 path.
- [ ] `source/state/EKF.{h,cpp}` are NOT deleted or modified.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` passes with no new failures beyond the 2 pre-existing baseline. (Parity of EKFTiny vs test_ekf.py is verified in ticket 004; this ticket just confirms the build is clean.)

## Implementation Plan

### Approach

Write EKFTiny as a near-copy of EKF with the following structural changes:
- Replace `float _P[5][5]` and `float _x[5]` private members with `ekf_t _ekf`.
- In `predict()`: build the `fx[5]` and `F[25]` flat arrays (same arithmetic as current `EKF::predict`), then call `ekf_predict(&_ekf, fx, F, Q_scaled)`.
- In `updatePosition()`: compute S, S⁻¹, Mahalanobis d², and (if gate passes) call `ekf_update` with M=2 H/z/hx/R arrays. D3 inflation writes to `_ekf.P[i*5+j]` directly.
- In `updateHeading()` and `updateVelocity()`: copy the scalar update math from `EKF.cpp` unchanged, reading/writing `_ekf.x[]` and `_ekf.P[]` directly.

### Files to create

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/state/EKFTiny.h`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/state/EKFTiny.cpp`

### Files NOT modified in this ticket

- `source/state/EKF.h`, `source/state/EKF.cpp` — untouched
- `source/control/Odometry.h`, `source/control/Odometry.cpp` — untouched
- Any test file — untouched

### Testing plan

**Verification command:** `uv run --with pytest python -m pytest tests/simulation -q`

After this ticket the sim build includes `EKFTiny.cpp` via the `STATE_SOURCES` glob.
EKFTiny is not wired into any production code yet, so no existing test exercises it directly.
Confirm the build succeeds and no new failures appear.

Parity verification (test_ekf.py passing against EKFTiny) is the scope of ticket 004.

### Documentation updates

None required beyond inline comments in EKFTiny.{h,cpp}.
