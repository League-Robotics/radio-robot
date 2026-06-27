---
status: in-progress
sprint: '022'
tickets:
- 022-001
---

# Plan: EKF Pose Fusion in Firmware

## Context

The demo notebook (`host_tests/demo_figure_eight.ipynb`) proves that an EKF
beats the current complementary filter at every noise level:
- Dead reckoning (Exp 1): ~49 mm RMS cross-track error
- Complementary filter (Exp 2, OTOS + camera): ~32 mm
- EKF (Exp 3): ~19 mm

The firmware currently uses `Odometry::correct()` — a fixed-alpha complementary
blend with an outlier gate. The code already has a comment: *"EKF upgrade path
replaces this later."* This plan delivers that upgrade.

The EKF fuses two signals:
- **Predict**: encoder-derived arc increments (dCenter, dTheta) — exact-arc
  integration, same as the current `predict()` step
- **Update**: OTOS x/y position, at ~100 ms cadence (same as the current
  `correct()` cadence)

Between OTOS updates the EKF pose equals encoder dead-reckoning. After each
OTOS update the Kalman gain applies an optimal correction — larger when the
covariance P is high (encoders have drifted), smaller when it's low (recent
OTOS fix). This replaces the fixed `alphaPos = 0.15` with a gain that adapts
automatically to accumulated uncertainty.

---

## Files to Create

### `source/control/EKF.h`

3-state EKF: `x = [x_mm, y_mm, θ_rad]`. No external math library — inline
float arithmetic. Public API:

```cpp
class EKF {
public:
    EKF(float q_xy, float q_theta, float r_otos_xy);
    void setPose(float x, float y, float theta);
    // Called from Odometry::predict() — same dCenter/dTheta already computed there.
    void predict(float dCenter, float dTheta, float theta_before);
    // Called from Odometry::correctEKF() — OTOS position observation only.
    void update(float x_otos, float y_otos);
    float x() const; float y() const; float theta() const;
private:
    float _x[3];      // state vector
    float _P[3][3];   // covariance
    float _Q[3][3];   // process noise (diagonal)
    float _r;         // OTOS position noise variance (scalar, same for x and y)
};
```

**Predict math** (encoder-based, not commanded v/ω — more accurate):
```
θ_mid = θ + dTheta/2
dx = dCenter·cos(θ_mid)
dy = dCenter·sin(θ_mid)
State: x += dx; y += dy; θ = wrapPi(θ + dTheta)
Jacobian F:
  [[1, 0, -dCenter·sin(θ_mid)],
   [0, 1,  dCenter·cos(θ_mid)],
   [0, 0,  1                 ]]
P = F·P·F^T + Q
```

**Update math** (2D position-only from OTOS; heading not observed — matching demo):
```
H = [[1,0,0],[0,1,0]]
y = [x_otos - x_est, y_otos - y_est]
S = H·P·H^T + R   (2×2 — inverted analytically: inv([[a,b],[c,d]]) = 1/(ad-bc)·[[d,-b],[-c,a]])
K = P·H^T·S^-1    (3×2 Kalman gain)
x += K·y
P = (I - K·H)·P
θ = wrapPi(θ)
```

### `source/control/EKF.cpp`

Implementation. All matrix ops are unrolled at compile time (3×3/3×2 — no
loops needed, no malloc). `wrapPi` via `atan2f(sinf, cosf)`.

---

## Files to Modify

### `source/control/Odometry.h`

- Add `#include "EKF.h"` and `EKF _ekf` member (constructed with config params
  in `Robot::Robot()` or via an `initEKF(cfg)` call).
- Add `void correctEKF(HardwareState& s, float x_otos, float y_otos)`.
- Keep `correct()` unchanged (still used by existing unit tests; can be removed
  in a follow-on).
- `setPose()` / `zero()` must also call `_ekf.setPose()` so the EKF resets to
  the new pose rather than diverging.

### `source/control/Odometry.cpp`

**`predict()`** — after the midpoint integration that already writes
`s.poseX/Y/Hrad`, add:
```cpp
_ekf.predict(dCenter, dTheta, s.poseHrad - dTheta);  // theta_before
// Write EKF state back as the authoritative pose
s.poseX    = _ekf.x();
s.poseY    = _ekf.y();
s.poseHrad = _ekf.theta();
```
(Between OTOS updates the EKF state is identical to the midpoint integration
result, so this changes nothing until the first `correctEKF()` call.)

**New `correctEKF()`**:
```cpp
void Odometry::correctEKF(HardwareState& s, float x_otos, float y_otos) {
    _ekf.update(x_otos, y_otos);
    s.poseX    = _ekf.x();
    s.poseY    = _ekf.y();
    s.poseHrad = _ekf.theta();
}
```

**`setPose()` / `zero()`** — add `_ekf.setPose(x, y, h)` at end of each.

### `source/robot/Robot.h` / `Robot.cpp`

`Robot::otosCorrect()` (line 177–178):
```cpp
// BEFORE:
odometry.correct(state.inputs, p.x, p.y, p.h, config.alphaPos, config.alphaYaw, config.otosGate);
// AFTER:
odometry.correctEKF(state.inputs, p.x, p.y);
```
`otosH` is still stored in `state.inputs.otosH` for telemetry; heading is NOT
fused via EKF (matching the demo; OTOS heading fusion can be added later).

The EKF needs to be initialized with Q/R config values. The cleanest place is
`Robot::Robot()` or a new `Robot::initEKF()` called from the constructor — pass
`config.ekfQxy`, `config.ekfQtheta`, `config.ekfROtosXy` to `Odometry::initEKF(cfg)`.

### `source/types/Config.h`

Add to `RobotConfig` after the existing `otosGate` block:
```cpp
// EKF sensor fusion parameters
float ekfQxy;       // process noise: position (mm²) — default 2.0
float ekfQtheta;    // process noise: heading (rad²) — default 0.005
float ekfROtosXy;   // OTOS measurement noise: position (mm²) — default 50.0
```

### `scripts/gen_default_config.py`

Add to the output template (near the `alphaPos` block):
```cpp
// EKF sensor fusion
p.ekfQxy         = 2.0f;
p.ekfQtheta      = 0.005f;
p.ekfROtosXy     = 50.0f;
```
These match Experiment 3's tuned values from the demo.

### `source/robot/DefaultConfig.cpp`

Regenerated automatically by `gen_default_config.py` — do not edit by hand.

---

## Unit Tests

### New `tests/dev/test_ekf.py`

Python mirror of `EKF` (matching the class from the demo notebook), with tests:
- **Predict**: straight step moves x correctly; covariance P grows monotonically.
- **Update**: state moves toward observation; P shrinks.
- **Convergence**: repeated predict+update cycles drive state to truth.
- **No-update drift**: P diverges without corrections (quantifies why EKF > complementary filter).
- **Heading wrap**: predict across ±π boundary stays in (-π, π].

Pattern: mirrors `test_otos_fusion.py` — pure Python, no hardware needed.

---

## Verification

1. **Unit tests**: `uv run --with pytest python -m pytest tests/dev/test_ekf.py -v`
2. **Full test suite**: `uv run --with pytest python -m pytest` — no regressions.
3. **Sim notebook**: Re-run `demo_figure_eight.ipynb` — add Experiment 4 cell
   that drives the sim using ONLY firmware's built-in EKF (read pose via
   `get_state()`, not a Python-side EKF). Exp 4 should achieve similar RMS
   cross-track error to Exp 3 (~19 mm).
4. **Bench verification**: `uv run rogo` drive commands, observe telemetry pose
   fields for stability (no oscillation, clean convergence after turns).

---

## Out of scope for this sprint

- Fusing OTOS heading in the EKF update (H becomes 3×1; S is scalar — actually
  simple, but leave for follow-on to keep scope tight).
- Camera-fix injection (delayed high-accuracy update) — the firmware doesn't
  have a camera fix input path yet.
- SET/GET keys for `ekfQxy` / `ekfQtheta` / `ekfROtosXy` — add if tuning is
  needed, but start without.
