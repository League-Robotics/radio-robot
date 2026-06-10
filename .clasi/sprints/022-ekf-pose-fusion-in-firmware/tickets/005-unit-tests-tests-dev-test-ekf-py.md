---
id: "005"
title: "Unit tests (tests/dev/test_ekf.py)"
status: open
use-cases: [SUC-001, SUC-002, SUC-003]
depends-on: ["001"]
completes_issue: true
---

# T005: Unit tests (tests/dev/test_ekf.py)

## Description

Write a pure-Python mirror of the EKF class and a comprehensive pytest suite to
verify its mathematical correctness. This follows the pattern of
`tests/dev/test_otos_fusion.py` — no hardware required, no serial port, just
Python floating-point arithmetic.

The Python EKF must implement exactly the same predict/update math as the C++
`EKF` class created in T001 so that the tests serve as a specification the
firmware implementation must match.

## Acceptance Criteria

- [ ] `tests/dev/test_ekf.py` exists.
- [ ] A `EKF` Python class mirrors the C++ class with `__init__()`, `init()`,
  `set_pose()`, `predict()`, `update()`, `x`, `y`, `theta` properties.
- [ ] Test class `TestPredictStraight`:
  - [ ] Straight step (dCenter=100, dTheta=0, theta_before=0): `x` advances
    ~100 mm, `y` stays ~0, `theta` stays ~0.
  - [ ] `P[0][0]` and `P[1][1]` grow by `q_xy` after one predict step.
  - [ ] `P[2][2]` grows by `q_theta` after one predict step.
- [ ] Test class `TestPredictTurn`:
  - [ ] Pure rotation (dCenter=0, dTheta=pi/2, theta_before=0): `theta` is
    ~pi/2, `x` and `y` near 0.
  - [ ] Combined arc (dCenter=100, dTheta=pi/4, theta_before=0): `x` and `y`
    match the expected midpoint integration values.
- [ ] Test class `TestHeadingWrap`:
  - [ ] Predict across +pi boundary stays in (-pi, pi].
  - [ ] Predict across -pi boundary stays in (-pi, pi].
- [ ] Test class `TestUpdate`:
  - [ ] After an update when EKF state is 20 mm off from OTOS in x: `x` moves
    toward the observation.
  - [ ] `P[0][0]` decreases after an update.
  - [ ] `P[1][1]` decreases after an update.
  - [ ] Heading is not changed by the update (only position is observed).
- [ ] Test class `TestConvergence`:
  - [ ] 30 predict+update cycles with OTOS at truth (0, 0, 0) and initial
    state at (50, 50, 0): final `x` and `y` are within 5 mm of truth.
  - [ ] P covariance decreases (is smaller after 30 cycles than after 1).
- [ ] Test class `TestNoDriftWithoutUpdate`:
  - [ ] P diverges monotonically without corrections (P[0][0] after 10 predicts
    > P[0][0] after 1 predict).
- [ ] Test class `TestSetPose`:
  - [ ] `set_pose(100, 200, 0.5)` sets state to [100, 200, 0.5].
  - [ ] After `set_pose()`, P is zero (or near zero).
  - [ ] Predict after `set_pose()` advances from the new pose, not origin.
- [ ] `uv run --with pytest python -m pytest tests/dev/test_ekf.py -v` passes
  all cases with zero failures.
- [ ] `uv run --with pytest python -m pytest` (full suite) passes with no
  regressions.

## Implementation Plan

### Approach

Pattern: copy the structure of `tests/dev/test_otos_fusion.py`. The file
starts with the Python mirror class, then test classes, then tests.

**Python EKF class structure:**

```python
import math
import pytest

def wrap_pi(theta):
    return math.atan2(math.sin(theta), math.cos(theta))

class EKF:
    def __init__(self):
        self._x = [0.0, 0.0, 0.0]
        self._P = [[0.0]*3 for _ in range(3)]
        self._Q = [[0.0]*3 for _ in range(3)]
        self._r = 0.0

    def init(self, q_xy, q_theta, r_otos_xy):
        self._Q[0][0] = q_xy
        self._Q[1][1] = q_xy
        self._Q[2][2] = q_theta
        self._r = r_otos_xy
        self._x = [0.0, 0.0, 0.0]
        self._P = [[0.0]*3 for _ in range(3)]

    def set_pose(self, x, y, theta):
        self._x = [x, y, theta]
        self._P = [[0.0]*3 for _ in range(3)]

    def predict(self, dCenter, dTheta, theta_before):
        theta_mid = theta_before + dTheta / 2.0
        dx = dCenter * math.cos(theta_mid)
        dy = dCenter * math.sin(theta_mid)
        self._x[0] += dx
        self._x[1] += dy
        self._x[2] = wrap_pi(self._x[2] + dTheta)
        # Jacobian F (3x3): identity + F[0][2]=-dCenter*sin(theta_mid),
        #                                      F[1][2]=+dCenter*cos(theta_mid)
        a = -dCenter * math.sin(theta_mid)
        b =  dCenter * math.cos(theta_mid)
        # P = F*P*F^T + Q  (unrolled)
        # ... (implement fully)

    def update(self, x_otos, y_otos):
        # H = [[1,0,0],[0,1,0]]
        # S = H*P*H^T + R
        # K = P*H^T*S_inv
        # ... (implement fully)
        self._x[2] = wrap_pi(self._x[2])

    @property
    def x(self): return self._x[0]
    @property
    def y(self): return self._x[1]
    @property
    def theta(self): return self._x[2]
```

The programmer must implement the full unrolled `P = F*P*F^T + Q` and the
`update()` matrix math — these are the core of the test's value as a spec.

**Default test parameters:**

Use `q_xy=1.0`, `q_theta=0.01`, `r_otos_xy=10.0` for test clarity (not the
firmware defaults — these are chosen so gains are numerically verifiable).

### Files to create

- `tests/dev/test_ekf.py`

### Files to modify

None.

### Testing plan

```
uv run --with pytest python -m pytest tests/dev/test_ekf.py -v
uv run --with pytest python -m pytest
```

Both commands must pass cleanly.

### Documentation updates

Add a one-line comment at the top of `test_ekf.py` referencing the C++ class it
mirrors and the sprint it was created in (sprint 022, ticket T005).
