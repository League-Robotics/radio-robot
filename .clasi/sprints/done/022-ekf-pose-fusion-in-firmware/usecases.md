---
status: draft
---

# Sprint 022 Use Cases

## SUC-001: EKF Predict Step Tracks Encoder Dead-Reckoning

- **Actor**: Firmware odometry task (called every control tick via `Odometry::predict()`)
- **Preconditions**: EKF is initialised with `initEKF()`; encoder deltas `dCenter` and
  `dTheta` are available from the midpoint integration already computed in `predict()`.
- **Main Flow**:
  1. `Odometry::predict()` computes `dCenter`, `dTheta`, and `thetaMid` using the
     existing midpoint kinematics.
  2. `predict()` calls `_ekf.predict(dCenter, dTheta, theta_before)`.
  3. The EKF propagates state `[x, y, theta]` using the arc-segment motion model and
     advances the covariance matrix P via the Jacobian F.
  4. `predict()` writes the EKF state back into `s.poseX`, `s.poseY`, `s.poseHrad`.
- **Postconditions**: `HardwareState` pose reflects the EKF prediction; P has grown by Q.
- **Acceptance Criteria**:
  - [ ] After a straight 100 mm forward predict step, `x` advances by ~100 mm and P[0][0]
        grows by `ekfQxy`.
  - [ ] After a 90-degree turn predict step, `theta` is near ±π/2 and P[2][2] grows by
        `ekfQtheta`.
  - [ ] Predict across the ±π heading boundary yields `theta` in (-π, π].

---

## SUC-002: EKF Update Step Corrects Pose from OTOS Position

- **Actor**: Firmware OTOS correction task (`Robot::otosCorrect()` at ~100 ms cadence)
- **Preconditions**: OTOS sensor returns a valid position; EKF covariance P is non-zero.
- **Main Flow**:
  1. `Robot::otosCorrect()` reads `p.x`, `p.y` from `otos.readTransformed(config)`.
  2. `Robot::otosCorrect()` calls `odometry.correctEKF(state.inputs, p.x, p.y)`.
  3. `Odometry::correctEKF()` calls `_ekf.update(x_otos, y_otos)`.
  4. The EKF computes the 2D innovation, innovation covariance S (analytically inverted),
     and 3×2 Kalman gain K. State is corrected; P shrinks by the information gained.
  5. `correctEKF()` writes the updated EKF state back into `HardwareState`.
- **Postconditions**: Pose is moved toward the OTOS observation by the Kalman gain; P is
  smaller than before the update; `otosH` remains in `state.inputs.otosH` for telemetry.
- **Acceptance Criteria**:
  - [ ] After an update when EKF state is 20 mm from OTOS truth, the corrected pose is
        closer to truth than the pre-update pose.
  - [ ] P[0][0] and P[1][1] decrease after an update.
  - [ ] OTOS heading (`p.h`) is stored in `state.inputs.otosH` but is NOT fed into the EKF
        update (heading channel left for a follow-on sprint).
  - [ ] Repeated predict+update cycles converge pose to within a small error of truth.

---

## SUC-003: EKF Parameters Loaded from Config at Boot

- **Actor**: Robot firmware startup (`Robot` constructor)
- **Preconditions**: `RobotConfig` contains `ekfQxy`, `ekfQtheta`, and `ekfROtosXy` fields
  with values from `DefaultConfig.cpp` (or overridden by SET commands before motion starts).
- **Main Flow**:
  1. `Robot::Robot()` constructs `odometry` (default ctor).
  2. Constructor calls `odometry.initEKF(config.ekfQxy, config.ekfQtheta,
     config.ekfROtosXy)`.
  3. `Odometry::initEKF()` calls `_ekf.init(q_xy, q_theta, r_otos_xy)`, which populates
     the Q diagonal and R scalar.
  4. The EKF state is reset to `[0, 0, 0]` and P to the identity (or zero).
- **Postconditions**: EKF is ready to accept `predict()` and `update()` calls with the
  calibrated noise parameters; defaults match the values tuned in the sprint 021 demo
  notebook (Q_xy=2.0, Q_theta=0.005, R_xy=50.0).
- **Acceptance Criteria**:
  - [ ] `defaultRobotConfig()` returns a config with `ekfQxy = 2.0f`, `ekfQtheta = 0.005f`,
        `ekfROtosXy = 50.0f`.
  - [ ] `gen_default_config.py` emits those three fields in the generated `.cpp`.
  - [ ] EKF initialised with `setPose(0, 0, 0)` has state vector `[0, 0, 0]`.
