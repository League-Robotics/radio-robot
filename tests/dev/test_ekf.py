"""test_ekf.py — Unit tests for EKF class (source/control/EKF.h/.cpp).

Pure-Python mirror of the C++ EKF implementation.
Verifies predict/update math, covariance growth/shrinkage, convergence,
and heading wrap-safety.

Sprint 022, Ticket T005.
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def wrap_pi(theta: float) -> float:
    """Keep heading in (-π, π] using atan2 identity."""
    return math.atan2(math.sin(theta), math.cos(theta))


# ---------------------------------------------------------------------------
# Pure-Python EKF mirror — exactly matches source/control/EKF.cpp
# ---------------------------------------------------------------------------

class EKF:
    """Python mirror of the C++ EKF class (sprint 022, ticket T001).

    State: [x_mm, y_mm, theta_rad]
    Motion model: arc-segment (midpoint integration).
    Observation model: 2D position-only (OTOS x, y; heading not observed).
    """

    def __init__(self):
        self._x = [0.0, 0.0, 0.0]
        self._P = [[0.0] * 3 for _ in range(3)]
        self._Q = [[0.0] * 3 for _ in range(3)]
        self._r = 0.0

    def init(self, q_xy: float, q_theta: float, r_otos_xy: float) -> None:
        """Initialize noise parameters and reset state to origin."""
        self._Q[0][0] = q_xy
        self._Q[1][1] = q_xy
        self._Q[2][2] = q_theta
        self._r = r_otos_xy
        self._x = [0.0, 0.0, 0.0]
        self._P = [[0.0] * 3 for _ in range(3)]

    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Overwrite state with a known pose; reset covariance to zero."""
        self._x = [float(x), float(y), float(theta)]
        self._P = [[0.0] * 3 for _ in range(3)]

    def predict(self, dCenter: float, dTheta: float, theta_before: float) -> None:
        """Predict step: arc-segment motion model.

        Jacobian F is 3x3 identity with:
          F[0][2] = a = -dCenter * sin(theta_mid)
          F[1][2] = b =  dCenter * cos(theta_mid)

        P update: P = F*P*F^T + Q  (fully unrolled, matches EKF.cpp exactly).
        """
        theta_mid = theta_before + dTheta * 0.5
        ct = math.cos(theta_mid)
        st = math.sin(theta_mid)

        self._x[0] += dCenter * ct
        self._x[1] += dCenter * st
        self._x[2] = wrap_pi(self._x[2] + dTheta)

        # Jacobian non-identity entries
        a = -dCenter * st   # F[0][2]
        b =  dCenter * ct   # F[1][2]

        # Unpack P for readability (mirrors EKF.cpp local variables)
        p00 = self._P[0][0]; p01 = self._P[0][1]; p02 = self._P[0][2]
        p10 = self._P[1][0]; p11 = self._P[1][1]; p12 = self._P[1][2]
        p20 = self._P[2][0]; p21 = self._P[2][1]; p22 = self._P[2][2]

        # T = F*P (F is identity with F[0][2]=a, F[1][2]=b):
        #   T[0][j] = P[0][j] + a*P[2][j]
        #   T[1][j] = P[1][j] + b*P[2][j]
        #   T[2][j] = P[2][j]
        t00 = p00 + a * p20;  t01 = p01 + a * p21;  t02 = p02 + a * p22
        t10 = p10 + b * p20;  t11 = p11 + b * p21;  t12 = p12 + b * p22
        t20 = p20;             t21 = p21;             t22 = p22

        # New P = T*F^T + Q
        # (F^T is identity with FT[2][0]=a, FT[2][1]=b):
        #   Result[i][0] = T[i][0] + T[i][2]*a
        #   Result[i][1] = T[i][1] + T[i][2]*b
        #   Result[i][2] = T[i][2]
        self._P[0][0] = t00 + t02 * a + self._Q[0][0]
        self._P[0][1] = t01 + t02 * b
        self._P[0][2] = t02
        self._P[1][0] = t10 + t12 * a
        self._P[1][1] = t11 + t12 * b + self._Q[1][1]
        self._P[1][2] = t12
        self._P[2][0] = t20 + t22 * a
        self._P[2][1] = t21 + t22 * b
        self._P[2][2] = t22 + self._Q[2][2]

    def update(self, x_otos: float, y_otos: float) -> None:
        """Update step: 2D position-only observation from OTOS.

        H = [[1,0,0],[0,1,0]]  — position-only observation.
        S = H*P*H^T + R        — 2x2 innovation covariance.
        K = P*H^T * S_inv      — 3x2 Kalman gain.
        _x += K * y_inn
        P  = (I - K*H) * P
        """
        yi0 = x_otos - self._x[0]
        yi1 = y_otos - self._x[1]

        # Innovation covariance S (2x2)
        s00 = self._P[0][0] + self._r
        s01 = self._P[0][1]
        s10 = self._P[1][0]
        s11 = self._P[1][1] + self._r

        # Analytic 2x2 inverse
        det = s00 * s11 - s01 * s10
        if -1e-9 < det < 1e-9:
            return  # singular — skip update

        inv_det = 1.0 / det
        si00 =  s11 * inv_det
        si01 = -s01 * inv_det
        si10 = -s10 * inv_det
        si11 =  s00 * inv_det

        # Kalman gain K = P*H^T * S_inv  (3x2)
        # P*H^T selects columns 0 and 1 of P
        k00 = self._P[0][0] * si00 + self._P[0][1] * si10
        k01 = self._P[0][0] * si01 + self._P[0][1] * si11
        k10 = self._P[1][0] * si00 + self._P[1][1] * si10
        k11 = self._P[1][0] * si01 + self._P[1][1] * si11
        k20 = self._P[2][0] * si00 + self._P[2][1] * si10
        k21 = self._P[2][0] * si01 + self._P[2][1] * si11

        # State update: _x += K * y_inn
        self._x[0] += k00 * yi0 + k01 * yi1
        self._x[1] += k10 * yi0 + k11 * yi1
        self._x[2] += k20 * yi0 + k21 * yi1
        self._x[2] = wrap_pi(self._x[2])

        # Covariance update: P = (I - K*H) * P
        # (I-KH)*P[i][j] = P[i][j] - K[i][0]*P[0][j] - K[i][1]*P[1][j]
        p00 = self._P[0][0]; p01 = self._P[0][1]; p02 = self._P[0][2]
        p10 = self._P[1][0]; p11 = self._P[1][1]; p12 = self._P[1][2]
        p20 = self._P[2][0]; p21 = self._P[2][1]; p22 = self._P[2][2]

        self._P[0][0] = p00 - k00 * p00 - k01 * p10
        self._P[0][1] = p01 - k00 * p01 - k01 * p11
        self._P[0][2] = p02 - k00 * p02 - k01 * p12
        self._P[1][0] = p10 - k10 * p00 - k11 * p10
        self._P[1][1] = p11 - k10 * p01 - k11 * p11
        self._P[1][2] = p12 - k10 * p02 - k11 * p12
        self._P[2][0] = p20 - k20 * p00 - k21 * p10
        self._P[2][1] = p21 - k20 * p01 - k21 * p11
        self._P[2][2] = p22 - k20 * p02 - k21 * p12

    @property
    def x(self) -> float:
        return self._x[0]

    @property
    def y(self) -> float:
        return self._x[1]

    @property
    def theta(self) -> float:
        return self._x[2]


# ---------------------------------------------------------------------------
# Default test noise parameters (chosen for numerical clarity, not firmware defaults)
# ---------------------------------------------------------------------------

Q_XY    = 1.0
Q_THETA = 0.01
R_XY    = 10.0


# ---------------------------------------------------------------------------
# TestPredictStraight — straight-line motion, covariance growth
# ---------------------------------------------------------------------------

class TestPredictStraight:
    """Straight-line predict: state advances correctly and P grows by Q."""

    def _make_ekf(self) -> EKF:
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        return e

    def test_straight_x_advances(self):
        """dCenter=100, dTheta=0, theta_before=0 → x≈100."""
        e = self._make_ekf()
        e.predict(100.0, 0.0, 0.0)
        assert e.x == pytest.approx(100.0, abs=1e-9)

    def test_straight_y_stays_zero(self):
        """dCenter=100, dTheta=0, theta_before=0 → y≈0."""
        e = self._make_ekf()
        e.predict(100.0, 0.0, 0.0)
        assert e.y == pytest.approx(0.0, abs=1e-9)

    def test_straight_theta_stays_zero(self):
        """dCenter=100, dTheta=0, theta_before=0 → theta≈0."""
        e = self._make_ekf()
        e.predict(100.0, 0.0, 0.0)
        assert e.theta == pytest.approx(0.0, abs=1e-9)

    def test_p00_grows_by_q_xy(self):
        """P[0][0] increases by q_xy after one predict from zero P."""
        e = self._make_ekf()
        # Starting from P=0, after one predict with theta_mid=0:
        # a = -100*sin(0) = 0, b = 100*cos(0) = 100
        # T[0][j] = P[0][j] + 0 = 0; t02 = 0
        # P_new[0][0] = t00 + t02*a + Q[0][0] = 0 + 0 + Q_XY
        e.predict(100.0, 0.0, 0.0)
        assert e._P[0][0] == pytest.approx(Q_XY, abs=1e-9)

    def test_p11_grows_by_q_xy(self):
        """P[1][1] increases by q_xy after one predict from zero P."""
        e = self._make_ekf()
        e.predict(100.0, 0.0, 0.0)
        assert e._P[1][1] == pytest.approx(Q_XY, abs=1e-9)

    def test_p22_grows_by_q_theta(self):
        """P[2][2] increases by q_theta after one predict from zero P."""
        e = self._make_ekf()
        e.predict(100.0, 0.0, 0.0)
        assert e._P[2][2] == pytest.approx(Q_THETA, abs=1e-9)


# ---------------------------------------------------------------------------
# TestPredictTurn — pure rotation and combined arc
# ---------------------------------------------------------------------------

class TestPredictTurn:
    """Turning motion: heading and position integrate correctly."""

    def _make_ekf(self) -> EKF:
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        return e

    def test_pure_rotation_theta(self):
        """dCenter=0, dTheta=pi/2, theta_before=0 → theta≈pi/2."""
        e = self._make_ekf()
        e.predict(0.0, math.pi / 2, 0.0)
        assert e.theta == pytest.approx(math.pi / 2, abs=1e-9)

    def test_pure_rotation_x_stays_zero(self):
        """Pure rotation: x stays at 0 (no translation)."""
        e = self._make_ekf()
        e.predict(0.0, math.pi / 2, 0.0)
        assert e.x == pytest.approx(0.0, abs=1e-9)

    def test_pure_rotation_y_stays_zero(self):
        """Pure rotation: y stays at 0 (no translation)."""
        e = self._make_ekf()
        e.predict(0.0, math.pi / 2, 0.0)
        assert e.y == pytest.approx(0.0, abs=1e-9)

    def test_arc_x_matches_midpoint_integration(self):
        """dCenter=100, dTheta=pi/4, theta_before=0: x = 100*cos(pi/8)."""
        e = self._make_ekf()
        e.predict(100.0, math.pi / 4, 0.0)
        expected_x = 100.0 * math.cos(math.pi / 8)
        assert e.x == pytest.approx(expected_x, abs=1e-9)

    def test_arc_y_matches_midpoint_integration(self):
        """dCenter=100, dTheta=pi/4, theta_before=0: y = 100*sin(pi/8)."""
        e = self._make_ekf()
        e.predict(100.0, math.pi / 4, 0.0)
        expected_y = 100.0 * math.sin(math.pi / 8)
        assert e.y == pytest.approx(expected_y, abs=1e-9)


# ---------------------------------------------------------------------------
# TestHeadingWrap — angle stays in (-π, π] across both wrap boundaries
# ---------------------------------------------------------------------------

class TestHeadingWrap:
    """Heading wrapping across the ±π discontinuity."""

    def _make_ekf(self) -> EKF:
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        return e

    def test_wrap_across_positive_pi(self):
        """Predict across +π boundary: result stays in (-π, π]."""
        e = self._make_ekf()
        # Set theta near +π then add a positive dTheta to push it over
        e.set_pose(0.0, 0.0, math.pi - 0.1)
        e.predict(0.0, 0.3, math.pi - 0.1)
        assert -math.pi < e.theta <= math.pi

    def test_wrap_across_negative_pi(self):
        """Predict across -π boundary: result stays in (-π, π]."""
        e = self._make_ekf()
        # Set theta near -π then subtract dTheta to push it over
        e.set_pose(0.0, 0.0, -(math.pi - 0.1))
        e.predict(0.0, -0.3, -(math.pi - 0.1))
        assert -math.pi < e.theta <= math.pi

    def test_wrap_positive_pi_value_is_correct(self):
        """Crossing +π by 0.2 rad: result should be near -π+0.1."""
        e = self._make_ekf()
        e.set_pose(0.0, 0.0, math.pi - 0.1)
        e.predict(0.0, 0.2, math.pi - 0.1)
        # After wrap: theta should be near -(pi - 0.1) == -pi + 0.1
        expected = wrap_pi(math.pi - 0.1 + 0.2)
        assert e.theta == pytest.approx(expected, abs=1e-9)

    def test_wrap_negative_pi_value_is_correct(self):
        """Crossing -π by -0.2 rad: result should be near +π-0.1."""
        e = self._make_ekf()
        e.set_pose(0.0, 0.0, -(math.pi - 0.1))
        e.predict(0.0, -0.2, -(math.pi - 0.1))
        expected = wrap_pi(-(math.pi - 0.1) - 0.2)
        assert e.theta == pytest.approx(expected, abs=1e-9)


# ---------------------------------------------------------------------------
# TestUpdate — Kalman update pulls state toward observation and reduces P
# ---------------------------------------------------------------------------

class TestUpdate:
    """Kalman update: state moves toward observation, covariance shrinks."""

    def _make_ekf_with_covariance(self) -> EKF:
        """Return an EKF that has non-zero P (needed for K to be non-zero)."""
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        # Run a predict to build up covariance before updating
        e.predict(100.0, 0.0, 0.0)
        # Manually set state to x=20 so we can test the pull toward 0
        e._x[0] = 20.0
        e._x[1] = 0.0
        return e

    def test_x_moves_toward_observation(self):
        """EKF at x=20, OTOS at x=0: after update, x < 20 (moves toward 0)."""
        e = self._make_ekf_with_covariance()
        x_before = e.x
        e.update(0.0, 0.0)
        assert e.x < x_before

    def test_p00_decreases_after_update(self):
        """P[0][0] is smaller after an update (Kalman gain reduces uncertainty)."""
        e = self._make_ekf_with_covariance()
        p00_before = e._P[0][0]
        e.update(0.0, 0.0)
        assert e._P[0][0] < p00_before

    def test_p11_decreases_after_update(self):
        """P[1][1] is smaller after an update."""
        e = self._make_ekf_with_covariance()
        p11_before = e._P[1][1]
        e.update(0.0, 0.0)
        assert e._P[1][1] < p11_before

    def test_theta_not_changed_by_update(self):
        """Heading is not changed by update (only x,y are observed)."""
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        # Run predict to build non-zero P
        e.predict(0.0, 0.0, 0.0)
        # Set a specific non-zero theta
        e._x[2] = 1.0
        theta_before = e.theta
        e.update(0.0, 0.0)
        # Theta should be unchanged (K[2][*] * yi will be near zero when
        # P[2][0] and P[2][1] are zero, which they are after a straight predict)
        assert e.theta == pytest.approx(theta_before, abs=1e-9)


# ---------------------------------------------------------------------------
# TestConvergence — repeated predict+update cycles converge to truth
# ---------------------------------------------------------------------------

class TestConvergence:
    """30 predict+update cycles drive state toward OTOS truth."""

    def test_x_converges_to_truth(self):
        """After 30 cycles with OTOS at (0,0), x within 5mm of truth."""
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        e.set_pose(50.0, 50.0, 0.0)
        for _ in range(30):
            e.predict(0.0, 0.0, 0.0)   # no motion — pure correction test
            e.update(0.0, 0.0)
        assert abs(e.x) < 5.0

    def test_y_converges_to_truth(self):
        """After 30 cycles with OTOS at (0,0), y within 5mm of truth."""
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        e.set_pose(50.0, 50.0, 0.0)
        for _ in range(30):
            e.predict(0.0, 0.0, 0.0)
            e.update(0.0, 0.0)
        assert abs(e.y) < 5.0

    def test_covariance_decreases_over_cycles(self):
        """P[0][0] decreases from its peak (after predicts only) once updates begin.

        Strategy: run 10 predict-only steps to build up a large P, then start
        the predict+update loop. P should shrink from the elevated level toward
        steady state as the Kalman filter folds in observations.
        """
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        e.set_pose(50.0, 50.0, 0.0)

        # Build up covariance via predict-only (no OTOS yet)
        for _ in range(10):
            e.predict(0.0, 0.0, 0.0)
        p00_peak = e._P[0][0]

        # Now run 1 predict+update and record P
        e.predict(0.0, 0.0, 0.0)
        e.update(0.0, 0.0)
        p00_after_1 = e._P[0][0]

        # Run 29 more predict+update cycles
        for _ in range(29):
            e.predict(0.0, 0.0, 0.0)
            e.update(0.0, 0.0)
        p00_after_30 = e._P[0][0]

        # After starting updates, P should have dropped from its peak
        assert p00_after_1 < p00_peak, (
            f"P[0][0] should decrease after first update: {p00_after_1} vs peak {p00_peak}"
        )
        # And P after 30 cycles should be at or below after-1-cycle level
        # (it converges to a steady state that is below the predict-only level)
        assert p00_after_30 <= p00_after_1 + 1e-6, (
            f"P[0][0] after 30 cycles ({p00_after_30}) should be <= after 1 cycle ({p00_after_1})"
        )


# ---------------------------------------------------------------------------
# TestNoDriftWithoutUpdate — covariance grows monotonically without corrections
# ---------------------------------------------------------------------------

class TestNoDriftWithoutUpdate:
    """Without update steps, P diverges (uncertainty grows)."""

    def test_p00_grows_over_predicts(self):
        """P[0][0] after 10 predicts > P[0][0] after 1 predict."""
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)

        e.predict(10.0, 0.0, 0.0)
        p00_after_1 = e._P[0][0]

        for _ in range(9):
            e.predict(10.0, 0.0, 0.0)
        p00_after_10 = e._P[0][0]

        assert p00_after_10 > p00_after_1


# ---------------------------------------------------------------------------
# TestSetPose — set_pose() overwrites state and zeros covariance
# ---------------------------------------------------------------------------

class TestSetPose:
    """set_pose() resets state and covariance correctly."""

    def test_set_pose_sets_x(self):
        """set_pose(100, 200, 0.5) → x=100."""
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        e.set_pose(100.0, 200.0, 0.5)
        assert e.x == pytest.approx(100.0, abs=1e-9)

    def test_set_pose_sets_y(self):
        """set_pose(100, 200, 0.5) → y=200."""
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        e.set_pose(100.0, 200.0, 0.5)
        assert e.y == pytest.approx(200.0, abs=1e-9)

    def test_set_pose_sets_theta(self):
        """set_pose(100, 200, 0.5) → theta=0.5."""
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        e.set_pose(100.0, 200.0, 0.5)
        assert e.theta == pytest.approx(0.5, abs=1e-9)

    def test_set_pose_zeros_covariance(self):
        """After set_pose(), all P entries are zero."""
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        # Build up some covariance first
        for _ in range(5):
            e.predict(10.0, 0.1, 0.0)
        # Now reset
        e.set_pose(0.0, 0.0, 0.0)
        for i in range(3):
            for j in range(3):
                assert e._P[i][j] == pytest.approx(0.0, abs=1e-12), (
                    f"P[{i}][{j}] = {e._P[i][j]} is not zero after set_pose"
                )

    def test_predict_after_set_pose_advances_from_new_pose(self):
        """Predict after set_pose(100, 0, 0) with dCenter=50 → x≈150."""
        e = EKF()
        e.init(Q_XY, Q_THETA, R_XY)
        e.set_pose(100.0, 0.0, 0.0)
        e.predict(50.0, 0.0, 0.0)
        assert e.x == pytest.approx(150.0, abs=1e-9)
