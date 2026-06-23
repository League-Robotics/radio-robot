#!/usr/bin/env python3
"""test_046_006_otos_lateral_vy.py — Unit tests for 046-006 OTOS lateral velocity (vy).

Tests the two new software pieces introduced in ticket 046-006:

1. readVelocityTransformed3 sign/scale correctness — a pure Python mirror
   of the body-frame rotation applied in OtosSensor::readVelocityTransformed3.
   Verifies that vx, vy, and omega are computed with the correct sign and
   scale from the raw OTOS registers, for both the aligned (odomYawDeg=0)
   and rotated mounting cases.

2. complementary filter (fusedVy) — a pure Python mirror of the filter in
   Odometry::correctEKF.  Verifies that:
   - The filter blends toward the OTOS reading with the configured alpha.
   - Alpha=1.0 makes fusedVy track vy_otos exactly (first-order lag τ=1 tick).
   - Alpha=0.0 leaves fusedVy unchanged (no update from OTOS).
   - After N ticks at constant vy_otos, fusedVy converges to vy_otos.
   - fusedVy stays 0 when vy_otos=0 (rest test — no drift).

Sprint 046, Ticket 006.
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Pure-Python mirror of OtosSensor::readVelocityTransformed3
#
# The OTOS chip returns raw velocity registers in its chip frame (rvx, rvy, rvh).
# readVelocityTransformed3 applies:
#   1. LSB → mm/s conversion.
#   2. odomUpsideDown flip (negate vx, vy, omega if True).
#   3. Rotation by -odomYawDeg to get body frame.
#
# omega passes through unchanged by the rotation (mounting-offset derivative
# is zero — the yaw offset is constant, so d/dt(yaw) = d/dt(chip_yaw)).
# ---------------------------------------------------------------------------

K_VEL_MMPS_PER_LSB = 0.305          # mm/s per LSB
K_OMEGA_RADPS_PER_LSB = 0.00549 * (math.pi / 180.0)  # rad/s per LSB


def read_velocity_transformed3(
    rvx: int, rvy: int, rvh: int,
    odom_yaw_deg: float = 0.0,
    odom_upside_down: bool = False,
) -> tuple[float, float, float]:
    """Python mirror of OtosSensor::readVelocityTransformed3.

    Args:
        rvx, rvy, rvh: signed 16-bit raw velocity register values.
        odom_yaw_deg: sensor mounting yaw offset (degrees).
        odom_upside_down: True if sensor is mounted upside-down.

    Returns:
        (vx_mmps, vy_mmps, omega_rads) — body-frame velocity.
    """
    vxF = rvx * K_VEL_MMPS_PER_LSB
    vyF = rvy * K_VEL_MMPS_PER_LSB
    whF = rvh * K_OMEGA_RADPS_PER_LSB

    if odom_upside_down:
        vxF = -vxF
        vyF = -vyF
        whF = -whF

    ang_rad = -odom_yaw_deg * (math.pi / 180.0)
    c = math.cos(ang_rad)
    s = math.sin(ang_rad)

    # Rotate chip-native (vxF, vyF) into robot body frame.
    vx_body = c * vxF - s * vyF
    vy_body = s * vxF + c * vyF

    return vx_body, vy_body, whF


# ---------------------------------------------------------------------------
# Tests — readVelocityTransformed3 (sign / scale)
# ---------------------------------------------------------------------------

class TestReadVelocityTransformed3Scale:
    """Verify LSB-to-SI scaling for vx, vy, omega at yaw=0 (aligned mounting)."""

    def test_vx_scale_forward(self):
        """Pure forward motion (rvx=1000, rvy=0, rvh=0) → vx=305 mm/s."""
        vx, vy, omega = read_velocity_transformed3(1000, 0, 0)
        assert vx == pytest.approx(1000 * K_VEL_MMPS_PER_LSB, rel=1e-6)
        assert vy == pytest.approx(0.0, abs=1e-9)
        assert omega == pytest.approx(0.0, abs=1e-9)

    def test_vy_scale_lateral(self):
        """Pure lateral motion (rvx=0, rvy=500, rvh=0) → vy=152.5 mm/s."""
        vx, vy, omega = read_velocity_transformed3(0, 500, 0)
        assert vx == pytest.approx(0.0, abs=1e-9)
        assert vy == pytest.approx(500 * K_VEL_MMPS_PER_LSB, rel=1e-6)
        assert omega == pytest.approx(0.0, abs=1e-9)

    def test_omega_scale(self):
        """Pure rotation (rvx=0, rvy=0, rvh=1000) → omega in rad/s."""
        vx, vy, omega = read_velocity_transformed3(0, 0, 1000)
        assert vx == pytest.approx(0.0, abs=1e-9)
        assert vy == pytest.approx(0.0, abs=1e-9)
        assert omega == pytest.approx(1000 * K_OMEGA_RADPS_PER_LSB, rel=1e-6)

    def test_reverse_forward_negative_vx(self):
        """Reverse motion → negative vx."""
        vx, vy, omega = read_velocity_transformed3(-1000, 0, 0)
        assert vx == pytest.approx(-1000 * K_VEL_MMPS_PER_LSB, rel=1e-6)

    def test_negative_lateral_vy(self):
        """Negative lateral register → negative vy_body."""
        _, vy, _ = read_velocity_transformed3(0, -500, 0)
        assert vy == pytest.approx(-500 * K_VEL_MMPS_PER_LSB, rel=1e-6)

    def test_combined_vx_vy_omega_all_positive(self):
        """All three channels non-zero at yaw=0 — no cross-talk."""
        vx, vy, omega = read_velocity_transformed3(100, 200, 300)
        assert vx == pytest.approx(100 * K_VEL_MMPS_PER_LSB, rel=1e-6)
        assert vy == pytest.approx(200 * K_VEL_MMPS_PER_LSB, rel=1e-6)
        assert omega == pytest.approx(300 * K_OMEGA_RADPS_PER_LSB, rel=1e-6)


class TestReadVelocityTransformed3UpsideDown:
    """Upside-down flip negates vx, vy, and omega."""

    def test_upside_down_negates_vx(self):
        """Upside-down: positive rvx → negative vx_body."""
        vx, vy, omega = read_velocity_transformed3(1000, 0, 0, odom_upside_down=True)
        assert vx == pytest.approx(-1000 * K_VEL_MMPS_PER_LSB, rel=1e-6)

    def test_upside_down_negates_vy(self):
        """Upside-down: positive rvy → negative vy_body."""
        _, vy, _ = read_velocity_transformed3(0, 500, 0, odom_upside_down=True)
        assert vy == pytest.approx(-500 * K_VEL_MMPS_PER_LSB, rel=1e-6)

    def test_upside_down_negates_omega(self):
        """Upside-down: positive rvh → negative omega."""
        _, _, omega = read_velocity_transformed3(0, 0, 1000, odom_upside_down=True)
        assert omega == pytest.approx(-1000 * K_OMEGA_RADPS_PER_LSB, rel=1e-6)

    def test_upside_down_false_preserves_sign(self):
        """Normal mounting: positive rvx → positive vx_body."""
        vx, _, _ = read_velocity_transformed3(500, 0, 0, odom_upside_down=False)
        assert vx > 0.0


class TestReadVelocityTransformed3Rotation:
    """Mounting rotation (odomYawDeg) mixes vx and vy; omega unchanged."""

    def test_90_deg_yaw_vx_to_vy(self):
        """90° yaw: pure chip +X forward (rvx=1000, rvy=0) → body vy."""
        # ang_rad = -90°; c=0, s=-1
        # vx_body = 0*vxF - (-1)*vyF = vyF = 0
        # vy_body = -1*vxF + 0*vyF = -vxF
        vx, vy, omega = read_velocity_transformed3(1000, 0, 0, odom_yaw_deg=90.0)
        expected_vx = 0.0
        expected_vy = -1000 * K_VEL_MMPS_PER_LSB  # -305.0 mm/s
        assert vx == pytest.approx(expected_vx, abs=1e-5)
        assert vy == pytest.approx(expected_vy, rel=1e-5)

    def test_90_deg_yaw_omega_unchanged(self):
        """90° yaw does not change omega."""
        _, _, omega = read_velocity_transformed3(0, 0, 500, odom_yaw_deg=90.0)
        assert omega == pytest.approx(500 * K_OMEGA_RADPS_PER_LSB, rel=1e-6)

    def test_180_deg_yaw_negates_both_linear(self):
        """180° yaw: cos(-180°)=-1, sin(-180°)=0 → vx and vy negated."""
        vx, vy, _ = read_velocity_transformed3(1000, 500, 0, odom_yaw_deg=180.0)
        # vx_body = -1*vxF - 0*vyF = -vxF
        # vy_body = 0*vxF + (-1)*vyF = -vyF
        assert vx == pytest.approx(-1000 * K_VEL_MMPS_PER_LSB, rel=1e-5)
        assert vy == pytest.approx(-500 * K_VEL_MMPS_PER_LSB, rel=1e-5)

    def test_45_deg_yaw_mixes_vx_vy_equally(self):
        """45° yaw: pure chip X maps to both body vx and vy components."""
        c45 = math.cos(math.radians(-45.0))
        s45 = math.sin(math.radians(-45.0))
        lsb = 1000
        vx, vy, _ = read_velocity_transformed3(lsb, 0, 0, odom_yaw_deg=45.0)
        assert vx == pytest.approx(c45 * lsb * K_VEL_MMPS_PER_LSB, rel=1e-5)
        assert vy == pytest.approx(s45 * lsb * K_VEL_MMPS_PER_LSB, rel=1e-5)

    def test_zero_yaw_is_identity(self):
        """0° yaw: transform is identity (vx and vy pass through unchanged)."""
        vx, vy, omega = read_velocity_transformed3(300, 150, 200, odom_yaw_deg=0.0)
        assert vx == pytest.approx(300 * K_VEL_MMPS_PER_LSB, rel=1e-6)
        assert vy == pytest.approx(150 * K_VEL_MMPS_PER_LSB, rel=1e-6)
        assert omega == pytest.approx(200 * K_OMEGA_RADPS_PER_LSB, rel=1e-6)


# ---------------------------------------------------------------------------
# Pure-Python mirror of the complementary filter for fusedVy
# (Odometry::correctEKF — mecanum build only, 046-006)
# ---------------------------------------------------------------------------

class ComplementaryFilterVy:
    """Python mirror of the fusedVy complementary filter.

    fusedVy = alpha * vy_otos + (1 - alpha) * fusedVy

    Initialized to 0.0 (same as _fusedVy in Odometry).
    """

    def __init__(self, alpha: float = 0.8):
        self.alpha = alpha
        self.fused_vy = 0.0

    def update(self, vy_otos: float) -> float:
        """Apply one complementary filter step and return the new fused_vy."""
        self.fused_vy = self.alpha * vy_otos + (1.0 - self.alpha) * self.fused_vy
        return self.fused_vy


class TestComplementaryFilterVy:
    """Verify the complementary filter math for fusedVy."""

    def test_alpha_1_tracks_otos_immediately(self):
        """Alpha=1.0: fusedVy equals vy_otos after one step."""
        f = ComplementaryFilterVy(alpha=1.0)
        result = f.update(150.0)
        assert result == pytest.approx(150.0, abs=1e-9)

    def test_alpha_0_leaves_fused_vy_unchanged(self):
        """Alpha=0.0: OTOS reading has no effect on fusedVy."""
        f = ComplementaryFilterVy(alpha=0.0)
        result = f.update(500.0)
        # fusedVy starts at 0, alpha=0 → remains 0
        assert result == pytest.approx(0.0, abs=1e-9)

    def test_alpha_08_single_step(self):
        """Alpha=0.8: after one step from 0, fusedVy = 0.8 * vy_otos."""
        f = ComplementaryFilterVy(alpha=0.8)
        result = f.update(100.0)
        assert result == pytest.approx(80.0, abs=1e-9)

    def test_alpha_08_second_step(self):
        """Alpha=0.8: second step blends the previous filtered value."""
        f = ComplementaryFilterVy(alpha=0.8)
        f.update(100.0)   # after: fused = 80.0
        result = f.update(100.0)
        # fused = 0.8*100 + 0.2*80 = 80 + 16 = 96.0
        assert result == pytest.approx(96.0, abs=1e-9)

    def test_convergence_to_constant_vy_otos(self):
        """After many steps at constant vy_otos, fusedVy converges to vy_otos."""
        f = ComplementaryFilterVy(alpha=0.8)
        vy_target = 200.0
        for _ in range(50):
            f.update(vy_target)
        # After 50 steps: error = 200 * (1-0.8)^50 = 200 * 0.2^50 ≈ 0 (negligible)
        assert f.fused_vy == pytest.approx(vy_target, abs=1e-6)

    def test_zero_vy_otos_stays_zero(self):
        """When OTOS reports vy=0, fusedVy stays at 0 (no drift)."""
        f = ComplementaryFilterVy(alpha=0.8)
        for _ in range(20):
            f.update(0.0)
        assert f.fused_vy == pytest.approx(0.0, abs=1e-12)

    def test_step_change_decays_to_new_value(self):
        """After a step change in vy_otos, fusedVy decays exponentially to new value."""
        f = ComplementaryFilterVy(alpha=0.8)
        # Steady state at 100 mm/s
        for _ in range(50):
            f.update(100.0)
        assert f.fused_vy == pytest.approx(100.0, abs=1e-5)

        # Step change to 0
        for _ in range(50):
            f.update(0.0)
        assert f.fused_vy == pytest.approx(0.0, abs=1e-5)

    def test_negative_vy_otos(self):
        """Negative vy_otos (strafe left) is handled correctly."""
        f = ComplementaryFilterVy(alpha=1.0)
        result = f.update(-150.0)
        assert result == pytest.approx(-150.0, abs=1e-9)

    def test_filter_state_decays_geometrically(self):
        """With vy_otos=0 after a step, fusedVy decays as (1-alpha)^n * initial."""
        alpha = 0.8
        f = ComplementaryFilterVy(alpha=alpha)
        # Set initial state by one step with alpha=1
        f.alpha = 1.0
        f.update(100.0)
        assert f.fused_vy == pytest.approx(100.0, abs=1e-9)

        # Switch back to alpha=0.8, feed vy=0: should decay geometrically
        f.alpha = alpha
        for i in range(10):
            expected = 100.0 * ((1.0 - alpha) ** (i + 1))
            result = f.update(0.0)
            assert result == pytest.approx(expected, rel=1e-6), \
                f"Step {i+1}: expected {expected:.6f} got {result:.6f}"

    def test_non_diverging_over_many_steps(self):
        """fusedVy does not grow or diverge over 1000 steps at constant vy_otos."""
        f = ComplementaryFilterVy(alpha=0.8)
        vy_target = 400.0  # mm/s (typical max)
        for _ in range(1000):
            f.update(vy_target)
        # Must converge to vy_target, not diverge
        assert f.fused_vy == pytest.approx(vy_target, abs=1e-3)

    def test_alternating_vy_tracks_both_signs(self):
        """Filter correctly tracks alternating sign vy_otos readings."""
        f = ComplementaryFilterVy(alpha=1.0)  # alpha=1 = exact tracking
        f.update(100.0)
        assert f.fused_vy == pytest.approx(100.0, abs=1e-9)
        f.update(-100.0)
        assert f.fused_vy == pytest.approx(-100.0, abs=1e-9)
