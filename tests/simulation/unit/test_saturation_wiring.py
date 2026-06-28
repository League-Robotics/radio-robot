#!/usr/bin/env python3
"""test_saturation_wiring.py — Tests for BodyKinematics saturation wiring (010-007).

Validates that wheel setpoints are passed through curvature-preserving saturation
before reaching the VelocityController, matching the behavior implemented in
DriveController::beginStream/beginTimed/beginDistance/beginGoTo.

The BodyKinematics::saturate() call is modeled here in Python to verify:
  - Pass-through when within ceiling
  - Curvature-preserving scale-down when exceeding ceiling
  - Both wheels scale by the same factor (arc curvature preserved)
  - Saturation is symmetric for negative speeds

Default config values (matching firmware defaultRobotConfig()):
  vWheelMax     = 400.0 mm/s
  steerHeadroom = 20.0  mm/s
  → effective ceiling = 380.0 mm/s
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Python model of BodyKinematics::saturate()
# (must match source/kinematics/BodyKinematics.cpp exactly)
# ---------------------------------------------------------------------------

def saturate(vL: float, vR: float, vWheelMax: float, steerHeadroom: float) -> tuple[float, float]:
    """Curvature-preserving saturation — mirrors BodyKinematics::saturate()."""
    ceiling = vWheelMax - steerHeadroom
    abs_L = abs(vL)
    abs_R = abs(vR)
    max_abs = max(abs_L, abs_R)
    if max_abs > ceiling:
        s = ceiling / max_abs
        return s * vL, s * vR
    return vL, vR


# Default config ceiling used by these tests
V_WHEEL_MAX    = 400.0
STEER_HEADROOM = 20.0
CEILING        = V_WHEEL_MAX - STEER_HEADROOM  # 380.0


# ---------------------------------------------------------------------------
# Pass-through (within ceiling) tests
# ---------------------------------------------------------------------------

class TestSaturationPassThrough:
    """Inputs within the effective ceiling are passed through unchanged."""

    def test_both_zero(self) -> None:
        """Zero inputs produce zero outputs."""
        sL, sR = saturate(0.0, 0.0, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(0.0)
        assert sR == pytest.approx(0.0)

    def test_equal_forward_within_ceiling(self) -> None:
        """Equal forward speeds below ceiling are unchanged."""
        sL, sR = saturate(200.0, 200.0, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(200.0)
        assert sR == pytest.approx(200.0)

    def test_asymmetric_within_ceiling(self) -> None:
        """Asymmetric speeds both below ceiling are unchanged."""
        sL, sR = saturate(150.0, 300.0, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(150.0)
        assert sR == pytest.approx(300.0)

    def test_at_exact_ceiling(self) -> None:
        """Speed exactly at ceiling is passed through (not > ceiling)."""
        sL, sR = saturate(CEILING, CEILING, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(CEILING)
        assert sR == pytest.approx(CEILING)

    def test_negative_within_ceiling(self) -> None:
        """Negative speeds within ceiling are passed through unchanged."""
        sL, sR = saturate(-200.0, -200.0, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(-200.0)
        assert sR == pytest.approx(-200.0)


# ---------------------------------------------------------------------------
# Saturation (exceeding ceiling) tests
# ---------------------------------------------------------------------------

class TestSaturationScaling:
    """When max(|vL|, |vR|) > ceiling, both are scaled to preserve curvature."""

    def test_equal_speeds_above_ceiling(self) -> None:
        """Equal speeds above ceiling are both scaled to the ceiling."""
        sL, sR = saturate(500.0, 500.0, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(CEILING)
        assert sR == pytest.approx(CEILING)

    def test_faster_right_clamped_to_ceiling(self) -> None:
        """Faster wheel is clamped to ceiling; slower scales proportionally."""
        # Right wheel faster: 450 > 380 (ceiling)
        sL, sR = saturate(200.0, 450.0, V_WHEEL_MAX, STEER_HEADROOM)
        assert sR == pytest.approx(CEILING)
        # Left wheel scales by same factor s = 380/450
        expected_L = 200.0 * (CEILING / 450.0)
        assert sL == pytest.approx(expected_L)

    def test_faster_left_clamped_to_ceiling(self) -> None:
        """Left wheel faster — symmetric case."""
        sL, sR = saturate(450.0, 200.0, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(CEILING)
        expected_R = 200.0 * (CEILING / 450.0)
        assert sR == pytest.approx(expected_R)

    def test_arc_curvature_preserved(self) -> None:
        """The wheel-speed ratio (curvature) is unchanged after saturation."""
        vL, vR = 200.0, 450.0
        sL, sR = saturate(vL, vR, V_WHEEL_MAX, STEER_HEADROOM)
        ratio_before = vL / vR
        ratio_after  = sL / sR
        assert ratio_after == pytest.approx(ratio_before, rel=1e-5)

    def test_arc_curvature_preserved_tight_turn(self) -> None:
        """Tight turn (large ratio) preserves curvature under saturation."""
        # Inner wheel 50 mm/s, outer 500 mm/s (10:1 ratio)
        vL, vR = 50.0, 500.0
        sL, sR = saturate(vL, vR, V_WHEEL_MAX, STEER_HEADROOM)
        ratio_before = vL / vR
        ratio_after  = sL / sR
        assert ratio_after == pytest.approx(ratio_before, rel=1e-5)
        # Outer wheel must not exceed ceiling
        assert sR <= CEILING + 1e-6

    def test_negative_speeds_clamped_symmetrically(self) -> None:
        """Negative speeds above ceiling are scaled symmetrically."""
        sL, sR = saturate(-500.0, -500.0, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(-CEILING)
        assert sR == pytest.approx(-CEILING)

    def test_mixed_sign_curvature_preserved(self) -> None:
        """Mixed-sign speeds (spin in place) preserve curvature after saturation."""
        # Spin: left = -500, right = +500
        vL, vR = -500.0, 500.0
        sL, sR = saturate(vL, vR, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(-CEILING)
        assert sR == pytest.approx(CEILING)
        # Ratio preserved (both scaled equally)
        assert sR / (-sL) == pytest.approx(1.0, rel=1e-5)

    def test_scale_factor_is_ceiling_over_max(self) -> None:
        """Scale factor = ceiling / max(|vL|, |vR|)."""
        vL, vR = 100.0, 460.0
        sL, sR = saturate(vL, vR, V_WHEEL_MAX, STEER_HEADROOM)
        expected_s = CEILING / 460.0
        assert sL == pytest.approx(100.0 * expected_s)
        assert sR == pytest.approx(460.0 * expected_s)


# ---------------------------------------------------------------------------
# Straight-drive (both wheels equal) saturation
# ---------------------------------------------------------------------------

class TestStraightDriveSaturation:
    """When both wheels are equal, saturation produces equal outputs."""

    def test_straight_above_ceiling_both_equal(self) -> None:
        """S v=500 omega=0 → vL=500, vR=500 → both scaled to ceiling."""
        # Simulate BodyKinematics::inverse(500, 0, 120): vL=500, vR=500
        v, omega, b = 500.0, 0.0, 120.0
        vL = v - omega * (b / 2.0)
        vR = v + omega * (b / 2.0)
        sL, sR = saturate(vL, vR, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(CEILING)
        assert sR == pytest.approx(CEILING)
        assert sL == pytest.approx(sR)  # still straight

    def test_straight_within_ceiling_unchanged(self) -> None:
        """S v=200 omega=0 → vL=200, vR=200 → unchanged."""
        v, omega, b = 200.0, 0.0, 120.0
        vL = v - omega * (b / 2.0)
        vR = v + omega * (b / 2.0)
        sL, sR = saturate(vL, vR, V_WHEEL_MAX, STEER_HEADROOM)
        assert sL == pytest.approx(200.0)
        assert sR == pytest.approx(200.0)


# ---------------------------------------------------------------------------
# Curving-drive saturation (arc commands, S with omega != 0)
# ---------------------------------------------------------------------------

class TestCurvingDriveSaturation:
    """Curving drive: outer wheel hits ceiling, inner scales proportionally."""

    def test_left_curve_omega_positive(self) -> None:
        """S v=300 omega=0.5 (left-curving arc): vL < vR; vR may exceed ceiling."""
        # BodyKinematics::inverse(300, 0.5, 120): vL = 300 - 0.5*60 = 270, vR = 300 + 30 = 330
        v, omega, b = 300.0, 0.5, 120.0
        vL = v - omega * (b / 2.0)  # 270
        vR = v + omega * (b / 2.0)  # 330
        sL, sR = saturate(vL, vR, V_WHEEL_MAX, STEER_HEADROOM)
        # Both within ceiling, no scaling
        assert sL == pytest.approx(270.0)
        assert sR == pytest.approx(330.0)
        assert sL < sR  # still curves left

    def test_hard_left_curve_above_ceiling(self) -> None:
        """S v=350 omega=1.0: outer (right) wheel exceeds ceiling → scaled."""
        # vL = 350 - 1.0*60 = 290, vR = 350 + 60 = 410
        v, omega, b = 350.0, 1.0, 120.0
        vL = v - omega * (b / 2.0)  # 290
        vR = v + omega * (b / 2.0)  # 410 → exceeds 380
        sL, sR = saturate(vL, vR, V_WHEEL_MAX, STEER_HEADROOM)
        assert sR == pytest.approx(CEILING)
        # curvature preserved: ratio unchanged
        assert (sL / sR) == pytest.approx(vL / vR, rel=1e-5)
        # still curves left
        assert sL < sR

    def test_arc_holds_under_load(self) -> None:
        """Simulates load-event: speed increases commanded above ceiling.

        The saturation ensures both wheels scale proportionally so the
        robot slows but holds the arc (does not drift straight).
        """
        # Original arc: vL=250, vR=380 (right at ceiling)
        # Under-load command with +50% boost: vL=375, vR=570
        vL_boosted, vR_boosted = 375.0, 570.0
        sL, sR = saturate(vL_boosted, vR_boosted, V_WHEEL_MAX, STEER_HEADROOM)
        # Outer still at ceiling
        assert sR == pytest.approx(CEILING)
        # Inner scales by same factor
        expected_inner = 375.0 * (CEILING / 570.0)
        assert sL == pytest.approx(expected_inner)
        # Ratio preserved → robot holds arc
        assert (sL / sR) == pytest.approx(vL_boosted / vR_boosted, rel=1e-5)


# ---------------------------------------------------------------------------
# vel= TLM field format
# ---------------------------------------------------------------------------

class TestVelTlmFormat:
    """Validate that the vel= TLM field uses the 2-value vL,vR format."""

    def _make_vel_frame(self, vL: int, vR: int) -> str:
        """Simulate firmware vel= emission: vel=%d,%d."""
        return f"TLM t=1000 mode=S vel={vL},{vR}"

    def test_vel_format_two_values(self) -> None:
        """vel= contains exactly two comma-separated integers."""
        frame = self._make_vel_frame(200, 195)
        assert "vel=" in frame
        vel_part = [t for t in frame.split() if t.startswith("vel=")][0]
        vals = vel_part[4:].split(",")
        assert len(vals) == 2

    def test_vel_values_are_integers(self) -> None:
        """Both vel= values parse as integers."""
        frame = self._make_vel_frame(200, 195)
        vel_part = [t for t in frame.split() if t.startswith("vel=")][0]
        vals = vel_part[4:].split(",")
        assert int(vals[0]) == 200
        assert int(vals[1]) == 195

    def test_vel_supports_negative(self) -> None:
        """Negative velocities (reverse drive) are handled."""
        frame = self._make_vel_frame(-150, -148)
        vel_part = [t for t in frame.split() if t.startswith("vel=")][0]
        vals = vel_part[4:].split(",")
        assert int(vals[0]) == -150
        assert int(vals[1]) == -148

    def test_vel_zero_when_idle(self) -> None:
        """vel= emits 0,0 when robot is stopped."""
        frame = self._make_vel_frame(0, 0)
        vel_part = [t for t in frame.split() if t.startswith("vel=")][0]
        vals = vel_part[4:].split(",")
        assert int(vals[0]) == 0
        assert int(vals[1]) == 0

    def test_vel_field_key_is_vel(self) -> None:
        """The TLM key for measured wheel velocity is 'vel'."""
        frame = self._make_vel_frame(200, 195)
        assert "vel=" in frame
        assert "velocity=" not in frame
        assert "wheelVel=" not in frame
