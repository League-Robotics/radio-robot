#!/usr/bin/env python3
"""test_body_kinematics.py — Unit tests for BodyKinematics math (010-002).

Pure Python implementation of the same equations defined in:
  source/control/BodyKinematics.h / .cpp

Tests verify:
  - inverse(v, omega, b) -> (vL, vR): vL = v - omega*(b/2), vR = v + omega*(b/2)
  - forward(vL, vR, b) -> (v, omega): v = (vR+vL)/2, omega = (vR-vL)/b
  - inverse then forward round-trip returns original (v, omega) within float epsilon
  - saturate with vL=300, vR=500, vWheelMax=400, headroom=20 scales both by 380/500 = 0.76
  - curvature kappa = (vR-vL)/(b*(vR+vL)/2) is preserved after saturation scaling

Unit conventions (match docs/kinematics-model.md §1.3 and §1.7):
  v     : mm/s (body forward speed)
  omega : rad/s CCW-positive (body yaw rate)
  vL,vR : mm/s (wheel speeds, signed)
  b     : mm (track width)
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Pure Python mirrors of BodyKinematics C++ functions
# ---------------------------------------------------------------------------

def bk_inverse(v: float, omega: float, b: float) -> tuple[float, float]:
    """vL = v - omega*(b/2), vR = v + omega*(b/2)."""
    half_b = b / 2.0
    vL = v - omega * half_b
    vR = v + omega * half_b
    return vL, vR


def bk_forward(vL: float, vR: float, b: float) -> tuple[float, float]:
    """v = (vR+vL)/2, omega = (vR-vL)/b."""
    v = (vR + vL) / 2.0
    omega = (vR - vL) / b
    return v, omega


def bk_saturate(vL: float, vR: float,
                vWheelMax: float, steerHeadroom: float) -> tuple[float, float]:
    """Scale both wheel speeds when max(|vL|, |vR|) > (vWheelMax - steerHeadroom)."""
    ceiling = vWheelMax - steerHeadroom
    max_abs = max(abs(vL), abs(vR))
    if max_abs > ceiling:
        s = ceiling / max_abs
        return s * vL, s * vR
    return vL, vR


# ---------------------------------------------------------------------------
# Tests — inverse map
# ---------------------------------------------------------------------------

class TestInverse:
    """Verify vL = v - omega*(b/2), vR = v + omega*(b/2)."""

    def test_straight_line(self):
        """Zero yaw rate: both wheels same speed as body."""
        vL, vR = bk_inverse(v=200.0, omega=0.0, b=120.0)
        assert vL == pytest.approx(200.0)
        assert vR == pytest.approx(200.0)

    def test_spin_in_place(self):
        """Zero forward speed: wheels equal and opposite."""
        vL, vR = bk_inverse(v=0.0, omega=1.0, b=120.0)
        assert vL == pytest.approx(-60.0)   # 0 - 1.0*(120/2)
        assert vR == pytest.approx(+60.0)   # 0 + 1.0*(120/2)

    def test_right_turn_arc(self):
        """Positive omega (CCW) makes right wheel faster."""
        b = 120.0
        v = 100.0
        omega = 0.5  # rad/s
        vL, vR = bk_inverse(v, omega, b)
        assert vL == pytest.approx(v - omega * b / 2)
        assert vR == pytest.approx(v + omega * b / 2)
        assert vR > vL  # right wheel faster for CCW turn

    def test_left_turn_arc(self):
        """Negative omega (CW) makes left wheel faster."""
        b = 120.0
        v = 100.0
        omega = -0.5  # rad/s (CW)
        vL, vR = bk_inverse(v, omega, b)
        assert vL > vR  # left wheel faster for CW turn

    def test_reverse_straight(self):
        """Negative body speed maps to both wheels negative."""
        vL, vR = bk_inverse(v=-150.0, omega=0.0, b=120.0)
        assert vL == pytest.approx(-150.0)
        assert vR == pytest.approx(-150.0)

    def test_known_values(self):
        """Spot-check with exact arithmetic: v=100, omega=1, b=120."""
        # vL = 100 - 1*(60) = 40
        # vR = 100 + 1*(60) = 160
        vL, vR = bk_inverse(100.0, 1.0, 120.0)
        assert vL == pytest.approx(40.0)
        assert vR == pytest.approx(160.0)


# ---------------------------------------------------------------------------
# Tests — forward map
# ---------------------------------------------------------------------------

class TestForward:
    """Verify v = (vR+vL)/2, omega = (vR-vL)/b."""

    def test_straight_line(self):
        """Equal wheel speeds: zero omega, v = wheel speed."""
        v, omega = bk_forward(vL=200.0, vR=200.0, b=120.0)
        assert v == pytest.approx(200.0)
        assert omega == pytest.approx(0.0)

    def test_spin_in_place(self):
        """Opposite equal wheel speeds: v=0, omega = 2*speed/b."""
        v, omega = bk_forward(vL=-60.0, vR=60.0, b=120.0)
        assert v == pytest.approx(0.0)
        assert omega == pytest.approx(1.0)  # (60-(-60))/120 = 120/120 = 1 rad/s

    def test_known_values(self):
        """Spot-check: vL=40, vR=160, b=120."""
        # v = (160+40)/2 = 100
        # omega = (160-40)/120 = 120/120 = 1.0 rad/s
        v, omega = bk_forward(40.0, 160.0, 120.0)
        assert v == pytest.approx(100.0)
        assert omega == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# Tests — round-trip (inverse then forward)
# ---------------------------------------------------------------------------

class TestRoundTrip:
    """inverse() followed by forward() must recover original (v, omega)."""

    TRACK_WIDTH = 120.0  # mm (robot default)

    @pytest.mark.parametrize("v,omega", [
        (100.0, 0.0),       # straight
        (0.0, 1.0),         # spin in place
        (200.0, 0.5),       # forward arc CCW
        (200.0, -0.5),      # forward arc CW
        (-150.0, 0.3),      # reverse arc
        (50.0, 2.0),        # tight turn
        (300.0, 0.0),       # fast straight
        (0.0, -1.5),        # reverse spin
    ])
    def test_round_trip(self, v, omega):
        """forward(inverse(v, omega, b), b) == (v, omega)."""
        b = self.TRACK_WIDTH
        vL, vR = bk_inverse(v, omega, b)
        v2, omega2 = bk_forward(vL, vR, b)
        assert v2 == pytest.approx(v, abs=1e-5), f"v mismatch for ({v}, {omega})"
        assert omega2 == pytest.approx(omega, abs=1e-5), f"omega mismatch for ({v}, {omega})"


# ---------------------------------------------------------------------------
# Tests — saturation
# ---------------------------------------------------------------------------

class TestSaturate:
    """Verify curvature-preserving saturation (docs/kinematics-model.md §1.7)."""

    def test_no_saturation_below_ceiling(self):
        """Wheel speeds below ceiling pass through unchanged."""
        vL, vR = bk_saturate(200.0, 300.0, vWheelMax=400.0, steerHeadroom=20.0)
        # max(200, 300) = 300 <= 380 ceiling — no scaling
        assert vL == pytest.approx(200.0)
        assert vR == pytest.approx(300.0)

    def test_no_saturation_at_exact_ceiling(self):
        """Wheel speed exactly at ceiling passes through unchanged."""
        vL, vR = bk_saturate(300.0, 380.0, vWheelMax=400.0, steerHeadroom=20.0)
        # max = 380.0, ceiling = 380.0 — equal, no scaling
        assert vL == pytest.approx(300.0)
        assert vR == pytest.approx(380.0)

    def test_ticket_example(self):
        """AC-specified example: vL=300, vR=500, vWheelMax=400, headroom=20 -> scale 0.76."""
        # ceiling = 400 - 20 = 380
        # max(|300|, |500|) = 500 > 380 → s = 380/500 = 0.76
        vL, vR = bk_saturate(300.0, 500.0, vWheelMax=400.0, steerHeadroom=20.0)
        assert vL == pytest.approx(0.76 * 300.0)
        assert vR == pytest.approx(0.76 * 500.0)

    def test_scale_factor_correct(self):
        """Faster wheel sits exactly at ceiling after scaling."""
        vWheelMax, headroom = 400.0, 20.0
        ceiling = vWheelMax - headroom  # 380
        vL, vR = bk_saturate(300.0, 500.0, vWheelMax, headroom)
        assert max(abs(vL), abs(vR)) == pytest.approx(ceiling, rel=1e-6)

    def test_curvature_preserved_after_saturation(self):
        """Wheel speed ratio (and thus arc curvature) is unchanged by saturation.

        Curvature κ = (vR - vL) / (b * (vR + vL) / 2).  Because both wheels
        scale by the same factor s, κ is invariant: s cancels top and bottom.
        We verify the ratio vR/vL is unchanged instead (equivalent for same-sign).
        """
        vL_in, vR_in = 300.0, 500.0
        vL_out, vR_out = bk_saturate(vL_in, vR_in, vWheelMax=400.0, steerHeadroom=20.0)
        ratio_before = vR_in / vL_in
        ratio_after = vR_out / vL_out
        assert ratio_after == pytest.approx(ratio_before, rel=1e-6)

    def test_curvature_formula_preserved(self):
        """κ = (vR-vL)/(b*(vR+vL)/2) is preserved after saturation."""
        b = 120.0  # track width mm
        vL_in, vR_in = 300.0, 500.0
        vL_out, vR_out = bk_saturate(vL_in, vR_in, vWheelMax=400.0, steerHeadroom=20.0)

        # Avoid divide-by-zero (sum is non-zero here)
        kappa_before = (vR_in - vL_in) / (b * (vR_in + vL_in) / 2.0)
        kappa_after  = (vR_out - vL_out) / (b * (vR_out + vL_out) / 2.0)
        assert kappa_after == pytest.approx(kappa_before, rel=1e-6)

    def test_negative_speeds_saturated(self):
        """Saturation works correctly for negative wheel speeds (reverse arc)."""
        vL, vR = bk_saturate(-300.0, -500.0, vWheelMax=400.0, steerHeadroom=20.0)
        # max(300, 500) = 500 > 380 → s = 380/500 = 0.76
        assert vL == pytest.approx(-0.76 * 300.0)
        assert vR == pytest.approx(-0.76 * 500.0)

    def test_opposite_sign_speeds_saturated(self):
        """Saturation handles pivot turns (one wheel positive, one negative)."""
        vL_in, vR_in = -300.0, 500.0
        vL_out, vR_out = bk_saturate(vL_in, vR_in, vWheelMax=400.0, steerHeadroom=20.0)
        # max(300, 500) = 500 > 380 → s = 380/500 = 0.76
        assert vL_out == pytest.approx(-0.76 * 300.0)
        assert vR_out == pytest.approx(0.76 * 500.0)

    def test_steer_headroom_respected(self):
        """Ceiling is vWheelMax - steerHeadroom, not vWheelMax."""
        # With steerHeadroom=20, ceiling=380. Verify max(|vL|, |vR|) after
        # saturation equals 380, not 400.
        vL, vR = bk_saturate(200.0, 450.0, vWheelMax=400.0, steerHeadroom=20.0)
        assert max(abs(vL), abs(vR)) == pytest.approx(380.0, rel=1e-6)

    def test_no_headroom(self):
        """Zero steerHeadroom means ceiling equals vWheelMax."""
        vL, vR = bk_saturate(200.0, 500.0, vWheelMax=400.0, steerHeadroom=0.0)
        # max = 500 > 400 → s = 400/500 = 0.8
        assert vL == pytest.approx(0.8 * 200.0)
        assert vR == pytest.approx(0.8 * 500.0)
        assert max(abs(vL), abs(vR)) == pytest.approx(400.0, rel=1e-6)

    def test_straight_saturation_preserves_equal_ratio(self):
        """Straight-line saturation: both wheels same value, ratio=1 preserved."""
        vL, vR = bk_saturate(500.0, 500.0, vWheelMax=400.0, steerHeadroom=20.0)
        # s = 380/500 = 0.76
        assert vL == pytest.approx(vR)
        assert max(abs(vL), abs(vR)) == pytest.approx(380.0, rel=1e-6)
