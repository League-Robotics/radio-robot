#!/usr/bin/env python3
"""test_mecanum_kinematics.py — Unit tests for MecanumKinematics and
BodyKinematics array overloads (046-002).

Pure Python implementation of the equations defined in:
  source/control/MecanumKinematics.h / .cpp
  source/control/BodyKinematics.h / .cpp  (array overloads)

Wheel index order (canonical):
  [0] = FR (Front-Right)
  [1] = FL (Front-Left)
  [2] = BR (Back-Right)
  [3] = BL (Back-Left)

Combined geometry constant: k = halfTrackMm + halfWheelbaseMm

Inverse kinematics (raw, before sign correction):
  FR_raw =  vx - vy - k * omega
  FL_raw =  vx + vy + k * omega
  BR_raw =  vx + vy - k * omega
  BL_raw =  vx - vy + k * omega
  wheels[i] = raw[i] * signs[i]

Forward kinematics (after dividing out signs):
  w[i] = wheels[i] * signs[i]
  vx    = ( w[0] + w[1] + w[2] + w[3]) / 4
  vy    = (-w[0] + w[1] + w[2] - w[3]) / 4
  omega = (-w[0] + w[1] - w[2] + w[3]) / (4 * k)
"""

from __future__ import annotations

import math
import random
import pytest


# ---------------------------------------------------------------------------
# Pure Python mirrors of MecanumKinematics C++ functions
# ---------------------------------------------------------------------------

DEFAULT_SIGNS = [1, 1, 1, 1]   # identity signs (all +1)
NEZHA_SIGNS   = [-1, 1, -1, 1] # from architecture: FR/BR=-1, FL/BL=+1


def mk_inverse(vx: float, vy: float, omega: float,
               half_track: float, half_wheelbase: float,
               signs: list[int] | None = None) -> list[float]:
    """Body twist -> 4 wheel speeds.

    wheels = [FR, FL, BR, BL]
    """
    if signs is None:
        signs = DEFAULT_SIGNS
    k = half_track + half_wheelbase
    raw_fr =  vx - vy - k * omega
    raw_fl =  vx + vy + k * omega
    raw_br =  vx + vy - k * omega
    raw_bl =  vx - vy + k * omega
    raw = [raw_fr, raw_fl, raw_br, raw_bl]
    return [raw[i] * signs[i] for i in range(4)]


def mk_forward(wheels: list[float],
               half_track: float, half_wheelbase: float,
               signs: list[int] | None = None) -> tuple[float, float, float]:
    """4 wheel speeds -> (vx, vy, omega)."""
    if signs is None:
        signs = DEFAULT_SIGNS
    k = half_track + half_wheelbase
    w = [wheels[i] * signs[i] for i in range(4)]
    vx    = ( w[0] + w[1] + w[2] + w[3]) / 4.0
    vy    = (-w[0] + w[1] + w[2] - w[3]) / 4.0
    omega = (-w[0] + w[1] - w[2] + w[3]) / (4.0 * k)
    return vx, vy, omega


def mk_saturate(wheels: list[float], v_wheel_max: float) -> list[float]:
    """Uniform scale when max(|wheel|) > v_wheel_max."""
    max_abs = max(abs(w) for w in wheels)
    if max_abs > v_wheel_max:
        s = v_wheel_max / max_abs
        return [s * w for w in wheels]
    return list(wheels)


# ---------------------------------------------------------------------------
# Pure Python mirrors of BodyKinematics array overloads
# ---------------------------------------------------------------------------

def bk_inverse_arr(vx: float, vy: float, omega: float,
                   b: float) -> list[float]:
    """wheels[2] = [vL, vR]; vy is ignored (differential)."""
    half_b = b / 2.0
    vL = vx - omega * half_b
    vR = vx + omega * half_b
    return [vL, vR]


def bk_forward_arr(wheels: list[float], b: float) -> tuple[float, float, float]:
    """(vx, vy=0, omega) from wheels[2] = [vL, vR]."""
    vL, vR = wheels[0], wheels[1]
    vx    = (vR + vL) / 2.0
    omega = (vR - vL) / b
    return vx, 0.0, omega


def bk_saturate_arr(wheels: list[float],
                    v_wheel_max: float, steer_headroom: float) -> list[float]:
    """Uniform scale for 2-wheel array form."""
    ceiling = v_wheel_max - steer_headroom
    max_abs = max(abs(w) for w in wheels)
    if max_abs > ceiling:
        s = ceiling / max_abs
        return [s * w for w in wheels]
    return list(wheels)


# ---------------------------------------------------------------------------
# Shared geometry fixture
# ---------------------------------------------------------------------------

HALF_TRACK    = 80.0    # mm
HALF_WHEELBASE = 70.0   # mm
K = HALF_TRACK + HALF_WHEELBASE  # 150.0 mm


# ===========================================================================
# MecanumKinematics tests
# ===========================================================================

class TestMecanumInverse:
    """Verify the 4-wheel inverse equations."""

    def test_pure_forward_all_wheels_equal(self):
        """vx=200, vy=0, omega=0 -> all four wheels = 200 (with identity signs)."""
        wheels = mk_inverse(200.0, 0.0, 0.0, HALF_TRACK, HALF_WHEELBASE)
        assert wheels[0] == pytest.approx(200.0)  # FR
        assert wheels[1] == pytest.approx(200.0)  # FL
        assert wheels[2] == pytest.approx(200.0)  # BR
        assert wheels[3] == pytest.approx(200.0)  # BL

    def test_pure_strafe_wheel_pattern(self):
        """vx=0, vy=200, omega=0 -> FR/BL=-200, FL/BR=+200 (identity signs).

        From the equations:
          FR_raw = 0 - 200 - 0 = -200
          FL_raw = 0 + 200 + 0 = +200
          BR_raw = 0 + 200 - 0 = +200
          BL_raw = 0 - 200 + 0 = -200
        """
        wheels = mk_inverse(0.0, 200.0, 0.0, HALF_TRACK, HALF_WHEELBASE)
        assert wheels[0] == pytest.approx(-200.0)  # FR
        assert wheels[1] == pytest.approx(+200.0)  # FL
        assert wheels[2] == pytest.approx(+200.0)  # BR
        assert wheels[3] == pytest.approx(-200.0)  # BL

    def test_pure_rotate_signs(self):
        """vx=0, vy=0, omega=1 -> FR/BR negative, FL/BL positive (identity signs).

        From the equations (k=150):
          FR_raw = 0 - 0 - 150*1 = -150
          FL_raw = 0 + 0 + 150*1 = +150
          BR_raw = 0 + 0 - 150*1 = -150
          BL_raw = 0 - 0 + 150*1 = +150
        """
        omega = 1.0
        wheels = mk_inverse(0.0, 0.0, omega, HALF_TRACK, HALF_WHEELBASE)
        assert wheels[0] == pytest.approx(-K * omega)  # FR
        assert wheels[1] == pytest.approx(+K * omega)  # FL
        assert wheels[2] == pytest.approx(-K * omega)  # BR
        assert wheels[3] == pytest.approx(+K * omega)  # BL

    def test_known_values_k_geometry(self):
        """Spot-check with k=1 (half_track=0.5, half_wheelbase=0.5)."""
        vx, vy, omega = 10.0, 5.0, 2.0
        wheels = mk_inverse(vx, vy, omega, 0.5, 0.5)
        # k=1; FR = 10-5-2=3, FL=10+5+2=17, BR=10+5-2=13, BL=10-5+2=7
        assert wheels[0] == pytest.approx(3.0)   # FR
        assert wheels[1] == pytest.approx(17.0)  # FL
        assert wheels[2] == pytest.approx(13.0)  # BR
        assert wheels[3] == pytest.approx(7.0)   # BL

    def test_signs_applied(self):
        """Negative signs flip each wheel output independently."""
        signs = [-1, 1, -1, 1]
        wheels = mk_inverse(100.0, 0.0, 0.0, HALF_TRACK, HALF_WHEELBASE,
                            signs=signs)
        # raw all +100; with signs: FR=-100, FL=+100, BR=-100, BL=+100
        assert wheels[0] == pytest.approx(-100.0)
        assert wheels[1] == pytest.approx(+100.0)
        assert wheels[2] == pytest.approx(-100.0)
        assert wheels[3] == pytest.approx(+100.0)


class TestMecanumForward:
    """Verify the 4-wheel forward equations."""

    def test_pure_forward_recovers(self):
        """Four equal wheel speeds of 200 -> vx=200, vy=0, omega=0."""
        wheels = [200.0, 200.0, 200.0, 200.0]
        vx, vy, omega = mk_forward(wheels, HALF_TRACK, HALF_WHEELBASE)
        assert vx    == pytest.approx(200.0, abs=1e-6)
        assert vy    == pytest.approx(0.0,   abs=1e-6)
        assert omega == pytest.approx(0.0,   abs=1e-6)

    def test_pure_strafe_recovers(self):
        """FR/BL=-200, FL/BR=+200 -> vy=200, vx=0, omega=0."""
        wheels = [-200.0, +200.0, +200.0, -200.0]
        vx, vy, omega = mk_forward(wheels, HALF_TRACK, HALF_WHEELBASE)
        assert vx    == pytest.approx(0.0,   abs=1e-6)
        assert vy    == pytest.approx(200.0, abs=1e-6)
        assert omega == pytest.approx(0.0,   abs=1e-6)

    def test_pure_rotate_recovers(self):
        """FR/BR=-150, FL/BL=+150 (k=150, omega=1) -> omega=1, vx=vy=0."""
        omega_in = 1.0
        wheels = [-K * omega_in, +K * omega_in,
                  -K * omega_in, +K * omega_in]
        vx, vy, omega = mk_forward(wheels, HALF_TRACK, HALF_WHEELBASE)
        assert vx    == pytest.approx(0.0,      abs=1e-6)
        assert vy    == pytest.approx(0.0,      abs=1e-6)
        assert omega == pytest.approx(omega_in, abs=1e-6)

    def test_signs_divide_out(self):
        """With Nezha signs [-1,1,-1,1], forward recovers the original twist."""
        signs = [-1, 1, -1, 1]
        vx_in, vy_in, omega_in = 100.0, 0.0, 0.0
        wheels = mk_inverse(vx_in, vy_in, omega_in, HALF_TRACK, HALF_WHEELBASE,
                            signs=signs)
        vx, vy, omega = mk_forward(wheels, HALF_TRACK, HALF_WHEELBASE, signs=signs)
        assert vx    == pytest.approx(vx_in,    abs=1e-5)
        assert vy    == pytest.approx(vy_in,    abs=1e-5)
        assert omega == pytest.approx(omega_in, abs=1e-5)


class TestMecanumRoundTrip:
    """inverse() then forward() must recover the original twist within 1e-4."""

    @pytest.mark.parametrize("vx,vy,omega", [
        (200.0,   0.0,  0.0),    # pure forward
        (0.0,   200.0,  0.0),    # pure strafe
        (0.0,     0.0,  1.0),    # pure rotate CCW
        (0.0,     0.0, -1.0),    # pure rotate CW
        (150.0,  75.0,  0.5),    # combined
        (-100.0, 50.0, -0.3),    # reverse + strafe + rotate
        (300.0, -100.0, 0.8),    # large values
        (0.0,     0.0,  0.0),    # zero twist (trivial)
    ])
    def test_round_trip_identity_signs(self, vx, vy, omega):
        """forward(inverse(twist)) == twist within 1e-4 (identity signs)."""
        wheels = mk_inverse(vx, vy, omega, HALF_TRACK, HALF_WHEELBASE)
        vx2, vy2, omega2 = mk_forward(wheels, HALF_TRACK, HALF_WHEELBASE)
        assert vx2    == pytest.approx(vx,    abs=1e-4)
        assert vy2    == pytest.approx(vy,    abs=1e-4)
        assert omega2 == pytest.approx(omega, abs=1e-4)

    @pytest.mark.parametrize("vx,vy,omega", [
        (200.0,   0.0,  0.0),
        (0.0,   150.0,  0.0),
        (100.0, -50.0,  0.5),
        (-80.0,  30.0, -0.7),
    ])
    def test_round_trip_nezha_signs(self, vx, vy, omega):
        """forward(inverse(twist)) == twist with Nezha signs [-1,1,-1,1]."""
        signs = [-1, 1, -1, 1]
        wheels = mk_inverse(vx, vy, omega, HALF_TRACK, HALF_WHEELBASE, signs=signs)
        vx2, vy2, omega2 = mk_forward(wheels, HALF_TRACK, HALF_WHEELBASE, signs=signs)
        assert vx2    == pytest.approx(vx,    abs=1e-4)
        assert vy2    == pytest.approx(vy,    abs=1e-4)
        assert omega2 == pytest.approx(omega, abs=1e-4)

    def test_round_trip_random(self):
        """Random twists round-trip within 1e-4 (identity signs)."""
        rng = random.Random(42)
        for _ in range(20):
            vx    = rng.uniform(-300.0, 300.0)
            vy    = rng.uniform(-300.0, 300.0)
            omega = rng.uniform(-2.0, 2.0)
            wheels = mk_inverse(vx, vy, omega, HALF_TRACK, HALF_WHEELBASE)
            vx2, vy2, omega2 = mk_forward(wheels, HALF_TRACK, HALF_WHEELBASE)
            assert vx2    == pytest.approx(vx,    abs=1e-4), f"vx round-trip failed ({vx},{vy},{omega})"
            assert vy2    == pytest.approx(vy,    abs=1e-4), f"vy round-trip failed ({vx},{vy},{omega})"
            assert omega2 == pytest.approx(omega, abs=1e-4), f"omega round-trip failed ({vx},{vy},{omega})"


class TestMecanumSaturate:
    """Verify uniform saturation preserves direction."""

    def test_no_saturation_below_limit(self):
        """Wheel speeds at or below limit pass through unchanged."""
        wheels = [100.0, -150.0, 200.0, -180.0]
        out = mk_saturate(wheels, 200.0)
        for i in range(4):
            assert out[i] == pytest.approx(wheels[i])

    def test_saturation_at_exact_limit(self):
        """Exactly at limit: pass through unchanged."""
        wheels = [200.0, -200.0, 100.0, -50.0]
        out = mk_saturate(wheels, 200.0)
        for i in range(4):
            assert out[i] == pytest.approx(wheels[i])

    def test_saturation_scales_uniformly(self):
        """When max exceeds limit, all outputs scale by same factor."""
        wheels = [100.0, 300.0, -200.0, 150.0]
        v_max = 200.0
        out = mk_saturate(wheels, v_max)
        # max abs = 300 > 200 -> s = 200/300
        s_expected = 200.0 / 300.0
        for i in range(4):
            assert out[i] == pytest.approx(wheels[i] * s_expected, rel=1e-6)

    def test_fastest_wheel_at_limit(self):
        """After saturation, the fastest wheel is exactly at v_wheel_max."""
        wheels = [100.0, 350.0, -200.0, 50.0]
        v_max = 250.0
        out = mk_saturate(wheels, v_max)
        assert max(abs(w) for w in out) == pytest.approx(v_max, rel=1e-6)

    def test_direction_preserved_after_saturation(self):
        """Saturated wheels, when run through forward, recover the original direction."""
        vx, vy, omega = 200.0, 100.0, 1.5
        wheels = mk_inverse(vx, vy, omega, HALF_TRACK, HALF_WHEELBASE)
        v_max = 150.0  # intentionally lower than the wheel speeds
        saturated = mk_saturate(wheels, v_max)
        # Forward of saturated wheels: direction must match original (ratios equal).
        vx2, vy2, omega2 = mk_forward(saturated, HALF_TRACK, HALF_WHEELBASE)
        # All components should be proportionally reduced — same direction.
        scale = max(abs(w) for w in wheels) / v_max  # > 1.0
        assert vx2    == pytest.approx(vx    / scale, rel=1e-5)
        assert vy2    == pytest.approx(vy    / scale, rel=1e-5)
        assert omega2 == pytest.approx(omega / scale, rel=1e-5)

    def test_negative_dominant_wheel(self):
        """Saturation works correctly when the dominant wheel is negative."""
        wheels = [50.0, -400.0, 100.0, -200.0]
        v_max = 300.0
        out = mk_saturate(wheels, v_max)
        # max abs = 400 > 300 -> s = 300/400 = 0.75
        assert out[1] == pytest.approx(-300.0, rel=1e-6)
        assert max(abs(w) for w in out) == pytest.approx(v_max, rel=1e-6)

    def test_pure_strafe_saturated_round_trips(self):
        """Saturated pure-strafe still recovers vy direction after forward."""
        wheels = mk_inverse(0.0, 400.0, 0.0, HALF_TRACK, HALF_WHEELBASE)
        v_max = 200.0
        sat = mk_saturate(wheels, v_max)
        vx, vy, omega = mk_forward(sat, HALF_TRACK, HALF_WHEELBASE)
        assert vx    == pytest.approx(0.0,   abs=1e-5)
        assert omega == pytest.approx(0.0,   abs=1e-5)
        assert vy    > 0.0  # direction preserved


class TestMecanumEquationsManual:
    """Manual spot-checks of the raw equations with k=1."""

    def test_identity_geometry_k1(self):
        """With k=1 (halfTrack=0.5, halfWheelbase=0.5), verify explicit arithmetic."""
        vx, vy, omega = 10.0, 5.0, 2.0
        wheels = mk_inverse(vx, vy, omega, 0.5, 0.5)  # k=1
        # FR = 10 - 5 - 2 = 3
        # FL = 10 + 5 + 2 = 17
        # BR = 10 + 5 - 2 = 13
        # BL = 10 - 5 + 2 = 7
        assert wheels[0] == pytest.approx(3.0)
        assert wheels[1] == pytest.approx(17.0)
        assert wheels[2] == pytest.approx(13.0)
        assert wheels[3] == pytest.approx(7.0)
        # Forward recovery
        vx2, vy2, omega2 = mk_forward(wheels, 0.5, 0.5)
        assert vx2    == pytest.approx(10.0, abs=1e-6)
        assert vy2    == pytest.approx(5.0,  abs=1e-6)
        assert omega2 == pytest.approx(2.0,  abs=1e-6)

    def test_strafe_sign_correctness(self):
        """Confirm sign correctness: vy=150 should give FL=+150, FR=-150.

        This guards against a common swap error where the sign of vy in the
        forward map uses +FR instead of -FR (which would make forward strafe
        read as a rotation, not a lateral move).
        """
        vx, vy, omega = 0.0, 150.0, 0.0
        wheels = mk_inverse(vx, vy, omega, HALF_TRACK, HALF_WHEELBASE)
        # FR = 0 - 150 - 0 = -150
        # FL = 0 + 150 + 0 = +150
        assert wheels[0] == pytest.approx(-150.0)  # FR
        assert wheels[1] == pytest.approx(+150.0)  # FL
        assert wheels[2] == pytest.approx(+150.0)  # BR
        assert wheels[3] == pytest.approx(-150.0)  # BL

        vx2, vy2, omega2 = mk_forward(wheels, HALF_TRACK, HALF_WHEELBASE)
        assert vy2    == pytest.approx(150.0, abs=1e-6), "forward must recover vy=150"
        assert vx2    == pytest.approx(0.0,   abs=1e-6)
        assert omega2 == pytest.approx(0.0,   abs=1e-6)


# ===========================================================================
# BodyKinematics array-overload tests (046-002)
# ===========================================================================

TRACK_WIDTH = 120.0  # mm

class TestBodyKinematicsArrayOverloads:
    """Verify the array-form overloads match the scalar BodyKinematics functions."""

    def test_inverse_array_matches_scalar_straight(self):
        """Array inverse gives same vL, vR as scalar (vy ignored)."""
        vx, omega = 200.0, 0.0
        wheels = bk_inverse_arr(vx, 0.0, omega, TRACK_WIDTH)
        # Scalar: vL = 200 - 0 = 200, vR = 200 + 0 = 200
        assert wheels[0] == pytest.approx(200.0)  # vL
        assert wheels[1] == pytest.approx(200.0)  # vR

    def test_inverse_array_spin_in_place(self):
        """Array inverse spin: vL = -60, vR = +60 for omega=1, b=120."""
        wheels = bk_inverse_arr(0.0, 0.0, 1.0, TRACK_WIDTH)
        assert wheels[0] == pytest.approx(-60.0)
        assert wheels[1] == pytest.approx(+60.0)

    def test_inverse_array_vy_ignored(self):
        """vy is ignored by the differential adapter."""
        w_with_vy    = bk_inverse_arr(100.0, 999.0, 0.5, TRACK_WIDTH)
        w_without_vy = bk_inverse_arr(100.0,   0.0, 0.5, TRACK_WIDTH)
        assert w_with_vy[0] == pytest.approx(w_without_vy[0], abs=1e-6)
        assert w_with_vy[1] == pytest.approx(w_without_vy[1], abs=1e-6)

    def test_forward_array_straight(self):
        """Array forward: equal wheels -> vx = wheel speed, omega = 0."""
        wheels = [200.0, 200.0]
        vx, vy, omega = bk_forward_arr(wheels, TRACK_WIDTH)
        assert vx    == pytest.approx(200.0)
        assert vy    == pytest.approx(0.0)
        assert omega == pytest.approx(0.0)

    def test_forward_array_vy_always_zero(self):
        """Array forward always sets vy = 0."""
        wheels = [40.0, 160.0]
        vx, vy, omega = bk_forward_arr(wheels, TRACK_WIDTH)
        assert vy == pytest.approx(0.0)

    @pytest.mark.parametrize("v,omega", [
        (100.0,  0.0),
        (0.0,    1.0),
        (200.0,  0.5),
        (-150.0, 0.3),
        (50.0,   2.0),
    ])
    def test_array_round_trip(self, v, omega):
        """bk_forward_arr(bk_inverse_arr(v, 0, omega)) recovers (v, 0, omega)."""
        wheels = bk_inverse_arr(v, 0.0, omega, TRACK_WIDTH)
        vx2, vy2, omega2 = bk_forward_arr(wheels, TRACK_WIDTH)
        assert vx2    == pytest.approx(v,     abs=1e-6), f"vx mismatch ({v},{omega})"
        assert vy2    == pytest.approx(0.0,   abs=1e-6)
        assert omega2 == pytest.approx(omega, abs=1e-6), f"omega mismatch ({v},{omega})"

    def test_saturate_array_below_ceiling(self):
        """Array saturate passes through when below ceiling."""
        wheels_in = [200.0, 300.0]
        out = bk_saturate_arr(wheels_in, v_wheel_max=400.0, steer_headroom=20.0)
        assert out[0] == pytest.approx(200.0)
        assert out[1] == pytest.approx(300.0)

    def test_saturate_array_scales_uniformly(self):
        """Array saturate scales both wheels by ceiling / max(|wi|)."""
        wheels_in = [300.0, 500.0]
        out = bk_saturate_arr(wheels_in, v_wheel_max=400.0, steer_headroom=20.0)
        # ceiling=380, max=500 -> s=380/500=0.76
        assert out[0] == pytest.approx(0.76 * 300.0, rel=1e-6)
        assert out[1] == pytest.approx(0.76 * 500.0, rel=1e-6)

    def test_saturate_array_preserves_curvature(self):
        """Curvature ratio vR/vL preserved after array saturation."""
        wheels_in = [300.0, 500.0]
        out = bk_saturate_arr(wheels_in, v_wheel_max=400.0, steer_headroom=20.0)
        assert (out[1] / out[0]) == pytest.approx(500.0 / 300.0, rel=1e-6)
