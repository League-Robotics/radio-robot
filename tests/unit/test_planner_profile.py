"""tests/unit/test_planner_profile.py -- 106-004 (SUC-027).

Exhaustively covers ``robot_radio.planner.profile`` -- the pure trapezoidal
profile generator (straight distance + in-place turn). No I/O, no hardware,
no sim; every test constructs a ``ProfileLimits`` and calls
``profile_for_distance()``/``profile_for_turn()`` directly.

Sections:
1. Trapezoid shape -- accel phase never exceeds ``a_max``, cruise phase
   holds exactly ``v_max``, area under the v-t curve equals the commanded
   distance/angle within tolerance.
2. Triangle shape -- a short distance/angle that never reaches ``v_max``
   produces a distinct shape (no cruise plateau), not a truncated trapezoid.
3. Sign preservation -- both a positive and a negative distance/angle are
   tested, proving the sign holds through EVERY setpoint (never a
   ``fabs``-blind check anywhere in these tests, per binding requirement
   #1 of ``host-planner-design-lessons-from-drive-v2-review.md``).
4. Terminal velocity -- the final setpoint of every profile lands at
   exactly 0.0, never a sign-reversal creep-back.
5. Cadence sampling -- setpoints are spaced by the requested cadence
   (except the final, possibly-shorter interval).
6. Validation -- zero distance/angle, non-positive limits, and non-finite
   values all raise ``ValueError`` immediately, producing no setpoints.

Collected under ``tests/unit/`` per ``pyproject.toml``'s ``testpaths``.
"""

from __future__ import annotations

import math

import pytest

from robot_radio.planner.profile import (
    ProfileLimits,
    profile_for_distance,
    profile_for_turn,
)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _area_under_curve(setpoints, field: str) -> float:
    """Trapezoidal-rule integral of ``field`` over ``elapsed`` -- should
    reconstruct the original commanded distance/angle."""
    total = 0.0
    for a, b in zip(setpoints, setpoints[1:]):
        dt = b.elapsed - a.elapsed
        v_a = getattr(a, field)
        v_b = getattr(b, field)
        total += 0.5 * (v_a + v_b) * dt
    return total


def _velocities(setpoints, field: str) -> list[float]:
    return [getattr(sp, field) for sp in setpoints]


# ---------------------------------------------------------------------------
# 1. Trapezoid shape
# ---------------------------------------------------------------------------


def test_straight_trapezoid_area_matches_distance():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)  # [mm/s] [mm/s^2]
    distance = 1000.0  # [mm] -- long enough to reach cruise
    setpoints = profile_for_distance(distance, limits, cadence=0.01)

    assert _area_under_curve(setpoints, "v_x") == pytest.approx(distance, abs=1.0)


def test_straight_trapezoid_reaches_and_holds_v_max():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    setpoints = profile_for_distance(1000.0, limits, cadence=0.01)

    v_x = _velocities(setpoints, "v_x")
    at_cruise = [v for v in v_x if v == pytest.approx(limits.v_max, abs=1e-9)]
    assert len(at_cruise) >= 2, "expected multiple samples holding exactly v_max"


def test_straight_trapezoid_accel_phase_never_exceeds_a_max():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    cadence = 0.01
    setpoints = profile_for_distance(1000.0, limits, cadence=cadence)

    for a, b in zip(setpoints, setpoints[1:]):
        dt = b.elapsed - a.elapsed
        if dt <= 0:
            continue
        slope = (b.v_x - a.v_x) / dt
        assert abs(slope) <= limits.a_max + 1e-6, (
            f"slope {slope} exceeds a_max {limits.a_max} between "
            f"t={a.elapsed} and t={b.elapsed}"
        )


def test_turn_trapezoid_area_matches_angle():
    limits = ProfileLimits(v_max=2.0, a_max=6.0)  # [rad/s] [rad/s^2]
    angle = math.pi  # [rad]
    setpoints = profile_for_turn(angle, limits, cadence=0.01)

    assert _area_under_curve(setpoints, "omega") == pytest.approx(angle, abs=0.02)


def test_turn_trapezoid_holds_omega_max():
    limits = ProfileLimits(v_max=2.0, a_max=6.0)
    setpoints = profile_for_turn(math.pi, limits, cadence=0.01)

    omega = _velocities(setpoints, "omega")
    at_cruise = [w for w in omega if w == pytest.approx(limits.v_max, abs=1e-9)]
    assert len(at_cruise) >= 2


def test_straight_profile_holds_omega_zero_throughout():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    setpoints = profile_for_distance(1000.0, limits, cadence=0.02)
    assert all(sp.omega == 0.0 for sp in setpoints)


def test_turn_profile_holds_v_x_zero_throughout():
    limits = ProfileLimits(v_max=2.0, a_max=6.0)
    setpoints = profile_for_turn(math.pi, limits, cadence=0.02)
    assert all(sp.v_x == 0.0 for sp in setpoints)


# ---------------------------------------------------------------------------
# 2. Triangle shape (distinct from trapezoid, not a truncated version)
# ---------------------------------------------------------------------------


def test_straight_triangle_never_reaches_v_max():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    # Short enough that d_acc_to_v_max (= v_max^2 / a_max = 80mm) exceeds it.
    distance = 40.0  # [mm]
    setpoints = profile_for_distance(distance, limits, cadence=0.001)

    v_x = _velocities(setpoints, "v_x")
    assert max(v_x) < limits.v_max
    assert max(v_x) > 0.0


def test_straight_triangle_area_matches_distance():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    distance = 40.0
    setpoints = profile_for_distance(distance, limits, cadence=0.001)

    assert _area_under_curve(setpoints, "v_x") == pytest.approx(distance, abs=0.5)


def test_triangle_shape_distinct_from_trapezoid():
    """A triangle profile has no plateau at v_max; a trapezoid does. This
    proves the triangle branch is a genuinely different shape, not a
    truncated trapezoid."""
    limits = ProfileLimits(v_max=200.0, a_max=500.0)

    triangle = profile_for_distance(40.0, limits, cadence=0.001)
    trapezoid = profile_for_distance(1000.0, limits, cadence=0.001)

    triangle_v = _velocities(triangle, "v_x")
    trapezoid_v = _velocities(trapezoid, "v_x")

    assert not any(v == pytest.approx(limits.v_max, abs=1e-9) for v in triangle_v)
    assert any(v == pytest.approx(limits.v_max, abs=1e-9) for v in trapezoid_v)
    # Triangle's peak velocity is strictly below the trapezoid's cruise plateau.
    assert max(triangle_v) < max(trapezoid_v)


def test_turn_triangle_never_reaches_omega_max():
    limits = ProfileLimits(v_max=2.0, a_max=6.0)
    angle = 0.1  # [rad] -- short turn
    setpoints = profile_for_turn(angle, limits, cadence=0.001)

    omega = _velocities(setpoints, "omega")
    assert max(omega) < limits.v_max


# ---------------------------------------------------------------------------
# 3. Sign preservation (positive and negative, straight and turn)
# ---------------------------------------------------------------------------


def test_negative_distance_preserves_sign_throughout():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    setpoints = profile_for_distance(-1000.0, limits, cadence=0.01)

    # Every non-terminal, non-initial sample must be strictly negative --
    # never a sign flip mid-sequence.
    interior = setpoints[1:-1]
    assert all(sp.v_x < 0.0 for sp in interior)
    assert setpoints[0].v_x == 0.0
    assert setpoints[-1].v_x == 0.0
    assert _area_under_curve(setpoints, "v_x") == pytest.approx(-1000.0, abs=1.0)


def test_negative_angle_preserves_sign_throughout():
    limits = ProfileLimits(v_max=2.0, a_max=6.0)
    setpoints = profile_for_turn(-math.pi, limits, cadence=0.01)

    interior = setpoints[1:-1]
    assert all(sp.omega < 0.0 for sp in interior)
    assert setpoints[-1].omega == 0.0
    assert _area_under_curve(setpoints, "omega") == pytest.approx(-math.pi, abs=0.02)


def test_positive_and_negative_distance_are_mirror_images():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    forward = profile_for_distance(1000.0, limits, cadence=0.01)
    reverse = profile_for_distance(-1000.0, limits, cadence=0.01)

    assert len(forward) == len(reverse)
    for f, r in zip(forward, reverse):
        assert f.elapsed == pytest.approx(r.elapsed)
        assert f.v_x == pytest.approx(-r.v_x, abs=1e-9)


# ---------------------------------------------------------------------------
# 4. Terminal velocity -- exact zero, never a creep-back
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("distance", [1000.0, -1000.0, 40.0, -40.0])
def test_straight_terminal_setpoint_is_exact_zero(distance):
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    setpoints = profile_for_distance(distance, limits, cadence=0.01)
    assert setpoints[-1].v_x == 0.0


@pytest.mark.parametrize("angle", [math.pi, -math.pi, 0.1, -0.1])
def test_turn_terminal_setpoint_is_exact_zero(angle):
    limits = ProfileLimits(v_max=2.0, a_max=6.0)
    setpoints = profile_for_turn(angle, limits, cadence=0.01)
    assert setpoints[-1].omega == 0.0


def test_no_sign_reversal_anywhere_in_sequence():
    """No setpoint's velocity has the OPPOSITE sign of the commanded
    direction at any point -- decel must coast to zero, never overshoot
    past zero and creep back with a reversed sign."""
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    setpoints = profile_for_distance(-500.0, limits, cadence=0.01)
    assert all(sp.v_x <= 0.0 for sp in setpoints)


# ---------------------------------------------------------------------------
# 5. Cadence sampling correctness
# ---------------------------------------------------------------------------


def test_cadence_spacing_is_respected_except_final_interval():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    cadence = 0.02
    setpoints = profile_for_distance(1000.0, limits, cadence=cadence)

    for a, b in zip(setpoints, setpoints[1:-1]):
        assert (b.elapsed - a.elapsed) == pytest.approx(cadence, abs=1e-9)


def test_elapsed_times_are_monotonically_non_decreasing():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    setpoints = profile_for_distance(1000.0, limits, cadence=0.03)

    for a, b in zip(setpoints, setpoints[1:]):
        assert b.elapsed >= a.elapsed


def test_first_setpoint_starts_at_rest():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    setpoints = profile_for_distance(1000.0, limits, cadence=0.01)
    assert setpoints[0].elapsed == 0.0
    assert setpoints[0].v_x == 0.0


def test_finer_cadence_yields_more_setpoints():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    coarse = profile_for_distance(1000.0, limits, cadence=0.1)
    fine = profile_for_distance(1000.0, limits, cadence=0.01)
    assert len(fine) > len(coarse)


# ---------------------------------------------------------------------------
# 6. Validation -- reject degenerate/invalid inputs at the boundary
# ---------------------------------------------------------------------------


def test_zero_distance_raises():
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    with pytest.raises(ValueError):
        profile_for_distance(0.0, limits)


def test_zero_angle_raises():
    limits = ProfileLimits(v_max=2.0, a_max=6.0)
    with pytest.raises(ValueError):
        profile_for_turn(0.0, limits)


@pytest.mark.parametrize("v_max", [0.0, -1.0])
def test_non_positive_v_max_raises(v_max):
    limits = ProfileLimits(v_max=v_max, a_max=500.0)
    with pytest.raises(ValueError):
        profile_for_distance(1000.0, limits)


@pytest.mark.parametrize("a_max", [0.0, -1.0])
def test_non_positive_a_max_raises(a_max):
    limits = ProfileLimits(v_max=200.0, a_max=a_max)
    with pytest.raises(ValueError):
        profile_for_distance(1000.0, limits)


@pytest.mark.parametrize("bad", [math.inf, -math.inf, math.nan])
def test_non_finite_distance_raises(bad):
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    with pytest.raises(ValueError):
        profile_for_distance(bad, limits)


@pytest.mark.parametrize("bad", [math.inf, -math.inf, math.nan])
def test_non_finite_angle_raises(bad):
    limits = ProfileLimits(v_max=2.0, a_max=6.0)
    with pytest.raises(ValueError):
        profile_for_turn(bad, limits)


@pytest.mark.parametrize("bad", [math.inf, math.nan])
def test_non_finite_v_max_raises(bad):
    limits = ProfileLimits(v_max=bad, a_max=500.0)
    with pytest.raises(ValueError):
        profile_for_distance(1000.0, limits)


@pytest.mark.parametrize("bad", [math.inf, math.nan])
def test_non_finite_a_max_raises(bad):
    limits = ProfileLimits(v_max=200.0, a_max=bad)
    with pytest.raises(ValueError):
        profile_for_distance(1000.0, limits)


@pytest.mark.parametrize("bad", [0.0, -0.01, math.inf, math.nan])
def test_invalid_cadence_raises(bad):
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    with pytest.raises(ValueError):
        profile_for_distance(1000.0, limits, cadence=bad)


def test_invalid_input_produces_no_setpoints():
    """A rejected call raises before generating anything -- there is no
    partial/degenerate sequence to inspect."""
    limits = ProfileLimits(v_max=200.0, a_max=500.0)
    try:
        profile_for_distance(0.0, limits)
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "distance" in str(exc)
