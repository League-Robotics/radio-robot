"""tests/unit/test_legacy_translate.py -- 097-002 (M4 Legacy Verb Translator).

Covers ``host/robot_radio/robot/legacy_translate.py``'s three functions,
each a pure/stateless transcription of one CURRENT firmware handler
(``source/commands/motion_commands.cpp``) plus the shared
``BodyKinematics::forward()`` (``source/kinematics/body_kinematics.cpp``):

- ``forward()`` -- the full two-output kinematic map (v, omega), checked
  directly against the cited equations for several (l, r, trackwidth)
  combinations.
- ``wheel_targets_for_drive()`` -- ``handleS()``'s per-wheel-speed
  passthrough (no kinematics at all).
- ``segment_for_timed()`` -- ``handleT()``'s l/r-sign-then-distance
  computation (``distance = v * (ms / 1000)``).
- ``segment_for_distance()`` -- ``handleD()``'s sign-then-distance
  computation (``distance = sign(v) * mm``).

Every expected value below is hand-computed from the cited source
constants/equations, not re-derived from this module's own implementation
(the "transcribe, verify, don't trust by inspection" discipline
architecture-update.md (097) Risk 1 asks for) -- see each test's own
docstring/comment for the exact firmware function/file being checked
against.

Collected under ``tests/unit/`` (host-side unit/tooling check, not
sim/bench/playfield-scoped -- see ``tests/CLAUDE.md``); ``pyproject.toml``'s
``testpaths`` includes ``tests/unit`` so ``uv run python -m pytest`` collects
it.
"""

from __future__ import annotations

import pytest

from robot_radio.robot import legacy_translate


# ---------------------------------------------------------------------------
# forward() -- BodyKinematics::forward() (source/kinematics/body_kinematics.cpp):
#   v_out     = (vR + vL) * 0.5
#   omega_out = (vR - vL) / b
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("v_left", "v_right", "trackwidth", "expected_v", "expected_omega"),
    [
        # Straight: vL == vR -> omega == 0.
        (200.0, 200.0, 128.0, 200.0, 0.0),
        # Pure differential turn: vL == -vR -> v == 0.
        (-200.0, 200.0, 128.0, 0.0, 400.0 / 128.0),
        # Asymmetric, negative overall (reverse).
        (-300.0, -100.0, 150.0, -200.0, 200.0 / 150.0),
        # Zero trackwidth-independent case (v only, arbitrary b).
        (500.0, 500.0, 64.0, 500.0, 0.0),
    ],
)
def test_forward_matches_body_kinematics_forward_equations(
    v_left, v_right, trackwidth, expected_v, expected_omega
):
    v, omega = legacy_translate.forward(v_left, v_right, trackwidth)
    assert v == pytest.approx(expected_v)
    assert omega == pytest.approx(expected_omega)


# ---------------------------------------------------------------------------
# wheel_targets_for_drive() -- handleS() (motion_commands.cpp): S <l> <r> ->
# msg::WheelTargets{speed=l, speed=r}, count=2. No kinematics.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("v_left", "v_right"),
    [(200, 150), (-200, 200), (0, 0), (1000, -1000)],
)
def test_wheel_targets_for_drive_is_a_direct_per_wheel_passthrough(v_left, v_right):
    wt = legacy_translate.wheel_targets_for_drive(v_left, v_right)
    assert len(wt.w) == 2
    assert wt.w[0].speed == pytest.approx(float(v_left))
    assert wt.w[1].speed == pytest.approx(float(v_right))


# ---------------------------------------------------------------------------
# segment_for_timed() -- handleT() (motion_commands.cpp):
#   v = BodyKinematics::forward(l, r, trackwidth).v   (omega discarded)
#   Motion::Segment.distance = v * (ms / 1000.0)
#   every other MotionSegment field left at its 0 default
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("v_left", "v_right", "duration", "expected_distance"),
    [
        # Straight, 1s at 200 mm/s -> 200mm.
        (200, 200, 1000, 200.0),
        # Straight, 500ms at 200 mm/s -> 100mm.
        (200, 200, 500, 100.0),
        # Reverse straight.
        (-150, -150, 2000, -300.0),
        # Asymmetric l/r: v = (l+r)/2 = (100+300)/2 = 200; 750ms -> 150mm.
        (100, 300, 750, 150.0),
        # Pure spin (vL == -vR): v == 0 -> distance == 0 regardless of ms.
        (200, -200, 1000, 0.0),
    ],
)
def test_segment_for_timed_matches_handle_t_distance_computation(
    v_left, v_right, duration, expected_distance
):
    seg = legacy_translate.segment_for_timed(v_left, v_right, duration)
    assert seg.distance == pytest.approx(expected_distance)
    # Every other field stays at its proto3 zero default (handleT() leaves
    # Motion::Segment's speedMax/etc. at 0 -- "falls back to the executor's
    # configured default").
    assert seg.direction == pytest.approx(0.0)
    assert seg.final_heading == pytest.approx(0.0)
    assert seg.speed_max == pytest.approx(0.0)
    assert seg.accel_max == pytest.approx(0.0)
    assert seg.jerk_max == pytest.approx(0.0)
    assert seg.time == pytest.approx(0.0)
    assert seg.v == pytest.approx(0.0)
    assert seg.omega == pytest.approx(0.0)
    assert seg.stream is False


# ---------------------------------------------------------------------------
# segment_for_distance() -- handleD() (motion_commands.cpp):
#   v = BodyKinematics::forward(l, r, trackwidth).v   (omega discarded)
#   sign = (v < 0.0f) ? -1.0f : 1.0f
#   Motion::Segment.distance = sign * mm
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("v_left", "v_right", "travel", "expected_distance"),
    [
        # Straight forward: v = 200 > 0 -> sign +1.
        (200, 200, 500, 500.0),
        # Straight reverse: v = -200 < 0 -> sign -1.
        (-200, -200, 500, -500.0),
        # Asymmetric but net-positive v: v = (50+250)/2 = 150 > 0 -> +1.
        (50, 250, 1000, 1000.0),
        # Asymmetric, net-negative v: v = (-300 + -100)/2 = -200 < 0 -> -1.
        (-300, -100, 250, -250.0),
        # v == 0 (pure spin): handleD()'s own ternary (v < 0) is false for
        # v == 0, so sign == +1 -- NOT 0. This is the exact edge case the
        # firmware's ternary resolves one specific way; transcribed, not
        # re-derived.
        (200, -200, 300, 300.0),
    ],
)
def test_segment_for_distance_matches_handle_d_sign_computation(
    v_left, v_right, travel, expected_distance
):
    seg = legacy_translate.segment_for_distance(v_left, v_right, travel)
    assert seg.distance == pytest.approx(expected_distance)
    assert seg.direction == pytest.approx(0.0)
    assert seg.final_heading == pytest.approx(0.0)
    assert seg.speed_max == pytest.approx(0.0)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
