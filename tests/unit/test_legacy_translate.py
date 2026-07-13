"""tests/unit/test_legacy_translate.py -- 097-002/097-004 (M4 Legacy Verb
Translator).

Covers ``host/robot_radio/robot/legacy_translate.py``'s functions, each a
pure/stateless transcription of one CURRENT firmware handler
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
- ``segment_for_rt()`` (097-004) -- ``handleRT()``'s pure in-place-turn
  segment (``finalHeading = relAngle``, centidegrees -> radians).
- ``segment_for_move()`` (097-004) -- ``handleMove()``'s 1:1 field copy
  off ``parseMove``'s packed args, including the centidegree -> radian
  conversion for direction/finalHeading/yaw_*.
- ``segment_for_mover()`` (097-004) -- ``handleMover()``'s REPLACE-
  semantics segment: signed v/omega, ``speed_max``/``yaw_rate_max`` as
  their absolute value, ``stream`` unconditionally ``True``.

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
    """100-007, THE CUTOVER: segment_for_timed() now builds a v2 primitive
    (arc_length, not the retired `distance` field) via segment_for_seg()."""
    seg = legacy_translate.segment_for_timed(v_left, v_right, duration)
    assert seg.arc_length == pytest.approx(expected_distance)
    assert seg.delta_heading == pytest.approx(0.0)
    assert seg.exit_speed == pytest.approx(0.0)
    assert seg.primitive is True
    # Every retired field stays at its proto3 zero default -- segment_for_seg()
    # never touches them.
    assert seg.distance == pytest.approx(0.0)
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
    """100-007, THE CUTOVER: segment_for_distance() now builds a v2
    primitive (arc_length, not the retired `distance` field)."""
    seg = legacy_translate.segment_for_distance(v_left, v_right, travel)
    assert seg.arc_length == pytest.approx(expected_distance)
    assert seg.delta_heading == pytest.approx(0.0)
    assert seg.exit_speed == pytest.approx(0.0)
    assert seg.primitive is True
    assert seg.distance == pytest.approx(0.0)
    assert seg.direction == pytest.approx(0.0)
    assert seg.final_heading == pytest.approx(0.0)
    assert seg.speed_max == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# kCdegToRad (motion_commands.cpp): constexpr float kCdegToRad =
#   3.14159265f / 18000.0f;
# Computed independently here (not imported from legacy_translate's own
# module-level _CDEG_TO_RAD) so a transcription error in that constant
# would still be caught -- this file's own "verify, don't trust by
# inspection" discipline (see its own header docstring).
# ---------------------------------------------------------------------------
_CDEG_TO_RAD = 3.14159265 / 18000.0


# ---------------------------------------------------------------------------
# segment_for_rt() -- handleRT() (motion_commands.cpp): RT <relAngle> -> one
# pure in-place-turn Motion::Segment: distance=0, finalHeading=relAngle
# (relative, CCW+), converted centidegrees -> radians via kCdegToRad.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("rel_angle",),
    [(9000,), (-4500,), (0,), (18000,), (-18000,)],
)
def test_segment_for_rt_matches_handle_rt_final_heading_computation(rel_angle):
    """100-007, THE CUTOVER: segment_for_rt() now builds a v2 primitive
    pivot (delta_heading, not the retired `final_heading` field) via
    segment_for_seg()."""
    seg = legacy_translate.segment_for_rt(rel_angle)
    assert seg.delta_heading == pytest.approx(rel_angle * _CDEG_TO_RAD)
    assert seg.arc_length == pytest.approx(0.0)
    assert seg.exit_speed == pytest.approx(0.0)
    assert seg.primitive is True
    # Every retired field stays at its proto3 zero default.
    assert seg.distance == pytest.approx(0.0)
    assert seg.direction == pytest.approx(0.0)
    assert seg.final_heading == pytest.approx(0.0)
    assert seg.speed_max == pytest.approx(0.0)
    assert seg.stream is False


# ---------------------------------------------------------------------------
# segment_for_move() -- handleMove() (motion_commands.cpp): MOVE
# <distance_mm> <direction_cdeg> <finalHeading_cdeg> [v=][a=][j=][w=][wa=]
# [wj=][s=] -> Motion::Segment, a 1:1 field copy off parseMove's packed
# args; direction/finalHeading/w/wa/wj convert cdeg -> rad via kCdegToRad,
# v/a/j pass through unconverted, s=1 -> stream=True.
# ---------------------------------------------------------------------------


def test_segment_for_move_is_a_1to1_field_copy_with_cdeg_to_rad_conversion():
    seg = legacy_translate.segment_for_move(
        500, 9000, -9000,
        speed_max=300, accel_max=800, jerk_max=5000,
        yaw_rate_max=4500, yaw_accel_max=100000, yaw_jerk_max=200000,
        stream=True,
    )
    assert seg.distance == pytest.approx(500.0)
    assert seg.direction == pytest.approx(9000 * _CDEG_TO_RAD)
    assert seg.final_heading == pytest.approx(-9000 * _CDEG_TO_RAD)
    assert seg.speed_max == pytest.approx(300.0)
    assert seg.accel_max == pytest.approx(800.0)
    assert seg.jerk_max == pytest.approx(5000.0)
    assert seg.yaw_rate_max == pytest.approx(4500 * _CDEG_TO_RAD)
    assert seg.yaw_accel_max == pytest.approx(100000 * _CDEG_TO_RAD)
    assert seg.yaw_jerk_max == pytest.approx(200000 * _CDEG_TO_RAD)
    assert seg.stream is True


def test_segment_for_move_defaults_match_kvfloat_zero_sentinel():
    """An absent kv key defaults to 0.0 (kvFloat()'s own default), matching
    Motion::Segment's "0 => executor's configured default" convention
    (parseMove's own doc comment)."""
    seg = legacy_translate.segment_for_move(0, 0, 0)
    assert seg.speed_max == pytest.approx(0.0)
    assert seg.accel_max == pytest.approx(0.0)
    assert seg.jerk_max == pytest.approx(0.0)
    assert seg.yaw_rate_max == pytest.approx(0.0)
    assert seg.yaw_accel_max == pytest.approx(0.0)
    assert seg.yaw_jerk_max == pytest.approx(0.0)
    assert seg.stream is False


# ---------------------------------------------------------------------------
# segment_for_mover() -- handleMover() (motion_commands.cpp): MOVER
# <distance_mm> <direction_cdeg> <finalHeading_cdeg> [t=][v=][w=][a=][j=]
# [wa=][wj=] -> Motion::Segment, REPLACE semantics: stream ALWAYS True;
# v/omega SIGNED; speed_max=|v|, yaw_rate_max=|omega| (converted).
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("v", "omega"),
    [(300.0, 4500.0), (-300.0, -4500.0), (0.0, 0.0), (-150.0, 2000.0)],
)
def test_segment_for_mover_speed_max_and_yaw_rate_max_are_absolute_value(v, omega):
    seg = legacy_translate.segment_for_mover(0, 0, 0, time=400, v=v, omega=omega)
    assert seg.v == pytest.approx(v)
    assert seg.omega == pytest.approx(omega * _CDEG_TO_RAD)
    # handleMover(): seg.speedMax = fabsf(v); seg.yawRateMax = fabsf(w) * kCdegToRad;
    assert seg.speed_max == pytest.approx(abs(v))
    assert seg.yaw_rate_max == pytest.approx(abs(omega) * _CDEG_TO_RAD)
    assert seg.time == pytest.approx(400.0)


def test_segment_for_mover_stream_is_always_true():
    """handleMover()'s own unconditional ``seg.stream = true;`` -- unlike
    MOVE's caller-controlled s=1 kv, MOVER is always a streaming segment
    (the deadman-velocity teleop shape), even with every kwarg left at its
    default."""
    seg = legacy_translate.segment_for_mover(0, 0, 0)
    assert seg.stream is True


def test_segment_for_mover_distance_direction_final_heading_pass_through():
    seg = legacy_translate.segment_for_mover(500, 9000, -4500, v=0.0, omega=0.0)
    assert seg.distance == pytest.approx(500.0)
    assert seg.direction == pytest.approx(9000 * _CDEG_TO_RAD)
    assert seg.final_heading == pytest.approx(-4500 * _CDEG_TO_RAD)


# ---------------------------------------------------------------------------
# segment_for_arc() (100-007, THE CUTOVER -- supersedes the 097 open-loop
# velocity-pulse approximation) -- handleR()'s omega=speed/radius formula
# (transcribed; radius==0 -> omega=0) now maps onto a REAL, single,
# primitive=true arc segment: arc_length = speed*(duration/1000),
# delta_heading = arc_length/radius, exit_speed = 0 -- see the function's
# own docstring for the full derivation.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("speed", "radius", "duration", "expected_arc_length", "expected_delta_heading"),
    [
        (200.0, 500.0, 1000.0, 200.0, 200.0 / 500.0),
        (-200.0, 500.0, 1000.0, -200.0, -200.0 / 500.0),
        (200.0, -500.0, 1000.0, 200.0, -200.0 / 500.0),
        (200.0, 0.0, 1000.0, 200.0, 0.0),  # radius == 0 -> delta_heading == 0
        (200.0, 500.0, 500.0, 100.0, 100.0 / 500.0),  # duration scales arc_length
    ],
)
def test_segment_for_arc_matches_handle_r_formula(
    speed, radius, duration, expected_arc_length, expected_delta_heading
):
    seg = legacy_translate.segment_for_arc(speed, radius, duration=duration)
    assert seg.arc_length == pytest.approx(expected_arc_length)
    assert seg.delta_heading == pytest.approx(expected_delta_heading)
    assert seg.exit_speed == pytest.approx(0.0)
    assert seg.primitive is True


def test_segment_for_arc_default_duration_is_1000ms():
    seg = legacy_translate.segment_for_arc(200.0, 500.0)
    assert seg.arc_length == pytest.approx(200.0)  # 200 mm/s * 1.0s


# ---------------------------------------------------------------------------
# segment_for_turn() (097; primitive shape 100-007) -- handleTURN()'s
# original closed-loop turn is approximated open-loop AS IF the robot
# always starts at heading 0 (the SAME pure in-place-turn shape
# segment_for_rt() builds): arc_length=0, delta_heading=heading (converted
# cdeg -> rad).
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("heading",),
    [(9000,), (-4500,), (0,), (18000,), (-18000,)],
)
def test_segment_for_turn_matches_segment_for_rt_shape(heading):
    """The open-loop approximation is BYTE-IDENTICAL in shape to
    segment_for_rt() -- both build a pure in-place-turn primitive segment;
    only the semantic interpretation (relative vs. from-zero-absolute)
    differs at the call site, not the wire payload."""
    turn_seg = legacy_translate.segment_for_turn(heading)
    rt_seg = legacy_translate.segment_for_rt(heading)
    assert turn_seg.delta_heading == pytest.approx(heading * _CDEG_TO_RAD)
    assert turn_seg.delta_heading == pytest.approx(rt_seg.delta_heading)
    assert turn_seg.arc_length == pytest.approx(0.0)
    assert turn_seg.speed_max == pytest.approx(0.0)
    assert turn_seg.stream is False
    assert turn_seg.primitive is True


# ---------------------------------------------------------------------------
# segment_for_goto_relative() (097; primitive-list shape 100-007) --
# handleG()'s original Planner pursuit loop is approximated open-loop as a
# pivot-then-straight primitive PAIR (via primitives_for_move(), since
# final_heading == direction always omits the trailing pivot): arc_length=
# hypot(x,y) on the straight phase, delta_heading=atan2(y,x) on the leading
# pivot phase (finish facing the direction of travel, not the start
# heading).
# ---------------------------------------------------------------------------


def test_segment_for_goto_relative_distance_and_direction():
    import math

    segs = legacy_translate.segment_for_goto_relative(300.0, 400.0, speed=150.0)
    assert len(segs) == 2, "pivot-then-straight -- final_heading == direction always"
    pivot, straight = segs
    assert pivot.delta_heading == pytest.approx(math.atan2(400.0, 300.0))
    assert pivot.arc_length == pytest.approx(0.0)
    assert straight.arc_length == pytest.approx(500.0)  # 3-4-5 triangle
    assert straight.delta_heading == pytest.approx(0.0)
    for seg in segs:
        assert seg.primitive is True


def test_segment_for_goto_relative_at_origin_is_a_zero_distance_no_op():
    """x=y=0 -- both phases degenerate (zero pivot, zero straight) --
    primitives_for_move() omits BOTH, returning an empty list."""
    segs = legacy_translate.segment_for_goto_relative(0.0, 0.0)
    assert segs == []


def test_segment_for_goto_relative_default_speed_is_zero_sentinel():
    """speed (the old speed_max ceiling) has no v2 primitive equivalent --
    primitives_for_move() never sets it at all (see that function's own
    docstring); a caller passing speed=0 (the default) or any other value
    gets byte-identical primitives either way."""
    with_speed = legacy_translate.segment_for_goto_relative(300.0, 400.0, speed=150.0)
    without_speed = legacy_translate.segment_for_goto_relative(300.0, 400.0)
    assert [(s.arc_length, s.delta_heading) for s in with_speed] == \
        [(s.arc_length, s.delta_heading) for s in without_speed]


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
