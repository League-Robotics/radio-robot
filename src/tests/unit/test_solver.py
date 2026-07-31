"""src/tests/unit/test_solver.py -- ticket 127-005.

Pure, synthetic-input unit tests for ``robot_radio.pathplan.solver`` --
``solveArcToPoint()``, the single-arc goto solver (design issue T4). No
hardware, no sim binary, no camera daemon, no serial port -- every input
is a hand-built ``Pose``.

Covers (ticket acceptance criteria):
  1. On-heading target (straight ahead) -> zero curvature.
  2. A 90-degree-offset target -> hand-computed arc (tangent-circle
     half-turn identity).
  3. A moderate (30-degree) off-heading target, both sides -> symmetric
     left/right curvature.
  4. The target-behind guard -> an explicit stop signal, not a
     degenerate/near-infinite-curvature twist.
  5. The zero-distance degenerate case -> also an explicit stop.
  6. The curvature slew limit -- a rapid sequence of calls toward a
     target that would otherwise demand a large single-call curvature
     step is shown to RAMP, never STEP.
  7. Purity / no hidden state: same inputs -> same outputs.
  8. The target's final heading is ignored.
  9. ``ArcSolution`` structurally cannot express an Angle-stopped move.
  10. ``hasPassedWaypoint()``/``advanceWaypointIndex()`` -- the pure-pursuit
      waypoint advance rule (out-of-process, 2026-07-30): projection-based
      "has passed", the lookahead-floor skip, multi-waypoint advance
      within one call, never advancing to/past the terminal index, and the
      ``_MAX_PASS_CHORDS`` proximity gate that stops a "passed" verdict
      from being trusted across an unrelated, far-away section of a path
      that loops back near itself (the measured failure this gate fixes:
      an un-gated version advanced straight past six waypoints of a
      12-point closed square off one false positive).
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from robot_radio.nav.pose import Pose
from robot_radio.pathplan.solver import (
    MAX_WHEEL_STEP,
    ArcSolution,
    SolverLimits,
    advanceWaypointIndex,
    hasPassedWaypoint,
    solveArcToPoint,
)

_TRACK_WIDTH = 128.0  # [mm] tovez.json drivetrain trackwidth (matches hil_drive.py hilLimits())
_SPEED = 150.0  # [mm/s] matches ticket 001's own Edge-B measurement speed

# Effectively-unclamped limits for tests that want to see the RAW arc
# geometry without the (separately, and thoroughly, tested) curvature
# slew limit interfering.
_UNCLAMPED = SolverLimits(trackWidth=_TRACK_WIDTH, speed=_SPEED, maxWheelStep=1.0e9)

# The real, derived (default) limits -- used by the slew-limit tests.
_DEFAULT = SolverLimits(trackWidth=_TRACK_WIDTH, speed=_SPEED)

_HERE = Pose(x=0.0, y=0.0, heading=0.0)


def _cm(mm: float) -> float:
    return mm / 10.0


# --- Derivation regression lock --------------------------------------------

def test_max_wheel_step_derivation():
    """Locks the derived constant to its documented first-principles
    formula (module docstring, out-of-process fix 2026-07-30): a slew
    budget (_SLEW_ACCEL, a deliberate fraction of the plant's own
    kPlantGain/kPlantTau acceleration authority) spent once per planner
    solve (_SOLVE_PERIOD, matching planner.CYCLE_PERIOD), NOT the old
    braking-derived aDecel * controlPeriod figure."""
    assert MAX_WHEEL_STEP == pytest.approx(2500.0 * 0.1)
    assert MAX_WHEEL_STEP == pytest.approx(250.0)
    # Still comfortably under ticket 001's measured Edge-B hazard
    # (433.3333 mm/s, sim tier, CASE2_EDGE_B_DISCONTINUITY_MM_S) -- the
    # whole point of the derivation -- while sitting above the firmware's
    # own plannerLimits.aMax = 300 mm/s^2 authority per solve (30 mm/s per
    # 0.1s), so aMax stays the binding constraint in normal operation.
    assert MAX_WHEEL_STEP < 433.3333
    assert MAX_WHEEL_STEP > 300.0 * 0.1


def test_default_behind_angle_is_90_degrees():
    assert SolverLimits(trackWidth=_TRACK_WIDTH, speed=_SPEED).behindAngle == pytest.approx(math.pi / 2.0)


# --- On-heading target -------------------------------------------------

def test_on_heading_target_gives_zero_curvature():
    target = Pose(x=_cm(200.0), y=0.0, heading=1.23)  # 200 mm straight ahead
    result = solveArcToPoint(_HERE, target, _UNCLAMPED)
    assert result.stop is False
    assert result.omega == pytest.approx(0.0, abs=1e-9)
    assert result.arcLength == pytest.approx(200.0)
    assert result.v_x == pytest.approx(_SPEED)


# --- 90-degree-offset target (hand-computed tangent-circle half-turn) --

def test_90_degree_offset_target():
    # Target 100mm directly to the LEFT of the robot's heading. The
    # tangent circle through the origin and (0, 100mm) is a half-circle
    # (radius 50mm) -- the well-known "target at 90 degrees needs a
    # 180-degree arc" tangent-circle identity (turn angle = 2*bearing).
    target = Pose(x=0.0, y=_cm(100.0), heading=0.0)
    result = solveArcToPoint(_HERE, target, _UNCLAMPED)
    assert result.stop is False
    assert result.omega == pytest.approx(2.0 * _SPEED / 100.0)  # kappa = 2/R, R = distance/2 = 50mm
    assert result.omega == pytest.approx(3.0)
    assert result.arcLength == pytest.approx(50.0 * math.pi)  # R * turn_angle = 50 * pi
    assert result.omega > 0.0  # target to the left -> positive (CCW) omega


# --- Moderate (30-degree) off-heading target, both sides ---------------

def test_off_heading_target_moderate_left():
    bearing = math.pi / 6.0  # 30 degrees
    distance = 200.0  # [mm]
    dx = distance * math.cos(bearing)
    dy = distance * math.sin(bearing)
    target = Pose(x=_cm(dx), y=_cm(dy), heading=0.0)
    result = solveArcToPoint(_HERE, target, _UNCLAMPED)
    assert result.stop is False
    assert result.omega == pytest.approx(_SPEED * 2.0 * math.sin(bearing) / distance)
    assert result.omega == pytest.approx(0.75)
    assert result.arcLength == pytest.approx(distance * bearing / math.sin(bearing))
    assert result.arcLength == pytest.approx(209.4395102)


def test_off_heading_target_symmetric_right_mirrors_left():
    bearing = math.pi / 6.0
    distance = 200.0
    dx = distance * math.cos(bearing)
    left = solveArcToPoint(_HERE, Pose(x=_cm(dx), y=_cm(distance * math.sin(bearing)), heading=0.0), _UNCLAMPED)
    right = solveArcToPoint(_HERE, Pose(x=_cm(dx), y=_cm(-distance * math.sin(bearing)), heading=0.0), _UNCLAMPED)
    assert right.omega == pytest.approx(-left.omega)
    assert right.arcLength == pytest.approx(left.arcLength)
    assert left.arcLength > 0.0
    assert right.arcLength > 0.0


# --- Target-behind guard -------------------------------------------------

def test_target_directly_behind_triggers_stop():
    target = Pose(x=_cm(-200.0), y=0.0, heading=0.0)  # 200mm directly behind
    result = solveArcToPoint(_HERE, target, _UNCLAMPED)
    assert result.stop is True
    assert result.v_x == 0.0
    assert result.omega == 0.0
    assert result.arcLength == 0.0
    assert result.bearing == pytest.approx(math.pi)


def test_target_well_past_behind_threshold_triggers_stop():
    # 120 degrees off-heading -- past the default 90-degree guard, still
    # short of directly behind. Must stop, not emit a degenerate arc.
    bearing = math.radians(120.0)
    distance = 200.0
    target = Pose(x=_cm(distance * math.cos(bearing)), y=_cm(distance * math.sin(bearing)), heading=0.0)
    result = solveArcToPoint(_HERE, target, _UNCLAMPED)
    assert result.stop is True
    assert result.bearing == pytest.approx(bearing)


def test_target_just_inside_behind_threshold_does_not_stop():
    # 80 degrees off-heading -- still inside the default 90-degree guard.
    bearing = math.radians(80.0)
    distance = 200.0
    target = Pose(x=_cm(distance * math.cos(bearing)), y=_cm(distance * math.sin(bearing)), heading=0.0)
    result = solveArcToPoint(_HERE, target, _UNCLAMPED)
    assert result.stop is False
    assert result.omega > 0.0
    assert result.arcLength > 0.0


# --- Zero-distance degenerate case ----------------------------------------

def test_target_at_robots_own_position_triggers_stop():
    target = Pose(x=_HERE.x, y=_HERE.y, heading=2.5)  # same position, any heading
    result = solveArcToPoint(_HERE, target, _UNCLAMPED)
    assert result.stop is True
    assert result.v_x == 0.0
    assert result.omega == 0.0
    assert result.arcLength == 0.0
    assert result.bearing == 0.0


# --- Curvature slew limit -- ramp, never step -----------------------------
#
# With the corrected MAX_WHEEL_STEP (250.0 mm/s per solve, up from the old
# 6.0), maxOmegaStep = 2*250/128 = 3.90625 rad/s -- large enough that the
# 90-degree/100mm target used pre-fix (unclamped omega 3.0 rad/s) no
# longer exceeds the budget at all, so it can't demonstrate clamping any
# more. These tests use a closer 90-degree target (25.6mm, chosen so the
# unclamped omega, 11.71875 rad/s, is an exact 3x multiple of the new
# maxOmegaStep) so the clamp still visibly engages.

_CLOSE_LEFT_TARGET = Pose(x=0.0, y=_cm(25.6), heading=0.0)
_CLOSE_RIGHT_TARGET = Pose(x=0.0, y=_cm(-25.6), heading=0.0)
_CLOSE_UNCLAMPED_OMEGA = 11.71875  # [rad/s] speed * 2*sin(90deg) / 25.6mm


def test_slew_limit_clamps_a_single_large_curvature_step():
    # 90-degree-left target close enough that its unclamped omega
    # (11.71875 rad/s) is well above the derived per-solve budget --
    # with the real (default, derived) limits, starting from rest.
    result = solveArcToPoint(_HERE, _CLOSE_LEFT_TARGET, _DEFAULT, previousOmega=0.0)
    maxOmegaStep = 2.0 * MAX_WHEEL_STEP / _TRACK_WIDTH
    assert result.omega == pytest.approx(maxOmegaStep)
    # Nowhere near the naive/unclamped 11.71875 rad/s -- this is the
    # "ramp not step" property under test.
    assert result.omega < _CLOSE_UNCLAMPED_OMEGA


def test_slew_limit_ramps_toward_the_target_over_successive_calls():
    maxOmegaStep = 2.0 * MAX_WHEEL_STEP / _TRACK_WIDTH
    unclampedOmega = _CLOSE_UNCLAMPED_OMEGA

    previous = 0.0
    steps = 0
    for _ in range(64):
        result = solveArcToPoint(_HERE, _CLOSE_LEFT_TARGET, _DEFAULT, previousOmega=previous)
        delta = result.omega - previous
        # Never a step larger than the derived per-solve budget.
        assert delta <= maxOmegaStep + 1e-9
        assert delta >= -1e-9  # monotonically ramping toward the target here
        previous = result.omega
        steps += 1
        if result.omega >= unclampedOmega - 1e-9:
            break

    assert previous == pytest.approx(unclampedOmega)
    # Converges in exactly the number of steps the budget predicts
    # (11.71875 / 3.90625 == 3), not in one jump.
    assert steps == round(unclampedOmega / maxOmegaStep)
    assert steps > 1


def test_slew_limit_clamps_an_abrupt_reversal():
    # Steady state at the close-left target's own unclamped omega (as if
    # converged toward it), then the target flips to the mirror-image
    # right target (naive unclamped omega is the negative of that). Must
    # ramp down, not jump straight to the naive reversed value.
    maxOmegaStep = 2.0 * MAX_WHEEL_STEP / _TRACK_WIDTH
    previousOmega = _CLOSE_UNCLAMPED_OMEGA
    result = solveArcToPoint(_HERE, _CLOSE_RIGHT_TARGET, _DEFAULT, previousOmega=previousOmega)
    assert result.omega == pytest.approx(previousOmega - maxOmegaStep)
    # Nowhere near the naive -11.71875 in a single call.
    assert result.omega > 0.0


# --- Purity / final-heading-ignored ---------------------------------------

def test_same_inputs_produce_same_outputs():
    target = Pose(x=_cm(150.0), y=_cm(50.0), heading=0.7)
    a = solveArcToPoint(_HERE, target, _UNCLAMPED, previousOmega=0.1)
    b = solveArcToPoint(_HERE, target, _UNCLAMPED, previousOmega=0.1)
    assert a == b


def test_target_final_heading_is_ignored():
    a = solveArcToPoint(_HERE, Pose(x=_cm(150.0), y=_cm(50.0), heading=0.0), _UNCLAMPED)
    b = solveArcToPoint(_HERE, Pose(x=_cm(150.0), y=_cm(50.0), heading=math.pi), _UNCLAMPED)
    c = solveArcToPoint(_HERE, Pose(x=_cm(150.0), y=_cm(50.0), heading=-2.4), _UNCLAMPED)
    assert a == b == c


# --- Structural: never an Angle-stopped move ------------------------------

def test_arc_solution_has_no_angle_stop_field():
    fieldNames = {f.name for f in dataclasses.fields(ArcSolution)}
    assert fieldNames == {"v_x", "omega", "arcLength", "stop", "bearing"}
    assert "stop_angle" not in fieldNames
    assert "angle" not in fieldNames


# ---------------------------------------------------------------------------
# 10. hasPassedWaypoint() / advanceWaypointIndex() -- pure-pursuit advance
#     rule (out-of-process, 2026-07-30).
# ---------------------------------------------------------------------------

_FLOOR = 22.5  # [mm] matches planner._lookaheadFloorFor(150.0)'s own derivation


# --- hasPassedWaypoint(): projection onto the path, not proximity ---------

def test_has_passed_waypoint_true_when_robot_is_ahead_along_the_path():
    # Path direction is +x (east); robot sits east of the waypoint -> passed.
    waypoint = Pose(x=0.0, y=0.0, heading=0.0)
    nextWaypoint = Pose(x=_cm(100.0), y=0.0, heading=0.0)
    robot = Pose(x=_cm(10.0), y=_cm(500.0), heading=0.0)  # far off to the SIDE, but ahead
    assert hasPassedWaypoint(robot, waypoint, nextWaypoint) is True


def test_has_passed_waypoint_false_when_robot_is_behind_along_the_path():
    waypoint = Pose(x=0.0, y=0.0, heading=0.0)
    nextWaypoint = Pose(x=_cm(100.0), y=0.0, heading=0.0)
    robot = Pose(x=_cm(-10.0), y=_cm(500.0), heading=0.0)  # off to the side, but BEHIND
    assert hasPassedWaypoint(robot, waypoint, nextWaypoint) is False


def test_has_passed_waypoint_exactly_at_the_waypoint_counts_as_passed():
    # Zero displacement -> dot product exactly 0.0 -> the ">= 0" boundary.
    waypoint = Pose(x=_cm(50.0), y=_cm(50.0), heading=0.0)
    nextWaypoint = Pose(x=_cm(150.0), y=_cm(50.0), heading=0.0)
    assert hasPassedWaypoint(waypoint, waypoint, nextWaypoint) is True


def test_has_passed_waypoint_degenerate_coincident_next_defaults_to_passed():
    # waypoint == nextWaypoint -> zero-length path direction -> dot product
    # is always exactly 0.0 -> "passed" is the documented safe default.
    waypoint = Pose(x=_cm(50.0), y=_cm(50.0), heading=0.0)
    robot = Pose(x=_cm(-500.0), y=_cm(500.0), heading=0.0)
    assert hasPassedWaypoint(robot, waypoint, waypoint) is True


def test_has_passed_waypoint_is_scale_invariant_cm_units_not_converted():
    # Only the SIGN of the dot product matters -- no mm conversion needed.
    waypoint = Pose(x=0.0, y=0.0, heading=0.0)
    nextWaypoint = Pose(x=1.0, y=0.0, heading=0.0)  # 1 cm ahead
    justPassed = Pose(x=0.001, y=0.0, heading=0.0)
    justBefore = Pose(x=-0.001, y=0.0, heading=0.0)
    assert hasPassedWaypoint(justPassed, waypoint, nextWaypoint) is True
    assert hasPassedWaypoint(justBefore, waypoint, nextWaypoint) is False


# --- advanceWaypointIndex(): lookahead floor, multi-step, terminal bound --

def test_advance_stays_put_when_far_ahead_and_not_passed():
    waypoints = [Pose(x=0.0, y=0.0, heading=0.0), Pose(x=_cm(100.0), y=0.0, heading=0.0)]
    robot = Pose(x=_cm(-500.0), y=0.0, heading=0.0)  # nowhere near waypoint 0
    assert advanceWaypointIndex(robot, waypoints, 0, _FLOOR) == 0


def test_advance_steps_forward_once_when_genuinely_passed():
    waypoints = [
        Pose(x=0.0, y=0.0, heading=0.0),
        Pose(x=_cm(100.0), y=0.0, heading=0.0),
        Pose(x=_cm(200.0), y=0.0, heading=0.0),
    ]
    robot = Pose(x=_cm(50.0), y=0.0, heading=0.0)  # past waypoint 0, well short of waypoint 1
    assert advanceWaypointIndex(robot, waypoints, 0, _FLOOR) == 1


def test_advance_never_reaches_the_terminal_index_by_pass_through():
    # A dense run (~30 mm apart, matching square_tour.py's own fillet
    # chord spacing) with the robot genuinely past waypoints 0-2, close
    # enough to each for the _MAX_PASS_CHORDS proximity gate to trust the
    # verdict -- advanceWaypointIndex() must stop AT MOST at lastIndex
    # (never past it, never treats the terminal waypoint as itself
    # pass-through-advanceable).
    waypoints = [
        Pose(x=0.0, y=0.0, heading=0.0),
        Pose(x=_cm(30.0), y=0.0, heading=0.0),
        Pose(x=_cm(60.0), y=0.0, heading=0.0),
        Pose(x=_cm(90.0), y=0.0, heading=0.0),  # terminal
    ]
    robot = Pose(x=_cm(85.0), y=0.0, heading=0.0)  # past 0, 1, and 2
    result = advanceWaypointIndex(robot, waypoints, 0, _FLOOR)
    assert result == 3  # == lastIndex, never 4 (out of range) or beyond
    assert result == len(waypoints) - 1


def test_advance_skips_multiple_closely_spaced_waypoints_in_one_call():
    # A dense run of waypoints ~30 mm apart (matching square_tour.py's own
    # fillet chord spacing) with the robot already past the first three,
    # but not yet within range (passed OR too-close) of the fourth.
    waypoints = [
        Pose(x=0.0, y=0.0, heading=0.0),
        Pose(x=_cm(30.0), y=0.0, heading=0.0),
        Pose(x=_cm(60.0), y=0.0, heading=0.0),
        Pose(x=_cm(90.0), y=0.0, heading=0.0),
        Pose(x=_cm(500.0), y=0.0, heading=0.0),  # terminal, far ahead
    ]
    robot = Pose(x=_cm(65.0), y=0.0, heading=0.0)  # past waypoints 0,1,2
    assert advanceWaypointIndex(robot, waypoints, 0, _FLOOR) == 3


def test_advance_skips_a_target_inside_the_lookahead_floor():
    # Waypoint 0 sits directly ahead but INSIDE the lookahead floor -- must
    # be skipped even though the robot has not, by the projection test,
    # technically "passed" it yet (it is still short of it).
    waypoints = [
        Pose(x=_cm(10.0), y=0.0, heading=0.0),   # 10 mm ahead of the robot
        Pose(x=_cm(200.0), y=0.0, heading=0.0),
    ]
    robot = Pose(x=0.0, y=0.0, heading=0.0)
    assert 10.0 < _FLOOR  # sanity: this scenario only makes sense if 10mm < floor
    assert advanceWaypointIndex(robot, waypoints, 0, _FLOOR) == 1


def test_advance_does_not_skip_a_target_outside_the_lookahead_floor():
    waypoints = [
        Pose(x=_cm(50.0), y=0.0, heading=0.0),  # 50 mm ahead -- outside the floor
        Pose(x=_cm(200.0), y=0.0, heading=0.0),
    ]
    robot = Pose(x=0.0, y=0.0, heading=0.0)
    assert 50.0 > _FLOOR
    assert advanceWaypointIndex(robot, waypoints, 0, _FLOOR) == 0


# --- _MAX_PASS_CHORDS proximity gate: the measured cascade-bug fix -------

def test_advance_does_not_trust_a_passed_verdict_from_far_across_a_looping_path():
    # Reproduces the measured 2026-07-30 failure: a closed/looping path (a
    # rounded square) has LATE waypoints geometrically close to its EARLY
    # ones. A robot only partway around the first corner can satisfy
    # hasPassedWaypoint()'s own half-plane test for a waypoint on the far
    # side of the square PURELY by being on the correct side of that
    # waypoint's own local chord direction -- despite being ~580 mm away,
    # nowhere near having actually traveled that section. The proximity
    # gate (_MAX_PASS_CHORDS) must refuse to trust that verdict.
    farWaypoint = Pose(x=_cm(45.0), y=_cm(487.9), heading=0.0)
    farNext = Pose(x=_cm(12.1), y=_cm(455.0), heading=0.0)  # ~46.5 mm chord
    waypoints = [
        Pose(x=0.0, y=0.0, heading=0.0),
        farWaypoint,
        farNext,
        Pose(x=_cm(90.0), y=0.0, heading=0.0),  # terminal
    ]
    robot = Pose(x=_cm(430.0), y=_cm(53.0), heading=0.0)  # ~580 mm from farWaypoint
    # Confirm the raw dot-product test WOULD say "passed" here (the
    # precondition this test is guarding against), then confirm
    # advanceWaypointIndex() does NOT act on it because the robot is not
    # plausibly near this section of the path.
    assert hasPassedWaypoint(robot, farWaypoint, farNext) is True
    assert advanceWaypointIndex(robot, waypoints, 1, _FLOOR) == 1


def test_advance_trusts_a_passed_verdict_within_a_few_chord_lengths():
    # The same waypoint pair, but with the robot genuinely nearby (within
    # _MAX_PASS_CHORDS * chordLength) -- the gate must not block a
    # legitimate, local "passed" verdict.
    waypoint = Pose(x=_cm(45.0), y=_cm(487.9), heading=0.0)
    nextWaypoint = Pose(x=_cm(12.1), y=_cm(455.0), heading=0.0)  # ~46.5 mm chord
    waypoints = [waypoint, nextWaypoint, Pose(x=_cm(0.0), y=_cm(410.0), heading=0.0)]
    robot = Pose(x=_cm(43.0), y=_cm(487.9) - _cm(23.0), heading=0.0)  # ~23 mm away, on the passed side
    assert advanceWaypointIndex(robot, waypoints, 0, _FLOOR) == 1
