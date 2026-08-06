"""src/tests/unit/test_solver.py -- ticket 127-005 originally; shrunk
135-007.

Pure, synthetic-input unit tests for ``robot_radio.pathplan.solver`` --
now just ``pursuitTarget()``, the lookahead-circle pure-pursuit target
picker. No hardware, no sim binary, no camera daemon, no serial port --
every input is a hand-built ``Pose``.

History (135-007): this file used to also cover ``solveArcToPoint()``/
``ArcSolution``/``SolverLimits``/the curvature slew limit -- ten of the
original test groups, covering on-heading/off-heading/symmetric solves,
the target-behind guard, the zero-distance degenerate case, the slew-clamp
ramp/reversal behavior, purity, final-heading-ignored, and the structural
"never an Angle-stopped move" guarantee. Those functions are DELETED
(135-007 moved single-arc solving into the firmware, ``Motion::ArcSolver``)
and their tests are deleted alongside them -- NOT a coverage loss: they
were ported test-for-test into
`src/motion/navigator/tests/arc_solver_test.cpp` (17 checks, confirmed by
direct comparison against this file's pre-135-007 history: identical
representative poses/targets/tolerances for every one of the ten groups
above, `kTrackWidth`/`kSpeed` matching this file's own `_TRACK_WIDTH`/
`_SPEED`, `unclampedLimits()`/`defaultLimits()` matching `_UNCLAMPED`/
`_DEFAULT`) before being removed here -- a coverage RELOCATION, not a
coverage loss. See `planner.py`'s own module docstring for the full
history of what moved and why.

Covers (ticket acceptance criteria):
  1. ``pursuitTarget()`` -- lookahead-circle pure pursuit against a
      waypoint POLYLINE (out-of-process, 2026-07-31, replacing the
      ``hasPassedWaypoint()``/``advanceWaypointIndex()`` pass-predicate
      that caused the playfield runaway; see ``solver.py``'s own
      "RETIRED" section comment): monotone forward projection, the
      lookahead-circle intersection, both fallbacks (robot off the path,
      path run out), the forward search window that keeps a CLOSED path
      from snapping onto its own far side, and -- the regression test that
      matters -- that a robot which has genuinely OVERSHOT a dense
      waypoint still gets a target ahead of it rather than one behind.
"""

from __future__ import annotations

import pytest

from robot_radio.nav.pose import Pose
from robot_radio.pathplan.solver import PursuitTarget, pursuitTarget

_HERE = Pose(x=0.0, y=0.0, heading=0.0)


def _cm(mm: float) -> float:
    return mm / 10.0


# ---------------------------------------------------------------------------
# pursuitTarget() -- lookahead-circle pure pursuit (out-of-process,
# 2026-07-31). Replaces the retired hasPassedWaypoint()/
# advanceWaypointIndex() pass-predicate; see solver.py's own "RETIRED"
# section comment for the playfield runaway that motivated the change.
# ---------------------------------------------------------------------------

_LOOKAHEAD = 114.0  # [mm] matches planner._lookaheadFor(150.0)'s own derivation


def _straightPath(lengths):
    """A straight east-running polyline whose vertices sit at the given
    cumulative distances [mm] from the origin."""
    return [Pose(x=_cm(d), y=0.0, heading=0.0) for d in lengths]


# --- the ordinary case: a point one lookahead along the path -------------

def test_pursuit_target_is_one_lookahead_along_a_straight_path():
    waypoints = _straightPath([0.0, 100.0, 200.0, 300.0, 400.0])
    robot = Pose(x=_cm(50.0), y=0.0, heading=0.0)
    target = pursuitTarget(robot, waypoints, 0, _LOOKAHEAD)
    assert target.onCircle is True
    assert target.crossTrack == pytest.approx(0.0, abs=1e-9)
    # Exactly `_LOOKAHEAD` from the robot, straight ahead.
    assert target.point.x * 10.0 == pytest.approx(50.0 + _LOOKAHEAD, abs=1e-6)
    assert target.point.y == pytest.approx(0.0, abs=1e-9)


def test_pursuit_target_reports_cross_track_and_remaining():
    waypoints = _straightPath([0.0, 200.0, 400.0])
    robot = Pose(x=_cm(100.0), y=_cm(30.0), heading=0.0)  # 30 mm north of the path
    target = pursuitTarget(robot, waypoints, 0, _LOOKAHEAD)
    assert target.crossTrack == pytest.approx(30.0, abs=1e-6)
    assert target.remaining == pytest.approx(300.0, abs=1e-6)  # 400 - 100


# --- THE REGRESSION: an overshooting robot still gets a target AHEAD -----

def test_overshooting_a_dense_waypoint_still_yields_a_forward_target():
    """The playfield runaway in one assertion.

    Dense waypoints (~32 mm apart, square_tour.py's own 250 mm-leg fillet
    chord spacing) with the robot 120 mm PAST the second one -- far more
    overshoot than the retired pass-predicate's proximity gate would trust,
    which is exactly what latched it: it refused to advance, the next
    target stayed behind the robot, the solver's behind-guard fired every
    cycle, and the only thing left driving was a stale re-send.

    `pursuitTarget()` cannot express that state: the projection is the
    CLOSEST point on the remaining path, so a target ahead of the robot
    always exists.

    135-007: this test used to also feed the picked target through
    ``solveArcToPoint()`` and assert its behind-guard never fired --
    ``solveArcToPoint()`` is deleted (single-arc solving moved firmware-
    side, ``Motion::ArcSolver``), so that half of the check is gone too;
    the equivalent guard is exercised directly against real bearings by
    `src/motion/navigator/tests/arc_solver_test.cpp`
    (`testTargetJustInsideBehindThresholdDoesNotStop` et al.), not by this
    file any more. What remains here -- the geometric guarantee that
    `pursuitTarget()` itself always returns a point AHEAD of an
    overshooting robot -- is unchanged and still the whole point of this
    test.
    """
    waypoints = _straightPath([0.0, 32.0, 64.0, 96.0, 400.0])
    robot = Pose(x=_cm(184.0), y=0.0, heading=0.0)  # 120 mm past waypoint 3
    target = pursuitTarget(robot, waypoints, 1, _LOOKAHEAD)
    assert target.point.x > robot.x, "target must be ahead of an overshooting robot"


# --- monotone projection: never slides back onto driven path -------------

def test_projection_never_moves_backward_along_the_path():
    waypoints = _straightPath([0.0, 100.0, 200.0, 300.0, 400.0])
    # Robot physically nearest segment 0, but told it is already on segment 2.
    robot = Pose(x=_cm(10.0), y=0.0, heading=0.0)
    target = pursuitTarget(robot, waypoints, 2, _LOOKAHEAD)
    assert target.segment >= 2


def test_projection_advances_as_the_robot_advances():
    waypoints = _straightPath([0.0, 100.0, 200.0, 300.0, 400.0])
    segment = 0
    seen = []
    for travelled in range(0, 400, 25):
        robot = Pose(x=_cm(float(travelled)), y=0.0, heading=0.0)
        target = pursuitTarget(robot, waypoints, segment, _LOOKAHEAD)
        segment = target.segment
        seen.append(segment)
    assert seen == sorted(seen), "the projection segment must be monotone"
    assert seen[-1] > seen[0], "and it must actually advance"


# --- the forward search window: a CLOSED path's far side is off limits ---

def test_a_closed_path_does_not_steer_at_its_own_far_side():
    """A square's LAST leg passes right back next to its first. Measured
    2026-07-31 with the lookahead-circle search left unwindowed: the very
    first solve, with the robot still short of waypoint 0, found an
    intersection on the RETURN leg ~10 mm away and commanded +3.9 rad/s
    into it -- the robot spun on the spot."""
    waypoints = [
        Pose(x=0.0, y=0.0, heading=0.0),
        Pose(x=_cm(400.0), y=0.0, heading=0.0),
        Pose(x=_cm(400.0), y=_cm(400.0), heading=0.0),
        Pose(x=0.0, y=_cm(400.0), heading=0.0),
        Pose(x=_cm(1.0), y=_cm(10.0), heading=0.0),  # returns beside the start
    ]
    robot = Pose(x=0.0, y=0.0, heading=0.0)
    target = pursuitTarget(robot, waypoints, 0, _LOOKAHEAD)
    # The target must be on the OUTBOUND leg (+x), not the return leg.
    assert target.point.x > 0.0
    assert target.point.y == pytest.approx(0.0, abs=1e-9)
    assert target.segment == 0


# --- fallbacks -----------------------------------------------------------

def test_robot_farther_off_the_path_than_the_lookahead_steers_at_the_path():
    waypoints = _straightPath([0.0, 400.0])
    robot = Pose(x=_cm(100.0), y=_cm(500.0), heading=0.0)  # 500 mm off to the side
    target = pursuitTarget(robot, waypoints, 0, _LOOKAHEAD)
    assert target.onCircle is False
    assert target.crossTrack == pytest.approx(500.0, abs=1e-6)
    # Steer at the closest point on the path, i.e. straight at it.
    assert target.point.x * 10.0 == pytest.approx(100.0, abs=1e-6)
    assert target.point.y == pytest.approx(0.0, abs=1e-9)


def test_less_than_a_lookahead_of_path_left_steers_at_the_terminal_waypoint():
    waypoints = _straightPath([0.0, 100.0])
    robot = Pose(x=_cm(80.0), y=0.0, heading=0.0)  # 20 mm of path left
    target = pursuitTarget(robot, waypoints, 0, _LOOKAHEAD)
    assert target.onCircle is False
    assert target.point.x == pytest.approx(waypoints[-1].x)
    assert target.remaining == pytest.approx(20.0, abs=1e-6)


def test_a_single_waypoint_path_is_its_own_target():
    only = Pose(x=_cm(250.0), y=0.0, heading=0.0)
    robot = Pose(x=0.0, y=0.0, heading=0.0)
    target = pursuitTarget(robot, [only], 0, _LOOKAHEAD)
    assert target.point.x == pytest.approx(only.x)
    assert target.crossTrack == pytest.approx(250.0, abs=1e-6)


# --- purity --------------------------------------------------------------

def test_pursuit_target_is_pure():
    waypoints = _straightPath([0.0, 100.0, 200.0, 300.0])
    robot = Pose(x=_cm(50.0), y=_cm(5.0), heading=0.3)
    first = pursuitTarget(robot, waypoints, 0, _LOOKAHEAD)
    second = pursuitTarget(robot, waypoints, 0, _LOOKAHEAD)
    assert first == second
    assert isinstance(first, PursuitTarget)
