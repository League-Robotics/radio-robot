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
    formula (module docstring): aDecel * controlPeriod, hil_drive.py's
    measured host-loop constants (aDecel=120 mm/s^2, controlPeriod=50ms)."""
    assert MAX_WHEEL_STEP == pytest.approx(120.0 * (50.0 / 1000.0))
    assert MAX_WHEEL_STEP == pytest.approx(6.0)
    # ~72x smaller than ticket 001's measured Edge-B hazard (433.3333 mm/s,
    # sim tier, CASE2_EDGE_B_DISCONTINUITY_MM_S) -- the whole point of the
    # derivation.
    assert 433.3333 / MAX_WHEEL_STEP == pytest.approx(72.22, abs=0.5)


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

def test_slew_limit_clamps_a_single_large_curvature_step():
    # Same 90-degree-left target as the geometry test above, whose
    # UNCLAMPED omega is 3.0 rad/s -- but this time with the real
    # (default, derived) limits, starting from rest.
    target = Pose(x=0.0, y=_cm(100.0), heading=0.0)
    result = solveArcToPoint(_HERE, target, _DEFAULT, previousOmega=0.0)
    maxOmegaStep = 2.0 * MAX_WHEEL_STEP / _TRACK_WIDTH
    assert result.omega == pytest.approx(maxOmegaStep)
    # Nowhere near the naive/unclamped 3.0 rad/s -- this is the "ramp not
    # step" property under test.
    assert result.omega < 0.5


def test_slew_limit_ramps_toward_the_target_over_successive_calls():
    target = Pose(x=0.0, y=_cm(100.0), heading=0.0)
    maxOmegaStep = 2.0 * MAX_WHEEL_STEP / _TRACK_WIDTH
    unclampedOmega = 3.0

    omega = 0.0
    previous = 0.0
    steps = 0
    for _ in range(64):
        result = solveArcToPoint(_HERE, target, _DEFAULT, previousOmega=previous)
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
    # (3.0 / 0.09375 == 32), not in one jump.
    assert steps == round(unclampedOmega / maxOmegaStep)
    assert steps > 1


def test_slew_limit_clamps_an_abrupt_reversal():
    # Steady state at +3.0 rad/s (as if converged toward a left target),
    # then the target flips to the mirror-image right target (naive
    # unclamped omega -3.0). Must ramp down, not jump straight to -3.0.
    target = Pose(x=0.0, y=_cm(-100.0), heading=0.0)
    maxOmegaStep = 2.0 * MAX_WHEEL_STEP / _TRACK_WIDTH
    result = solveArcToPoint(_HERE, target, _DEFAULT, previousOmega=3.0)
    assert result.omega == pytest.approx(3.0 - maxOmegaStep)
    assert result.omega > 0.0  # nowhere near the naive -3.0 in a single call


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
