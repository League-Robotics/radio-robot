"""robot_radio.pathplan.solver -- solveArcToPoint, the goto solver
(127-005, design issue T4).

Pure math, no I/O: given a current pose and a target point, there is
exactly one circular arc through the target that is tangent to the
current heading. This module computes that arc's curvature and length and
packages the result as the parameters for a Distance-stopped body-frame
twist Move (``NezhaProtocol.move_twist(..., stop_distance=arcLength)``) --
never an Angle-stopped Move (design issue "never change axis while
moving" constraint -- see ``ArcSolution``'s own docstring for why this
holds by construction, not by a runtime check).

``solveArcToPoint()`` and its helpers take only their own arguments and
return only derived values -- no wire types in, no wire types out, no
robot, no hidden/module-level mutable state. The one piece of state a
caller-visible safety behavior needs (the curvature slew limit, which is
a property of a SEQUENCE of calls) is threaded through explicitly as the
``previousOmega`` argument -- the caller (the planner loop, ticket 006)
owns carrying it forward, this function never remembers anything between
calls itself.

Units: ``Pose.x``/``Pose.y`` are centimetres (``nav.pose.Pose``'s own
convention, matching the camera/world frame); ``ArcSolution.v_x``/
``arcLength`` are in the firmware wire's own units (mm/s, mm --
``NezhaProtocol.move_twist``'s convention), the same cm-to-mm boundary
``pathplan.world_pose``'s ``_POSITION_SCALE`` draws. ``omega`` is rad/s
and uses ``Pose.heading``'s own sign convention (CCW-positive, matching
camera yaw / ``nav.pose.heading_error``) in both frames -- any wire-level
sign correction a specific drivetrain's motor polarity might need is the
caller's concern (the planner loop that actually issues ``move_twist()``,
ticket 006), not this pure-geometry module's.

Two safety behaviors this module enforces, per sprint 127's Architecture
Decision 4 and the design issue's own T4 section:

1. **Curvature slew limit** (``_clampOmegaStep``) -- ``omega`` never steps
   between successive calls, only ramps. See the derivation below.
2. **Target-behind guard** (``limits.behindAngle``) -- arc-to-point
   degenerates toward infinite curvature as the target's body-frame
   bearing approaches +-180 degrees (a circle tangent to the current
   heading at the origin only ever touches the tangent line itself at
   that one point -- there is no finite-radius arc that reaches a point
   directly behind). Detected explicitly via the bearing magnitude and
   signalled with ``ArcSolution(stop=True)`` rather than emitting an
   extreme, physically-unrealizable arc -- safe because the axis change
   the caller's subsequent turn-in-place then needs happens from rest
   (ticket 001 case 4: zero discontinuity, trim integrator resets to 0
   cleanly on any axis change from rest).

--------------------------------------------------------------------------
Curvature slew limit -- derivation
--------------------------------------------------------------------------
Ticket 001 (127-001) measured the hazard this limit exists to prevent:
replacing an in-flight 150 mm/s straight (``axisPerLambda = 1.0``) with a
tight arc (``unitLeft=-0.5, unitRight=1.0``, ``axisPerLambda = 0.25``) via
``replace=True`` makes the firmware's carried ``profileVelocity_`` (held
in axis/body-speed units) get reinterpreted under the NEW move's
``axisPerLambda`` at ``src/motion/planner/planner.cpp:1075``
(``profileVelocity_ / active_.axisPerLambda``), producing a single-tick
commanded-wheel discontinuity of

    CASE2_EDGE_B_DISCONTINUITY_MM_S = 433.3333 mm/s

(ticket 001 Completion Notes, sim tier -- ``Planner::commandedLeft()``/
``commandedRight()``, the raw value the firmware's profiler computes with
zero actuation lag, i.e. the quantity the HOST controls). Ticket 001
explicitly recommends sizing against this sim figure rather than its own
bench-measured ~20 mm/s figure: the bench number is downstream of ~150 ms
actuation delay plus the wheel-velocity PID's own damping, both of which
are tuning artifacts a future retune could remove, not a safety property
to depend on.

This solver cannot fix the firmware's carry-over bug itself (host-only
change; the firmware defect is tracked separately as
``clasi/issues/replace-rescales-carried-profile-velocity-by-new-shape.md``
for a later sprint) -- instead it bounds how much the CURVATURE is
allowed to change between successive solves, from first principles: the
change in each wheel's commanded speed that a curvature step induces (at
a fixed forward speed) must stay within what the drivetrain's own
acceleration authority could physically achieve in one control period:

    maxWheelStep = aDecel * controlPeriod

``aDecel``/``controlPeriod`` are taken from ``hil_drive.py``'s
``hilLimits()`` (``src/motion/planner/bench/hil_drive.py``) -- the
MEASURED host-loop constants for this exact host-controls-a-replace-loop
topology, not ``src/firm/main.cpp``'s own book values (``aDecel=250``,
``controlPeriod=47ms``, an un-measured design target). ``aDecel=120
mm/s^2`` is documented there as "measured: bridge braking authority ~4x
weaker than sim plant" -- the most conservative (weakest) real
deceleration figure on record -- and ``controlPeriod=50ms`` is the exact
cadence ticket 001's own case 5 (20 Hz high-rate replacement) exercised
cleanly on hardware. Using the weaker authority figure is deliberate: a
safety bound should use the worst-case measured capability, not the
nominal one.

    maxWheelStep = aDecel * (controlPeriod / 1000.0)
                 = 120.0 * 0.050
                 = 6.0 mm/s per solve

6.0 mm/s is ~72x smaller than ticket 001's measured 433.3333 mm/s hazard
figure -- this physically-grounded, per-control-period authority bound
lands nowhere near the discontinuity that made Edge B unsafe, which is
exactly the point: this solver never asks for a curvature step large
enough to reproduce Edge B's own numbers, regardless of how the
firmware's carry-over bug reinterprets whatever it is asked for. A future
reader re-deriving this after ``hilLimits()``'s constants change need only
recompute ``aDecel * (controlPeriod / 1000.0)`` and re-check it stays
comfortably below whatever ticket 001 (or a future re-measurement of
Edge B) reports.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from robot_radio.nav.pose import Pose

_POSITION_SCALE = 10.0  # [mm/cm] Pose.x/y are cm; this module's own geometry and returns are mm.

_A_DECEL = 120.0  # [mm/s^2] hil_drive.py hilLimits() -- measured, weaker than the sim plant's book value
_CONTROL_PERIOD = 50.0  # [ms] hil_drive.py hilLimits() controlPeriod -- matches ticket 001's 20 Hz case 5 cadence

MAX_WHEEL_STEP = _A_DECEL * (_CONTROL_PERIOD / 1000.0)  # [mm/s] per solve; see module docstring derivation (== 6.0)

_MIN_DISTANCE = 1e-6  # [mm] below this, target is the robot's own position -- stop rather than divide by ~0
_MIN_BEARING = 1e-9  # [rad] below this, treat as exactly on-heading -- avoids a 0/0 in the general arc formula


@dataclass(frozen=True)
class SolverLimits:
    """Physical/safety limits ``solveArcToPoint`` respects.

    trackWidth: [mm] wheel separation -- differential-drive kinematics,
        used to convert the curvature slew limit's wheel-speed budget
        (``maxWheelStep``) into an omega-step budget.
    speed: [mm/s] the commanded forward body-frame speed for the emitted
        twist. This solver does not profile speed -- it emits one
        constant ``v_x`` per call (the firmware's own Move acceleration
        profile ramps into it); a caller wanting a slower approach speed
        near the goal passes a smaller ``speed``.
    maxWheelStep: [mm/s] the curvature slew limit, expressed as the
        maximum allowed per-wheel commanded-speed change between
        successive solves. Defaults to ``MAX_WHEEL_STEP`` -- see the
        module docstring for the full derivation.
    behindAngle: [rad] a target whose body-frame bearing magnitude
        exceeds this is treated as "behind" -- triggers the target-behind
        guard (stop-then-turn) instead of an extreme arc. Defaults to
        pi/2 (90 degrees): the tangent-circle arc formula is smooth and
        well-behaved for the whole |bearing| <= 90 degree range (radius
        shrinks monotonically from infinite at 0 degrees to
        distance/2 at 90 degrees) and only becomes pathological beyond
        it, so 90 degrees is a conservative cutoff that never lets the
        arc branch see the degenerate region at all.
    """

    trackWidth: float
    speed: float
    maxWheelStep: float = MAX_WHEEL_STEP
    behindAngle: float = math.pi / 2.0


@dataclass(frozen=True)
class ArcSolution:
    """One ``solveArcToPoint`` result -- either an arc twist or a stop
    signal. There is no field here that could express an Angle-stopped
    Move -- ``arcLength`` is always the Distance (Linear-axis) stop
    condition for ``move_twist(..., stop_distance=arcLength)`` -- so the
    "never change axis while moving" constraint (design issue) holds by
    construction, not by a runtime check.

    v_x: [mm/s] commanded forward body-frame speed. 0.0 when ``stop``.
    omega: [rad/s] commanded body-frame yaw rate, ``Pose.heading``'s own
        sign convention (CCW-positive). 0.0 when ``stop``.
    arcLength: [mm] the Distance stop condition -- always Linear-axis,
        never Angle. 0.0 when ``stop``.
    stop: True when the target-behind guard (or the zero-distance
        degenerate case) fired. The caller should halt (``estop()``, not
        drive an arc) and is expected to turn in place toward the target
        separately -- safe because that axis change then happens from
        rest (ticket 001 case 4). ``bearing`` below is provided so the
        caller does not have to recompute it for that turn.
    bearing: [rad] signed body-frame bearing to the target (0 = straight
        ahead, positive = left, matching the omega sign convention).
        Always the real computed bearing when ``stop`` is True from the
        target-behind guard; 0.0 for the zero-distance degenerate case
        (no direction to turn toward) and for an ordinary (non-stop)
        solution.
    """

    v_x: float  # [mm/s]
    omega: float  # [rad/s]
    arcLength: float  # [mm]
    stop: bool = False
    bearing: float = 0.0  # [rad]


def _clampOmegaStep(omega: float, previousOmega: float, trackWidth: float,
                     maxWheelStep: float) -> float:
    """Clamp ``omega``'s change from ``previousOmega`` so the per-wheel
    command delta it induces (differential-drive kinematics: a wheel's
    speed changes by ``|domega| * trackWidth / 2`` for a curvature-only
    change at fixed forward speed) never exceeds ``maxWheelStep`` -- the
    curvature slew limit. A non-positive ``trackWidth`` disables the
    clamp (there is no meaningful wheel-speed budget to convert to)."""
    if trackWidth <= 0.0:
        return omega
    maxOmegaStep = 2.0 * maxWheelStep / trackWidth  # [rad/s]
    delta = omega - previousOmega
    if delta > maxOmegaStep:
        return previousOmega + maxOmegaStep
    if delta < -maxOmegaStep:
        return previousOmega - maxOmegaStep
    return omega


def solveArcToPoint(currentPose: Pose, targetPoint: Pose, limits: SolverLimits,
                     previousOmega: float = 0.0) -> ArcSolution:
    """Compute the single circular arc from ``currentPose`` to
    ``targetPoint`` that is tangent to ``currentPose.heading``, and
    package it as a Distance-stopped body-frame twist.

    ``targetPoint`` is a full ``Pose`` (x, y, heading) so other, e.g.
    holonomic, drivetrains can reuse this signature later, but THIS
    differential-drive solver IGNORES ``targetPoint.heading`` -- only
    ``targetPoint.x``/``targetPoint.y`` matter. Honoring the target's
    final heading means a terminal turn-from-rest and is a later
    increment (sprint.md Out of Scope: "Terminal-theta honoring in the
    solver"), not this ticket.

    Pure function: for the same ``currentPose``/``targetPoint``/
    ``limits``/``previousOmega`` this always returns the same
    ``ArcSolution`` -- no I/O, no hidden state. ``previousOmega`` is the
    caller's own record of the PREVIOUS call's ``omega`` (0.0 for the
    first call in a sequence) -- curvature slew-limiting is a property of
    a SEQUENCE of calls, and since this function keeps no state of its
    own, the caller threads it through explicitly.

    Geometry: the target is rotated into the robot's own body frame
    (forward = +x, left = +y, matching ``Pose.heading``'s CCW-positive
    convention). Let ``distance`` be the straight-line distance to the
    target and ``bearing`` the signed body-frame angle to it (0 = ahead,
    positive = left). For a circle tangent to the forward direction at
    the origin, the well-known tangent-circle identities give the turn
    angle ``2*bearing`` and radius ``distance / (2*sin(bearing))``, from
    which:

        omega = speed * 2*sin(bearing) / distance
        arcLength = distance * bearing / sin(bearing)

    (both continuous through ``bearing == 0``, handled as an explicit
    on-heading special case below to avoid a 0/0). Two guards fire before
    this formula is evaluated -- see ``ArcSolution.stop``'s own
    docstring: the zero-distance degenerate case, and the target-behind
    guard (``|bearing| > limits.behindAngle``). Finally, ``omega`` is
    curvature-slew-clamped against ``previousOmega`` (see
    ``_clampOmegaStep`` and the module docstring's derivation) before
    being returned -- note this means the returned ``arcLength`` is
    computed for the UNCLAMPED tangent arc, not the clamped trajectory;
    that is intentional, not an oversight -- the planner loop (ticket
    006) continuously re-solves and replaces the in-flight Move, so any
    geometric drift the clamp introduces self-corrects on the very next
    solve rather than needing to be reconciled here.
    """
    dx = (targetPoint.x - currentPose.x) * _POSITION_SCALE  # [mm]
    dy = (targetPoint.y - currentPose.y) * _POSITION_SCALE  # [mm]
    heading = currentPose.heading

    # World -> body frame (forward = +x_b, left = +y_b).
    cosH = math.cos(heading)
    sinH = math.sin(heading)
    bodyX = dx * cosH + dy * sinH
    bodyY = -dx * sinH + dy * cosH

    distance = math.hypot(bodyX, bodyY)  # [mm]
    if distance < _MIN_DISTANCE:
        # Target is (numerically) the robot's own position -- no
        # direction to aim an arc at.
        return ArcSolution(v_x=0.0, omega=0.0, arcLength=0.0, stop=True, bearing=0.0)

    bearing = math.atan2(bodyY, bodyX)  # [rad]
    if abs(bearing) > limits.behindAngle:
        return ArcSolution(v_x=0.0, omega=0.0, arcLength=0.0, stop=True, bearing=bearing)

    if abs(bearing) < _MIN_BEARING:
        omega = 0.0
        arcLength = distance
    else:
        sinBearing = math.sin(bearing)
        omega = limits.speed * 2.0 * sinBearing / distance
        arcLength = distance * bearing / sinBearing

    omega = _clampOmegaStep(omega, previousOmega, limits.trackWidth, limits.maxWheelStep)
    return ArcSolution(v_x=limits.speed, omega=omega, arcLength=arcLength, stop=False, bearing=0.0)
