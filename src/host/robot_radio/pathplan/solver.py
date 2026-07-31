"""robot_radio.pathplan.solver -- solveArcToPoint, the goto solver
(127-005, design issue T4).

History note (128-008): ``path/arc.py`` used to carry a per-wheel
left/right-distance tangent-arc formulation, and ``path/catmull_rom.py``
used to carry a spline lookahead-target picker with its own
circle/segment-intersection helper. Both solved the same two problems this
module solves independently -- ``solveArcToPoint()`` and
``pursuitTarget()`` respectively -- and were deleted as dead duplicates
(zero callers) once sprint 127 shipped this module. Recorded here, not
silently erased.

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
This bound exists for exactly one reason: to keep a ``replace=True``
transition from landing as an abrupt step while the firmware's own
profiler cannot ramp it cleanly. ``src/motion/planner/planner.cpp:1075``
reinterprets the carried ``profileVelocity_`` (held in axis/body-speed
units) under the NEW move's ``axisPerLambda`` on every ``replace`` -- a
firmware defect tracked separately as
``clasi/issues/replace-rescales-carried-profile-velocity-by-new-shape.md``
-- and ticket 001 (127-001) measured the single-tick commanded-wheel
discontinuity that bug produces: replacing an in-flight 150 mm/s straight
(``axisPerLambda = 1.0``) with a tight arc (``unitLeft=-0.5,
unitRight=1.0``, ``axisPerLambda = 0.25``) produces

    CASE2_EDGE_B_DISCONTINUITY_MM_S = 433.3333 mm/s

(ticket 001 Completion Notes, sim tier -- ``Planner::commandedLeft()``/
``commandedRight()``, the raw value the firmware's profiler computes with
zero actuation lag, i.e. the quantity the HOST controls). This solver
cannot fix the firmware's carry-over bug itself (host-only change) --
instead it bounds how much the CURVATURE is allowed to change between
successive solves, so the host never asks for a step anywhere near Edge
B's own numbers, regardless of how the firmware's carry-over bug
reinterprets whatever it is asked for.

**This is NOT an acceleration limiter.** The firmware's own profiler
already enforces one, on every wheel command it receives, independent of
anything this host solver does:

    plannerLimits.aMax = 300 mm/s^2   (src/firm/main.cpp)

A host-side slew bound set BELOW that figure would fight the firmware's
own limiter for no reason; one set comfortably ABOVE it never binds in
normal operation, because the firmware's profiler is always the tighter
constraint by construction. This bound is sized to stay above ``aMax``
deliberately, so it is only ever felt on the specific ``replace``
discontinuity it exists to smooth, not as a general motion limiter.

The budget is a deliberate fraction of the wheel's own physical
acceleration authority, taken from the plant model in
``src/firm/main.cpp``:

    kPlantGain = 1370.0   [mm/s per duty]
    kPlantTau  = 0.23     [s]
    plantAccel = kPlantGain / kPlantTau ~= 5957 mm/s^2  -- what a wheel
                                                            can PHYSICALLY do

``_SLEW_ACCEL = 2500 mm/s^2`` is ~42% of that -- comfortably above the
firmware's own ``aMax = 300 mm/s^2`` (so ``aMax`` stays the binding
constraint in normal operation) while remaining well short of the plant's
own physical ceiling:

    MAX_WHEEL_STEP = _SLEW_ACCEL * _SOLVE_PERIOD
                   = 2500.0 * 0.1
                   = 250.0 mm/s per solve

``_SOLVE_PERIOD`` is the planner loop's own re-solve cadence
(``planner.CYCLE_PERIOD = 0.1``): the budget is spent once per SOLVE, not
once per control tick, so it must be derived from the solve interval, not
from some other loop's period. If ``planner.CYCLE_PERIOD`` ever changes,
``_SOLVE_PERIOD`` here must be updated to match or this budget silently
drifts out of sync with the loop that actually spends it -- exactly the
mistake in the derivation this replaces, which derived a 50 ms
``_CONTROL_PERIOD`` from ``hil_drive.py``'s host-bench control-loop
constant but then spent it once every 100 ms (the real
``planner.CYCLE_PERIOD``), silently halving the already-too-conservative
budget a second time.

250.0 mm/s per solve stays comfortably under ticket 001's measured
433.3333 mm/s Edge-B hazard figure -- this solver still never asks for a
curvature step large enough to reproduce Edge B's own numbers -- while no
longer sitting ~100x below what the drivetrain, and the firmware's own
``aMax`` limiter, can actually do. (The prior derivation borrowed
``hil_drive.py``'s weakest measured BRAKING figure, ``aDecel=120 mm/s^2``,
to bound a STEERING change -- a different physical quantity with no
principled connection to Edge B's own failure mode. That reasoning is
retired, not merely adjusted; see git history if it is ever needed again.)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from robot_radio.nav.pose import Pose

_POSITION_SCALE = 10.0  # [mm/cm] Pose.x/y are cm; this module's own geometry and returns are mm.

_PLANT_GAIN = 1370.0  # [mm/s per duty] src/firm/main.cpp kPlantGain
_PLANT_TAU = 0.23  # [s] src/firm/main.cpp kPlantTau
_PLANT_ACCEL = _PLANT_GAIN / _PLANT_TAU  # [mm/s^2] ~5957 -- what a wheel can PHYSICALLY do

# [mm/s^2] slew budget -- a deliberate fraction (~42%) of plant capability.
# NOT an acceleration limit: the firmware's own profiler owns that
# (plannerLimits.aMax = 300). See the module docstring derivation.
_SLEW_ACCEL = 2500.0

# [s] the planner loop's own re-solve cadence (planner.CYCLE_PERIOD). The
# budget is spent once per SOLVE, so it must be derived from the solve
# interval -- deriving from a 50 ms control period and spending it every
# 100 ms was a 2x error.
_SOLVE_PERIOD = 0.1

MAX_WHEEL_STEP = _SLEW_ACCEL * _SOLVE_PERIOD  # [mm/s] per solve; see module docstring derivation (== 250.0)

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


# ---------------------------------------------------------------------------
# Path following: lookahead-circle pure pursuit (out-of-process, 2026-07-31)
# ---------------------------------------------------------------------------
#
# A caller driving a DENSE waypoint sequence (e.g. square_tour.py's rounded-
# corner fillets) cannot simply call this solver's own single-target loop
# (``pathplan.planner.gotoWorld()``) once per waypoint: that loop's arrival
# tolerance has a physical floor,
#
#     floor = speed * (cyclePeriod + actuationDelay)
#           = 150 * (0.1 + 0.15) = 37.5 mm
#
# (the loop cannot detect and act on "arrived" any faster than one solve
# cycle plus the actuation lag), which sits ABOVE the smallest chord a
# rounded-corner fillet can produce (~32.4 mm, square_tour.py's own
# TARGET_CORNER_RADIUS-at-the-leg/4-ceiling case) -- two walls with no gap,
# at any speed low enough for the floor to matter. Slowing the robot only
# shrinks the floor, it does not remove the conflict -- and trading speed
# for it was explicitly rejected (stakeholder, 2026-07-30: "What do you
# mean 80 mm/s! That is too slow!").
#
# RETIRED (2026-07-30, one day): the first answer to that was a
# pass-predicate -- ``hasPassedWaypoint()``/``advanceWaypointIndex()``, a
# half-plane dot-product test that advanced an INDEX into ``waypoints`` the
# instant the robot was judged to have passed the waypoint at that index,
# and then steered straight at ``waypoints[index]``. It passed 9/9 sim runs
# at the 500 mm leg and then, on its first playfield run (250 mm leg), drove
# ~920 mm in a straight line without turning at all and was stopped only by
# the geofence. Do NOT reintroduce it, and do not "fix" it by widening its
# proximity gate; the failure is structural, not a mistuned constant:
#
#   MEASURED 2026-07-31, reproduced in sim (`--mode goto --leg 250` with the
#   documented ~20% inbound command loss injected): 743 of 754 solve cycles
#   returned ``ArcSolution(stop=True)``. The trace shows the whole mechanism:
#
#   1. Steering at ``waypoints[index]`` means the commanded target is ONE
#      CHORD away -- 32.4 mm on the 250 mm-leg fillet. Nothing bounds that
#      target's BEARING. Passing a fillet waypoint by even ~29 mm puts the
#      next one at bearing 105 deg, i.e. past ``limits.behindAngle``.
#   2. The behind-guard then fires, so the planner sends nothing.
#   3. ``advanceWaypointIndex()`` cannot rescue it either: the robot is past
#      waypoint N but has not yet crossed waypoint N+1's own half-plane, so
#      the index does not move.
#   4. Deadlock -- the pose can only change by driving, and the only thing
#      that then drives is the planner's progress-stall backstop re-sending
#      the LAST GOOD arc, which was solved for a waypoint now far behind
#      (i.e. "go straight"). Every such resend makes the geometry worse:
#      positive feedback, straight ahead, until the geofence.
#
#   A slow control cycle is all it takes to enter that state -- one cycle
#   stretched by ``_sendVerifiedTwist()``'s own ack retries (the hardware
#   link's ~20% inbound loss) let the robot travel 68 mm instead of 15 mm and
#   overshoot the first fillet waypoint. The 500 mm-leg sim never hit it
#   because its chords are 46.6 mm and its cycles never stretched.
#
# THE FIX IS A DIFFERENT TARGET, not a different advance predicate. Real
# pure pursuit does not steer at a waypoint; it steers at the point on the
# PATH that is one lookahead distance ``L`` from the robot -- the
# lookahead-circle intersection. That is bearing-bounded BY CONSTRUCTION:
# the target sits ``L`` away in the direction the path goes, so it can only
# fall behind the robot if the robot's own heading is more than ~90 deg off
# the path, which is a genuine "we have left the path" condition worth
# stopping for -- not the routine consequence of a 29 mm overshoot. It also
# dissolves the original floor-vs-chord conflict outright: the target is a
# point ON the polyline, never a vertex OF it, so the waypoint spacing and
# the arrival tolerance stop interacting at all. Only the LAST waypoint
# still needs a real, tolerance-gated arrival test
# (``pathplan.planner.followPath()``'s own job) -- a path has to stop
# somewhere.


# How far ahead of the robot's own projection the projection search is
# allowed to run, as a multiple of the lookahead distance. This is the
# forward-only replacement for the retired pass-predicate's `_MAX_PASS_CHORDS`
# proximity gate, and it exists for the same real reason: a CLOSED path (a
# rounded square) has late segments sitting close to its own early ones in
# world space, so an unbounded nearest-point search can snap the projection
# to a section of path the robot has not travelled. Bounding the search
# window is safe in a way the retired gate was not, because this window is
# ONE-SIDED: the robot's current segment is always inside it, so the
# projection can always advance. The retired gate was two-sided -- it
# refused to trust a "passed" verdict from too far away, which blocked the
# TRUE positives from a genuinely overshooting robot exactly as readily as
# the false positives it was aimed at, and that is what latched.
_SEARCH_LOOKAHEADS = 4.0

# Minimum number of path segments the projection search always considers,
# however short they are relative to the lookahead -- guarantees the search
# can always reach the NEXT segment even on a path whose segments are much
# longer than `_SEARCH_LOOKAHEADS * lookahead`.
_MIN_SEARCH_SEGMENTS = 2


@dataclass(frozen=True)
class PursuitTarget:
    """One lookahead-circle pure-pursuit solve against a waypoint polyline.

    point: the world-frame point (``Pose``, cm, ``heading`` always 0.0 --
        this is a position to steer at, not a pose to attain) to feed
        ``solveArcToPoint()`` as its ``targetPoint``.
    segment: index of the polyline segment ``[waypoints[segment],
        waypoints[segment + 1]]`` the robot's own closest-point projection
        landed on. The caller carries this forward as the next call's
        ``fromSegment`` so the projection is MONOTONE -- it never slides
        backward onto path the robot has already driven.
    crossTrack: [mm] distance from the robot to that projection -- how far
        off the path it actually is. Reported, never acted on here; the
        caller decides what an excessive value means.
    remaining: [mm] path arc length from the projection to the terminal
        waypoint. The caller's own end-of-path bookkeeping.
    onCircle: True when ``point`` came from a real lookahead-circle
        intersection; False when it fell back to the projection itself
        (robot farther off the path than ``lookahead`` -- steer at the path
        first) or to the terminal waypoint (less than ``lookahead`` of path
        left). Diagnostic only.
    """

    point: Pose
    segment: int
    crossTrack: float   # [mm]
    remaining: float    # [mm]
    onCircle: bool


def _closestPointOnSegment(px: float, py: float, ax: float, ay: float,
                            bx: float, by: float) -> "tuple[float, float, float]":
    """Closest point to (px, py) on the segment (ax, ay)-(bx, by), as
    ``(t, x, y)`` with ``t`` clamped to [0, 1]. Same units in, same units
    out -- unit-agnostic. A degenerate (zero-length) segment returns its own
    start point at ``t = 0.0``."""
    dx = bx - ax
    dy = by - ay
    lengthSq = dx * dx + dy * dy
    if lengthSq <= 0.0:
        return 0.0, ax, ay
    t = ((px - ax) * dx + (py - ay) * dy) / lengthSq
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return t, ax + dx * t, ay + dy * t


def _circleSegmentExit(px: float, py: float, radius: float, ax: float, ay: float,
                        bx: float, by: float) -> "float | None":
    """The LARGEST ``t`` in [0, 1] at which the segment (ax, ay)-(bx, by)
    crosses the circle of radius ``radius`` centred on (px, py), or None if
    it never does. "Largest" == the farthest-along-the-path intersection,
    which is the one pure pursuit wants (the point where the path LEAVES the
    lookahead circle, not where it enters).

    ``radius`` and the coordinates must share units (this module calls it in
    cm)."""
    dx = bx - ax
    dy = by - ay
    fx = ax - px
    fy = ay - py
    a = dx * dx + dy * dy
    if a <= 0.0:
        return None
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return None
    root = math.sqrt(discriminant)
    for t in ((-b + root) / (2.0 * a), (-b - root) / (2.0 * a)):
        if 0.0 <= t <= 1.0:
            return t
    return None


def _searchEndSegment(waypoints: "list[Pose]", fromSegment: int, lookahead: float) -> int:
    """Last segment index (inclusive) the projection search may consider,
    starting at ``fromSegment`` -- see ``_SEARCH_LOOKAHEADS`` /
    ``_MIN_SEARCH_SEGMENTS`` for why the search is windowed at all."""
    lastSegment = len(waypoints) - 2
    budget = _SEARCH_LOOKAHEADS * lookahead  # [mm]
    end = fromSegment
    travelled = 0.0
    while end < lastSegment:
        a = waypoints[end]
        b = waypoints[end + 1]
        travelled += math.hypot(b.x - a.x, b.y - a.y) * _POSITION_SCALE
        if (end - fromSegment) >= (_MIN_SEARCH_SEGMENTS - 1) and travelled >= budget:
            break
        end += 1
    return end


def pursuitTarget(currentPose: Pose, waypoints: "list[Pose]", fromSegment: int,
                   lookahead: float) -> PursuitTarget:
    """Lookahead-circle pure pursuit against the polyline through
    ``waypoints``: project the robot onto the path, then return the point
    ``lookahead`` [mm] farther along it -- the point to steer at.

    ``fromSegment`` is the segment the PREVIOUS call's projection landed on;
    the search never looks behind it, so the projection is monotone along
    the path and a closed path (a square) can never snap the robot back onto
    a section it already drove. It is also windowed AHEAD (see
    ``_SEARCH_LOOKAHEADS``). Pass 0 for the first call.

    Three outcomes, all of them a real point to steer at -- this function
    never returns "no target", because a follower with no target is a
    follower that either stops dead or keeps running whatever it last sent
    (the retired pass-predicate's own failure mode; see this section's
    module comment):

    * The ordinary case: the lookahead circle crosses the path ahead ->
      ``onCircle=True``, ``point`` exactly ``lookahead`` from the robot.
    * Robot farther off the path than ``lookahead`` (nothing to intersect)
      -> ``point`` is the projection itself: steer AT the path, rejoin it,
      and the ordinary case resumes once within ``lookahead`` of it.
    * Less than ``lookahead`` of path remaining -> ``point`` is the terminal
      waypoint. The caller's arrival test ends the run.

    Pure geometry, no I/O -- unit-testable standalone. ``waypoints`` are in
    ``Pose``'s own cm; ``lookahead``, ``crossTrack`` and ``remaining`` are
    mm (this module's own boundary, ``_POSITION_SCALE``).
    """
    if len(waypoints) < 2:
        only = waypoints[0] if waypoints else Pose(x=currentPose.x, y=currentPose.y, heading=0.0)
        distance = math.hypot(only.x - currentPose.x, only.y - currentPose.y) * _POSITION_SCALE
        return PursuitTarget(point=Pose(x=only.x, y=only.y, heading=0.0), segment=0,
                             crossTrack=distance, remaining=0.0, onCircle=False)

    lastSegment = len(waypoints) - 2
    fromSegment = max(0, min(fromSegment, lastSegment))
    endSegment = _searchEndSegment(waypoints, fromSegment, lookahead)

    # --- 1. monotone, windowed closest-point projection --------------------
    bestSegment = fromSegment
    bestT = 0.0
    bestDistance = float("inf")
    bestPoint = (waypoints[fromSegment].x, waypoints[fromSegment].y)
    for segment in range(fromSegment, endSegment + 1):
        a = waypoints[segment]
        b = waypoints[segment + 1]
        t, x, y = _closestPointOnSegment(currentPose.x, currentPose.y, a.x, a.y, b.x, b.y)
        distance = math.hypot(currentPose.x - x, currentPose.y - y)
        if distance < bestDistance:
            bestDistance = distance
            bestSegment = segment
            bestT = t
            bestPoint = (x, y)
    crossTrack = bestDistance * _POSITION_SCALE  # [mm]

    # --- 2. remaining path length from the projection ----------------------
    remaining = math.hypot(waypoints[bestSegment + 1].x - bestPoint[0],
                           waypoints[bestSegment + 1].y - bestPoint[1]) * _POSITION_SCALE
    for segment in range(bestSegment + 1, lastSegment + 1):
        a = waypoints[segment]
        b = waypoints[segment + 1]
        remaining += math.hypot(b.x - a.x, b.y - a.y) * _POSITION_SCALE

    # --- 3. lookahead-circle intersection, searched FORWARD from the
    #        projection (so an intersection behind the robot's own progress
    #        can never be chosen) and windowed exactly like the projection
    #        search above.
    #
    #        The window matters as much here as it does there, and for the
    #        same reason: on a CLOSED path the far side of the loop comes
    #        back within a lookahead radius of the near side. MEASURED
    #        2026-07-31 with this search left unwindowed: on
    #        square_tour.py's own rounded square the very first solve, with
    #        the robot still 219 mm short of waypoint 0, found a circle
    #        intersection on SEGMENT 8 -- the return leg, which passes
    #        within ~10 mm of the start -- and commanded omega = +3.9 rad/s
    #        into it. The robot spun on the spot. A windowed search picks
    #        nothing there and correctly falls through to the
    #        "steer at the path" branch below.
    radius = lookahead / _POSITION_SCALE  # [cm]
    circleEnd = _searchEndSegment(waypoints, bestSegment, lookahead)
    for segment in range(bestSegment, circleEnd + 1):
        a = waypoints[segment]
        b = waypoints[segment + 1]
        # On the projection's OWN segment, only the stretch ahead of the
        # projection counts -- otherwise the circle's backward intersection
        # on this same segment would be picked and the robot would steer at
        # a point it has already driven past.
        startT = bestT if segment == bestSegment else 0.0
        ax = a.x + (b.x - a.x) * startT
        ay = a.y + (b.y - a.y) * startT
        t = _circleSegmentExit(currentPose.x, currentPose.y, radius, ax, ay, b.x, b.y)
        if t is None:
            continue
        point = Pose(x=ax + (b.x - ax) * t, y=ay + (b.y - ay) * t, heading=0.0)
        return PursuitTarget(point=point, segment=bestSegment, crossTrack=crossTrack,
                             remaining=remaining, onCircle=True)

    # No intersection anywhere ahead. Either the robot is farther off the
    # path than `lookahead` (steer at the path itself) or the path has run
    # out (steer at its end).
    if crossTrack >= lookahead:
        fallback = Pose(x=bestPoint[0], y=bestPoint[1], heading=0.0)
    else:
        terminal = waypoints[-1]
        fallback = Pose(x=terminal.x, y=terminal.y, heading=0.0)
    return PursuitTarget(point=fallback, segment=bestSegment, crossTrack=crossTrack,
                         remaining=remaining, onCircle=False)
