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
# Pure-pursuit waypoint advance rule (out-of-process, 2026-07-30 -- this IS
# ticket 008's own algorithm, built here so 008 becomes "feed it a
# different path" rather than a fresh design).
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
# The fix is a different control model, not a smaller tolerance: STOP
# requiring arrival at interior waypoints. ``advanceWaypointIndex()``
# advances the target the instant the robot has PASSED it (via
# ``hasPassedWaypoint()``'s own projection test, not proximity) -- overshoot
# becomes the normal, expected case, so the arrival-tolerance floor above
# never applies to an interior waypoint at all. Only the LAST waypoint in a
# sequence still needs a real, tolerance-gated arrival test
# (``pathplan.planner.followPath()``'s own job) -- a path has to stop
# somewhere.


# `hasPassedWaypoint()`'s dot-product test is a LOCAL linear
# approximation -- a half-plane through `waypoint`, perpendicular to the
# path direction there. It is only a trustworthy proxy for "the robot has
# actually traveled this stretch of path" NEAR the waypoint; far from it,
# the half-plane extends to infinity and can be satisfied by a robot that
# is nowhere near this section of the path at all -- measured 2026-07-30
# on `square_tour.py`'s own rounded square (a CLOSED loop, so its late
# waypoints sit close to its own early ones in world space): a robot only
# partway around the first corner satisfied "passed" for a waypoint
# ~580 mm away, on the FAR side of the square, purely because it happened
# to be on that waypoint's own forward half-plane -- `advanceWaypointIndex()`
# then chained straight through six waypoints (half the path) in one call
# off that single false positive.
#
# `_MAX_PASS_CHORDS` bounds how far `advanceWaypointIndex()` will trust a
# "passed" verdict, scaled to the LOCAL chord length between `waypoint` and
# `nextWaypoint` (self-describing from the waypoint sequence's own density
# -- no external, path-specific constant needed): a dense fillet's ~30-47 mm
# chords get a correspondingly tight trust radius, while a long straight
# leg's own multi-hundred-mm "chord" (the gap between one corner's last
# sample and the next corner's first) gets a proportionally wide one. 3.0
# chords is enough slack for the legitimate multi-waypoint skip
# `advanceWaypointIndex()`'s own docstring describes ("several closely-
# spaced waypoints... e.g. a corner fillet's dense samples") without
# trusting a verdict from an entirely different, unrelated section of a
# path that loops back near itself.
_MAX_PASS_CHORDS = 3.0


def hasPassedWaypoint(currentPose: Pose, waypoint: Pose, nextWaypoint: Pose) -> bool:
    """True once the robot has passed ``waypoint``, judged by projecting its
    displacement FROM the waypoint onto the path's own direction THERE --
    the chord from ``waypoint`` to ``nextWaypoint`` -- not by proximity to
    the waypoint itself. A non-negative dot product means the robot sits on
    or beyond the waypoint's own forward side of the path, regardless of how
    far off to the side it might be: proximity alone would falsely say "not
    passed" for a robot that cut a corner wide, and falsely say "passed" for
    one still approaching from directly beside the waypoint.

    Pure geometry, no I/O -- unit-testable standalone. Works directly in
    ``Pose``'s own cm units: only the SIGN of the dot product matters, so no
    mm conversion is needed (the test is scale-invariant).

    Degenerate case: if ``waypoint`` and ``nextWaypoint`` coincide (a
    zero-length path direction -- should not occur for a real waypoint
    sequence, where consecutive points are always distinct), the dot
    product is always exactly 0.0, so this returns True. "Passed" is the
    safe default when there is no path direction to project onto: it lets
    ``advanceWaypointIndex()``'s caller move on rather than stall forever
    pointed at an ill-defined direction.
    """
    pathDx = nextWaypoint.x - waypoint.x
    pathDy = nextWaypoint.y - waypoint.y
    robotDx = currentPose.x - waypoint.x
    robotDy = currentPose.y - waypoint.y
    return (robotDx * pathDx + robotDy * pathDy) >= 0.0


def advanceWaypointIndex(currentPose: Pose, waypoints: "list[Pose]", index: int,
                         lookaheadFloor: float) -> int:
    """Advance ``index`` through ``waypoints`` while the waypoint currently
    at ``index`` has either been PASSED (``hasPassedWaypoint()``, projection
    onto the path) or sits closer than ``lookaheadFloor`` [mm] -- pure
    pursuit is unstable steering at a target inside the actuation-lag
    distance (see ``planner._lookaheadFloorFor()``'s own derivation), so a
    target that ends up too close gets skipped past exactly like a passed
    one, rather than steered at directly.

    Never advances to or past the LAST index in ``waypoints`` -- the
    terminal waypoint is always reached by an explicit arrival test
    (``planner.followPath()``'s own job), never by pass-through, so this
    function stops one short of it by construction (the ``while index <
    lastIndex`` bound below).

    May advance by MORE than one index in a single call: at speed, a robot
    can genuinely pass several closely-spaced waypoints within one solve
    cycle (e.g. a corner fillet's dense samples,
    ``square_tour.gotoSquareWaypoints()``'s own ``CORNER_SEGMENTS_PER_CORNER``
    hops) -- this loops rather than capping at one, so a fast-moving robot
    never gets stuck re-targeting a waypoint already well behind it. Bounded
    by ``_MAX_PASS_CHORDS`` (see its own module-level comment): a "passed"
    verdict is only trusted within a few chord-lengths of the waypoint, so
    this cannot cascade across an entire unrelated section of a path that
    loops back near itself.

    Pure, no I/O -- unit-testable standalone."""
    lastIndex = len(waypoints) - 1
    while index < lastIndex:
        waypoint = waypoints[index]
        nextWaypoint = waypoints[index + 1]
        chordMm = math.hypot(nextWaypoint.x - waypoint.x,
                             nextWaypoint.y - waypoint.y) * _POSITION_SCALE
        distanceMm = math.hypot(currentPose.x - waypoint.x,
                                currentPose.y - waypoint.y) * _POSITION_SCALE
        near = distanceMm <= max(chordMm * _MAX_PASS_CHORDS, lookaheadFloor)
        passed = near and hasPassedWaypoint(currentPose, waypoint, nextWaypoint)
        tooClose = distanceMm < lookaheadFloor
        if not (passed or tooClose):
            break
        index += 1
    return index
