"""robot_radio.pathplan.solver -- pursuitTarget(), the path-following
lookahead-point picker (127-005 originally, retained by 135-007).

History note (128-008): ``path/arc.py`` used to carry a per-wheel
left/right-distance tangent-arc formulation, and ``path/catmull_rom.py``
used to carry a spline lookahead-target picker with its own
circle/segment-intersection helper. Both solved the same two problems this
module used to solve independently -- the (since-deleted) single-arc goto
solver and ``pursuitTarget()`` respectively -- and were deleted as dead
duplicates (zero callers) once sprint 127 shipped this module. Recorded
here, not silently erased.

History note (135-007): ``solveArcToPoint()``/``ArcSolution``/
``SolverLimits``/``MAX_WHEEL_STEP``/``_clampOmegaStep()`` -- the
single-arc tangent-circle solver, its curvature slew clamp, and their
supporting plant-derived constants -- are DELETED as of sprint 135. Sprint
135 (`clasi/sprints/135-go-to-navigator-in-the-motion-library/`) moved
single-target arc-solving into the firmware itself
(``Motion::ArcSolver``, `src/motion/navigator/arc_solver.{h,cpp}`, ticket
002) -- a direct C++ port of this same geometry and slew-clamp derivation,
ctest-covered by `src/motion/navigator/tests/arc_solver_test.cpp` (17
checks: on-heading/off-heading/symmetric solves, the target-behind guard,
the zero-distance degenerate case, the slew-clamp ramp/reversal behavior,
purity, and the structural "never an Angle-stopped move" guarantee --
confirmed to cover this module's own former test surface,
`test_solver.py`'s pre-135-007 history, before those Python tests were
deleted alongside the functions). The host no longer solves arcs at all
for a point target: `pathplan.planner.gotoWorld()`/`gotoRobot()` now send
a `GO_TO` wire command and let `Motion::Navigator` drive it to completion
internally. See git history (pre-135-007) for the full prior derivation
if it is ever needed again -- it is not reproduced here since it no
longer describes any code in this file.

Pure math, no I/O: ``pursuitTarget()`` projects the robot onto a waypoint
polyline and returns the point one lookahead distance farther along it --
the lookahead-circle intersection. This is a HOST-side concern sprint
135's own Out of Scope section keeps host-side deliberately ("Pure
pursuit's own lookahead-point selection algorithm -- stays host-side,
unchanged in its own logic"): picking "which point on the path is next"
is a different problem from driving to one point, and the firmware's
internal `GO_TO` re-solve has no notion of a path at all, only of
whatever single target it was last given.

Units: ``Pose.x``/``Pose.y`` are centimetres (``nav.pose.Pose``'s own
convention, matching the camera/world frame); this module's own geometry
returns (``crossTrack``, ``remaining``) are in millimetres, the same
cm-to-mm boundary ``pathplan.world_pose``'s ``_POSITION_SCALE`` draws.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from robot_radio.nav.pose import Pose

_POSITION_SCALE = 10.0  # [mm/cm] Pose.x/y are cm; this module's own geometry and returns are mm.


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
