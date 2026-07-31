"""robot_radio.pathplan.planner -- gotoWorld/gotoRobot, the outer position
loop (127-006, design issue T5).

One implementation, two entry points: `gotoWorld()` is the FULL control
loop (read telemetry -> update `WorldPose` -> solve -> `move_twist(...,
replace=True)` when the solution has moved materially -> repeat until
arrival or give-up); `gotoRobot()` is a thin composition through
`gotoWorld()` that first converts a ROBOT-frame target into a world-frame
one using the current `WorldPose` reading, then delegates the entire loop
to `gotoWorld()`. There is exactly one loop body in this module.

Two required properties (design issue T5, sprint.md SUC-006):

1. **Throttled replacement** -- re-solve every cycle, but only SEND
   (`move_twist(replace=True)`) when the solution has moved materially
   (curvature or remaining distance past a threshold, `ReplaceThreshold`
   / `_shouldReplace()`). Cuts link traffic and command churn.
2. **Explicit termination** -- an arrival tolerance and a give-up rule
   (`GiveUpLimits` / `_giveUpReason()`), never an infinite null-the-error
   loop. Below the drivetrain deadband, commands get zeroed and the robot
   stalls (the failure already fixed once via the `copysign(deadband)`
   boost, project memory `deadband-dead-zone-and-boost-fix.md`) -- this
   loop's own arrival tolerance must never be tighter than what the
   drivetrain can actually reach, which is exactly what
   `TERMINATION_TOLERANCE` (below) is provisional pending ticket 007's
   measurement of the real floor.

Move-id monotonicity (Architecture Open Question 2, sprint.md): the
firmware's `RobotLoop::handleMove()` dedups on `Move.id` BEFORE honoring
`replace` (`src/firm/app/robot_loop.cpp:224`, verified by ticket 002) --
a planner that reused or failed to advance ids would have its own
replacements silently vanish while still acking OK. `MoveIdAllocator`
(below) is this loop's one, explicit, strictly-monotonic id source; it
never emits `0` (id 0 is the dedup-EXEMPT sentinel -- see
`MoveIdAllocator`'s own docstring for why that makes it the wrong choice
for a continuously-replacing loop).

Only ever emits Distance-stopped twist moves (`move_twist(...,
stop_distance=...)`), matching `solveArcToPoint()`'s own "never an
Angle-stopped move" guarantee (`pathplan.solver`'s own docstring):
`axisOf()` keys on the STOP CONDITION, not wheel signs, so a
Distance-stopped twist is `Axis::Linear` however tight the arc --
staying on one axis for the whole loop avoids the firmware's
`profileVelocity_` carry-over hazard
(`clasi/issues/replace-rescales-carried-profile-velocity-by-new-shape.md`)
entirely, rather than needing the curvature slew limit to defend against
an axis change too. This loop never emits an Angle-stopped move, a
`move_wheels()`, or a planned `stop()` mid-motion -- when the solver's
target-behind guard fires (`ArcSolution.stop=True`), this loop treats it
as an explicit GIVE-UP (see `_targetBehindReason()`), not as a cue to
turn in place: a terminal in-place turn is safe only FROM REST (ticket
001 case 4) and honoring a target's final heading via such a turn is
sprint.md's own "Out of Scope: Terminal-theta honoring in the solver" --
a separate, later increment, not this ticket's.

Geofence discipline: checked INSIDE the ~10 Hz time-advance primitive
(`_advance()`, below), never between segments -- mirrors the idiom
`otos_calibration_bench.drainFrames()` already established (this module
reimplements the same shape rather than importing that function, since
it lives in a bench script and `pathplan` must never depend on
`src/tests/bench/` -- sprint.md's own stated dependency direction,
`pathplan` -> `field`/`path`/`protocol`, never the reverse). Every halt
path in this module calls `proto.estop()` (never the PLANNED `stop()` --
project memory `estop-not-stop-for-halting.md`), from a single `finally`
block in `gotoWorld()` that runs on every return AND on any exception
(including a `GeofenceViolation` raised out of `_advance()`) -- a halt
that raises must not be swallowed, so that exception is left to
propagate after the `finally` block's own `estop()` runs.

Ack verification, retry, and liveness (OOP fix, 2026-07-30, stakeholder
directive -- a live playfield run measured `gotoWorld()` sending exactly
ONE `move_twist()` over 264 solve iterations and 30s, with the robot
never moving at all): this loop used to discard `move_twist()`'s own
returned `corr_id` and never checked it -- a lost enqueue command (the
radio relay has documented sporadic loss, `.claude/rules/
hardware-bench-testing.md`) was invisible, and `_shouldReplace()`'s own
throttle then made the loss PERMANENT, since a stationary robot keeps
re-solving to the same (therefore suppressed) solution forever. Two
independent defenses now exist, and they are deliberately NOT the same
mechanism:

1. `_sendVerifiedTwist()` verifies the ENQUEUE ack for every send,
   retrying (`AckRetry`) up to a bounded number of attempts, matching
   `square_tour.Tour.sendVerified()`'s own approach -- scanning the SAME
   `_advance()` drain this loop already runs, never a second
   `wait_for_ack()` read (`sendVerified()`'s own docstring: "on a
   single-consumer telemetry queue two readers starve each other"). Every
   RETRY of one logical send reuses the SAME `Move.id` -- see
   `_sendVerifiedTwist()`'s own docstring for why that specific reuse is
   correct (it mirrors `RobotLoop::handleMove()`'s own documented
   dedup-ring contract), and why it is the ONE exception to "every
   logical send gets a fresh id" below.
2. `ProgressCheck` is a liveness backstop independent of ack outcome: if
   the robot's own world pose has not moved past `threshold` for
   `window` seconds, this loop force-resends regardless of what
   `_shouldReplace()` says -- because an enqueue can be acked OK and the
   robot still not move (a stalled/deadband move, or a move that
   finished short and the loop's solve happens to look unchanged). A
   forced resend always draws a FRESH `Move.id` from the allocator
   (unlike the ack-retry case above): it needs the firmware to actually
   restart the move via `planner_.move()`, not be silently re-acked as a
   no-op duplicate of a move already (believed) in flight.

`GotoResult.retries`/`.forcedResends`/`.unacked` (below) surface this
machinery's own activity, so a run that failed because commands were not
being acked no longer looks identical, in its own printed reason, to one
where the robot moved but simply never converged.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from typing import Callable

from robot_radio.field import Geofence
from robot_radio.nav.pose import Pose
from robot_radio.pathplan.solver import (
    ArcSolution,
    SolverLimits,
    pursuitTarget,
    solveArcToPoint,
)
from robot_radio.pathplan.world_pose import Transform2, WorldPose
from robot_radio.robot.protocol import NezhaProtocol

_POSITION_SCALE = 10.0  # [mm/cm] Pose.x/y are cm (nav.pose's own convention); this module's distance math is mm, matching TERMINATION_TOLERANCE and move_twist()'s own units.

# ---------------------------------------------------------------------------
# Provisional termination tolerance -- sprint.md Architecture Decision 4.
# ---------------------------------------------------------------------------
#
# The minimum reliable move distance/turn angle that should set this loop's
# arrival tolerance is UNMEASURED at the time this ticket (127-006) is
# implemented -- ticket 007 measures it (127-007 depends on 006, not the
# other way around: the measurement can only be taken once a real
# `gotoWorld()` loop exists to attempt short moves under its own
# throttling/termination logic). This is the ONE named, isolated
# definition site -- ticket 007 updates this exact constant with its
# measured value as part of its own acceptance; do not duplicate this
# number at any other call site in this module.
#
# PROVISIONAL -- pending ticket 007's measured minimum reliable move
# distance/turn angle; do not treat as final.
#
# Starting value taken from the design issue's own actuation-delay
# analysis, not a guess: ~150 ms actuation delay (the same figure
# `pathplan.solver`'s module docstring and `hil_drive.py`'s
# `actuationDelay` cite) means the carrot distance -- how far ahead of the
# robot the commanded target needs to stay for a ~5-10 Hz outer loop to
# keep up -- has to be >=100 mm. 100 mm is exactly that floor.
TERMINATION_TOLERANCE = 100.0  # [mm] world-frame distance from target counted as "arrived"

# --- Per-move timeout (move_twist()'s own safety backstop) -----------------
#
# Previously a single flat MOVE_TIMEOUT = 3000.0 ms for every move,
# regardless of arcLength -- fine for a 250 mm/150 mm/s leg (~1.7s) but
# scales badly: a longer leg's ideal drive time can exceed a flat 3s
# backstop outright, aborting a move that was still genuinely converging.
# _moveTimeoutFor() (below) derives the timeout from THIS solve's own
# arcLength and the solver's cruise speed instead, mirroring
# square_tour.py's own leg-timeout shape (`LEG / CRUISE * 1000 * 3 +
# 3000`) -- ideal duration times a generous multiplier, plus a floor
# covering accel/decel ramps and one ack round trip, capped so a bad
# solve (e.g. an unreachable, far-off-field target) cannot request an
# enormous timeout that would itself blow past this loop's own
# GiveUpLimits budget.
_MOVE_TIMEOUT_MULTIPLIER = 3.0    # generous margin over the ideal (arcLength / speed) drive duration
_MOVE_TIMEOUT_FLOOR = 3000.0      # [ms] additive floor -- accel/decel ramps + one ack round trip
_MOVE_TIMEOUT_CAP = 20000.0       # [ms] hard cap regardless of arcLength

CYCLE_PERIOD = 0.1       # [s] outer loop pacing / time-advance window -- matches sprint.md's stated realistic ~10 Hz outer-loop rate and the geofence's own ~10 Hz check cadence
_GEOFENCE_CHECK_PERIOD = 0.1  # [s] matches the "~10 Hz" geofence cadence -- never between segments
_POLL_SLEEP = 0.01       # [s] telemetry poll granularity inside _advance()

_ERR_FULL = 4  # envelope.proto ErrCode.ERR_FULL -- "destination queue full" (retryable; matches square_tour.py's own local ERR_FULL constant)

# --- Pure-pursuit lookahead distance (out-of-process, 2026-07-31) ---------
#
# `followPath()` steers at the point on the path one LOOKAHEAD distance
# ahead of the robot (`solver.pursuitTarget()`). That distance is bounded
# from both sides, and both bounds are physical:
#
# FLOOR -- steering lag, and it is the binding one. Between commanding a
# curvature and the robot actually holding it there is a dead time
# (150 ms, the same `actuationDelay` figure TERMINATION_TOLERANCE's own
# derivation above and hil_drive.py's `limits.actuationDelay = 150.0` use)
# plus the wheel plant's own first-order time constant (230 ms,
# `kPlantTau` in src/firm/main.cpp -- the same constant `solver.py`'s slew
# derivation cites). At CRUISE the robot therefore covers
#
#     speed * (_ACTUATION_DELAY + _PLANT_TAU) = 150 * 0.38 = 57 mm
#
# before a steering command takes hold. Pure pursuit with a lookahead of
# that ORDER is the textbook under-damped case: the robot's heading lags
# the path, the geometry answers a lagging heading with a TIGHTER arc, the
# lag delivers that tighter arc late, and the result overshoots and rings.
# MEASURED 2026-07-31 in the sim reproduction with L = 60 mm: entering the
# first 62.5 mm fillet the commanded omega swung +3.6 -> -2.8 -> +3.4 rad/s
# and the heading rang +-25 deg about its target -- barely stable in clean
# sim, and tipped over by any dropped command. `_LOOKAHEAD_GAIN = 2.0`
# buys the standard factor-of-two margin over that lag distance:
#
#     L = 2.0 * 150 * 0.38 = 114 mm
#
# CEILING -- corner-cutting. Pure pursuit tracks INSIDE a curve; for a
# fillet of radius r the steady-state deviation is about L^2 / (8r), and a
# lookahead approaching the fillet's own arc length cuts the corner off
# entirely. square_tour.py's tightest fillet is the 250 mm-leg case:
# r = 62.5 mm, quarter-arc 98.2 mm, so L = 114 mm cuts about
# 114^2 / (8 * 62.5) = 26 mm off that corner. That is real and visible --
# the rounded square comes out rounder than commanded -- and it is the
# price of stability at this speed on this radius. MEASURED 2026-07-31
# (leg 250, 10% injected command loss, 6 seeds each): L = 60 mm reached
# the terminal waypoint in 1 run of 6, L = 120 mm in 4 of 6, L = 150 mm in
# 4 of 6, L = 180 mm in 2 of 6 (the last cutting the corners so hard it
# starts missing them).
#
# THE TWO BOUNDS ARE UNCOMFORTABLY CLOSE, and that is a finding, not a
# tuning success: a 62.5 mm-radius corner taken at 150 mm/s asks for
# 2.4 rad/s of sustained yaw from a drivetrain whose steering lag covers
# 57 mm of travel. There is no lookahead that both tracks that corner
# tightly and damps the lag. Widening the corner (a longer leg, or
# relaxing square_tour.py's own `legLength / 4` radius ceiling) or slowing
# through the fillets are the only ways to open the gap; a curvature
# FEED-FORWARD term (command the path's own known curvature, correct with
# pure pursuit rather than deriving all of it from the chase geometry)
# would remove the dependence on lookahead altogether and is the right
# next increment.
_ACTUATION_DELAY = 0.15  # [s] command dead time
_PLANT_TAU = 0.23        # [s] src/firm/main.cpp kPlantTau -- wheel plant time constant
_LOOKAHEAD_GAIN = 2.0    # margin over the lag distance; see the derivation above
_MIN_LOOKAHEAD = 60.0    # [mm] hard floor for a very slow commanded speed


def _lookaheadFor(speed: float) -> float:
    """[mm] pure-pursuit lookahead distance for a commanded `speed` [mm/s]
    -- see the module-level comment above `_ACTUATION_DELAY` for the two
    physical bounds it sits between and why they nearly touch. For this
    project's CRUISE speed (150 mm/s): 2.0 * 150 * 0.38 = 114 mm. Pure, no
    I/O -- unit-testable standalone."""
    return max(speed * _LOOKAHEAD_GAIN * (_ACTUATION_DELAY + _PLANT_TAU), _MIN_LOOKAHEAD)


# --- followPath() and the target-behind guard: DO NOT widen it ------------
#
# Three rejected approaches, in order, before the one `followPath()` actually
# uses -- kept here as a durable record so this ground is not re-covered:
#
# 1. An UNBOUNDED "if behind, treat like passed and skip ahead" fallback.
#    Measured 2026-07-30: chained straight through six waypoints (half a
#    12-point square) off a SINGLE stop at one corner, reporting a false
#    "12/12 reached" while the robot had driven barely one leg. Bounding it
#    to a single forced step did not fully fix this either (see #2).
# 2. WIDENING `SolverLimits.behindAngle` for `followPath()`'s own solves
#    (interior only, then interior+terminal) on the theory that a
#    continuously re-solving loop "self-corrects" any transient sharp arc,
#    so a wider guard just lets it try. Measured 2026-07-30: intermittently
#    flaky (~1 run in 3) EVEN at 150 degrees -- some runs still exceeded it
#    (162.6 deg observed), and WorldPose samples between consecutive
#    waypoint crossings on a failing run jumped BACKWARD in world x by over
#    100 mm between two notifications only slightly apart, consistent with
#    `clasi/issues/replace-rescales-carried-profile-velocity-by-new-shape.md`
#    -- the documented firmware hazard where `Planner::commandedLeft()`/
#    `commandedRight()` reinterpret the carried `profileVelocity_` under a
#    NEW move's `axisPerLambda` on every `replace=True`. A wider guard
#    directly means BIGGER curvature (smaller axisPerLambda) swings between
#    consecutive replaces -- exactly the shape of discontinuity the
#    solver's own curvature slew limit (`MAX_WHEEL_STEP`, `solver.py`'s own
#    module docstring) exists to stay clear of. Widening the guard was
#    deliberately courting the same hazard the slew limit was built to
#    avoid, not sidestepping it.
# 3. Sending NOTHING while the guard fires, and letting `ProgressCheck`'s
#    liveness backstop re-send the LAST SENT arc once the robot had
#    genuinely stalled. This is the one that reached the playfield, and it
#    is the one that ran away: MEASURED 2026-07-31 (sim reproduction,
#    `--mode goto --leg 250` with the hardware link's documented ~20%
#    inbound command loss injected) 743 of 754 solve cycles returned
#    `stop=True`, and the ONLY thing driving the robot for all of them was
#    that backstop re-sending an arc solved for a waypoint already far
#    behind -- i.e. "go straight". Each resend put the robot further off
#    the path, which kept the guard firing: positive feedback, in a
#    straight line, until the geofence. On hardware this drove ~920 mm with
#    the heading unchanged (+20 deg -> +18 deg).
#
#    So: a stale resend is not a liveness backstop, it is an open-loop
#    command issued precisely when the closed loop has said it does not
#    know where to go. `followPath()` no longer does it -- see
#    `PathAbandoned` and the solve call site below. `ProgressCheck`-forced
#    resends survive only on the path where the solver DID return a real
#    arc for the CURRENT target.
#
# The root fix is upstream of all three: `solver.pursuitTarget()` steers at
# a point one lookahead distance along the path rather than at the next
# waypoint, which bounds the target's bearing by construction. See that
# function's own section comment in `solver.py`.


def _moveTimeoutFor(arcLength: float, speed: float) -> float:
    """Derive one move's `timeout` [ms] from its own `arcLength` [mm] and
    the solver's configured cruise `speed` [mm/s] -- see the module-level
    comment above this function for the rationale and the constants it
    uses. Pure, no I/O -- unit-testable standalone."""
    safeSpeed = max(speed, 1.0)  # guard a zero/misconfigured speed rather than divide by zero
    idealDuration = abs(arcLength) / safeSpeed * 1000.0  # [ms]
    return min(idealDuration * _MOVE_TIMEOUT_MULTIPLIER + _MOVE_TIMEOUT_FLOOR, _MOVE_TIMEOUT_CAP)


class MoveRejected(RuntimeError):
    """Raised by `_sendVerifiedTwist()` when the firmware NACKs a
    `move_twist()` with a non-retryable `ErrCode` -- anything other than
    `ERR_FULL` (e.g. `ERR_BADARG`: a malformed Move, a caller bug no
    retry can fix). Propagates out of `gotoWorld()` after its own
    `finally` block's `estop()` runs -- "a halt that raises must not be
    swallowed" (this module's own docstring)."""


@dataclass(frozen=True)
class ReplaceThreshold:
    """Throttled-replacement thresholds -- `_shouldReplace()`'s own
    material-change test. Ordinary loop-pacing knobs, not a measured
    physical quantity (unlike `TERMINATION_TOLERANCE` above).

    omegaThreshold: [rad/s] a solved omega must differ from the last SENT
        omega by more than this to trigger a replacement. Set above the
        solver's own per-solve curvature-slew-clamp step so a still-
        converging sequence keeps sending (each solve's clamped omega
        differs from the last sent value by close to the clamp's own
        step while still converging) while a STABLE solution (omega no
        longer changing) stops sending.
    arcLengthThreshold: [mm] a solved arc length must differ from the
        last SENT arc length by more than this to trigger a replacement
        -- absorbs ordinary per-cycle noise in the measured pose without
        replacing on every single cycle.
    refreshFraction: re-send once the robot has covered this fraction
        of the arc length it was last COMMANDED, regardless of whether the
        solution itself has moved. Needed by `followPath()`, whose target
        is a fixed lookahead distance along the path: on a straight run
        both the solved `omega` (0) and the solved `arcLength` (the
        lookahead) are CONSTANT cycle after cycle, so neither threshold
        above ever fires -- the bounded Move simply runs out and the robot
        sits still until `ProgressCheck`'s 2 s liveness backstop notices
        (measured 2026-07-31 in the sim reproduction: repeated ~0.4 s of
        motion followed by ~2 s stopped, all the way down the first leg).
        A move must be refreshed BEFORE it expires, not after; 0.5 leaves
        a full half of the commanded arc as margin for the replacement's
        own ack round trip.
    """

    omegaThreshold: float = 0.05        # [rad/s]
    arcLengthThreshold: float = 15.0    # [mm]
    refreshFraction: float = 0.5


@dataclass(frozen=True)
class GiveUpLimits:
    """Give-up policy for one `gotoWorld()`/`gotoRobot()` call -- an
    EXPLICIT budget, not an infinite null-the-error loop (design issue T5
    / sprint.md SUC-006). Ordinary loop-pacing knobs -- ticket 007 does
    not measure or tune these (it measures/tunes `TERMINATION_TOLERANCE`
    only).

    maxIterations: hard cap on throttled solve cycles.
    giveUpTimeout: [s] hard wall-clock cap, independent of iteration
        count (protects a live hardware/sim session against an
        unexpectedly slow outer-loop rate).
    """

    maxIterations: int = 200
    giveUpTimeout: float = 30.0  # [s]


@dataclass(frozen=True)
class AckRetry:
    """Enqueue-ack verification/retry policy for `gotoWorld()`'s own
    `move_twist()` sends (`_sendVerifiedTwist()`, below) -- mirrors
    `square_tour.Tour.sendVerified()`'s own measured DAPLink/radio
    inbound-loss workaround (~20% of inbound command packets dropped by
    the USB->UART bridge in that script's own measurement), applied here
    to `gotoWorld()`'s send path, which previously discarded
    `move_twist()`'s returned `corr_id` entirely.

    maxAttempts: wire sends of ONE logical move before giving up on it
        (matches `sendVerified()`'s own 4).
    ackTimeout: [s] per-attempt ack deadline (matches `sendVerified()`'s
        own 0.5s-per-attempt window).
    """

    maxAttempts: int = 4
    ackTimeout: float = 0.5  # [s]


@dataclass(frozen=True)
class ProgressCheck:
    """Liveness backstop independent of ack outcome -- forces a re-send
    even when `_shouldReplace()` says "unchanged" if the robot's own
    world pose has not advanced past `threshold` for `window` seconds.
    Needed because `_shouldReplace()` only ever compares the newest solve
    against the last SENT solution: with the robot genuinely stationary
    (a stalled/deadband move, or an ack that was lost despite
    `AckRetry`'s own retries) the solve keeps returning close to the same
    solution forever, so the throttle alone would never re-send -- this
    is exactly the OOP ticket's own measured failure (264 solve
    iterations, 1 send, 0 motion, 30s give-up).

    window: [s] how long the pose may sit within `threshold` before this
        loop force-resends regardless of the throttle. 2.0s is several
        multiples of the ~150ms actuation delay
        (`TERMINATION_TOLERANCE`'s own docstring) and matches
        `AckRetry`'s OWN worst-case single-send budget (4 attempts *
        0.5s ackTimeout = 2.0s) -- long enough that one send's own
        internal ack retries never themselves look like a stall, short
        enough to catch a genuinely stuck robot within a handful of
        cycles rather than riding out the whole `GiveUpLimits.
        giveUpTimeout`.
    threshold: [mm] minimum world-frame displacement inside `window` to
        count as "made progress" -- set a small margin above
        `ReplaceThreshold.arcLengthThreshold`'s own 15 mm noise floor.
    """

    window: float = 2.0     # [s]
    threshold: float = 20.0  # [mm]


@dataclass(frozen=True)
class GotoResult:
    """One `gotoWorld()`/`gotoRobot()` call's outcome.

    success: True iff the robot arrived within `tolerance` of the target.
    reason: a human-readable explanation -- always set, on EVERY outcome
        (arrival, give-up, or target-behind), never blank.
    finalPose: the last known world-frame `Pose`, or None if no
        telemetry was ever ingested during the call.
    iterations: total solve cycles run.
    sent: total LOGICAL `move_twist()` replacements decided upon (<=
        iterations, since throttling suppresses most of them once the
        solution has converged) -- each one may itself have taken more
        than one wire attempt; see `retries` below.
    retries: total EXTRA wire `move_twist()` attempts beyond each logical
        send's own first attempt, summed across the whole call (`AckRetry`
        -- visibility into ack loss on the link, not solver behavior).
    forcedResends: how many of `sent` were triggered by `ProgressCheck`'s
        liveness backstop rather than `_shouldReplace()`'s own throttle
        -- a nonzero count means the robot sat still long enough to force
        a fresh `Move.id` at least once.
    unacked: how many of `sent` NEVER received a clean ack despite
        `AckRetry.maxAttempts` tries -- a nonzero count on a give-up
        result means the link/command channel was the problem, not the
        solver.
    """

    success: bool
    reason: str
    finalPose: "Pose | None"
    iterations: int
    sent: int
    retries: int = 0
    forcedResends: int = 0
    unacked: int = 0


class MoveIdAllocator:
    """Strictly monotonic `Move.id` source for one planner session.

    Never emits `0`: `Move.id == 0` is the firmware's dedup-EXEMPT
    sentinel (ticket 002's own verification contract, "id-0 exemption")
    -- a move sent with id 0 is NEVER deduped, which is exactly wrong for
    a loop that continuously replaces the in-flight Move, since an id-0
    replacement whose enqueue ack is lost and retried could double-
    execute instead of being caught by the dedup ring. Starting at 1 (or
    any caller-chosen positive value) keeps every replacement inside the
    dedup-protected id space.

    Pure, no I/O -- unit-testable standalone. A single instance must be
    shared across every `gotoWorld()`/`gotoRobot()` call in one robot
    session that issues more than one goto in sequence (e.g. a
    multi-waypoint tour, ticket 007/008): the DEFAULT behaviour when a
    caller omits `moveIds` is a fresh, call-scoped allocator restarting
    at 1, which is only safe for a single isolated goto call across the
    whole boot lifetime of the robot's dedup ring -- reusing a fresh
    allocator across multiple sequential calls in the SAME boot session
    risks a later, genuinely-new move being misread as a duplicate of an
    earlier one that reused the same low id.
    """

    def __init__(self, start: int = 1) -> None:
        if start <= 0:
            raise ValueError(f"MoveIdAllocator: start must be > 0 (id 0 is the dedup-exempt "
                              f"sentinel), got {start!r}")
        self._next = start

    def next(self) -> int:
        moveId = self._next
        self._next += 1
        return moveId


def _shouldReplace(lastSent: "ArcSolution | None", candidate: ArcSolution,
                    omegaThreshold: float, arcLengthThreshold: float) -> bool:
    """True when `candidate` has moved materially from `lastSent` -- the
    throttle decision (design issue T5: "re-solve every frame, but only
    send when the solution has moved materially"). Always True for the
    very first solve (`lastSent is None`, nothing sent yet). Pure -- no
    I/O, unit-testable with synthetic `ArcSolution`s."""
    if lastSent is None:
        return True
    if abs(candidate.omega - lastSent.omega) > omegaThreshold:
        return True
    if abs(candidate.arcLength - lastSent.arcLength) > arcLengthThreshold:
        return True
    return False


def _giveUpReason(iterations: int, elapsed: float, limits: GiveUpLimits) -> "str | None":
    """Pure give-up check -- no I/O. Returns an explicit reason string once
    either budget (`GiveUpLimits.maxIterations`/`giveUpTimeout`) is
    exhausted, else None. Never a silent infinite retry -- the caller
    always gets a stated reason."""
    if iterations >= limits.maxIterations:
        return (f"gave up after {iterations} solve iterations "
                f"(limit {limits.maxIterations}) without reaching the target")
    if elapsed >= limits.giveUpTimeout:
        return (f"gave up after {elapsed:.1f}s (limit {limits.giveUpTimeout:.1f}s) "
                f"without reaching the target")
    return None


def _targetBehindReason(bearing: float) -> str:
    """The explicit give-up message for the solver's target-behind guard
    (`ArcSolution.stop=True` with a nonzero `bearing`) -- pure, no I/O,
    unit-testable standalone. This loop treats a behind target as a
    GIVE-UP, not a cue to turn in place: see this module's own docstring
    for why (terminal-theta / in-place-turn honoring is sprint.md's own
    Out of Scope)."""
    return (f"target is behind the robot's current heading "
            f"(bearing={math.degrees(bearing):+.1f} deg from straight ahead) -- "
            f"in-place reorientation is out of scope for this loop "
            f"(sprint 127 Out of Scope: terminal-theta honoring in the solver)")


def _readFrames(proto: NezhaProtocol) -> "list":
    """Drain whatever telemetry frames are currently queued -- dispatches
    to whichever shape `proto`'s own connection exposes, making
    `gotoWorld()` genuinely backend-agnostic (this ticket's own
    instruction: "the same NezhaProtocol interface works against both"
    hardware and sim).

    `NezhaProtocol.read_pending_binary_tlm_frames()` (the ordinary,
    hardware-path method) assumes `self._conn.drain_binary_tlm()`
    (`SerialConnection`'s own raw-pb2 drain, which THAT method then
    adapts into `TLMFrame`s) -- `SimConfigConn` has no
    `drain_binary_tlm()`, since `SimLoop.read_pending_binary_tlm_frames()`
    already returns adapted `TLMFrame`s directly. `square_tour.
    SimBackend.advance()` already works around this exact asymmetry by
    calling `self._sim.read_pending_binary_tlm_frames()` on the raw
    `SimLoop`, bypassing `NezhaProtocol` entirely -- this helper makes
    that dispatch `gotoWorld()`'s own problem instead of pushing the same
    workaround onto every future caller: prefer the CONNECTION's own
    `read_pending_binary_tlm_frames()` when it has one (`SimConfigConn`),
    falling back to the ordinary protocol-level method otherwise
    (`SerialConnection`, via `NezhaProtocol`'s own adapter)."""
    conn = proto._conn  # noqa: SLF001 -- no public accessor; see this function's own docstring
    readFromConn = getattr(conn, "read_pending_binary_tlm_frames", None)
    if readFromConn is not None:
        return readFromConn()
    return proto.read_pending_binary_tlm_frames()


def _advance(proto: NezhaProtocol, seconds: float,
             geofence: "Geofence | None") -> "list":
    """Drain telemetry for `seconds` of wall time, checking the geofence
    on the SAME timebase the robot is moving on -- never between
    segments (`.claude/rules/playfield-testing.md` "never drive blind").
    Mirrors `otos_calibration_bench.drainFrames()`'s idiom (this module's
    own docstring explains why it is reimplemented here rather than
    imported). Raises `GeofenceViolation` if the geofence fires --
    `estop()` has already been sent internally by `Geofence._halt()` by
    the time that happens; the caller's own `finally` block sends it
    again (idempotent) and lets the exception propagate."""
    frames = []
    deadline = time.monotonic() + seconds
    nextCheck = 0.0
    while time.monotonic() < deadline:
        frames.extend(_readFrames(proto))
        now = time.monotonic()
        if geofence is not None and now >= nextCheck:
            geofence.check()  # raises GeofenceViolation; estop() already sent internally
            nextCheck = now + _GEOFENCE_CHECK_PERIOD
        time.sleep(_POLL_SLEEP)
    return frames


def _recordAcks(frames: "list", ackSeen: "dict[int, object]") -> None:
    """Fold every frame's bounded ack ring into `ackSeen` (corr_id ->
    `AckEntry`) -- shared by `gotoWorld()`'s own per-cycle drain and
    `_sendVerifiedTwist()`'s own retry-wait drain, mirroring
    `square_tour.Tour.record()`'s identical fold (that one keyed by both
    corr_id and Move.id, since it also awaits completion acks; this
    module never does, so corr_id is the only key space it needs)."""
    for frame in frames:
        for ack in frame.acks:
            ackSeen[ack.corr_id] = ack


def _sendVerifiedTwist(proto: NezhaProtocol, worldPose: WorldPose,
                       geofence: "Geofence | None", ackSeen: "dict[int, object]",
                       *, moveId: int, kwargs: dict, cyclePeriod: float,
                       ackRetry: AckRetry) -> "tuple[bool, int]":
    """Send one `move_twist(..., move_id=moveId, **kwargs)`, verifying the
    ENQUEUE ack and retrying on loss -- mirrors `square_tour.Tour.
    sendVerified()`'s own approach (this module's own OOP-fix docstring
    instruction): matches on the envelope's own `corr_id` (what
    `move_twist()` returns -- NOT `moveId`, which the firmware only
    echoes on the separate COMPLETION ack) by scanning frames THIS
    function's own `_advance()` calls drain, never a second
    `wait_for_ack()` read -- `sendVerified()`'s own docstring: "on a
    single-consumer telemetry queue two readers starve each other."
    Ingests every drained frame into `worldPose` exactly like the main
    loop does, so a multi-attempt wait never starves pose tracking.

    Every retry attempt of this call reuses the SAME `moveId` -- deliberate,
    not an oversight. `RobotLoop::handleMove()`'s own comment
    (`src/firm/app/robot_loop.cpp`) documents exactly this shape: "a
    retried enqueue whose original ack was lost carries this same id
    under a fresh corr_id... ack it as success... skips [estop()]...
    precedes any replace handling, so a duplicate cannot restart a move
    mid-flight." The firmware's dedup ring (`alreadyAccepted()`) exists
    precisely so a retried SEND of the SAME intended move is idempotent:
    if the original enqueue actually landed and only its ack was lost on
    the way back, a same-id retry gets silently re-acked OK without
    restarting a move already in flight. A NEW id per retry would defeat
    that protection -- the firmware would see a genuinely new move and
    restart it, mid-flight, on every lost ack. Contrast this with
    `gotoWorld()`'s own call site, which allocates a FRESH id every time
    it decides to (re-)send AT ALL (including a `ProgressCheck`-forced
    resend of an apparently-unchanged solution) -- see that call site's
    own comment for why a fresh id is correct there.

    Treats `ERR_FULL` (queue transiently full) as retryable. Any OTHER
    nonzero `err_code` (e.g. `ERR_BADARG`) is a caller bug a retry cannot
    fix -- raises `MoveRejected` immediately rather than burning the rest
    of `ackRetry.maxAttempts` on a command that will never succeed.

    Returns `(acked, attempts)`: `acked` is True iff a clean
    (`err_code == 0`) ack was observed within `ackRetry.maxAttempts`
    tries; `attempts` is how many `move_twist()` calls were actually made
    (1..`maxAttempts`) -- the caller derives `attempts - 1` as this one
    send's own RETRY count for `GotoResult.retries`.
    """
    for attempt in range(1, ackRetry.maxAttempts + 1):
        corr = proto.move_twist(move_id=moveId, **kwargs)
        waited = 0.0
        entry = None
        while waited < ackRetry.ackTimeout:
            frames = _advance(proto, cyclePeriod, geofence)
            _recordAcks(frames, ackSeen)
            for frame in frames:
                worldPose.ingest(frame)
            waited += cyclePeriod
            entry = ackSeen.get(corr)
            if entry is not None:
                break
        if entry is None:
            continue  # no ack at all within this attempt's window -- retry
        if entry.ok:
            return True, attempt
        if entry.err_code != _ERR_FULL:
            raise MoveRejected(
                f"move_twist() (move_id={moveId}, corr_id={corr}) was NACKed with "
                f"a non-retryable ErrCode {entry.err_code} (not ERR_FULL) -- "
                f"retrying would not help")
        # ERR_FULL -- fall through and retry with a fresh corr_id, same moveId.
    return False, ackRetry.maxAttempts


def gotoWorld(proto: NezhaProtocol, worldPose: WorldPose, x: float, y: float,
              theta: "float | None" = None, *, limits: SolverLimits,
              geofence: "Geofence | None" = None,
              tolerance: float = TERMINATION_TOLERANCE,
              giveUp: GiveUpLimits = GiveUpLimits(),
              throttle: ReplaceThreshold = ReplaceThreshold(),
              ackRetry: AckRetry = AckRetry(),
              progress: ProgressCheck = ProgressCheck(),
              moveTimeout: "float | None" = None,
              cyclePeriod: float = CYCLE_PERIOD,
              moveIds: "MoveIdAllocator | None" = None) -> GotoResult:
    """Drive the robot to a WORLD-frame target `(x, y)` -- the ONE control
    loop this module implements (`gotoRobot()` composes through this
    function rather than reimplementing it).

    x/y: [cm] world/camera-frame target position -- `nav.pose.Pose`'s own
        convention, matching `WorldPose`/camera-fix coordinates
        throughout this package.
    theta: [rad] ACCEPTED for a future holonomic drivetrain's benefit,
        forwarded into the target `Pose` -- IGNORED by this
        differential-drive loop, exactly as `solveArcToPoint()` itself
        documents ignoring `targetPoint.heading` (sprint.md Out of
        Scope: "Terminal-theta honoring in the solver").
    limits: the solver's physical/safety limits (trackWidth, speed, the
        curvature slew limit, the target-behind angle) -- required,
        robot-specific, no sensible default.
    geofence: checked inside `_advance()`'s own ~10 Hz time-advance
        primitive on every cycle -- None disables it (e.g. a sim run
        with no camera).
    tolerance: [mm] arrival tolerance -- see `TERMINATION_TOLERANCE`'s
        own module-level docstring for why it defaults to a PROVISIONAL
        value.
    giveUp: the give-up policy (`GiveUpLimits`) -- iteration and
        wall-clock caps.
    throttle: the replacement-throttle policy (`ReplaceThreshold`).
    ackRetry: the enqueue-ack verification/retry policy (`AckRetry`) --
        see `_sendVerifiedTwist()`'s own docstring for the full mechanism
        and the move-id-reuse rationale.
    progress: the liveness backstop (`ProgressCheck`) that forces a
        re-send when the pose has not moved for too long, independent of
        both the throttle and the ack outcome -- see its own docstring.
    moveTimeout: [ms] each issued `move_twist()`'s own safety backstop.
        `None` (the default) derives it per-move from that move's own
        `arcLength` and `limits.speed` (`_moveTimeoutFor()`) -- a fixed
        value here overrides that derivation for every move this call
        sends.
    cyclePeriod: [s] this loop's own re-solve cadence / `_advance()`
        window.
    moveIds: the `Move.id` source -- see `MoveIdAllocator`'s own
        docstring for why a caller issuing MULTIPLE sequential goto
        calls in one robot session MUST share one instance across all
        of them; the default creates a fresh, call-scoped allocator only
        safe for a single isolated call.

    Returns a `GotoResult` -- never raises for an ordinary give-up
    (target-behind, iteration cap, or timeout); DOES let a
    `GeofenceViolation` (or any other exception `_advance()`/the wire
    layer raises), or a `MoveRejected` (a non-retryable NACK -- see
    `_sendVerifiedTwist()`), propagate after this function's own
    `finally` block has sent `estop()` -- "a halt that raises must not be
    swallowed."
    """
    target = Pose(x=x, y=y, heading=theta if theta is not None else 0.0)
    allocator = moveIds if moveIds is not None else MoveIdAllocator()

    startTime = time.monotonic()
    iterations = 0
    sentCount = 0
    retryCount = 0
    forcedResendCount = 0
    unackedCount = 0
    sentSolution: "ArcSolution | None" = None
    sentOmega = 0.0
    ackSeen: "dict[int, object]" = {}
    # ProgressCheck bookkeeping: the pose/time this loop last saw the
    # robot make material progress toward the target. Seeded from the
    # FIRST pose this loop ever sees (below), not from `None`/time-zero,
    # so the window starts counting from when navigation actually begins,
    # not from before the first telemetry frame arrives.
    progressPose: "Pose | None" = None
    progressCheckTime = time.monotonic()

    try:
        while True:
            frames = _advance(proto, cyclePeriod, geofence)
            _recordAcks(frames, ackSeen)
            for frame in frames:
                worldPose.ingest(frame)
            currentPose = worldPose.worldPose()

            distance = None
            if currentPose is not None:
                distance = math.hypot(target.x - currentPose.x,
                                      target.y - currentPose.y) * _POSITION_SCALE
                if distance <= tolerance:
                    return GotoResult(
                        success=True,
                        reason=f"arrived within {tolerance:.0f} mm (distance={distance:.1f} mm)",
                        finalPose=currentPose, iterations=iterations, sent=sentCount,
                        retries=retryCount, forcedResends=forcedResendCount, unacked=unackedCount)

            iterations += 1
            elapsed = time.monotonic() - startTime
            giveUpReason = _giveUpReason(iterations, elapsed, giveUp)
            if giveUpReason is not None:
                detail = ("no telemetry received" if currentPose is None
                          else f"closest distance reached: {distance:.1f} mm")
                # Distinguish "the link was the problem" from "the robot
                # moved but never converged" -- the OOP ticket's own
                # complaint: an unqualified give-up reason sent the
                # stakeholder hunting the solver when the solver was
                # innocent and the commands simply never landed.
                if unackedCount > 0:
                    detail += (f"; {unackedCount} of {sentCount} move(s) never received a "
                              f"clean ack despite {ackRetry.maxAttempts} attempts each -- "
                              f"link/command loss, not a solver problem")
                elif sentCount > 0:
                    detail += (f"; all {sentCount} move(s) acked ok "
                              f"({forcedResendCount} forced by the progress-stall backstop) -- "
                              f"the robot moved but did not converge within tolerance")
                else:
                    detail += "; no move was ever sent"
                return GotoResult(success=False, reason=f"{giveUpReason} ({detail})",
                                  finalPose=currentPose, iterations=iterations, sent=sentCount,
                                  retries=retryCount, forcedResends=forcedResendCount,
                                  unacked=unackedCount)

            if currentPose is None:
                continue  # no pose yet -- nothing to solve against

            # ProgressCheck: has the pose moved past `progress.threshold`
            # since the last checkpoint? If so, reset the checkpoint --
            # the robot is making progress and the throttle should be
            # left alone. If not, and `progress.window` has elapsed,
            # force a resend below regardless of the throttle.
            if progressPose is None:
                progressPose = currentPose
                progressCheckTime = time.monotonic()
                forceResend = False
            else:
                moved = math.hypot(currentPose.x - progressPose.x,
                                   currentPose.y - progressPose.y) * _POSITION_SCALE
                if moved > progress.threshold:
                    progressPose = currentPose
                    progressCheckTime = time.monotonic()
                    forceResend = False
                else:
                    forceResend = (time.monotonic() - progressCheckTime) >= progress.window

            solution = solveArcToPoint(currentPose, target, limits, previousOmega=sentOmega)
            if solution.stop:
                return GotoResult(success=False, reason=_targetBehindReason(solution.bearing),
                                  finalPose=currentPose, iterations=iterations, sent=sentCount,
                                  retries=retryCount, forcedResends=forcedResendCount,
                                  unacked=unackedCount)

            if _shouldReplace(sentSolution, solution, throttle.omegaThreshold,
                              throttle.arcLengthThreshold) or forceResend:
                # Every logical send here -- whether from the ordinary
                # throttle or a ProgressCheck-forced resend -- draws a
                # FRESH Move.id. A forced resend in particular needs the
                # firmware to actually restart the move via
                # planner_.move(), not be silently re-acked as a no-op
                # duplicate of a move already believed in flight -- see
                # _sendVerifiedTwist()'s own docstring for the contrasting
                # case (an ack-loss RETRY of one send, which DOES reuse
                # its id).
                moveId = allocator.next()
                effectiveTimeout = (moveTimeout if moveTimeout is not None
                                    else _moveTimeoutFor(solution.arcLength, limits.speed))
                acked, attempts = _sendVerifiedTwist(
                    proto, worldPose, geofence, ackSeen,
                    moveId=moveId,
                    kwargs=dict(v_x=solution.v_x, v_y=0.0, omega=solution.omega,
                               stop_distance=solution.arcLength, timeout=effectiveTimeout,
                               replace=True),
                    cyclePeriod=cyclePeriod, ackRetry=ackRetry)
                sentSolution = solution
                sentOmega = solution.omega
                sentCount += 1
                retryCount += attempts - 1
                if forceResend:
                    forcedResendCount += 1
                if not acked:
                    unackedCount += 1
                # Whether or not this send was acked, give it a full
                # progress window before judging the robot stalled again
                # -- otherwise an unacked-but-not-yet-window-expired send
                # would force ANOTHER resend on literally the next cycle.
                progressPose = currentPose
                progressCheckTime = time.monotonic()
    finally:
        # estop(), never the PLANNED stop() -- every halt path in this
        # module funnels through this one call site. Runs on every return
        # above AND on any exception (e.g. GeofenceViolation out of
        # _advance(), or MoveRejected out of _sendVerifiedTwist()); the
        # exception is not caught here, so it still propagates after this
        # line -- a halt that raises must not be swallowed.
        proto.estop()


def gotoRobot(proto: NezhaProtocol, worldPose: WorldPose, x: float, y: float,
              theta: "float | None" = None, *, limits: SolverLimits,
              geofence: "Geofence | None" = None,
              tolerance: float = TERMINATION_TOLERANCE,
              giveUp: GiveUpLimits = GiveUpLimits(),
              throttle: ReplaceThreshold = ReplaceThreshold(),
              ackRetry: AckRetry = AckRetry(),
              progress: ProgressCheck = ProgressCheck(),
              moveTimeout: "float | None" = None,
              cyclePeriod: float = CYCLE_PERIOD,
              moveIds: "MoveIdAllocator | None" = None) -> GotoResult:
    """Drive the robot to a ROBOT-frame target `(x, y)` -- a THIN
    composition through `gotoWorld()`, sharing the SAME kwargs contract
    (see that function's own docstring for every parameter's meaning; not
    re-derived here). This function contains no loop, no solve call, and
    no `move_twist()` call of its own -- `gotoWorld()` is the ONE control
    loop implementation (sprint.md SUC-006's own acceptance criterion).

    x/y: [cm] target position in the ROBOT's OWN current body frame
        (forward = +x, left = +y -- `Pose.heading`'s CCW-positive
        convention, the same body-frame convention
        `solveArcToPoint()`'s own bearing uses).

    Converts the robot-frame offset into a world-frame target via the
    CURRENT world pose (`WorldPose.worldPose()`) treated as a transform
    from the robot's own current frame into world frame -- the same
    "a world pose IS a transform from a local frame" relationship
    `WorldPose` itself embodies for `T_world_from_odom`
    (`pathplan.world_pose.Transform2`, reused here directly, not
    reimplemented).

    Raises `RuntimeError` if `worldPose` has no current pose yet (no
    telemetry frame has been `ingest()`-ed) -- there is no current pose
    to compose the robot-frame offset against.
    """
    currentPose = worldPose.worldPose()
    if currentPose is None:
        raise RuntimeError(
            "gotoRobot(): WorldPose has no current pose yet (ingest() a telemetry "
            "frame first) -- cannot compose a robot-frame target without a world "
            "pose to compose it against")
    toWorld = Transform2(x=currentPose.x, y=currentPose.y, rotation=currentPose.heading)
    worldTarget = toWorld.apply(Pose(x=x, y=y, heading=theta if theta is not None else 0.0))
    # theta (if given at all) is composed into the world frame the same way
    # x/y are -- forwarding the RAW, un-composed theta would silently mean
    # something different ("this world heading") than what the caller
    # asked for ("this heading relative to the robot's own current
    # heading"). Still entirely ignored downstream by this differential
    # solver either way (see gotoWorld()'s own docstring) -- composed here
    # only so a future holonomic-drivetrain consumer sees the correct
    # value, not a coincidentally-similar-looking wrong one.
    composedTheta = None if theta is None else worldTarget.heading
    return gotoWorld(proto, worldPose, worldTarget.x, worldTarget.y, composedTheta,
                     limits=limits, geofence=geofence, tolerance=tolerance,
                     giveUp=giveUp, throttle=throttle, ackRetry=ackRetry, progress=progress,
                     moveTimeout=moveTimeout, cyclePeriod=cyclePeriod, moveIds=moveIds)


@dataclass(frozen=True)
class FollowPathResult:
    """One `followPath()` call's outcome -- the multi-waypoint analogue of
    `GotoResult`. Every field shares `GotoResult`'s own meaning (see that
    dataclass's docstring); only the one addition is documented here.

    waypointsReached: how many of the call's `waypoints` the robot reached
        (advanced onto or past) before this call returned -- always
        `len(waypoints)` on a successful (`success=True`) return, since
        arrival at the terminal waypoint implies every interior one was
        already passed; fewer on a give-up partway through the path.
    """

    success: bool
    reason: str
    finalPose: "Pose | None"
    iterations: int
    sent: int
    waypointsReached: int
    retries: int = 0
    forcedResends: int = 0
    unacked: int = 0


# --- Off-path abandonment (out-of-process, 2026-07-31) --------------------
#
# `followPath()` gives up, rather than driving, once it can no longer solve
# an arc to its own lookahead target for this long. Sized off `ProgressCheck.
# window`'s own reasoning: long enough that a transient mid-corner guard
# firing (the legitimate case -- pure pursuit cuts inside a fillet, so the
# heading can briefly lag the nominal tangent) rides out without aborting a
# healthy run, short enough that a robot which has genuinely left the path
# is stopped in about a second rather than driven blind for the rest of the
# give-up budget. Cycle-count, not seconds, so it scales with `cyclePeriod`.
_MAX_UNSOLVABLE_CYCLES = 10

# The Move's Distance stop condition is the arc to the lookahead target and
# NOTHING LONGER -- deliberately, and it is a safety property, not an
# oversight. Pure pursuit produces a CURVATURE, not a destination, so it is
# tempting to give the Move a generous distance and let the loop replace it;
# do not. Whenever the loop cannot get a replacement through (the ack
# retries on a lossy link cost up to 2 s), the robot runs whatever it was
# last told, open loop, for exactly that distance. Keeping it at one
# lookahead means the robot COASTS TO A STOP roughly where the last valid
# solve expected it to be, instead of continuing to accumulate path error
# while the host is busy retrying. MEASURED 2026-07-31 (leg 250, 10%
# injected command loss, 6 seeds): inflating the commanded distance to
# `max(arcLength, speed * 0.6 s)` -- barely longer than one lookahead --
# dropped the completion rate from 4/6 to 2/6, because every retry window
# became 90 mm of un-steered travel instead of a stop.
#
# `ReplaceThreshold.refreshFraction` is what keeps this from stalling in
# NORMAL operation: the move is replaced after half its own arc, so it never
# expires under a healthy link.


def followPath(proto: NezhaProtocol, worldPose: WorldPose, waypoints: "list[Pose]",
               limits: SolverLimits, *, geofence: "Geofence | None" = None,
               tolerance: float = TERMINATION_TOLERANCE,
               giveUp: GiveUpLimits = GiveUpLimits(),
               throttle: ReplaceThreshold = ReplaceThreshold(),
               ackRetry: AckRetry = AckRetry(),
               progress: ProgressCheck = ProgressCheck(),
               moveTimeout: "float | None" = None,
               cyclePeriod: float = CYCLE_PERIOD,
               moveIds: "MoveIdAllocator | None" = None,
               lookahead: "float | None" = None,
               onWaypoint: "Callable[[int, Pose], None] | None" = None) -> FollowPathResult:
    """Drive the robot through a SEQUENCE of world-frame waypoints as ONE
    continuous pure-pursuit run, instead of calling `gotoWorld()` once per
    waypoint and requiring a full stop-and-arrive at every one. See the
    module-level comment above `solver.pursuitTarget()` for why a dense
    waypoint sequence (e.g. a rounded-corner square) needs this instead of
    N separate `gotoWorld()` calls -- the arrival-tolerance floor vs.
    fillet-chord-length ceiling conflict, the stakeholder's own rejection
    of trading speed to shrink the floor (2026-07-30), and the playfield
    runaway the first (pass-predicate) answer to it produced.

    No waypoint is ever a steering target. Each cycle,
    `solver.pursuitTarget()` projects the robot onto the POLYLINE through
    `waypoints` and returns the point one `lookahead` (`_lookaheadFor()`,
    or the caller's own override) farther along it -- the lookahead-circle
    intersection. Waypoints are therefore only vertices of the path being
    followed, never things to arrive at, which is what dissolves the
    arrival-tolerance-floor-vs-chord-spacing conflict outright. Only the
    LAST waypoint gets a real, tolerance-gated arrival test (`tolerance`,
    the same `TERMINATION_TOLERANCE`-shaped parameter `gotoWorld()` itself
    uses) -- this loop has to stop somewhere.

    The projection is MONOTONE: the segment it landed on is carried into
    the next cycle as the search's own starting point, so a closed path (a
    square, whose terminal waypoints sit right next to its first ones) can
    never snap the robot back onto a stretch it has already driven.

    Per-cycle ordering is INGEST POSE -> PICK TARGET -> SOLVE. Because the
    target is a lookahead distance along the path rather than the next
    waypoint, `solveArcToPoint()`'s own behind-guard firing is no longer a
    routine mid-corner event -- it now means the robot's heading is more
    than `SolverLimits.behindAngle` off the direction the path goes, i.e.
    it has genuinely left the path. This loop therefore sends NOTHING on
    such a cycle and, if it cannot solve for `_MAX_UNSOLVABLE_CYCLES`
    consecutive cycles, ABANDONS the path (an unsuccessful
    `FollowPathResult`, with the `finally` block's `estop()` bringing the
    robot to rest). It specifically does NOT re-send the last known-good
    arc: that is an open-loop command issued exactly when the closed loop
    has said it does not know where to go, and it is what turned a stall
    into a 920 mm straight-line runaway on the playfield -- see rejected
    approach 3 in the module-level comment above this function.

    `onWaypoint(index, pose)`, if given, is called once for every DISTINCT
    waypoint index the path projection advances past (increasing order,
    possibly several per cycle), and once more for the terminal waypoint on
    a successful arrival. Lets a caller (e.g. `square_tour.runGotoTour()`)
    hang a camera fix or a chart mark off waypoint crossings without this
    loop needing to know anything about cameras or charts itself. `pose` is
    the world pose at the moment the crossing was DETECTED -- the robot is
    still moving at that instant (path following never stops at an interior
    vertex), so a camera fix taken from this hook is a MOVING fix, not the
    "at REST" boundary fix `.claude/rules/playfield-testing.md` mandates
    for a geofence-armed run; reconciling that is left to whoever runs this
    on the playfield next.

    All other parameters, the `estop()`-on-every-exit contract (this
    function's own `finally` block, run on every return AND on any
    exception), and the enqueue-ack-retry / liveness-backstop machinery
    match `gotoWorld()`'s own -- see that function's docstring for the full
    description; not re-derived here.
    """
    if not waypoints:
        raise ValueError("followPath(): waypoints must be non-empty")
    lastIndex = len(waypoints) - 1
    carrot = lookahead if lookahead is not None else _lookaheadFor(limits.speed)
    allocator = moveIds if moveIds is not None else MoveIdAllocator()

    startTime = time.monotonic()
    iterations = 0
    sentCount = 0
    retryCount = 0
    forcedResendCount = 0
    unackedCount = 0
    sentSolution: "ArcSolution | None" = None
    sentOmega = 0.0
    sentPose: "Pose | None" = None   # world pose at the last send (refresh bookkeeping)
    sentDistance = 0.0               # [mm] Distance stop condition of the last send
    ackSeen: "dict[int, object]" = {}
    progressPose: "Pose | None" = None
    progressCheckTime = time.monotonic()
    segment = 0  # monotone path-projection segment, carried across cycles
    unsolvableCycles = 0
    reachedIndex = -1  # highest waypoint index onWaypoint() has already been told about

    def notifyUpTo(uptoIndexInclusive: int, pose: Pose) -> None:
        nonlocal reachedIndex
        if onWaypoint is None:
            reachedIndex = max(reachedIndex, uptoIndexInclusive)
            return
        while reachedIndex < uptoIndexInclusive:
            reachedIndex += 1
            onWaypoint(reachedIndex, pose)

    try:
        while True:
            frames = _advance(proto, cyclePeriod, geofence)
            _recordAcks(frames, ackSeen)
            for frame in frames:
                worldPose.ingest(frame)
            currentPose = worldPose.worldPose()

            if currentPose is None:
                iterations += 1
                giveUpReason = _giveUpReason(iterations, time.monotonic() - startTime, giveUp)
                if giveUpReason is not None:
                    return FollowPathResult(
                        success=False, reason=f"{giveUpReason} (no telemetry received)",
                        finalPose=None, iterations=iterations, sent=sentCount,
                        waypointsReached=reachedIndex + 1, retries=retryCount,
                        forcedResends=forcedResendCount, unacked=unackedCount)
                continue

            # PICK TARGET before SOLVE (ordering guarantee -- see docstring).
            pursuit = pursuitTarget(currentPose, waypoints, segment, carrot)
            segment = pursuit.segment
            # The projection sitting on segment N means waypoints[0..N-1]
            # are behind it -- notify those, and only those.
            if segment > 0:
                notifyUpTo(segment - 1, currentPose)

            target = pursuit.point
            terminal = waypoints[lastIndex]
            distance = math.hypot(terminal.x - currentPose.x,
                                  terminal.y - currentPose.y) * _POSITION_SCALE
            # Gate arrival on being ON the last segment as well as near the
            # terminal waypoint: a CLOSED path's terminal waypoint sits right
            # next to its own start, so a distance-only test would "arrive"
            # before the robot had driven anything at all.
            #
            # "Near the terminal waypoint" is EITHER within `tolerance` of it
            # OR having run the path out (`remaining <= tolerance`) while
            # still tracking it (`crossTrack` within a lookahead). The second
            # clause is what makes the endgame robust: a follower steering at
            # a lookahead point necessarily arrives with some overshoot, and
            # a proximity-only test that the robot sails past leaves the
            # terminal waypoint BEHIND it -- which the behind-guard then
            # reports as "no arc", abandoning a path that was in fact
            # complete. MEASURED 2026-07-31: that alone accounted for every
            # "reached 15 of 17" give-up in the 10%-loss sweep.
            arrived = distance <= tolerance or (
                pursuit.remaining <= tolerance and pursuit.crossTrack <= max(tolerance, carrot))
            if segment >= lastIndex - 1 and arrived:
                notifyUpTo(lastIndex, currentPose)
                return FollowPathResult(
                    success=True,
                    reason=f"arrived within {tolerance:.0f} mm of the terminal waypoint "
                           f"(distance={distance:.1f} mm)",
                    finalPose=currentPose, iterations=iterations, sent=sentCount,
                    waypointsReached=reachedIndex + 1, retries=retryCount,
                    forcedResends=forcedResendCount, unacked=unackedCount)

            iterations += 1
            elapsed = time.monotonic() - startTime
            giveUpReason = _giveUpReason(iterations, elapsed, giveUp)
            if giveUpReason is not None:
                detail = (f"on path segment {segment}/{lastIndex - 1}; cross-track "
                          f"{pursuit.crossTrack:.1f} mm; {pursuit.remaining:.1f} mm of path left")
                if unackedCount > 0:
                    detail += (f"; {unackedCount} of {sentCount} move(s) never received a "
                              f"clean ack despite {ackRetry.maxAttempts} attempts each -- "
                              f"link/command loss, not a solver problem")
                elif sentCount > 0:
                    detail += (f"; all {sentCount} move(s) acked ok "
                              f"({forcedResendCount} forced by the progress-stall backstop)")
                else:
                    detail += "; no move was ever sent"
                return FollowPathResult(
                    success=False, reason=f"{giveUpReason} ({detail})",
                    finalPose=currentPose, iterations=iterations, sent=sentCount,
                    waypointsReached=reachedIndex + 1, retries=retryCount,
                    forcedResends=forcedResendCount, unacked=unackedCount)

            # ProgressCheck bookkeeping -- identical shape to gotoWorld()'s own.
            if progressPose is None:
                progressPose = currentPose
                progressCheckTime = time.monotonic()
                forceResend = False
            else:
                moved = math.hypot(currentPose.x - progressPose.x,
                                   currentPose.y - progressPose.y) * _POSITION_SCALE
                if moved > progress.threshold:
                    progressPose = currentPose
                    progressCheckTime = time.monotonic()
                    forceResend = False
                else:
                    forceResend = (time.monotonic() - progressCheckTime) >= progress.window

            solution = solveArcToPoint(currentPose, target, limits, previousOmega=sentOmega)
            # `target` is a point one lookahead distance ALONG THE PATH from
            # the robot's own projection, so the behind-guard firing here is
            # no longer a routine mid-corner event: it means the robot's
            # heading is more than `limits.behindAngle` off the direction the
            # path goes -- it has left the path, and no arc from here reaches
            # the path's own continuation.
            #
            # The response is to send NOTHING and, if that persists, to stop
            # following. It is emphatically NOT to re-send the last
            # known-good arc: that arc was solved for a target the robot has
            # since left behind, so re-sending it is an open-loop command
            # issued exactly when the closed loop has said it does not know
            # where to go. MEASURED 2026-07-31 (sim reproduction of the
            # playfield runaway): with the old stale-resend backstop, 743 of
            # 754 solve cycles hit this branch and every one of them drove
            # the robot further off the path in a straight line -- positive
            # feedback until the geofence. See rejected approach 3 in the
            # module-level comment above this function.
            if solution.stop:
                unsolvableCycles += 1
                if unsolvableCycles >= _MAX_UNSOLVABLE_CYCLES:
                    return FollowPathResult(
                        success=False,
                        reason=(f"abandoned the path: no arc to the lookahead target for "
                                f"{unsolvableCycles} consecutive cycles -- the robot's heading "
                                f"is {math.degrees(solution.bearing):+.1f} deg off the path "
                                f"direction and it is {pursuit.crossTrack:.1f} mm off the path "
                                f"(segment {segment}/{lastIndex - 1}). Halting rather than "
                                f"driving on a stale command"),
                        finalPose=currentPose, iterations=iterations, sent=sentCount,
                        waypointsReached=reachedIndex + 1, retries=retryCount,
                        forcedResends=forcedResendCount, unacked=unackedCount)
                continue
            unsolvableCycles = 0

            # The Move's own Distance stop condition -- see the block comment
            # above `_MAX_UNSOLVABLE_CYCLES`/the command-distance note for why
            # this is exactly one lookahead arc and never more. Never longer
            # than the path that is actually left, either.
            commandDistance = min(pursuit.remaining, solution.arcLength)

            # Refresh before the in-flight Move expires: on a straight run
            # both `omega` and `arcLength` are constant, so neither
            # `_shouldReplace()` threshold ever fires and the Move would
            # simply run out under the robot. See `ReplaceThreshold.
            # refreshFraction`.
            expiring = False
            if sentPose is not None and sentDistance > 0.0:
                covered = math.hypot(currentPose.x - sentPose.x,
                                     currentPose.y - sentPose.y) * _POSITION_SCALE
                expiring = covered >= sentDistance * throttle.refreshFraction

            if (_shouldReplace(sentSolution, solution, throttle.omegaThreshold,
                               throttle.arcLengthThreshold) or expiring or forceResend):
                moveId = allocator.next()
                effectiveTimeout = (moveTimeout if moveTimeout is not None
                                    else _moveTimeoutFor(commandDistance, limits.speed))
                acked, attempts = _sendVerifiedTwist(
                    proto, worldPose, geofence, ackSeen,
                    moveId=moveId,
                    kwargs=dict(v_x=solution.v_x, v_y=0.0, omega=solution.omega,
                               stop_distance=commandDistance, timeout=effectiveTimeout,
                               replace=True),
                    cyclePeriod=cyclePeriod, ackRetry=ackRetry)
                sentSolution = solution
                sentOmega = solution.omega
                sentPose = currentPose
                sentDistance = commandDistance
                sentCount += 1
                retryCount += attempts - 1
                if forceResend:
                    forcedResendCount += 1
                if not acked:
                    unackedCount += 1
                progressPose = currentPose
                progressCheckTime = time.monotonic()
    finally:
        # estop(), never the PLANNED stop() -- same contract as gotoWorld()'s
        # own finally block: runs on every return above AND on any
        # exception, and does not swallow it.
        proto.estop()
