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
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from robot_radio.field import Geofence
from robot_radio.nav.pose import Pose
from robot_radio.pathplan.solver import ArcSolution, SolverLimits, solveArcToPoint
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

MOVE_TIMEOUT = 3000.0    # [ms] move_twist()'s own safety backstop -- well above this loop's re-issue cadence, so a stalled host loop still self-stops the firmware
CYCLE_PERIOD = 0.1       # [s] outer loop pacing / time-advance window -- matches sprint.md's stated realistic ~10 Hz outer-loop rate and the geofence's own ~10 Hz check cadence
_GEOFENCE_CHECK_PERIOD = 0.1  # [s] matches the "~10 Hz" geofence cadence -- never between segments
_POLL_SLEEP = 0.01       # [s] telemetry poll granularity inside _advance()


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
    """

    omegaThreshold: float = 0.05        # [rad/s]
    arcLengthThreshold: float = 15.0    # [mm]


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
class GotoResult:
    """One `gotoWorld()`/`gotoRobot()` call's outcome.

    success: True iff the robot arrived within `tolerance` of the target.
    reason: a human-readable explanation -- always set, on EVERY outcome
        (arrival, give-up, or target-behind), never blank.
    finalPose: the last known world-frame `Pose`, or None if no
        telemetry was ever ingested during the call.
    iterations: total solve cycles run.
    sent: total `move_twist()` replacements actually issued (<=
        iterations, since throttling suppresses most of them once the
        solution has converged).
    """

    success: bool
    reason: str
    finalPose: "Pose | None"
    iterations: int
    sent: int


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


def gotoWorld(proto: NezhaProtocol, worldPose: WorldPose, x: float, y: float,
              theta: "float | None" = None, *, limits: SolverLimits,
              geofence: "Geofence | None" = None,
              tolerance: float = TERMINATION_TOLERANCE,
              giveUp: GiveUpLimits = GiveUpLimits(),
              throttle: ReplaceThreshold = ReplaceThreshold(),
              moveTimeout: float = MOVE_TIMEOUT,
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
    moveTimeout: [ms] each issued `move_twist()`'s own safety backstop.
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
    layer raises) propagate after this function's own `finally` block
    has sent `estop()` -- "a halt that raises must not be swallowed."
    """
    target = Pose(x=x, y=y, heading=theta if theta is not None else 0.0)
    allocator = moveIds if moveIds is not None else MoveIdAllocator()

    startTime = time.monotonic()
    iterations = 0
    sentCount = 0
    sentSolution: "ArcSolution | None" = None
    sentOmega = 0.0

    try:
        while True:
            frames = _advance(proto, cyclePeriod, geofence)
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
                        finalPose=currentPose, iterations=iterations, sent=sentCount)

            iterations += 1
            elapsed = time.monotonic() - startTime
            giveUpReason = _giveUpReason(iterations, elapsed, giveUp)
            if giveUpReason is not None:
                detail = ("no telemetry received" if currentPose is None
                          else f"closest distance reached: {distance:.1f} mm")
                return GotoResult(success=False, reason=f"{giveUpReason} ({detail})",
                                  finalPose=currentPose, iterations=iterations, sent=sentCount)

            if currentPose is None:
                continue  # no pose yet -- nothing to solve against

            solution = solveArcToPoint(currentPose, target, limits, previousOmega=sentOmega)
            if solution.stop:
                return GotoResult(success=False, reason=_targetBehindReason(solution.bearing),
                                  finalPose=currentPose, iterations=iterations, sent=sentCount)

            if _shouldReplace(sentSolution, solution, throttle.omegaThreshold,
                              throttle.arcLengthThreshold):
                moveId = allocator.next()
                proto.move_twist(v_x=solution.v_x, v_y=0.0, omega=solution.omega,
                                 stop_distance=solution.arcLength, timeout=moveTimeout,
                                 replace=True, move_id=moveId)
                sentSolution = solution
                sentOmega = solution.omega
                sentCount += 1
    finally:
        # estop(), never the PLANNED stop() -- every halt path in this
        # module funnels through this one call site. Runs on every return
        # above AND on any exception (e.g. GeofenceViolation out of
        # _advance()); the exception is not caught here, so it still
        # propagates after this line -- a halt that raises must not be
        # swallowed.
        proto.estop()


def gotoRobot(proto: NezhaProtocol, worldPose: WorldPose, x: float, y: float,
              theta: "float | None" = None, *, limits: SolverLimits,
              geofence: "Geofence | None" = None,
              tolerance: float = TERMINATION_TOLERANCE,
              giveUp: GiveUpLimits = GiveUpLimits(),
              throttle: ReplaceThreshold = ReplaceThreshold(),
              moveTimeout: float = MOVE_TIMEOUT,
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
                     giveUp=giveUp, throttle=throttle, moveTimeout=moveTimeout,
                     cyclePeriod=cyclePeriod, moveIds=moveIds)
