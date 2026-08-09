"""robot_radio.pathplan.planner -- gotoWorld/gotoRobot/followPath, thin
``GO_TO`` senders over the accepted-id ack ring (135-007; originally the
outer position loop, 127-006, design issue T5).

History (135-007): before sprint 135, this module ran its OWN arc-solving
and replace-throttling loop -- read telemetry, update ``WorldPose``, solve
a tangent arc (``pathplan.solver.solveArcToPoint()``), send
``move_twist(..., replace=True)`` when the solution moved materially,
repeat until arrival or give-up. Sprint 135 moved that whole policy
firmware-side (``Motion::Navigator``, `src/firm/motion/navigator/`, tickets
002-004): one ``GO_TO`` wire command now drives a world- or robot-frame
target to completion on its own, re-solving against live OTOS pose every
internal cycle with no further host involvement. Proven end-to-end in sim
(ticket 005) and on real hardware (ticket 006, `tovez`) before this
ticket deleted the host loop -- see sprint.md's own "breaking changes go
last" sequencing rule.

What is left here, and why each piece is still needed:

- **``gotoWorld()``/``gotoRobot()``** are now THIN senders: build one
  ``GoTo`` command (frame WORLD or ROBOT respectively), send it with
  ack-verified retry against the ENQUEUE ack (link loss is real
  regardless of where arc-solving runs), then wait for the single
  COMPLETION ack ``Motion::Navigator`` emits when the goto ends (Done or
  Aborted). Neither function transforms a robot-frame target itself any
  more -- the FIRMWARE resolves ``frame=ROBOT`` to world coordinates once,
  at acceptance, using its own live OTOS pose (fresher and more accurate
  than any host-side snapshot could be) -- so ``gotoRobot()`` no longer
  needs a current ``WorldPose`` fix before it can even start, unlike its
  pre-135-007 form.
- **``followPath()``** keeps ``pursuitTarget()``'s own lookahead-point
  picking (`pathplan.solver`) completely UNCHANGED -- picking "which point
  on the path is next" is a host-side problem the firmware's internal
  per-goto re-solve does not have (it only ever knows about ONE target,
  never a path) -- and streams each picked point as a ``GO_TO`` instead of
  driving arcs itself. Because the firmware now owns bearing/pivot
  handling internally (``Motion::Navigator``'s stop-then-pivot-then-arc
  sequencing, SUC-004 of sprint 135), this loop no longer needs ANY
  target-behind guard, unsolvable-cycle counter, or stale-arc-resend
  policy of its own -- the entire "three rejected approaches" saga that
  used to live in this module's own comments (a playfield runaway among
  them) is now moot by construction; see git history (pre-135-007) if the
  detail is ever needed again.
- **The ack-verified retry/dedup machinery** (``AckRetry``,
  ``_recordAcks()``, ``_readFrames()``, ``_advance()``,
  ``_sendVerifiedGoTo()`` -- the ``GO_TO`` analogue of the deleted
  ``_sendVerifiedTwist()``) and **``MoveIdAllocator``** survive UNCHANGED
  in spirit: a lost/corrupted wire command is exactly as real whether the
  firmware solves the arc or the host does, and ticket 001's own measured
  inbound-loss figure sizes this retry policy honestly rather than against
  folklore.

**Deleted as dead code** (135-007, confirmed via
``grep -rn "solveArcToPoint\\|ReplaceThreshold\\|_clampOmegaStep" src/``
returning nothing outside version control): ``pathplan.solver.
solveArcToPoint()``/``ArcSolution``/``SolverLimits``/``MAX_WHEEL_STEP``/
``_clampOmegaStep()`` (the single-arc solver and its curvature slew clamp
-- ported to ``Motion::ArcSolver``, C++-ctest-covered by
`src/firm/motion/navigator/tests/arc_solver_test.cpp`); this module's own
``ReplaceThreshold`` (the material-change replace throttle -- the
firmware's `Motion::Navigator` runs an equivalent throttle internally,
`kNavOmegaReplaceThreshold`/`kNavArcLengthReplaceThreshold`/
`kNavRefreshFraction` in `src/firm/motion/navigator/navigator.cpp`, ported from
this exact class); ``ProgressCheck`` (the liveness backstop that used to
force a resend when the robot sat still despite a clean ack -- no longer
needed: a ``GO_TO``'s own ``timeout`` field is the firmware's bounded
backstop now, and SUC-005 gives OTOS staleness/disconnect its own bounded,
fault-flagged abort path internally); ``_moveTimeoutFor()`` (per-move
timeout derived from a solved arc length -- there is no arc length to
derive from any more, since the host never solves one).

Ack verification, retry, and liveness (history, pre-135-007, OOP fix
2026-07-30, stakeholder directive -- a live playfield run measured the OLD
``gotoWorld()`` sending exactly ONE ``move_twist()`` over 264 solve
iterations and 30s, with the robot never moving at all): the ORIGINAL
motivation for ``_sendVerifiedTwist()``'s ack-verify/retry mechanism
carries forward unchanged into ``_sendVerifiedGoTo()`` below -- a lost
enqueue command (the radio relay has documented sporadic loss,
``.claude/rules/hardware-bench-testing.md``) must never be invisible.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from typing import Callable

from robot_radio.field import Geofence
from robot_radio.nav.pose import Pose
from robot_radio.pathplan.solver import pursuitTarget
from robot_radio.pathplan.world_pose import WorldPose
from robot_radio.robot.protocol import GOTO_FRAME_ROBOT, GOTO_FRAME_WORLD, NezhaProtocol

_POSITION_SCALE = 10.0  # [mm/cm] Pose.x/y are cm (nav.pose's own convention); this module's distance math is mm, matching move_twist()'s/go_to()'s own units.

CYCLE_PERIOD = 0.1       # [s] outer loop pacing / time-advance window -- matches sprint.md's stated realistic ~10 Hz outer-loop rate and the geofence's own ~10 Hz check cadence
_GEOFENCE_CHECK_PERIOD = 0.1  # [s] matches the "~10 Hz" geofence cadence -- never between segments
_POLL_SLEEP = 0.01       # [s] telemetry poll granularity inside _advance()

_ERR_FULL = 4  # envelope.proto ErrCode.ERR_FULL -- "destination queue full" (retryable; matches square_tour.py's own local ERR_FULL constant)

# [mm] followPath()'s own terminal-waypoint arrival-test default -- gates
# ONLY the host-side "have we reached the end of the path" test
# (pursuitTarget()'s remaining/crossTrack + distance-to-terminal check,
# below); unrelated to the firmware's own NavigatorLimits::
# defaultArrivalTolerance, which gotoWorld()/gotoRobot() now defer to via
# go_to()'s own arrive=0.0 default. Numerically the same value the
# pre-135-007 TERMINATION_TOLERANCE used (100 mm) -- the physical floor
# that value was derived from (one solve cycle's worth of travel plus
# actuation lag) is a property of THIS host loop's own cadence, unchanged
# by where arc-solving runs, so the number carries forward even though the
# constant it used to share with gotoWorld()/gotoRobot() does not.
_DEFAULT_PATH_ARRIVAL_TOLERANCE = 100.0

# --- Pure-pursuit lookahead distance (out-of-process, 2026-07-31) ---------
#
# `followPath()` steers at the point on the path one LOOKAHEAD distance
# ahead of the robot (`solver.pursuitTarget()`). That distance is bounded
# from both sides, and both bounds are physical:
#
# FLOOR -- steering lag, and it is the binding one. Between commanding a
# curvature and the robot actually holding it there is a dead time
# (150 ms, the same `actuationDelay` figure `hil_drive.py`'s
# `limits.actuationDelay = 150.0` uses) plus the wheel plant's own
# first-order time constant (230 ms, `kPlantTau` in src/firm/main.cpp). At
# CRUISE the robot therefore covers
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
# These bounds are unchanged by 135-007: they derive from the robot's own
# actuation lag and plant time constant, not from where arc-solving runs.
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


# followPath() sends its currently-picked pursuit target as a fresh GO_TO
# on EVERY cycle -- unconditionally, no throttle. This was NOT the first
# design tried: a 0.5 s wall-clock throttle (`_STREAM_PERIOD`, matching
# ticket 005's own sim-tested EXTERNAL-mode streaming cadence,
# `goto_protocol_harness.cpp`'s `scenarioStreamedTargetsNeverRestBeforeFinal`
# -- "streamed every 10 cycles/500ms") was tried first and REVERTED after
# measurement, not on suspicion:
#
# MEASURED 2026-08-06 against `square_tour.py --sim --mode goto` (a
# rounded-square path with 62.5-90 mm fillets -- much tighter cornering
# than ticket 005's own validated scenario, whose own comment explicitly
# scoped it to "every leg's bearing change small enough to stay well under
# NavigatorLimits::turnFirstAngle", i.e. NOT this case): with the 0.5 s
# throttle, 3 of 4 runs FAILED outright (90 s give-up, ~140 sends, barely
# 15/17 waypoints reached -- roughly 20 mm/s average progress against a
# 150 mm/s CRUISE) and the one run that passed still took far longer than
# expected. Root cause, confirmed by lowering the throttle and re-running:
# `Motion::Navigator`'s own bearing-to-target check
# (`NavigatorLimits::turnFirstAngle`, ~50 deg, `navigator.cpp`'s `tick()`)
# runs every INTERNAL 50 ms tick against whatever target it was last
# handed -- a target 0.5 s stale, on a tight fillet at 150 mm/s, has had
# time for the robot's own heading to drift far enough past it to cross
# that threshold, which triggers a full stop-then-pivot-then-arc
# sequence (SUC-004) INSTEAD of a smooth continued cruise. That sequence
# costs a deceleration, an in-place pivot, and a re-acceleration -- far
# slower than cruising, and it can recur on every fillet, which is
# exactly the measured slowdown. Sending every cycle keeps the Navigator's
# own target fresh enough (bearing error bounded by one cycle's worth of
# heading change, not several) that this never triggers: 5 consecutive
# re-runs with no throttle at all all passed cleanly, matching the
# original design's own (correct) instinct before a throttle was added
# without this evidence. See this ticket's own Completion Notes for the
# full measurement.


class MoveRejected(RuntimeError):
    """Raised by `_sendVerifiedGoTo()` when the firmware NACKs a `go_to()`
    with a non-retryable `ErrCode` -- anything other than `ERR_FULL` (e.g.
    `ERR_BADARG`: a malformed GoTo, or `ERR_NOT_CONFIGURED`: OTOS not
    connected -- a caller/precondition problem no retry can fix).
    Propagates out of `gotoWorld()`/`gotoRobot()`/`followPath()` after
    their own `finally` block's `estop()` runs -- "a halt that raises must
    not be swallowed" (this module's own docstring). Name kept from the
    pre-135-007 `move_twist()`-era version of this exception -- the
    underlying idea ("a non-retryable NACK on a send this loop made") is
    unchanged, only the wire command that can trigger it is different."""


@dataclass(frozen=True)
class GiveUpLimits:
    """Give-up policy for one `gotoWorld()`/`gotoRobot()`/`followPath()`
    call -- an EXPLICIT budget, not an infinite null-the-error loop (design
    issue T5 / sprint.md SUC-006). UNCHANGED by 135-007.

    maxIterations: hard cap on outer-loop wait/poll cycles (still spent at
        `cyclePeriod` per iteration, exactly as before -- only WHAT the
        loop does inside each iteration changed: waiting for a completion
        ack or picking the next pursuit target, not re-solving an arc).
    giveUpTimeout: [s] hard wall-clock cap, independent of iteration
        count (protects a live hardware/sim session against an
        unexpectedly slow outer-loop rate). Also the source, converted to
        milliseconds, for each `go_to()`'s own wire-level `timeout`
        backstop (see `gotoWorld()`'s own docstring for why tying the two
        together is the right simplification for a thin sender).
    """

    maxIterations: int = 200
    giveUpTimeout: float = 30.0  # [s]


@dataclass(frozen=True)
class AckRetry:
    """Enqueue-ack verification/retry policy for this module's `go_to()`
    sends (`_sendVerifiedGoTo()`, below) -- mirrors `square_tour.Tour.
    sendVerified()`'s own measured DAPLink/radio inbound-loss workaround.
    UNCHANGED by 135-007 (kept per the ticket's own instruction) other than
    the send it now guards.

    maxAttempts: wire sends of ONE logical `go_to()` before giving up on
        it (matches `sendVerified()`'s own 4).
    ackTimeout: [s] per-attempt ack deadline (matches `sendVerified()`'s
        own 0.5s-per-attempt window).
    """

    maxAttempts: int = 4
    ackTimeout: float = 0.5  # [s]


@dataclass(frozen=True)
class GotoResult:
    """One `gotoWorld()`/`gotoRobot()` call's outcome.

    success: True iff the single completion ack `Motion::Navigator` sent
        for this goto reported `ok` (i.e. the firmware itself declares
        arrival, not a host-side distance check -- unlike the pre-135-007
        version of this loop, this thin sender never judges arrival
        itself).
    reason: a human-readable explanation -- always set, on EVERY outcome
        (completion, give-up, or an unacked enqueue), never blank.
    finalPose: the last known world-frame `Pose` this call observed via
        telemetry, or None if no telemetry was ever ingested during the
        call.
    iterations: total completion-ack wait/poll cycles run (excludes the
        enqueue send's own internal ack-retry polling, tracked separately
        via `retries`).
    sent: total logical `go_to()` sends this call made -- always 1 for
        `gotoWorld()`/`gotoRobot()` (one target, one send; kept as a field
        for symmetry with `FollowPathResult.sent`, which genuinely varies).
    retries: total EXTRA wire `go_to()` attempts beyond the first, for that
        one logical send (`AckRetry` -- visibility into ack loss on the
        link, not navigation behavior).
    unacked: 1 if the enqueue was NEVER cleanly acked despite
        `AckRetry.maxAttempts` tries (in which case `success` is always
        False and this call never even started waiting for a completion
        ack) -- 0 otherwise. A nonzero value means the link/command
        channel was the problem, not the navigator.
    """

    success: bool
    reason: str
    finalPose: "Pose | None"
    iterations: int
    sent: int
    retries: int = 0
    unacked: int = 0


class MoveIdAllocator:
    """Strictly monotonic id source for one planner session -- shared by
    every `Move.id` a caller in this codebase issues AND, since 135-007,
    every `GoTo.id` `gotoWorld()`/`gotoRobot()`/`followPath()` issue too.
    One allocator, one id space: a caller mixing Moves and gotos in the
    same robot boot session shares ONE instance across both so a goto id
    can never collide with a Move id (or vice versa).

    Never emits `0`: `Move.id == 0` is the firmware's dedup-EXEMPT
    sentinel for Moves (ticket 002's own verification contract, "id-0
    exemption") -- a move sent with id 0 is NEVER deduped, which is
    exactly wrong for a loop that continuously replaces the in-flight
    Move, since an id-0 replacement whose enqueue ack is lost and retried
    could double-execute instead of being caught by the dedup ring.
    `GoTo.id` has no such dedup ring at all (verified directly against
    `src/firm/app/robot_loop.cpp`'s `handleGoto()`, 135-007) so id 0 is
    not specially dangerous for a goto the way it is for a Move -- but
    starting at 1 (or any caller-chosen positive value) keeps every id
    this allocator issues inside the same, single, unambiguous space
    regardless of which command family eventually uses it.

    Pure, no I/O -- unit-testable standalone. A single instance must be
    shared across every `gotoWorld()`/`gotoRobot()`/`followPath()` call in
    one robot session that issues more than one goto in sequence (e.g. a
    multi-waypoint tour): the DEFAULT behaviour when a caller omits
    `moveIds` is a fresh, call-scoped allocator restarting at 1, which is
    only safe for a single isolated call across the whole boot lifetime of
    the robot's dedup ring -- reusing a fresh allocator across multiple
    sequential calls in the SAME boot session risks a later, genuinely-new
    id being misread as a duplicate of an earlier one that reused the same
    low id.
    """

    def __init__(self, start: int = 1) -> None:
        if start <= 0:
            raise ValueError(f"MoveIdAllocator: start must be > 0 (id 0 is the dedup-exempt "
                              f"sentinel for Moves), got {start!r}")
        self._next = start

    def next(self) -> int:
        moveId = self._next
        self._next += 1
        return moveId


def _giveUpReason(iterations: int, elapsed: float, limits: GiveUpLimits) -> "str | None":
    """Pure give-up check -- no I/O. Returns an explicit reason string once
    either budget (`GiveUpLimits.maxIterations`/`giveUpTimeout`) is
    exhausted, else None. Never a silent infinite retry -- the caller
    always gets a stated reason. UNCHANGED by 135-007."""
    if iterations >= limits.maxIterations:
        return (f"gave up after {iterations} solve iterations "
                f"(limit {limits.maxIterations}) without reaching the target")
    if elapsed >= limits.giveUpTimeout:
        return (f"gave up after {elapsed:.1f}s (limit {limits.giveUpTimeout:.1f}s) "
                f"without reaching the target")
    return None


def _readFrames(proto: NezhaProtocol) -> "list":
    """Drain whatever telemetry frames are currently queued -- dispatches
    to whichever shape `proto`'s own connection exposes, making every
    function in this module genuinely backend-agnostic (the same
    `NezhaProtocol` interface works against both hardware and sim).
    UNCHANGED by 135-007.

    `NezhaProtocol.read_pending_binary_tlm_frames()` (the ordinary,
    hardware-path method) assumes `self._conn.drain_binary_tlm()`
    (`SerialConnection`'s own raw-pb2 drain, which THAT method then
    adapts into `TLMFrame`s) -- `SimConfigConn` has no
    `drain_binary_tlm()`, since `SimLoop.read_pending_binary_tlm_frames()`
    already returns adapted `TLMFrame`s directly. Prefer the CONNECTION's
    own `read_pending_binary_tlm_frames()` when it has one
    (`SimConfigConn`), falling back to the ordinary protocol-level method
    otherwise (`SerialConnection`, via `NezhaProtocol`'s own adapter)."""
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
    UNCHANGED by 135-007: the firmware now owns arc-solving, but the host
    still watches the robot move via telemetry and can still halt it, so
    this primitive's own job is identical. Raises `GeofenceViolation` if
    the geofence fires -- `estop()` has already been sent internally by
    `Geofence._halt()` by the time that happens; the caller's own
    `finally` block sends it again (idempotent) and lets the exception
    propagate."""
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
    """Fold every frame's bounded ack ring into `ackSeen` (corr_id/goto_id
    -> `AckEntry`) -- shared by every wait loop in this module. UNCHANGED
    by 135-007: a `GoTo`'s completion ack rides the SAME bounded ack ring
    a Move's completion ack does (`Telemetry.acks`), keyed on `GoTo.id`
    exactly like a Move's is keyed on `Move.id`, so this fold needs no
    GO_TO-specific logic at all."""
    for frame in frames:
        for ack in frame.acks:
            ackSeen[ack.corr_id] = ack


def _sendVerifiedGoTo(proto: NezhaProtocol, worldPose: WorldPose,
                      geofence: "Geofence | None", ackSeen: "dict[int, object]",
                      *, gotoId: int, x: float, y: float, frame: int,
                      speed: float, arrive: float, timeout: float,
                      cyclePeriod: float, ackRetry: AckRetry) -> "tuple[bool, int]":
    """Send one `proto.go_to(x, y, frame=frame, ..., goto_id=gotoId)`,
    verifying the ENQUEUE ack and retrying on loss -- the `GO_TO` analogue
    of the deleted `_sendVerifiedTwist()` (135-007; see git history for the
    `move_twist()`-era version this replaces). Matches that function's own
    approach exactly: scans the SAME `_advance()` drain this call's own
    wait loop runs, never a second `wait_for_ack()` read (`square_tour.
    Tour.sendVerified()`'s own docstring: "on a single-consumer telemetry
    queue two readers starve each other"). Ingests every drained frame into
    `worldPose` exactly like every caller's main loop does, so a
    multi-attempt wait never starves pose tracking.

    Every retry attempt of this call reuses the SAME `gotoId` -- but for a
    DIFFERENT underlying reason than `_sendVerifiedTwist()`'s own id reuse
    had. A Move retry is safe because the firmware's dedup ring
    (`alreadyAccepted()`) makes a same-id resend idempotent when the
    original already landed. `GO_TO` has NO such ring
    (`RobotLoop::handleGoto()` -- verified directly, 135-007): a retried
    `go_to()` simply calls `Motion::Navigator::start()` again with the
    SAME x/y/frame, which is harmless (a redundant restart toward an
    identical target) rather than wrong, but it is not "recovered
    idempotently" in the Move sense -- it is "safe to repeat because
    nothing about the request changed." Reusing the id still matters: it
    is what lets the eventual completion ack be attributed to the send
    THIS call made, whichever wire attempt actually landed.

    Treats `ERR_FULL` (queue transiently full) as retryable, matching
    `_sendVerifiedTwist()`'s own precedent (`GO_TO` has no queue of its own
    to fill, but the wire's `ErrCode` set is shared, and treating it
    identically costs nothing). Any OTHER nonzero `err_code` (e.g.
    `ERR_BADARG`, `ERR_NOT_CONFIGURED` -- OTOS disconnected) is a
    caller/precondition problem a retry cannot fix -- raises
    `MoveRejected` immediately rather than burning the rest of
    `ackRetry.maxAttempts` on a command that will never succeed.

    Returns `(acked, attempts)`: `acked` is True iff a clean
    (`err_code == 0`) ack was observed within `ackRetry.maxAttempts`
    tries; `attempts` is how many `go_to()` calls were actually made
    (1..`maxAttempts`) -- the caller derives `attempts - 1` as this one
    send's own RETRY count for `GotoResult.retries`/
    `FollowPathResult.retries`.
    """
    for attempt in range(1, ackRetry.maxAttempts + 1):
        corr = proto.go_to(x, y, frame=frame, speed=speed, arrive=arrive,
                           timeout=timeout, goto_id=gotoId)
        waited = 0.0
        entry = None
        while waited < ackRetry.ackTimeout:
            frames = _advance(proto, cyclePeriod, geofence)
            _recordAcks(frames, ackSeen)
            for tlmFrame in frames:
                worldPose.ingest(tlmFrame)
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
                f"go_to() (goto_id={gotoId}, corr_id={corr}) was NACKed with "
                f"a non-retryable ErrCode {entry.err_code} (not ERR_FULL) -- "
                f"retrying would not help")
        # ERR_FULL -- fall through and retry with a fresh corr_id, same gotoId.
    return False, ackRetry.maxAttempts


def _gotoAndWait(proto: NezhaProtocol, worldPose: WorldPose, x: float, y: float,
                 frame: int, *, speed: float, arrive: float,
                 geofence: "Geofence | None", giveUp: GiveUpLimits,
                 ackRetry: AckRetry, cyclePeriod: float,
                 moveIds: "MoveIdAllocator | None") -> GotoResult:
    """Shared implementation behind `gotoWorld()`/`gotoRobot()` -- send ONE
    `GO_TO` (`frame` selects WORLD or ROBOT; the caller-facing wrappers
    below pick it) with ack-verified retry against the ENQUEUE ack, then
    wait for the single completion ack `Motion::Navigator` emits when this
    goto ends (Done or Aborted), tracking telemetry the whole time (for
    `GotoResult.finalPose` and the geofence -- "never drive blind").

    `giveUp.giveUpTimeout` derives BOTH this call's own wait-loop deadline
    AND the wire `go_to()`'s own `timeout` backstop (converted to ms) --
    a deliberate simplification for a thin sender: there is no longer a
    host-side arc to size a per-move timeout from (`_moveTimeoutFor()` is
    deleted), and a caller wanting the robot to keep trying longer than it
    is willing to wait around for would need a SEPARATE call anyway once
    this one returns. `goto_otos.py`'s own `goto()`/`main()` establishes
    the same one-timeout-serves-both convention already.

    `x`/`y` are in the caller's own units -- see `gotoWorld()`/
    `gotoRobot()`'s own docstrings for the cm convention; this function
    converts to the wire's mm via `_POSITION_SCALE` right before sending.
    """
    allocator = moveIds if moveIds is not None else MoveIdAllocator()
    gotoId = allocator.next()

    startTime = time.monotonic()
    iterations = 0
    ackSeen: "dict[int, object]" = {}
    finalPose: "Pose | None" = None

    xMm, yMm = x * _POSITION_SCALE, y * _POSITION_SCALE
    timeoutMs = giveUp.giveUpTimeout * 1000.0

    try:
        acked, attempts = _sendVerifiedGoTo(
            proto, worldPose, geofence, ackSeen, gotoId=gotoId,
            x=xMm, y=yMm, frame=frame, speed=speed, arrive=arrive,
            timeout=timeoutMs, cyclePeriod=cyclePeriod, ackRetry=ackRetry)
        retries = attempts - 1
        if not acked:
            return GotoResult(
                success=False,
                reason=(f"go_to() enqueue was never acked despite "
                        f"{ackRetry.maxAttempts} attempts -- link/command loss, "
                        f"not a navigation problem"),
                finalPose=worldPose.worldPose(), iterations=0, sent=1,
                retries=retries, unacked=1)

        # Wait for the ONE completion ack Motion::Navigator emits when this
        # goto ends -- keyed on gotoId, exactly like a Move's own
        # completion ack is keyed on Move.id.
        while True:
            currentPose = worldPose.worldPose()
            if currentPose is not None:
                finalPose = currentPose

            entry = ackSeen.get(gotoId)
            if entry is not None:
                return GotoResult(
                    success=bool(entry.ok),
                    reason=("arrived (completion ack ok)" if entry.ok else
                            f"goto aborted (completion ack err_code={entry.err_code})"),
                    finalPose=finalPose, iterations=iterations, sent=1,
                    retries=retries, unacked=0)

            iterations += 1
            elapsed = time.monotonic() - startTime
            giveUpReason = _giveUpReason(iterations, elapsed, giveUp)
            if giveUpReason is not None:
                return GotoResult(
                    success=False,
                    reason=f"{giveUpReason} (no completion ack observed for this goto)",
                    finalPose=finalPose, iterations=iterations, sent=1,
                    retries=retries, unacked=0)

            frames = _advance(proto, cyclePeriod, geofence)
            _recordAcks(frames, ackSeen)
            for tlmFrame in frames:
                worldPose.ingest(tlmFrame)
    finally:
        # estop(), never the PLANNED stop() -- every halt path in this
        # module funnels through this one call site. Runs on every return
        # above AND on any exception (e.g. GeofenceViolation out of
        # _advance(), or MoveRejected out of _sendVerifiedGoTo()); the
        # exception is not caught here, so it still propagates after this
        # line -- a halt that raises must not be swallowed. Harmless on a
        # clean arrival too: Motion::Navigator has already brought the
        # robot to rest by the time the completion ack lands, so this is
        # an idempotent, defensive re-assertion, not a live stop.
        proto.estop()


def gotoWorld(proto: NezhaProtocol, worldPose: WorldPose, x: float, y: float, *,
              speed: float = 0.0, arrive: float = 0.0,
              geofence: "Geofence | None" = None,
              giveUp: GiveUpLimits = GiveUpLimits(),
              ackRetry: AckRetry = AckRetry(),
              cyclePeriod: float = CYCLE_PERIOD,
              moveIds: "MoveIdAllocator | None" = None) -> GotoResult:
    """Drive the robot to a WORLD-frame target `(x, y)` -- a thin `GO_TO`
    sender (135-007; `gotoRobot()` below is `gotoWorld()`'s ROBOT-frame
    sibling, not a composition through it any more -- both are equally
    thin over the shared `_gotoAndWait()` implementation).

    x/y: [cm] world/camera-frame target position -- `nav.pose.Pose`'s own
        convention, matching `WorldPose`/camera-fix coordinates throughout
        this package. Converted to the wire's mm internally.
    speed: [mm/s] cruise-speed override forwarded to `go_to()`'s own
        `speed` parameter; `0.0` (the default) falls open to the robot's
        configured `NavigatorLimits::speed`.
    arrive: [mm] arrival-tolerance override forwarded to `go_to()`'s own
        `arrive` parameter; `0.0` falls open to
        `NavigatorLimits::defaultArrivalTolerance`.
    geofence: checked inside `_advance()`'s own ~10 Hz time-advance
        primitive on every cycle -- None disables it (e.g. a sim run
        with no camera).
    giveUp: the give-up policy (`GiveUpLimits`) -- also derives this
        goto's own wire-level `timeout` backstop (converted to ms); see
        `_gotoAndWait()`'s own docstring for why the two share one value.
    ackRetry: the enqueue-ack verification/retry policy (`AckRetry`) --
        see `_sendVerifiedGoTo()`'s own docstring for the full mechanism.
    cyclePeriod: [s] this call's own telemetry-poll cadence while it waits
        for the completion ack.
    moveIds: the shared monotonic id source (`MoveIdAllocator`) -- a
        caller issuing MULTIPLE sequential goto/Move calls in one robot
        session MUST share one instance across all of them (see that
        class's own docstring); the default creates a fresh, call-scoped
        allocator only safe for a single isolated call.

    Returns a `GotoResult` -- never raises for an ordinary outcome
    (completion, an aborted goto, an unacked enqueue, or a wait-loop
    give-up); DOES let a `GeofenceViolation` (or any other exception
    `_advance()`/the wire layer raises), or a `MoveRejected` (a
    non-retryable NACK -- see `_sendVerifiedGoTo()`), propagate after this
    function's own `finally` block has sent `estop()` -- "a halt that
    raises must not be swallowed."
    """
    return _gotoAndWait(proto, worldPose, x, y, GOTO_FRAME_WORLD, speed=speed,
                        arrive=arrive, geofence=geofence, giveUp=giveUp,
                        ackRetry=ackRetry, cyclePeriod=cyclePeriod, moveIds=moveIds)


def gotoRobot(proto: NezhaProtocol, worldPose: WorldPose, x: float, y: float, *,
              speed: float = 0.0, arrive: float = 0.0,
              geofence: "Geofence | None" = None,
              giveUp: GiveUpLimits = GiveUpLimits(),
              ackRetry: AckRetry = AckRetry(),
              cyclePeriod: float = CYCLE_PERIOD,
              moveIds: "MoveIdAllocator | None" = None) -> GotoResult:
    """Drive the robot to a ROBOT-frame target `(x, y)` -- `gotoWorld()`'s
    thin ROBOT-frame sibling (135-007). Shares every parameter's meaning
    with `gotoWorld()` (not re-derived here) except the frame `x`/`y` are
    interpreted in.

    x/y: [cm] target position in the ROBOT's OWN current body frame
        (forward = +x, left = +y -- `Pose.heading`'s CCW-positive
        convention). Resolved to world coordinates ONCE, FIRMWARE-side
        (`App::RobotLoop::handleGoto()`), at the moment this `GO_TO` is
        accepted -- using the robot's own live OTOS pose, which is fresher
        and more precise than any host-side `WorldPose` snapshot could be
        by the time a command reaches the wire. Unlike this function's
        pre-135-007 form, `worldPose` is therefore no longer READ to
        compute this call's target (only written to, via ordinary
        telemetry ingestion, for `GotoResult.finalPose` and the
        geofence) -- a caller may call `gotoRobot()` on a totally fresh,
        never-seeded `WorldPose` and it still works correctly, since the
        offset never leaves the robot's own frame until the firmware
        resolves it.
    """
    return _gotoAndWait(proto, worldPose, x, y, GOTO_FRAME_ROBOT, speed=speed,
                        arrive=arrive, geofence=geofence, giveUp=giveUp,
                        ackRetry=ackRetry, cyclePeriod=cyclePeriod, moveIds=moveIds)


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
    unacked: int = 0


def followPath(proto: NezhaProtocol, worldPose: WorldPose, waypoints: "list[Pose]",
               *, speed: float, arrive: float = 0.0,
               geofence: "Geofence | None" = None,
               tolerance: float = _DEFAULT_PATH_ARRIVAL_TOLERANCE,
               giveUp: GiveUpLimits = GiveUpLimits(),
               ackRetry: AckRetry = AckRetry(),
               cyclePeriod: float = CYCLE_PERIOD,
               moveIds: "MoveIdAllocator | None" = None,
               lookahead: "float | None" = None,
               onWaypoint: "Callable[[int, Pose], None] | None" = None) -> FollowPathResult:
    """Drive the robot through a SEQUENCE of world-frame waypoints as ONE
    continuous pure-pursuit run, streaming `GO_TO` targets instead of
    calling `gotoWorld()` once per waypoint and requiring a full
    stop-and-arrive at every one (135-007; originally 127-007/127-008 --
    see the module-level comment above `solver.pursuitTarget()` for why a
    dense waypoint sequence, e.g. a rounded-corner square, needs this
    instead of N separate `gotoWorld()` calls).

    No waypoint is ever a steering target. Each cycle,
    `solver.pursuitTarget()` (UNCHANGED by 135-007 -- this is exactly the
    "pure pursuit's own lookahead-point selection algorithm stays
    host-side" carve-out sprint 135's own Out of Scope section names)
    projects the robot onto the POLYLINE through `waypoints` and returns
    the point one `lookahead` (`_lookaheadFor()`, or the caller's own
    override) farther along it. Waypoints are therefore only vertices of
    the path being followed, never things to arrive at. Only the LAST
    waypoint gets a real, tolerance-gated arrival test (`tolerance`) --
    this loop has to stop somewhere. This host-side arrival test is
    UNCHANGED by 135-007 (it never depended on how arcs got solved); what
    changed is only what happens once a target is picked -- see below.

    The projection is MONOTONE: the segment it landed on is carried into
    the next cycle as the search's own starting point, so a closed path (a
    square, whose terminal waypoints sit right next to its first ones) can
    never snap the robot back onto a stretch it has already driven.

    Per-cycle ordering is INGEST POSE -> PICK TARGET -> SEND, EVERY CYCLE,
    unconditionally -- the currently-picked target is sent as a fresh
    `GO_TO` (frame WORLD, ack-verified retry against the ENQUEUE ack) on
    every single iteration, not on some throttled interval. See the
    module-level comment above this section for why: a wall-clock send
    throttle was tried first and MEASURED to cause repeated spurious
    stop-then-pivot-then-arc sequences on tight cornering (`Motion::
    Navigator`'s own bearing-to-target check runs every internal tick
    against whatever target it was last handed, and a stale target drifts
    past the pivot-first bearing threshold on a tight fillet) -- sending
    every cycle keeps that target fresh enough this never triggers.
    Between sends, `Motion::Navigator` still keeps converging on the
    LAST-accepted target entirely on its own if a send is ever lost
    (sprint 135 Decision 2: "a lost or late EXTERNAL target update is
    benign by construction") -- this loop just never deliberately WITHHOLDS
    one. Because the target is a lookahead point ON THE PATH rather than a
    raw waypoint, and because the firmware now handles any bearing/pivot
    issue internally (`Motion::Navigator`'s own stop-then-pivot-then-arc
    sequencing, SUC-004), this loop no longer needs, and no longer has,
    any target-behind guard, unsolvable-cycle counter, or stale-arc-resend
    policy of its own -- see this module's own top-of-file docstring for
    what that replaces and why it is safe to have simply deleted.

    `onWaypoint(index, pose)`, if given, is called once for every DISTINCT
    waypoint index the path projection advances past (increasing order,
    possibly several per cycle), and once more for the terminal waypoint
    on a successful arrival -- UNCHANGED by 135-007.

    All other parameters, the `estop()`-on-every-exit contract (this
    function's own `finally` block, run on every return AND on any
    exception), and the enqueue-ack-retry machinery match `gotoWorld()`'s
    own -- see that function's docstring for the full description; not
    re-derived here.
    """
    if not waypoints:
        raise ValueError("followPath(): waypoints must be non-empty")
    lastIndex = len(waypoints) - 1
    carrot = lookahead if lookahead is not None else _lookaheadFor(speed)
    allocator = moveIds if moveIds is not None else MoveIdAllocator()

    startTime = time.monotonic()
    iterations = 0
    sentCount = 0
    retryCount = 0
    unackedCount = 0
    ackSeen: "dict[int, object]" = {}
    segment = 0  # monotone path-projection segment, carried across cycles
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
            for tlmFrame in frames:
                worldPose.ingest(tlmFrame)
            currentPose = worldPose.worldPose()

            if currentPose is None:
                iterations += 1
                giveUpReason = _giveUpReason(iterations, time.monotonic() - startTime, giveUp)
                if giveUpReason is not None:
                    return FollowPathResult(
                        success=False, reason=f"{giveUpReason} (no telemetry received)",
                        finalPose=None, iterations=iterations, sent=sentCount,
                        waypointsReached=reachedIndex + 1, retries=retryCount,
                        unacked=unackedCount)
                continue

            # PICK TARGET before SEND (ordering guarantee -- see docstring).
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
            # terminal waypoint BEHIND it.
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
                    unacked=unackedCount)

            iterations += 1
            elapsed = time.monotonic() - startTime
            giveUpReason = _giveUpReason(iterations, elapsed, giveUp)
            if giveUpReason is not None:
                detail = (f"on path segment {segment}/{lastIndex - 1}; cross-track "
                          f"{pursuit.crossTrack:.1f} mm; {pursuit.remaining:.1f} mm of path left")
                if unackedCount > 0:
                    detail += (f"; {unackedCount} of {sentCount} send(s) never received a "
                              f"clean ack despite {ackRetry.maxAttempts} attempts each -- "
                              f"link/command loss, not a navigation problem")
                else:
                    detail += f"; {sentCount} send(s) acked ok"
                return FollowPathResult(
                    success=False, reason=f"{giveUpReason} ({detail})",
                    finalPose=currentPose, iterations=iterations, sent=sentCount,
                    waypointsReached=reachedIndex + 1, retries=retryCount,
                    unacked=unackedCount)

            # Send EVERY cycle, unconditionally -- see this function's own
            # docstring and the module-level comment above _lookaheadFor()
            # for why a throttle here is actively unsafe on tight cornering
            # (measured, not assumed).
            moveId = allocator.next()
            xMm, yMm = target.x * _POSITION_SCALE, target.y * _POSITION_SCALE
            timeoutMs = giveUp.giveUpTimeout * 1000.0
            acked, attempts = _sendVerifiedGoTo(
                proto, worldPose, geofence, ackSeen, gotoId=moveId,
                x=xMm, y=yMm, frame=GOTO_FRAME_WORLD, speed=speed, arrive=arrive,
                timeout=timeoutMs, cyclePeriod=cyclePeriod, ackRetry=ackRetry)
            sentCount += 1
            retryCount += attempts - 1
            if not acked:
                unackedCount += 1
    finally:
        # estop(), never the PLANNED stop() -- same contract as gotoWorld()'s
        # own finally block: runs on every return above AND on any
        # exception, and does not swallow it.
        proto.estop()
