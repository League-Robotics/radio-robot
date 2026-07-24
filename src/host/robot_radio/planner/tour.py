"""robot_radio.planner.tour -- tour geometry ownership + chained execution.

Owns `TOUR_1`/`TOUR_2` (typed via `parse_tour()`) and the shared per-leg
execution loop the TestGUI (`testgui/__main__.py`'s `_TourRunner`) drives:
sends ONE `Move` command per leg (`transport.move()`), enqueueing the NEXT
leg while the CURRENT one is still active (one-leg lookahead, SUC-003) so
firmware's own bounded queue and boundary-velocity carry sequence
compatible legs without decelerating to a stop. Per-leg completion is
driven by that `Move`'s own completion EVENT (`docs/protocol-v4.md`
section 7.2, matched on `Move.id`, scanned off the bounded `acks` ring --
121-002 fixed this module's own completion poll to scan the ring rather
than only the single scalar "freshest ack" slot, closing
`tour-1-final-leg-completes-only-on-stop.md`), never a host-timed settle
delay or a polled `fault_bits` check. Full history (the sprint 107 origin,
the 109-008 Move-queue adoption, the 2026-07-22 protocol-v4 port, the
121-002 ack-ring fix): src/host/robot_radio/DESIGN.md.

Boundary: inside -- the geometry data, parsing it into typed leg specs,
the per-leg run loop (build a `Move` per leg, enqueue one leg ahead, poll
for each leg's own completion event, capture the measured pose before leg
1 and after the final leg), and closure-delta computation. Outside -- the
`Move` wire encoding itself (calls `transport.move()`/`transport.
read_pending_binary_tlm_frames()`, never builds a `CommandEnvelope`),
heading readback (read-only -- firmware owns the closed loop), the
transport itself (accepts a `MoveTransport`-compatible object, never
imports `NezhaProtocol`/`SerialConnection`/`SimConnection` directly), and
any GUI/trace-file-format concern (the optional `row_callback`/`on_leg`
hooks are the only surface offered to a caller wanting a trace or
narration).

Usage
-----
    from robot_radio.planner.tour import TOUR_1, parse_tour, run_tour

    legs = parse_tour(TOUR_1)
    result = run_tour(transport, params, heading, legs)
    print(result.closure.position_delta, result.closure.heading_delta)

No hardware, no sim transport required for this module's own test gate --
see `src/tests/testgui/test_tour_stop.py`'s own `_FakeTransport` double and
`test_tour_closure_gate.py`'s sim-backed coverage.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Literal, Protocol, Sequence

from robot_radio.controllers.pid import normalize_angle
from robot_radio.planner.executor import RunOutcome, RunState, TickResult

if TYPE_CHECKING:
    from robot_radio.planner.heading import HeadingCorrector
    from robot_radio.planner.model import PlannerParams
    from robot_radio.robot.protocol import AckEntry, TLMFrame

logger = logging.getLogger(__name__)


class MoveTransport(Protocol):
    """The exact slice of `NezhaProtocol`'s public surface `run_tour()`
    depends on -- a `Protocol` (structural, duck-typed), mirroring
    `planner.executor.TwistTransport`'s own convention, so a unit test can
    hand this module a lightweight fake with no real serial port / protobuf
    codec behind it (see `tests/unit/test_planner_tour.py`'s `FakeTransport`).

    Mirrors `SimLoop.move()`'s own kwargs exactly (protocol v4's `Move`
    schema, `docs/protocol-v4.md` section 4) -- a velocity variant
    (`v_x`/`v_y`/`omega` for a `MoveTwist`, OR `v_left`/`v_right` BOTH given
    for a `MoveWheels`) plus exactly one stop condition
    (`stop_time`/`stop_distance`/`stop_angle`) plus a required `timeout`
    safety backstop. `id` becomes `Move.id` (the later COMPLETION ack's
    key, per `docs/protocol-v4.md` section 7.2) -- the envelope's own
    `corr_id` (the EARLIER enqueue ack's key) is a SEPARATE, independently
    assigned value on BOTH transports: a real `NezhaProtocol.move()`'s own
    envelope `corr_id` is auto-assigned by the connection, distinct from
    `id`/`Move.id`; `SimLoop.move()` draws its own envelope `corr_id` from
    the SAME per-instance counter `id` itself defaults from, but as a
    SEPARATE draw, so the two are independent there too (turn-prediction
    -campaign fix -- `SimLoop.move()` used to set `corr_id == id`, which
    aliased the enqueue ack onto the exact key `_drain_and_poll()` polls
    for; see that method's own doc comment for the full failure mode this
    closed). THIS module only ever polls for `id`'s own COMPLETION ack (see
    `_drain_and_poll()`), never the enqueue ack, so the distinction is
    transparent to `run_tour()` -- now genuinely, not just by construction
    on one of the two transports. Both a real `NezhaProtocol` (`.move()`
    added by this same fix) and a `robot_radio.io.sim_loop.SimLoop` satisfy
    this Protocol as-is -- no adapter needed in production.
    """

    def move(self, *, v_x: float = 0.0, v_y: float = 0.0, omega: float = 0.0,
             v_left: "float | None" = None, v_right: "float | None" = None,
             stop_time: "float | None" = None, stop_distance: "float | None" = None,
             stop_angle: "float | None" = None, timeout: float,
             replace: bool = True, id: "int | None" = None) -> int: ...

    def stop(self) -> int: ...

    def read_pending_binary_tlm_frames(self) -> "list[TLMFrame]": ...


# TLMFrame.pose's heading is integer centidegrees (matches `heading.py`'s
# own `_HEADING_SCALE`, duplicated here since that's a private constant
# there). AC3 pins tour closure to `TLMFrame.pose` specifically, independent
# of whichever source the caller's `HeadingCorrector` reads for its own
# per-tick trim -- see this module's own `_frame_pose_rad()`.
_HEADING_SCALE = math.pi / 18000.0  # [rad/cdeg]


# ---------------------------------------------------------------------------
# Tour geometry. A "tour" is an ordered list of legacy `D`/`RT` wire-string
# steps (never sent as-is -- both verbs are retired); `parse_tour()` below
# turns them into typed `TourLeg`s that `run_tour()` drives via `Move`
# commands instead. Regression-protected against `testgui/commands.py`'s
# own `TOURS` dict identity (src/tests/testgui/test_tour1_geometry.py).
# ---------------------------------------------------------------------------

TOUR_1: list[str] = [
    "D 200 200 345",
    "RT 9000",
    "D 200 200 240",
    "RT 9000",
    "D 200 200 700",
    "RT 9000",
    "D 200 200 480",
    "RT 9000",
    "D 200 200 700",
    "RT 9000",
    "D 200 200 240",
    "RT 9000",
    "D 200 200 345"
]


TOUR_2: list[str] = [
    "D 200 200 345",
    "RT 9000",
    "D 200 200 240",
    "RT 12400",
    "D 200 200 850",
    "RT -21700",
    "D 200 200 700",
    "RT 14600",
    "D 200 200 850",
    "RT 21500",
    "D 200 200 700",
    "RT -9000",
    "D 200 200 240",
    "RT -9000",
    "D 200 200 345",
]


# ---------------------------------------------------------------------------
# Defaults for MOVE-queue tour execution (109-008, re-pinned 2026-07-22 for
# protocol v4). Accel/jerk ceilings are entirely firmware's own config now
# (not a per-command wire value or a host-computed profile). `v_max` sizes
# a straight leg's `MoveTwist.v_x` fallback; `omega_max` (from
# `PlannerParams`, this module's own `params` argument) supplies every turn
# leg's `MoveTwist.omega`, since `RT` carries no rate of its own on the
# wire -- duplicated from `testgui/transport.py`'s `_UNMANAGED_YAW_RATE`
# rather than imported (`planner/` must never import `testgui/`).
# ---------------------------------------------------------------------------

DEFAULT_V_MAX = 150.0  # [mm/s] straight-leg linear ceiling fallback (no per-leg speed on the D step)
DEFAULT_FINAL_SETTLE = 0.6  # [s] post-terminal-DONE settle window before capturing the closure end pose --
# the completion event fires the instant the stop condition is met, but the PLANT needs a little
# more real time to physically settle before its reported pose reflects where it actually stopped.
DEFAULT_MOVE_TIMEOUT = 15.0  # [s] bound on how long run_tour() waits for one leg's own terminal
# completion ack before giving up (RunOutcome.FAULT) -- always bounded, mirroring
# NezhaProtocol.wait_for_ack()'s own "never infinite" contract.

# `Move.timeout` -- the WIRE safety backstop (docs/protocol-v4.md section 4.1: REQUIRED, <=0 is
# ERR_BADARG) -- distinct from DEFAULT_MOVE_TIMEOUT above (this module's own HOST-side poll bound).
# Sized off each leg's own expected duration, same factor/floor testgui/transport.py's
# `_move_timeout_for()` uses (duplicated, not imported -- see the block comment above for why).
_MOVE_TIMEOUT_FACTOR = 3.0    # [multiple of expected duration]
_MOVE_MIN_TIMEOUT = 2000.0    # [ms]

# `Move.id` values `run_tour()` assigns to its own legs -- offset far above any realistic
# session-scoped envelope `corr_id` so a leg's COMPLETION ack (echoes `Move.id`) can never be
# mistaken for some OTHER command's ENQUEUE ack (echoes `corr_id`) landing in the same ack
# ring/slot -- see `_drain_and_poll()`'s own docstring for why that collision would be a
# false-positive.
_TOUR_MOVE_ID_BASE = 1 << 20
DEFAULT_POLL_INTERVAL = 0.05  # [s] sleep between two consecutive ack-ring polls

_BEGIN_DRAIN_RETRIES = 5  # bounded retry -- a single read_pending_binary_tlm_frames() call can
_BEGIN_DRAIN_RETRY_INTERVAL = 0.05  # [s] legitimately race an idle queue between two ~25Hz pushes and come back empty.


# ---------------------------------------------------------------------------
# Typed legs + parser
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TourLeg:
    """One parsed step of a tour -- a signed straight distance or a signed
    in-place turn angle, matching `RT`'s own sign convention (positive
    CCW/left)."""

    kind: Literal["distance", "turn"]
    value: float  # [mm] signed, "distance" legs; [deg] signed, "turn" legs
    speed: float | None = None  # [mm/s] magnitude, "distance" legs only -- the D wire
    # string's own left/right speed field (TOUR_1/2's own data always has left==right;
    # a future tour with left!=right yields this leg's own average magnitude). Always
    # None for "turn" legs -- RT carries no rate field on the wire.


def parse_tour(wire_steps: Sequence[str]) -> list[TourLeg]:
    """Parse a `TOUR_1`/`TOUR_2`-shaped wire-string list into an ordered
    `TourLeg` sequence. Recognizes exactly two verbs:

    - `"D <left> <right> <mm>"` -- a straight leg; `mm` (signed) becomes the
      leg's own `value`, `left`/`right` become its `speed` (average
      magnitude, honoring the wire string's own authored speed intent --
      `run_tour()`'s own docstring explains why preserving it costs nothing
      extra in safety).
    - `"RT <cdeg>"` -- a turn leg; the signed centidegree argument is
      converted to signed degrees (`/100`).

    Any other verb raises `ValueError` immediately -- defense in depth: a
    future tour author who adds an unsupported step gets a clear error, not
    silent misparsing.
    """
    legs: list[TourLeg] = []
    for step in wire_steps:
        tokens = step.split()
        if not tokens:
            raise ValueError(f"parse_tour(): empty step in tour list: {step!r}")
        verb = tokens[0]
        if verb == "D":
            if len(tokens) != 4:
                raise ValueError(
                    f"parse_tour(): malformed D step (expected 'D <left> <right> <mm>'): {step!r}")
            left, right, mm = (float(t) for t in tokens[1:])
            legs.append(TourLeg(kind="distance", value=mm, speed=(abs(left) + abs(right)) / 2.0))
        elif verb == "RT":
            if len(tokens) != 2:
                raise ValueError(f"parse_tour(): malformed RT step (expected 'RT <cdeg>'): {step!r}")
            cdeg = float(tokens[1])
            legs.append(TourLeg(kind="turn", value=cdeg / 100.0))
        else:
            raise ValueError(
                f"parse_tour(): unsupported step verb {verb!r} in step {step!r} -- only "
                f"'D'/'RT' are recognized tour geometry")
    return legs


# ---------------------------------------------------------------------------
# Per-leg / whole-tour results
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TourLegResult:
    """What one leg's own run through the firmware `Move` queue did."""

    index: int
    leg: TourLeg
    outcome: RunOutcome
    heading_before: float | None  # [rad] measured_heading() at this leg's own enqueue, ABSOLUTE
    # since-boot (App::Odometry never resets across a boot session -- see run_tour()'s own docstring)
    heading_after: float | None  # [rad] measured_heading() at this leg's own terminal ack, same caveat
    duration: float  # [s] wall/elapsed time this leg's own wait-for-terminal loop took
    fault: bool  # True iff this leg's own outcome was RunOutcome.FAULT
    tick_count: int  # number of ack-ring polls this leg's own wait loop made


@dataclass(frozen=True)
class TourClosure:
    """Tour-wide pose closure: the measured pose immediately before leg 1's
    own `Move` is sent vs. the measured pose after the final leg's settle
    window. Both poses are read from `TLMFrame.pose` specifically (AC3) --
    the encoder-derived dead-reckoned pose -- independent of whichever
    source (`pose` or `otos`) the caller's `HeadingCorrector` is itself
    configured to read for its own heading_before/heading_after readback.

    `None` fields mean the tour never reached a state where that pose could
    be captured (no telemetry ever arrived before leg 1's own `Move` was
    sent, or the tour stopped before its final leg completed -- see
    `run_tour()`'s own docstring: closure is only computed for a tour that
    runs every leg to `RunOutcome.COMPLETED`)."""

    start_pose: tuple[float, float, float] | None  # (x, y, heading) [mm, mm, rad]
    end_pose: tuple[float, float, float] | None  # (x, y, heading) [mm, mm, rad]
    position_delta: float | None  # [mm] Euclidean distance between end_pose and start_pose x/y
    heading_delta: float | None  # [rad] normalize_angle(end heading - start heading)


@dataclass(frozen=True)
class TourResult:
    """The whole tour's own outcome: every leg's result, in order, plus the
    tour-wide closure. `stopped_at`/`stopped_outcome` are `None` iff every
    leg completed (`RunOutcome.COMPLETED`); otherwise they identify the leg
    index and outcome that ended the tour early -- no leg after it was
    attempted."""

    legs: list[TourLegResult]
    closure: TourClosure
    stopped_at: int | None
    stopped_outcome: RunOutcome | None


# Per-tick trace hook -- ticket 005's bench script uses this to capture a
# full commanded-vs-measured trace without tour.py itself knowing about
# CSV/JSON file formats. Args: (tick_index [monotonically increasing across
# the WHOLE tour], leg_index, leg, this poll's own TickResult (109-008: no
# per-tick v_x/omega any more -- see TickResult's own construction below --
# `done`/`outcome` are still meaningful), the latest drained TLMFrame at
# this poll or None).
RowCallback = Callable[[int, int, "TourLeg", "TickResult", "TLMFrame | None"], None]

# Per-leg narration hook -- ticket 003's TestGUI uses this instead of the
# per-tick hook above (it only wants "[TOUR] leg i/N: ..." progress lines,
# not a full trace). Args: (leg_index, total_legs, leg, this leg's own
# TourLegResult).
OnLegCallback = Callable[[int, int, "TourLeg", TourLegResult], None]


def _frame_pose_rad(frame: "TLMFrame | None") -> tuple[float, float, float] | None:
    """`(x, y, heading)` from `frame.pose`, heading converted centidegrees ->
    radians. `None` if `frame` is `None` or carries no `pose` field (e.g. a
    pre-fault frame, or a build that never populated it)."""
    if frame is None or frame.pose is None:
        return None
    x, y, h_cdeg = frame.pose
    return (float(x), float(y), h_cdeg * _HEADING_SCALE)


def _compute_closure(
    start_pose: tuple[float, float, float] | None,
    end_pose: tuple[float, float, float] | None,
) -> TourClosure:
    if start_pose is None or end_pose is None:
        return TourClosure(start_pose=start_pose, end_pose=end_pose,
                          position_delta=None, heading_delta=None)
    dx = end_pose[0] - start_pose[0]
    dy = end_pose[1] - start_pose[1]
    return TourClosure(
        start_pose=start_pose, end_pose=end_pose,
        position_delta=math.hypot(dx, dy),
        heading_delta=normalize_angle(end_pose[2] - start_pose[2]))


def _move_timeout_for(duration_s: float) -> float:  # [ms]
    """Expected-duration-based `Move.timeout` safety backstop -- see
    `_MOVE_TIMEOUT_FACTOR`/`_MOVE_MIN_TIMEOUT`'s own module-level comment
    (duplicated from `testgui/transport.py`'s identically-named/-valued
    helper, not imported -- see this module's own file header)."""
    return max(_MOVE_MIN_TIMEOUT, duration_s * 1000.0 * _MOVE_TIMEOUT_FACTOR)


def _move_kwargs_for_leg(leg: TourLeg, v_max: float, turn_rate: float,
                         move_id: int, *, replace: bool) -> dict:
    """Translate one parsed `TourLeg` into `MoveTransport.move()`'s own
    kwargs (protocol v4's `Move` schema, `docs/protocol-v4.md` section 4).

    A "distance" leg becomes a straight `MoveTwist(v_x=...)` + a
    `stop_distance` of `abs(leg.value)` -- honoring the leg's own
    wire-authored `speed` as the commanded rate when present (`D`'s own
    left/right speed field), falling back to `v_max` otherwise, same
    fallback rule the pre-109-008 profile path used. A "turn" leg becomes a
    pure in-place pivot `MoveTwist(omega=...)` + a `stop_angle` of
    `radians(abs(leg.value))` -- `RT` carries no rate of its own on the
    wire, so `turn_rate` (this module's own `run_tour()` resolves it from
    `PlannerParams.omega_max`, see the constants block above) supplies it.

    `Move.timeout` (the wire safety backstop, REQUIRED per
    `docs/protocol-v4.md` section 4.1) is sized off this leg's own expected
    duration (`_move_timeout_for()`). `move_id` becomes `Move.id` -- the key
    THIS leg's own COMPLETION ack echoes (never the ENQUEUE ack, which
    echoes the envelope's own `corr_id` instead -- see `_drain_and_poll()`'s
    own docstring). `replace` is the caller's own queue-semantics choice
    (`run_tour()` uses `True` only for the very first leg sent, `False` for
    every subsequent one -- see that function's own docstring for why: it
    is what turns the one-leg lookahead into a real `MoveQueue` enqueue
    rather than a preempt)."""
    if leg.kind == "distance":
        speed = leg.speed if leg.speed else v_max
        if speed <= 0.0:
            raise ValueError(f"run_tour(): non-positive speed for leg {leg!r}")
        v_x = math.copysign(speed, leg.value)
        timeout = _move_timeout_for(abs(leg.value) / speed)
        return dict(v_x=v_x, stop_distance=abs(leg.value), timeout=timeout,
                   replace=replace, id=move_id)
    omega = math.copysign(turn_rate, leg.value)
    angle = math.radians(abs(leg.value))
    timeout = _move_timeout_for(angle / turn_rate)
    return dict(omega=omega, stop_angle=angle, timeout=timeout,
               replace=replace, id=move_id)


def _drain_and_poll(transport: "MoveTransport", move_id: int,
                    latest_frame: list) -> "tuple[TLMFrame, AckEntry] | None":
    """Non-blocking: drain every currently-pending `TLMFrame`, updating
    `latest_frame[0]` to the last one drained (mirrors the pre-109-008
    path's own `ex.latest_frame` bookkeeping), and return the
    `(frame, ack_entry)` pair for the FIRST drained frame that carries an
    ack matching `move_id`, or `None` if none of them carry one yet.

    121-002 (tour-1-final-leg-completes-only-on-stop.md): scans each
    drained frame's bounded `acks` ring (`frame.acks`, `docs/protocol-v4.md`
    section 7.1, 120) FIRST, mirroring `io/serial_conn.py`'s
    `_match_ack_in_frames()`/`SerialConnection.wait_for_ack()` own matching
    policy (first `(frame, ring-entry)` pair, frames in arrival order, ring
    entries oldest-pushed-first) -- that helper cannot be imported directly
    (it scans raw `pb2.ReplyEnvelope` objects off `drain_binary_tlm()`, not
    the already-adapted `TLMFrame`/`AckEntry` dataclasses this module's own
    `MoveTransport.read_pending_binary_tlm_frames()` returns -- the same
    "small reimplementation, not an import" call `io/sim_config.py`'s
    `SimConfigConn.poll_ack()` already made for the identical layering
    reason; see that method's own doc comment), so this is the TLMFrame-
    layer counterpart of the same policy. Falls back to the single
    "freshest ack" scalar slot (`frame.ack`, valid iff `frame.ack_fresh`)
    ONLY when a frame's own ring carries no match -- a real wire frame that
    sets `frame.ack` always pushes the SAME ack onto `frame.acks` in the
    SAME cycle (`docs/protocol-v4.md` section 7.1's "purely additive"
    wire change), so this fallback exists for test doubles that populate
    only the scalar slot (e.g. `test_tour1_geometry.py`'s
    `_FakeTwistTransport`), not for any real frame the ring itself would
    miss.

    Root cause this replaces (planning-time finding, confirmed by
    reproduction -- see this module's own file header / the ticket): reading
    ONLY `frame.ack` meant a `Move`'s own completion ack was observable on
    exactly ONE drained frame; if THAT specific frame was ever dropped (the
    lossy bench link, not Sim, which never drops a frame it produced), the
    completion was invisible forever even though the SAME ack kept riding
    the ring for `kAckRingDepth`-1 more frames after it -- exactly the
    "single ack slot" mechanism the 120 ack ring exists to route around,
    which `wait_for_ack()` already used and this function did not.

    `docs/protocol-v4.md` section 7.2: a `Move`'s own COMPLETION ack echoes
    `Move.id` -- NEVER the enqueue envelope's own `corr_id`, which is a
    DIFFERENT ack (sent earlier, when the command was merely accepted onto
    the queue, not yet finished). Matching on `move_id` here is therefore
    correct and sufficient PROVIDED `move_id` cannot collide with some other
    command's own auto-assigned envelope `corr_id` landing in the same ring/
    slot -- `_TOUR_MOVE_ID_BASE`'s own comment is why that is true for
    every `move_id` `run_tour()` assigns."""
    terminal: "tuple[TLMFrame, AckEntry] | None" = None
    for frame in transport.read_pending_binary_tlm_frames():
        latest_frame[0] = frame
        if terminal is not None:
            continue
        for entry in frame.acks:
            if entry.corr_id == move_id:
                terminal = (frame, entry)
                break
        if terminal is None and frame.ack is not None and frame.ack.corr_id == move_id:
            terminal = (frame, frame.ack)
    return terminal


def _wait_for_move_terminal(transport: "MoveTransport", move_id: int, latest_frame: list,
                            *, timeout: float, poll_interval: float,
                            clock_fn: Callable[[], float],
                            sleep_fn: Callable[[float], None],
                            should_stop: Callable[[], bool] | None,
                            row_callback: RowCallback | None,
                            global_tick_index_box: list, leg_index: int, leg: TourLeg,
                            ) -> "tuple[tuple[TLMFrame, AckEntry] | None, int, bool]":
    """Poll for `move_id`'s own terminal (completion-ack-bearing) frame, up
    to `timeout` seconds, sleeping `poll_interval` between polls. Also polls
    `should_stop()` once per iteration (mirrors the pre-109-008 path's own
    "polled once per tick, not just once per leg" contract) -- on a `True`
    result, stops polling immediately and returns `(None, tick_count, True)`
    (the `True` flag tells the caller to send `transport.stop()` and report
    `RunOutcome.STOPPED`, matching the OLD path's `stop_now()` shape).
    `row_callback`, if given, fires once per poll iteration.

    Returns `(terminal_or_None, tick_count, stop_requested)` --
    `terminal_or_None` is `None` on either a `should_stop()` interrupt or a
    genuine timeout (the caller distinguishes the two via `stop_requested`);
    when not `None`, it is the `(frame, ack_entry)` pair
    `_drain_and_poll()` matched on `move_id` (121-002: the ack ring, not
    only the single scalar slot -- see that function's own docstring) --
    the caller reads `frame`'s own `fault_move_timeout` flag plus
    `ack_entry.ok` (`_outcome_for_terminal_frame()`) to learn whether this
    leg ended via its stop condition or its `Move.timeout` backstop.
    """
    deadline = clock_fn() + timeout
    tick_count = 0
    while True:
        if should_stop is not None and should_stop():
            return None, tick_count, True

        terminal = _drain_and_poll(transport, move_id, latest_frame)
        tick_count += 1
        if row_callback is not None:
            result = TickResult(v_x=0.0, omega=0.0, corr_id=move_id,
                                done=(terminal is not None), outcome=None)
            row_callback(global_tick_index_box[0], leg_index, leg, result, latest_frame[0])
        global_tick_index_box[0] += 1

        if terminal is not None:
            return terminal, tick_count, False
        if clock_fn() >= deadline:
            return None, tick_count, False
        sleep_fn(poll_interval)


def _outcome_for_terminal_frame(frame: "TLMFrame", ack: "AckEntry") -> RunOutcome:
    """Map one leg's own terminal `(frame, ack)` pair (`_drain_and_poll()`'s
    own match on this leg's `Move.id`) onto a `RunOutcome`.

    121-002: `ack` is the SPECIFIC ring entry (or, on the scalar-slot
    fallback, `frame.ack`) `_drain_and_poll()` actually matched -- read
    `ok`/`err_code` off THAT entry, never off `frame.ack` directly, since a
    ring match's own entry and the enclosing frame's "freshest ack" scalar
    slot can genuinely disagree (the frame carrying `move_id`'s completion
    somewhere in its ring is not necessarily the SAME frame whose scalar
    slot is fresh for `move_id` -- it may be fresh for some OTHER, later
    command instead by the time this frame is read). `fault_move_timeout`
    stays a `frame`-level flag (`TLMFrame.flags` bit 15), not part of any
    ack, so it is still read off `frame` directly.

    `docs/protocol-v4.md` section 7.3 (AS-BUILT): a GENUINE completion
    ack's own `ack_err` is UNCONDITIONALLY 0, regardless of whether the
    `Move` ended via its own stop condition or via its `timeout` backstop
    -- the two outcomes are distinguished ONLY by `flags` bit 15
    (`TLMFrame.fault_move_timeout`) on the SAME frame, never by a nonzero
    `ack_err`. A timed-out `Move` is reported as a tour fault, matching the
    "stop immediately on anything but success" contract `run_tour()` has
    always had -- a tour never expects one of its own legs to hit its
    safety-backstop timeout; that only happens if the stop condition (a
    reachable distance/angle) was never met, a real problem worth stopping
    the tour for.

    Defense in depth (turn-prediction-campaign fix): a nonzero `ack_err` on
    the matched entry is ALSO treated as a fault, never `COMPLETED` --
    covers an `ERR_FULL` enqueue-rejection ack that reaches this function
    despite `SimLoop.move()`'s own corr_id/move_id aliasing fix (see that
    method's doc comment), e.g. a future caller that reintroduces the
    aliasing, or a genuinely-colliding `Move.id`/envelope `corr_id` pair.
    `ack_err` is unconditionally 0 on every REAL completion ack (the
    comment above), so this check is a no-op on the happy path and only
    ever fires on an entry that was never a real completion in the first
    place."""
    if not ack.ok:
        return RunOutcome.FAULT
    return RunOutcome.FAULT if frame.fault_move_timeout else RunOutcome.COMPLETED


# ---------------------------------------------------------------------------
# run_tour() -- 109-008: sends one Move per leg, one-leg lookahead so
# firmware's own boundary-velocity carry (ticket 006) can sequence
# compatible same-v_max legs without decelerating to a stop at the boundary.
# ---------------------------------------------------------------------------


def run_tour(
    transport: "MoveTransport",
    params: "PlannerParams",
    heading: "HeadingCorrector",
    legs: Sequence[TourLeg],
    *,
    v_max: float = DEFAULT_V_MAX,
    omega_max: float | None = None,  # [rad/s] turn-leg yaw rate override; None (the default,
    # every existing caller) resolves to params.omega_max -- see the constants block above for why
    # this is no longer simply "UNUSED".
    final_settle: float = DEFAULT_FINAL_SETTLE,
    move_timeout: float = DEFAULT_MOVE_TIMEOUT,
    poll_interval: float = DEFAULT_POLL_INTERVAL,
    row_callback: RowCallback | None = None,
    on_leg: OnLegCallback | None = None,
    should_stop: Callable[[], bool] | None = None,
    clock_fn: Callable[[], float] = time.monotonic,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> TourResult:
    """Run every `TourLeg` in `legs`, in order, by sending one `Move` command
    per leg to `transport` and waiting for each one's own terminal
    (completion-ack-bearing) frame.

    Stops immediately -- no further legs attempted -- the instant any leg's
    own outcome is anything other than `RunOutcome.COMPLETED` (a fault or an
    external `should_stop()` request); the returned `TourResult.stopped_at`/
    `stopped_outcome` identify which leg and why. `should_stop`, if given, is
    polled once per ack-slot poll (not just once per leg) so a caller
    (ticket 003's `_TourRunner.stop()`) can interrupt mid-leg -- on a `True`
    result `transport.stop()` is called (flushes the firmware queue and
    stops immediately, same as the pre-109-008 path's `stop_now()`) and the
    leg is reported `RunOutcome.STOPPED`.

    One-leg lookahead (SUC-003): leg N+1's own `Move` is sent immediately
    after leg N's (while leg N is still active, not after it completes) --
    the ONE piece of host-side sequencing this function still does -- so
    firmware's own `App::MoveQueue` (1 active + 4 pending,
    `docs/protocol-v4.md` section 5.1) can carry velocity through a
    compatible boundary instead of decelerating to a stop: the FIRST leg is
    sent `replace=True` (starts immediately, matching every other "just
    drive this" caller in this tree); every subsequent leg is sent
    `replace=False` (enqueued behind whichever leg is currently active, a
    real `MoveQueue` enqueue, never a preempt). Everything past that is
    event-driven: this function does not compute a profile, does not time a
    settle window between legs, and does not poll raw fault bits -- see
    this module's own file header for why that last point is also this
    ticket's own resolution of `tour1-freeze-investigation-2026-07-15.md`.

    Tour closure: the measured pose (`TLMFrame.pose`, via `_frame_pose_rad()`)
    is captured once, from whatever telemetry is already pending immediately
    before leg 1's own `Move` is sent (retried a bounded few times if the
    queue is momentarily empty -- mirrors the pre-109-008 path's own
    `begin()` retry), and once more after the FINAL leg's settle window
    (only when every leg completes -- see `TourClosure`'s own docstring for
    why an early-stopped tour reports `None` closure fields). `App::Odometry`
    never resets across a boot session, so both readings -- and their delta
    -- are always RELATIVE to each other, never an absolute zero.

    `row_callback`/`on_leg` are optional, independent extension points (see
    their own type docstrings above) -- a caller that wants neither may omit
    both.
    """
    if not legs:
        raise ValueError("run_tour(): legs must be non-empty")

    turn_rate = omega_max if omega_max else params.omega_max
    if turn_rate <= 0.0:
        raise ValueError(f"run_tour(): turn-leg yaw rate must be > 0, got {turn_rate!r}")

    # Distinct, incrementing Move.id per leg (see _TOUR_MOVE_ID_BASE's own
    # comment for why this range is safe against a corr_id collision). The
    # first leg preempts (replace=True); every later leg enqueues behind the
    # one before it (replace=False) -- see this function's own docstring.
    move_ids = [_TOUR_MOVE_ID_BASE + i for i in range(len(legs))]
    move_kwargs = [
        _move_kwargs_for_leg(leg, v_max, turn_rate, move_ids[i], replace=(i == 0))
        for i, leg in enumerate(legs)
    ]
    latest_frame: list = [None]
    global_tick_index_box = [0]

    def send_leg(i: int) -> None:
        transport.move(**move_kwargs[i])

    # Capture the start pose from whatever's already pending, bounded retry
    # (mirrors StreamingExecutor.begin()'s own rationale: a single
    # read_pending_binary_tlm_frames() call can race an idle ~25Hz queue).
    start_pose: tuple[float, float, float] | None = None
    for _ in range(_BEGIN_DRAIN_RETRIES):
        for frame in transport.read_pending_binary_tlm_frames():
            latest_frame[0] = frame
        start_pose = _frame_pose_rad(latest_frame[0])
        if start_pose is not None:
            break
        sleep_fn(_BEGIN_DRAIN_RETRY_INTERVAL)

    send_leg(0)
    if len(legs) > 1:
        send_leg(1)  # one-leg lookahead -- see this function's own docstring

    leg_results: list[TourLegResult] = []
    end_pose: tuple[float, float, float] | None = None
    stopped_at: int | None = None
    stopped_outcome: RunOutcome | None = None

    for index, leg in enumerate(legs):
        heading_before = heading.measured_heading(latest_frame[0])
        leg_start = clock_fn()

        logger.info("run_tour(): leg %d/%d (%s, value=%r) starting (Move id=%s)",
                   index + 1, len(legs), leg.kind, leg.value, move_ids[index])

        terminal, tick_count, stop_requested = _wait_for_move_terminal(
            transport, move_ids[index], latest_frame, timeout=move_timeout,
            poll_interval=poll_interval, clock_fn=clock_fn, sleep_fn=sleep_fn,
            should_stop=should_stop, row_callback=row_callback,
            global_tick_index_box=global_tick_index_box, leg_index=index, leg=leg)

        if stop_requested:
            logger.warning("run_tour(): should_stop() requested mid-leg %d/%d -- stopping now",
                          index + 1, len(legs))
            transport.stop()
            outcome = RunOutcome.STOPPED
        elif terminal is None:
            logger.error("run_tour(): leg %d/%d timed out waiting for Move id=%s terminal ack",
                        index + 1, len(legs), move_ids[index])
            outcome = RunOutcome.FAULT
        else:
            outcome = _outcome_for_terminal_frame(*terminal)

        heading_after = heading.measured_heading(latest_frame[0])
        duration = clock_fn() - leg_start

        leg_result = TourLegResult(
            index=index, leg=leg, outcome=outcome, heading_before=heading_before,
            heading_after=heading_after, duration=duration,
            fault=(outcome == RunOutcome.FAULT), tick_count=tick_count)
        leg_results.append(leg_result)
        logger.info("run_tour(): leg %d/%d outcome=%s polls=%d duration=%.2fs",
                   index + 1, len(legs), outcome.value, tick_count, duration)

        if on_leg is not None:
            on_leg(index, len(legs), leg, leg_result)

        if outcome != RunOutcome.COMPLETED:
            stopped_at = index
            stopped_outcome = outcome
            logger.error("run_tour(): stopping -- leg %d/%d ended with outcome=%s, no further "
                        "legs attempted", index + 1, len(legs), outcome.value)
            break

        # Keep exactly one leg queued ahead of the active one (see this
        # function's own docstring) -- send leg index+2 now that leg
        # `index` (the one two ahead of the NEXT one to enqueue) has
        # completed and leg `index+1` is the new active command.
        if index + 2 < len(legs):
            send_leg(index + 2)

        if index == len(legs) - 1:
            # Final leg -- settle window before capturing the tour's own
            # closure end pose (AC3): the terminal ack already confirmed
            # DONE, but the plant needs real time to physically settle.
            sleep_fn(final_settle)
            for frame in transport.read_pending_binary_tlm_frames():
                latest_frame[0] = frame
            end_pose = _frame_pose_rad(latest_frame[0])

    closure = _compute_closure(start_pose, end_pose)
    return TourResult(legs=leg_results, closure=closure, stopped_at=stopped_at,
                      stopped_outcome=stopped_outcome)
