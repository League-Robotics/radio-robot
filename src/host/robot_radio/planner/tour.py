"""robot_radio.planner.tour -- tour geometry ownership + chained execution.

Sprint 107 ticket 002 (SUC-033, `architecture-update.md` Decision 3, corrected
during that document's own self-review to keep the dependency direction
`[Presentation] -> [Domain]`, not the reverse). `TOUR_1`/`TOUR_2` used to live
in `testgui/commands.py` as raw firmware wire strings (`D`/`RT`) built for the
now-deleted `Motion::SegmentExecutor` (sprint 102/103). The GEOMETRY those
wire strings encode is still a real, tuned, reusable asset -- this module
OWNS that data and parses it into typed legs.

109-008 (host adoption of sprint 109's `Move` wire message): `run_tour()` no
longer builds a `profile.py` setpoint sequence and streams it through a
`planner.executor.StreamingExecutor` -- DISTANCE/pivot trajectory planning
(jerk-limited profile shape, heading PD closure) now lives ENTIRELY in
firmware (`Motion::Executor`/`App::Pilot`, sprint 109 tickets 003-006).
`run_tour()` instead sends ONE `Move` command per leg (`transport.move()`)
and lets the firmware's own fixed-depth command queue and boundary-velocity
carry (ticket 006) sequence them -- enqueueing the NEXT leg while the CURRENT
one is still active (one-command lookahead) so two compatible same-`v_max`
legs carry velocity through their shared boundary instead of decelerating to
a stop, per sprint 109's SUC-003. Per-leg completion is driven by the
command's own completion EVENT, not a host-timed settle delay or a
host-computed profile exhaustion check -- this is also sprint 109 ticket
008's own resolution of `tour1-freeze-investigation-2026-07-15.md`: the old
streaming path's `tick()` polled raw `fault_bits` every cadence tick and
stopped the WHOLE tour the instant any bit was set (including a transient,
firmware-self-recovered blip); this path has no such polling at all -- the
ONLY thing that ends a leg is that leg's OWN `Move` reaching a terminal
state, so a transient fault bit that firmware's own `MotorArmor` recovers
from without aborting the active command has no way to freeze/stop a tour
that would otherwise complete.

**Ported (2026-07-22, `testgui-motion-paths-dead-after-move-cutover.md`)
onto protocol v4's `Move`/single-ack-slot wire shape** (sprint 115/116's
frame-v2 rewrite + MOVE protocol cutover, `docs/protocol-v4.md`) -- this
module's own body had gone dormant (raised `AttributeError` at import,
referencing `telemetry_pb2.ACK_STATUS_DONE` and the pre-115 depth-3
`AckEntry` ring, both deleted). The 109-008 "one `Move` per leg, one-leg
lookahead, event-driven completion" SHAPE described above is unchanged and
still exactly what this module does; only the WIRE mechanics changed:
- `MoveTransport.move()`'s own kwargs are now the current `Move` schema
  (`v_x`/`omega`/`stop_distance`/`stop_angle`/`timeout`/`replace`/`id` --
  see that Protocol's own docstring below), not the deleted sprint-109 arc
  shape (`distance`/`delta_heading`/`v_max`/`omega`/`time`).
- There is no more `AckStatus` taxonomy (`DONE`/`TRIVIAL`/`SUPERSEDED`/
  `FLUSHED`/`TIMEOUT`/`SOLVE_FAIL`) and no depth-3 ack ring. `Telemetry` now
  carries a SINGLE ack slot (`ack_corr`/`ack_err`, `TLMFrame.ack`) that
  rides EITHER a command's ENQUEUE ack (echoing the envelope's own
  `corr_id`) OR a `Move`'s own COMPLETION ack (echoing `Move.id` instead --
  `docs/protocol-v4.md` section 7.2) -- never both for the same command.
  `_drain_and_poll()`/`_wait_for_move_terminal()` below poll for a frame
  whose ack slot matches THIS leg's own `Move.id` (never the corr_id
  `transport.move()` itself might return), and `_outcome_for_terminal_frame()`
  reads the SAME frame's `fault_move_timeout` flag (bit 15) to distinguish a
  stop-condition completion from a timeout ending -- the completion ack's
  own `ack_err` is unconditionally 0 either way (protocol-v4.md section
  7.3, AS-BUILT), so it carries no outcome information by itself.

`planner.executor.StreamingExecutor`/`planner.profile` themselves are
UNCHANGED and UNTOUCHED by this ticket -- only TOURS (this module) moved to
the MOVE-queue path. No live gamepad/teleop UI is wired into this checkout
today (nothing in `testgui/` currently constructs a `StreamingExecutor`
outside this module and `tests/bench/`'s own bench scripts); this ticket's
"demote the host planner to teleop input shaping only" description refers to
ticket 003's `TIMED` `Move` wire primitive existing for a FUTURE gamepad/
teleop consumer to use, not a live one being preserved here -- there is no
DISTANCE/pivot planning logic left in `host/robot_radio/planner/` for THIS
module to keep or remove either way (this module never built its own
profile/queue logic beyond calling `profile.py`/`executor.py`, both of which
stay exactly as they were).

This is the single, shared per-leg execution loop both the TestGUI
(`testgui/__main__.py`'s `_TourRunner`, ticket 003) and the bench script
(`tests/bench/tour_bench_run.py`, ticket 005) call -- no duplicated per-leg
execution/telemetry-capture logic between them.

Boundary (architecture-update.md's own words, updated 109-008): inside -- the
geometry data itself, parsing it into typed leg specs, the per-leg run loop
(build a `Move` per leg, enqueue one leg ahead, poll the ack ring for each
leg's own completion event, capture the measured pose before leg 1 and after
the final leg), and closure-delta computation. Outside -- the `Move`
encoding/wire round trip itself (calls `transport.move()`/`transport.
read_pending_binary_tlm_frames()`, never builds a `CommandEnvelope` itself),
heading readback (calls `heading.py` indirectly, read-only -- there is no
heading PD to run host-side any more, firmware owns that closed loop), the
wire/transport itself (accepts a `MoveTransport`-compatible object from the
caller -- never imports `NezhaProtocol`/`SerialConnection`/`SimConnection`
directly), and any GUI or trace-file-format concern (the optional
`row_callback`/`on_leg` hooks are the only surface offered to a caller that
wants a trace or narration -- this module itself writes nothing to disk and
imports no Qt).

Usage
-----
    from robot_radio.planner.tour import TOUR_1, parse_tour, run_tour

    legs = parse_tour(TOUR_1)
    result = run_tour(transport, params, heading, legs)
    print(result.closure.position_delta, result.closure.heading_delta)

No hardware, no sim transport required for this module's own unit-test gate
-- see `tests/unit/test_planner_tour.py`'s `FakeTransport` double convention
(mirrors `tests/unit/test_planner_executor.py`'s own).
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
    from robot_radio.robot.protocol import TLMFrame

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
    safety backstop. `id` doubles as the envelope's own `corr_id` (the
    ENQUEUE ack's correlation key) and `Move.id` (the later COMPLETION
    ack's key, per `docs/protocol-v4.md` section 7.2) for `SimLoop`; a real
    `NezhaProtocol.move()` keeps the two independent (its own envelope
    `corr_id` is auto-assigned by the connection, distinct from `id`/
    `Move.id`) -- either way, THIS module only ever polls for `id`'s own
    COMPLETION ack (see `_drain_and_poll()`), never the enqueue ack, so the
    distinction is transparent to `run_tour()`. Both a real `NezhaProtocol`
    (`.move()` added by this same fix) and a `robot_radio.io.sim_loop.SimLoop`
    satisfy this Protocol as-is -- no adapter needed in production.
    """

    def move(self, *, v_x: float = 0.0, v_y: float = 0.0, omega: float = 0.0,
             v_left: "float | None" = None, v_right: "float | None" = None,
             stop_time: "float | None" = None, stop_distance: "float | None" = None,
             stop_angle: "float | None" = None, timeout: float,
             replace: bool = True, id: "int | None" = None) -> int: ...

    def stop(self) -> int: ...

    def read_pending_binary_tlm_frames(self) -> "list[TLMFrame]": ...


# TLMFrame.pose's heading is integer centidegrees (matches `heading.py`'s own
# `_HEADING_SCALE` convention -- duplicated here, not imported, since that
# name is `heading.py`'s own private module constant; AC3 pins tour closure
# specifically to `TLMFrame.pose`, independent of whichever source (`pose` or
# `otos`) the caller's `HeadingCorrector` happens to be configured to read
# for its own per-tick trim -- see this module's own `_frame_pose_rad()`).
_HEADING_SCALE = math.pi / 18000.0  # [rad/cdeg]


# ---------------------------------------------------------------------------
# Tour geometry -- moved verbatim from testgui/commands.py (Decision 3)
# ---------------------------------------------------------------------------
#
# A "tour" is an ordered list of firmware wire strings -- the legacy `D`/`RT`
# steps a pre-102 GUI sent one at a time. The wire strings themselves are no
# longer sent as-is (both verbs are retired -- see this module's own header);
# `parse_tour()` below reads the SAME geometry data and turns it into typed
# `TourLeg`s that `run_tour()` drives through profile.py/executor.py instead.
# Values copied byte-for-byte from testgui/commands.py -- same leg distances/
# angles, same order (regression-protected directly against this data by
# `tests/unit/test_planner_tour.py`).

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
# protocol v4). `a_max`/`alpha_max`/`cadence` from the pre-109-008
# profile-streaming path are GONE -- DISTANCE/pivot accel/jerk ceilings are
# entirely firmware's own `Motion::Executor` config now, not a per-command
# wire value or a host-computed profile.py shape. `v_max` survives: it sizes
# a straight leg's own `MoveTwist.v_x` when the leg carries no wire-authored
# speed of its own. `omega_max` also survives, repurposed: `RT` carries no
# rate of its own on the wire (unlike `D`, whose left/right speed IS the
# leg's rate), so `run_tour()` needs a host-picked constant turn rate for
# every turn leg's own `MoveTwist.omega` -- `PlannerParams.omega_max` (this
# module's own `params` argument) is that constant, matching
# `testgui/transport.py`'s own `_UNMANAGED_YAW_RATE` (2.0 rad/s, same
# numeric default) used for the SAME D/RT dispatch on both other tour-less
# call paths (`_dispatch_managed_move()`/`_build_sim_move()`) -- duplicated
# rather than imported (`planner/` must never import `testgui/`, the
# `[Presentation] -> [Domain]` dependency direction this module's own header
# explains).
# ---------------------------------------------------------------------------

DEFAULT_V_MAX = 150.0  # [mm/s] straight-leg linear ceiling fallback (no per-leg speed on the D step)

DEFAULT_INTER_LEG_SETTLE = 1.0  # [s] UNUSED this ticket -- retained only for run_tour()'s own
# call-signature back-compat with existing callers (tests/bench/tour_bench_run.py passes it as a
# kwarg). Pre-109-008 this was a host-timed gap between two legs' own StreamingExecutor runs
# (retuned 0.3 -> 1.0 by ticket 107-005's bench session after a kFaultWedgeLatch trip at the
# straight->turn boundary -- see tour1-freeze-investigation-2026-07-15.md). The MOVE-queue path
# has no equivalent host-timed gap between QUEUED legs at all: firmware's own boundary-velocity
# carry (ticket 006) sequences the transition, and a transient fault bit no longer stops the tour
# on its own (see this module's own file header) -- there is nothing left for a host-side sleep to
# protect against between two enqueued legs.
DEFAULT_FINAL_SETTLE = 0.6  # [s] post-terminal-DONE settle window before capturing the tour's own
# closure end pose -- the final leg's own completion event fires the instant the ENCODER-relative
# distance criterion is met, but the PLANT needs a little more real time to physically settle
# (dwell) before its reported pose reflects where it actually stopped; matches the pre-109-008
# path's own settle-window convention and value.
DEFAULT_MOVE_TIMEOUT = 15.0  # [s] bound on how long run_tour() waits for one leg's own terminal
# completion ack before giving up and reporting RunOutcome.FAULT itself -- this wait is ALWAYS
# bounded, mirroring NezhaProtocol.wait_for_ack()'s own "never infinite" contract; a real leg is
# expected to finish in well under this (TOUR_1's longest leg is 850mm at a ~150mm/s ceiling, a
# few seconds; TOUR_2's widest turn is ~217deg at omega_max=2.0rad/s, under 2s).

# `Move.timeout` -- the WIRE safety backstop (docs/protocol-v4.md section 4.1: REQUIRED, <=0 is
# ERR_BADARG) -- is distinct from DEFAULT_MOVE_TIMEOUT above (this module's own HOST-side poll
# bound). Sized off each leg's own expected duration, same factor/floor
# testgui/transport.py's `_move_timeout_for()` uses for the SAME D/RT dispatch shape (duplicated,
# not imported -- see the constants block above for why).
_MOVE_TIMEOUT_FACTOR = 3.0    # [multiple of expected duration]
_MOVE_MIN_TIMEOUT = 2000.0    # [ms]

# `Move.id` values `run_tour()` assigns to its own legs (see `_move_kwargs_for_leg()`) -- offset
# far above any realistic session-scoped envelope `corr_id` (`SerialConnection._corr_counter`/
# `SimLoop._corr_id`, both start at 0 and increment by 1 per command sent this session) so a leg's
# own COMPLETION ack (which echoes `Move.id`) can never be mistaken for some OTHER command's
# ENQUEUE ack (which echoes the auto-assigned envelope `corr_id`) landing in the single ack slot
# while this leg's completion is being polled for -- see `_drain_and_poll()`'s own docstring for
# why that collision would be a false-positive "leg complete".
_TOUR_MOVE_ID_BASE = 1 << 20
DEFAULT_POLL_INTERVAL = 0.05  # [s] sleep between two consecutive ack-ring polls while waiting for
# a leg's own terminal status (mirrors the pre-109-008 path's own tick cadence order of magnitude).

_BEGIN_DRAIN_RETRIES = 5  # mirrors executor.py's StreamingExecutor.begin() own bounded retry --
_BEGIN_DRAIN_RETRY_INTERVAL = 0.05  # [s] same rationale: a single read_pending_binary_tlm_frames()
# call can legitimately race an idle queue between two ~25Hz pushes and come back empty.


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
                    latest_frame: list) -> "TLMFrame | None":
    """Non-blocking: drain every currently-pending `TLMFrame`, updating
    `latest_frame[0]` to the last one drained (mirrors the pre-109-008
    path's own `ex.latest_frame` bookkeeping), and return the FIRST drained
    frame whose ack slot is fresh AND matches `move_id`, or `None` if none
    of them carry one yet.

    `docs/protocol-v4.md` section 7.2: a `Move`'s own COMPLETION ack echoes
    `Move.id` (`frame.ack.corr_id`, valid iff `frame.ack_fresh`/
    `frame.ack is not None`) -- NEVER the enqueue envelope's own `corr_id`,
    which is a DIFFERENT ack (sent earlier, when the command was merely
    accepted onto the queue, not yet finished). Matching on `move_id` here
    is therefore correct and sufficient PROVIDED `move_id` cannot collide
    with some other command's own auto-assigned envelope `corr_id` landing
    in the same single ack slot -- `_TOUR_MOVE_ID_BASE`'s own comment is why
    that is true for every `move_id` `run_tour()` assigns."""
    terminal: "TLMFrame | None" = None
    for frame in transport.read_pending_binary_tlm_frames():
        latest_frame[0] = frame
        if terminal is None and frame.ack is not None and frame.ack.corr_id == move_id:
            terminal = frame
    return terminal


def _wait_for_move_terminal(transport: "MoveTransport", move_id: int, latest_frame: list,
                            *, timeout: float, poll_interval: float,
                            clock_fn: Callable[[], float],
                            sleep_fn: Callable[[float], None],
                            should_stop: Callable[[], bool] | None,
                            row_callback: RowCallback | None,
                            global_tick_index_box: list, leg_index: int, leg: TourLeg,
                            ) -> "tuple[TLMFrame | None, int, bool]":
    """Poll for `move_id`'s own terminal (completion-ack-bearing) frame, up
    to `timeout` seconds, sleeping `poll_interval` between polls. Also polls
    `should_stop()` once per iteration (mirrors the pre-109-008 path's own
    "polled once per tick, not just once per leg" contract) -- on a `True`
    result, stops polling immediately and returns `(None, tick_count, True)`
    (the `True` flag tells the caller to send `transport.stop()` and report
    `RunOutcome.STOPPED`, matching the OLD path's `stop_now()` shape).
    `row_callback`, if given, fires once per poll iteration.

    Returns `(terminal_frame_or_None, tick_count, stop_requested)` --
    `terminal_frame_or_None` is `None` on either a `should_stop()` interrupt
    or a genuine timeout (the caller distinguishes the two via
    `stop_requested`); when not `None`, it is the drained `TLMFrame` whose
    ack slot matched `move_id` -- the caller reads its own
    `fault_move_timeout` flag (`_outcome_for_terminal_frame()`) to learn
    whether this leg ended via its stop condition or its `Move.timeout`
    backstop.
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


def _outcome_for_terminal_frame(frame: "TLMFrame") -> RunOutcome:
    """Map one leg's own terminal frame (the frame `_drain_and_poll()`
    matched on this leg's `Move.id`) onto a `RunOutcome`.

    `docs/protocol-v4.md` section 7.3 (AS-BUILT): the completion ack's own
    `ack_err` is UNCONDITIONALLY 0, regardless of whether the `Move` ended
    via its own stop condition or via its `timeout` backstop -- the two
    outcomes are distinguished ONLY by `flags` bit 15
    (`TLMFrame.fault_move_timeout`) on the SAME frame, never by a nonzero
    `ack_err`. A timed-out `Move` is reported as a tour fault, matching the
    "stop immediately on anything but success" contract `run_tour()` has
    always had -- a tour never expects one of its own legs to hit its
    safety-backstop timeout; that only happens if the stop condition (a
    reachable distance/angle) was never met, a real problem worth stopping
    the tour for."""
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
    a_max: float = 0.0,  # UNUSED (109-008) -- kept for call-signature back-compat, see file header
    omega_max: float | None = None,  # [rad/s] turn-leg yaw rate override; None (the default,
    # every existing caller) resolves to params.omega_max -- see the constants block above for why
    # this is no longer simply "UNUSED".
    alpha_max: float = 0.0,  # UNUSED (109-008), see file header
    cadence: float | None = None,  # UNUSED (109-008), see file header
    inter_leg_settle: float = DEFAULT_INTER_LEG_SETTLE,  # UNUSED (109-008), see file header
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
            outcome = _outcome_for_terminal_frame(terminal)

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
