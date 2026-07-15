"""robot_radio.planner.tour -- tour geometry ownership + chained execution.

Sprint 107 ticket 002 (SUC-033, `architecture-update.md` Decision 3, corrected
during that document's own self-review to keep the dependency direction
`[Presentation] -> [Domain]`, not the reverse). `TOUR_1`/`TOUR_2` used to live
in `testgui/commands.py` as raw firmware wire strings (`D`/`RT`) built for the
now-deleted `Motion::SegmentExecutor` (sprint 102/103). The GEOMETRY those
wire strings encode is still a real, tuned, reusable asset -- this module
OWNS that data, parses it into typed legs, and chains each leg through a
`planner.executor.StreamingExecutor` (ticket 001's fault-baseline-exclusion
and heading-gain fixes already applied there), so the geometry is drivable
again without resurrecting the deleted segment/replace envelope arms.

This is the single, shared per-leg execution loop both the TestGUI
(`testgui/__main__.py`'s `_TourRunner`, ticket 003) and the bench script
(`tests/bench/tour_bench_run.py`, ticket 005) call -- no duplicated per-leg
execution/telemetry-capture logic between them.

Boundary (architecture-update.md's own words): inside -- the geometry data
itself, parsing it into typed leg specs, the per-leg run loop (build a
`profile.py` sequence, run it through the caller-supplied executor, capture
the measured pose before leg 1 and after the final leg), and closure-delta
computation. Outside -- profile math (calls `profile.py`), pacing/safety/
preemption (calls `executor.py`, never reimplements a binding requirement),
heading correction (calls `heading.py` indirectly via the executor it is
handed), the wire/transport itself (accepts a `TwistTransport`-compatible
object from the caller, same as `executor.py` -- never imports
`NezhaProtocol`/`SerialConnection`/`SimConnection` directly), and any GUI or
trace-file-format concern (the optional `row_callback`/`on_leg` hooks are the
only surface offered to a caller that wants a trace or narration -- this
module itself writes nothing to disk and imports no Qt).

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
from typing import TYPE_CHECKING, Callable, Literal, Sequence

from robot_radio.controllers.pid import normalize_angle
from robot_radio.planner.executor import RunOutcome, RunState, StreamingExecutor
from robot_radio.planner.profile import ProfileLimits, profile_for_distance, profile_for_turn

if TYPE_CHECKING:
    from robot_radio.planner.executor import TickResult, TwistTransport
    from robot_radio.planner.heading import HeadingCorrector
    from robot_radio.planner.model import PlannerParams
    from robot_radio.robot.protocol import TLMFrame

logger = logging.getLogger(__name__)

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
# Bench-safe defaults for legs whose wire string carries no rate/accel field
# -- matches tests/bench/profiled_motion_verify.py's own DEFAULT_* constants
# (106-006), well under PlannerParams' own hard ceilings (v_max=200mm/s,
# omega_max=2.0rad/s -- executor.py's own defense-in-depth ceiling clamp
# caps anything unsafe regardless).
# ---------------------------------------------------------------------------

DEFAULT_V_MAX = 150.0  # [mm/s] straight-leg cruise velocity fallback (no per-leg speed on the D step)
DEFAULT_A_MAX = 400.0  # [mm/s^2] straight-leg accel/decel
DEFAULT_OMEGA_MAX = 1.0  # [rad/s] turn-leg cruise rate -- RT carries no rate field, always this default,
# NOT PlannerParams' own more aggressive omega_max=2.0 hard ceiling.
DEFAULT_ALPHA_MAX = 3.0  # [rad/s^2] turn-leg accel/decel, same rationale as DEFAULT_OMEGA_MAX.

DEFAULT_INTER_LEG_SETTLE = 1.0  # [s] gap between two legs' own run loops, giving the plant real
# time to decelerate before the NEXT leg's begin() re-baselines telemetry. Retuned from 0.3 ->
# 1.0 by ticket 107-005's own bench session (Completion Notes, "Bench findings" #1): the 0.3s
# value reproduced a kFaultWedgeLatch trip at the straight->turn boundary on the FIRST turn leg
# (traces 20260715T201348Z/20260715T201419Z) -- 0/N repeats after widening to 1.0s. This is the
# GUI-driven default too (testgui/__main__.py's `_TourRunner` calls `run_tour()` with no
# `inter_leg_settle` override), so the production default must match the bench-proven value, not
# just the bench script's own CLI override.
DEFAULT_FINAL_SETTLE = 0.6  # [s] post-terminal-stop settle window before capturing the tour's own
# closure end pose -- matches profiled_motion_verify.py's own settle-window convention (106-006):
# the terminal tick() sends stop() and returns immediately, but the PLANT needs real time to
# actually decelerate before its reported pose reflects where it actually stopped.


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
    """What one leg's own run through the executor did."""

    index: int
    leg: TourLeg
    outcome: RunOutcome
    heading_before: float | None  # [rad] measured_heading() at this leg's own begin(), ABSOLUTE
    # since-boot (App::Odometry never resets across a boot session -- see run_tour()'s own docstring)
    heading_after: float | None  # [rad] measured_heading() at this leg's own run end, same caveat
    duration: float  # [s] wall/elapsed time this leg's own tick loop took (excludes inter-leg settle)
    fault: bool  # True iff this leg's own outcome was RunOutcome.FAULT
    tick_count: int  # number of tick() calls this leg's own run loop made


@dataclass(frozen=True)
class TourClosure:
    """Tour-wide pose closure: the measured pose immediately before leg 1's
    `begin()` vs. the measured pose after the final leg's settle window.
    Both poses are read from `TLMFrame.pose` specifically (AC3) -- the
    encoder-derived dead-reckoned pose -- independent of whichever source
    (`pose` or `otos`) the caller's `HeadingCorrector` is itself configured
    to read for its own per-tick trim.

    `None` fields mean the tour never reached a state where that pose could
    be captured (no telemetry ever arrived at leg 1's begin(), or the tour
    stopped before its final leg completed -- see `run_tour()`'s own
    docstring: closure is only computed for a tour that runs every leg to
    `RunOutcome.COMPLETED`)."""

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
# the WHOLE tour, matching profiled_motion_verify.py's own CSV column
# convention], leg_index, leg, this tick's TickResult, the latest drained
# TLMFrame at this tick or None).
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


# ---------------------------------------------------------------------------
# run_tour() -- chains every leg through a StreamingExecutor
# ---------------------------------------------------------------------------


def run_tour(
    transport: "TwistTransport",
    params: "PlannerParams",
    heading: "HeadingCorrector",
    legs: Sequence[TourLeg],
    *,
    v_max: float = DEFAULT_V_MAX,
    a_max: float = DEFAULT_A_MAX,
    omega_max: float = DEFAULT_OMEGA_MAX,
    alpha_max: float = DEFAULT_ALPHA_MAX,
    cadence: float | None = None,
    inter_leg_settle: float = DEFAULT_INTER_LEG_SETTLE,
    final_settle: float = DEFAULT_FINAL_SETTLE,
    row_callback: RowCallback | None = None,
    on_leg: OnLegCallback | None = None,
    should_stop: Callable[[], bool] | None = None,
    clock_fn: Callable[[], float] = time.monotonic,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> TourResult:
    """Run every `TourLeg` in `legs`, in order, through one
    `StreamingExecutor` built from `transport`/`params`/`heading`.

    Stops immediately -- no further legs attempted -- the instant any leg's
    own outcome is anything other than `RunOutcome.COMPLETED` (a fault, a
    bounded-overshoot trip, or an external `should_stop()` request); the
    returned `TourResult.stopped_at`/`stopped_outcome` identify which leg and
    why. `should_stop`, if given, is polled once per tick (not just once per
    leg) so a caller (ticket 003's `_TourRunner.stop()`) can interrupt mid-leg,
    not only at a leg boundary -- on a `True` result the current leg is
    stopped via `StreamingExecutor.stop_now()` (an immediate stop, no
    replan -- executor.py's own binding requirement #4) and reported with
    `RunOutcome.STOPPED`.

    Each leg's own profile is built via `profile_for_distance()`/
    `profile_for_turn()`: a "distance" leg honors its own wire-authored
    `TourLeg.speed` as `v_max` when present (falling back to this function's
    own `v_max` argument otherwise) -- `executor.py`'s own defense-in-depth
    ceiling clamp already caps anything unsafe, so faithfully preserving the
    tour's authored speed intent costs nothing extra in safety; a "turn" leg
    always uses `omega_max`/`alpha_max` (RT carries no rate field on the
    wire) -- bench-safe defaults, NOT `PlannerParams`' own more aggressive
    hard ceilings.

    Tour closure: the measured pose (`TLMFrame.pose`, via `_frame_pose_rad()`)
    is captured once, immediately after leg 1's own `begin()` call (before
    that leg's tick loop runs -- effectively "immediately before leg 1's
    begin()", and reuses `begin()`'s own bounded-retry telemetry drain rather
    than duplicating it), and once more after the FINAL leg's settle window
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

    effective_cadence = cadence if cadence is not None else params.streaming_interval
    ex = StreamingExecutor(transport, params, heading, clock_fn=clock_fn, sleep_fn=sleep_fn)

    leg_results: list[TourLegResult] = []
    start_pose: tuple[float, float, float] | None = None
    end_pose: tuple[float, float, float] | None = None
    stopped_at: int | None = None
    stopped_outcome: RunOutcome | None = None
    global_tick_index = 0

    for index, leg in enumerate(legs):
        if leg.kind == "distance":
            axis = "linear"
            limits = ProfileLimits(v_max=leg.speed if leg.speed else v_max, a_max=a_max)
            setpoints = profile_for_distance(leg.value, limits, cadence=effective_cadence)
            target = leg.value
        else:
            axis = "angular"
            limits = ProfileLimits(v_max=omega_max, a_max=alpha_max)
            angle_rad = math.radians(leg.value)
            setpoints = profile_for_turn(angle_rad, limits, cadence=effective_cadence)
            target = angle_rad

        ex.begin(setpoints, target=target, axis=axis)
        if index == 0:
            start_pose = _frame_pose_rad(ex.latest_frame)
        heading_before = heading.measured_heading(ex.latest_frame)
        leg_start = clock_fn()

        logger.info("run_tour(): leg %d/%d (%s, value=%r) starting",
                   index + 1, len(legs), leg.kind, leg.value)

        outcome: RunOutcome | None = None
        tick_count = 0
        while ex.state == RunState.RUNNING:
            if should_stop is not None and should_stop():
                logger.warning("run_tour(): should_stop() requested mid-leg %d/%d -- stopping now",
                              index + 1, len(legs))
                ex.stop_now()
                outcome = RunOutcome.STOPPED
                break
            result = ex.tick()
            if row_callback is not None:
                row_callback(global_tick_index, index, leg, result, ex.latest_frame)
            global_tick_index += 1
            tick_count += 1
            if result.done:
                outcome = result.outcome
                break
            sleep_fn(params.streaming_interval)
        assert outcome is not None  # the while loop always sets one before exiting

        heading_after = heading.measured_heading(ex.latest_frame)
        duration = clock_fn() - leg_start

        leg_result = TourLegResult(
            index=index, leg=leg, outcome=outcome, heading_before=heading_before,
            heading_after=heading_after, duration=duration,
            fault=(outcome == RunOutcome.FAULT), tick_count=tick_count)
        leg_results.append(leg_result)
        logger.info("run_tour(): leg %d/%d outcome=%s ticks=%d duration=%.2fs",
                   index + 1, len(legs), outcome.value, tick_count, duration)

        if on_leg is not None:
            on_leg(index, len(legs), leg, leg_result)

        if outcome != RunOutcome.COMPLETED:
            stopped_at = index
            stopped_outcome = outcome
            logger.error("run_tour(): stopping -- leg %d/%d ended with outcome=%s, no further "
                        "legs attempted", index + 1, len(legs), outcome.value)
            break

        if index == len(legs) - 1:
            # Final leg -- settle window before capturing the tour's own
            # closure end pose (AC3): the terminal tick() already sent
            # stop(), but the plant needs real time to actually decelerate.
            sleep_fn(final_settle)
            settle_frames = transport.read_pending_binary_tlm_frames()
            final_frame = settle_frames[-1] if settle_frames else ex.latest_frame
            end_pose = _frame_pose_rad(final_frame)
        else:
            # Inter-leg settle -- let the plant physically decelerate before
            # the NEXT leg's own begin() re-baselines telemetry; the drained
            # batch is discarded (begin() re-drains fresh for itself).
            sleep_fn(inter_leg_settle)
            transport.read_pending_binary_tlm_frames()

    closure = _compute_closure(start_pose, end_pose)
    return TourResult(legs=leg_results, closure=closure, stopped_at=stopped_at,
                      stopped_outcome=stopped_outcome)
