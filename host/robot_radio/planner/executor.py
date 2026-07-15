"""robot_radio.planner.executor -- streaming twist executor.

`StreamingExecutor` walks a `planner/profile.py` setpoint sequence, sending
one `twist()` per streaming tick at `planner/model.py`'s live-tunable
pacing interval (default ~150ms -- the ONLY empirically soak-tested paced
rate, per `ack-ring-intermittent-delivery-gap.md` finding 2; NOT the
~25Hz telemetry cadence from ticket 001, an unrelated, unvalidated-for-
commands quantity -- `architecture-update.md` Decision 6), re-arming the
firmware deadman on every send, and continuously draining telemetry
(`NezhaProtocol.read_pending_binary_tlm_frames()`, wrapping `SerialConnection
.drain_binary_tlm()`) between sends -- NEVER gating a control decision on a
bounded `wait_for_ack()` call (`ack-ring-intermittent-delivery-gap.md`'s
own explicit recommendation for this exact use case, architecture-update.md
Decision 5).

This module is where every one of the ten binding requirements from
`host-planner-design-lessons-from-drive-v2-review.md` is actually
implemented. Binding-requirement -> mechanism map (reproduced from
architecture-update.md Step 6's own disposition table):

 1. Sign-aware completion, no fabsf-blind predicates
    -- `_within_bound()` below builds a signed `[lo, hi]` interval around
    the target using `min()`/`max()` against 0.0, never `abs()`/`fabsf()`
    on the measured value. Grep-verified: this module contains no `abs(`
    call on a signed measured/target quantity.
 2. No silent drops
    -- every validation clamp, degraded-feedback condition, and fault-bit
    observation logs loudly (`logger.warning`/`logger.error`) before
    acting; Decision 5's continuous-drain design (this module never calls
    `wait_for_ack()`) removes the one place most likely to silently stall.
 3. Clock discipline across replans
    -- one `self._run_start` elapsed-time clock captured in `begin()`,
    consumed by every `tick()` in the same run; `begin()` is the ONLY
    place that (re)captures it, and every run (including a preemption's
    replan) calls `begin()` fresh.
 4. Preemption invalidates chain state
    -- `preempt()`/`stop_now()` always call `NezhaProtocol.stop()` FIRST,
    then `begin()` re-drains telemetry and rebuilds `self._baseline`/
    `self._commanded_heading` from THAT fresh frame -- never carries the
    interrupted run's baseline/clock/index forward.
 5. Validate wire inputs
    -- `_clamp_ceiling()` re-validates every `twist()` magnitude against
    `PlannerParams`' ceilings immediately before send, independent of
    `profile.py`'s own boundary validation (defense in depth).
 6. Bounded overshoot
    -- `_within_bound()`, checked every tick against
    `overshoot_bound_linear`/`overshoot_bound_angular`; tripping it ends
    the run with a logged `RunOutcome.OVERSHOOT`, not a silent accept.
 7. Terminal-phase care, no zero-dwell reversal
    -- the terminal setpoint of any run always triggers an explicit
    `NezhaProtocol.stop()` call in `tick()` (never reliance on the
    deadman timing out); `profile.py` already guarantees no sign-reversal
    shape in the setpoint sequence itself.
 8. Latency as a first-class parameter
    -- `PlannerParams.latency_tau` is consumed by the heading loop's own
    correction timing (`tick()`'s `lead_heading` computation below): since
    a twist() sent THIS tick only actuates ~`latency_tau` later, the
    heading corrector is aimed at where the profile's own trajectory WILL
    BE by then (`commanded_heading + setpoint.omega * latency_tau`, a
    first-order dead-time lead compensation), not where it is at the
    instant of sending -- a straight leg (`omega == 0`) is unaffected
    (lead term is zero), so this only changes behavior on a turn.
    `link_latency_margin` is the separate, additive term folded into each
    twist()'s `duration` so the deadman never expires between ticks.
 9. Everything tunable live
    -- every field this module reads comes from `self._params.<field>`,
    read fresh each `tick()` call, never cached at construction.
 10. Heading-loop bandwidth verified empirically
    -- out of this module's scope; ticket 006's bench session is the
    empirical measurement (this module's own gains are a starting point).

No hardware, no sim transport required for this module's own unit-test
gate -- see `tests/unit/test_planner_executor.py`'s own header for the
fake `TwistTransport` double convention it uses.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Callable, Protocol, Sequence

if TYPE_CHECKING:
    from robot_radio.planner.heading import HeadingCorrector
    from robot_radio.planner.model import PlannerParams
    from robot_radio.planner.profile import ProfileSetpoint
    from robot_radio.robot.protocol import TLMFrame

logger = logging.getLogger(__name__)


class TwistTransport(Protocol):
    """The exact slice of `NezhaProtocol`'s public surface this module
    depends on -- a `Protocol` (structural, duck-typed) rather than a
    concrete import of `NezhaProtocol` itself, so a unit test can hand this
    module a lightweight fake with no real serial port / protobuf codec
    behind it (see `tests/unit/test_planner_executor.py`'s `FakeTransport`).
    A real `NezhaProtocol` instance already satisfies this Protocol as-is
    -- no adapter needed in production."""

    def twist(self, v_x: float, omega: float, duration: float) -> int: ...  # [mm/s] [rad/s] [ms]

    def stop(self) -> int: ...

    def read_pending_binary_tlm_frames(self) -> "list[TLMFrame]": ...


class RunState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    DONE = "done"


class RunOutcome(str, Enum):
    COMPLETED = "completed"      # setpoints exhausted, terminal stop() sent
    STOPPED = "stopped"          # preempt()/stop_now() ended the run early
    FAULT = "fault"              # a drained frame's fault_bits was nonzero
    OVERSHOOT = "overshoot"      # measured progress left the outer bound


@dataclass(frozen=True)
class TickResult:
    """What one `tick()` call did."""

    v_x: float  # [mm/s] the (validated) value actually sent
    omega: float  # [rad/s] the (validated) value actually sent, trim included
    corr_id: int  # the twist()/stop() corr_id this tick sent
    done: bool  # True once this tick ended the run (any RunOutcome)
    outcome: RunOutcome | None  # set only when done is True


_LINEAR = "linear"
_ANGULAR = "angular"

# 107-001: bounded retry for begin()'s own first telemetry drain -- see
# begin()'s own docstring for why a single non-blocking drain can
# legitimately race an idle queue.
_BEGIN_DRAIN_RETRIES = 5
_BEGIN_DRAIN_RETRY_INTERVAL = 0.05  # [s]


class StreamingExecutor:
    """Streams a profile's setpoint sequence as paced `twist()` calls,
    applying heading correction and continuous telemetry-driven safety
    checks. See this module's own header for the binding-requirement map.
    """

    def __init__(self, transport: TwistTransport, params: "PlannerParams",
                 heading: "HeadingCorrector",
                 clock_fn: Callable[[], float] = time.monotonic,
                 sleep_fn: Callable[[float], None] = time.sleep) -> None:
        self._transport = transport
        self._params = params
        self._heading = heading
        self._clock_fn = clock_fn
        self._sleep_fn = sleep_fn

        self._setpoints: list["ProfileSetpoint"] = []
        self._index = 0
        self._axis = _LINEAR
        self._target = 0.0
        self._run_start: float | None = None
        self._baseline = 0.0
        self._fault_baseline = 0
        self._commanded_heading = 0.0
        self._latest_frame: "TLMFrame | None" = None
        self._state = RunState.IDLE

    @property
    def state(self) -> RunState:
        return self._state

    @property
    def latest_frame(self) -> "TLMFrame | None":
        """The most recently drained telemetry frame -- exposed read-only
        for callers/tests that want to inspect feedback without reaching
        into executor internals."""
        return self._latest_frame

    # ------------------------------------------------------------------
    # Run lifecycle
    # ------------------------------------------------------------------

    def begin(self, setpoints: Sequence["ProfileSetpoint"], target: float,
             axis: str = _LINEAR) -> None:  # [mm] or [rad], matching axis
        """Start a fresh run: captures a NEW segment-global elapsed clock
        (binding requirement #3), discards any prior run's setpoints, and
        re-baselines progress/commanded-heading from a FRESHLY drained
        telemetry frame (binding requirement #4) -- never carries state
        from a previous run, including one this call is preempting.

        Also captures that same first-drained frame's `fault_bits` as this
        run's own `_fault_baseline` (107-001, mirroring `rig_soak.py`'s
        "only a bit that turns on DURING the run counts as new" convention):
        on real hardware `kFaultI2CSafetyNet` is a boot-time one-shot latch
        that is essentially always present by the time any run begins, and
        a benign `kFaultWedgeLatch` boundary latch can assert during an
        idle gap between runs -- neither should fault-stop `tick()` on tick
        2 of every run. Re-baselining per-`begin()` call (not once per
        process) means `preempt()` (which always calls `begin()` fresh)
        automatically gets a new fault baseline too, same as it already
        gets a fresh progress baseline.

        The first drain is retried a few times (bounded, `_BEGIN_DRAIN_RETRIES`
        attempts) if it comes back empty, before falling back to the
        `None`/zero baseline: `read_pending_binary_tlm_frames()` is a
        non-blocking poll of an async-pushed queue (`~25Hz` push cadence),
        so a single call can legitimately race an idle queue between two
        pushes and return nothing yet -- confirmed on real hardware
        (107-001's own bench session) immediately after the standing
        preflight's own reverse nudge. Without the retry, that race
        silently produces a `_fault_baseline` of 0 even though a fault bit
        (e.g. an already-latched `kFaultWedgeLatch`) is genuinely already
        asserted, so the very NEXT drained frame looks like a brand-new
        fault and false-positive fault-stops the run on tick 1 -- exactly
        the footgun this ticket exists to remove. Mirrors
        `profiled_motion_verify.py`'s own pre-existing bench-script-local
        retry for `baseline_heading`, promoted here because it affects the
        fault baseline the same way.
        """
        if not setpoints:
            raise ValueError("begin(): setpoints must be non-empty")
        if axis not in (_LINEAR, _ANGULAR):
            raise ValueError(f"begin(): unknown axis {axis!r}")

        self._setpoints = list(setpoints)
        self._index = 0
        self._axis = axis
        self._target = float(target)
        self._run_start = self._clock_fn()
        self._heading.reset()

        frames = self._transport.read_pending_binary_tlm_frames()
        retries = 0
        while not frames and retries < _BEGIN_DRAIN_RETRIES:
            self._sleep_fn(_BEGIN_DRAIN_RETRY_INTERVAL)
            frames = self._transport.read_pending_binary_tlm_frames()
            retries += 1
        self._latest_frame = frames[-1] if frames else None
        self._baseline = self._progress(self._latest_frame) or 0.0
        self._fault_baseline = (
            self._latest_frame.fault_bits
            if self._latest_frame is not None and self._latest_frame.fault_bits is not None
            else 0)
        measured_heading = self._heading.measured_heading(self._latest_frame)
        self._commanded_heading = measured_heading if measured_heading is not None else 0.0

        self._state = RunState.RUNNING
        logger.info(
            "StreamingExecutor.begin(): axis=%s target=%r setpoints=%d "
            "baseline=%r commanded_heading=%r fault_baseline=%r",
            axis, target, len(self._setpoints), self._baseline, self._commanded_heading,
            self._fault_baseline)

    def preempt(self, setpoints: Sequence["ProfileSetpoint"], target: float,
               axis: str = _LINEAR) -> None:
        """Preempt any in-progress run and start a new one cleanly (binding
        requirement #4): stop the drivetrain FIRST, discard whatever
        setpoints remained, then `begin()` the new run -- which itself
        re-drains telemetry for a fresh baseline, never the interrupted
        run's carried state."""
        logger.warning(
            "StreamingExecutor.preempt(): stopping in-progress run "
            "(state=%s, %d/%d setpoints sent) and starting a new profile",
            self._state, self._index, len(self._setpoints))
        self._transport.stop()
        self.begin(setpoints, target, axis)

    def stop_now(self) -> None:
        """Immediate stop, no replan -- calls `NezhaProtocol.stop()` and
        ends the current run (binding requirement #4's "stop() immediate"
        half; `preempt()` is the "then replan" half)."""
        logger.warning(
            "StreamingExecutor.stop_now(): stopping (state=%s, %d/%d "
            "setpoints sent)", self._state, self._index, len(self._setpoints))
        self._transport.stop()
        self._state = RunState.DONE

    # ------------------------------------------------------------------
    # Per-tick execution
    # ------------------------------------------------------------------

    def tick(self) -> TickResult:
        """Send exactly one twist for the current setpoint, drain
        telemetry, and advance. Never gates on `wait_for_ack()` (binding
        requirement #2, Decision 5) -- the drained frames feed the NEXT
        tick's heading/completion check, this tick's own send is fire-and-
        poll."""
        if self._state != RunState.RUNNING:
            raise RuntimeError("tick(): no run in progress -- call begin() first")
        if self._run_start is None:  # pragma: no cover -- begin() always sets this
            raise RuntimeError("tick(): internal state error -- no run clock")

        setpoint = self._setpoints[self._index]
        now = self._clock_fn() - self._run_start

        # Binding requirement #8: latency_tau dead-time lead compensation
        # -- this tick's twist() only actuates ~latency_tau later, so aim
        # the heading corrector at where the profile's own trajectory will
        # be by then, not where it is right now. Zero on a straight leg
        # (setpoint.omega == 0), so "hold heading" is unaffected.
        lead_heading = self._commanded_heading + setpoint.omega * self._params.latency_tau
        trim = self._heading.update(lead_heading, self._latest_frame, now)

        v_x = self._clamp_ceiling(setpoint.v_x, self._params.v_max, "v_x")
        omega = self._clamp_ceiling(setpoint.omega + trim, self._params.omega_max, "omega")

        duration = (self._params.streaming_interval + self._params.link_latency_margin) * 1000.0  # [ms]
        corr_id = self._transport.twist(v_x, omega, duration)

        # Continuous telemetry drain (Decision 5) -- feeds the NEXT tick's
        # heading/completion check; also this tick's own fault-bit check.
        # Baseline-relative (107-001): a bit already present in this run's
        # own `_fault_baseline` (captured by begin()) never trips the fault
        # gate -- only a bit that turns on NEW relative to that baseline
        # does.
        frames = self._transport.read_pending_binary_tlm_frames()
        fault = any((f.fault_bits & ~self._fault_baseline) for f in frames
                    if f.fault_bits is not None)
        if frames:
            self._latest_frame = frames[-1]

        # Advance the profile's own planned heading trajectory
        # (trapezoidal integration of the SETPOINT's own omega, not the
        # measured one) -- this is what makes a straight profile "hold
        # heading" (omega stays 0 throughout, so commanded_heading never
        # moves) and a turn profile "track" (commanded_heading advances
        # with the profile's own planned turn) the SAME mechanism, with no
        # special-casing here.
        if self._index > 0:
            prev = self._setpoints[self._index - 1]
            dt = setpoint.elapsed - prev.elapsed
            self._commanded_heading += 0.5 * (prev.omega + setpoint.omega) * dt

        self._index += 1

        if fault:
            logger.error(
                "StreamingExecutor.tick(): fault bit observed mid-run "
                "(index=%d/%d) -- stopping", self._index, len(self._setpoints))
            self._transport.stop()
            self._state = RunState.DONE
            return TickResult(v_x=v_x, omega=omega, corr_id=corr_id,
                             done=True, outcome=RunOutcome.FAULT)

        measured = self._progress(self._latest_frame)
        if measured is not None and not self._within_bound(measured):
            logger.error(
                "StreamingExecutor.tick(): bounded-overshoot violation "
                "(measured=%r target=%r axis=%s) -- stopping",
                measured, self._target, self._axis)
            self._transport.stop()
            self._state = RunState.DONE
            return TickResult(v_x=v_x, omega=omega, corr_id=corr_id,
                             done=True, outcome=RunOutcome.OVERSHOOT)

        if self._index >= len(self._setpoints):
            # Terminal setpoint always gets an explicit stop() call
            # (binding requirement #7) -- never reliance on the deadman
            # timeout alone.
            stop_corr_id = self._transport.stop()
            self._state = RunState.DONE
            logger.info("StreamingExecutor.tick(): profile complete "
                       "(%d setpoints sent)", len(self._setpoints))
            return TickResult(v_x=v_x, omega=omega, corr_id=stop_corr_id,
                             done=True, outcome=RunOutcome.COMPLETED)

        return TickResult(v_x=v_x, omega=omega, corr_id=corr_id, done=False, outcome=None)

    def run(self, setpoints: Sequence["ProfileSetpoint"], target: float,
           axis: str = _LINEAR) -> RunOutcome:
        """Blocking convenience: `begin()` then `tick()` in a loop, pacing
        with `sleep_fn` between sends. Tests generally drive `tick()`
        directly (no real sleeping); this is the production entry point."""
        self.begin(setpoints, target, axis)
        outcome: RunOutcome | None = None
        while self._state == RunState.RUNNING:
            result = self.tick()
            if result.done:
                outcome = result.outcome
                break
            self._sleep_fn(self._params.streaming_interval)
        assert outcome is not None  # tick() always sets one once done=True
        return outcome

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _clamp_ceiling(self, value: float, ceiling: float, name: str) -> float:
        """Binding requirement #5: re-validate `value` against `ceiling`
        (a `PlannerParams` field, read fresh by the caller) immediately
        before it is sent -- independent of, and in addition to,
        `profile.py`'s own boundary validation. Clamps (rather than
        raising) so a single defense-in-depth backstop never crashes a
        live control loop; a clamp that actually changes the value is
        always logged loudly (binding requirement #2), never silent."""
        clamped = max(-ceiling, min(ceiling, value))
        if clamped != value:
            logger.warning(
                "StreamingExecutor: %s=%r exceeded ceiling %r -- clamped "
                "to %r", name, value, ceiling, clamped)
        return clamped

    def _progress(self, frame: "TLMFrame | None") -> float | None:
        """Signed measured progress along this run's own axis, relative to
        NOTHING (the caller subtracts `self._baseline`) -- `linear` reads
        the mean of `frame.enc` (mm), `angular` reads the heading
        corrector's own selected source (radians), matching the SAME
        pose-source choice (`otos_untrusted`) heading correction uses."""
        if frame is None:
            return None
        if self._axis == _LINEAR:
            if frame.enc is None:
                return None
            return (frame.enc[0] + frame.enc[1]) / 2.0
        return self._heading.measured_heading(frame)

    def _within_bound(self, measured_raw: float) -> bool:
        """Binding requirements #1/#6: a SIGN-AWARE bounded-overshoot
        check -- builds the interval `[min(0, target), max(0, target)] +-
        tolerance` and tests containment. This is deliberately never an
        `abs()`/`fabsf()` comparison against the measured value: the
        interval itself already encodes the commanded direction (a
        negative target's own interval sits entirely at or below zero), so
        a measurement that overshoots in the WRONG direction is caught
        exactly as reliably as one that overshoots too far in the RIGHT
        direction -- an `abs(measured) > abs(target) + tol`-style check
        would miss the wrong-direction case entirely.
        """
        measured = measured_raw - self._baseline
        target = self._target
        tolerance = (self._params.overshoot_bound_linear if self._axis == _LINEAR
                    else self._params.overshoot_bound_angular)
        lo = min(0.0, target) - tolerance
        hi = max(0.0, target) + tolerance
        return lo <= measured <= hi
