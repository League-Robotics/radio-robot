"""tests/unit/test_planner_executor.py -- 106-005 (SUC-028/SUC-029).

Covers `robot_radio.planner.executor.StreamingExecutor` against a fake
`TwistTransport` double (`FakeTransport` below) -- no real serial port, no
sim, no hardware. Mirrors this tree's existing convention for testing
`protocol.py` callers (`tests/unit/test_protocol_binary_client.py`'s
loopback-serial doubles) at one level higher: `FakeTransport` implements
exactly the `TwistTransport` structural interface `executor.py` depends
on (`twist()`/`stop()`/`read_pending_binary_tlm_frames()`), letting these
tests script a synthetic TLM stream directly (on-time frames, late
frames -- an empty drain followed by a multi-frame catch-up batch --
dropped frames -- an empty drain forever) without any protobuf/serial
plumbing.

One test section per binding-requirement acceptance criterion in ticket
005 (`clasi/sprints/106-.../tickets/005-....md`), in the SAME order as
that ticket's own Acceptance Criteria list.

Collected under `tests/unit/` per `pyproject.toml`'s `testpaths`.
"""

from __future__ import annotations

import ast
import inspect
import math

import pytest

from robot_radio.planner import executor as executor_module
from robot_radio.planner.executor import RunOutcome, RunState, StreamingExecutor
from robot_radio.planner.heading import HeadingCorrector
from robot_radio.planner.model import PlannerParams
from robot_radio.planner.profile import ProfileLimits, ProfileSetpoint, profile_for_distance
from robot_radio.robot.protocol import TLMFrame


# ---------------------------------------------------------------------------
# Fake transport -- scripted TwistTransport double
# ---------------------------------------------------------------------------


class FakeTransport:
    """Records every twist()/stop() call; serves telemetry frames from a
    scripted, per-tick queue of batches -- an empty batch models a
    late/dropped frame (nothing new since the last drain), a multi-frame
    batch models several frames arriving between two drains (a late
    catch-up)."""

    def __init__(self) -> None:
        self.twist_calls: list[tuple[float, float, float]] = []
        self.stop_calls: int = 0
        self._corr_id = 0
        self._batches: list[list[TLMFrame]] = []

    def queue(self, *frames: TLMFrame) -> None:
        """Queue ONE batch (possibly empty) to be returned by the next
        `read_pending_binary_tlm_frames()` call."""
        self._batches.append(list(frames))

    def queue_empty(self) -> None:
        self._batches.append([])

    def twist(self, v_x: float, omega: float, duration: float) -> int:
        self._corr_id += 1
        self.twist_calls.append((v_x, omega, duration))
        return self._corr_id

    def stop(self) -> int:
        self._corr_id += 1
        self.stop_calls += 1
        return self._corr_id

    def read_pending_binary_tlm_frames(self) -> list[TLMFrame]:
        if self._batches:
            return self._batches.pop(0)
        return []


def _frame(enc=(0, 0), pose=(0, 0, 0), otos=(0, 0, 0), fault_bits=0):
    return TLMFrame(enc=enc, pose=pose, otos=otos, fault_bits=fault_bits, event_bits=0)


def _executor(params: PlannerParams | None = None, otos_untrusted: bool = True,
             clock=None):
    params = params or PlannerParams(streaming_interval=0.1, link_latency_margin=0.1)
    from types import SimpleNamespace
    robot_config = SimpleNamespace(geometry=SimpleNamespace(otos_untrusted=otos_untrusted))
    heading = HeadingCorrector(params, robot_config=robot_config)
    transport = FakeTransport()
    if clock is None:
        counter = iter(i * 0.01 for i in range(100000))
        clock = lambda: next(counter)  # noqa: E731 -- test-local fake clock
    ex = StreamingExecutor(transport, params, heading, clock_fn=clock, sleep_fn=lambda s: None)
    return ex, transport, params


def _straight_setpoints(distance=500.0, v_max=200.0, a_max=500.0, cadence=0.1):
    return profile_for_distance(distance, ProfileLimits(v_max=v_max, a_max=a_max), cadence=cadence)


def _ast_calls_named(module, name: str) -> bool:
    """True if `module`'s source contains an actual CALL to `name` (a
    `Name`/`Attribute` node, e.g. `abs(x)` or `foo.wait_for_ack(...)`) --
    walks the parsed AST rather than grepping raw text, so a docstring that
    merely MENTIONS the name (this module's own header does, extensively,
    explaining why it is absent) never produces a false positive."""
    tree = ast.parse(inspect.getsource(module))
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Name) and func.id == name:
                return True
            if isinstance(func, ast.Attribute) and func.attr == name:
                return True
    return False


# ---------------------------------------------------------------------------
# AC 1: sign-aware completion, bounded outer tolerance in BOTH directions
# ---------------------------------------------------------------------------


def test_within_bound_never_calls_abs_or_fabs_on_a_signed_quantity():
    """Code-inspection check via the AST (not raw-text grep -- this
    module's own header docstring extensively MENTIONS `abs()`/`fabs()`
    while explaining why neither is used, which would false-positive a
    plain substring search): no `abs(...)`/`fabs(...)` CALL node exists
    anywhere in the module. `_within_bound()` is built entirely from
    `min()`/`max()` interval containment (see its own docstring)."""
    assert not _ast_calls_named(executor_module, "abs")
    assert not _ast_calls_named(executor_module, "fabs")


def test_within_bound_is_sign_aware_for_positive_and_negative_targets():
    ex, _, params = _executor()
    ex._axis = "linear"
    ex._baseline = 0.0
    ex._target = 500.0
    params.overshoot_bound_linear = 10.0

    assert ex._within_bound(500.0) is True     # exactly on target
    assert ex._within_bound(505.0) is True     # within tolerance
    assert ex._within_bound(600.0) is False    # overshoot past bound
    # A small negative excursion near the START (0) is legitimate startup
    # jitter, within tolerance of the interval's own lower end -- NOT a
    # "wrong direction" failure by itself.
    assert ex._within_bound(-5.0) is True
    # Far past the tolerance band on the wrong side of zero IS a failure.
    assert ex._within_bound(-50.0) is False

    ex._target = -500.0
    assert ex._within_bound(-500.0) is True
    assert ex._within_bound(-600.0) is False
    assert ex._within_bound(5.0) is True       # small excursion near start, still fine
    assert ex._within_bound(50.0) is False     # WRONG direction for a negative target


# ---------------------------------------------------------------------------
# AC 2: no control decision gated on a bounded wait_for_ack() call
# ---------------------------------------------------------------------------


def test_no_wait_for_ack_call_anywhere_in_executor():
    """AST-based (not raw-text) check -- this module's own header docstring
    MENTIONS `wait_for_ack()` several times while explaining why it is
    never called; a plain substring search over the docstring would
    false-positive. `_ast_calls_named()` only matches an actual CALL
    node, so it correctly finds none."""
    assert not _ast_calls_named(executor_module, "wait_for_ack")


def test_tick_does_not_require_a_reply_to_proceed():
    """FakeTransport.twist()/stop() never model a reply at all -- if tick()
    were gated on an ack it would have nothing to wait on. Ticking past
    every setpoint with only empty telemetry batches proves no such gate
    exists."""
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=50.0, cadence=0.1)
    ex.begin(setpoints, target=50.0, axis="linear")
    for _ in setpoints:
        transport.queue_empty()

    result = None
    for _ in range(len(setpoints)):
        result = ex.tick()
        if result.done:
            break
    assert result is not None and result.done


# ---------------------------------------------------------------------------
# AC 3: single segment-global elapsed-time clock per run
# ---------------------------------------------------------------------------


def test_run_start_clock_is_captured_once_at_begin_not_rebased_per_tick():
    clock_values = iter([100.0, 100.2, 100.4, 100.6, 100.8, 101.0, 200.0, 200.2])
    ex, transport, _ = _executor(clock=lambda: next(clock_values))
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=0.0, omega=0.0),
                 ProfileSetpoint(elapsed=0.1, v_x=10.0, omega=0.0),
                 ProfileSetpoint(elapsed=0.2, v_x=0.0, omega=0.0)]

    ex.begin(setpoints, target=1.0, axis="linear")   # consumes clock() == 100.0
    run_start = ex._run_start
    assert run_start == 100.0

    transport.queue_empty()
    ex.tick()  # consumes 100.2 -> now = 0.2, run_start unchanged
    assert ex._run_start == run_start


def test_preemption_captures_a_fresh_clock_never_rebasing_the_stale_one():
    # begin() consumes ONE clock() call; preempt()'s own internal begin()
    # consumes the SECOND -- no tick() runs in between, so exactly two
    # values are consumed total.
    clock_values = iter([100.0, 500.0])
    ex, transport, _ = _executor(clock=lambda: next(clock_values))
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=0.0, omega=0.0),
                 ProfileSetpoint(elapsed=0.1, v_x=10.0, omega=0.0)]

    ex.begin(setpoints, target=1.0, axis="linear")
    first_run_start = ex._run_start
    assert first_run_start == 100.0

    transport.queue_empty()
    ex.preempt(setpoints, target=1.0, axis="linear")
    assert ex._run_start == 500.0
    assert ex._run_start != first_run_start


# ---------------------------------------------------------------------------
# AC 4: preempting a running profile plans from injected "current" state
# ---------------------------------------------------------------------------


def test_preempt_stops_first_then_replans_from_fresh_telemetry_not_carried_state():
    ex, transport, _ = _executor()
    old_setpoints = _straight_setpoints(distance=500.0, cadence=0.1)

    transport.queue(_frame(enc=(0, 0)))
    ex.begin(old_setpoints, target=500.0, axis="linear")
    assert ex._baseline == 0.0

    # Advance partway through the old run.
    transport.queue(_frame(enc=(50, 50)))
    ex.tick()
    assert ex._index == 1

    new_setpoints = _straight_setpoints(distance=300.0, cadence=0.1)
    # A fresh frame representing genuinely different "current" state -- if
    # preempt() carried the OLD baseline/index forward, the new run's
    # baseline would incorrectly stay at the old value instead of this one.
    transport.queue(_frame(enc=(9000, 9000)))
    ex.preempt(new_setpoints, target=300.0, axis="linear")

    assert transport.stop_calls == 1  # stop() called BEFORE the replan
    assert ex._index == 0             # old remaining setpoints discarded
    assert ex._baseline == 9000.0     # replanned from the FRESH frame
    assert ex._setpoints == list(new_setpoints)


def test_stop_now_stops_immediately_with_no_replan():
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=500.0, cadence=0.1)
    transport.queue_empty()
    ex.begin(setpoints, target=500.0, axis="linear")

    ex.stop_now()

    assert transport.stop_calls == 1
    assert ex.state == RunState.DONE


# ---------------------------------------------------------------------------
# AC 5: every twist() magnitude validated against model.py's ceilings
# ---------------------------------------------------------------------------


def test_v_x_clamped_to_ceiling_immediately_before_send(caplog):
    ex, transport, params = _executor()
    params.v_max = 100.0
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=500.0, omega=0.0)]
    transport.queue_empty()
    ex.begin(setpoints, target=500.0, axis="linear")

    with caplog.at_level("WARNING"):
        ex.tick()

    assert transport.twist_calls[0][0] == pytest.approx(100.0)
    assert any("exceeded ceiling" in r.message for r in caplog.records)


def test_omega_clamped_to_ceiling_immediately_before_send(caplog):
    ex, transport, params = _executor()
    params.omega_max = 1.0
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=0.0, omega=5.0)]
    transport.queue_empty()
    ex.begin(setpoints, target=5.0, axis="angular")

    with caplog.at_level("WARNING"):
        ex.tick()

    assert transport.twist_calls[0][1] == pytest.approx(1.0)
    assert any("exceeded ceiling" in r.message for r in caplog.records)


def test_in_bound_magnitudes_pass_through_unclamped():
    ex, transport, params = _executor()
    params.v_max = 200.0
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=50.0, omega=0.0)]
    transport.queue_empty()
    ex.begin(setpoints, target=50.0, axis="linear")

    ex.tick()

    assert transport.twist_calls[0][0] == pytest.approx(50.0)


# ---------------------------------------------------------------------------
# AC 6: terminal setpoint is an explicit stop() call, no zero-dwell reversal
# ---------------------------------------------------------------------------


def test_terminal_setpoint_sends_explicit_stop():
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=50.0, cadence=0.1)
    ex.begin(setpoints, target=50.0, axis="linear")

    result = None
    for _ in range(len(setpoints)):
        transport.queue_empty()
        result = ex.tick()
        if result.done:
            break

    assert result is not None
    assert result.outcome == RunOutcome.COMPLETED
    assert transport.stop_calls == 1
    assert ex.state == RunState.DONE


def test_completion_never_reintroduces_a_sign_reversal():
    """profile.py already guarantees its terminal setpoint lands at exactly
    zero -- confirm the executor sends that terminal zero as-is (no
    reversal applied) before its own stop() call."""
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=50.0, cadence=0.1)
    ex.begin(setpoints, target=50.0, axis="linear")

    last_v_x = None
    for _ in range(len(setpoints)):
        transport.queue_empty()
        result = ex.tick()
        last_v_x = result.v_x
        if result.done:
            break

    assert last_v_x == 0.0


# ---------------------------------------------------------------------------
# AC 7: streaming cadence / accel-decel limits / gains adjustable live
# ---------------------------------------------------------------------------


def test_streaming_interval_change_is_reflected_in_next_ticks_duration():
    ex, transport, params = _executor()
    params.streaming_interval = 0.1
    params.link_latency_margin = 0.1
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=0.0, omega=0.0),
                 ProfileSetpoint(elapsed=0.1, v_x=0.0, omega=0.0)]
    transport.queue_empty()
    ex.begin(setpoints, target=0.1, axis="linear")

    transport.queue_empty()
    ex.tick()
    first_duration = transport.twist_calls[0][2]
    assert first_duration == pytest.approx(200.0)  # (0.1+0.1)*1000

    params.streaming_interval = 0.3  # live mutation, no reconstruction
    transport.queue_empty()
    ex.tick()
    second_duration = transport.twist_calls[1][2]
    assert second_duration == pytest.approx(400.0)  # (0.3+0.1)*1000


def test_heading_gain_change_is_reflected_in_next_ticks_trim():
    """Executor.tick() computes this tick's trim from `self._latest_frame`
    -- the frame drained by the PREVIOUS tick (or begin()'s own baseline
    read), per Decision 5's continuous-drain design. So: begin() with a
    zero-heading baseline, tick #1 sees that baseline (zero trim
    regardless of kp) and drains a DRIFTED frame, tick #2 (kp still 0)
    consumes that drifted frame and confirms zero trim, THEN kp is raised
    live and tick #3 consumes a drifted frame again and confirms a
    nonzero trim -- proving the live kp mutation (not a stale, construction-
    time-cached gain) is what changed the outcome."""
    ex, transport, params = _executor()
    params.heading_kp = 0.0
    params.heading_omega_clamp = 10.0
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=10.0, omega=0.0),
                 ProfileSetpoint(elapsed=0.1, v_x=10.0, omega=0.0),
                 ProfileSetpoint(elapsed=0.2, v_x=10.0, omega=0.0)]
    transport.queue(_frame(pose=(0, 0, 0)))  # begin() baseline: zero heading
    ex.begin(setpoints, target=1.0, axis="linear")

    transport.queue(_frame(pose=(0, 0, 1000)))  # drained by tick #1, used by tick #2
    r0 = ex.tick()
    assert r0.omega == pytest.approx(0.0)  # tick #1 still sees the zero baseline

    transport.queue(_frame(pose=(0, 0, 1000)))  # drained by tick #2, used by tick #3
    r1 = ex.tick()
    assert r1.omega == pytest.approx(0.0)  # kp still 0.0 -> zero trim despite drift

    params.heading_kp = 5.0  # live mutation, no reconstruction
    transport.queue(_frame(pose=(0, 0, 1000)))
    r2 = ex.tick()
    assert r2.omega != pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Binding requirement #8: latency_tau is CONSUMED by the heading loop's own
# correction timing (a dead-time lead compensation on a turn leg), not
# merely a declared-but-unread PlannerParams field.
# ---------------------------------------------------------------------------


def test_latency_tau_zero_produces_no_lead_on_a_turn():
    params = PlannerParams(heading_kp=10.0, heading_ki=0.0, heading_kd=0.0,
                          heading_omega_clamp=100.0, omega_max=100.0, latency_tau=0.0,
                          streaming_interval=0.1, link_latency_margin=0.1)
    ex, transport, _ = _executor(params=params, otos_untrusted=True)
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=0.0, omega=2.0),
                 ProfileSetpoint(elapsed=0.1, v_x=0.0, omega=2.0)]
    transport.queue(_frame(pose=(0, 0, 0)))  # baseline: measured heading 0
    ex.begin(setpoints, target=math.pi / 2, axis="angular")

    transport.queue_empty()
    result = ex.tick()

    # No lead (latency_tau=0) -> lead_heading == commanded_heading (0) ==
    # baseline measured (0) -> zero error -> zero trim -> result.omega is
    # just the setpoint's own omega (2.0), unmodified.
    assert result.omega == pytest.approx(2.0, abs=1e-9)


def test_latency_tau_nonzero_leads_the_commanded_heading_on_a_turn():
    params = PlannerParams(heading_kp=10.0, heading_ki=0.0, heading_kd=0.0,
                          heading_omega_clamp=100.0, omega_max=100.0, latency_tau=0.2,
                          streaming_interval=0.1, link_latency_margin=0.1)
    ex, transport, _ = _executor(params=params, otos_untrusted=True)
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=0.0, omega=2.0),
                 ProfileSetpoint(elapsed=0.1, v_x=0.0, omega=2.0)]
    transport.queue(_frame(pose=(0, 0, 0)))  # baseline: measured heading 0
    ex.begin(setpoints, target=math.pi / 2, axis="angular")

    transport.queue_empty()
    result = ex.tick()

    # lead_heading = commanded(0) + omega(2.0) * latency_tau(0.2) = 0.4 rad.
    # error = 0.4 - measured(0) = 0.4 -> first PID call returns kp*error =
    # 4.0 trim, added onto the setpoint's own omega (2.0) -> 6.0 total.
    assert result.omega == pytest.approx(2.0 + 10.0 * 0.4, abs=1e-6)


def test_latency_tau_lead_is_zero_on_a_straight_leg():
    """omega == 0 throughout a straight profile -> the lead term
    (omega * latency_tau) is always exactly zero, so a nonzero latency_tau
    never disturbs "hold heading" behavior."""
    params = PlannerParams(heading_kp=10.0, heading_ki=0.0, heading_kd=0.0,
                          heading_omega_clamp=100.0, latency_tau=0.5,
                          streaming_interval=0.1, link_latency_margin=0.1)
    ex, transport, _ = _executor(params=params, otos_untrusted=True)
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=10.0, omega=0.0),
                 ProfileSetpoint(elapsed=0.1, v_x=10.0, omega=0.0)]
    transport.queue(_frame(pose=(0, 0, 0)))
    ex.begin(setpoints, target=1.0, axis="linear")

    transport.queue_empty()
    result = ex.tick()

    assert result.omega == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# AC 8/9: HeadingCorrector otos_untrusted source + output clamp -- covered
# exhaustively in test_planner_heading.py; executor-level check that the
# SAME corrector instance is what the executor actually consults.
# ---------------------------------------------------------------------------


def test_executor_uses_the_heading_correctors_selected_source():
    params = PlannerParams(heading_kp=10.0, heading_omega_clamp=10.0,
                          streaming_interval=0.1, link_latency_margin=0.1)
    from types import SimpleNamespace
    heading = HeadingCorrector(params, robot_config=SimpleNamespace(
        geometry=SimpleNamespace(otos_untrusted=True)))
    transport = FakeTransport()
    clock_values = iter([0.0, 0.1, 0.2])
    ex = StreamingExecutor(transport, params, heading, clock_fn=lambda: next(clock_values),
                          sleep_fn=lambda s: None)

    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=10.0, omega=0.0),
                 ProfileSetpoint(elapsed=0.1, v_x=10.0, omega=0.0)]
    # pose drifted, otos did not -- only "pose" (otos_untrusted=True) should drive a trim.
    transport.queue(_frame(pose=(0, 0, 0), otos=(0, 0, 0)))
    ex.begin(setpoints, target=1.0, axis="linear")

    transport.queue(_frame(pose=(0, 0, 5000), otos=(0, 0, 0)))
    r0 = ex.tick()
    assert r0.omega == pytest.approx(0.0)  # tick #1 still sees the zero baseline

    transport.queue_empty()
    result = ex.tick()
    assert result.omega != pytest.approx(0.0)  # tick #2: pose drift produced a trim


# ---------------------------------------------------------------------------
# AC 10: a fault bit observed mid-run produces a logged stop, never silence
# ---------------------------------------------------------------------------


def test_fault_bit_mid_run_stops_and_logs(caplog):
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=500.0, cadence=0.1)
    transport.queue_empty()
    ex.begin(setpoints, target=500.0, axis="linear")

    transport.queue(_frame(fault_bits=1))

    with caplog.at_level("ERROR"):
        result = ex.tick()

    assert result.done is True
    assert result.outcome == RunOutcome.FAULT
    assert transport.stop_calls == 1
    assert any("fault bit" in r.message for r in caplog.records)


def test_no_fault_bit_does_not_stop():
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=500.0, cadence=0.1)
    transport.queue_empty()
    ex.begin(setpoints, target=500.0, axis="linear")

    transport.queue(_frame(fault_bits=0))
    result = ex.tick()

    assert result.outcome is None
    assert transport.stop_calls == 0


# ---------------------------------------------------------------------------
# 107-001: fault-check baseline exclusion -- a bit ALREADY present in the
# run's own first-drained frame (begin()'s baseline) never fault-stops the
# run; a bit that turns on freshly DURING a run still does (regression-
# protected, not weakened). Mirrors real hardware: kFaultI2CSafetyNet is a
# boot-time one-shot latch essentially always present by the time any run
# begins.
# ---------------------------------------------------------------------------


def test_baseline_fault_bit_present_at_begin_does_not_trip_the_run():
    """The first frame drained by begin() itself carries a nonzero
    fault_bits (e.g. kFaultI2CSafetyNet, latched from boot) -- the run must
    NOT fault-stop on it, on tick 2 or ever, as long as no NEW bit appears."""
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=500.0, cadence=0.1)

    transport.queue(_frame(fault_bits=1))  # begin()'s own baseline frame
    ex.begin(setpoints, target=500.0, axis="linear")
    assert ex._fault_baseline == 1

    # Every subsequent frame still carries the SAME baseline bit, nothing
    # new -- must never fault-stop.
    result = None
    for _ in range(len(setpoints)):
        transport.queue(_frame(fault_bits=1))
        result = ex.tick()
        if result.done:
            break

    assert result is not None
    assert result.outcome == RunOutcome.COMPLETED
    assert transport.stop_calls == 1  # only the terminal stop(), not a fault stop


def test_new_fault_bit_during_run_still_stops_after_zero_baseline(caplog):
    """A zero-baseline run (begin()'s own frame carries no fault bits) that
    later sees a bit turn on DURING the run must still fault-stop --
    baseline exclusion narrows the check, it does not disable it."""
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=500.0, cadence=0.1)

    transport.queue(_frame(fault_bits=0))
    ex.begin(setpoints, target=500.0, axis="linear")
    assert ex._fault_baseline == 0

    transport.queue(_frame(fault_bits=2))  # a bit turns on fresh mid-run

    with caplog.at_level("ERROR"):
        result = ex.tick()

    assert result.done is True
    assert result.outcome == RunOutcome.FAULT
    assert transport.stop_calls == 1


def test_begin_retries_an_empty_first_drain_before_defaulting_the_baseline():
    """107-001 (HITL-discovered): `read_pending_binary_tlm_frames()` is a
    non-blocking poll of an async-pushed queue -- a single call in begin()
    can legitimately race an idle queue and return nothing yet (confirmed
    on real hardware immediately after the standing preflight's own
    reverse nudge). begin() must retry a bounded few times before falling
    back to a zero/None baseline, so a fault bit that is ALREADY asserted
    (just not queued yet at the exact instant of the first read) is still
    correctly captured as baseline -- not missed and then mistaken for a
    brand-new fault on the very next tick."""
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=500.0, cadence=0.1)

    transport.queue_empty()  # races the queue -- nothing pushed yet
    transport.queue_empty()
    transport.queue(_frame(fault_bits=1))  # arrives on the 3rd drain attempt
    ex.begin(setpoints, target=500.0, axis="linear")

    assert ex._fault_baseline == 1
    assert ex.latest_frame is not None and ex.latest_frame.fault_bits == 1


def test_begin_falls_back_to_zero_baseline_if_every_retry_is_empty():
    """The bounded retry still gives up eventually -- begin() must not
    hang or raise if telemetry genuinely never arrives."""
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=500.0, cadence=0.1)

    for _ in range(10):
        transport.queue_empty()

    ex.begin(setpoints, target=500.0, axis="linear")

    assert ex._fault_baseline == 0
    assert ex.latest_frame is None


def test_new_fault_bit_on_top_of_a_baseline_bit_still_stops():
    """The baseline carries one bit (e.g. bit 0, kFaultI2CSafetyNet); a
    DIFFERENT bit (bit 1) turning on fresh during the run must still
    fault-stop -- the exclusion is per-bit (a bitmask), not "any nonzero
    fault_bits value seen at begin() disables the check entirely"."""
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=500.0, cadence=0.1)

    transport.queue(_frame(fault_bits=1))  # baseline: bit 0 only
    ex.begin(setpoints, target=500.0, axis="linear")
    assert ex._fault_baseline == 1

    transport.queue(_frame(fault_bits=3))  # bit 0 (baseline) + bit 1 (NEW)

    result = ex.tick()

    assert result.done is True
    assert result.outcome == RunOutcome.FAULT


# ---------------------------------------------------------------------------
# AC 11 (full suite green) is enforced by CI/the verification command, not
# a test in this file.
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Bounded overshoot (binding requirement #6) mid-run fault
# ---------------------------------------------------------------------------


def test_bounded_overshoot_mid_run_stops_and_logs(caplog):
    ex, transport, params = _executor()
    params.overshoot_bound_linear = 10.0
    setpoints = _straight_setpoints(distance=500.0, cadence=0.1)
    transport.queue(_frame(enc=(0, 0)))
    ex.begin(setpoints, target=500.0, axis="linear")

    # Measured progress FAR beyond target + tolerance.
    transport.queue(_frame(enc=(9000, 9000)))

    with caplog.at_level("ERROR"):
        result = ex.tick()

    assert result.done is True
    assert result.outcome == RunOutcome.OVERSHOOT
    assert transport.stop_calls == 1
    assert any("overshoot" in r.message.lower() for r in caplog.records)


# ---------------------------------------------------------------------------
# Heading: hold on a straight, track on a turn (same mechanism, no
# special-casing) -- correction sign/magnitude
# ---------------------------------------------------------------------------


def test_straight_holds_initial_heading_correcting_drift_toward_zero():
    params = PlannerParams(heading_kp=5.0, heading_omega_clamp=10.0,
                          streaming_interval=0.1, link_latency_margin=0.1)
    ex, transport, _ = _executor(params=params, otos_untrusted=True)
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=10.0, omega=0.0),
                 ProfileSetpoint(elapsed=0.1, v_x=10.0, omega=0.0)]

    transport.queue(_frame(pose=(0, 0, 0)))
    ex.begin(setpoints, target=1.0, axis="linear")
    assert ex._commanded_heading == pytest.approx(0.0)

    # Robot has drifted to a POSITIVE heading; commanded stays 0 (straight).
    # tick #1 still sees the zero baseline (drains the drifted frame for
    # tick #2's own correction -- Decision 5's continuous-drain design).
    transport.queue(_frame(pose=(0, 0, 3000)))  # +30 deg
    r0 = ex.tick()
    assert r0.omega == pytest.approx(0.0)

    transport.queue_empty()
    result = ex.tick()

    # error = commanded(0) - measured(+) = negative -> kp>0 -> negative trim
    assert result.omega < 0.0
    # commanded heading is unchanged by a straight leg (omega=0 throughout).
    assert ex._commanded_heading == pytest.approx(0.0)


def test_turn_commanded_heading_advances_with_profile_omega():
    ex, transport, _ = _executor()
    setpoints = [ProfileSetpoint(elapsed=0.0, v_x=0.0, omega=0.0),
                 ProfileSetpoint(elapsed=0.1, v_x=0.0, omega=1.0),
                 ProfileSetpoint(elapsed=0.2, v_x=0.0, omega=1.0)]
    transport.queue(_frame(pose=(0, 0, 0)))
    ex.begin(setpoints, target=math.pi / 2, axis="angular")
    assert ex._commanded_heading == pytest.approx(0.0)

    transport.queue(_frame(pose=(0, 0, 0)))
    ex.tick()  # sends setpoints[0] (omega=0), integrates toward setpoints[1]
    first_commanded = ex._commanded_heading

    transport.queue(_frame(pose=(0, 0, 0)))
    ex.tick()  # sends setpoints[1] (omega=1.0), integrates toward setpoints[2]
    second_commanded = ex._commanded_heading

    assert second_commanded > first_commanded  # trajectory advanced


# ---------------------------------------------------------------------------
# Synthetic TLM stream shapes: on-time, late, dropped
# ---------------------------------------------------------------------------


def test_on_time_frames_update_latest_frame_every_tick():
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=50.0, cadence=0.1)
    transport.queue(_frame(enc=(0, 0)))
    ex.begin(setpoints, target=50.0, axis="linear")

    transport.queue(_frame(enc=(10, 10)))
    ex.tick()
    assert ex.latest_frame.enc == (10, 10)

    transport.queue(_frame(enc=(20, 20)))
    ex.tick()
    assert ex.latest_frame.enc == (20, 20)


def test_late_frames_do_not_stall_ticking_and_catch_up_is_applied():
    """A 'late' delivery: one tick's drain is empty (nothing new yet), the
    NEXT tick's drain returns a multi-frame catch-up batch -- executor
    keeps the previous frame across the gap, then adopts the LATEST of the
    catch-up batch (matching the ack-ring "first match wins, re-delivery
    tolerated" precedent's own spirit: never blocks waiting)."""
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=50.0, cadence=0.1)
    transport.queue(_frame(enc=(0, 0)))
    ex.begin(setpoints, target=50.0, axis="linear")

    transport.queue_empty()  # late -- nothing new this tick
    ex.tick()
    assert ex.latest_frame.enc == (0, 0)  # holds the last known frame

    transport.queue(_frame(enc=(5, 5)), _frame(enc=(15, 15)))  # catch-up batch
    ex.tick()
    assert ex.latest_frame.enc == (15, 15)  # adopts the newest of the batch


def test_dropped_frames_forever_still_completes_the_run_via_setpoint_count():
    """No telemetry ever arrives after begin()'s own baseline read -- the
    executor must still walk every setpoint and stop cleanly (it never
    blocks waiting on telemetry to arrive, per Decision 5)."""
    ex, transport, _ = _executor()
    setpoints = _straight_setpoints(distance=30.0, cadence=0.1)
    transport.queue_empty()
    ex.begin(setpoints, target=30.0, axis="linear")

    result = None
    for _ in range(len(setpoints)):
        transport.queue_empty()
        result = ex.tick()
        if result.done:
            break

    assert result is not None
    assert result.outcome == RunOutcome.COMPLETED
    assert len(transport.twist_calls) == len(setpoints)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
