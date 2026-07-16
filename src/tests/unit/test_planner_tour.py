"""src/tests/unit/test_planner_tour.py -- 107-002 (SUC-033).

Covers `robot_radio.planner.tour`: the pure `parse_tour()` parser (regression-
protected directly against `TOUR_1`/`TOUR_2`'s own real geometry data), and
`run_tour()`'s leg-chaining/closure-bookkeeping/preemption behavior against a
`FakeTransport` double -- no real serial port, no sim, no hardware.

`FakeTransport` here is deliberately simpler than `test_planner_executor.py`'s
own batch-queue double: `read_pending_binary_tlm_frames()` always returns
whatever single frame is currently set on `transport.current_frame` (`None`
means "nothing queued yet", matching a genuinely empty telemetry stream).
This module's own tests are about LEG CHAINING and CLOSURE MATH, not about
`StreamingExecutor`'s own per-tick telemetry-staleness handling (already
exhaustively covered by `test_planner_executor.py`) -- a "current frame"
double keeps every scenario below deterministic without needing to count
exactly how many `read_pending_binary_tlm_frames()` calls `begin()`'s own
bounded retry makes. Tests that need STAGED telemetry (a value that changes
partway through a run) mutate `transport.current_frame` from inside a
`row_callback`/`on_leg` hook -- both fire synchronously, in-order, from
inside `run_tour()`'s own call stack, so this is exact, not timing-dependent.

Collected under `src/tests/unit/` per `pyproject.toml`'s `testpaths`.
"""

from __future__ import annotations

import ast
import inspect
import math
from types import SimpleNamespace

import pytest

from robot_radio.planner import tour as tour_module
from robot_radio.planner.executor import RunOutcome
from robot_radio.planner.heading import HeadingCorrector
from robot_radio.planner.model import PlannerParams
from robot_radio.planner.tour import (
    TOUR_1,
    TOUR_2,
    TourClosure,
    TourLeg,
    _compute_closure,
    parse_tour,
    run_tour,
)
from robot_radio.robot.protocol import TLMFrame


# ---------------------------------------------------------------------------
# Fake transport -- "current frame" double (see module docstring above)
# ---------------------------------------------------------------------------


class FakeTransport:
    def __init__(self) -> None:
        self.twist_calls: list[tuple[float, float, float]] = []
        self.stop_calls: int = 0
        self._corr_id = 0
        self.current_frame: TLMFrame | None = None

    def twist(self, v_x: float, omega: float, duration: float) -> int:
        self._corr_id += 1
        self.twist_calls.append((v_x, omega, duration))
        return self._corr_id

    def stop(self) -> int:
        self._corr_id += 1
        self.stop_calls += 1
        return self._corr_id

    def read_pending_binary_tlm_frames(self) -> list[TLMFrame]:
        return [self.current_frame] if self.current_frame is not None else []


def _frame(pose=(0, 0, 0), fault_bits=0):
    return TLMFrame(pose=pose, fault_bits=fault_bits, event_bits=0)


def _params(**overrides):
    defaults = dict(streaming_interval=0.05, link_latency_margin=0.05)
    defaults.update(overrides)
    return PlannerParams(**defaults)


def _heading(params, otos_untrusted=True):
    robot_config = SimpleNamespace(geometry=SimpleNamespace(otos_untrusted=otos_untrusted))
    return HeadingCorrector(params, robot_config=robot_config)


def _clock():
    counter = iter(i * 0.01 for i in range(1_000_000))
    return lambda: next(counter)


# Extreme limits collapse every profile to exactly 2 setpoints (one non-zero
# sample at t=0, one terminal zero) -- see this file's own header: these
# tests are about chaining/closure, not profile shape (already covered by
# test_planner_profile.py/test_planner_executor.py).
_FAST_KW = dict(v_max=1000.0, a_max=100000.0, omega_max=1000.0, alpha_max=100000.0, cadence=0.05)


def _short_legs():
    return [
        TourLeg(kind="distance", value=10.0, speed=1000.0),
        TourLeg(kind="turn", value=10.0),
        TourLeg(kind="distance", value=10.0, speed=1000.0),
    ]


# ---------------------------------------------------------------------------
# parse_tour() -- regression-protects TOUR_1/TOUR_2's own real geometry
# ---------------------------------------------------------------------------


def test_parse_tour_1_leg_count_matches_step_count():
    legs = parse_tour(TOUR_1)
    assert len(legs) == len(TOUR_1) == 13


def test_parse_tour_1_first_and_last_leg():
    legs = parse_tour(TOUR_1)
    assert legs[0] == TourLeg(kind="distance", value=345.0, speed=200.0)
    assert legs[-1] == TourLeg(kind="distance", value=345.0, speed=200.0)


def test_parse_tour_1_turn_legs():
    legs = parse_tour(TOUR_1)
    turn_legs = [leg for leg in legs if leg.kind == "turn"]
    assert len(turn_legs) == 6
    assert all(leg.value == pytest.approx(90.0) for leg in turn_legs)
    assert all(leg.speed is None for leg in turn_legs)


def test_parse_tour_2_leg_count_matches_step_count():
    legs = parse_tour(TOUR_2)
    assert len(legs) == len(TOUR_2) == 15


def test_parse_tour_2_first_and_last_leg():
    legs = parse_tour(TOUR_2)
    assert legs[0] == TourLeg(kind="distance", value=345.0, speed=200.0)
    assert legs[-1] == TourLeg(kind="distance", value=345.0, speed=200.0)


def test_parse_tour_2_preserves_negative_turn_signs():
    legs = parse_tour(TOUR_2)
    # "RT -21700" -> -217.0 deg; "RT -9000" -> -90.0 deg.
    turn_values = [leg.value for leg in legs if leg.kind == "turn"]
    assert turn_values == pytest.approx([90.0, 124.0, -217.0, 146.0, 215.0, -90.0, -90.0])


def test_parse_tour_rejects_unknown_verb():
    with pytest.raises(ValueError, match="unsupported step verb"):
        parse_tour(["FOO 1 2 3"])


def test_parse_tour_rejects_malformed_d_step():
    with pytest.raises(ValueError, match="malformed D step"):
        parse_tour(["D 200 200"])


def test_parse_tour_rejects_malformed_rt_step():
    with pytest.raises(ValueError, match="malformed RT step"):
        parse_tour(["RT"])


def test_parse_tour_rejects_empty_step():
    with pytest.raises(ValueError, match="empty step"):
        parse_tour([""])


# ---------------------------------------------------------------------------
# tour.py never imports NezhaProtocol/SerialConnection/SimConnection
# directly -- AST-based (not raw-text grep, mirroring
# test_planner_executor.py's own `_ast_calls_named` convention): this
# module's own docstrings and TYPE_CHECKING-only imports mention some of
# these names, which would false-positive a plain substring search.
# ---------------------------------------------------------------------------


def _ast_imports_name(module, name: str) -> bool:
    tree = ast.parse(inspect.getsource(module))
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if any(alias.name == name for alias in node.names):
                return True
        if isinstance(node, ast.Import):
            if any(alias.name == name for alias in node.names):
                return True
    return False


@pytest.mark.parametrize("name", ["NezhaProtocol", "SerialConnection", "SimConnection"])
def test_tour_module_never_imports_transport_concretes(name):
    assert not _ast_imports_name(tour_module, name)


# ---------------------------------------------------------------------------
# run_tour(): clean multi-leg run -- COMPLETED for every leg, closure computed
# ---------------------------------------------------------------------------


def test_clean_multi_leg_run_completes_every_leg_and_computes_closure():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    def on_leg(index, total, leg, result):
        # Drift the "current" telemetry only once the FINAL leg has finished
        # its own run -- run_tour()'s own final settle-window drain (which
        # happens right after this callback returns) is what should observe
        # the drift, not any earlier per-leg reading.
        if index == total - 1:
            transport.current_frame = _frame(pose=(120, -40, 9000))  # +90 deg

    result = run_tour(transport, params, heading, legs, sleep_fn=lambda s: None,
                      clock_fn=_clock(), on_leg=on_leg, **_FAST_KW)

    assert len(result.legs) == 3
    assert all(leg_result.outcome == RunOutcome.COMPLETED for leg_result in result.legs)
    assert result.stopped_at is None
    assert result.stopped_outcome is None
    # Every leg sent an explicit terminal stop() (executor.py binding
    # requirement #7) -- 3 legs -> 3 stop() calls, no fault/overshoot stops.
    assert transport.stop_calls == 3

    assert result.legs[0].heading_before == pytest.approx(0.0)
    assert result.closure.start_pose == pytest.approx((0.0, 0.0, 0.0))
    assert result.closure.end_pose == pytest.approx((120.0, -40.0, math.pi / 2.0))
    assert result.closure.position_delta == pytest.approx(math.hypot(120.0, -40.0))
    assert result.closure.heading_delta == pytest.approx(math.pi / 2.0)


def test_clean_run_never_attempts_a_leg_after_the_last_one():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    seen_indices: list[int] = []

    def on_leg(index, total, leg, result):
        seen_indices.append(index)

    run_tour(transport, params, heading, legs, sleep_fn=lambda s: None, clock_fn=_clock(),
             on_leg=on_leg, **_FAST_KW)

    assert seen_indices == [0, 1, 2]


# ---------------------------------------------------------------------------
# run_tour(): a leg that faults mid-tour -- remaining legs NOT attempted
# ---------------------------------------------------------------------------


def test_fault_mid_tour_stops_immediately_reports_leg_index_and_outcome():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0), fault_bits=0)
    params = _params()
    heading = _heading(params)
    legs = _short_legs()  # leg index 1 (turn) is where the fault appears

    ticks_seen_for_leg1 = 0

    def row_callback(tick_index, leg_index, leg, result, frame):
        nonlocal ticks_seen_for_leg1
        if leg_index == 1:
            ticks_seen_for_leg1 += 1
            if ticks_seen_for_leg1 == 1:
                # A NEW fault bit relative to leg 1's own begin()-time
                # baseline (0) -- must trip on the NEXT tick, never silently.
                transport.current_frame = _frame(pose=(0, 0, 0), fault_bits=1)

    result = run_tour(transport, params, heading, legs, sleep_fn=lambda s: None,
                      clock_fn=_clock(), row_callback=row_callback, **_FAST_KW)

    assert len(result.legs) == 2  # leg 2 (index 2) never attempted
    assert result.legs[0].outcome == RunOutcome.COMPLETED
    assert result.legs[1].outcome == RunOutcome.FAULT
    assert result.legs[1].fault is True
    assert result.stopped_at == 1
    assert result.stopped_outcome == RunOutcome.FAULT

    # No closure -- the tour never reached its own final leg's settle window.
    assert result.closure.end_pose is None
    assert result.closure.position_delta is None
    assert result.closure.heading_delta is None
    # start_pose IS still available -- captured at leg 1 (index 0)'s own begin().
    assert result.closure.start_pose == pytest.approx((0.0, 0.0, 0.0))


def test_overshoot_mid_tour_stops_immediately_and_reports_it():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))
    params = _params(overshoot_bound_angular=0.01)  # tight -- easy to trip deliberately
    heading = _heading(params)
    legs = _short_legs()

    def row_callback(tick_index, leg_index, leg, result, frame):
        if leg_index == 1 and result.outcome is None:
            # Report a wildly-drifted heading -- well outside the tight
            # overshoot bound above -- for the turn leg's own next tick.
            transport.current_frame = _frame(pose=(0, 0, 18000))  # +180 deg

    result = run_tour(transport, params, heading, legs, sleep_fn=lambda s: None,
                      clock_fn=_clock(), row_callback=row_callback, **_FAST_KW)

    assert len(result.legs) == 2
    assert result.legs[1].outcome == RunOutcome.OVERSHOOT
    assert result.stopped_at == 1
    assert result.stopped_outcome == RunOutcome.OVERSHOOT


# ---------------------------------------------------------------------------
# run_tour(): preemption mid-leg via should_stop()
# ---------------------------------------------------------------------------


def test_should_stop_preempts_mid_leg_no_further_legs_attempted():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    polls = {"count": 0}

    def should_stop():
        polls["count"] += 1
        # False on the first poll (tick 1 of leg 0 proceeds), True on the
        # second (stop before tick 2 of leg 0 -- mid-leg, not at a boundary).
        return polls["count"] >= 2

    result = run_tour(transport, params, heading, legs, sleep_fn=lambda s: None,
                      clock_fn=_clock(), should_stop=should_stop, **_FAST_KW)

    assert len(result.legs) == 1
    assert result.legs[0].outcome == RunOutcome.STOPPED
    assert result.legs[0].tick_count == 1  # only the first tick was sent
    assert result.stopped_at == 0
    assert result.stopped_outcome == RunOutcome.STOPPED
    assert transport.stop_calls == 1  # stop_now()'s own immediate stop() call
    # Only ONE twist() was ever sent for the whole tour -- leg 0's own single
    # completed tick -- proving no further tick, let alone a further leg,
    # was attempted after the stop request.
    assert len(transport.twist_calls) == 1


# ---------------------------------------------------------------------------
# row_callback / on_leg hooks -- independent, both optional
# ---------------------------------------------------------------------------


def test_row_callback_receives_every_tick_across_the_whole_tour():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    rows: list[tuple[int, int]] = []

    def row_callback(tick_index, leg_index, leg, result, frame):
        rows.append((tick_index, leg_index))

    run_tour(transport, params, heading, legs, sleep_fn=lambda s: None, clock_fn=_clock(),
             row_callback=row_callback, **_FAST_KW)

    # 3 legs * 2 ticks each (see _FAST_KW's own comment) = 6 rows, tick_index
    # monotonically increasing across the WHOLE tour (not reset per leg).
    assert [t for t, _ in rows] == [0, 1, 2, 3, 4, 5]
    assert [leg_index for _, leg_index in rows] == [0, 0, 1, 1, 2, 2]


def test_on_leg_receives_every_leg_result_in_order_with_correct_total():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    calls: list[tuple[int, int, str]] = []

    def on_leg(index, total, leg, result):
        calls.append((index, total, leg.kind))

    run_tour(transport, params, heading, legs, sleep_fn=lambda s: None, clock_fn=_clock(),
             on_leg=on_leg, **_FAST_KW)

    assert calls == [(0, 3, "distance"), (1, 3, "turn"), (2, 3, "distance")]


def test_run_tour_works_with_neither_hook_supplied():
    """Ticket 003's TestGUI can ignore the per-tick hook entirely (AC4) --
    confirm run_tour() doesn't require either optional callback."""
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))
    params = _params()
    heading = _heading(params)

    result = run_tour(transport, params, heading, _short_legs(), sleep_fn=lambda s: None,
                      clock_fn=_clock(), **_FAST_KW)

    assert len(result.legs) == 3
    assert result.stopped_at is None


# ---------------------------------------------------------------------------
# run_tour(): a straight leg honors its own wire-authored speed
# ---------------------------------------------------------------------------


def test_distance_leg_v_max_uses_the_legs_own_speed_when_present():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))
    params = _params()
    heading = _heading(params)
    leg = TourLeg(kind="distance", value=500.0, speed=77.0)

    run_tour(transport, params, heading, [leg], sleep_fn=lambda s: None, clock_fn=_clock(),
            v_max=999.0, a_max=100.0, cadence=0.05)

    # The profile's own peak/cruise v_x never exceeds the leg's OWN 77mm/s
    # speed -- not the (much larger) v_max fallback passed to run_tour().
    assert all(abs(v_x) <= 77.0 + 1e-6 for v_x, _, _ in transport.twist_calls)


def test_turn_leg_always_uses_run_tour_omega_alpha_defaults():
    """RT carries no rate field on the wire -- a turn leg's own profile
    always uses run_tour()'s omega_max/alpha_max arguments, never anything
    read off the leg itself (TourLeg.speed is always None for a turn)."""
    leg = TourLeg(kind="turn", value=45.0)
    assert leg.speed is None


# ---------------------------------------------------------------------------
# Guard rails
# ---------------------------------------------------------------------------


def test_run_tour_rejects_an_empty_leg_list():
    transport = FakeTransport()
    params = _params()
    heading = _heading(params)
    with pytest.raises(ValueError, match="non-empty"):
        run_tour(transport, params, heading, [], sleep_fn=lambda s: None, clock_fn=_clock())


# ---------------------------------------------------------------------------
# Closure math -- direct unit tests of _compute_closure()
# ---------------------------------------------------------------------------


def test_compute_closure_simple_drift():
    start = (0.0, 0.0, 0.0)
    end = (30.0, 40.0, math.pi / 4.0)
    closure = _compute_closure(start, end)
    assert closure.position_delta == pytest.approx(50.0)  # 3-4-5 triangle
    assert closure.heading_delta == pytest.approx(math.pi / 4.0)


def test_compute_closure_wraps_heading_the_short_way():
    """A start heading near +pi and an end heading near -pi are only a small
    angular step apart (wrapping through +-pi), NOT the ~2*pi naive
    difference -- normalize_angle() (reused from controllers.pid, no new
    angle-wrap implementation) must produce the short-way delta."""
    start = (0.0, 0.0, 3.0)
    end = (0.0, 0.0, -3.0)
    closure = _compute_closure(start, end)
    assert closure.heading_delta == pytest.approx(2 * math.pi - 6.0)
    assert abs(closure.heading_delta) < 1.0


def test_compute_closure_no_movement_is_zero_delta():
    pose = (10.0, -5.0, 1.2)
    closure = _compute_closure(pose, pose)
    assert closure.position_delta == pytest.approx(0.0)
    assert closure.heading_delta == pytest.approx(0.0)


def test_compute_closure_missing_start_or_end_is_none():
    assert _compute_closure(None, (0.0, 0.0, 0.0)) == TourClosure(
        start_pose=None, end_pose=(0.0, 0.0, 0.0), position_delta=None, heading_delta=None)
    assert _compute_closure((0.0, 0.0, 0.0), None) == TourClosure(
        start_pose=(0.0, 0.0, 0.0), end_pose=None, position_delta=None, heading_delta=None)
    assert _compute_closure(None, None) == TourClosure(
        start_pose=None, end_pose=None, position_delta=None, heading_delta=None)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
