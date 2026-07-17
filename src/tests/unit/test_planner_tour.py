"""src/tests/unit/test_planner_tour.py -- 107-002 (SUC-033), rewritten 109-008
for MOVE-queue tours.

Covers `robot_radio.planner.tour`: the pure `parse_tour()` parser (regression-
protected directly against `TOUR_1`/`TOUR_2`'s own real geometry data), and
`run_tour()`'s leg-chaining/closure-bookkeeping/preemption behavior against a
`FakeTransport` double -- no real serial port, no sim, no hardware.

109-008 rewire: `run_tour()` no longer streams a `profile.py` setpoint
sequence through a `StreamingExecutor` -- it sends one `Move` per leg
(`transport.move()`) and waits for that leg's own terminal ack-ring entry
(`AckEntry.status` -- DONE/TRIVIAL/SUPERSEDED/FLUSHED/TIMEOUT/SOLVE_FAIL, see
`tour.py`'s own file header). `FakeTransport` here mirrors
`test_planner_executor.py`'s own double convention but exposes `move()`
instead of `twist()`, and its `current_frame`/`acks` shape is what a test
controls to simulate a leg's own Move reaching a terminal status.

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
    DEFAULT_V_MAX,
    TOUR_1,
    TOUR_2,
    TourClosure,
    TourLeg,
    _compute_closure,
    parse_tour,
    run_tour,
)
from robot_radio.robot.pb2 import telemetry_pb2
from robot_radio.robot.protocol import AckEntry, TLMFrame

# ---------------------------------------------------------------------------
# Fake transport -- "current frame" double (see module docstring above)
# ---------------------------------------------------------------------------


class FakeTransport:
    def __init__(self) -> None:
        self.move_calls: list[dict] = []
        self.stop_calls: int = 0
        self._next_id = 0
        self.current_frame: TLMFrame | None = None

    def move(self, **kwargs) -> int:
        self._next_id += 1
        self.move_calls.append(kwargs)
        return self._next_id

    def stop(self) -> int:
        self.stop_calls += 1
        return 0

    def read_pending_binary_tlm_frames(self) -> list[TLMFrame]:
        return [self.current_frame] if self.current_frame is not None else []


def _ack(corr_id: int, status: int = telemetry_pb2.ACK_STATUS_DONE) -> AckEntry:
    return AckEntry(corr_id=corr_id, ok=(status == telemetry_pb2.ACK_STATUS_OK),
                    err_code=0, status=status)


def _frame(pose=(0, 0, 0), acks: tuple = ()):
    return TLMFrame(pose=pose, acks=acks, fault_bits=0, event_bits=0)


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


# Small bounds so a timeout (a test that never injects the expected ack) fails
# fast rather than hanging the suite.
_FAST_KW = dict(move_timeout=1.0, poll_interval=0.0)


def _short_legs():
    return [
        TourLeg(kind="distance", value=10.0, speed=1000.0),
        TourLeg(kind="turn", value=10.0),
        TourLeg(kind="distance", value=10.0, speed=1000.0),
    ]


# ---------------------------------------------------------------------------
# parse_tour() -- regression-protects TOUR_1/TOUR_2's own real geometry
# (unchanged by 109-008 -- parse_tour() itself never touched profile/executor)
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
# 109-008: Move-sequence-per-leg encoding for Tour 1/Tour 2 -- every parsed
# leg translates to the SAME `Move` kwargs `run_tour()` actually sends
# (`tour_module._move_kwargs_for_leg()`), independent of ever running the
# tour end to end. A "distance" leg is a straight DISTANCE-mode arc
# (delta_heading=0, v_max honors the leg's own wire-authored speed); a
# "turn" leg is a pure pivot (distance=0, v_max=0.0 -- ignored by firmware
# for a pivot, see tour.py's own file header).
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("tour_name,tour_data", [("TOUR_1", TOUR_1), ("TOUR_2", TOUR_2)])
def test_move_kwargs_for_every_leg_matches_the_parsed_geometry(tour_name, tour_data):
    legs = parse_tour(tour_data)
    for leg in legs:
        kwargs = tour_module._move_kwargs_for_leg(leg, v_max=DEFAULT_V_MAX)
        assert set(kwargs) == {"distance", "delta_heading", "v_max"}
        if leg.kind == "distance":
            assert kwargs["distance"] == pytest.approx(leg.value)
            assert kwargs["delta_heading"] == pytest.approx(0.0)
            assert kwargs["v_max"] == pytest.approx(leg.speed if leg.speed else DEFAULT_V_MAX)
        else:
            assert kwargs["distance"] == pytest.approx(0.0)
            assert kwargs["delta_heading"] == pytest.approx(math.radians(leg.value))
            assert kwargs["v_max"] == pytest.approx(0.0)


def test_move_kwargs_never_produce_a_timed_command():
    """Tours are DISTANCE mode exclusively -- `_move_kwargs_for_leg()` must
    never populate `time`/`omega`/`replace` (TIMED mode's own fields; a
    tour leg relies on the DEFAULT `move()` kwargs for these, matching
    `Move`'s own `time == 0` => DISTANCE mode discriminant)."""
    for leg in parse_tour(TOUR_1) + parse_tour(TOUR_2):
        kwargs = tour_module._move_kwargs_for_leg(leg, v_max=DEFAULT_V_MAX)
        assert "time" not in kwargs
        assert "omega" not in kwargs
        assert "replace" not in kwargs


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
    legs = _short_legs()
    # Pre-seed the ack ring with DONE for every leg's own (deterministic,
    # sequentially-assigned) id -- 3 legs -> ids 1, 2, 3 -- so each leg's
    # wait loop finds its terminal ack on the FIRST poll.
    transport.current_frame = _frame(pose=(0, 0, 0), acks=(_ack(1), _ack(2), _ack(3)))
    params = _params()
    heading = _heading(params)

    def on_leg(index, total, leg, result):
        if index == total - 1:
            transport.current_frame = _frame(pose=(120, -40, 9000), acks=(_ack(1), _ack(2), _ack(3)))

    result = run_tour(transport, params, heading, legs, on_leg=on_leg, **_FAST_KW)

    assert len(result.legs) == 3
    assert all(leg_result.outcome == RunOutcome.COMPLETED for leg_result in result.legs)
    assert result.stopped_at is None
    assert result.stopped_outcome is None
    # Every completed leg sent exactly one Move -- 3 legs -> 3 move() calls.
    assert len(transport.move_calls) == 3

    assert result.legs[0].heading_before == pytest.approx(0.0)
    assert result.closure.start_pose == pytest.approx((0.0, 0.0, 0.0))
    assert result.closure.end_pose == pytest.approx((120.0, -40.0, math.pi / 2.0))
    assert result.closure.position_delta == pytest.approx(math.hypot(120.0, -40.0))
    assert result.closure.heading_delta == pytest.approx(math.pi / 2.0)


def test_clean_run_never_attempts_a_leg_after_the_last_one():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0), acks=(_ack(1), _ack(2), _ack(3)))
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    seen_indices: list[int] = []

    def on_leg(index, total, leg, result):
        seen_indices.append(index)

    run_tour(transport, params, heading, legs, on_leg=on_leg, **_FAST_KW)

    assert seen_indices == [0, 1, 2]


def test_one_leg_lookahead_enqueues_leg_1_before_leg_0_completes():
    """SUC-003: leg N+1's own Move is sent WHILE leg N is still active, not
    after it completes -- both leg 0 and leg 1's Move calls must already be
    present before this function even runs its own wait loop for leg 0."""
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))  # no acks yet -- nothing completes
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    def on_leg(index, total, leg, result):
        pass

    # Use a should_stop that fires immediately, so the tour aborts on leg 0's
    # very first poll -- but by THEN, both leg 0 and leg 1 should already
    # have been sent (the lookahead send happens before the wait loop starts).
    result = run_tour(transport, params, heading, legs,
                      should_stop=lambda: True, **_FAST_KW)

    assert len(transport.move_calls) == 2  # leg 0 AND leg 1, leg 2 never (leg 0 never completed)
    assert result.legs[0].outcome == RunOutcome.STOPPED


# ---------------------------------------------------------------------------
# run_tour(): a leg that faults mid-tour -- remaining legs NOT attempted
# ---------------------------------------------------------------------------


def test_fault_mid_tour_stops_immediately_reports_leg_index_and_outcome():
    transport = FakeTransport()
    # Leg 0 (id=1) completes DONE; leg 1 (id=2) completes SOLVE_FAIL (a fault);
    # leg 2 (id=3) is never attempted.
    transport.current_frame = _frame(pose=(0, 0, 0),
                                     acks=(_ack(1), _ack(2, telemetry_pb2.ACK_STATUS_SOLVE_FAIL)))
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    result = run_tour(transport, params, heading, legs, **_FAST_KW)

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
    # start_pose IS still available -- captured before leg 1 (index 0)'s own Move.
    assert result.closure.start_pose == pytest.approx((0.0, 0.0, 0.0))
    # 3 Move calls -- leg 0 and leg 1 are sent up front (one-leg lookahead),
    # and leg 2's Move is ALSO sent once leg 0 completes normally (the
    # lookahead send is keyed off the PRECEDING leg completing, independent
    # of what happens to the leg that runs next -- leg 1 here faults, but
    # that doesn't retroactively un-send leg 2's already-queued Move).
    assert len(transport.move_calls) == 3


def test_move_enqueue_rejection_is_a_fault():
    """ERR (e.g. ERR_FULL -- the queue was full) is a terminal, non-OK ack
    just like a later completion event -- run_tour() must treat it as a
    fault, not hang waiting for a completion event that will never come."""
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0), acks=(_ack(1, telemetry_pb2.ACK_STATUS_ERR),))
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    result = run_tour(transport, params, heading, legs, **_FAST_KW)

    assert result.legs[0].outcome == RunOutcome.FAULT
    assert result.stopped_at == 0


def test_move_timeout_with_no_terminal_ack_is_a_fault():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))  # never carries a terminal ack
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    result = run_tour(transport, params, heading, legs, move_timeout=0.02, poll_interval=0.0)

    assert result.legs[0].outcome == RunOutcome.FAULT
    assert result.stopped_at == 0


# ---------------------------------------------------------------------------
# run_tour(): preemption mid-leg via should_stop()
# ---------------------------------------------------------------------------


def test_should_stop_preempts_mid_leg_no_further_legs_attempted():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0))  # never completes on its own
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    polls = {"count": 0}

    def should_stop():
        polls["count"] += 1
        return polls["count"] >= 2

    result = run_tour(transport, params, heading, legs, should_stop=should_stop, **_FAST_KW)

    assert len(result.legs) == 1
    assert result.legs[0].outcome == RunOutcome.STOPPED
    assert result.stopped_at == 0
    assert result.stopped_outcome == RunOutcome.STOPPED
    assert transport.stop_calls == 1  # the immediate stop() call on preemption


# ---------------------------------------------------------------------------
# row_callback / on_leg hooks -- independent, both optional
# ---------------------------------------------------------------------------


def test_row_callback_receives_every_poll_across_the_whole_tour():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0), acks=(_ack(1), _ack(2), _ack(3)))
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    rows: list[tuple[int, int]] = []

    def row_callback(tick_index, leg_index, leg, result, frame):
        rows.append((tick_index, leg_index))

    run_tour(transport, params, heading, legs, row_callback=row_callback, **_FAST_KW)

    # 3 legs, each completing on its own first poll -> 3 rows, tick_index
    # monotonically increasing across the WHOLE tour (not reset per leg).
    assert [t for t, _ in rows] == [0, 1, 2]
    assert [leg_index for _, leg_index in rows] == [0, 1, 2]


def test_on_leg_receives_every_leg_result_in_order_with_correct_total():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0), acks=(_ack(1), _ack(2), _ack(3)))
    params = _params()
    heading = _heading(params)
    legs = _short_legs()

    calls: list[tuple[int, int, str]] = []

    def on_leg(index, total, leg, result):
        calls.append((index, total, leg.kind))

    run_tour(transport, params, heading, legs, on_leg=on_leg, **_FAST_KW)

    assert calls == [(0, 3, "distance"), (1, 3, "turn"), (2, 3, "distance")]


def test_run_tour_works_with_neither_hook_supplied():
    """Ticket 003's TestGUI can ignore the per-tick hook entirely -- confirm
    run_tour() doesn't require either optional callback."""
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0), acks=(_ack(1), _ack(2), _ack(3)))
    params = _params()
    heading = _heading(params)

    result = run_tour(transport, params, heading, _short_legs(), **_FAST_KW)

    assert len(result.legs) == 3
    assert result.stopped_at is None


# ---------------------------------------------------------------------------
# run_tour(): a straight leg honors its own wire-authored speed
# ---------------------------------------------------------------------------


def test_distance_leg_v_max_uses_the_legs_own_speed_when_present():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0), acks=(_ack(1),))
    params = _params()
    heading = _heading(params)
    leg = TourLeg(kind="distance", value=500.0, speed=77.0)

    run_tour(transport, params, heading, [leg], v_max=999.0, **_FAST_KW)

    assert len(transport.move_calls) == 1
    assert transport.move_calls[0]["v_max"] == pytest.approx(77.0)
    assert transport.move_calls[0]["distance"] == pytest.approx(500.0)
    assert transport.move_calls[0]["delta_heading"] == pytest.approx(0.0)


def test_distance_leg_falls_back_to_run_tour_v_max_when_leg_has_none():
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0), acks=(_ack(1),))
    params = _params()
    heading = _heading(params)
    leg = TourLeg(kind="distance", value=500.0, speed=None)

    run_tour(transport, params, heading, [leg], v_max=42.0, **_FAST_KW)

    assert transport.move_calls[0]["v_max"] == pytest.approx(42.0)


def test_turn_leg_becomes_a_pure_pivot_move_with_zero_v_max():
    """RT carries no rate field on the wire, and Motion::Executor plans a
    pivot's rotational channel off PlannerConfig, never the wire Move.v_max
    -- so a turn leg's own Move always carries distance=0, v_max=0.0."""
    transport = FakeTransport()
    transport.current_frame = _frame(pose=(0, 0, 0), acks=(_ack(1),))
    params = _params()
    heading = _heading(params)
    leg = TourLeg(kind="turn", value=45.0)
    assert leg.speed is None

    run_tour(transport, params, heading, [leg], **_FAST_KW)

    call = transport.move_calls[0]
    assert call["distance"] == pytest.approx(0.0)
    assert call["v_max"] == pytest.approx(0.0)
    assert call["delta_heading"] == pytest.approx(math.radians(45.0))


# ---------------------------------------------------------------------------
# Guard rails
# ---------------------------------------------------------------------------


def test_run_tour_rejects_an_empty_leg_list():
    transport = FakeTransport()
    params = _params()
    heading = _heading(params)
    with pytest.raises(ValueError, match="non-empty"):
        run_tour(transport, params, heading, [], **_FAST_KW)


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
