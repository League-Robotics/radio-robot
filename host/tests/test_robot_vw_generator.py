"""Unit tests for Nezha.vw() body-velocity generator (Sprint 036, ticket 004).

Tests verify:
1. test_vw_yields_per_tick          — yields once per TLM tick; state updated each tick.
2. test_vw_resends_keepalive        — VW re-sent after keepalive interval elapses.
3. test_vw_break_sends_stop_and_stream_off — break sends STOP + STREAM 0 cleanly.
4. test_vw_safety_stop_exits_cleanly — EVT safety_stop terminates the generator
                                        without raising.

CRITICAL memory safety: read_lines side_effect must NEVER be an infinite
iterator or bare MagicMock() — always use a finite side_effect list that
ends with a terminal event or an explicit StopIteration boundary. An
instant-returning MagicMock for read_lines will spin the generator
unboundedly and OOM the test process.
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock, call, patch

import pytest

from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.robot.nezha import Nezha


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mock_conn() -> MagicMock:
    """Create a mock SerialConnection with safe, non-blocking defaults."""
    conn = MagicMock()
    conn.is_open = True
    conn.mode = "relay"
    conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
    conn.send_fast.return_value = None
    conn.read_lines.return_value = []
    return conn


def _nezha(conn: MagicMock) -> Nezha:
    proto = NezhaProtocol(conn)
    return Nezha(proto)


# ===========================================================================
# Test 1: yields once per TLM tick; state updated before each yield
# ===========================================================================

class TestVwYieldsPerTick:
    """vw() yields None once per TLM frame and updates robot.state each time."""

    def test_yields_once_per_tlm_frame(self) -> None:
        """Three TLM frames → three yields; fourth call returns safety_stop."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["TLM t=200 enc=20,19"],
            ["TLM t=300 enc=30,29"],
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        tick_count = 0
        for _ in robot.vw(200, 500):
            tick_count += 1
        assert tick_count == 3

    def test_state_updated_before_each_yield(self) -> None:
        """robot.state.encoders is set from the TLM frame before each yield."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=50,48"],
            ["TLM t=200 enc=100,99"],
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        observed: list[tuple] = []
        for _ in robot.vw(200, 500):
            observed.append(robot.state.encoders)

        assert observed[0] == (50, 48)
        assert observed[1] == (100, 99)

    def test_yield_value_is_none(self) -> None:
        """Each yielded value is None (callers read robot.state directly)."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=5,5"],
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        yielded_values = list(robot.vw(100, 0))
        assert yielded_values == [None]

    def test_stream_enabled_at_start(self) -> None:
        """STREAM <period_ms> is sent before the first VW command."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        list(robot.vw(200, 500, period_ms=40))

        all_sends = [c[0][0] for c in conn.send.call_args_list]
        assert any(cmd == "STREAM 40" for cmd in all_sends), \
            f"Expected 'STREAM 40' in send calls: {all_sends}"

    def test_vw_command_sent_initially(self) -> None:
        """VW <v> <omega> is sent as the initial drive command via send_fast."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        list(robot.vw(200, 500))

        fast_calls = [c[0][0] for c in conn.send_fast.call_args_list]
        assert "VW 200 500" in fast_calls, \
            f"Expected 'VW 200 500' in send_fast calls: {fast_calls}"


# ===========================================================================
# Test 2: VW re-sent as keepalive when interval elapses
# ===========================================================================

class TestVwKeepalive:
    """VW command is re-sent as keepalive within the firmware watchdog window."""

    def test_vw_resent_when_keepalive_interval_elapses(self) -> None:
        """VW re-sent after period_ms * 0.30 / 1000 seconds have elapsed."""
        conn = _mock_conn()

        # Controlled clock: first call sets baseline, subsequent calls advance time
        # so the keepalive interval (40 * 0.30 / 1000 = 0.012 s) is exceeded.
        call_times = [0.0, 0.0, 1.0, 1.0, 1.0]
        time_iter = iter(call_times)

        def _fake_monotonic() -> float:
            try:
                return next(time_iter)
            except StopIteration:
                return 2.0

        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        with patch("robot_radio.robot.nezha.time.monotonic", side_effect=_fake_monotonic):
            list(robot.vw(200, 500, period_ms=40))

        fast_calls = [c[0][0] for c in conn.send_fast.call_args_list]
        vw_sends = [c for c in fast_calls if c.startswith("VW")]
        # Initial send + at least one keepalive re-send
        assert len(vw_sends) >= 2, \
            f"Expected at least 2 VW sends (initial + keepalive), got: {vw_sends}"

    def test_vw_not_resent_when_interval_not_elapsed(self) -> None:
        """VW is NOT re-sent when the keepalive interval has not elapsed."""
        conn = _mock_conn()

        # Clock stays at 0.0 throughout — no time passes, no keepalive needed.
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        with patch("robot_radio.robot.nezha.time.monotonic", return_value=0.0):
            list(robot.vw(200, 500, period_ms=40))

        fast_calls = [c[0][0] for c in conn.send_fast.call_args_list]
        vw_sends = [c for c in fast_calls if c.startswith("VW")]
        # Only the initial send; no keepalive because interval never elapsed
        assert len(vw_sends) == 1, \
            f"Expected exactly 1 VW send when interval hasn't elapsed, got: {vw_sends}"


# ===========================================================================
# Test 3: break sends STOP + STREAM 0
# ===========================================================================

class TestVwBreakCleanup:
    """Caller break triggers GeneratorExit; STOP and STREAM 0 are sent."""

    def test_break_sends_stop(self) -> None:
        """STOP is sent via send_fast when the caller breaks out of the loop."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["TLM t=200 enc=20,19"],  # not reached after break
            ["EVT safety_stop"],       # not reached
        ]

        robot = _nezha(conn)
        for _ in robot.vw(200, 500):
            break  # GeneratorExit after first tick

        fast_calls = [c[0][0] for c in conn.send_fast.call_args_list]
        assert "STOP" in fast_calls, \
            f"Expected 'STOP' in send_fast calls after break: {fast_calls}"

    def test_break_sends_stream_off(self) -> None:
        """STREAM 0 is sent (via conn.send) when the caller breaks."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["EVT safety_stop"],  # not reached
        ]

        robot = _nezha(conn)
        for _ in robot.vw(200, 500, period_ms=40):
            break

        all_sends = [c[0][0] for c in conn.send.call_args_list]
        assert any(cmd == "STREAM 0" for cmd in all_sends), \
            f"Expected 'STREAM 0' in send calls after break: {all_sends}"

    def test_break_does_not_raise(self) -> None:
        """Breaking out of the vw() loop does not propagate an exception."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        try:
            for _ in robot.vw(200, 500):
                break
        except Exception as exc:
            pytest.fail(f"vw() raised an unexpected exception on break: {exc!r}")

    def test_break_stop_sent_before_stream_off(self) -> None:
        """STOP is sent via send_fast before STREAM 0 (order of cleanup)."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        call_order: list[str] = []

        def record_fast(cmd: str) -> None:
            call_order.append(f"fast:{cmd}")

        def record_send(cmd: str, **kw) -> dict:
            call_order.append(f"send:{cmd}")
            return {"sent": cmd, "mode": "relay", "responses": ["OK"]}

        conn.send_fast.side_effect = record_fast
        conn.send.side_effect = record_send

        for _ in robot.vw(200, 500):
            break

        # Find the cleanup STOP and STREAM 0 in order
        stop_idx = next(
            (i for i, s in enumerate(call_order) if s == "fast:STOP"), None
        )
        stream_off_idx = next(
            (i for i, s in enumerate(call_order) if s == "send:STREAM 0"), None
        )
        assert stop_idx is not None, f"STOP not found in call order: {call_order}"
        assert stream_off_idx is not None, f"STREAM 0 not found in call order: {call_order}"
        assert stop_idx < stream_off_idx, \
            f"STOP must come before STREAM 0; order: {call_order}"


# ===========================================================================
# Test 4: EVT safety_stop exits cleanly
# ===========================================================================

class TestVwSafetyStop:
    """EVT safety_stop terminates the generator naturally without raising."""

    def test_safety_stop_as_first_line_exits_cleanly(self) -> None:
        """safety_stop as the very first line: generator yields nothing and exits."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        ticks = list(robot.vw(200, 500))
        assert ticks == [], f"Expected no yields before safety_stop, got: {ticks}"

    def test_safety_stop_after_tlm_exits_cleanly(self) -> None:
        """safety_stop after some TLM frames: generator terminates without raising."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        try:
            ticks = list(robot.vw(200, 500))
        except Exception as exc:
            pytest.fail(f"vw() raised on safety_stop: {exc!r}")
        assert len(ticks) == 1

    def test_safety_stop_does_not_send_explicit_stop(self) -> None:
        """On EVT safety_stop the generator exits via return — no STOP sent.

        The firmware already stopped; sending STOP would be redundant.
        (GeneratorExit cleanup is bypassed when the generator exits naturally.)
        """
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        list(robot.vw(200, 500))

        fast_calls = [c[0][0] for c in conn.send_fast.call_args_list]
        assert "STOP" not in fast_calls, \
            f"STOP must NOT be sent on natural safety_stop exit: {fast_calls}"

    def test_safety_stop_state_reflects_last_tlm(self) -> None:
        """After safety_stop, robot.state reflects the last TLM frame received."""
        conn = _mock_conn()
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=77,75"],
            ["EVT safety_stop"],
        ]

        robot = _nezha(conn)
        list(robot.vw(200, 500))

        assert robot.state.encoders == (77, 75)
