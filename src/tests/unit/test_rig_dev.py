"""src/tests/unit/test_rig_dev.py — 104-006 (bench script family rewritten onto
the binary twist/config/stop plane).

Unit coverage for the PURE (no I/O, no hardware) helpers in
`src/tests/bench/rig_dev.py` — `waveform()` and `secondary_to_dict()` — plus the
`Rig` class's own forwarding contract, exercised against fake `conn`/`proto`
objects (never a real `SerialConnection`/serial port, matching the project's
own `_FakeFastConn`-style precedent in `test_twist_stop_ack_matcher.py`).

`src/tests/bench/` is "HITL CLI tools, not pytest-collected" (`tests/CLAUDE.md`),
so this test loads `rig_dev.py` directly by file path via `importlib`
rather than a package import, mirroring `test_device_bus_bringup_bench.py`'s
own precedent (before its retirement, 104-006) for testing a `src/tests/bench/`
script's pure logic in isolation.
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest

from robot_radio.robot.pb2 import telemetry_pb2
from robot_radio.robot.protocol import AckEntry, TLMFrame

_BENCH_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "bench" / "rig_dev.py"


def _load_bench_module():
    spec = importlib.util.spec_from_file_location("rig_dev", _BENCH_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def rig_dev():
    return _load_bench_module()


# ---------------------------------------------------------------------------
# waveform()
# ---------------------------------------------------------------------------

class TestWaveform:
    def test_sine_starts_at_zero(self, rig_dev):
        assert rig_dev.waveform("sine", 0.0, period=4.0, amp=150.0) == pytest.approx(0.0)

    def test_sine_peaks_at_quarter_period(self, rig_dev):
        v = rig_dev.waveform("sine", 1.0, period=4.0, amp=150.0)
        assert v == pytest.approx(150.0)

    def test_sine_troughs_at_three_quarter_period(self, rig_dev):
        v = rig_dev.waveform("sine", 3.0, period=4.0, amp=150.0)
        assert v == pytest.approx(-150.0)

    def test_square_high_first_half(self, rig_dev):
        assert rig_dev.waveform("square", 0.0, period=4.0, amp=100.0) == 100.0
        assert rig_dev.waveform("square", 1.9, period=4.0, amp=100.0) == 100.0

    def test_square_low_second_half(self, rig_dev):
        assert rig_dev.waveform("square", 2.1, period=4.0, amp=100.0) == -100.0

    def test_wraps_across_multiple_periods(self, rig_dev):
        # t=5.0 with period=4.0 -> same phase as t=1.0 (quarter period).
        assert rig_dev.waveform("sine", 5.0, period=4.0, amp=150.0) == pytest.approx(150.0)


# ---------------------------------------------------------------------------
# secondary_to_dict()
# ---------------------------------------------------------------------------

class TestSecondaryToDict:
    def test_adapts_every_field(self, rig_dev):
        secondary = telemetry_pb2.TelemetrySecondary(
            now=12345,
            has_cmd_vel=True, cmd_vel_left=100.0, cmd_vel_right=-50.0,
            acc_left=1.5, acc_right=-2.5,
            glitch_left=3, glitch_right=4,
            ts_left=1000, ts_right=1001,
        )

        d = rig_dev.secondary_to_dict(secondary)

        assert d == {
            "t": 12345,
            "cmd_vel_left": 100.0,
            "cmd_vel_right": -50.0,
            "acc_left": 1.5,
            "acc_right": -2.5,
            "glitch_left": 3,
            "glitch_right": 4,
            "ts_left": 1000,
            "ts_right": 1001,
        }

    def test_cmd_vel_none_when_has_cmd_vel_false(self, rig_dev):
        secondary = telemetry_pb2.TelemetrySecondary(now=1, has_cmd_vel=False)

        d = rig_dev.secondary_to_dict(secondary)

        assert d["cmd_vel_left"] is None
        assert d["cmd_vel_right"] is None


# ---------------------------------------------------------------------------
# Rig — forwarding contract against fake conn/proto (no serial port)
# ---------------------------------------------------------------------------

class _FakeProto:
    def __init__(self) -> None:
        self.twist_calls: list[tuple] = []
        self.stop_calls = 0
        self.config_calls: list[dict] = []
        self.wait_for_ack_calls: list[tuple] = []
        self._next_corr_id = 0
        self.ack_to_return: AckEntry | None = None
        self.tlm_frames_to_return: list[TLMFrame] = []

    def _next_id(self) -> int:
        self._next_corr_id += 1
        return self._next_corr_id

    def twist(self, v_x: float, omega: float, duration: float) -> int:
        self.twist_calls.append((v_x, omega, duration))
        return self._next_id()

    def stop(self) -> int:
        self.stop_calls += 1
        return self._next_id()

    def config(self, **deltas) -> int:
        self.config_calls.append(deltas)
        return self._next_id()

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> AckEntry | None:
        self.wait_for_ack_calls.append((corr_id, timeout))
        return self.ack_to_return

    def read_pending_binary_tlm_frames(self) -> list[TLMFrame]:
        return self.tlm_frames_to_return


class _FakeConn:
    def __init__(self) -> None:
        self.disconnected = False
        self.secondary_to_return: list = []

    def drain_binary_secondary_tlm(self) -> list:
        return self.secondary_to_return

    def disconnect(self) -> None:
        self.disconnected = True


class TestRigForwarding:
    def test_twist_forwards_args_and_returns_corr_id(self, rig_dev):
        proto = _FakeProto()
        rig = rig_dev.Rig(_FakeConn(), proto)

        corr_id = rig.twist(v_x=150.0, omega=0.5, duration=300.0)

        assert corr_id == 1
        assert proto.twist_calls == [(150.0, 0.5, 300.0)]

    def test_stop_forwards_and_returns_corr_id(self, rig_dev):
        proto = _FakeProto()
        rig = rig_dev.Rig(_FakeConn(), proto)

        corr_id = rig.stop()

        assert corr_id == 1
        assert proto.stop_calls == 1

    def test_config_forwards_kwargs(self, rig_dev):
        proto = _FakeProto()
        rig = rig_dev.Rig(_FakeConn(), proto)

        rig.config(sTimeout=1000)

        assert proto.config_calls == [{"sTimeout": 1000}]

    def test_wait_for_ack_forwards_corr_id_and_timeout(self, rig_dev):
        proto = _FakeProto()
        proto.ack_to_return = AckEntry(corr_id=7, ok=True, err_code=0)
        rig = rig_dev.Rig(_FakeConn(), proto)

        ack = rig.wait_for_ack(7, timeout=250)

        assert ack is proto.ack_to_return
        assert proto.wait_for_ack_calls == [(7, 250)]

    def test_read_tlm_returns_proto_frames(self, rig_dev):
        proto = _FakeProto()
        frame = TLMFrame(t=1, enc=(10, 20))
        proto.tlm_frames_to_return = [frame]
        rig = rig_dev.Rig(_FakeConn(), proto)

        assert rig.read_tlm() == [frame]

    def test_read_secondary_tlm_adapts_every_frame(self, rig_dev):
        proto = _FakeProto()
        conn = _FakeConn()
        conn.secondary_to_return = [
            telemetry_pb2.TelemetrySecondary(now=1, glitch_left=1, glitch_right=2),
            telemetry_pb2.TelemetrySecondary(now=2, glitch_left=3, glitch_right=4),
        ]
        rig = rig_dev.Rig(conn, proto)

        rows = rig.read_secondary_tlm()

        assert [r["t"] for r in rows] == [1, 2]
        assert [r["glitch_left"] for r in rows] == [1, 3]

    def test_close_stops_and_disconnects_even_if_stop_raises(self, rig_dev):
        class _RaisingProto(_FakeProto):
            def stop(self) -> int:
                raise ConnectionError("port gone")

        proto = _RaisingProto()
        conn = _FakeConn()
        rig = rig_dev.Rig(conn, proto)

        rig.close()  # must not raise

        assert conn.disconnected is True
