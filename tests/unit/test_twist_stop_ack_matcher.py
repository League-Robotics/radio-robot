"""tests/unit/test_twist_stop_ack_matcher.py — 103-009 (P3 minimal host
slice: NezhaProtocol.twist()/stop() + the ack-ring matcher).

Covers this ticket's additions to `host/robot_radio/robot/protocol.py`,
none of which need live hardware or even a real `SerialConnection`:

1. `NezhaProtocol.twist()`/`stop()` — schema-level envelope-building tests
   against a minimal fake connection (a bare `send_envelope_fast()` stub,
   no reader thread/reply-queue machinery): assert the built
   `CommandEnvelope` carries the correct oneof arm and field values, and
   that each call returns the corr_id the fake assigned.

2. `TLMFrame.from_pb2()`'s new `acks`/`fault_bits`/`event_bits` fields —
   built from synthetic `telemetry_pb2.Telemetry` messages with scripted
   `acks` lists, confirming the P4 schema (confirmed directly against the
   regenerated `telemetry_pb2`/`envelope_pb2`, ticket 001) round-trips onto
   `AckEntry`/`TLMFrame` correctly. Also a regression test for the
   `has_cmd_vel` crash this ticket fixed (telemetry.proto moved `cmd_vel`
   to `TelemetrySecondary`; the old `from_pb2()` referenced the now-gone
   `has_cmd_vel`/`cmd_vel_left`/`cmd_vel_right` fields and raised
   `AttributeError` on every real `Telemetry` frame).

3. `NezhaProtocol.wait_for_ack()` — 104-003 promoted the actual poll/match/
   timeout algorithm out of this method into
   `SerialConnection.wait_for_ack()` (see
   `tests/unit/test_serial_conn_ack_ring.py` for that algorithm's own
   dedicated coverage: exact match, ring re-delivery tolerance, ring-wrap,
   bounded timeout — all against synthetic frames, no `NezhaProtocol`
   involved). What remains here is `NezhaProtocol.wait_for_ack()`'s own thin
   adapter role: delegate to `self._conn.wait_for_ack(corr_id, timeout)` and
   wrap the raw `telemetry_pb2.AckEntry` result in this module's own
   `AckEntry` dataclass (or pass `None` through unchanged on a timeout) —
   exercised against a fake connection that implements only `wait_for_ack()`.

Collected under `tests/unit/` — `pyproject.toml`'s `testpaths` includes
`tests/unit`, so `uv run python -m pytest` collects it by default.
"""

from __future__ import annotations

import pytest

from robot_radio.robot.pb2 import envelope_pb2, telemetry_pb2
from robot_radio.robot.protocol import AckEntry, NezhaProtocol, TLMFrame

# ---------------------------------------------------------------------------
# 1. twist() / stop() — schema-level envelope construction
# ---------------------------------------------------------------------------


class _FakeFastConn:
    """Minimal fake connection: implements ONLY `send_envelope_fast()` --
    twist()/stop() call nothing else on `self._conn`. Assigns corr_ids the
    same way `SerialConnection._corr_counter` does (1, 2, 3, ...) and
    records every envelope handed to it, with no serial port, no reader
    thread, no reply-queue machinery at all."""

    def __init__(self) -> None:
        self.sent: list["envelope_pb2.CommandEnvelope"] = []
        self._next_corr_id = 0

    def send_envelope_fast(self, envelope: "envelope_pb2.CommandEnvelope") -> int:
        self._next_corr_id += 1
        envelope.corr_id = self._next_corr_id
        self.sent.append(envelope)
        return self._next_corr_id


def test_twist_builds_correct_envelope_and_returns_corr_id():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    corr_id = proto.twist(v_x=150.0, omega=0.25, duration=300.0)

    assert corr_id == 1
    sent = conn.sent[0]
    assert sent.corr_id == 1
    assert sent.WhichOneof("cmd") == "twist"
    assert sent.twist.v_x == pytest.approx(150.0)
    assert sent.twist.omega == pytest.approx(0.25)
    assert sent.twist.duration == pytest.approx(300.0)


def test_stop_builds_correct_envelope_and_returns_corr_id():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    corr_id = proto.stop()

    assert corr_id == 1
    sent = conn.sent[0]
    assert sent.corr_id == 1
    assert sent.WhichOneof("cmd") == "stop"
    # Stop{} is a zero-field arm -- there is nothing else to assert about
    # its payload beyond "the oneof arm is stop".


def test_twist_and_stop_each_get_a_fresh_corr_id():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    c1 = proto.twist(v_x=100.0, omega=0.0, duration=100.0)
    c2 = proto.stop()
    c3 = proto.twist(v_x=-100.0, omega=0.1, duration=100.0)

    assert [c1, c2, c3] == [1, 2, 3]
    assert len(conn.sent) == 3


# ---------------------------------------------------------------------------
# 2. TLMFrame.from_pb2() acks/fault_bits/event_bits
# ---------------------------------------------------------------------------


def _telemetry_with_acks(acks: list[tuple[int, bool, int]], **kwargs) -> "telemetry_pb2.Telemetry":
    """Build a synthetic `telemetry_pb2.Telemetry` with a scripted acks
    list -- (corr_id, ok, err_code) triples."""
    pb_acks = [
        telemetry_pb2.AckEntry(
            corr_id=corr_id,
            status=telemetry_pb2.ACK_STATUS_OK if ok else telemetry_pb2.ACK_STATUS_ERR,
            err_code=err_code,
        )
        for corr_id, ok, err_code in acks
    ]
    return telemetry_pb2.Telemetry(acks=pb_acks, **kwargs)


def test_from_pb2_exposes_acks_fault_bits_event_bits():
    telemetry = _telemetry_with_acks(
        [(7, True, 0)], now=100, fault_bits=0b101, event_bits=0b010)

    frame = TLMFrame.from_pb2(telemetry)

    assert frame.acks == (AckEntry(corr_id=7, ok=True, err_code=0),)
    assert frame.fault_bits == 0b101
    assert frame.event_bits == 0b010


def test_from_pb2_acks_defaults_to_empty_tuple_not_none():
    """acks/fault_bits/event_bits are unconditional fields (no has_* flag,
    same treatment as `active`) -- always populated, never left at the
    dataclass's own None default, even when the ring is empty."""
    telemetry = telemetry_pb2.Telemetry(now=1)

    frame = TLMFrame.from_pb2(telemetry)

    assert frame.acks == ()
    assert frame.fault_bits == 0
    assert frame.event_bits == 0


def test_from_pb2_err_ack_carries_err_code():
    telemetry = _telemetry_with_acks([(9, False, envelope_pb2.ERR_RANGE)])

    frame = TLMFrame.from_pb2(telemetry)

    ack = frame.acks[0]
    assert ack.corr_id == 9
    assert ack.ok is False
    assert ack.err_code == envelope_pb2.ERR_RANGE


def test_from_pb2_does_not_crash_on_a_full_primary_frame_and_cmd_vel_stays_none():
    """Regression test: the pre-fix from_pb2() read has_cmd_vel/cmd_vel_left/
    cmd_vel_right, fields telemetry.proto no longer declares on the primary
    Telemetry message (103-001 moved them to TelemetrySecondary) --
    AttributeError on every real frame. cmd_vel is a permanent gap for a
    frame built from the primary Telemetry stream (see from_pb2()'s own
    docstring); the rest of the frame must still decode cleanly."""
    from robot_radio.robot.pb2 import common_pb2, planner_pb2

    telemetry = telemetry_pb2.Telemetry(
        now=5000, mode=planner_pb2.IDLE, seq=1,
        has_enc=True, enc_left=10.0, enc_right=11.0,
        has_vel=True, vel_left=1.0, vel_right=1.0,
        has_pose=True, pose=common_pb2.Pose2D(x=0.0, y=0.0, h=0.0),
        acks=[telemetry_pb2.AckEntry(corr_id=1, status=telemetry_pb2.ACK_STATUS_OK)],
        fault_bits=0, event_bits=0,
    )

    frame = TLMFrame.from_pb2(telemetry)

    assert frame.cmd_vel is None
    assert frame.enc == (10, 11)
    assert frame.acks == (AckEntry(corr_id=1, ok=True, err_code=0),)


# ---------------------------------------------------------------------------
# 3. wait_for_ack() -- 104-003: thin adapter over SerialConnection.wait_for_ack()
# ---------------------------------------------------------------------------


class _FakeConnWithAck:
    """Minimal fake connection: implements ONLY `wait_for_ack()` --
    `NezhaProtocol.wait_for_ack()` (104-003) delegates the ENTIRE poll/
    match/timeout algorithm to `SerialConnection.wait_for_ack()`; this fake
    lets the delegation itself be tested (call forwarded with the right
    args, raw pb2 AckEntry adapted to this module's AckEntry dataclass,
    `None` passed through unchanged) without a real queue/thread. The
    algorithm's own scenario coverage (exact match, ring re-delivery
    tolerance, ring-wrap, bounded timeout) lives in
    `tests/unit/test_serial_conn_ack_ring.py`, against the real
    `SerialConnection.wait_for_ack()`."""

    def __init__(self, result: "telemetry_pb2.AckEntry | None") -> None:
        self.result = result
        self.calls: list[tuple[int, int]] = []

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "telemetry_pb2.AckEntry | None":
        self.calls.append((corr_id, timeout))
        return self.result


def test_wait_for_ack_delegates_to_shared_matcher_and_adapts_ok_result():
    raw_ack = telemetry_pb2.AckEntry(
        corr_id=5, status=telemetry_pb2.ACK_STATUS_OK, err_code=0)
    conn = _FakeConnWithAck(raw_ack)
    proto = NezhaProtocol(conn)

    ack = proto.wait_for_ack(5, timeout=250)

    assert ack == AckEntry(corr_id=5, ok=True, err_code=0)
    assert conn.calls == [(5, 250)]


def test_wait_for_ack_delegates_to_shared_matcher_and_adapts_err_result():
    raw_ack = telemetry_pb2.AckEntry(
        corr_id=9, status=telemetry_pb2.ACK_STATUS_ERR, err_code=envelope_pb2.ERR_BADARG)
    conn = _FakeConnWithAck(raw_ack)
    proto = NezhaProtocol(conn)

    ack = proto.wait_for_ack(9)

    assert ack == AckEntry(corr_id=9, ok=False, err_code=envelope_pb2.ERR_BADARG)
    assert conn.calls == [(9, 500)]  # default timeout forwarded unchanged


def test_wait_for_ack_passes_none_through_on_shared_matcher_timeout():
    conn = _FakeConnWithAck(None)
    proto = NezhaProtocol(conn)

    ack = proto.wait_for_ack(5, timeout=50)

    assert ack is None
    assert conn.calls == [(5, 50)]
