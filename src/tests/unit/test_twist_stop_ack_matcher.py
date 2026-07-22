"""src/tests/unit/test_twist_stop_ack_matcher.py — 103-009 (P3 minimal host
slice: NezhaProtocol.twist()/stop() + the ack matcher), sections 2/3
rewritten for the single-ack-slot design (115-003 frame v2).

Covers this ticket's additions to `src/host/robot_radio/robot/protocol.py`,
none of which need live hardware or even a real `SerialConnection`:

1. `NezhaProtocol.twist()`/`stop()` — schema-level envelope-building tests
   against a minimal fake connection (a bare `send_envelope_fast()` stub,
   no reader thread/reply-queue machinery): assert the built
   `CommandEnvelope` carries the correct oneof arm and field values, and
   that each call returns the corr_id the fake assigned.

2. `TLMFrame.from_pb2()`'s `flags`-derived fields — built from synthetic
   `telemetry_pb2.Telemetry` messages with scripted `flags`, confirming the
   frame-v2 schema (115-003) round-trips onto `TLMFrame` correctly. Also a
   regression test for the `has_cmd_vel` crash a much earlier ticket fixed
   (telemetry.proto moved `cmd_vel` to `TelemetrySecondary`; an even older
   `from_pb2()` referenced the now-gone `has_cmd_vel`/`cmd_vel_left`/
   `cmd_vel_right` fields and raised `AttributeError` on every real
   `Telemetry` frame) — kept green across the frame-v2 rewrite too.

3. `NezhaProtocol.wait_for_ack()` — 104-003 promoted the actual poll/match/
   timeout algorithm out of this method into
   `SerialConnection.wait_for_ack()` (see
   `src/tests/unit/test_serial_conn_ack_ring.py` for that algorithm's own
   dedicated coverage: exact match, slot-overwrite, bounded timeout — all
   against synthetic frames, no `NezhaProtocol` involved). What remains
   here is `NezhaProtocol.wait_for_ack()`'s own thin adapter role: delegate
   to `self._conn.wait_for_ack(corr_id, timeout)` and wrap the matched raw
   `telemetry_pb2.Telemetry` frame's ack slot in this module's own
   `AckEntry` dataclass (or pass `None` through unchanged on a timeout) —
   exercised against a fake connection that implements only `wait_for_ack()`.

Collected under `src/tests/unit/` — `pyproject.toml`'s `testpaths` includes
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
# 2. TLMFrame.from_pb2() flags-derived fields (115-003 frame v2)
# ---------------------------------------------------------------------------

# flags bits used directly by this test file (mirrors protocol.py's own
# module-private _FLAG_* constants -- duplicated here since they are
# private; see telemetry.proto's own bit-table comment for the
# authoritative numbering).
_FLAG_ACK_FRESH = 1 << 5
_FLAG_FAULT_WEDGE_LATCH = 1 << 7
_FLAG_EVENT_BOOT_READY = 1 << 11


def test_from_pb2_exposes_ack_and_fault_and_event_flags():
    telemetry = telemetry_pb2.Telemetry(
        now=100, flags=_FLAG_ACK_FRESH | _FLAG_FAULT_WEDGE_LATCH | _FLAG_EVENT_BOOT_READY,
        ack_corr=7, ack_err=0,
    )

    frame = TLMFrame.from_pb2(telemetry)

    assert frame.ack == AckEntry(corr_id=7, ok=True, err_code=0)
    assert frame.ack_fresh is True
    assert frame.fault_wedge_latch is True
    assert frame.event_boot_ready is True
    # Bits not set stay False -- flags-derived properties never default-True.
    assert frame.fault_i2c_nak_timeout is False
    assert frame.event_deadman_expired is False


def test_from_pb2_ack_is_none_when_not_fresh():
    """ack_corr/ack_err hold their last-written value on every ordinary
    telemetry push (they are plain scalar fields, not gated by a
    presence flag of their own) -- only `ack_fresh` (flags bit 5) says
    whether THIS frame carries a genuinely new ack. When it is clear,
    `TLMFrame.ack` stays None even though ack_corr/ack_err are non-zero on
    the wire, so a caller never mistakes a stale value for a fresh one."""
    telemetry = telemetry_pb2.Telemetry(now=1, ack_corr=7, ack_err=0)  # flags defaults to 0 (not fresh)

    frame = TLMFrame.from_pb2(telemetry)

    assert frame.ack is None
    assert frame.ack_fresh is False
    # Raw ack_corr/ack_err are still populated unconditionally (mirrors the
    # "always present, just check freshness yourself" contract).
    assert frame.ack_corr == 7
    assert frame.ack_err == 0


def test_from_pb2_flags_defaults_to_zero_not_none():
    """`flags` is an unconditional field (no has_* flag, same "always
    present" treatment `active` already had pre-115) -- always populated,
    never left at the dataclass's own None default."""
    telemetry = telemetry_pb2.Telemetry(now=1)

    frame = TLMFrame.from_pb2(telemetry)

    assert frame.flags == 0
    assert frame.ack_fresh is False
    assert frame.fault_wedge_latch is False


def test_from_pb2_err_ack_carries_err_code():
    telemetry = telemetry_pb2.Telemetry(
        flags=_FLAG_ACK_FRESH, ack_corr=9, ack_err=envelope_pb2.ERR_RANGE)

    frame = TLMFrame.from_pb2(telemetry)

    ack = frame.ack
    assert ack.corr_id == 9
    assert ack.ok is False
    assert ack.err_code == envelope_pb2.ERR_RANGE


def test_from_pb2_does_not_crash_on_a_full_primary_frame_and_cmd_vel_stays_none():
    """Regression test: an even earlier from_pb2() read has_cmd_vel/
    cmd_vel_left/cmd_vel_right, fields telemetry.proto no longer declares on
    the primary Telemetry message (103-001 moved them to
    TelemetrySecondary) -- AttributeError on every real frame. cmd_vel is a
    permanent gap for a frame built from the primary Telemetry stream (see
    from_pb2()'s own docstring); the rest of the frame must still decode
    cleanly, including the frame-v2 (115-003) reading objects and single ack
    slot."""
    from robot_radio.robot.pb2 import common_pb2

    telemetry = telemetry_pb2.Telemetry(
        now=5000, mode=telemetry_pb2.IDLE, seq=1,
        flags=_FLAG_ACK_FRESH,
        enc_left=telemetry_pb2.EncoderReading(position=10.0, velocity=1.0, time=5000),
        enc_right=telemetry_pb2.EncoderReading(position=11.0, velocity=1.0, time=5000),
        pose=common_pb2.Pose2D(x=0.0, y=0.0, h=0.0),
        ack_corr=1, ack_err=0,
    )

    frame = TLMFrame.from_pb2(telemetry)

    assert frame.cmd_vel is None
    assert frame.enc == (10, 11)
    assert frame.ack == AckEntry(corr_id=1, ok=True, err_code=0)


# ---------------------------------------------------------------------------
# 3. wait_for_ack() -- 104-003: thin adapter over SerialConnection.wait_for_ack()
# ---------------------------------------------------------------------------


class _FakeConnWithAck:
    """Minimal fake connection: implements ONLY `wait_for_ack()` --
    `NezhaProtocol.wait_for_ack()` (104-003) delegates the ENTIRE poll/
    match/timeout algorithm to `SerialConnection.wait_for_ack()`; this fake
    lets the delegation itself be tested (call forwarded with the right
    args, the matched raw pb2 Telemetry frame's ack slot adapted to this
    module's AckEntry dataclass, `None` passed through unchanged) without a
    real queue/thread. The algorithm's own scenario coverage (exact match,
    slot-overwrite, bounded timeout) lives in
    `src/tests/unit/test_serial_conn_ack_ring.py`, against the real
    `SerialConnection.wait_for_ack()`."""

    def __init__(self, result: "telemetry_pb2.Telemetry | None") -> None:
        self.result = result
        self.calls: list[tuple[int, int]] = []

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "telemetry_pb2.Telemetry | None":
        self.calls.append((corr_id, timeout))
        return self.result


def test_wait_for_ack_delegates_to_shared_matcher_and_adapts_ok_result():
    raw_telemetry = telemetry_pb2.Telemetry(flags=_FLAG_ACK_FRESH, ack_corr=5, ack_err=0)
    conn = _FakeConnWithAck(raw_telemetry)
    proto = NezhaProtocol(conn)

    ack = proto.wait_for_ack(5, timeout=250)

    assert ack == AckEntry(corr_id=5, ok=True, err_code=0)
    assert conn.calls == [(5, 250)]


def test_wait_for_ack_delegates_to_shared_matcher_and_adapts_err_result():
    raw_telemetry = telemetry_pb2.Telemetry(
        flags=_FLAG_ACK_FRESH, ack_corr=9, ack_err=envelope_pb2.ERR_BADARG)
    conn = _FakeConnWithAck(raw_telemetry)
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
