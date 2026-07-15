"""tests/unit/test_serial_conn_telemetry_secondary.py — 104-003 (serial_conn
ack-ring matcher hardening + TelemetrySecondary consumption).

103-001 (``protos/telemetry.proto``, Decision 3) declared ``TelemetrySecondary``
-- the slower ~5 Hz diagnostic frame carrying ``acc``/``glitch``/``ts``/
``cmd_vel`` fields pruned OUT of the always-on primary ``Telemetry`` message
-- and resolved its wire framing: a SECOND, independently-armored ``*B``
line, NOT a ``ReplyEnvelope.body`` oneof arm (that oneof is fixed at
``ok``/``err``/``tlm``, envelope.proto). ``source/app/telemetry.cpp``'s
``emitSecondary()`` (confirmed against the merged tree) armors and sends a
BARE ``msg::TelemetrySecondary`` directly -- never wrapped in a
``ReplyEnvelope`` -- under the exact same ``*B<base64>`` prefix a
``ReplyEnvelope`` line uses. No host consumer decoded this frame before this
ticket, even though the wire framing had been decided since 103-001.

This file covers ``serial_conn.py``'s new decode path
(``_handle_binary_reply()``'s ``TelemetrySecondary`` fallback,
``_binary_secondary_queue``, ``drain_binary_secondary_tlm()``/
``read_binary_secondary_tlm()``), no live hardware, no real serial port
(mirrors ``test_serial_conn_binary_plane.py``'s own ``_FakeSerial``/
``_new_conn()`` pattern):

1. A synthetic ``TelemetrySecondary``, armored exactly as
   ``source/app/telemetry.cpp`` armors it, round-trips through
   ``_reader_loop()`` into ``_binary_secondary_queue`` with every field
   (``acc``, ``glitch``, ``ts``, ``cmd_vel``) intact -- this ticket's own
   required round-trip test.
2. Disambiguation: an ordinary ``ReplyEnvelope`` (``ok``/``err``/``tlm``)
   line is routed exactly as before (never misrouted to the secondary
   queue), and a ``TelemetrySecondary`` line is never misrouted to
   ``_reply_queues``/``_binary_tlm_queue`` -- proving the two message types
   sharing one armor prefix coexist correctly in the same reader-thread
   session.
3. ``drain_binary_secondary_tlm()`` / ``read_binary_secondary_tlm()`` --
   the TelemetrySecondary counterparts of ``drain_binary_tlm()``/
   ``read_binary_tlm()``, same non-blocking/blocking-poll and
   drop-oldest-on-overflow contracts.

Collected under ``tests/unit/`` — ``pyproject.toml``'s ``testpaths`` includes
``tests/unit``, so ``uv run python -m pytest`` collects it by default.
"""

from __future__ import annotations

import base64
import queue

import pytest

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.pb2 import envelope_pb2, telemetry_pb2


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _FakeSerial:
    """Minimal readline()-based stand-in for pyserial.Serial (mirrors
    test_serial_conn_binary_plane.py's own fixture) -- feeds a fixed
    sequence of lines to ``_reader_loop()`` when called SYNCHRONOUSLY, no
    threading, cannot hang."""

    is_open = True

    def __init__(self, lines: list[bytes]):
        self._lines = list(lines)

    def readline(self) -> bytes:
        if not self._lines:
            raise RuntimeError("fake serial exhausted (mimics a closed port)")
        return self._lines.pop(0)


def _new_conn() -> SerialConnection:
    return SerialConnection()


def _armor(message) -> str:
    """Armor a pb2 message exactly as source/app/telemetry.cpp's
    emitSecondary() (bare TelemetrySecondary) and Comms::sendReply()
    (ReplyEnvelope) both do: `*B` + base64(serialized bytes)."""
    return "*B" + base64.b64encode(message.SerializeToString()).decode("ascii")


def _synthetic_secondary(**kwargs) -> "telemetry_pb2.TelemetrySecondary":
    defaults = dict(
        now=12345,
        has_cmd_vel=True,
        cmd_vel_left=150.0,
        cmd_vel_right=-150.0,
        acc_left=12.5,
        acc_right=-8.25,
        glitch_left=3,
        glitch_right=0,
        ts_left=12300,
        ts_right=12290,
    )
    defaults.update(kwargs)
    return telemetry_pb2.TelemetrySecondary(**defaults)


# ---------------------------------------------------------------------------
# 1. Round-trip: every field (acc, glitch, ts, cmd_vel) decodes correctly
# ---------------------------------------------------------------------------


def test_telemetry_secondary_round_trips_every_field_through_reader_loop():
    conn = _new_conn()
    secondary = _synthetic_secondary()
    armored = _armor(secondary)

    conn._ser = _FakeSerial([(armored + "\n").encode("ascii")])
    conn._reader_loop()

    decoded = conn._binary_secondary_queue.get_nowait()
    assert isinstance(decoded, telemetry_pb2.TelemetrySecondary)
    assert decoded.now == 12345
    assert decoded.has_cmd_vel is True
    assert decoded.cmd_vel_left == pytest.approx(150.0)
    assert decoded.cmd_vel_right == pytest.approx(-150.0)
    assert decoded.acc_left == pytest.approx(12.5)
    assert decoded.acc_right == pytest.approx(-8.25)
    assert decoded.glitch_left == 3
    assert decoded.glitch_right == 0
    assert decoded.ts_left == 12300
    assert decoded.ts_right == 12290


def test_telemetry_secondary_has_cmd_vel_false_round_trips():
    conn = _new_conn()
    secondary = _synthetic_secondary(has_cmd_vel=False, cmd_vel_left=0.0,
                                      cmd_vel_right=0.0)
    armored = _armor(secondary)

    conn._ser = _FakeSerial([(armored + "\n").encode("ascii")])
    conn._reader_loop()

    decoded = conn._binary_secondary_queue.get_nowait()
    assert decoded.has_cmd_vel is False


# ---------------------------------------------------------------------------
# 2. Disambiguation: ReplyEnvelope and TelemetrySecondary share `*B` but
#    coexist correctly.
# ---------------------------------------------------------------------------


def test_reply_envelope_lines_still_route_normally_alongside_telemetry_secondary():
    """A ReplyEnvelope (ok body) and a TelemetrySecondary, fed in the SAME
    reader-thread session, each land in the correct queue -- the new
    TelemetrySecondary fallback must not disturb existing ReplyEnvelope
    routing (tlm push, corr-id'd ok/err), and vice versa."""
    conn = _new_conn()
    reply_q: queue.Queue = queue.Queue()
    conn._reply_queues["9"] = reply_q

    ack = envelope_pb2.ReplyEnvelope(corr_id=9)
    ack.ok.q = 2
    ack.ok.rem = 5.0
    ack_armored = _armor(ack)

    push = envelope_pb2.ReplyEnvelope(corr_id=0)
    push.tlm.now = 42
    push.tlm.seq = 8
    push_armored = _armor(push)

    secondary_armored = _armor(_synthetic_secondary(now=999))

    conn._ser = _FakeSerial([
        (ack_armored + "\n").encode("ascii"),
        (secondary_armored + "\n").encode("ascii"),
        (push_armored + "\n").encode("ascii"),
    ])
    conn._reader_loop()

    ack_reply = reply_q.get_nowait()
    assert ack_reply.corr_id == 9
    assert ack_reply.WhichOneof("body") == "ok"

    tlm_reply = conn._binary_tlm_queue.get_nowait()
    assert tlm_reply.corr_id == 0
    assert tlm_reply.WhichOneof("body") == "tlm"
    assert tlm_reply.tlm.now == 42

    secondary = conn._binary_secondary_queue.get_nowait()
    assert secondary.now == 999

    # Neither line leaked into the other's queue.
    assert conn._binary_tlm_queue.empty()
    assert conn._binary_secondary_queue.empty()
    assert reply_q.empty()


def test_telemetry_secondary_never_registers_a_reply_queue_entry():
    """TelemetrySecondary carries no corr_id at all -- confirms the fallback
    path never touches _reply_queues (there is nothing to key a lookup by)."""
    conn = _new_conn()
    armored = _armor(_synthetic_secondary())

    conn._ser = _FakeSerial([(armored + "\n").encode("ascii")])
    conn._reader_loop()

    assert conn._reply_queues == {}
    assert conn._binary_tlm_queue.empty()


def test_malformed_binary_line_still_dropped_not_misrouted_to_secondary_queue():
    """A corrupted `*B` line (neither a valid ReplyEnvelope NOR a valid
    TelemetrySecondary) must still be dropped silently, not crash the
    reader thread and not land in either queue."""
    conn = _new_conn()

    conn._ser = _FakeSerial([b"*Bnot-valid-base64!!!\n"])
    conn._reader_loop()  # must not raise

    assert conn._binary_tlm_queue.empty()
    assert conn._binary_secondary_queue.empty()
    assert conn._reply_queues == {}


# ---------------------------------------------------------------------------
# 3. drain_binary_secondary_tlm() / read_binary_secondary_tlm()
# ---------------------------------------------------------------------------


class _StaticOpenSerial:
    """A fake `_ser` that only needs to answer `is_open` truthfully --
    these accessors never touch `_ser.readline()`/`write()` (they poll
    `_binary_secondary_queue`, which the reader thread fills independently
    via _handle_binary_reply())."""

    is_open = True


def test_drain_binary_secondary_tlm_returns_all_queued_frames_and_empties_queue():
    conn = _new_conn()
    for now in (1, 2, 3):
        conn._binary_secondary_queue.put_nowait(_synthetic_secondary(now=now))

    frames = conn.drain_binary_secondary_tlm()

    assert [f.now for f in frames] == [1, 2, 3]
    assert conn._binary_secondary_queue.empty()


def test_drain_binary_secondary_tlm_on_empty_queue_returns_empty_list():
    conn = _new_conn()
    assert conn.drain_binary_secondary_tlm() == []


def test_read_binary_secondary_tlm_returns_frames_already_queued():
    conn = _new_conn()
    conn._ser = _StaticOpenSerial()
    for now in (10, 20):
        conn._binary_secondary_queue.put_nowait(_synthetic_secondary(now=now))

    frames = conn.read_binary_secondary_tlm(duration=30)

    assert [f.now for f in frames] == [10, 20]
    assert conn._binary_secondary_queue.empty()


def test_read_binary_secondary_tlm_not_connected_returns_empty_list_immediately():
    conn = _new_conn()  # _ser stays None -- never connected
    assert conn.read_binary_secondary_tlm(duration=500) == []


def test_read_binary_secondary_tlm_times_out_with_empty_list_when_nothing_arrives():
    conn = _new_conn()
    conn._ser = _StaticOpenSerial()
    assert conn.read_binary_secondary_tlm(duration=30) == []


def test_binary_secondary_queue_drops_oldest_on_overflow():
    """Matches _binary_tlm_queue's own documented drop-oldest-on-overflow
    policy (test_binary_tlm_queue_drops_oldest_on_overflow in
    test_serial_conn_binary_plane.py) -- same contract, TelemetrySecondary
    counterpart. Uses a small monkey-patched queue depth (3) instead of the
    real _TLM_QUEUE_DEPTH (256) so the test stays fast."""
    conn = _new_conn()
    conn._binary_secondary_queue = queue.Queue(maxsize=3)

    for now in range(5):
        conn._handle_binary_reply(_armor(_synthetic_secondary(now=now)))

    remaining = []
    while not conn._binary_secondary_queue.empty():
        remaining.append(conn._binary_secondary_queue.get_nowait())

    assert [f.now for f in remaining] == [2, 3, 4]


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
