"""src/tests/unit/test_serial_conn_telemetry_secondary.py — 124-009
(robot-state-blackboard-...md, issue's own "TelemetrySecondary dies")
regression coverage.

``TelemetrySecondary`` -- the slower ~5 Hz diagnostic frame this file used
to cover end to end (103-001 Decision 3's independently-armored ``*B``
line; 104-003's ``serial_conn.py`` decode path: the
``_handle_binary_reply()`` fallback, ``_binary_secondary_queue``,
``drain_binary_secondary_tlm()``/``read_binary_secondary_tlm()``) -- is
DELETED OUTRIGHT, not deprecated: the message type, the wire schema arm,
and every host-side consumer of it are gone. It emitted nothing but `now`
in production (no firmware caller ever populated the rest of it).

This file's own job now is the regression proof `state ⊇ wire` this
ticket calls for: TelemetrySecondary genuinely cannot come back
unnoticed, on either side of the wire.

Collected under ``src/tests/unit/`` — ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by
default.
"""

from __future__ import annotations

import pytest

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.pb2 import envelope_pb2, telemetry_pb2

from _wire_test_helpers import FakeSerial, binary_frame


def _new_conn() -> SerialConnection:
    return SerialConnection()


def test_telemetry_secondary_is_not_in_the_regenerated_pb2_module():
    """The structural proof: `telemetry_pb2` -- regenerated from
    `protos/telemetry.proto` by the SAME `gen_pb2.py`/`grpc_tools.protoc`
    pipeline `build.py` always runs -- has no `TelemetrySecondary`
    attribute at all. A future accidental re-introduction of the message
    in the proto would make this assertion fail immediately, long before
    any wire-level behavior could silently drift."""
    assert not hasattr(telemetry_pb2, "TelemetrySecondary")


def test_serial_connection_has_no_secondary_queue_or_accessors():
    """The host-side decode path (`_binary_secondary_queue`,
    `drain_binary_secondary_tlm()`, `read_binary_secondary_tlm()`) is
    deleted along with the message type it existed to carry -- not left
    behind as dead code that would raise `AttributeError` if ever
    (re-)called."""
    conn = _new_conn()
    assert not hasattr(conn, "_binary_secondary_queue")
    assert not hasattr(conn, "drain_binary_secondary_tlm")
    assert not hasattr(conn, "read_binary_secondary_tlm")


def test_bare_corr_id_only_frame_counts_as_malformed_not_misrouted():
    """124-009's own behavior change: a frame that decodes as a
    `ReplyEnvelope` with `corr_id` set but no `body` oneof populated (the
    exact shape a stray/corrupted frame -- or, historically, a bare
    TelemetrySecondary frame's own field-number collision -- produces)
    used to retry as `TelemetrySecondary`; now there is no second shape to
    retry against, so it is simply malformed. Mirrors
    `test_serial_conn_binary_plane.py`'s own malformed-frame coverage,
    scoped to this specific "successfully parses, empty oneof" case."""
    conn = _new_conn()
    bare = envelope_pb2.ReplyEnvelope(corr_id=123456)
    # Deliberately NOT setting ok/err/tlm -- WhichOneof("body") stays None.

    conn._ser = FakeSerial(binary_frame(bare, b"TLM"))
    conn._reader_loop()  # must not raise

    assert conn._binary_tlm_queue.empty()
    assert conn._reply_queues == {}
    assert conn.malformed_frame_count >= 1


def test_reply_envelope_routing_is_unaffected_by_the_deletion():
    """Sanity check: ordinary `ReplyEnvelope` routing (corr-id'd `ok`, and
    an unsolicited `tlm` push) is completely unaffected by
    TelemetrySecondary's removal -- there was never a SECOND queue for
    these to leak into."""
    conn = _new_conn()
    import queue

    reply_q: queue.Queue = queue.Queue()
    conn._reply_queues["9"] = reply_q

    ack = envelope_pb2.ReplyEnvelope(corr_id=9)
    ack.ok.q = 2
    ack.ok.rem = 5.0

    push = envelope_pb2.ReplyEnvelope(corr_id=0)
    push.tlm.now = 42
    push.tlm.seq = 8

    conn._ser = FakeSerial(binary_frame(ack, b"OK") + binary_frame(push, b"TLM"))
    conn._reader_loop()

    ack_reply = reply_q.get_nowait()
    assert ack_reply.corr_id == 9
    assert ack_reply.WhichOneof("body") == "ok"

    tlm_reply = conn._binary_tlm_queue.get_nowait()
    assert tlm_reply.corr_id == 0
    assert tlm_reply.WhichOneof("body") == "tlm"
    assert tlm_reply.tlm.now == 42


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
