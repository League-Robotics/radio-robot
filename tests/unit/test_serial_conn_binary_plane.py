"""tests/unit/test_serial_conn_binary_plane.py — 095-002 (M7 Host Codec Mirror).

Extended by 096-007 (M6 Host Config/Telemetry Client) with two more
``_reader_loop`` demux tests (``tlm``/``cfg`` body arms) -- see that
ticket's own file-header note below, just above the two new tests.

Covers the three things ticket 095-002 asks for, none of which need live
hardware:

1. ``host/robot_radio/robot/pb2/`` is importable, including a cross-file
   reference (proves the flat-import sys.path shim in
   ``host/robot_radio/robot/pb2/__init__.py`` actually works, not just that
   ``envelope_pb2`` itself parses).
2. ``SerialConnection._reader_loop``'s new ``*B<base64>`` branch correctly
   classifies and demuxes a binary reply by corr-id, WITHOUT disturbing the
   existing TLM/EVT/OK/ERR/CFG/ID/keepalive/`#`-comment branches (fed in the
   same pass, interleaved with the new branch, to prove they still coexist).
3. ``SerialConnection.send_envelope()`` round-trips a full write -> reader-
   thread -> corr-id-queue -> blocking-read cycle against a synthetic
   loopback transport (no real serial port).

Collected under ``tests/unit/`` (host-side unit/tooling check, not
sim/bench/playfield-scoped — see ``tests/CLAUDE.md``); ``pyproject.toml``'s
``testpaths`` includes ``tests/unit`` so ``uv run python -m pytest`` collects
it.
"""

import base64
import queue

import pytest

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.pb2 import config_pb2, envelope_pb2


# ---------------------------------------------------------------------------
# 1. pb2 import smoke test
# ---------------------------------------------------------------------------


def test_envelope_pb2_importable_and_roundtrips():
    """`from robot_radio.robot.pb2 import envelope_pb2` works at runtime, and
    a message defined directly in envelope.proto serializes/parses."""
    env = envelope_pb2.CommandEnvelope(corr_id=5)
    env.ping.SetInParent()

    data = env.SerializeToString()
    env2 = envelope_pb2.CommandEnvelope.FromString(data)

    assert env2.corr_id == 5
    assert env2.WhichOneof("cmd") == "ping"


def test_envelope_pb2_cross_file_import_resolves():
    """envelope_pb2.py contains bare top-level cross-file imports (protoc's
    flat -I protos output, e.g. `import drivetrain_pb2 as drivetrain__pb2`)
    that only resolve because host/robot_radio/robot/pb2/__init__.py inserts
    its own directory onto sys.path before any *_pb2 submodule loads.
    Touching a field whose TYPE lives in a different .proto file (drive ->
    DrivetrainCommand, defined in drivetrain.proto, referenced via
    MotionSegment's sibling `wheels`/`w` chain into common.proto's
    WheelTarget) exercises the actual cross-module reference end to end,
    not just envelope_pb2's own locally-defined messages."""
    env = envelope_pb2.CommandEnvelope()
    env.drive.wheels.w.add(speed=123.0)

    assert env.WhichOneof("cmd") == "drive"
    assert env.drive.WhichOneof("control") == "wheels"
    assert env.drive.wheels.w[0].speed == pytest.approx(123.0)


# ---------------------------------------------------------------------------
# 2. _reader_loop classify/demux test (synthetic lines, no hardware)
# ---------------------------------------------------------------------------


class _FakeSerial:
    """Minimal readline()-based stand-in for pyserial.Serial.

    Feeds a fixed sequence of lines to ``_reader_loop()`` when called
    SYNCHRONOUSLY (not via ``_start_reader()``'s background thread) --  the
    loop exits on its own once the fake line source is exhausted (raising,
    same as ``_reader_loop``'s own "port closed or gone" except-break path),
    so the test needs no threading and cannot hang.
    """

    is_open = True

    def __init__(self, lines: list[bytes]):
        self._lines = list(lines)

    def readline(self) -> bytes:
        if not self._lines:
            raise RuntimeError("fake serial exhausted (mimics a closed port)")
        return self._lines.pop(0)


def _new_conn() -> SerialConnection:
    """A SerialConnection with no real I/O performed (the constructor never
    touches a port -- ``_ser`` stays None until a test assigns a fake)."""
    return SerialConnection()


def test_reader_loop_routes_binary_reply_by_corr_id():
    """A synthetic `*B<base64>` line is dearmored, parsed as a
    ReplyEnvelope, and delivered to `_reply_queues[str(envelope.corr_id)]`
    -- exactly as an `OK ... #<id>` text reply is delivered today."""
    conn = _new_conn()
    reply_q: queue.Queue = queue.Queue()
    conn._reply_queues["42"] = reply_q

    envelope = envelope_pb2.ReplyEnvelope(corr_id=42)
    envelope.ok.q = 3
    envelope.ok.rem = 12.5
    armored = "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")

    conn._ser = _FakeSerial([(armored + "\n").encode("ascii")])
    conn._reader_loop()

    reply = reply_q.get_nowait()
    assert isinstance(reply, envelope_pb2.ReplyEnvelope)
    assert reply.corr_id == 42
    assert reply.WhichOneof("body") == "ok"
    assert reply.ok.q == 3
    assert reply.ok.rem == pytest.approx(12.5)


def test_reader_loop_binary_branch_coexists_with_every_existing_branch():
    """One pass through _reader_loop with TLM/EVT/OK/ERR/keepalive/`#`-comment
    lines interleaved with a `*B` line: every existing branch's routing is
    unaffected by the new branch's presence (behavioral proof to go with the
    source-diff proof that no existing branch's CODE changed)."""
    conn = _new_conn()
    conn._reply_queues["5"] = queue.Queue()
    conn._reply_queues["6"] = queue.Queue()
    conn._reply_queues["42"] = queue.Queue()

    envelope = envelope_pb2.ReplyEnvelope(corr_id=42)
    envelope.err.code = envelope_pb2.ERR_RANGE
    armored = "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")

    conn._ser = _FakeSerial([
        b"# relay comment line\n",
        b"TLM t=100 enc=0,0\n",
        b"OK keepalive\n",
        b"EVT done S\n",
        b"OK #5\n",
        (armored + "\n").encode("ascii"),
        b"ERR badarg #6\n",
    ])
    conn._reader_loop()

    # Text-plane branches: unchanged behavior.
    assert conn._tlm_queue.get_nowait() == "TLM t=100 enc=0,0"
    assert conn._evt_queue.get_nowait() == "EVT done S"
    assert conn._reply_queues["5"].get_nowait() == "OK #5"
    assert conn._reply_queues["6"].get_nowait() == "ERR badarg #6"
    assert conn._tlm_queue.empty()
    assert conn._evt_queue.empty()

    # New binary branch: routed by the envelope's own corr_id.
    reply = conn._reply_queues["42"].get_nowait()
    assert isinstance(reply, envelope_pb2.ReplyEnvelope)
    assert reply.corr_id == 42
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == envelope_pb2.ERR_RANGE


def test_reader_loop_routes_binary_tlm_reply_by_corr_id():
    """096-007 (M6 Host Config/Telemetry Client, ticket acceptance criterion
    "serial_conn.py's ReplyEnvelope demux correctly routes tlm/cfg body arms
    through the existing _reply_queues/_tlm_queue machinery with zero code
    changes to that file"): a synthetic `*B<base64>` ReplyEnvelope{tlm=
    Telemetry{...}} line -- 096's NEW body oneof arm, not exercised by any
    095-002 test above -- demuxes through the SAME corr-id-keyed
    _reply_queues machinery the ok/err arms already prove out above. No new
    branch was added to _reader_loop()/_handle_binary_reply() to make this
    pass (see this file's own diff for 096-007 -- zero lines changed in
    serial_conn.py itself)."""
    conn = _new_conn()
    reply_q: queue.Queue = queue.Queue()
    conn._reply_queues["7"] = reply_q

    envelope = envelope_pb2.ReplyEnvelope(corr_id=7)
    envelope.tlm.now = 12345
    envelope.tlm.seq = 3
    armored = "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")

    conn._ser = _FakeSerial([(armored + "\n").encode("ascii")])
    conn._reader_loop()

    reply = reply_q.get_nowait()
    assert isinstance(reply, envelope_pb2.ReplyEnvelope)
    assert reply.corr_id == 7
    assert reply.WhichOneof("body") == "tlm"
    assert reply.tlm.now == 12345
    assert reply.tlm.seq == 3


def test_reader_loop_routes_binary_cfg_reply_by_corr_id():
    """096-007: a synthetic `*B<base64>` ReplyEnvelope{cfg=ConfigSnapshot{...}}
    line -- 096's other NEW body oneof arm -- demuxes the same way, proving
    095's generic corr-id routing covers 096's additions with zero
    serial_conn.py changes (this ticket's own acceptance criterion)."""
    conn = _new_conn()
    reply_q: queue.Queue = queue.Queue()
    conn._reply_queues["8"] = reply_q

    envelope = envelope_pb2.ReplyEnvelope(corr_id=8)
    envelope.cfg.target = config_pb2.CONFIG_PLANNER
    envelope.cfg.planner.min_speed = 42.0
    armored = "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")

    conn._ser = _FakeSerial([(armored + "\n").encode("ascii")])
    conn._reader_loop()

    reply = reply_q.get_nowait()
    assert isinstance(reply, envelope_pb2.ReplyEnvelope)
    assert reply.corr_id == 8
    assert reply.WhichOneof("body") == "cfg"
    assert reply.cfg.target == config_pb2.CONFIG_PLANNER
    assert reply.cfg.WhichOneof("patch") == "planner"
    assert reply.cfg.planner.min_speed == pytest.approx(42.0)


def test_reader_loop_binary_reply_with_no_registered_queue_is_dropped():
    """No queue registered for the envelope's corr_id -- dropped silently,
    same "no listener" semantics the text plane's OK/ERR/CFG/ID branch has
    (matches the reader loop's own docstring for that branch)."""
    conn = _new_conn()
    envelope = envelope_pb2.ReplyEnvelope(corr_id=999)
    envelope.ok.SetInParent()
    armored = "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")

    conn._ser = _FakeSerial([(armored + "\n").encode("ascii")])
    conn._reader_loop()  # must not raise

    assert conn._reply_queues == {}


def test_reader_loop_malformed_binary_line_is_dropped_not_raised():
    """Malformed base64/protobuf bytes after the `*B` prefix must not crash
    the reader thread -- dropped silently, like any other undecodable line."""
    conn = _new_conn()
    conn._reply_queues["1"] = queue.Queue()

    conn._ser = _FakeSerial([b"*Bnot-valid-base64!!!\n"])
    conn._reader_loop()  # must not raise

    assert conn._reply_queues["1"].empty()


# ---------------------------------------------------------------------------
# 3. send_envelope() loopback round-trip (real reader thread, mock transport)
# ---------------------------------------------------------------------------


class _LoopbackSerial:
    """Mock transport for send_envelope()'s round-trip test.

    On write(), if the written line is a `*B<base64>` CommandEnvelope,
    synthesizes an Ack ReplyEnvelope (echoing corr_id) and queues it for the
    next readline() -- exercising send_envelope()'s full
    write -> reader-thread -> _handle_binary_reply -> queue -> blocking-read
    path with no real serial port.
    """

    is_open = True

    def __init__(self):
        self._pending: queue.Queue = queue.Queue()

    def write(self, data: bytes) -> int:
        text = data.decode("ascii").strip()
        if text.startswith("*B"):
            raw = base64.b64decode(text[2:])
            cmd = envelope_pb2.CommandEnvelope.FromString(raw)
            reply = envelope_pb2.ReplyEnvelope(corr_id=cmd.corr_id)
            reply.ok.q = 1
            reply.ok.rem = 0.0
            armored = "*B" + base64.b64encode(reply.SerializeToString()).decode("ascii")
            self._pending.put((armored + "\n").encode("ascii"))
        return len(data)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        try:
            return self._pending.get(timeout=0.2)
        except queue.Empty:
            return b""


def test_send_envelope_round_trips_against_loopback():
    conn = _new_conn()
    conn._ser = _LoopbackSerial()
    conn._start_reader()
    try:
        env = envelope_pb2.CommandEnvelope()
        env.ping.SetInParent()
        result = conn.send_envelope(env, read_timeout=500)
    finally:
        conn._stop_reader()

    assert "error" not in result
    reply = result["reply"]
    assert reply is not None
    assert reply.corr_id == env.corr_id  # send_envelope() assigns corr_id
    assert reply.WhichOneof("body") == "ok"
    assert reply.ok.q == 1
    # Reply queue cleaned up after delivery (no leak across calls).
    assert conn._reply_queues == {}


def test_send_envelope_not_connected_returns_error():
    conn = _new_conn()  # _ser stays None -- never connected
    env = envelope_pb2.CommandEnvelope()
    env.ping.SetInParent()

    result = conn.send_envelope(env, read_timeout=100)

    assert "error" in result


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
