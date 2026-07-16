"""src/tests/unit/test_serial_conn_binary_plane.py — 095-002 (M7 Host Codec Mirror).

Extended by 096-007 (M6 Host Config/Telemetry Client) with two more
``_reader_loop`` demux tests (``tlm``/``cfg`` body arms) -- see that
ticket's own file-header note below, just above the two new tests.

Extended again by 097-001 (Binary telemetry push-frame queue): fixes a
real bug 096-007's own `tlm`-body test exposed but didn't catch --
firmware's `telemetryEmitBinary()` push frames always carry `corr_id=0`,
and no `_reply_queues` entry is ever registered under `"0"`, so every
binary telemetry push frame was silently dropped. `_handle_binary_reply()`
now special-cases a `tlm` body BEFORE the corr-id lookup and routes it,
unconditionally, to a new bounded `_binary_tlm_queue`. The 096-007 test
that asserted the OLD (corr-id) routing for a `tlm` body is updated in
place (see its docstring for the supersession note); three new tests cover
the ticket's specific acceptance criteria (corr_id=0 routing, coexistence
with a corr-id-keyed reply in one session, overflow drop-oldest).

Extended again by 097-003 (NezhaProtocol Telemetry Conversion): adds tests
for `drain_binary_tlm()`/`read_binary_tlm()` -- the drain/read accessors
097-001 deferred to this ticket (its own first real caller,
`NezhaProtocol.snap()`/`.read_binary_tlm_frames()`/
`.read_pending_binary_tlm_frames()`, `protocol.py`).

Covers the three things ticket 095-002 asks for, none of which need live
hardware:

1. ``src/host/robot_radio/robot/pb2/`` is importable, including a cross-file
   reference (proves the flat-import sys.path shim in
   ``src/host/robot_radio/robot/pb2/__init__.py`` actually works, not just that
   ``envelope_pb2`` itself parses).
2. ``SerialConnection._reader_loop``'s new ``*B<base64>`` branch correctly
   classifies and demuxes a binary reply by corr-id, WITHOUT disturbing the
   existing TLM/EVT/OK/ERR/CFG/ID/keepalive/`#`-comment branches (fed in the
   same pass, interleaved with the new branch, to prove they still coexist).
3. ``SerialConnection.send_envelope()`` round-trips a full write -> reader-
   thread -> corr-id-queue -> blocking-read cycle against a synthetic
   loopback transport (no real serial port).

Collected under ``src/tests/unit/`` (host-side unit/tooling check, not
sim/bench/playfield-scoped — see ``tests/CLAUDE.md``); ``pyproject.toml``'s
``testpaths`` includes ``tests/unit`` so ``uv run python -m pytest`` collects
it.
"""

import base64
import queue

import pytest

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.pb2 import envelope_pb2


# ---------------------------------------------------------------------------
# 1. pb2 import smoke test
# ---------------------------------------------------------------------------


def test_envelope_pb2_importable_and_roundtrips():
    """`from robot_radio.robot.pb2 import envelope_pb2` works at runtime, and
    a message defined directly in envelope.proto serializes/parses.

    104-002: ``ping`` was pruned by 103-001's schema prune (reserved, not a
    live oneof arm) -- ``stop`` is the P4 wire's live zero-field arm and
    exercises the same "importable + roundtrips" property."""
    env = envelope_pb2.CommandEnvelope(corr_id=5)
    env.stop.SetInParent()

    data = env.SerializeToString()
    env2 = envelope_pb2.CommandEnvelope.FromString(data)

    assert env2.corr_id == 5
    assert env2.WhichOneof("cmd") == "stop"


def test_envelope_pb2_cross_file_import_resolves():
    """envelope_pb2.py contains bare top-level cross-file imports (protoc's
    flat -I protos output, e.g. `import config_pb2 as config__pb2`) that
    only resolve because src/host/robot_radio/robot/pb2/__init__.py inserts its
    own directory onto sys.path before any *_pb2 submodule loads.

    104-002: ``drive`` (DrivetrainCommand) was pruned by 103-001 -- ``config``
    (ConfigDelta -> DrivetrainConfigPatch, defined in config.proto) is the
    P4 wire's live cross-file reference and exercises the actual
    cross-module reference end to end, not just envelope_pb2's own
    locally-defined messages."""
    env = envelope_pb2.CommandEnvelope()
    env.config.drivetrain.trackwidth = 128.0

    assert env.WhichOneof("cmd") == "config"
    assert env.config.WhichOneof("patch") == "drivetrain"
    assert env.config.drivetrain.trackwidth == pytest.approx(128.0)


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


def test_reader_loop_routes_binary_tlm_reply_to_binary_tlm_queue():
    """SUPERSEDED by 097-001 (ticket 001 of this sprint, "Binary telemetry
    push-frame queue"): 096-007 originally asserted that a `tlm` body
    demuxed through the SAME corr-id-keyed _reply_queues machinery as
    ok/err/cfg (see architecture-update.md Decision 2). That was correct as
    written but exposed a real host bug: firmware's telemetryEmitBinary()
    (096) sends every `tlm` body as an unsolicited push frame with
    `corr_id=0`, and no send()/send_envelope() call ever registers a queue
    under "0" -- so every real binary telemetry push frame was silently
    dropped. 097-001 fixes this by special-casing `WhichOneof("body") ==
    "tlm"` in `_handle_binary_reply()`, routing it unconditionally (BEFORE
    the corr-id lookup) to the new bounded `_binary_tlm_queue` instead.
    This test is updated to assert the NEW routing; it still uses a
    nonzero corr_id (7) to prove the tlm branch does not even look at
    corr_id -- see test_reader_loop_routes_binary_tlm_corr_id_zero below
    for the realistic corr_id=0 push-frame case, and
    test_reader_loop_binary_tlm_and_corr_id_reply_coexist_in_one_session
    for the AC's "both in the same reader-thread session" scenario."""
    conn = _new_conn()
    # No queue registered under "7" -- if the tlm branch fell through to the
    # corr-id lookup (the pre-097-001 behavior) this reply would be silently
    # dropped, not delivered.  The absence of _reply_queues["7"] is itself
    # part of the proof that routing no longer depends on corr_id.

    envelope = envelope_pb2.ReplyEnvelope(corr_id=7)
    envelope.tlm.now = 12345
    envelope.tlm.seq = 3
    armored = "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")

    conn._ser = _FakeSerial([(armored + "\n").encode("ascii")])
    conn._reader_loop()

    assert conn._reply_queues == {}  # never touched -- tlm skips corr-id routing
    reply = conn._binary_tlm_queue.get_nowait()
    assert isinstance(reply, envelope_pb2.ReplyEnvelope)
    assert reply.corr_id == 7
    assert reply.WhichOneof("body") == "tlm"
    assert reply.tlm.now == 12345
    assert reply.tlm.seq == 3


def test_reader_loop_routes_binary_tlm_corr_id_zero_to_binary_tlm_queue():
    """097-001's realistic case: firmware's telemetryEmitBinary() always
    sets corr_id=0 on push frames.  A `*B`-armored ReplyEnvelope{tlm,
    corr_id: 0} lands in `_binary_tlm_queue`, not `_reply_queues` -- the
    ticket's first required test."""
    conn = _new_conn()

    envelope = envelope_pb2.ReplyEnvelope(corr_id=0)
    envelope.tlm.now = 999
    envelope.tlm.seq = 1
    armored = "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")

    conn._ser = _FakeSerial([(armored + "\n").encode("ascii")])
    conn._reader_loop()

    assert conn._reply_queues == {}
    assert "0" not in conn._reply_queues
    reply = conn._binary_tlm_queue.get_nowait()
    assert reply.corr_id == 0
    assert reply.WhichOneof("body") == "tlm"
    assert reply.tlm.now == 999


def test_reader_loop_binary_tlm_and_corr_id_reply_coexist_in_one_session():
    """097-001 acceptance criterion: a corr-id-keyed direct reply (a
    simulated Ack) and a corr_id=0 push frame (Telemetry) fed in the SAME
    reader-thread session each land in the correct queue -- the tlm branch
    added ahead of the corr-id lookup must not disturb ok/err/cfg/id/echo
    routing, and vice versa."""
    conn = _new_conn()
    reply_q: queue.Queue = queue.Queue()
    conn._reply_queues["9"] = reply_q

    ack = envelope_pb2.ReplyEnvelope(corr_id=9)
    ack.ok.q = 2
    ack.ok.rem = 5.0
    ack_armored = "*B" + base64.b64encode(ack.SerializeToString()).decode("ascii")

    push = envelope_pb2.ReplyEnvelope(corr_id=0)
    push.tlm.now = 42
    push.tlm.seq = 8
    push_armored = "*B" + base64.b64encode(push.SerializeToString()).decode("ascii")

    conn._ser = _FakeSerial([
        (push_armored + "\n").encode("ascii"),
        (ack_armored + "\n").encode("ascii"),
    ])
    conn._reader_loop()

    ack_reply = reply_q.get_nowait()
    assert ack_reply.corr_id == 9
    assert ack_reply.WhichOneof("body") == "ok"
    assert ack_reply.ok.q == 2

    tlm_reply = conn._binary_tlm_queue.get_nowait()
    assert tlm_reply.corr_id == 0
    assert tlm_reply.WhichOneof("body") == "tlm"
    assert tlm_reply.tlm.now == 42
    assert tlm_reply.tlm.seq == 8


def test_binary_tlm_queue_drops_oldest_on_overflow():
    """097-001 acceptance criterion: pushing more frames than the queue
    depth drops the OLDEST frame, matching _tlm_queue's documented
    drop-oldest-on-overflow policy.  Uses a small monkey-patched queue
    depth (3) instead of the real _TLM_QUEUE_DEPTH (256) so the test stays
    fast; the overflow logic itself is depth-agnostic."""
    conn = _new_conn()
    conn._binary_tlm_queue = queue.Queue(maxsize=3)

    def _armored_tlm(seq: int) -> str:
        envelope = envelope_pb2.ReplyEnvelope(corr_id=0)
        envelope.tlm.seq = seq
        return "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")

    # Push 5 frames (seq 0..4) through a depth-3 queue directly via
    # _handle_binary_reply -- oldest (0, 1) must be dropped, leaving (2, 3, 4).
    for seq in range(5):
        conn._handle_binary_reply(_armored_tlm(seq))

    remaining = []
    while not conn._binary_tlm_queue.empty():
        remaining.append(conn._binary_tlm_queue.get_nowait())

    assert [r.tlm.seq for r in remaining] == [2, 3, 4]


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
        env.stop.SetInParent()  # 104-002: ping pruned, stop is the live zero-field arm
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
    env.stop.SetInParent()  # 104-002: ping pruned, stop is the live zero-field arm

    result = conn.send_envelope(env, read_timeout=100)

    assert "error" in result


# ---------------------------------------------------------------------------
# 3b. send_envelope_fast() (103-009 -- P4 telemetry-only return path)
# ---------------------------------------------------------------------------


class _RecordingSerial:
    """A fake `_ser` that just records every write -- send_envelope_fast()
    never reads a reply (that is the whole point), so there is nothing to
    synthesize on write() the way `_LoopbackSerial` does for send_envelope()."""

    is_open = True

    def __init__(self) -> None:
        self.writes: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def flush(self) -> None:
        pass


def test_send_envelope_fast_writes_armored_envelope_and_returns_corr_id():
    conn = _new_conn()
    conn._ser = _RecordingSerial()
    env = envelope_pb2.CommandEnvelope()
    env.stop.SetInParent()

    corr_id = conn.send_envelope_fast(env)

    assert corr_id == 1
    assert env.corr_id == 1  # envelope.corr_id assigned in place
    [written] = conn._ser.writes
    text = written.decode("ascii").strip()
    assert text.startswith("*B")
    decoded = envelope_pb2.CommandEnvelope.FromString(base64.b64decode(text[2:]))
    assert decoded.corr_id == 1
    assert decoded.WhichOneof("cmd") == "stop"


def test_send_envelope_fast_registers_no_reply_queue():
    """The whole point of the _fast suffix: no _reply_queues entry is ever
    registered for this corr_id -- there is nothing to wait on and nothing
    to leak/clean up."""
    conn = _new_conn()
    conn._ser = _RecordingSerial()
    env = envelope_pb2.CommandEnvelope()
    env.stop.SetInParent()

    conn.send_envelope_fast(env)

    assert conn._reply_queues == {}


def test_send_envelope_fast_shares_the_corr_counter_with_send_envelope():
    """Binary corr-ids never collide regardless of which send path issued
    them -- send_envelope_fast() draws from the SAME _corr_counter sequence
    send_envelope() uses."""
    conn = _new_conn()
    conn._ser = _LoopbackSerial()
    conn._start_reader()
    try:
        env1 = envelope_pb2.CommandEnvelope()
        env1.stop.SetInParent()
        result = conn.send_envelope(env1, read_timeout=200)
        assert "error" not in result

        env2 = envelope_pb2.CommandEnvelope()
        env2.stop.SetInParent()
        corr_id2 = conn.send_envelope_fast(env2)
    finally:
        conn._stop_reader()

    assert corr_id2 == env1.corr_id + 1


def test_send_envelope_fast_not_connected_raises():
    conn = _new_conn()  # _ser stays None -- never connected
    env = envelope_pb2.CommandEnvelope()
    env.stop.SetInParent()

    with pytest.raises(ConnectionError):
        conn.send_envelope_fast(env)


# ---------------------------------------------------------------------------
# 4. drain_binary_tlm() / read_binary_tlm() (097-003)
# ---------------------------------------------------------------------------


class _StaticOpenSerial:
    """A fake `_ser` that only needs to answer `is_open` truthfully -- these
    two accessors never touch `_ser.readline()`/`write()` (they poll
    `_binary_tlm_queue`, which the reader thread fills independently), so a
    minimal stand-in is enough."""

    is_open = True


def _armored_tlm_reply(seq: int, corr_id: int = 0) -> envelope_pb2.ReplyEnvelope:
    envelope = envelope_pb2.ReplyEnvelope(corr_id=corr_id)
    envelope.tlm.seq = seq
    return envelope


def test_drain_binary_tlm_returns_all_queued_frames_and_empties_queue():
    conn = _new_conn()
    for seq in range(3):
        conn._binary_tlm_queue.put_nowait(_armored_tlm_reply(seq))

    frames = conn.drain_binary_tlm()

    assert [f.tlm.seq for f in frames] == [0, 1, 2]
    assert conn._binary_tlm_queue.empty()


def test_drain_binary_tlm_on_empty_queue_returns_empty_list():
    conn = _new_conn()
    assert conn.drain_binary_tlm() == []


def test_read_binary_tlm_returns_frames_already_queued():
    conn = _new_conn()
    conn._ser = _StaticOpenSerial()
    for seq in range(2):
        conn._binary_tlm_queue.put_nowait(_armored_tlm_reply(seq))

    frames = conn.read_binary_tlm(duration=30)

    assert [f.tlm.seq for f in frames] == [0, 1]
    assert conn._binary_tlm_queue.empty()


def test_read_binary_tlm_not_connected_returns_empty_list_immediately():
    conn = _new_conn()  # _ser stays None -- never connected
    assert conn.read_binary_tlm(duration=500) == []


def test_read_binary_tlm_times_out_with_empty_list_when_nothing_arrives():
    conn = _new_conn()
    conn._ser = _StaticOpenSerial()
    assert conn.read_binary_tlm(duration=30) == []


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
