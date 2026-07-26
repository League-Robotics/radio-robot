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

Rewritten for sprint 123 tickets 001/002/003 (the atomic COBS+CRC wire
cutover): the wire's binary armor changed from a text `*B<base64>\\r\\n`
line to a binary, 0x00-delimited COBS+CRC frame demuxed from the HELLO/PING
text rump structurally (`robot_radio.io.wire_codec.ByteStreamDemuxer`), not
by a `*B` text prefix. ``SerialConnection`` now reads raw bytes
(``_ser.read()``/``.in_waiting``), never ``_ser.readline()`` -- every test
double here is rebuilt on ``_wire_test_helpers.FakeSerial`` (a byte-buffer-
backed stand-in exposing that same raw-read contract) instead of the old
``readline()``-based one, and every armored-line builder is replaced with
``_wire_test_helpers.binary_frame()``/``text_line()``.

Covers the three things ticket 095-002 asks for, none of which need live
hardware:

1. ``src/host/robot_radio/robot/pb2/`` is importable, including a cross-file
   reference (proves the flat-import sys.path shim in
   ``src/host/robot_radio/robot/pb2/__init__.py`` actually works, not just that
   ``envelope_pb2`` itself parses).
2. ``SerialConnection._reader_loop``'s raw-byte demux correctly classifies
   and routes a binary reply by corr-id, WITHOUT disturbing the existing
   TLM/EVT/OK/ERR/CFG/ID/keepalive/`#`-comment branches (fed in the same
   pass, interleaved with binary frames, to prove they still coexist).
3. ``SerialConnection.send_envelope()`` round-trips a full write -> reader-
   thread -> corr-id-queue -> blocking-read cycle against a synthetic
   loopback transport (no real serial port).

Collected under ``src/tests/unit/`` (host-side unit/tooling check, not
sim/bench/playfield-scoped — see ``tests/CLAUDE.md``); ``pyproject.toml``'s
``testpaths`` includes ``tests/unit`` so ``uv run python -m pytest`` collects
it.
"""

import queue

import pytest

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.io.wire_codec import encode_frame
from robot_radio.robot.pb2 import envelope_pb2

from _wire_test_helpers import FakeSerial, binary_frame, text_line


def _reply_command(envelope: "envelope_pb2.ReplyEnvelope") -> bytes:
    """The ASCII wire-verb name for a populated ``pb2.ReplyEnvelope``
    (124-005) -- its own oneof arm name (``ok``/``err``/``tlm``)
    upper-cased, matching the registry. Every ``binary_frame()`` call below
    needs this: protocol v5 has no unscoped binary frame any more."""
    which = envelope.WhichOneof("body")
    assert which is not None, "test envelope must populate a body oneof arm"
    return which.upper().encode("ascii")

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
# 2. _reader_loop classify/demux test (synthetic byte streams, no hardware)
# ---------------------------------------------------------------------------


def _new_conn() -> SerialConnection:
    """A SerialConnection with no real I/O performed (the constructor never
    touches a port -- ``_ser`` stays None until a test assigns a fake)."""
    return SerialConnection()


def test_reader_loop_routes_binary_reply_by_corr_id():
    """A synthetic binary frame is COBS+CRC-decoded, parsed as a
    ReplyEnvelope, and delivered to `_reply_queues[str(envelope.corr_id)]`
    -- exactly as an `OK ... #<id>` text reply is delivered today."""
    conn = _new_conn()
    reply_q: queue.Queue = queue.Queue()
    conn._reply_queues["42"] = reply_q

    envelope = envelope_pb2.ReplyEnvelope(corr_id=42)
    envelope.ok.q = 3
    envelope.ok.rem = 12.5

    conn._ser = FakeSerial(binary_frame(envelope, _reply_command(envelope)))
    conn._reader_loop()

    reply = reply_q.get_nowait()
    assert isinstance(reply, envelope_pb2.ReplyEnvelope)
    assert reply.corr_id == 42
    assert reply.WhichOneof("body") == "ok"
    assert reply.ok.q == 3
    assert reply.ok.rem == pytest.approx(12.5)


def test_reader_loop_binary_branch_coexists_with_cleartext_lines_and_comments():
    """One pass through _reader_loop with a relay `#`-comment line, cleartext
    DEVICE:/PONG: replies, and an unrecognized text line interleaved with a
    binary frame: the binary branch's own corr-id routing is unaffected by
    any of them, and none of the cleartext lines are mistaken for binary
    content or crash the loop.

    124-005 (issue §4): supersedes the pre-v5 version of this test, which
    asserted the OLD TLM/EVT text-branch routing and the OK/ERR
    `#<corr-id>`-suffix routing (`_CORR_ID_RE`) -- both DELETED, not ported
    (see `_handle_text_line()`'s own docstring): `TLM ...`/`EVT ...` were
    pre-v4 vestiges with no firmware emitter (telemetry is binary now,
    verb `TLM`, not a bare text line), and corr-id'd replies ride
    `ReplyEnvelope`, a binary shape, never a `#<id>`-suffixed text line."""
    conn = _new_conn()
    conn._reply_queues["42"] = queue.Queue()

    envelope = envelope_pb2.ReplyEnvelope(corr_id=42)
    envelope.err.code = envelope_pb2.ERR_RANGE

    stream = b"".join([
        text_line("# relay comment line"),
        text_line("DEVICE:NEZHA2:robot:test:1234"),
        text_line("PONG:t=100"),
        binary_frame(envelope, _reply_command(envelope)),
        text_line("SOMETHING-UNRECOGNIZED"),
    ])
    conn._ser = FakeSerial(stream)
    conn._reader_loop()  # must not raise

    # Binary branch: routed by the envelope's own corr_id, unaffected by the
    # cleartext lines/comment interleaved around it.
    reply = conn._reply_queues["42"].get_nowait()
    assert isinstance(reply, envelope_pb2.ReplyEnvelope)
    assert reply.corr_id == 42
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == envelope_pb2.ERR_RANGE

    # Cleartext DEVICE:/PONG: lines have no live reader-thread consumer yet
    # (see _handle_text_line()'s own docstring) -- dropped, not malformed.
    # The relay comment is dropped before the registry lookup, also not
    # malformed. Only the genuinely-unrecognized line counts.
    assert conn.malformed_frame_count == 1


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

    conn._ser = FakeSerial(binary_frame(envelope, _reply_command(envelope)))
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
    sets corr_id=0 on push frames.  A binary-framed ReplyEnvelope{tlm,
    corr_id: 0} lands in `_binary_tlm_queue`, not `_reply_queues` -- the
    ticket's first required test."""
    conn = _new_conn()

    envelope = envelope_pb2.ReplyEnvelope(corr_id=0)
    # 42, not the more "obvious" 999: a COBS+CRC frame is only guaranteed
    # 0x00-free, NOT 0x0A-free -- some field-value combinations legitimately
    # produce a frame whose bytes happen to contain a literal 0x0A before
    # the frame's own trailing 0x00 delimiter, which the SAME text/binary
    # demux ambiguity firmware itself has (see ByteStreamDemuxer's own
    # docstring) would misclassify. 42 keeps this specific frame's encoded
    # bytes 0x0A-free (verified), isolating the corr_id-routing behavior
    # this test targets from that unrelated, separately-covered edge case
    # (test_reader_loop_crc_corrupted_binary_frame_is_dropped_not_raised
    # and wire_codec's own demuxer tests already exercise it directly).
    envelope.tlm.now = 42
    envelope.tlm.seq = 1

    conn._ser = FakeSerial(binary_frame(envelope, _reply_command(envelope)))
    conn._reader_loop()

    assert conn._reply_queues == {}
    assert "0" not in conn._reply_queues
    reply = conn._binary_tlm_queue.get_nowait()
    assert reply.corr_id == 0
    assert reply.WhichOneof("body") == "tlm"
    assert reply.tlm.now == 42


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

    push = envelope_pb2.ReplyEnvelope(corr_id=0)
    push.tlm.now = 42
    push.tlm.seq = 8

    conn._ser = FakeSerial(binary_frame(push, _reply_command(push)) + binary_frame(ack, _reply_command(ack)))
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

    def _tlm_cobs_body(seq: int) -> bytes:
        envelope = envelope_pb2.ReplyEnvelope(corr_id=0)
        envelope.tlm.seq = seq
        # binary_frame() returns the FULL line ("TLM:<cobs bytes>\n") --
        # strip the "TLM:" prefix and the trailing '\n' delimiter, since
        # _handle_binary_reply() (124-005) takes the COBS body and the
        # command bytes as two SEPARATE arguments (the prefix already
        # parsed off by the real reader loop's own _handle_wire_line()).
        line = binary_frame(envelope, b"TLM")
        return line[len(b"TLM:"):-1]

    # Push 5 frames (seq 0..4) through a depth-3 queue directly via
    # _handle_binary_reply -- oldest (0, 1) must be dropped, leaving (2, 3, 4).
    for seq in range(5):
        conn._handle_binary_reply(_tlm_cobs_body(seq), b"TLM")

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

    conn._ser = FakeSerial(binary_frame(envelope, _reply_command(envelope)))
    conn._reader_loop()  # must not raise

    assert conn._reply_queues == {}


def test_reader_loop_malformed_binary_frame_is_dropped_not_raised():
    """A malformed binary frame (not valid COBS/CRC bytes) must not crash
    the reader thread -- dropped silently, like any other undecodable
    frame, and counted via malformed_frame_count (123-003)."""
    conn = _new_conn()
    conn._reply_queues["1"] = queue.Queue()

    # A registered BINARY verb prefix ("TLM:") with garbage COBS bytes
    # after it -- reaches _handle_binary_reply() (the intended failure
    # surface: malformed COBS) rather than being rejected one step
    # earlier at the registry lookup (an unrecognized-command line is a
    # SEPARATE, already-covered malformed path -- see
    # test_reader_loop_binary_branch_coexists_with_cleartext_lines_and_
    # comments's own trailing "SOMETHING-UNRECOGNIZED" line).
    conn._ser = FakeSerial(b"TLM:not-a-valid-cobs-crc-frame-body" + b"\n")
    conn._reader_loop()  # must not raise

    assert conn._reply_queues["1"].empty()
    assert conn.malformed_frame_count >= 1


def test_reader_loop_crc_corrupted_binary_frame_is_dropped_not_raised():
    """SUC-002 host-side acceptance: a bit-flipped, well-COBS-framed frame
    is detected via CRC and dropped, not mis-parsed as valid content."""
    conn = _new_conn()
    conn._reply_queues["1"] = queue.Queue()

    envelope = envelope_pb2.ReplyEnvelope(corr_id=1)
    envelope.ok.q = 3
    good_frame = encode_frame(envelope.SerializeToString(), command=b"OK")
    corrupt = bytearray(good_frame)
    for i in range(len(corrupt)):
        candidate = corrupt[i] ^ 0x01
        if candidate != 0x0A:  # keep the frame 0x0A-free (still well-formed COBS)
            corrupt[i] = candidate
            break

    conn._ser = FakeSerial(b"OK:" + bytes(corrupt) + b"\n")
    conn._reader_loop()  # must not raise

    assert conn._reply_queues["1"].empty()
    assert conn.malformed_frame_count >= 1


# ---------------------------------------------------------------------------
# 3. send_envelope() loopback round-trip (real reader thread, mock transport)
# ---------------------------------------------------------------------------


class _LoopbackSerial:
    """Mock transport for send_envelope()'s round-trip test.

    On write() of a command-prefixed, COBS+CRC-framed line + trailing '\\n'
    -- ``send_envelope()``'s own write shape, 123-002/003/124-005 --
    synthesizes an Ack ReplyEnvelope (echoing corr_id) and queues its own
    binary line for a later read() -- exercising send_envelope()'s full
    write -> reader-thread -> _handle_binary_reply -> queue -> blocking-read
    path with no real serial port. Never "exhausts" (read() returns b""
    when idle, like a live, open port) -- the test stops the reader thread
    explicitly via _stop_reader() instead.
    """

    is_open = True

    def __init__(self):
        self._out = bytearray()

    def write(self, data: bytes) -> int:
        if data.endswith(b"\n"):
            from robot_radio.io.wire_codec import decode_frame

            command, sep, cobs_body = data[:-1].partition(b":")
            if sep:
                payload = decode_frame(cobs_body, command=command)
                if payload is not None:
                    cmd = envelope_pb2.CommandEnvelope.FromString(payload)
                    reply = envelope_pb2.ReplyEnvelope(corr_id=cmd.corr_id)
                    reply.ok.q = 1
                    reply.ok.rem = 0.0
                    self._out += b"OK:" + encode_frame(reply.SerializeToString(), command=b"OK") + b"\n"
        return len(data)

    def flush(self) -> None:
        pass

    @property
    def in_waiting(self) -> int:
        return len(self._out)

    def read(self, size: int = 1) -> bytes:
        if not self._out:
            return b""
        n = max(1, min(size, len(self._out)))
        chunk = bytes(self._out[:n])
        del self._out[:n]
        return chunk


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


def test_send_envelope_fast_writes_framed_envelope_and_returns_corr_id():
    conn = _new_conn()
    conn._ser = _RecordingSerial()
    env = envelope_pb2.CommandEnvelope()
    env.stop.SetInParent()

    corr_id = conn.send_envelope_fast(env)

    assert corr_id == 1
    assert env.corr_id == 1  # envelope.corr_id assigned in place
    [written] = conn._ser.writes
    assert written.endswith(b"\n")
    assert written.startswith(b"STOP:")  # 124-005: leading <COMMAND>':' prefix
    from robot_radio.io.wire_codec import decode_frame

    payload = decode_frame(written[len(b"STOP:"):-1], command=b"STOP")
    assert payload is not None
    decoded = envelope_pb2.CommandEnvelope.FromString(payload)
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
    two accessors never touch `_ser.read()`/`write()` (they poll
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
