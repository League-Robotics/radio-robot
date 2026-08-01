"""Serial connection management for micro:bit relay/direct devices.

Architecture (sprint 025, ticket 001):
---------------------------------------
SerialConnection owns the physical serial port and all I/O on it.  A single
background reader thread holds the only ``_ser.readline()`` call and
demultiplexes every incoming line into one of three queues:

- ``_reply_queues`` — keyed by corr-id; populated by the firmware's
  ``OK``/``ERR``/``CFG`` replies when the host appended ``#<id>`` to the
  command.  ``send()`` blocks on the corr-id-keyed queue entry it created
  before writing.
- ``_tlm_queue``   — bounded (256 frames); receives every ``TLM ...`` line.
  ``read_lines()`` and ``read_pending_lines()`` drain it.
- ``_evt_queue``   — unbounded; receives every ``EVT ...`` line.
  ``read_lines()`` and ``read_pending_lines()`` drain it; so does any caller
  of ``wait_for_evt_done()``.

Nothing outside ``SerialConnection`` reads from ``_ser`` directly.  The only
intentional internal ``_ser`` access points are:

- ``_banner_classify`` — ``write()`` / ``readline()`` during the pre-reader
                         HELLO classify handshake (before ``connect()`` returns).
- ``_relay_handshake``  — ``write()`` / ``readline()`` during the relay
                         ``!ECHO OFF`` / ``!MODE RAW250`` / ``!GO`` sequence,
                         also before the reader thread starts.
- ``_poll_ready``    — ``reset_input_buffer()``, ``write()``, ``readline()``
                       before the reader thread starts.
- ``handshake()``   — ``write()`` / ``flush()`` under ``_write_lock``, before
                       the reader thread starts (device-detection phase only).
- ``_reader_loop``  — sole owner of ``readline()`` after ``connect()`` returns.

Connection handshake (sprint 036, ticket 007):
----------------------------------------------
``connect()`` now performs a HELLO-classify step before the reader thread starts:

1. The port is opened **with DTR asserted** (pyserial default).  DTR pulses on
   open-time close/reopen, resetting any micro:bit on the port and causing it to
   emit a ``DEVICE:`` announcement banner.  There is no ``dtr = False`` override.

2. ``_banner_classify()`` sends ``HELLO`` repeatedly (up to ~10 times, ~200 ms
   apart) and reads each response until it captures a ``DEVICE:<ROLE>:...`` line.
   Parsed ROLE determines the connection mode:

   - ``RADIOBRIDGE`` → relay; proceed to ``_relay_handshake()``.
   - ``NEZHA2``      → direct USB robot; skip to readiness poll.

3. ``_relay_handshake()`` sends ``!ECHO OFF``, ``!MODE RAW250``, then ``!GO``
   and waits for ``# entering data plane``.  After ``!GO`` the relay is a
   transparent byte pipe.  All subsequent traffic is **plain** (no ``>`` prefix).

4. After the handshake, ``connect()`` proceeds with the PING readiness poll, then
   starts the reader thread.  From the reader thread's perspective the relay
   connection is indistinguishable from direct: the same plain send, same
   ``#<id>`` corr-id.

   There is no host keepalive. The ambient "+" keepalive daemon and its
   arm-on-demand contract (sprint 065) were deleted once protocol v5 removed
   the firmware safety-stop watchdog they fed -- "no deadman"
   (docs/protocol-v5.md section 5): every Move now carries its own bounded
   `timeout` backstop by construction, so an idle wire is not a hazard and a
   periodic "+" has nothing to reset.

Radio channel note:
   The relay's channel, group, and mode persist in its flash.  Matching those
   values between the relay and the robot is a bench-setup concern, not managed
   here.  ``_banner_classify()`` queries ``?`` and logs the relay's reported
   channel/group/mode so mismatches are visible in verbose output.

Reader loop:
   Lines beginning with ``#`` are relay status/comment lines.  The reader loop
   drops them silently; they do not generate protocol errors.

Ack matcher (104-003, promoted; ring-based matching since 120):
-----------------------------------------------------------
One piece of P4 wire-protocol support lives here, promoted/added by sprint
103 so every caller -- not just ``NezhaProtocol`` -- gets the same
guarantee without duplicating the algorithm:

- ``wait_for_ack(corr_id, timeout)`` -- the ack-ring matcher.
  ``move``/``stop``/``config`` commands get no synchronous reply; their
  outcome rides ``Telemetry.acks`` (a bounded, depth-4 ring of real
  ``App::Telemetry::ack()`` pushes, telemetry.proto) inside a subsequent
  ``Telemetry`` push (``_binary_tlm_queue``). This method polls that queue
  (via ``drain_binary_tlm()``) for a ring entry matching ``corr_id``,
  bounded by ``timeout``, returning on the FIRST match. 120
  (bench-single-ack-slot-observability-collapses-at-40ms.md) replaced the
  115-003 single scalar ``ack_corr``/``ack_err`` slot (which OVERWROTE on
  any same-primary-period collision -- the "ack-depth-1 tradeoff") with
  this ring: a push past depth 4 still evicts the OLDEST entry, so a
  bounded, but much larger, burst of other acks is now tolerated before
  this matcher would time out. Previously this loop lived inline in
  ``robot_radio.robot.protocol.NezhaProtocol.wait_for_ack()``; that method
  now delegates here so the algorithm has exactly one implementation.

``TelemetrySecondary`` (its own independently-armored ``*B`` line, the
``drain_binary_secondary_tlm()``/``read_binary_secondary_tlm()`` accessors
that used to expose it, and ``_handle_binary_reply()``'s own
ReplyEnvelope-vs-TelemetrySecondary disambiguation fallback) is DELETED
outright (124-009, robot-state-blackboard-...md, issue's own
"TelemetrySecondary dies") -- there is exactly one binary reply shape now,
``pb2.ReplyEnvelope``.
"""

import glob
import queue
import threading
import time
from typing import Any, TYPE_CHECKING

import serial

from robot_radio.io import wire_commands
from robot_radio.io.wire_codec import ByteStreamDemuxer, decode_frame, encode_frame

if TYPE_CHECKING:
    # Type-checking only: importing robot_radio.robot.pb2.envelope_pb2 at
    # RUNTIME module-load time would be circular -- robot_radio.robot's own
    # __init__.py imports robot_radio.robot.protocol, which imports
    # SerialConnection from THIS module, so importing anything under
    # robot_radio.robot (pb2 included) from serial_conn.py's top level would
    # re-enter this partially-initialized module. See _get_envelope_pb2()
    # below for the runtime (lazy, deferred-past-module-load) equivalent.
    # telemetry_pb2 -- no longer imported here (124-009): its only use was
    # TelemetrySecondary's own type annotations, now deleted.
    from robot_radio.robot.pb2 import envelope_pb2

BAUD_RATE = 115200
DEFAULT_PORT = "/dev/cu.usbmodem21431202"
READ_TIMEOUT_S = 0.12


# Active readiness-poll constants.
# After opening the serial port, the device is not immediately ready — the
# first command's reply is reliably lost if we simply sleep.  Instead we
# actively poll: send PING (v2), wait a short per-attempt window, retry until
# we see a valid response or hit the total timeout.
#
# Per-attempt read window: long enough to catch a single readline() from a
# responsive device, short enough that the poll loop is tight.
_POLL_ATTEMPT_DURATION = 130  # ms per PING attempt
# Total readiness budget for the normal (full PING) path.
_POLL_TOTAL_NORMAL_S = 1.5
# Total readiness budget for the fast (skip_ping / cache-hit) path.
# Shorter to preserve the cache speedup; device should already be running.
_POLL_TOTAL_FAST_S = 0.6

# HELLO-classify constants (sprint 036, ticket 007).
# Per-attempt delay between HELLO sends in the banner-classify loop.
_HELLO_ATTEMPT_DELAY_S = 0.20
# Total timeout budget for the HELLO-classify step.
_HELLO_CLASSIFY_TIMEOUT_S = 2.5
# Timeout for each relay command during the !GO handshake sequence.
_RELAY_CMD_TIMEOUT_S = 1.0
# Budget for the unsolicited READY line (RobotLoop::boot()'s tail). Boot is
# device-probe bound, so this tracks probe time, not link speed: measured
# ~5s on tovez over direct serial 2026-07-29. 10s leaves headroom for a
# slower/retrying probe without stalling a connect to firmware that predates
# READY, which times out here and reports ready=False rather than failing.
_READY_TIMEOUT_S = 10.0

# Bounded TLM queue depth: if the consumer is slow, oldest frames are dropped.
_TLM_QUEUE_DEPTH = 256

# _CORR_ID_RE (``#<digits>`` at the end of a reply line) -- DELETED (124-005,
# issue §4): the corr-id-suffix routing it fed (`_handle_text_line()`'s old
# OK/ERR/CFG/ID branch) is a pre-v4 vestige with no firmware emitter behind
# it; corr-id'd replies ride `ReplyEnvelope`, a binary shape, not text.

# Pre-123 binary-plane armor prefix (095-002, M7 Host Codec Mirror): a
# `*B<base64>` line carried one base64-encoded, serialized pb2.ReplyEnvelope.
# Sprint 123 (tickets 001/002/003) replaced this text armor with a binary,
# 0x00-delimited COBS+CRC frame demuxed STRUCTURALLY by ``ByteStreamDemuxer``
# (see ``robot_radio.io.wire_codec``) rather than by a text prefix -- there is
# no more `*B` byte sequence on the wire to check for. Kept only as a
# historical note; no code references it any more.

# Module-level cache for the lazily-imported envelope_pb2 module (see
# _get_envelope_pb2()'s docstring for why this cannot be a top-level
# import).
_envelope_pb2_module = None


def _get_envelope_pb2():
    """Lazily import and cache robot_radio.robot.pb2.envelope_pb2.

    Deferred past module-load time to break a circular import:
    robot_radio.robot's own __init__.py imports robot_radio.robot.protocol,
    which imports SerialConnection from THIS module -- so a top-level
    ``from robot_radio.robot.pb2 import envelope_pb2`` here would re-enter
    serial_conn.py while it is still being initialized (SerialConnection
    not yet defined) whenever something imports robot_radio.io.serial_conn
    before robot_radio.robot. Calling this from inside a method (after all
    modules have finished loading) has no such ordering constraint.
    """
    global _envelope_pb2_module
    if _envelope_pb2_module is None:
        from robot_radio.robot.pb2 import envelope_pb2 as _mod
        _envelope_pb2_module = _mod
    return _envelope_pb2_module


# _get_telemetry_pb2() -- DELETED (124-009): its only caller decoded
# TelemetrySecondary (robot-state-blackboard-...md, issue's own
# "TelemetrySecondary dies") -- msg::Telemetry itself decodes as part of
# ReplyEnvelope via _get_envelope_pb2(), no separate lazy import needed.


def _disable_hupcl(ser) -> None:
    """Clear the HUPCL termios flag so close() does NOT pulse DTR.

    On macOS/Linux the default tty behaviour asserts DTR when the last handle
    closes (HUPCL = "hang up on close"), which the micro:bit DAPLink interprets
    as a target reset. Clearing it lets a CLI command open/close the port
    without rebooting the robot. No-op on platforms without termios.
    """
    try:
        import termios
        fd = ser.fileno()
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~termios.HUPCL      # c_cflag
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except Exception:
        pass


def _split_wire_line(line: bytes) -> tuple[str | None, bytes]:
    """Parse one raw wire LINE into ``(verb, data)`` under protocol v5's
    uniform grammar (124-005, issue §1: the FIRST ``':'`` ends the command;
    everything after is data) -- the host-side mirror of
    ``App::Comms::dispatchLine()`` (comms.cpp). ``verb`` is looked up
    against the generated registry (``robot_radio.io.wire_commands`` --
    messages/commands.h's mirror); returns ``(None, b"")`` if the command
    bytes are not valid ASCII or are not a registered verb at all.

    A colon-less line has its own trailing ``'\\r'`` stripped before the
    lookup (a raw terminal/relay sending ``"\\r\\n"`` line endings) -- a
    colon-less line can only ever be attempting a no-data cleartext verb
    under this grammar, matching ``dispatchLine()``'s own reasoning."""
    command_bytes, sep, data = line.partition(b":")
    if not sep and command_bytes.endswith(b"\r"):
        command_bytes = command_bytes[:-1]
    try:
        command = command_bytes.decode("ascii")
    except UnicodeDecodeError:
        return None, b""
    if command not in wire_commands.VERB_BY_NAME:
        return None, b""
    return command, data


def _is_binary_verb_line(line: bytes) -> bool:
    """True if ``line``'s own parsed ``<COMMAND>`` prefix names a
    registered BINARY verb (``wire_commands.BINARY_VERBS`` -- ``TLM``/
    ``OK``/``ERR``/``MOVE``/``CONFIG``/``STOP``). The discriminator every
    PRE-reader-thread raw-line helper below uses to skip telemetry lines
    while still returning ``DEVICE:``/``PONG:``/``ID:``/``VER:`` replies
    and the RadioRelay's own ``#``-prefixed comment lines (which are not
    registry members at all, hence not "binary" either) as text."""
    command, _ = _split_wire_line(line)
    return command is not None and command in wire_commands.BINARY_VERBS


def _read_text_line_raw(ser, demux: ByteStreamDemuxer, deadline: float) -> str:
    """Read/demux raw bytes off ``ser`` until a complete cleartext line is
    demuxed or ``deadline`` (a ``time.time()``-based timestamp) passes.

    123-002/003/124-005: replaces a raw ``ser.readline()`` call for every
    PRE-reader-thread helper (``_banner_classify``/``_relay_handshake``/
    ``_poll_read_lines``/``probe_devices()``). A binary telemetry line may
    already be interleaved with the HELLO/PING/ID/VER rump at this point
    (the firmware emits telemetry every cycle regardless of whether HELLO
    has been sent yet) -- demuxed lines whose own parsed verb is a
    registered BINARY one (``_is_binary_verb_line()``) are dropped (nothing
    consumes telemetry before the reader thread starts; see
    ``SerialConnection``'s own module docstring for the pre-reader-thread
    access-point list). Returns ``""`` on timeout, mirroring pyserial's own
    ``readline()``-timeout return convention.
    """
    while time.time() < deadline:
        try:
            n = ser.in_waiting or 1
            chunk = ser.read(n)
        except Exception:
            return ""
        if not chunk:
            continue
        for line in demux.feed(chunk):
            if _is_binary_verb_line(line):
                continue  # telemetry line -- nothing consumes it pre-reader
            try:
                return line.decode("utf-8", "ignore")
            except Exception:
                continue
    return ""


def _envelope_command_name(envelope: "envelope_pb2.CommandEnvelope") -> bytes:
    """The ASCII wire-verb name for a populated ``pb2.CommandEnvelope``
    (124-005, issue §1/§3): its own oneof arm name
    (``config``/``stop``/``move`` -- envelope.proto's ``CommandEnvelope.cmd``
    oneof), upper-cased to match the registry (``robot_radio.io.
    wire_commands`` -- messages/commands.h's mirror: ``CONFIG``/``STOP``/
    ``MOVE``). ``send_envelope()``/``send_envelope_fast()`` use this both as
    the wire line's own leading ``<COMMAND>':'`` prefix and as the CRC's
    scope-extension argument (``encode_frame()``'s ``command=``).

    Falls back to ``b"MOVE"`` if no oneof arm is set (``WhichOneof("cmd")``
    is ``None``) -- unreachable in practice (every real caller populates
    exactly one arm before sending), but a call must still produce SOME
    registered verb name rather than an empty command that would fail the
    registry lookup outright on the firmware side."""
    which = envelope.WhichOneof("cmd")
    return (which or "move").upper().encode("ascii")


def _parse_device_banner(line: str) -> dict[str, Any] | None:
    """Parse a ``DEVICE:<ROLE>:<common>:<name>:<serial>`` announcement line.

    Tolerant of garbled prefix — locates ``DEVICE:`` anywhere in the line.
    Returns a dict with ``role``, ``common_name``, ``device_name``,
    ``serial_field`` keys, or ``None`` if no ``DEVICE:`` segment is found.
    """
    idx = line.find("DEVICE:")
    if idx < 0:
        return None
    parts = line[idx:].split(":")
    if len(parts) < 5:
        return None
    return {
        "role": parts[1],
        "common_name": parts[2],
        "device_name": parts[3],
        "serial_field": ":".join(parts[4:]),
    }


# flags bit 5 -- RESERVED (124-008: formerly ack_fresh, deleted with the
# single "freshest ack" scalar slot it gated -- issue §B4). The former
# _ACK_FRESH_BIT constant here is gone; _match_ack_in_frames() below
# needs no freshness gate at all (120: ring-based matching -- see that
# function's own docstring).


def _match_ack_in_frames(
    frames: "list[envelope_pb2.ReplyEnvelope]", corr_id: int
) -> "int | None":
    """Scan a batch of binary-plane ``tlm``-body ``ReplyEnvelope`` frames
    (as returned by ``drain_binary_tlm()``) for an ack-ring entry matching
    ``corr_id``.

    120 (bench-single-ack-slot-observability-collapses-at-40ms.md) replaces
    the single scalar ``ack_corr``/``ack_err`` slot (valid iff ``flags``
    bit 5 / ``ack_fresh``, both DELETED 124-008 issue §B4) this function
    used to scan with a scan over each frame's bounded ``acks`` ring (depth
    ``kAckRingDepth``=12, telemetry.proto) -- a corr_id present ANYWHERE in
    the ring was genuinely acked by ``App::Telemetry::ack()`` at some
    point. No freshness bit is needed to disambiguate a ring entry from a
    stale leftover value the way ``ack_fresh`` was needed for the single
    slot -- an entry is either genuinely in the ring (real) or it is not
    there at all.

    Matching policy (sprint 120 Architecture Step 7's open question,
    resolved here): return on the FIRST (frame, ring-entry) match, scanning
    frames in list order and, within each frame, ring entries in wire
    order (oldest-pushed first -- ``Telemetry::ack()``'s own push/evict
    order, ``telemetry.cpp``). Since a match is an exact ``corr_id``
    equality check, not a "freshest wins" precedence the old single-slot
    design needed, which entry is found first only matters if the SAME
    corr_id was somehow acked more than once (not expected in practice --
    each corr_id is assigned once per ``SerialConnection._corr_counter``
    and acked at most once by the firmware); oldest-first is chosen for a
    deterministic, easy-to-reason-about contract regardless.

    Returns the matching packed ``int`` ring entry itself (124-008: the
    real protobuf decoder hands back a bare int for a packed-scalar
    repeated field -- ``Telemetry.acks`` is ``repeated uint32``, packed
    ``corr_id<<4|err``; ``telemetry_pb2.AckEntry`` is deleted, issue §B4 --
    the caller unpacks ``corr_id``/``err`` via
    ``protocol.AckEntry.from_ring_entry()``) --
    ``SerialConnection.wait_for_ack()``'s own pure-function matching core,
    split out so it can be unit-tested directly against synthetic frame
    batches without a real queue/thread.

    Defensively re-checks ``WhichOneof("body") == "tlm"`` per frame (rather
    than assuming every element of ``frames`` already is one) so a caller
    can also feed it raw, unfiltered ``ReplyEnvelope`` batches in a test.
    """
    for reply in frames:
        if reply.WhichOneof("body") != "tlm":
            continue
        for entry in reply.tlm.acks:
            if (entry >> 4) == corr_id:
                return entry
    return None


class SerialConnection:
    """Manages a serial connection to a micro:bit relay or direct device.

    After ``connect()`` returns, a background reader thread is the sole owner
    of ``_ser.readline()``.  It demultiplexes incoming lines into:

    - ``_reply_queues[corr_id]`` for ``OK``/``ERR``/``CFG`` replies (also the
      corr-id-keyed binary ``ok``/``err``/``cfg``/``id``/``echo`` replies --
      see ``_handle_binary_reply()``).
    - ``_tlm_queue`` for text-plane ``TLM`` frames.
    - ``_binary_tlm_queue`` for binary-plane ``*B`` replies whose body is
      ``tlm`` (097-001) -- unsolicited push frames, always ``corr_id=0``,
      routed BEFORE the corr-id lookup above; see ``_handle_binary_reply()``.
    - ``_evt_queue`` for ``EVT`` lines.

    ``send()`` appends ``#<corr_id>`` to every command and blocks on the
    corr-id-keyed reply queue.  ``read_lines()`` drains ``_tlm_queue`` and
    ``_evt_queue`` without ever calling ``_ser.readline()``.
    """

    def __init__(self, port: str = DEFAULT_PORT, baud: int = BAUD_RATE,
                 mode: str | None = None, on_send=None, on_recv=None,
                 on_debug=None):
        self._port = port
        self._baud = baud
        self._mode = mode  # None = auto-detect from announcement
        self._ser: serial.Serial | None = None
        self.on_send = on_send  # callback(cmd_str) for verbose TX logging
        self.on_recv = on_recv  # callback(line_str) for verbose RX logging
                                # (every decoded line from the reader thread)
        # 129-003 (bench/Sim-only DBG debug channel): callback(line_str) for
        # every "DBG:<message>" line, routed straight here by
        # _handle_text_line() -- NEVER through _text_queue, and NEVER
        # allowed to raise past this class's reader thread. See
        # _handle_text_line()'s own doc comment for the historical defect
        # (a `_log` NameError inside an earlier session's DBG handler
        # killed the reader thread mid-session) this callback's own
        # try/except wrapper exists to make structurally impossible.
        self.on_debug = on_debug
        # Serial-write lock: serializes the keepalive thread's writes with the
        # main thread's command writes so their bytes never interleave.
        self._write_lock = threading.RLock()
        # Monotonic timestamp of the last byte written to the port (any command
        # OR a keepalive).  The keepalive loop only emits "+" once the wire has
        # been idle for a full period: the firmware resets its safety-stop
        # watchdog on ANY received line (LoopScheduler::runCommsIn), so a flowing
        # command stream already feeds the watchdog, and a redundant "+" packed
        # next to a command gets merged with it by the relay's RAW250 framing —
        # corrupting the command ("ERR unknown" / dropped reply).  Updated under
        # _write_lock by send()/send_fast() and the keepalive loop itself.

        # ── Reader thread infrastructure (sprint 025, ticket 001) ────────────
        # One queue per in-flight corr-id; created before write, deleted after
        # reply.  Keyed by str(corr_id); "" is the catch-all for un-correlated
        # OK/ERR/CFG replies.
        self._reply_queues: dict[str, queue.Queue] = {}
        self._reply_lock = threading.Lock()

        # Bounded CLEARTEXT reply queue. The cleartext verbs
        # (DEVICE/PONG/ID/VER/READY/STATUS/HELP) carry no corr-id and cannot
        # -- they are typed by a human at a terminal as often as they are
        # sent by code, and a `STATUS #7` is not a valid verb under the v5
        # grammar. So they route HERE, by arrival, rather than through
        # _reply_queues' corr-id matching. Without this they were parsed
        # correctly and then dropped on the floor, which reads from the
        # caller's side as a dead robot -- see send_cleartext().
        self._text_queue: queue.Queue = queue.Queue(maxsize=64)

        # Bounded TLM queue: drop oldest frame on overflow rather than blocking
        # the reader thread.
        self._tlm_queue: queue.Queue = queue.Queue(maxsize=_TLM_QUEUE_DEPTH)

        # Bounded binary-plane TLM queue (097-001): holds decoded
        # pb2.ReplyEnvelope objects whose body is `tlm` -- the binary
        # counterpart of _tlm_queue above, same depth constant and same
        # drop-oldest-on-overflow policy.  See _handle_binary_reply().
        # Drain/read accessors added 097-003 (drain_binary_tlm()/
        # read_binary_tlm()) -- see those methods below.
        self._binary_tlm_queue: queue.Queue = queue.Queue(maxsize=_TLM_QUEUE_DEPTH)

        # _binary_secondary_queue -- DELETED (124-009): TelemetrySecondary
        # itself is gone (robot-state-blackboard-...md).

        # EVT queue: unbounded — EVT lines must not be dropped.
        self._evt_queue: queue.Queue = queue.Queue()

        # Reader thread state.
        self._reader_thread: threading.Thread | None = None
        self._reader_stop = threading.Event()

        # Monotonically incrementing corr-id source for send().
        self._corr_counter: int = 0

        # 123-002/003: shared demuxer for every PRE-reader-thread raw read
        # (_banner_classify/_relay_handshake/_poll_ready/_poll_read_lines) --
        # one instance per connect() attempt (reset at the top of connect()),
        # so a partial line/frame split across two of those helper calls
        # within the SAME attempt is not lost. See _read_text_line_raw()'s
        # own doc comment for why these can no longer use a plain
        # self._ser.readline() call.
        self._handshake_demux = ByteStreamDemuxer()

        # 123-003: counted-fault surface for a binary frame that fails to
        # decode (malformed COBS, CRC mismatch, or bytes that do not decode
        # as a well-formed ReplyEnvelope) -- the host-side counterpart of
        # firmware's own App::Comms::malformedCount_. Never raises; a
        # caller that wants fault visibility reads this counter.
        self.malformed_frame_count: int = 0

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def port(self) -> str | None:
        return self._port if self.is_open else None

    @property
    def mode(self) -> str | None:
        return self._mode

    def connect(self, skip_ping: bool = False, reset: bool = False) -> dict[str, Any]:
        """Open port, classify device, run handshake, start reader thread.

        Handshake algorithm (sprint 036, ticket 007):

        1. Open the port with **DTR asserted** (pyserial default — no
           ``dtr = False`` override).  Opening the port toggles DTR, which
           resets any micro:bit (via the DAPLink) into a clean command plane.
           The relay then emits a ``DEVICE:`` boot announcement.  Without a
           reset (no DTR pulse) there is no boot banner; the classify step sends
           ``HELLO`` to request one explicitly.

        2. ``_banner_classify()`` sends ``HELLO`` repeatedly (up to the
           ``_HELLO_CLASSIFY_TIMEOUT_S`` budget) until a ``DEVICE:<ROLE>:...``
           line is captured.  The ``DEVICE:`` line is read RAW (before the
           reader thread starts) so it is never silently dropped by the reader.
           Returns ``(role, banner_line)``.

        3a. If ROLE is ``RADIOBRIDGE`` (relay): ``_relay_handshake()`` sends
            ``?`` (log channel/group/mode), ``!ECHO OFF``, ``!MODE RAW250``,
            then ``!GO``.  After ``# entering data plane`` the relay is a
            transparent byte pipe.  ``self._mode`` is set to ``"direct"``
            (indistinguishable from a direct robot connection from the reader's
            perspective).

        3b. If ROLE is ``NEZHA2`` (direct robot): no ``!GO`` needed.
            ``self._mode`` is set to ``"direct"``.

        After the handshake, ``connect()`` proceeds with the PING readiness
        poll, then starts the reader thread.  There is no keepalive to start
        daemon (sprint 065, ticket 005: arm-on-demand contract) -- call
        -- see this module's own docstring for why the daemon was deleted.

        Notes on HUPCL and DTR:
            On macOS/Linux close() pulses DTR via the HUPCL termios flag.
            For the relay this is desirable (next open will reset it again).
            For a direct robot connection it is undesirable (SET state lost).
            The ``reset`` parameter and ``_disable_hupcl`` control this.

        Radio channel note:
            Channel/group matching between relay and robot is a bench-setup
            concern.  ``_relay_handshake()`` queries and logs relay config via
            ``?`` so mismatches are visible to the operator.

        Args:
            skip_ping: When True (cache-hit fast path), skip the HELLO
                classify and readiness poll; use the cached ``self._mode``.
                The return dict will have ``lines=[]`` and ``pinged=False``.
            reset: When True, do NOT disable HUPCL — let close() pulse DTR and
                reset the device on exit.  Default False (preserve device state).
        """
        if self.is_open:
            if self._ser.port == self._port:
                return {"status": "already_connected", "port": self._port, "mode": self._mode}
            self._ser.close()

        # Fresh demuxer for this connect() attempt -- see its own __init__
        # doc comment.
        self._handshake_demux = ByteStreamDemuxer()

        try:
            # Open the port with DTR asserted (pyserial default).
            #
            # Historical note: an earlier version of this code forced
            # ``dtr=False`` to avoid resetting the device on open.  That
            # worked for direct robot connections where the device was already
            # running, but prevented the relay from emitting its DEVICE: banner
            # (no DTR pulse → no reset → no boot announcement).  The banner is
            # required for HELLO-classify.  DTR assertion is the correct default
            # for the relay path; for direct connections it merely resets the
            # robot into a clean state, which is benign.
            self._ser = serial.Serial(baudrate=self._baud, timeout=READ_TIMEOUT_S,
                                      dsrdtr=False, rtscts=False)
            self._ser.port = self._port
            # Do NOT force dtr=False here.  Let DTR stay asserted (the
            # dsrdtr=False kwarg above disables *hardware* flow-control, not
            # the DTR signal itself; pyserial defaults DTR to True when opening).
            self._ser.open()
            if not reset:
                # On macOS/Linux, close() pulses DTR via the HUPCL termios
                # flag, which the DAPLink reads as a target reset.  Clear HUPCL
                # so that close() leaves the line alone and does not reboot the
                # device on CLI exit.  Pass reset=True to deliberately reboot.
                _disable_hupcl(self._ser)

            if skip_ping:
                # Fast cache-hit path: skip HELLO classify and PING poll.
                # Mode was set by the caller from the session cache.
                if self._mode is None:
                    self._mode = "direct"
                self._start_reader()
                return {
                    "status": "connected",
                    "port": self._port,
                    "mode": self._mode,
                    "lines": [],
                    "pinged": False,
                }

            # HELLO-classify: identify device role BEFORE starting the reader.
            # All I/O here is raw (_ser direct); the reader thread is not yet
            # running so DEVICE: lines cannot be silently dropped by it.
            announce: dict[str, Any] | None = None
            relay_info: dict[str, Any] | None = None
            if self._mode is None:
                role, banner_line = self._banner_classify(
                    timeout_s=_HELLO_CLASSIFY_TIMEOUT_S)
                if banner_line:
                    announce = _parse_device_banner(banner_line)
                if role == "relay":
                    relay_info = self._relay_handshake(timeout_s=_RELAY_CMD_TIMEOUT_S)
                    self._mode = "direct"  # post-!GO: transparent plain pipe
                else:
                    # NEZHA2 or unknown role → treat as direct robot.
                    self._mode = "direct"
            elif self._mode == "relay":
                # Caller declared this a relay up-front, so skip role
                # auto-detection — but the relay STILL must be driven from its
                # control plane into the transparent data plane.  Opening the
                # port asserted DTR and reset the relay, so wait for its
                # DEVICE: banner (boot sync), then run the
                # !ECHO OFF / !MODE RAW250 / !GO handshake.  Without this every
                # command hits the relay control plane and comes back as
                # "# error: unknown command (try !HELP)".
                _role, banner_line = self._banner_classify(
                    timeout_s=_HELLO_CLASSIFY_TIMEOUT_S)
                if banner_line:
                    announce = _parse_device_banner(banner_line)
                relay_info = self._relay_handshake(timeout_s=_RELAY_CMD_TIMEOUT_S)
                self._mode = "direct"  # post-!GO: transparent plain pipe
                role = "relay"
            else:
                # Caller supplied an explicit non-relay mode; skip classify.
                role = "direct"

            # Liveness poll via PING. This proves the board ANSWERS; it does
            # NOT prove the loop will accept a Move -- comms_.pump() runs
            # inside RobotLoop::boot() (123-006), so PONG comes back happily
            # throughout a boot window in which every Move is rejected with
            # ERR_NOT_CONFIGURED. Readiness is the separate wait below.
            # _poll_ready uses _ser directly (reader not running yet).
            lines = self._poll_ready(total_timeout_s=_POLL_TOTAL_NORMAL_S)

            # Readiness: wait for the firmware's unsolicited READY line, which
            # it emits once from boot()'s tail (commands.proto READY row).
            # Measured 2026-07-29 over direct serial: without this, 5 of 6
            # fresh connections had their FIRST Move rejected, PING answering
            # throughout, with a ~5s gap. Non-fatal on timeout -- an older
            # firmware never sends READY, and refusing to connect to it would
            # be worse than proceeding; `ready` in the result says which
            # happened so a caller can decide.
            # READY may already have arrived DURING the PING poll above --
            # boot completes on its own schedule, not ours. _poll_ready calls
            # reset_input_buffer() before each attempt, so a READY landing in
            # that window is flushed and would never be seen by the wait
            # below: connect() then burns the full _READY_TIMEOUT_S and
            # reports ready=False on a robot that is, in fact, ready.
            # Checking the poll's own captured lines first closes that race.
            ready = any(ln.strip().startswith("READY") for ln in lines)
            if not ready:
                ready = self._wait_for_ready(timeout_s=_READY_TIMEOUT_S)

            self._start_reader()

            result: dict[str, Any] = {
                "status": "connected",
                "port": self._port,
                "mode": self._mode,
                "lines": lines,
                "pinged": bool(lines),
                "ready": ready,
            }
            if announce:
                result["announcement"] = announce
            if relay_info:
                result["relay_info"] = relay_info
            return result

        except Exception as exc:
            self._ser = None
            return {"error": str(exc), "port": self._port}

    # ── HELLO-classify and relay handshake (sprint 036, ticket 007) ─────────

    def _banner_classify(
        self, timeout_s: float = _HELLO_CLASSIFY_TIMEOUT_S
    ) -> tuple[str, str]:
        """Send HELLO until a DEVICE: banner arrives; return (role, banner_line).

        Operates on ``_ser`` directly (before the reader thread starts).
        Sends ``HELLO`` up to once per ``_HELLO_ATTEMPT_DELAY_S`` and reads
        until ``timeout_s`` is exhausted.

        Returns:
            (role, banner_line) where role is ``"relay"`` or ``"direct"``.
            If no banner is captured within the timeout, returns
            ``("direct", "")``.
        """
        deadline = time.time() + timeout_s
        next_hello = 0.0  # send immediately on the first iteration

        while time.time() < deadline:
            now = time.time()
            if now >= next_hello:
                try:
                    self._ser.write(b"HELLO\n")
                    self._ser.flush()
                except Exception:
                    break
                next_hello = now + _HELLO_ATTEMPT_DELAY_S

            # 123-002/003: raw read + demux, NOT self._ser.readline() -- a
            # binary telemetry frame may already be interleaved with the
            # HELLO rump at this point (the firmware emits telemetry every
            # cycle regardless of whether HELLO has been sent yet), and its
            # content may legitimately embed a literal 0x0A (see
            # _read_text_line_raw()'s own doc comment). Binary frames
            # demuxed here are dropped -- nothing consumes telemetry before
            # the reader thread starts.
            try:
                n = self._ser.in_waiting or 1
                chunk = self._ser.read(n)
            except Exception:
                break
            if not chunk:
                continue

            for line in self._handshake_demux.feed(chunk):
                if _is_binary_verb_line(line):
                    continue
                try:
                    text = line.decode("utf-8", "ignore").strip()
                except Exception:
                    continue
                if not text:
                    continue

                # Look for the DEVICE: announcement.
                idx = text.find("DEVICE:")
                if idx >= 0:
                    parts = text[idx:].split(":")
                    # DEVICE:<ROLE>:<common_name>:<device_name>:<serial>
                    role_field = parts[1].upper() if len(parts) >= 2 else ""
                    if "RADIOBRIDGE" in role_field or "RADIORELAY" in role_field:
                        return "relay", text
                    # NEZHA2 or any other robot type → direct
                    return "direct", text

        # Timeout reached without a banner.
        return "direct", ""

    def _relay_handshake(self, timeout_s: float = _RELAY_CMD_TIMEOUT_S) -> dict[str, Any]:
        """Run the relay command-plane setup and enter the data plane.

        Sequence (must be done before the reader thread starts):
          1. ``?``         — query and log channel/group/mode/power.
          2. ``!ECHO OFF`` — disable transponder echo.
          3. ``!MODE RAW250`` — select headerless 250-byte framing.
          4. ``!GO``       — enter the transparent data plane.

        Returns a dict with the relay's reported config (from ``?``) and
        whether ``# entering data plane`` was seen.

        Operates on ``_ser`` directly.  All relay responses are ``#``-prefixed
        comment lines which the reader loop would silently drop; we consume them
        here before the reader starts.
        """
        info: dict[str, Any] = {}

        def _send_relay_cmd(cmd_bytes: bytes, ack_fragment: str) -> str:
            """Send a relay command and wait for a line containing ack_fragment."""
            try:
                self._ser.write(cmd_bytes)
                self._ser.flush()
            except Exception:
                return ""
            deadline = time.time() + timeout_s
            # 123-002/003: raw read + demux (self._handshake_demux), NOT
            # self._ser.readline() -- see _read_text_line_raw()'s own doc
            # comment for why a binary telemetry frame interleaved at this
            # point makes a plain readline() unsafe.
            while time.time() < deadline:
                text = _read_text_line_raw(self._ser, self._handshake_demux, deadline)
                if not text:
                    break
                if ack_fragment in text:
                    return text
            return ""

        # Query relay config for logging; result is informational.
        query_resp = _send_relay_cmd(b"?\n", "channel:")
        if query_resp:
            info["relay_config"] = query_resp

        # !ECHO OFF — disable echo (transponder mode off).
        _send_relay_cmd(b"!ECHO OFF\n", "echo:")

        # !MODE RAW250 — headerless framing (must match robot firmware).
        _send_relay_cmd(b"!MODE RAW250\n", "mode:")

        # !GO — enter data plane.  Relay replies with "# entering data plane".
        go_resp = _send_relay_cmd(b"!GO\n", "entering data plane")
        info["entered_data_plane"] = "entering data plane" in go_resp

        return info

    def _start_reader(self) -> None:
        """Start the background reader thread.  Idempotent."""
        if self._reader_thread is not None and self._reader_thread.is_alive():
            return
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="serial-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def _stop_reader(self) -> None:
        """Signal the reader thread to stop and wait for it."""
        self._reader_stop.set()
        t = self._reader_thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=1.0)
        self._reader_thread = None

    def _reader_loop(self) -> None:
        """Background reader: sole owner of raw reads off ``_ser``.

        123-002/003/124-005: reads raw bytes (``_ser.read()``, never
        ``_ser.readline()``) and demuxes them through a ``ByteStreamDemuxer``
        (``robot_radio.io.wire_codec``) into complete wire LINES -- mirroring
        firmware's own ``App::Transport::readLine()`` contract exactly. A raw
        ``readline()`` call IS now safe against this wire (protocol v5's
        uniform grammar makes ``'\\n'`` an unconditional terminator -- see
        ``ByteStreamDemuxer``'s own docstring); this class keeps using the
        demuxer for its non-blocking partial-chunk buffering, not because a
        plain ``readline()`` would misparse.

        Each demuxed line is parsed under the uniform grammar
        (``_split_wire_line()``) and routed by its own verb, mirroring
        ``App::Comms::dispatchLine()``'s registry-driven dispatch (comms.cpp)
        -- see ``_handle_wire_line()``.
        """
        demux = ByteStreamDemuxer()
        while not self._reader_stop.is_set():
            try:
                if self._ser is None or not self._ser.is_open:
                    break
                n = self._ser.in_waiting or 1
                chunk = self._ser.read(n)
            except Exception:
                break  # port closed or gone — exit silently

            if not chunk:
                continue

            for line in demux.feed(chunk):
                self._handle_wire_line(line)

    def _handle_wire_line(self, line: bytes) -> None:
        """Parse one demuxed wire LINE under protocol v5's uniform grammar
        (124-005, issue §1: ``<COMMAND>[':' <data>]``) and route it by verb
        -- the reader-thread's own mirror of ``App::Comms::dispatchLine()``
        (comms.cpp). The registry (``wire_commands.VERB_BY_NAME``) is the
        SOLE discriminator for binary vs. cleartext data.

        - Lines beginning with ``#`` → relay status/comment lines (NOT part
          of the v5 grammar at all -- checked BEFORE the registry lookup),
          dropped silently, unchanged from pre-124.
        - An unrecognized/non-ASCII ``<COMMAND>`` → counted in
          ``malformed_frame_count`` (the host-side counterpart of
          firmware's own ``App::Comms::malformedCount_``), dropped.
        - A registered BINARY verb (``TLM``/``OK``/``ERR``) →
          ``_handle_binary_reply()`` (COBS-decode, CRC-verify -- scoped over
          the parsed verb -- then parse as a ``ReplyEnvelope``).
        - A registered CLEARTEXT verb (``DEVICE``/``PONG``/``ID``/``VER``) →
          ``_handle_text_line()``. None of these currently have a live
          reader-thread consumer (the pre-reader-thread handshake path --
          ``_banner_classify()``/``_poll_ready()`` -- already owns the
          synchronous DEVICE:/PONG: round trips connect() needs); dropped
          silently once classified, same "no listener registered" policy
          the pre-124 ``OK``/``ERR``/``CFG`` corr-id routing already used.
        """
        if line.startswith(b"#"):
            return  # relay status/comment line -- not v5 grammar, leave as-is

        command, data = _split_wire_line(line)
        if command is None:
            self.malformed_frame_count += 1
            return

        entry = wire_commands.VERB_BY_NAME[command]
        if entry.binary:
            # Verbose RX hook: raw bytes (not text) for a binary line --
            # the FULL line, command prefix included (124-005: a consumer
            # like testgui/binary_bridge.py's render_log_line() needs the
            # verb to correctly scope decode_frame()'s own CRC check) -- see
            # this class's own on_recv docstring.
            if self.on_recv:
                self.on_recv(line)
            self._handle_binary_reply(data, command.encode("ascii"))
        else:
            text = line.decode("utf-8", "ignore").strip()
            # Verbose RX hook: report every decoded cleartext line (incl.
            # keepalive-would-be/relay comment lines, though those return
            # above) before the routing/drop filters below.
            if self.on_recv:
                self.on_recv(text)
            self._handle_text_line(command, text)

    def _handle_text_line(self, command: str, text: str) -> None:
        """Route one cleartext reply already classified by
        ``_handle_wire_line()`` (``command`` is a registered CLEARTEXT verb
        -- ``DEVICE``/``PONG``/``ID``/``VER``/``DBG``; ``text`` is the FULL
        decoded line, verb included).

        124-005 (issue §4): the pre-v5 ``TLM``/``EVT`` text-branch routing
        and the ``OK``/``ERR``/``CFG``/``ID`` ``#<corr-id>``-suffix routing
        (``_CORR_ID_RE``) are DELETED, not ported -- both were pre-v4
        vestiges with no firmware emitter behind them (telemetry is binary
        now; corr-id'd replies ride ``ReplyEnvelope``, a binary shape).
        Cleartext replies now have a live consumer: ``_text_queue``, drained
        by ``send_cleartext()``. Before that they were parsed correctly and
        then DROPPED here, so every cleartext query looked from the caller's
        side exactly like an unreachable robot -- which cost a bench session
        chasing a radio link that was working the whole time.

        129-003 (bench/Sim-only DBG debug channel): ``DBG`` is intercepted
        HERE, before ``_text_queue`` -- it is an unsolicited, unbounded
        diagnostic stream (``App::debugf()``, ``app/debug.h``), not a
        request/reply verb a blocked ``send_cleartext()`` caller is waiting
        on, so it never enters that queue. Routed to ``on_debug`` instead,
        wrapped in its own try/except so a bug in a CALLER-supplied
        ``on_debug`` handler can never propagate out of this reader thread
        -- the exact historical defect this ticket's own acceptance
        criteria name: a ``_log`` ``NameError`` inside an earlier session's
        DBG handler killed the reader thread mid-session, silently ending
        TELEMETRY delivery too (this method has no other caller than the
        reader thread, ``_handle_wire_line()`` -- see ``_reader_loop()``'s
        own doc comment: nothing between here and there catches an
        exception). A malformed/oversized DBG line cannot reach this point
        already-broken either: ``_handle_wire_line()`` decodes with
        ``"utf-8", "ignore"`` before calling here, so ``text`` is always a
        plain (possibly garbled, never raising) Python string.
        """
        if command == "DBG":
            if self.on_debug is not None:
                try:
                    self.on_debug(text)
                except Exception:
                    pass
            return

        del command  # the full line (verb included) is what callers want
        if self._text_queue.full():
            try:
                self._text_queue.get_nowait()  # drop oldest, never block reader
            except queue.Empty:
                pass
        try:
            self._text_queue.put_nowait(text)
        except queue.Full:
            pass  # racing drain; dropping one status line is harmless

    def _handle_binary_reply(self, frame: bytes, command: bytes) -> None:
        """COBS-decode, CRC-verify, protobuf-decode, and route one binary
        reply LINE's own data (123-002/003/124-005; was a ``*B<base64>``
        armored text line pre-123).

        Called only from ``_handle_wire_line()`` (see its docstring), which
        has already stripped the wire line's own ``<COMMAND>':'`` prefix --
        ``frame`` is the COBS body alone, ``command`` the ASCII verb bytes
        that prefix scoped the CRC over (124-003/124-005, issue §3),
        threaded straight into ``wire_codec.decode_frame()``. A decode
        failure here (malformed COBS or a CRC mismatch, including a
        ``command`` that does not match what the line was actually encoded
        with) increments ``malformed_frame_count`` and the frame is dropped
        outright -- there are no bytes to try a second interpretation
        against.

        Exactly one message type rides this framing now (124-009,
        robot-state-blackboard-...md): ``pb2.ReplyEnvelope`` -- corr-id'd
        ``ok``/``err`` replies and unsolicited ``tlm`` pushes.
        ``pb2.TelemetrySecondary``, its former sibling (103-001 Decision 3,
        hardened 104-003), is DELETED outright, along with the
        ReplyEnvelope-vs-TelemetrySecondary disambiguation fallback this
        method used to need -- ``ReplyEnvelope.FromString()`` either
        succeeds with a populated ``body`` oneof, or the bytes are
        malformed, full stop. Every real ``ReplyEnvelope`` this firmware
        ever sends populates the ``body`` oneof -- ``Comms::sendReply()``
        (corr-id'd ``ok``/``err``) and ``Telemetry::emitPrimary()``
        (unsolicited ``tlm``, ``corr_id=0``) both always set one of the
        three arms; nothing constructs an empty one -- so a parse that
        raises OR succeeds with ``WhichOneof("body") is None`` is treated
        as malformed.

        097-001: a ``tlm`` body is checked FIRST, before the corr-id lookup,
        and routed unconditionally to the bounded, drop-oldest
        ``_binary_tlm_queue`` -- mirroring how ``_reader_loop``'s own
        ``text.startswith("TLM")`` branch is checked before its
        ``OK``/``ERR``/``CFG``/``ID`` corr-id branch. Binary telemetry push
        frames (firmware's ``telemetryEmitBinary()``, sprint 096) always
        carry ``corr_id=0``, and no ``send()``/``send_envelope()`` call ever
        registers a queue under ``"0"`` -- routing them through the corr-id
        table silently dropped every one of them (the bug that ticket
        fixed).

        Every other body (``ok``/``err``/``cfg``/``id``/``echo``) keeps
        routing to ``_reply_queues[str(envelope.corr_id)]``, the SAME queue
        lookup the text plane's ``OK``/``ERR``/``CFG``/``ID`` branch
        performs, keyed by the envelope's own ``corr_id`` field instead of a
        parsed ``#<id>`` suffix. If no queue is registered for that id, the
        reply is dropped silently (same "no listener" semantics as the text
        plane).

        Any decode/parse failure (malformed COBS, a CRC mismatch, malformed
        protobuf bytes, or bytes that parse but leave ``WhichOneof("body")``
        unset) increments ``malformed_frame_count`` and the frame is dropped
        -- a single corrupted binary reply must not crash the reader
        thread, matching this loop's existing tolerance for undecodable
        bytes elsewhere (e.g. the UTF-8-decode ``except Exception: return``
        in ``_handle_text_line()``).
        """
        raw_bytes = decode_frame(frame, command=command)
        if raw_bytes is None:
            self.malformed_frame_count += 1
            return

        reply = None
        try:
            reply = _get_envelope_pb2().ReplyEnvelope.FromString(raw_bytes)
        except Exception:
            reply = None

        if reply is not None and reply.WhichOneof("body") is not None:
            if reply.WhichOneof("body") == "tlm":
                # Bounded binary TLM queue: drop oldest frame on overflow,
                # mirroring _tlm_queue's own policy in _reader_loop's TLM
                # branch.
                if self._binary_tlm_queue.full():
                    try:
                        self._binary_tlm_queue.get_nowait()
                    except queue.Empty:
                        pass
                try:
                    self._binary_tlm_queue.put_nowait(reply)
                except queue.Full:
                    pass  # extremely unlikely race; drop
                return

            corr_id = str(reply.corr_id)
            with self._reply_lock:
                q = self._reply_queues.get(corr_id)
            if q is not None:
                q.put(reply)
            # If no queue is registered for this id, drop silently.
            return

        # Not a (recognizable) ReplyEnvelope -- 124-009: there is no second
        # shape to fall back to any more (TelemetrySecondary is deleted
        # outright, robot-state-blackboard-...md, issue's own
        # "TelemetrySecondary dies") -- drop, matching this loop's
        # tolerance for undecodable bytes elsewhere.
        self.malformed_frame_count += 1

    def _wait_for_ready(self, timeout_s: float) -> bool:
        """Block until the firmware will accept commands.

        Two signals, and BOTH are needed -- neither is sufficient alone:

        - the unsolicited ``READY`` line, emitted exactly once from
          ``RobotLoop::boot()``'s tail.  Edge-triggered: it is the immediate
          answer on a FRESH boot, and it is gone forever once it has passed.
        - ``event_boot_ready`` in the telemetry flags word.  Level-triggered:
          continuously present in every frame, so it is the only thing that
          can answer for a robot that booted BEFORE this connect().

        Opening the port does not reliably reset the board (verified
        2026-07-29: a plain open produced telemetry but no banner and no
        READY, because the robot had booted minutes earlier).  Waiting for
        the line alone therefore times out ~50% of the time on a robot that
        is perfectly ready -- which is exactly the bug this pairing fixes.

        Neither signal means the board is merely ALIVE: PING answers from
        inside ``boot()`` (123-006), so PONG is true throughout a window in
        which every Move is rejected with ERR_NOT_CONFIGURED
        (``rejectDuringBoot``, 125-001).

        Operates on ``_ser`` directly and must only be called before the
        reader thread starts (same contract as ``_poll_ready``).

        Returns True when ready, False on timeout.  False is not fatal --
        firmware predating READY still sets the flag, and firmware predating
        both is simply raced as before -- but the caller should expect its
        first Move to land in the boot window.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            for line in self._poll_read_lines(_POLL_ATTEMPT_DURATION,
                                              stop_token="READY"):
                if line.strip().startswith("READY"):
                    return True
            # No line this pass -- ask the always-present flag instead, for
            # the already-booted case where READY is long gone.
            if self._boot_ready_flag_set():
                return True
        return False

    def _boot_ready_flag_set(self) -> bool:
        """True if a decodable telemetry frame currently reports boot-ready.

        Reads whatever bytes are pending and looks for the flag; a frame that
        does not decode (partial read straddling this call) is simply skipped
        -- the caller retries.
        """
        try:
            from robot_radio.robot.protocol import TLMFrame
        except Exception:                                   # noqa: BLE001
            return False
        for env in self.drain_binary_tlm():
            try:
                frame = TLMFrame.from_pb2(env.tlm)
            except Exception:                               # noqa: BLE001
                continue
            if frame.event_boot_ready:
                return True
        return False

    def _poll_ready(self, total_timeout_s: float = _POLL_TOTAL_NORMAL_S) -> list[str]:
        """Poll PING until the device responds or total_timeout_s is exceeded.

        Sends PING (always plain — after the !GO handshake the relay is a
        transparent pipe), reads for _POLL_ATTEMPT_DURATION, and returns immediately
        if any non-empty response is received. Retries until total_timeout_s
        expires.  Returns the response lines from the first successful attempt
        (or []).

        This method uses ``_ser`` directly and must only be called before the
        reader thread starts.
        """
        deadline = time.time() + total_timeout_s
        cmd = b"PING\n"
        while time.time() < deadline:
            self._ser.reset_input_buffer()
            self._ser.write(cmd)
            self._ser.flush()
            lines = self._poll_read_lines(_POLL_ATTEMPT_DURATION, stop_token="PONG:")
            if lines:
                return lines
        return []

    def _poll_read_lines(self, duration: int,  # [ms]
                         stop_token: str | None = None) -> list[str]:
        """Read lines directly from ``_ser`` for up to ``duration``.

        Used exclusively by ``_poll_ready`` (before the reader thread
        starts) and as ``read_lines()``'s own pre-reader-thread fallback.

        123-002/003: raw read + demux (``self._handshake_demux``), NOT
        ``self._ser.readline()`` -- see ``_read_text_line_raw()``'s own doc
        comment for why a binary telemetry frame interleaved at this point
        makes a plain ``readline()`` unsafe.
        """
        lines: list[str] = []
        deadline = time.time() + (duration / 1000.0)
        while time.time() < deadline:
            text = _read_text_line_raw(self._ser, self._handshake_demux, deadline)
            if not text:
                break
            if "keepalive" in text:
                continue
            lines.append(text)
            if stop_token and stop_token in text:
                break
        return lines

    def disconnect(self) -> dict[str, Any]:
        """Stop the reader thread, then close the serial port."""
        if not self.is_open:
            return {"status": "not_connected"}
        self._stop_reader()
        port = self._port
        self._ser.close()
        self._ser = None
        return {"status": "disconnected", "port": port}

    def send_cleartext(self, verb: str, read_timeout: int = 1500) -> list[str]:  # [ms]
        """Send a bare cleartext verb and return the reply lines.

        Use this for HELLO/PING/ID/VER/STATUS/HELP -- NOT ``send()``, which
        appends a ``#<corr_id>`` suffix and matches the reply back by that
        id. Cleartext verbs carry no corr-id (``STATUS #7`` is not a valid
        verb under the v5 grammar), so through ``send()`` the command is
        malformed AND the reply is unroutable: it returns ``[]`` for a robot
        that answered perfectly.

        Args:
            verb: bare verb, no corr-id, no newline (e.g. ``"STATUS"``).
            read_timeout: how long to collect replies. Some verbs answer
                with several lines, so this always drains the full window
                rather than stopping at the first line.

        Returns:
            The decoded reply lines, verb included, in arrival order.
        """
        if not self.is_open:
            return []

        while not self._text_queue.empty():   # drop stale/unsolicited lines
            try:
                self._text_queue.get_nowait()
            except queue.Empty:
                break

        try:
            with self._write_lock:
                self._ser.write(f"{verb}\n".encode("utf-8"))
                self._ser.flush()
        except Exception:
            return []

        lines: list[str] = []
        deadline = time.time() + (read_timeout / 1000.0)
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            try:
                lines.append(self._text_queue.get(timeout=min(remaining, 0.05)))
            except queue.Empty:
                continue
        return lines

    def send(self, message: str, read_timeout: int = 500,  # [ms]
             stop_token: str | None = "OK") -> dict[str, Any]:
        """Send a plain command, read and return responses.

        Appends a ``#<corr_id>`` suffix to the command so the reader thread
        can route the reply to this call's private queue.  Blocks on that
        queue until a reply arrives or ``read_timeout + 500 ms`` timeout
        elapses.

        All commands are sent **plain** (no ``>`` prefix).  After the
        HELLO-classify / !GO handshake the relay is a transparent byte pipe,
        so no prefix is needed.  Direct robot connections were always plain.

        No ``reset_input_buffer()`` is called — the reader thread is the sole
        owner of the input side of the port.

        Args:
            message: Command string to send (without newline).
            read_timeout: Maximum time to wait for the primary reply, in
                milliseconds.  An extra 500 ms grace is added for queue
                blocking to account for in-flight bytes.
            stop_token: If set, return as soon as a line containing this
                substring is received.  Defaults to ``"OK"`` so blocking
                sends return early on the v2 OK response.  Pass ``None`` to
                always drain for the full ``read_timeout`` window.
        """
        if not self.is_open:
            return {"error": "Not connected. Call connect first."}

        # The relay's RAW250 framing can merge a keepalive "+" with the next
        # command, garbling it → the robot replies "ERR unknown" and never runs
        # it. That reply PROVES the command didn't execute, so re-sending is safe
        # (unlike a dropped OK ack, where the command DID run — never retry that).
        # Retry only on ERR-unknown, a few times, to mask the relay corruption.
        lines: list[str] = []
        for _attempt in range(3):
            # Assign a unique corr-id for this attempt.
            with self._reply_lock:
                self._corr_counter += 1
                corr_id = str(self._corr_counter)
                reply_q: queue.Queue = queue.Queue()
                self._reply_queues[corr_id] = reply_q

            # Build plain command with corr-id suffix.
            corr_suffix = f" #{corr_id}"
            cmd = f"{message}{corr_suffix}\n"

            if self.on_send:
                self.on_send(cmd.rstrip())

            try:
                with self._write_lock:
                    self._ser.write(cmd.encode("utf-8"))
                    self._ser.flush()
            except Exception as exc:
                with self._reply_lock:
                    self._reply_queues.pop(corr_id, None)
                return {"error": str(exc), "sent": message}

            # Drain reply queue until stop_token matched or deadline.
            timeout_s = (read_timeout / 1000.0) + 0.5
            lines = []
            deadline = time.time() + timeout_s
            try:
                while True:
                    remaining = deadline - time.time()
                    if remaining <= 0:
                        break
                    try:
                        line = reply_q.get(timeout=min(remaining, 0.05))
                    except queue.Empty:
                        continue
                    lines.append(line)
                    # An ERR reply is terminal — break immediately so a corrupted
                    # command retries fast instead of waiting the full read_timeout.
                    if (stop_token and stop_token in line) or line.startswith("ERR"):
                        break
            finally:
                with self._reply_lock:
                    self._reply_queues.pop(corr_id, None)

            # Corrupted-command retry: re-send only if the robot rejected garbage.
            if _attempt < 2 and any("ERR" in l and "unknown" in l for l in lines):
                time.sleep(0.03)
                continue
            break

        return {"sent": message, "mode": self._mode, "responses": lines}

    def send_envelope(self, envelope: "envelope_pb2.CommandEnvelope",
                      read_timeout: int = 500,  # [ms]
                      ) -> dict[str, Any]:
        """Send a binary ``pb2.CommandEnvelope``, block for its reply envelope.

        The binary-plane counterpart of ``send()``: serializes ``envelope``,
        COBS+CRC-frames it (123-002/003; 124-005 prepends the ASCII verb
        name -- ``"MOVE:"``/``"CONFIG:"``/``"STOP:"``, derived from
        ``envelope``'s own populated oneof arm via
        ``_envelope_command_name()`` -- and scopes the CRC over it too, per
        protocol v5's uniform grammar) and writes the line plus its trailing
        ``'\\n'`` delimiter (converged from the pre-124 0x00 -- issue §7),
        then blocks on the corr-id-keyed reply queue exactly like ``send()``
        does today -- the envelope's own ``corr_id`` field takes the place
        of ``send()``'s ``#<corr_id>`` text suffix. ``envelope.corr_id`` is
        assigned here (overwriting whatever the caller set) from the same
        ``_corr_counter`` sequence ``send()`` uses, so text and binary
        corr-ids never collide.

        Does NOT reuse ``send()``'s corrupted-command ERR-unknown retry: that
        retry keys off a TEXT reply's literal ``"ERR unknown"`` substring
        signalling relay-framing corruption ate the command. The binary
        plane has no equivalent signal defined yet -- a corrupted/malformed
        binary line fails to decode server-side and produces no reply at
        all, so there is nothing to pattern-match and retry against without
        a NAK the schema does not (yet) define. Ships as a single-attempt
        send; noted here per the ticket's instruction to flag rather than
        silently drop the retry behavior.

        Args:
            envelope: A populated ``pb2.CommandEnvelope``. Its ``corr_id``
                field is overwritten by this call.
            read_timeout: Maximum time to wait for the reply, in
                milliseconds. An extra 500 ms grace is added, matching
                ``send()``.

        Returns:
            ``{"sent": envelope, "mode": self._mode, "reply": ReplyEnvelope
            or None}`` on a send that reached the wire (``reply`` is
            ``None`` on timeout); ``{"error": str, ...}`` if the port isn't
            open or the write itself failed.
        """
        if not self.is_open:
            return {"error": "Not connected. Call connect first."}

        with self._reply_lock:
            self._corr_counter += 1
            corr_id = self._corr_counter
            reply_q: queue.Queue = queue.Queue()
            self._reply_queues[str(corr_id)] = reply_q

        envelope.corr_id = corr_id
        command = _envelope_command_name(envelope)
        frame = encode_frame(envelope.SerializeToString(), command=command)
        line = command + b":" + frame

        if self.on_send:
            self.on_send(line)

        try:
            with self._write_lock:
                self._ser.write(line + b"\n")
                self._ser.flush()
        except Exception as exc:
            with self._reply_lock:
                self._reply_queues.pop(str(corr_id), None)
            return {"error": str(exc), "sent": envelope}

        timeout_s = (read_timeout / 1000.0) + 0.5
        reply = None
        deadline = time.time() + timeout_s
        try:
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                try:
                    reply = reply_q.get(timeout=min(remaining, 0.05))
                    break
                except queue.Empty:
                    continue
        finally:
            with self._reply_lock:
                self._reply_queues.pop(str(corr_id), None)

        return {"sent": envelope, "mode": self._mode, "reply": reply}

    def send_envelope_fast(self, envelope: "envelope_pb2.CommandEnvelope") -> int:
        """Fire-and-forget binary send: assign a corr_id, write the armored
        envelope, return immediately -- no reply-queue registration, no wait.

        The binary-plane counterpart of ``send_fast()`` (103-009, P4
        "telemetry-only return path"): ``send_envelope()`` above registers
        ``_reply_queues[str(corr_id)]`` and blocks up to
        ``read_timeout + 0.5s`` for a ``ReplyEnvelope`` that answers this
        specific ``corr_id`` -- exactly right for arms the firmware still
        answers synchronously (``ping``/``id``/``get``/...), but wrong for
        ``twist``/``stop``/``config``: the P4 firmware reports THEIR outcome
        via the single ack slot riding inside the next ``Telemetry`` push
        (``telemetry.proto`` ``Telemetry.ack_corr``/``ack_err``), never a
        dedicated ``ReplyEnvelope`` for that ``corr_id`` -- waiting on a
        ``_reply_queues`` entry for one of those arms would always time out.
        This method skips that registration/wait entirely: it assigns
        ``envelope.corr_id`` from the SAME ``_corr_counter`` sequence
        ``send_envelope()`` uses (so binary corr-ids never collide whichever
        send path issued them), writes the command-prefixed, COBS+CRC-framed
        line (123-002/003/124-005; was the ``*B<base64>`` armored line
        pre-123), and returns the assigned corr_id for the caller to match
        against the ack slot itself (see ``NezhaProtocol.wait_for_ack()``).

        Raises ``ConnectionError`` if not connected, mirroring
        ``send_fast()``'s own not-open handling (unlike ``send_envelope()``,
        which returns an ``{"error": ...}`` dict -- this method's return
        type is a bare ``int`` corr_id, so there is no dict shape to fold an
        error into).
        """
        if not self.is_open:
            raise ConnectionError("Not connected. Call connect first.")

        with self._reply_lock:
            self._corr_counter += 1
            corr_id = self._corr_counter

        envelope.corr_id = corr_id
        command = _envelope_command_name(envelope)
        frame = encode_frame(envelope.SerializeToString(), command=command)
        line = command + b":" + frame

        if self.on_send:
            self.on_send(line)

        with self._write_lock:
            self._ser.write(line + b"\n")
            self._ser.flush()

        return corr_id

    def send_fast(self, message: str) -> None:
        """Fire-and-forget: send plain command, no response reading.

        Always plain (no ``>`` prefix) — after !GO the relay is transparent.
        """
        if not self.is_open:
            raise ConnectionError("Not connected. Call connect first.")
        cmd = f"{message}\n"

        if self.on_send:
            self.on_send(cmd.rstrip())
        with self._write_lock:
            self._ser.write(cmd.encode("utf-8"))
            self._ser.flush()

    def read_lines(self, duration: int = 500,  # [ms]
                   stop_token: str | None = None) -> list[str]:
        """Read lines from the TLM and EVT queues within the given duration.

        Drains ``_tlm_queue`` and ``_evt_queue`` — does NOT call
        ``_ser.readline()``.  The reader thread feeds these queues.

        Falls back to a direct ``_ser.readline()`` path if the reader thread
        is not running (e.g. during ``_poll_ready``).

        Args:
            duration: Maximum time to read for, in milliseconds (ceiling).
            stop_token: If set, return immediately after the first line that
                contains this substring is received.  Uses a plain substring
                check (``token in line``) so relay-prefix noise does not
                prevent matching.  When ``None`` (default), the loop always
                runs until the deadline.

        Returns:
            List of decoded, stripped response lines.
        """
        if not self.is_open:
            return []

        # If the reader thread is not running, fall back to direct _ser reads
        # (used by _poll_ready via _poll_read_lines; this branch is a safety
        # net for callers that connect without ping, etc.).
        if self._reader_thread is None or not self._reader_thread.is_alive():
            return self._poll_read_lines(duration, stop_token=stop_token)

        lines: list[str] = []
        deadline = time.time() + (duration / 1000.0)
        _sleep = 0.005  # 5 ms between drain attempts

        while time.time() < deadline:
            # Drain both queues in one pass.
            drained_this_pass = False
            for q in (self._tlm_queue, self._evt_queue):
                while True:
                    try:
                        line = q.get_nowait()
                    except queue.Empty:
                        break
                    lines.append(line)
                    drained_this_pass = True
                    if stop_token and stop_token in line:
                        return lines

            if not drained_this_pass:
                time.sleep(_sleep)

        return lines

    def read_pending_lines(self) -> list[str]:
        """Non-blocking drain of the TLM and EVT queues.

        Returns immediately with whatever is currently queued (may be empty).
        Does not block, does not touch ``_ser``.

        This is a named replacement for the ``_conn._ser.in_waiting`` peek
        pattern used in ``protocol.py`` before this sprint.  It has identical
        semantics (non-blocking drain) without exposing the internal serial
        object.

        Returns:
            List of all currently-queued TLM and EVT lines.
        """
        lines: list[str] = []
        for q in (self._tlm_queue, self._evt_queue):
            while True:
                try:
                    lines.append(q.get_nowait())
                except queue.Empty:
                    break
        return lines

    def drain_binary_tlm(self) -> list["envelope_pb2.ReplyEnvelope"]:
        """Non-blocking drain of ``_binary_tlm_queue`` (097-003).

        The binary-plane counterpart of ``read_pending_lines()``: returns
        every currently-queued binary telemetry push frame (raw
        ``pb2.ReplyEnvelope`` objects, body ``tlm`` -- see
        ``_handle_binary_reply()``) without blocking. Callers build a
        ``TLMFrame`` via ``TLMFrame.from_pb2(reply.tlm)`` (``protocol.py``);
        this method stays at the raw-envelope layer, matching
        ``read_pending_lines()``'s own "raw text, caller parses" split.
        """
        frames: list = []
        while True:
            try:
                frames.append(self._binary_tlm_queue.get_nowait())
            except queue.Empty:
                break
        return frames

    def read_binary_tlm(self, duration: int) -> list["envelope_pb2.ReplyEnvelope"]:  # [ms]
        """Block for up to ``duration`` ms, draining ``_binary_tlm_queue``
        (097-003).

        The binary-plane counterpart of ``read_lines()`` for ``_tlm_queue``:
        does NOT call ``_ser.readline()`` -- the reader thread already feeds
        ``_binary_tlm_queue`` independently, this just polls it. Returns
        every ``pb2.ReplyEnvelope`` (body ``tlm``) received during the
        window, in arrival order; may be empty if none arrived.
        """
        if not self.is_open:
            return []

        frames: list = []
        deadline = time.time() + (duration / 1000.0)
        _sleep = 0.005  # 5 ms between drain attempts

        while time.time() < deadline:
            drained_this_pass = False
            while True:
                try:
                    frames.append(self._binary_tlm_queue.get_nowait())
                except queue.Empty:
                    break
                drained_this_pass = True

            if not drained_this_pass:
                time.sleep(_sleep)

        return frames

    # drain_binary_secondary_tlm()/read_binary_secondary_tlm() -- DELETED
    # (124-009): TelemetrySecondary itself is gone
    # (robot-state-blackboard-...md, issue's own "TelemetrySecondary
    # dies") -- there is no second queue left to drain/poll.

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "int | None":  # [ms]
        """Poll incoming binary ``Telemetry`` pushes' bounded ack ring for an
        entry matching ``corr_id``, for up to ``timeout`` ms. Returns the
        matched raw packed ``int`` ring entry (124-008: a plain int,
        ``corr_id<<4|err`` -- ``telemetry_pb2.AckEntry`` is deleted, issue
        §B4; the caller unpacks via
        ``robot_radio.robot.protocol.AckEntry.from_ring_entry()``), or
        ``None`` if the deadline passes with no match -- this wait is
        always bounded, never infinite.

        The ONE shared ack matcher (104-003, promoted out of
        ``robot_radio.robot.protocol.NezhaProtocol.wait_for_ack()``, which
        delegates here -- see this module's own file-header note; ring-based
        matching since 120, bench-single-ack-slot-observability-collapses-
        at-40ms.md): every ``CommandEnvelope`` oneof arm (``move``/``stop``/
        ``config``) gets no synchronous ``ReplyEnvelope`` of its own on the
        P4 wire -- its outcome rides ``Telemetry.acks`` (a bounded, depth-4
        ring of real ``App::Telemetry::ack()`` pushes, oldest evicted first)
        inside the next one or more regular ``Telemetry`` pushes after the
        command reaches the firmware (103-009 Decision 2's "telemetry-only
        return path"). This matcher returns on the FIRST (frame, ring-entry)
        pair where a matching ``corr_id`` is found (via
        ``_match_ack_in_frames()`` below) -- no freshness bit to check, a
        ring entry is either genuinely present (real) or it is not.

        Ring saturation (more than ``kAckRingDepth``=12 OTHER commands acked
        before this one's entry is ever read) is the one remaining real,
        bounded failure mode -- narrower than the pre-120 single slot's
        "ANY other command acked in the same primary period" failure, but
        not eliminated by construction. It surfaces as this method's own
        ``timeout``, exactly like a corr_id that was never acked at all --
        there is no separate "evicted" outcome to report, because from the
        host's perspective the two are indistinguishable (no frame this
        method polled ever carried a matching entry, whether because none
        was ever pushed or because it fell off the ring before being read).
        This ticket's own rapid-fire N-enqueue bench test
        (``src/tests/bench/move_protocol_bench.py``) is the check that
        ``kAckRingDepth``=12 is enough in practice for the queue's own 5-deep
        ``ERR_FULL`` ceiling; retry-on-timeout still covers the residual
        rare case.

        Polls ``drain_binary_tlm()`` -- the same non-blocking binary-
        telemetry drain other callers already use -- in a short sleep loop.
        Telemetry is always-on in the P4 design (no ``STREAM`` arm to arm
        first), so there is nothing to arm before polling; this method only
        drains frames the firmware was already pushing. Note: draining is
        DESTRUCTIVE (frames not matching ``corr_id`` are consumed and
        discarded), so two concurrent ``wait_for_ack()`` calls for different
        corr_ids can race each other over the same queue -- pre-existing
        behavior carried over unchanged from the 103-009 implementation this
        method promotes, not a new defect.
        """
        deadline = time.monotonic() + (timeout / 1000.0)
        while True:
            ack = _match_ack_in_frames(self.drain_binary_tlm(), corr_id)
            if ack is not None:
                return ack
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.01)

    def handshake(self, line: bytes) -> None:
        """Write a raw line to the serial port, no relay prefix, no corr-id.

        Intended for the device-detection phase in ``cli.py`` — specifically
        the HELLO probe that identifies the relay/robot before ``connect()``
        is called.  This method is valid **only before the reader thread
        starts** (i.e. before ``connect()`` returns).  Calling it after the
        reader thread is running bypasses the demux layer and may cause the
        reader to discard the reply.

        Args:
            line: Raw bytes to write, including the trailing newline (e.g.
                ``b"HELLO\\n"``).
        """
        if not self.is_open:
            raise ConnectionError("Not connected. Call connect first.")
        with self._write_lock:
            self._ser.write(line)
            self._ser.flush()


def list_serial_ports() -> list[str]:
    """List USB modem serial ports."""
    return sorted(glob.glob("/dev/cu.usbmodem*"))


def probe_devices(read_timeout: int = 1200) -> list[dict[str, Any]]:  # [ms]
    """Probe each USB modem port with the HELLO-classify protocol.

    Sends ``HELLO`` repeatedly (matching ``_banner_classify``'s protocol; see
    ``.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md``) and
    watches for a ``DEVICE:`` announcement line. The retired ``>PING``
    relay-control-plane prefix is NOT used here: the current relay firmware's
    data-plane pipe does not recognize it on either a direct or
    relay-fronted port, so a probe using it can never observe a live device.

    Returns a list of dicts with port, lines, and a 'responsive' flag (True
    iff a DEVICE: banner line was seen within read_timeout).
    """
    results = []
    for port in list_serial_ports():
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=READ_TIMEOUT_S)
            time.sleep(0.25)
            ser.reset_input_buffer()
            lines: list[str] = []
            responsive = False
            deadline = time.time() + (read_timeout / 1000.0)
            next_hello = 0.0  # send immediately on the first iteration
            # 123-002/003: raw read + demux (one chunk per iteration, so the
            # periodic HELLO resend below still runs), NOT ser.readline() --
            # see _read_text_line_raw()'s own doc comment for why a binary
            # telemetry frame interleaved at this point makes a plain
            # readline() unsafe.
            demux = ByteStreamDemuxer()
            while time.time() < deadline:
                now = time.time()
                if now >= next_hello:
                    ser.write(b"HELLO\n")
                    ser.flush()
                    next_hello = now + _HELLO_ATTEMPT_DELAY_S
                try:
                    chunk = ser.read(ser.in_waiting or 1)
                except Exception:
                    break
                if not chunk:
                    continue
                for kind, payload in demux.feed(chunk):
                    if kind == "binary":
                        continue
                    text = payload.decode("utf-8", "ignore").strip()
                    if not text:
                        continue
                    lines.append(text)
                    if "DEVICE:" in text:
                        responsive = True
                        break
                if responsive:
                    break
            ser.close()
            results.append({"port": port, "lines": lines, "responsive": responsive})
        except Exception as exc:
            results.append({"port": port, "error": str(exc)})
    return results
