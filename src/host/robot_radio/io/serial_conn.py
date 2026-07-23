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
- ``_keepalive_loop`` — ``write()`` / ``flush()`` under ``_write_lock``.
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

   ``connect()`` does NOT start the keepalive daemon (sprint 065, ticket 005:
   arm-on-demand contract).  The daemon only runs while a caller has
   explicitly called ``start_keepalive()`` -- the layer that owns an
   open-ended motion session (e.g. the TestGUI's ``KeyboardDriver``, via
   ``Transport.arm_keepalive()``) is responsible for arming/disarming it.
   ``disconnect()`` still calls ``stop_keepalive()`` unconditionally as
   harmless, idempotent cleanup.

Radio channel note:
   The relay's channel, group, and mode persist in its flash.  Matching those
   values between the relay and the robot is a bench-setup concern, not managed
   here.  ``_banner_classify()`` queries ``?`` and logs the relay's reported
   channel/group/mode so mismatches are visible in verbose output.

Reader loop:
   Lines beginning with ``#`` are relay status/comment lines.  The reader loop
   drops them silently; they do not generate protocol errors.

Ack matcher and TelemetrySecondary decode (104-003, promoted; ring-based
matching since 120):
-----------------------------------------------------------
Two pieces of P4 wire-protocol support live here, promoted/added by sprint
103 so every caller -- not just ``NezhaProtocol`` -- gets the same
guarantee without duplicating either algorithm:

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
- ``drain_binary_secondary_tlm()`` / ``read_binary_secondary_tlm()`` -- the
  ``TelemetrySecondary`` counterparts of ``drain_binary_tlm()``/
  ``read_binary_tlm()``. ``TelemetrySecondary`` (the slower ~5 Hz
  acc/glitch/ts/cmd_vel diagnostic frame, telemetry.proto) rides its own
  independently-armored ``*B`` line (103-001 Decision 3) -- NOT a
  ``ReplyEnvelope.body`` oneof arm, since that oneof is fixed at
  ``ok``/``err``/``tlm``. The wire has no discriminator byte distinguishing
  a ``TelemetrySecondary`` line from a ``ReplyEnvelope`` line -- both share
  the identical ``*B`` prefix. ``_handle_binary_reply()`` disambiguates
  structurally: it tries ``ReplyEnvelope`` first (the common case), and
  falls back to ``TelemetrySecondary`` only when that parse either raises or
  succeeds with no oneof ``body`` populated (every real ``ReplyEnvelope``
  this firmware ever sends -- unsolicited ``tlm`` pushes and corr-id'd
  ``ok``/``err`` replies alike -- always sets one). See
  ``_handle_binary_reply()``'s own docstring for the full disambiguation
  rationale.
"""

import base64
import glob
import queue
import re
import threading
import time
from typing import Any, TYPE_CHECKING

import serial

if TYPE_CHECKING:
    # Type-checking only: importing robot_radio.robot.pb2.envelope_pb2 at
    # RUNTIME module-load time would be circular -- robot_radio.robot's own
    # __init__.py imports robot_radio.robot.protocol, which imports
    # SerialConnection from THIS module, so importing anything under
    # robot_radio.robot (pb2 included) from serial_conn.py's top level would
    # re-enter this partially-initialized module. See _get_envelope_pb2()/
    # _get_telemetry_pb2() below for the runtime (lazy, deferred-past-
    # module-load) equivalent.
    from robot_radio.robot.pb2 import envelope_pb2, telemetry_pb2

BAUD_RATE = 115200
DEFAULT_PORT = "/dev/cu.usbmodem21431202"
READ_TIMEOUT_S = 0.12

# System safety-stop watchdog keepalive. The firmware safety-stops ANY motion
# after sTimeoutMs (default 500) of host silence, so a host driving open-ended
# motion (S/VW/R) must continuously send "+" keepalives while doing so. We
# send them from a background daemon thread well inside that window, so if
# this process dies the keepalives stop and the robot safety-stops on its
# own. See LoopScheduler.cpp watchdog.
#
# Arm-on-demand contract (sprint 065, ticket 005): start_keepalive() is NOT
# called automatically by connect() -- the daemon only runs while explicitly
# armed by the layer that owns motion (e.g. the TestGUI's KeyboardDriver, via
# Transport.arm_keepalive()/disarm_keepalive()). A connected-but-idle port
# (bounded commands like T/D/G/TURN/RT carry their own TIME stop and never
# need it; a hung host process holding the port open must not keep an
# open-ended motion alive past the watchdog window) sends no ambient "+" at
# all. disconnect() still calls stop_keepalive() unconditionally as harmless,
# idempotent cleanup regardless of whether the daemon was ever armed.
_KEEPALIVE_PERIOD_S = 0.15

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

# Bounded TLM queue depth: if the consumer is slow, oldest frames are dropped.
_TLM_QUEUE_DEPTH = 256

# Corr-id pattern: ``#<digits>`` at the end of a reply line.
_CORR_ID_RE = re.compile(r"#(\d+)$")

# Binary-plane armor prefix (095-002, M7 Host Codec Mirror): a `*B<base64>`
# line carries one base64-encoded, serialized pb2.ReplyEnvelope. See
# architecture-update.md (095) Risk 5 for why `*` cannot collide with any
# text verb, `OK/ERR/...` reply prefix, or the relay's `#`-line convention.
_BINARY_ARMOR_PREFIX = "*B"

# Module-level cache for the lazily-imported envelope_pb2/telemetry_pb2
# modules (see _get_envelope_pb2()'s docstring for why this cannot be a
# top-level import).
_envelope_pb2_module = None
_telemetry_pb2_module = None


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


def _get_telemetry_pb2():
    """Lazily import and cache robot_radio.robot.pb2.telemetry_pb2 (104-003).

    Same circular-import hazard and same deferred-past-module-load fix as
    ``_get_envelope_pb2()`` above -- see that function's docstring. Used by
    ``_handle_binary_reply()`` to decode a ``TelemetrySecondary`` frame (see
    that method's own docstring for why a SECOND pb2 message type is decoded
    off the same ``*B`` armor prefix ``envelope_pb2.ReplyEnvelope`` uses).
    """
    global _telemetry_pb2_module
    if _telemetry_pb2_module is None:
        from robot_radio.robot.pb2 import telemetry_pb2 as _mod
        _telemetry_pb2_module = _mod
    return _telemetry_pb2_module


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


# flags bit 5 (ack_fresh) -- telemetry.proto Telemetry.flags. Mirrors
# robot_radio.robot.protocol's own _FLAG_ACK_FRESH constant; duplicated
# here (rather than imported) because importing robot_radio.robot.protocol
# from this module's top level would be circular -- see this file's own
# TYPE_CHECKING note above. Both share the SAME source of truth: the
# telemetry.proto bit-table comment. Retained for any future reader that
# still wants the single "freshest ack" slot -- _match_ack_in_frames()
# below no longer uses it (120: ring-based matching needs no freshness
# gate -- see that function's own docstring).
_ACK_FRESH_BIT = 1 << 5


def _match_ack_in_frames(
    frames: "list[envelope_pb2.ReplyEnvelope]", corr_id: int
) -> "telemetry_pb2.AckEntry | None":
    """Scan a batch of binary-plane ``tlm``-body ``ReplyEnvelope`` frames
    (as returned by ``drain_binary_tlm()``) for an ack-ring entry matching
    ``corr_id``.

    120 (bench-single-ack-slot-observability-collapses-at-40ms.md) replaces
    the single scalar ``ack_corr``/``ack_err`` slot (valid iff ``flags``
    bit 5 / ``ack_fresh``) this function used to scan with a scan over each
    frame's bounded ``acks`` ring (depth ``kAckRingDepth``=4,
    telemetry.proto) -- a corr_id present ANYWHERE in the ring was
    genuinely acked by ``App::Telemetry::ack()`` at some point. No
    freshness bit is needed to disambiguate a ring entry from a stale
    leftover value the way ``ack_fresh`` was needed for the single slot
    (whose ``ack_corr``/``ack_err`` hold their LAST-WRITTEN value on every
    ordinary frame, fresh or not) -- an entry is either genuinely in the
    ring (real) or it is not there at all.

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

    Returns the matching ``telemetry_pb2.AckEntry`` ring entry itself (the
    caller reads ``corr_id``/``err`` off it -- NOT the frame's own scalar
    ``ack_corr``/``ack_err``, which may belong to a DIFFERENT, later
    command by the time this frame is read) --
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
            if entry.corr_id == corr_id:
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
    - ``_binary_secondary_queue`` for binary-plane ``*B`` lines that decode
      as a ``TelemetrySecondary`` rather than a ``ReplyEnvelope`` (104-003)
      -- see ``_handle_binary_reply()``.
    - ``_evt_queue`` for ``EVT`` lines.

    ``send()`` appends ``#<corr_id>`` to every command and blocks on the
    corr-id-keyed reply queue.  ``read_lines()`` drains ``_tlm_queue`` and
    ``_evt_queue`` without ever calling ``_ser.readline()``.
    """

    def __init__(self, port: str = DEFAULT_PORT, baud: int = BAUD_RATE,
                 mode: str | None = None, on_send=None, on_recv=None):
        self._port = port
        self._baud = baud
        self._mode = mode  # None = auto-detect from announcement
        self._ser: serial.Serial | None = None
        self.on_send = on_send  # callback(cmd_str) for verbose TX logging
        self.on_recv = on_recv  # callback(line_str) for verbose RX logging
                                # (every decoded line from the reader thread)
        # Serial-write lock: serializes the keepalive thread's writes with the
        # main thread's command writes so their bytes never interleave.
        self._write_lock = threading.RLock()
        self._ka_thread: threading.Thread | None = None
        self._ka_stop = threading.Event()
        # Monotonic timestamp of the last byte written to the port (any command
        # OR a keepalive).  The keepalive loop only emits "+" once the wire has
        # been idle for a full period: the firmware resets its safety-stop
        # watchdog on ANY received line (LoopScheduler::runCommsIn), so a flowing
        # command stream already feeds the watchdog, and a redundant "+" packed
        # next to a command gets merged with it by the relay's RAW250 framing —
        # corrupting the command ("ERR unknown" / dropped reply).  Updated under
        # _write_lock by send()/send_fast() and the keepalive loop itself.
        self._last_write_s = time.monotonic()

        # ── Reader thread infrastructure (sprint 025, ticket 001) ────────────
        # One queue per in-flight corr-id; created before write, deleted after
        # reply.  Keyed by str(corr_id); "" is the catch-all for un-correlated
        # OK/ERR/CFG replies.
        self._reply_queues: dict[str, queue.Queue] = {}
        self._reply_lock = threading.Lock()

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

        # Bounded TelemetrySecondary queue (104-003): the slower ~5 Hz
        # diagnostic frame (acc/glitch/ts/cmd_vel -- telemetry.proto's own
        # TelemetrySecondary message) rides its OWN independently-armored
        # `*B` line (103-001 Decision 3), decoded by _handle_binary_reply()
        # and queued here -- the TelemetrySecondary counterpart of
        # _binary_tlm_queue above, same depth constant and same
        # drop-oldest-on-overflow policy. See drain_binary_secondary_tlm()/
        # read_binary_secondary_tlm() below.
        self._binary_secondary_queue: queue.Queue = queue.Queue(maxsize=_TLM_QUEUE_DEPTH)

        # EVT queue: unbounded — EVT lines must not be dropped.
        self._evt_queue: queue.Queue = queue.Queue()

        # Reader thread state.
        self._reader_thread: threading.Thread | None = None
        self._reader_stop = threading.Event()

        # Monotonically incrementing corr-id source for send().
        self._corr_counter: int = 0

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
        poll, then starts the reader thread.  It does NOT start the keepalive
        daemon (sprint 065, ticket 005: arm-on-demand contract) -- call
        ``start_keepalive()`` explicitly (or use a ``Transport``'s
        ``arm_keepalive()``) once an open-ended motion session begins.

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

            # Normal path: active readiness poll via PING.
            # _poll_ready uses _ser directly (reader not running yet).
            lines = self._poll_ready(total_timeout_s=_POLL_TOTAL_NORMAL_S)

            self._start_reader()

            result: dict[str, Any] = {
                "status": "connected",
                "port": self._port,
                "mode": self._mode,
                "lines": lines,
                "pinged": bool(lines),
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

            try:
                raw = self._ser.readline()
            except Exception:
                break
            if not raw:
                continue

            try:
                text = raw.decode("utf-8", "ignore").strip()
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
            while time.time() < deadline:
                try:
                    raw = self._ser.readline()
                except Exception:
                    break
                if not raw:
                    continue
                try:
                    text = raw.decode("utf-8", "ignore").strip()
                except Exception:
                    continue
                if not text:
                    continue
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
        """Background reader: sole owner of ``_ser.readline()``.

        Classifies each decoded, stripped line and routes it to the
        appropriate queue:

        - ``TLM ...``  → ``_tlm_queue``  (drop oldest if full)
        - ``EVT ...``  → ``_evt_queue``
        - ``OK``/``ERR``/``CFG`` with ``#<id>`` suffix → ``_reply_queues[id]``
        - ``OK``/``ERR``/``CFG`` with no corr-id → ``_reply_queues[""]``
        - ``OK keepalive`` / lines containing ``keepalive`` → dropped silently
        - Lines beginning with ``#`` → relay status/comment lines, dropped
        - ``*B<base64>`` (binary plane, 095-002) → dearmored, parsed as a
          ``pb2.ReplyEnvelope``. A ``tlm`` body (096's ``telemetryEmitBinary()``
          push frames, always ``corr_id=0``) routes to ``_binary_tlm_queue``
          (drop oldest if full) BEFORE the corr-id lookup below (097-001) --
          these are unsolicited pushes, never a reply a blocked ``send()``/
          ``send_envelope()`` call is waiting on. Every other body
          (``ok``/``err``/``cfg``/``id``/``echo``) routes to
          ``_reply_queues[envelope.corr_id]`` exactly like an
          ``OK``/``ERR``/``CFG``/``ID`` reply above. A ``*B`` line that does
          NOT decode as a ``ReplyEnvelope`` with a populated ``body`` oneof
          is retried as a ``TelemetrySecondary`` (104-003) and, on success,
          routed to ``_binary_secondary_queue`` (drop oldest if full) --
          see ``_handle_binary_reply()``'s own docstring.
        - Anything else → dropped silently
        """
        while not self._reader_stop.is_set():
            try:
                if self._ser is None or not self._ser.is_open:
                    break
                raw = self._ser.readline()
            except Exception:
                break  # port closed or gone — exit silently

            if not raw:
                continue

            try:
                text = raw.decode("utf-8", "ignore").strip()
            except Exception:
                continue

            if not text:
                continue

            # Verbose RX hook: report every decoded line (incl. keepalive/relay
            # comment lines) before the routing/drop filters below.
            if self.on_recv:
                self.on_recv(text)

            # Drop keepalive acks.
            if "keepalive" in text:
                continue

            # Drop relay comment/status lines (# channel:, # entering data
            # plane, # echo:, # mode:, # DBG ..., etc.).  These are relay
            # command-plane responses that should have been consumed during the
            # pre-reader handshake; any that leak through post-!GO are benign.
            if text.startswith("#"):
                continue

            if text.startswith("TLM"):
                # Bounded TLM queue: drop oldest on overflow.
                if self._tlm_queue.full():
                    try:
                        self._tlm_queue.get_nowait()
                    except queue.Empty:
                        pass
                try:
                    self._tlm_queue.put_nowait(text)
                except queue.Full:
                    pass  # extremely unlikely race; drop
                continue

            if text.startswith("EVT"):
                self._evt_queue.put(text)
                continue

            # Route OK/ERR/CFG/ID replies by corr-id.
            # NOTE: ID replies carry a trailing corr-id (e.g. "ID model=... #7")
            # and must be routed like OK/ERR/CFG — not dropped silently.
            if text.startswith(("OK", "ERR", "CFG", "ID")):
                m = _CORR_ID_RE.search(text)
                if m:
                    corr_id = m.group(1)
                else:
                    corr_id = ""
                with self._reply_lock:
                    q = self._reply_queues.get(corr_id)
                if q is not None:
                    q.put(text)
                # If no queue is registered for this id, drop silently.
                continue

            # Binary plane (095-002, M7): `*B<base64>` carries one
            # serialized pb2.ReplyEnvelope. Routed by the envelope's own
            # `corr_id` field -- the binary-plane equivalent of the
            # `#<digits>` suffix the text plane's OK/ERR/CFG/ID branch above
            # parses out of the line. A pure addition: this branch cannot be
            # reached by any text-plane reply (no existing reply prefix
            # starts with `*`).
            if text.startswith(_BINARY_ARMOR_PREFIX):
                self._handle_binary_reply(text)
                continue

            # All other lines: drop silently (diagnostics, unknown, etc.)

    def _handle_binary_reply(self, text: str) -> None:
        """Dearmor, decode, and route one ``*B<base64>`` binary reply line.

        Called only from ``_reader_loop`` (see its docstring). Strips the
        ``*B`` armor prefix and base64-decodes; a decode failure here
        (malformed base64) is swallowed and the line dropped outright --
        there are no bytes to try a second interpretation against.

        Two message types share this exact ``*B<base64>`` armor (103-001
        Decision 3, hardened 104-003): ``pb2.ReplyEnvelope`` (the common
        case -- corr-id'd ``ok``/``err`` replies and unsolicited ``tlm``
        pushes) and ``pb2.TelemetrySecondary`` (the slower ~5 Hz
        acc/glitch/ts/cmd_vel diagnostic frame, telemetry.proto). The wire
        carries NO discriminator byte between them -- ``TelemetrySecondary``
        rides as its OWN independently-armored line specifically because
        ``ReplyEnvelope.body``'s oneof is fixed at ``ok``/``err``/``tlm``
        (envelope.proto) and cannot grow a fourth arm for it
        (``src/firm/app/telemetry.cpp``'s ``emitSecondary()`` encodes and
        armors a bare ``TelemetrySecondary`` directly, never wrapping it in
        a ``ReplyEnvelope``).

        Disambiguation: try ``ReplyEnvelope`` FIRST. Every real
        ``ReplyEnvelope`` this firmware ever sends populates the ``body``
        oneof -- ``Comms::sendReply()`` (corr-id'd ``ok``/``err``) and
        ``Telemetry::emitPrimary()`` (unsolicited ``tlm``, ``corr_id=0``)
        both always set one of the three arms; nothing constructs an empty
        one. So if the ``ReplyEnvelope`` parse either raises OR succeeds
        with ``WhichOneof("body") is None``, the bytes were never a valid
        ``ReplyEnvelope`` in the first place, and the line is retried as a
        ``TelemetrySecondary``. (Protobuf's wire format is not
        self-describing about message type -- a field-number/wire-type
        collision between the two schemas could in principle parse
        "successfully" into a ``ReplyEnvelope`` with an empty oneof, which is
        exactly the case this fallback exists to catch, alongside an
        outright parse failure.)

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

        A successfully-decoded ``TelemetrySecondary`` (104-003) routes,
        unconditionally, to the bounded, drop-oldest
        ``_binary_secondary_queue`` -- there is no corr-id to route it by
        (``TelemetrySecondary`` carries no ``corr_id`` field at all; it is a
        pure unsolicited push, like primary ``tlm``).

        Any decode/parse failure of EITHER shape (malformed base64,
        malformed protobuf bytes, or bytes that are neither a well-formed
        ``ReplyEnvelope`` nor a well-formed ``TelemetrySecondary``) is
        swallowed and the line dropped -- a single corrupted binary reply
        must not crash the reader thread, matching this loop's existing
        tolerance for undecodable bytes elsewhere (e.g. the UTF-8-decode
        ``except Exception: continue`` above).
        """
        armored = text[len(_BINARY_ARMOR_PREFIX):]
        try:
            raw_bytes = base64.b64decode(armored)
        except Exception:
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

        # Not a (recognizable) ReplyEnvelope -- try TelemetrySecondary
        # (104-003; see this method's own docstring for the disambiguation
        # rationale).
        try:
            secondary = _get_telemetry_pb2().TelemetrySecondary.FromString(raw_bytes)
        except Exception:
            return  # neither shape decoded -- drop, matching this loop's
                     # tolerance for undecodable bytes elsewhere.

        if self._binary_secondary_queue.full():
            try:
                self._binary_secondary_queue.get_nowait()
            except queue.Empty:
                pass
        try:
            self._binary_secondary_queue.put_nowait(secondary)
        except queue.Full:
            pass  # extremely unlikely race; drop

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
            lines = self._poll_read_lines(_POLL_ATTEMPT_DURATION, stop_token="OK pong")
            if lines:
                return lines
        return []

    def _poll_read_lines(self, duration: int,  # [ms]
                         stop_token: str | None = None) -> list[str]:
        """Read lines directly from ``_ser`` for up to ``duration``.

        Used exclusively by ``_poll_ready`` (before the reader thread starts).
        """
        lines: list[str] = []
        deadline = time.time() + (duration / 1000.0)
        while time.time() < deadline:
            try:
                raw = self._ser.readline()
            except Exception:
                break
            if not raw:
                continue
            text = raw.decode("utf-8", "ignore").strip()
            if not text:
                continue
            if "keepalive" in text:
                continue
            lines.append(text)
            if stop_token and stop_token in text:
                break
        return lines

    def disconnect(self) -> dict[str, Any]:
        """Stop keepalive and reader threads, then close the serial port."""
        if not self.is_open:
            return {"status": "not_connected"}
        self.stop_keepalive()
        self._stop_reader()
        port = self._port
        self._ser.close()
        self._ser = None
        return {"status": "disconnected", "port": port}

    # ── safety-stop keepalive ────────────────────────────────────────────────
    def start_keepalive(self, period_s: float = _KEEPALIVE_PERIOD_S) -> None:
        """Start a background daemon thread that streams "+" keepalives so the
        firmware safety-stop watchdog never trips during normal operation. If
        this process dies the daemon thread dies with it, keepalives stop, and
        the robot safety-stops on its own. Idempotent."""
        if self._ka_thread is not None and self._ka_thread.is_alive():
            return
        self._ka_stop.clear()
        self._ka_thread = threading.Thread(
            target=self._keepalive_loop, args=(period_s,),
            name="serial-keepalive", daemon=True)
        self._ka_thread.start()

    def stop_keepalive(self) -> None:
        self._ka_stop.set()
        t = self._ka_thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=1.0)
        self._ka_thread = None

    def _keepalive_loop(self, period_s: float) -> None:
        # Always plain "+": after !GO the relay is a transparent pipe, and
        # direct connections were already plain.  The old ">+" relay prefix is
        # no longer used on any code path.
        #
        # Idle-gate (sprint 040): only emit "+" when the wire has been idle for a
        # full period.  The firmware resets its watchdog on EVERY received line,
        # so any command already serves as a keepalive; emitting a redundant "+"
        # next to a command lets the relay's RAW250 framing merge the two lines
        # and corrupt the command.  Suppressing "+" while commands flow (connect,
        # config, streaming) eliminates that collision, while true idle and long
        # blocking motions (host otherwise silent) still get "+" with nothing to
        # collide with.  Poll at half-period so idle keepalives stay regular and
        # worst-case silence (~1.5×period ≈ 225 ms) stays well under the default
        # sTimeoutMs (500).
        msg = b"+\n"
        poll_s = period_s / 2.0
        while not self._ka_stop.wait(poll_s):
            try:
                if not self.is_open:
                    break
                with self._write_lock:
                    if (time.monotonic() - self._last_write_s) < period_s:
                        continue  # a command fed the watchdog recently; skip "+"
                    if self._ser is not None:
                        self._ser.write(msg)
                        self._ser.flush()
                        self._last_write_s = time.monotonic()
            except Exception:
                break   # port closed / gone — let the robot safety-stop

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
                    self._last_write_s = time.monotonic()  # defer the next "+"
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
        base64-armors it as ``*B<base64>\\n``, writes it, and blocks on the
        corr-id-keyed reply queue exactly like ``send()`` does today -- the
        envelope's own ``corr_id`` field takes the place of ``send()``'s
        ``#<corr_id>`` text suffix. ``envelope.corr_id`` is assigned here
        (overwriting whatever the caller set) from the same
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
        armored = base64.b64encode(envelope.SerializeToString()).decode("ascii")
        line = f"{_BINARY_ARMOR_PREFIX}{armored}\n"

        if self.on_send:
            self.on_send(line.rstrip())

        try:
            with self._write_lock:
                self._ser.write(line.encode("ascii"))
                self._ser.flush()
                self._last_write_s = time.monotonic()  # defer the next "+"
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
        send path issued them), writes the ``*B<base64>`` armored line, and
        returns the assigned corr_id for the caller to match against the ack
        slot itself (see ``NezhaProtocol.wait_for_ack()``).

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
        armored = base64.b64encode(envelope.SerializeToString()).decode("ascii")
        line = f"{_BINARY_ARMOR_PREFIX}{armored}\n"

        if self.on_send:
            self.on_send(line.rstrip())

        with self._write_lock:
            self._ser.write(line.encode("ascii"))
            self._ser.flush()
            self._last_write_s = time.monotonic()  # defer the next "+"

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
            self._last_write_s = time.monotonic()  # defer the next "+"

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

    def drain_binary_secondary_tlm(self) -> list["telemetry_pb2.TelemetrySecondary"]:
        """Non-blocking drain of ``_binary_secondary_queue`` (104-003).

        The ``TelemetrySecondary`` counterpart of ``drain_binary_tlm()``:
        returns every currently-queued ``TelemetrySecondary`` frame (raw
        ``pb2.TelemetrySecondary`` objects -- see ``_handle_binary_reply()``)
        without blocking. Exposes ``acc``/``glitch``/``ts``/``cmd_vel``
        fields the same way primary telemetry fields are exposed: as plain
        attributes on the decoded pb2 message (``.acc_left``,
        ``.cmd_vel_right``, etc.) -- this module stays at the raw-decoded
        layer, matching ``drain_binary_tlm()``'s own "raw message, caller
        adapts" split.
        """
        frames: list = []
        while True:
            try:
                frames.append(self._binary_secondary_queue.get_nowait())
            except queue.Empty:
                break
        return frames

    def read_binary_secondary_tlm(self, duration: int) -> list["telemetry_pb2.TelemetrySecondary"]:  # [ms]
        """Block for up to ``duration`` ms, draining ``_binary_secondary_queue``
        (104-003).

        The ``TelemetrySecondary`` counterpart of ``read_binary_tlm()``: does
        NOT call ``_ser.readline()`` -- the reader thread already feeds
        ``_binary_secondary_queue`` independently via ``_handle_binary_reply()``,
        this just polls it. Returns every ``pb2.TelemetrySecondary`` received
        during the window, in arrival order; may be empty if none arrived.
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
                    frames.append(self._binary_secondary_queue.get_nowait())
                except queue.Empty:
                    break
                drained_this_pass = True

            if not drained_this_pass:
                time.sleep(_sleep)

        return frames

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "telemetry_pb2.AckEntry | None":  # [ms]
        """Poll incoming binary ``Telemetry`` pushes' bounded ack ring for an
        entry matching ``corr_id``, for up to ``timeout`` ms. Returns the
        matched raw ``pb2.AckEntry`` ring entry (the caller reads
        ``corr_id``/``err`` off it -- NOT the enclosing frame's own scalar
        ``ack_corr``/``ack_err``, which may belong to a different command by
        the time the frame is read), or ``None`` if the deadline passes with
        no match -- this wait is always bounded, never infinite.

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

        Ring saturation (more than ``kAckRingDepth``=4 OTHER commands acked
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
        ``kAckRingDepth``=4 is enough in practice for the queue's own 5-deep
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
            while time.time() < deadline:
                now = time.time()
                if now >= next_hello:
                    ser.write(b"HELLO\n")
                    ser.flush()
                    next_hello = now + _HELLO_ATTEMPT_DELAY_S
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", "ignore").strip()
                if text:
                    lines.append(text)
                    if "DEVICE:" in text:
                        responsive = True
                        break
            ser.close()
            results.append({"port": port, "lines": lines, "responsive": responsive})
        except Exception as exc:
            results.append({"port": port, "error": str(exc)})
    return results
