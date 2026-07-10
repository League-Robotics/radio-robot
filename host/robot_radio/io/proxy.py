"""proxy.py -- 097-004 M5: rogo Translator Proxy (``rogo proxy``).

A persistent, standalone text-v2-speaking bridge fronting the real,
binary-only robot connection -- the host's own answer to "how does a
legacy text client keep working once the firmware only speaks binary"
(097-006/007/008 gut the firmware text plane unconditionally; this proxy
is what makes that safe without migrating every legacy consumer first).
See ``clasi/sprints/097-.../architecture-update-r2.md`` Decision 9 + its
2026-07-10 addendum, and the authoritative implementation spec
``clasi/issues/rogo-translator-proxy-text-v2-binary-bridge-on-a-pty.md``.

Transport: a **PTY** (``os.openpty()``), NOT a Unix-domain socket.  Every
legacy consumer already opens its serial port as a plain device PATH
(``serial.Serial(path)`` / ``SerialConnection(port)``), so a PTY is a
zero-code-change drop-in -- a socket would force a code change into every
consumer, recreating the migration problem this proxy exists to avoid.
The proxy publishes a stable symlink (default ``~/.rogo/robot-pty``,
override via ``--link``) to the PTY slave device path; a legacy client
opens THAT symlink exactly like a real serial port.

**Single-client contract**: exactly ONE client is expected to have the PTY
slave open at a time.  This is DOCUMENTED here and in ``rogo proxy
--help``, not policed by this module -- a second concurrent client would
interleave reads/writes with the first, undefined for this bridge's
purposes.  Multi-client fan-out was r2 Decision 9's ORIGINAL ``AF_UNIX``
design; dropped by the PTY transport decision (a PTY has exactly one
"other end" by construction).  The routing core below
(``_handle_client_line``/``_EvtWatcher``) is transport-agnostic, so an
additive ``AF_UNIX`` listener remains cheap to add later if multi-client
need materializes.

**Threading model**: two background daemon threads, plus the caller's own
``run_forever()`` loop:

- **pty-reader thread** (``_reader_loop``): sole reader of the PTY master
  fd.  Line-splits incoming client text, routes each COMPLETE line through
  ``_handle_client_line()`` (which may itself do one or more blocking
  ``SerialConnection.send_envelope()`` round trips to the real robot --
  the reader processes lines SERIALLY, one in-flight command at a time,
  matching the wire's own single-command-at-a-time reality), and writes
  the rendered reply back to the master fd.
- **tlm-pump thread** (``_pump_loop``): the ONE place that ever arms/
  disarms the underlying (real) telemetry stream (``_stream_lock``
  guarded, so it never races a client's own ``STREAM``/``SNAP``/one-shot
  ``TLM`` request handled inline on the reader thread).  Drains
  ``SerialConnection.read_binary_tlm()``, feeds each frame's ``active``
  flag to ``_EvtWatcher``, forwards a rendered text ``TLM ...`` line to
  the client ONLY when the client has itself armed a stream (``STREAM
  n``); an internal watch-period stream armed solely to feed the
  ``_EvtWatcher`` while the client has none of its own is NEVER forwarded
  to the PTY.

Both threads write to the PTY master fd through ``_write_pty()`` under one
lock: TLM lines use the drop-on-``BlockingIOError`` policy (best-effort,
high-rate, a stalled/absent reader must never back up), replies/EVT lines
get a short bounded retry (must not silently vanish just because the
client's read buffer is momentarily full).
"""

from __future__ import annotations

import os
import sys
import threading
import time
import tty
from pathlib import Path
from typing import Any, Callable

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot import legacy_render as render
from robot_radio.robot import legacy_verbs
from robot_radio.robot import protocol
from robot_radio.robot.pb2 import envelope_pb2
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_LINK = "~/.rogo/robot-pty"

# kStreamFloorMs mirror (binary_channel.cpp/telemetry_commands.cpp): the
# firmware's own minimum non-zero STREAM period -- see protocol.py's own
# _STREAM_FLOOR_MS (096-007/097-003), duplicated here to keep this module
# free of importing protocol.py's private module-level constant name.
_STREAM_FLOOR_MS = 20  # [ms]

# Reader-loop / write retry tuning.
_READER_POLL_S = 0.01
_READER_ERROR_SLEEP_S = 0.2
_WRITE_RETRY_STEP_S = 0.01
_WRITE_RETRY_BUDGET_S = 1.0
_PUMP_DRAIN_MS = 50  # [ms] read_binary_tlm() window per pump-loop pass

# Verbs with NO binary arm at all, or gated behind a not-yet-landed one --
# local typed ERR, never a hang or silent drop (ticket requirement).
_ALWAYS_UNSUPPORTED_VERBS = frozenset({"QLEN", "G", "R", "TURN", "GRIP"})
_POSE_OTOS_VERBS = frozenset({"SI", "ZERO", "OI", "OZ", "OR", "OP", "OV", "OL", "OA"})
# Flip to True once sprint 098 lands binary pose/otos CommandEnvelope arms
# (envelope.proto's `pose`/`otos` fields are declared-only today --
# BinaryChannel replies ERR_UNIMPLEMENTED for them).
_POSE_OTOS_BINARY = False

# Relay-control lines a legacy client might send believing it faces a bare
# dongle (docs/protocol-v2.md's relay control-plane grammar) -- swallowed
# locally with a `# ok` comment reply; never forwarded (RelaySerial.
# configure() never checks these once the proxy owns the real connection).
_RELAY_CONTROL_PREFIXES = ("!MODE", "!CG", "!P", "!ECHO", "!GO")

_HELP_TEXT = ("S D T RT MOVE MOVER ECHO PING ID VER HELLO HELP STOP "
              "SET GET STREAM SNAP TLM")


class _EvtWatcher:
    """Synthesizes ``EVT done <VERB> [#id] reason=idle`` off the binary
    ``Telemetry.active`` flag.

    Current firmware emits **no EVT at all**
    (``CommandProcessor::emitEvent`` has zero producers, verified) yet
    legacy calibration scripts block on ``EVT done D/T`` -- this is NEW
    host-side scope, not a translation of an existing firmware signal.

    Pure state machine, no I/O -- ``arm()``/``observe()``/``clear()`` are
    called by the tlm-pump thread (``ProtocolBridge._pump_loop``) and are
    separately unit-testable without a PTY or a connection.

    States:
      ``IDLE``      -- nothing pending.
      ``WAIT_BUSY`` -- an Ack for T/D/RT/MOVE was just seen; waiting for
                       ``Telemetry.active`` to go ``True`` (2s cap).
      ``BUSY``      -- ``active`` observed ``True``; waiting for it to go
                       ``False`` again.

    Transitions: ``WAIT_BUSY`` -> ``active==True`` -> ``BUSY``; ``BUSY`` ->
    ``active==False`` -> emit + ``IDLE``.  The 2s ``WAIT_BUSY`` cap
    expiring while STILL ``WAIT_BUSY`` emits anyway (a short segment can
    finish between two telemetry frames -- late beats missing).  ``STOP``
    (``clear()``) drops any pending watch SILENTLY (v2 spec: STOP emits no
    event).  A new motion verb's ``arm()`` supersedes whatever was
    pending, silently (documented, not flagged as an error).

    **Gap, flagged plainly**: ``EVT safety_stop`` is NOT synthesizable --
    there is no binary watchdog-stop signal to watch for.  This is not a
    regression: firmware emits no EVT at all today either.
    """

    IDLE = "IDLE"
    WAIT_BUSY = "WAIT_BUSY"
    BUSY = "BUSY"

    _WAIT_BUSY_CAP_S = 2.0

    def __init__(self, enabled: bool = True):
        self._enabled = enabled
        self._state = self.IDLE
        self._verb: str | None = None
        self._corr_id: int | str | None = None
        self._deadline: float | None = None

    @property
    def state(self) -> str:
        return self._state

    @property
    def pending(self) -> bool:
        return self._state != self.IDLE

    def arm(self, verb: str, corr_id: int | str | None, now: float | None = None) -> None:
        """Ack for T/D/RT/MOVE just landed -- start (or supersede) a
        pending watch."""
        if not self._enabled:
            return
        now = now if now is not None else time.monotonic()
        self._state = self.WAIT_BUSY
        self._verb = verb
        self._corr_id = corr_id
        self._deadline = now + self._WAIT_BUSY_CAP_S

    def clear(self) -> None:
        """STOP clears any pending watch silently."""
        self._state = self.IDLE
        self._verb = None
        self._corr_id = None
        self._deadline = None

    def observe(self, active: bool, now: float | None = None) -> str | None:
        """Feed one ``Telemetry.active`` sample.  Returns a rendered ``EVT
        done ...`` line the instant one should fire, else ``None``."""
        if self._state == self.IDLE:
            return None
        now = now if now is not None else time.monotonic()
        if self._state == self.WAIT_BUSY:
            if active:
                self._state = self.BUSY
                return None
            if now >= self._deadline:
                return self._emit()
            return None
        if self._state == self.BUSY:
            if not active:
                return self._emit()
            return None
        return None

    def _emit(self) -> str:
        line = render.render_evt_done(self._verb, self._corr_id)
        self.clear()
        return line


class ProtocolBridge:
    """Owns the PTY + the real robot ``SerialConnection``; translates
    text-v2 <-> binary.  See this module's own docstring for the full
    design (transport choice, single-client contract, threading model)."""

    def __init__(self, conn: SerialConnection, link: str = DEFAULT_LINK,
                watch_period: int = 50,  # [ms]
                evt_enabled: bool = True,
                on_log: Callable[[str], None] | None = None):
        self._conn = conn
        self._proto = NezhaProtocol(conn)
        self.link = str(Path(link).expanduser())
        self._watch_period = max(_STREAM_FLOOR_MS, int(watch_period))
        self._on_log = on_log or (lambda _msg: None)

        self._master_fd: int | None = None
        self._slave_fd: int | None = None
        self.slave_path: str | None = None

        self._pty_write_lock = threading.RLock()
        self._stream_lock = threading.RLock()
        self._stop = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._pump_thread: threading.Thread | None = None

        self._device_id = envelope_pb2.DeviceId()
        self._client_stream_period = 0  # [ms] 0 == client has not armed STREAM
        self._last_upstream_period = 0  # [ms] what the real robot is currently streaming at
        self._evt_watcher = _EvtWatcher(enabled=evt_enabled)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> str:
        """Open the PTY, publish the symlink, cache the robot's DeviceId,
        start both background threads.  Returns the PTY slave device
        path (the symlink target -- ``self.link`` is the stable, published
        path a legacy client should actually open)."""
        self._master_fd, self._slave_fd = os.openpty()
        self.slave_path = os.ttyname(self._slave_fd)
        tty.setraw(self._slave_fd)
        os.set_blocking(self._master_fd, False)
        self._publish_symlink()

        self._device_id = self._fetch_device_id()

        self._stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="proxy-pty-reader", daemon=True)
        self._pump_thread = threading.Thread(
            target=self._pump_loop, name="proxy-tlm-pump", daemon=True)
        self._reader_thread.start()
        self._pump_thread.start()
        return self.slave_path

    def stop(self) -> None:
        """Stop both threads, remove the symlink, close both PTY fds.
        Idempotent."""
        self._stop.set()
        for t in (self._reader_thread, self._pump_thread):
            if t is not None and t.is_alive() and t is not threading.current_thread():
                t.join(timeout=1.0)
        self._reader_thread = None
        self._pump_thread = None
        self._remove_symlink()
        for attr in ("_master_fd", "_slave_fd"):
            fd = getattr(self, attr)
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
                setattr(self, attr, None)

    def run_forever(self) -> None:
        """Block until ``stop()`` is called from elsewhere (e.g. a signal
        handler installed by the caller -- ``cli.py``'s ``cmd_proxy``)."""
        try:
            while not self._stop.is_set():
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass

    # ------------------------------------------------------------------
    # PTY symlink publish/cleanup
    # ------------------------------------------------------------------

    def _publish_symlink(self) -> None:
        link_path = Path(self.link)
        link_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            if link_path.is_symlink() or link_path.exists():
                link_path.unlink()
        except OSError:
            pass
        os.symlink(self.slave_path, link_path)

    def _remove_symlink(self) -> None:
        link_path = Path(self.link)
        try:
            if link_path.is_symlink():
                link_path.unlink()
        except OSError:
            pass

    # ------------------------------------------------------------------
    # PTY I/O
    # ------------------------------------------------------------------

    def _write_pty(self, text: str, drop_ok: bool) -> None:
        """Write one line (+ ``\\n``) to the PTY master fd.

        ``drop_ok=True`` (TLM lines): a single non-blocking attempt: drop
        silently on ``BlockingIOError`` (a stalled/absent reader must never
        back up the pump thread).  ``drop_ok=False`` (replies/EVT): retry
        every ``_WRITE_RETRY_STEP_S`` up to ``_WRITE_RETRY_BUDGET_S`` before
        giving up -- must not silently vanish just because the client's
        read buffer is momentarily full.
        """
        if self._master_fd is None:
            return
        data = (text + "\n").encode("utf-8")
        deadline = time.monotonic() + (0.0 if drop_ok else _WRITE_RETRY_BUDGET_S)
        with self._pty_write_lock:
            while True:
                try:
                    os.write(self._master_fd, data)
                    return
                except BlockingIOError:
                    if drop_ok or time.monotonic() >= deadline:
                        return
                    time.sleep(_WRITE_RETRY_STEP_S)
                except OSError:
                    return

    def _reader_loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = os.read(self._master_fd, 4096)
            except BlockingIOError:
                time.sleep(_READER_POLL_S)
                continue
            except OSError:
                # Linux: client close -> EIO. macOS: blocks until reopen (so
                # this branch is effectively Linux-only in practice). Never
                # stop reading the master -- see this module's own PTY
                # lifecycle note (tcdrain deadlocks if the master isn't
                # being read).
                time.sleep(_READER_ERROR_SLEEP_S)
                continue
            if not chunk:
                time.sleep(_READER_POLL_S)
                continue
            buf += chunk
            while b"\n" in buf:
                raw_line, buf = buf.split(b"\n", 1)
                text = raw_line.decode("utf-8", "ignore").strip("\r").strip()
                if not text:
                    continue
                try:
                    reply_line = self._handle_client_line(text)
                except Exception as exc:  # a bad line must never kill the reader
                    reply_line = f"ERR internal {exc}"
                if reply_line:
                    self._write_pty(reply_line, drop_ok=False)

    # ------------------------------------------------------------------
    # Startup DeviceId cache (HELLO answers locally from this)
    # ------------------------------------------------------------------

    def _fetch_device_id(self) -> "envelope_pb2.DeviceId":
        env = envelope_pb2.CommandEnvelope()
        env.id.SetInParent()
        result = self._conn.send_envelope(env, read_timeout=500)
        reply = result.get("reply")
        if reply is not None and reply.WhichOneof("body") == "id":
            return reply.id
        return envelope_pb2.DeviceId()

    # ------------------------------------------------------------------
    # Client-line routing -- the full verb table.  See
    # clasi/issues/rogo-translator-proxy-text-v2-binary-bridge-on-a-pty.md
    # "Verb routing" for the authoritative table this mirrors.
    # ------------------------------------------------------------------

    def _handle_client_line(self, raw: str) -> str | None:
        stripped, corr_id_str = legacy_verbs.split_corr_id(raw)
        corr_id = int(corr_id_str) if corr_id_str else None

        if stripped == "?" or stripped.startswith(_RELAY_CONTROL_PREFIXES):
            return "# ok"

        if stripped == "+":
            try:
                self._conn.send_fast("+")
            except Exception:
                pass
            return None

        if stripped.startswith("*B"):
            return render.render_err("unsupported", "proxy-is-text-only", corr_id)

        verb, pos, kv = legacy_verbs.tokenize_send_line(stripped)
        if not verb:
            return None

        if verb == "HELLO":
            return self._handle_hello()
        if verb == "HELP":
            return render.render_ok("help", _HELP_TEXT, corr_id)
        if verb in _POSE_OTOS_VERBS:
            if not _POSE_OTOS_BINARY:
                return render.render_err("unsupported", verb, corr_id)
            # 098 lands binary pose/otos arms -- real routing goes here.
        if verb in _ALWAYS_UNSUPPORTED_VERBS or verb.startswith("DEV"):
            return render.render_err("unsupported", verb, corr_id)
        if verb == "SET":
            return self._handle_set(kv, corr_id)
        if verb == "GET":
            return self._handle_get(pos, corr_id)
        if verb == "STREAM":
            return self._handle_stream(pos, corr_id)
        if verb == "SNAP":
            return self._handle_snap()
        if verb == "TLM":
            return self._handle_tlm_oneshot(corr_id)
        if verb in legacy_verbs.BINARY_DISPATCH:
            return self._handle_binary_verb(verb, pos, kv, corr_id)

        return render.render_err("unsupported", verb, corr_id)

    # -- local (non-wire) handlers --------------------------------------

    def _handle_hello(self) -> str:
        # HELLO passthrough can't work -- SerialConnection's reader drops
        # DEVICE: lines (see this module's docstring). Answer locally from
        # the startup-cached DeviceId; if that cache is still empty (robot
        # wasn't answering at start() time), try once more live.
        if not self._device_id.serial and not self._device_id.name:
            fetched = self._fetch_device_id()
            if fetched.serial or fetched.name:
                self._device_id = fetched
        return render.render_device_banner(self._device_id)

    # -- one-arm binary verbs (S/D/T/RT/MOVE/MOVER/ECHO/PING/STOP/ID/VER) --

    def _handle_binary_verb(self, verb: str, pos: list[str], kv: dict[str, str],
                            corr_id: int | None) -> str:
        try:
            env = legacy_verbs.BINARY_DISPATCH[verb](pos, kv)
        except ValueError as exc:
            return render.render_err("badarg", str(exc), corr_id)

        result = self._conn.send_envelope(env, read_timeout=500)
        reply = result.get("reply")
        if reply is None:
            return render.render_err("unknown", "timeout", corr_id)

        which = reply.WhichOneof("body")
        if which == "err":
            return render.render_error(reply.err, corr_id)
        if which == "echo":
            return render.render_ok("echo", reply.echo.payload.decode("utf-8", "replace"), corr_id)
        if which == "id":
            if verb == "VER":
                return render.render_ok("ver", render.render_ver_body(reply.id), corr_id)
            return render.render_id_line(reply.id, corr_id)
        if which == "ok":
            line = render.render_ok_for_verb(verb, pos, kv, reply.ok, corr_id)
            if verb == "STOP":
                self._evt_watcher.clear()
            elif verb in render.EVT_ARMING_VERBS:
                self._evt_watcher.arm(verb, corr_id)
            return line
        return render.render_err("unknown", None, corr_id)

    # -- config (SET/GET) -- reuses NezhaProtocol/protocol.py's own
    # key-target maps rather than reimplementing the fan-out -----------

    def _handle_set(self, kv: dict[str, str], corr_id: int | None) -> str:
        if not kv:
            return render.render_err("badarg", "no key=value pairs", corr_id)
        bad = [k for k in kv if k not in protocol._ALL_SET_KEYS]
        if bad:
            return render.render_err("badkey", bad[0], corr_id)
        try:
            kwargs = {k: float(v) for k, v in kv.items()}
        except ValueError:
            return render.render_err("badarg", "bad value", corr_id)
        applied = self._proto.set_config(**kwargs)
        if applied is None:
            return render.render_err("badarg", "set failed", corr_id)
        body = " ".join(f"{k}={v}" for k, v in applied.items())
        return render.render_ok("set", body, corr_id)

    def _handle_get(self, pos: list[str], corr_id: int | None) -> str:
        requested = tuple(pos) if pos else render.ALL_GET_KEYS
        bad = [k for k in requested if k not in protocol._TARGET_FOR_KEY]
        if bad:
            return render.render_err("badkey", bad[0], corr_id)

        targets = sorted({protocol._TARGET_FOR_KEY[k] for k in requested})
        snapshots: dict[int, Any] = {}
        for target in targets:
            snapshot = self._proto.get_config_binary(target)
            if snapshot is not None:
                snapshots[target] = snapshot

        values: dict[str, float] = {}
        for key in requested:
            snapshot = snapshots.get(protocol._TARGET_FOR_KEY[key])
            if snapshot is None:
                continue
            raw = _raw_config_snapshot_value(key, snapshot)
            if raw is not None:
                values[key] = raw
        return render.render_cfg_line(values, corr_id, keys=requested)

    # -- telemetry (STREAM/SNAP/one-shot TLM) ---------------------------

    def _desired_upstream_period(self) -> int:
        """What the REAL robot's stream period should be right now: the
        client's own ``STREAM n`` if armed, else ``--watch-period`` while an
        ``_EvtWatcher`` watch is pending (internal-only, never forwarded to
        the PTY -- see ``_pump_loop``), else off."""
        if self._client_stream_period > 0:
            return self._client_stream_period
        if self._evt_watcher.pending:
            return self._watch_period
        return 0

    def _set_upstream_stream(self, period: int) -> bool:
        """Must be called with ``_stream_lock`` held."""
        if period == self._last_upstream_period:
            return True
        env = envelope_pb2.CommandEnvelope(
            stream=envelope_pb2.StreamControl(period=period, binary=True))
        result = self._conn.send_envelope(env, read_timeout=300)
        reply = result.get("reply")
        ok = reply is not None and reply.WhichOneof("body") == "ok"
        if ok:
            self._last_upstream_period = period
        return ok

    def _handle_stream(self, pos: list[str], corr_id: int | None) -> str:
        if not pos:
            return render.render_err("badarg", "period", corr_id)
        try:
            requested = int(float(pos[0]))
        except ValueError:
            return render.render_err("badarg", "period", corr_id)
        period = 0 if requested <= 0 else max(_STREAM_FLOOR_MS, requested)
        with self._stream_lock:
            self._client_stream_period = period
            ok = self._set_upstream_stream(self._desired_upstream_period())
        if not ok:
            return render.render_err("unknown", "stream", corr_id)
        return render.render_ok("stream", f"period={period}", corr_id)

    def _snap_binary_frame(self):
        """Arm-wait-disarm-restore, holding ``_stream_lock`` for the whole
        sequence so the pump thread's own period reconciliation can never
        interleave with it (mirrors NezhaProtocol.snap()'s own single-
        caller assumption -- see that method's docstring, 097-003 Decision
        4). Restores whatever period was in effect BEFORE this call, not
        blindly ``stream(0)`` -- a client mid-``STREAM`` session must not
        have its stream silently cancelled by an unrelated ``SNAP``/one-shot
        ``TLM`` in between."""
        with self._stream_lock:
            restore_to = self._desired_upstream_period()
            self._conn.drain_binary_tlm()
            self._set_upstream_stream(_STREAM_FLOOR_MS)
            frames = self._conn.read_binary_tlm(duration=400)
            self._set_upstream_stream(restore_to)
        return frames[0].tlm if frames else None

    def _handle_snap(self) -> str:
        frame = self._snap_binary_frame()
        if frame is None:
            return render.render_err("unknown", "snap-timeout", None)
        return render.render_tlm_line(frame)

    def _handle_tlm_oneshot(self, corr_id: int | None) -> str:
        frame = self._snap_binary_frame()
        if frame is None:
            return render.render_err("unknown", "tlm-timeout", corr_id)
        return render.render_ok("tlm", render.render_tlm_one_shot_body(frame), corr_id)

    # ------------------------------------------------------------------
    # tlm-pump thread -- see this module's own docstring for the full
    # threading-model description.
    # ------------------------------------------------------------------

    def _pump_loop(self) -> None:
        while not self._stop.is_set():
            with self._stream_lock:
                self._set_upstream_stream(self._desired_upstream_period())

            frames = self._conn.read_binary_tlm(duration=_PUMP_DRAIN_MS)
            for reply in frames:
                telemetry = reply.tlm
                evt_line = self._evt_watcher.observe(telemetry.active)
                if evt_line:
                    self._write_pty(evt_line, drop_ok=False)
                if self._client_stream_period > 0:
                    self._write_pty(render.render_tlm_line(telemetry), drop_ok=True)

            # WAIT_BUSY cap-expiry fires even with no fresh frame this pass
            # (e.g. the underlying stream is briefly starved) -- feed the
            # watcher a "no change" observation each pass so the 2s cap is
            # still honored on wall-clock time, not just on frame arrival.
            if not frames and self._evt_watcher.state == _EvtWatcher.WAIT_BUSY:
                evt_line = self._evt_watcher.observe(False)
                if evt_line:
                    self._write_pty(evt_line, drop_ok=False)


def _raw_config_snapshot_value(key: str, snapshot: "envelope_pb2.ConfigSnapshot") -> float | None:
    """Read one ``GET`` key's RAW numeric value out of a ``ConfigSnapshot``.

    Structurally identical to ``protocol.py``'s own
    ``_read_config_snapshot_value()`` (097-002) MINUS that function's final
    ``_format_config_value()`` call (6-significant-digit formatting) --
    this proxy needs the firmware's OWN exact wire format
    (``legacy_render.format_config_value()``, fixed-3-decimal/int/uint per
    key), not ``rogo``'s existing ``SET``/``GET`` CLI convention. A ~15-line
    duplication, cited here rather than silently diverging: if
    ``protocol.py``'s key-target maps ever grow a new key, this function
    must be updated in lockstep (both read the SAME module-level
    ``_DRIVETRAIN_KEYS``/``_MOTOR_PID_KEYS``/``_PLANNER_KEYS`` dicts, so a
    NEW key is automatically covered by the ``if key in ...`` branches
    below -- only a change to how an EXISTING key's value is nested inside
    ``ConfigSnapshot`` would need a matching edit here).
    """
    if key in protocol._DRIVETRAIN_KEYS:
        return getattr(snapshot.drivetrain, protocol._DRIVETRAIN_KEYS[key])
    if key in ("ml", "mr"):
        return snapshot.motor.travel_calib
    if key in protocol._MOTOR_PID_KEYS:
        return getattr(snapshot.motor, protocol._MOTOR_PID_KEYS[key])
    if key in protocol._PLANNER_KEYS:
        return snapshot.planner.min_speed
    if key == "sTimeout":
        return float(snapshot.watchdog)
    return None
