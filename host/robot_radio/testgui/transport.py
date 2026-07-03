"""robot_radio.testgui.transport — Transport ABC and Serial/Relay/Sim backends.

Defines the unified transport interface used by the Robot Test GUI so that
``app.py`` never branches on backend type.

Public surface
--------------
Transport (ABC)
    Abstract base class.  Concrete subclasses implement three groups:

    Lifecycle:
        connect()   — open connection and start background threads.
        disconnect()— stop all threads cleanly; must not hang.

    Commands:
        send(line)           — fire-and-forget (no reply read).
        command(line, read_ms) — send and collect reply lines joined as str.

    Keepalive (sprint 065, ticket 005; default no-ops, not abstract):
        arm_keepalive()    — arm the ambient host "+" keepalive for an
            open-ended (S/VW/R) motion session. connect() no longer arms it
            automatically; the caller that owns the motion session (e.g.
            KeyboardDriver) is responsible. Hardware backends
            (_HardwareTransport) delegate to SerialConnection.start_keepalive();
            SimTransport uses the inherited no-op default (no real serial
            link, no ambient-keepalive concept).
        disarm_keepalive() — disarm it (hardware backends delegate to
            SerialConnection.stop_keepalive()).

    Callbacks (set before connect()):
        on_telemetry: Callable[[TLMFrame], None] | None
            Called from the reader thread for every parsed TLM line.
            Callers that need Qt-thread safety must marshal via
            QMetaObject.invokeMethod or a thread-safe signal.
        on_truth: Callable[[(float, float, float) | None], None] | None
            Called from the camera-truth poller with (x_cm, y_cm, yaw_rad),
            or None when the daemon is not available.
        on_log: Callable[[str], None] | None
            Called for every sent command line and every received line
            (for the log pane).

SerialTransport(port: str)
    Wraps SerialConnection(port, mode="direct").

RelayTransport(port: str)
    Wraps SerialConnection(port, mode="relay") — !GO handshake is handled
    internally by SerialConnection.

Both concrete hardware backends:
- Start a TLM reader thread on connect() that reads lines from the serial
  connection's TLM/EVT queues, parses them via parse_tlm(), and invokes
  on_telemetry.
- Start a camera-truth polling thread on connect() that calls
  read_camera_pose() for tag 100 and invokes on_truth.  The aprilcam
  dependency is lazy / optional: if the daemon is not available the thread
  logs a warning and delivers None.
- Join all threads on disconnect().

SimTransport()
    Drives the ctypes firmware simulator (tests/_infra/sim/firmware.py Sim
    class) instead of real hardware.  Owns a background tick-thread that
    advances sim.tick_for() at wall-clock rate (~20 ms/step), drains
    sim.get_async_evts() for TLM/EVT lines, and delivers ground-truth pose
    from sim_get_true_pose_x/y/h via the on_truth callback.

    Unit conversion: sim true-pose is (x_mm, y_mm, h_rad); on_truth receives
    (x_cm, y_cm, yaw_rad) — x and y are divided by 10; heading is passed
    through unchanged (already radians).

    Before connecting, if the sim lib
    (tests/_infra/sim/build/libfirmware_host.{dylib,so}) is missing, a
    QMessageBox.warning is shown (when Qt is available) and connect() returns
    without connecting.

    A configurable field error profile is applied on connect, loaded via
    ``sim_prefs.load_sim_error_profile()`` (defaults: slip_turn_extra=0.26,
    otos_linear_noise=0.05, encoder_noise_mm=0.0, otos_yaw_noise=0.0 —
    matching the historical sim_field_profile fixture from
    tests/conftest.py). ``apply_error_profile(profile)`` re-applies live to
    a connected sim (the Sim Errors panel's Apply button).

Helpers:
    list_ports() -> list[str]
        Enumerate USB modem serial ports (wraps serial_conn.list_serial_ports).

    find_relay_port(port_list, probe_fn) -> str | None
        Pure, Qt-free relay auto-discovery.  Calls ``probe_fn(port)`` for
        each port in order; returns the first port whose banner contains
        ``"RADIOBRIDGE"``, or ``None``.  Exceptions from ``probe_fn`` are
        caught and the port is skipped.

    _relay_probe_banner(port, timeout_s) -> str | None
        Real I/O probe.  Opens the port with DTR asserted (pyserial default),
        sends ``HELLO`` (re-sent every ~0.4 s within the timeout window) and
        reads until a ``DEVICE:`` announcement line arrives.  Returns the
        banner line, or ``None`` on timeout or any I/O error.  Always closes
        the port before returning.

        A purely passive boot-banner wait (open and listen, never send
        anything) is wrong on two counts, live-verified against a real relay
        (see ``.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md``,
        correction of 2026-06-13 / sprint 036-007): some relays do not reset
        on open, so no banner is ever emitted; and even when a reset does
        happen, a micro:bit's boot time can exceed a short passive window.
        HELLO-classify (send ``HELLO``, read the ``DEVICE:`` reply) is the
        robust, bench-proven method — the same one ``SerialConnection``
        already uses. It is safe to send to a non-relay device too: a robot
        answers ``HELLO`` with its own ``DEVICE:`` banner, which lacks
        ``RADIOBRIDGE``, so ``find_relay_port``'s substring match still
        classifies correctly (skips the port).
"""

from __future__ import annotations

import abc
import logging
import math
import pathlib
import queue
import sys
import threading
import time
from typing import Callable

from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
from robot_radio.robot.protocol import TLMFrame, parse_tlm
from robot_radio.testgui import sim_prefs

_log = logging.getLogger(__name__)

# Type aliases
TruthPose = tuple[float, float, float]
TelemetryCB = Callable[[TLMFrame], None]
TruthCB = Callable[[TruthPose | None], None]
LogCB = Callable[[str], None]

# Camera polling interval and inter-read pause.
_TRUTH_POLL_INTERVAL_S = 0.2   # target pose rate ~5 Hz
_TLM_DRAIN_INTERVAL_S = 0.04   # drain TLM queue every 40 ms (~25 Hz ceiling)
_CAMERA_TAG_ID = 100


def list_ports() -> list[str]:
    """Return a sorted list of USB modem serial ports."""
    return list_serial_ports()


def find_relay_port(
    port_list: list[str],
    probe_fn: "Callable[[str], str | None]",
) -> "str | None":
    """Return the first port in port_list whose banner contains 'RADIOBRIDGE'.

    Iterates over ``port_list`` in order, calling ``probe_fn(port)`` for each
    candidate.  Returns the first port for which ``probe_fn`` returns a string
    containing ``"RADIOBRIDGE"``.  Stops early once a match is found.

    ``probe_fn`` exceptions are caught silently and the port is skipped.
    Returns ``None`` if no match is found or ``port_list`` is empty.

    This function is pure and Qt-free — it can be imported and tested without
    a ``QApplication`` instance.

    Parameters
    ----------
    port_list:
        Ordered list of serial port paths to probe.
    probe_fn:
        Callable that takes a port path and returns the device banner string
        or ``None`` if the port does not announce as a relay (or on error).
    """
    for port in port_list:
        try:
            banner = probe_fn(port)
        except Exception:
            continue
        if banner and "RADIOBRIDGE" in banner:
            return port
    return None


# Interval between HELLO retries within the probe window. The device may
# still be mid-boot when the first HELLO is sent, so we keep re-sending it
# until either a DEVICE: reply arrives or the deadline passes.
_RELAY_PROBE_HELLO_INTERVAL_S = 0.4
# Short settle pause after opening the port and before the first HELLO write.
_RELAY_PROBE_SETTLE_S = 0.15


def _relay_probe_banner(port: str, timeout_s: float = 2.0) -> "str | None":
    """Open port with DTR asserted, HELLO-classify, and return the DEVICE: line.

    A passive boot-banner wait (open and listen, never send anything) is
    wrong on two counts — live-verified against a real relay (see
    ``.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md``,
    correction of 2026-06-13 / sprint 036-007): some relays do not reset on
    open at all, so no banner is ever emitted; and even when a reset does
    happen, a micro:bit's boot time can exceed a short passive window.

    Instead this function HELLO-classifies: after opening (DTR asserted by
    default — do NOT pass ``dtr=False``), it sends ``HELLO\\n`` and reads
    lines until one starts with ``DEVICE:`` or ``timeout_s`` elapses.
    ``HELLO`` is re-sent every ``_RELAY_PROBE_HELLO_INTERVAL_S`` in case the
    first write lands while the device is still mid-boot. This is the same,
    bench-proven method ``SerialConnection`` already uses for the real
    connection handshake.

    ``HELLO`` is safe to send to a non-relay device: a robot answers with its
    own ``DEVICE:`` banner (e.g. ``DEVICE:NEZHA2:robot:tovez:<id>``), which
    lacks ``RADIOBRIDGE``, so ``find_relay_port``'s substring match still
    classifies correctly and skips the port.

    Returns ``None`` on timeout or any I/O / OS error.  Always closes the
    port before returning, regardless of outcome, so that non-relay devices
    probed along the way are not left open.

    Parameters
    ----------
    port:
        Serial port path, e.g. ``/dev/cu.usbmodem21421201``.
    timeout_s:
        Maximum time to wait for the ``DEVICE:`` announcement line.
    """
    import serial  # type: ignore[import]
    ser = None
    try:
        # Short per-read timeout so the loop wakes up often enough to
        # re-send HELLO and re-check the overall deadline — NOT timeout_s,
        # which would let a single blocking readline() eat the whole budget.
        ser = serial.Serial(port, 115200, timeout=0.2)
        ser.reset_input_buffer()
        time.sleep(_RELAY_PROBE_SETTLE_S)

        deadline = time.monotonic() + timeout_s
        next_hello = 0.0  # send immediately on the first iteration

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_hello:
                ser.write(b"HELLO\n")
                ser.flush()
                next_hello = now + _RELAY_PROBE_HELLO_INTERVAL_S

            line = ser.readline().decode("ascii", errors="replace").strip()
            if line.startswith("DEVICE:"):
                return line
        return None
    except Exception:
        return None
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Transport ABC
# ---------------------------------------------------------------------------

class Transport(abc.ABC):
    """Unified transport interface for the Robot Test GUI.

    Subclasses must implement ``connect()``, ``disconnect()``,
    ``send()``, and ``command()``.

    Callback slots (assign before calling ``connect()``):

    ``on_telemetry``
        Called with a ``TLMFrame`` for each parsed TLM line received
        from the robot.  Invoked from a background thread; GUI callers
        must marshal to the Qt main thread.

    ``on_truth``
        Called with ``(x_cm, y_cm, yaw_rad)`` when a camera-truth pose
        is available, or ``None`` when the camera daemon is not present.

    ``on_log``
        Called with a timestamped string for every sent command and
        every received line.  Used to populate the log pane.
    """

    def __init__(self) -> None:
        self.on_telemetry: TelemetryCB | None = None
        self.on_truth: TruthCB | None = None
        self.on_log: LogCB | None = None

    @property
    def turn_scrub_factor(self) -> float:
        """Fractional encoder over-report during turns for this backend.

        Backs the Sim Errors panel's display of the simulator's currently
        injected turn-scrub error (independent of trace display — the
        encoder trace is plotted directly from the firmware's ``encpose=``
        since 068-003).  0.0 = perfect (no scrub).  Hardware backends report
        0.0 until real turn-odometry calibration provides a value; the
        simulator overrides this with its injected ``slip_turn_extra``.
        """
        return 0.0

    # ------------------------------------------------------------------
    # Lifecycle (must be implemented)
    # ------------------------------------------------------------------

    @abc.abstractmethod
    def connect(self) -> None:
        """Open connection and start background threads.

        Must be idempotent — calling connect() on an already-connected
        transport is a no-op.
        """

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Stop all background threads and close the connection.

        Must block until all threads have exited (or timed out).  Must
        not raise even if already disconnected.
        """

    # ------------------------------------------------------------------
    # Commands (must be implemented)
    # ------------------------------------------------------------------

    @abc.abstractmethod
    def send(self, line: str) -> None:
        """Fire-and-forget: write ``line`` to the robot, no reply read."""

    @abc.abstractmethod
    def command(self, line: str, read_ms: int = 200) -> str:
        """Send ``line`` and collect reply lines; return joined as string.

        Returns an empty string on error or timeout.
        """

    # ------------------------------------------------------------------
    # Keepalive arm/disarm (sprint 065, ticket 005)
    # ------------------------------------------------------------------
    #
    # SerialConnection no longer arms its ambient "+" keepalive daemon on
    # connect() -- an ambient keepalive running for the entire lifetime of an
    # open port silently defeats the firmware motion watchdog for any hung
    # host process (see architecture-update.md, sprint 065, item 5).  Instead
    # the daemon is armed/disarmed by whichever layer actually owns an
    # open-ended motion session (e.g. the TestGUI's ``KeyboardDriver``, which
    # arms on drive-session start and disarms once its bounded STOP deadman
    # sequence completes).  These default to no-ops, not abstract methods, so
    # existing subclasses (and any future ones) do not break; ``SimTransport``
    # relies on the no-op default since it has no real serial link and no
    # ambient-keepalive concept -- its watchdog behavior is exercised
    # directly via ``sim_command()`` (tickets 002/003).

    def arm_keepalive(self) -> None:
        """Arm the ambient host keepalive for an open-ended motion session.

        No-op by default.  Hardware backends override this to start the
        underlying ``SerialConnection``'s background ``+`` keepalive thread.
        """

    def disarm_keepalive(self) -> None:
        """Disarm the ambient host keepalive.

        No-op by default.  Hardware backends override this to stop the
        underlying ``SerialConnection``'s background ``+`` keepalive thread.
        """

    # ------------------------------------------------------------------
    # Internal helpers shared across hardware backends
    # ------------------------------------------------------------------

    def _log(self, text: str) -> None:
        """Deliver a timestamped text entry to the log callback."""
        if self.on_log:
            ts = time.strftime("%H:%M:%S")
            try:
                self.on_log(f"[{ts}] {text}")
            except Exception:
                pass

    def _deliver_tlm(self, frame: TLMFrame) -> None:
        """Invoke on_telemetry safely."""
        if self.on_telemetry:
            try:
                self.on_telemetry(frame)
            except Exception:
                _log.exception("on_telemetry callback raised")

    def _deliver_truth(self, pose: TruthPose | None) -> None:
        """Invoke on_truth safely."""
        if self.on_truth:
            try:
                self.on_truth(pose)
            except Exception:
                _log.exception("on_truth callback raised")


# ---------------------------------------------------------------------------
# Shared hardware-backend mixin
# ---------------------------------------------------------------------------

class _HardwareTransport(Transport):
    """Common implementation for SerialTransport and RelayTransport.

    Subclasses must set ``self._mode`` in ``__init__`` before calling
    ``connect()``.
    """

    def __init__(self, port: str, mode: str) -> None:
        super().__init__()
        self._port = port
        self._mode = mode
        self._conn: SerialConnection | None = None

        # Background thread handles
        self._reader_thread: threading.Thread | None = None
        self._truth_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open serial connection and start reader + truth threads."""
        if self._conn is not None and self._conn.is_open:
            return

        self._stop_event.clear()

        # Wire log callbacks through SerialConnection's on_send/on_recv hooks.
        def _on_send(line: str) -> None:
            self._log(f"> {line}")

        def _on_recv(line: str) -> None:
            self._log(f"< {line}")

        self._conn = SerialConnection(
            port=self._port,
            mode=self._mode,
            on_send=_on_send,
            on_recv=_on_recv,
        )
        result = self._conn.connect()
        if "error" in result:
            self._conn = None
            raise ConnectionError(f"SerialConnection.connect failed: {result['error']}")

        # Start TLM reader thread.
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name=f"transport-reader-{self._port}",
            daemon=True,
        )
        self._reader_thread.start()

        # Start camera-truth polling thread.
        self._truth_thread = threading.Thread(
            target=self._truth_loop,
            name=f"transport-truth-{self._port}",
            daemon=True,
        )
        self._truth_thread.start()

    def disconnect(self) -> None:
        """Signal threads to stop, then close the connection."""
        self._stop_event.set()

        for t, name in (
            (self._reader_thread, "reader"),
            (self._truth_thread, "truth"),
        ):
            if t is not None and t.is_alive() and t is not threading.current_thread():
                t.join(timeout=2.0)
                if t.is_alive():
                    _log.warning("transport %s thread did not exit within 2 s", name)

        self._reader_thread = None
        self._truth_thread = None

        if self._conn is not None:
            try:
                self._conn.disconnect()
            except Exception:
                _log.exception("Error during SerialConnection.disconnect")
            self._conn = None

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def send(self, line: str) -> None:
        """Fire-and-forget write to the robot."""
        if self._conn is None or not self._conn.is_open:
            raise ConnectionError("Transport is not connected")
        self._conn.send_fast(line)

    def command(self, line: str, read_ms: int = 200) -> str:
        """Send a command and return collected reply lines as a string."""
        if self._conn is None or not self._conn.is_open:
            return ""
        result = self._conn.send(line, read_ms=read_ms)
        responses = result.get("responses", [])
        return "\n".join(responses)

    # ------------------------------------------------------------------
    # Keepalive arm/disarm
    # ------------------------------------------------------------------

    def arm_keepalive(self) -> None:
        """Start the underlying ``SerialConnection``'s ``+`` keepalive thread."""
        if self._conn is not None:
            self._conn.start_keepalive()

    def disarm_keepalive(self) -> None:
        """Stop the underlying ``SerialConnection``'s ``+`` keepalive thread."""
        if self._conn is not None:
            self._conn.stop_keepalive()

    # ------------------------------------------------------------------
    # Background threads
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        """Drain TLM queue and deliver TLMFrame objects to on_telemetry.

        The SerialConnection already has its own internal reader thread
        that fills the TLM queue.  This thread drains that queue and
        calls parse_tlm() on each line, forwarding results to
        on_telemetry.
        """
        while not self._stop_event.is_set():
            if self._conn is None or not self._conn.is_open:
                break
            try:
                lines = self._conn.read_pending_lines()
            except Exception:
                break

            for raw in lines:
                frame = parse_tlm(raw)
                if frame is not None:
                    self._deliver_tlm(frame)

            # Wait a short interval before draining again.
            self._stop_event.wait(timeout=_TLM_DRAIN_INTERVAL_S)

    def _truth_loop(self) -> None:
        """Poll the aprilcam daemon for ground-truth pose and invoke on_truth.

        The aprilcam dependency is optional.  If the daemon is not
        available (import error, connection error, or RuntimeError from
        read_camera_pose), deliver None to on_truth and log a warning once.
        Then back off and retry periodically so that a daemon that comes
        online later is detected.
        """
        playfield = self._open_playfield()

        while not self._stop_event.is_set():
            if playfield is None:
                # Try to open the playfield on each iteration so late-start
                # daemons are picked up.
                playfield = self._open_playfield()
                if playfield is None:
                    self._deliver_truth(None)
                    self._stop_event.wait(timeout=2.0)
                    continue

            try:
                from robot_radio.testkit.camera import read_camera_pose
                pose = read_camera_pose(playfield, tag_id=_CAMERA_TAG_ID, n=3, timeout=1.5)
                self._deliver_truth(pose)
            except RuntimeError:
                # No tag reading within timeout — not an error per se.
                self._deliver_truth(None)
            except Exception:
                _log.debug("Camera truth read failed", exc_info=True)
                self._deliver_truth(None)
                # Back off and try to reconnect.
                playfield = None

            self._stop_event.wait(timeout=_TRUTH_POLL_INTERVAL_S)

    def _open_playfield(self):
        """Attempt to open a Playfield from the aprilcam daemon.

        Returns a Playfield instance or None if the daemon is not available.
        """
        try:
            from robot_radio.field.playfield import Playfield
            return Playfield.open()
        except ImportError:
            # aprilcam not installed — silent, run without truth.
            return None
        except Exception:
            _log.debug("Could not open aprilcam playfield", exc_info=True)
            return None


# ---------------------------------------------------------------------------
# Concrete backends
# ---------------------------------------------------------------------------

class SerialTransport(_HardwareTransport):
    """Transport backend for a direct USB serial connection to the robot.

    Wraps ``SerialConnection(port, mode="direct")``.

    Parameters
    ----------
    port:
        Serial port path, e.g. ``/dev/cu.usbmodem21431202``.
    """

    def __init__(self, port: str) -> None:
        super().__init__(port=port, mode="direct")


class RelayTransport(_HardwareTransport):
    """Transport backend for a radio relay connection to the robot.

    Wraps ``SerialConnection(port, mode="relay")``.  The relay handshake
    (``!ECHO OFF`` / ``!MODE RAW250`` / ``!GO``) is handled automatically
    by ``SerialConnection.connect()``.

    Parameters
    ----------
    port:
        Serial port of the relay dongle, e.g. ``/dev/cu.usbmodem21421201``.
    """

    def __init__(self, port: str) -> None:
        super().__init__(port=port, mode="relay")


# ---------------------------------------------------------------------------
# Sim lib path helpers
# ---------------------------------------------------------------------------

def _sim_lib_name() -> str:
    """Return the platform-specific sim library filename."""
    return "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"


def _sim_lib_path() -> pathlib.Path:
    """Return the expected path for the firmware host simulation library.

    The library lives at tests/_infra/sim/build/ relative to the repo root.
    This function resolves the path regardless of the current working directory
    by walking up from this file's location.
    """
    # transport.py is at host/robot_radio/testgui/transport.py
    # Repo root is three levels up.
    _here = pathlib.Path(__file__).parent   # testgui/
    _host = _here.parent.parent             # host/
    _repo = _host.parent                    # repo root
    return _repo / "tests" / "_infra" / "sim" / "build" / _sim_lib_name()


# ---------------------------------------------------------------------------
# SimTransport
# ---------------------------------------------------------------------------

# Tick step in milliseconds — how many sim-ms we advance per wall-clock tick.
_SIM_TICK_STEP_MS = 20
# Wall-clock sleep between ticks (real-time pacing at 1.0 speed factor).
_SIM_TICK_SLEEP_S = _SIM_TICK_STEP_MS / 1000.0
# Ground-truth pose delivery rate (~5 Hz to match hardware truth polling).
_SIM_TRUTH_EVERY_N_TICKS = max(1, round(200 / _SIM_TICK_STEP_MS))

# Field-profile error parameters (mirrors tests/conftest.py sim_field_profile).
_SIM_SLIP_TURN_EXTRA = 0.26   # fractional encoder over-report during turns
_SIM_OTOS_LINEAR_NOISE = 0.05  # OTOS linear noise sigma (fraction of arc)

# How long connect() waits for the tick-thread to confirm Sim() construction
# succeeded (or failed) before giving up (CR-15 item 4).  Sim() construction
# is sub-millisecond in every observed run; this is generous headroom against
# a hang, not a steady-state expectation (see architecture-update.md sprint
# 066, Open Question 3).
_SIM_READY_TIMEOUT_S = 5.0


class SimTransport(Transport):
    """Transport backend that drives the ctypes firmware simulator.

    Owns a ``Sim`` instance (from ``tests/_infra/sim/firmware.py``) and a
    daemon tick-thread that advances simulation at wall-clock rate, drains
    ``sim.get_async_evts()`` for TLM/EVT lines, and delivers ground-truth
    pose via ``on_truth``.

    Unit conversion
    ---------------
    Sim true-pose returns ``(x_mm, y_mm, h_rad)``.  The ``on_truth`` callback
    receives ``(x_cm, y_cm, yaw_rad)`` — x and y are divided by 10 to convert
    from mm to cm; heading is passed through unchanged (already in radians).

    Thread safety
    -------------
    The ``Sim`` ctypes object is NOT thread-safe for concurrent ``tick_for()``
    and ``send_command()``.  The tick-thread owns the ``Sim`` exclusively.
    Commands submitted via ``send()`` / ``command()`` are placed in a
    ``queue.Queue``; the tick-thread drains that queue between ticks.
    ``command()`` provides a synchronous reply by pairing each command with a
    ``threading.Event`` and a one-element list for the response.

    Lib build check
    ---------------
    ``connect()`` checks for the sim lib before loading ``Sim``.  If the lib
    is missing, a ``QMessageBox.warning`` is shown (when Qt is available) and
    ``connect()`` returns without connecting.  If Qt is not available, a
    message is emitted via ``on_log`` instead.
    """

    def __init__(self) -> None:
        super().__init__()
        self._sim: "object | None" = None   # Sim instance, owned by tick-thread
        self._tick_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        # Queue items: (line: str, reply_list: list[str] | None, done_event: Event | None)
        # For send() (fire-and-forget): reply_list=None, done_event=None
        # For command() (synchronous): reply_list=[""]*1, done_event=Event
        self._cmd_queue: queue.Queue = queue.Queue()
        self._connected = False
        # Signaled by the tick-thread once Sim() construction has succeeded
        # or definitively failed (CR-15 item 4) — connect() waits on this
        # before reporting connected, so an early command()/send() call can
        # no longer race a not-yet-created Sim.
        self._sim_ready_event: threading.Event = threading.Event()
        # The last error profile actually applied to a running sim (via
        # connect()'s _apply_field_profile or a live apply_error_profile()
        # call) — issue testgui-sim-error-profile-config. None until then.
        self._error_profile: dict | None = None

    @property
    def turn_scrub_factor(self) -> float:
        """The ``slip_turn_extra`` fraction the sim currently injects.

        Reflects, in priority order: the profile actually applied to a
        running sim (``self._error_profile``); else the persisted
        ``sim_prefs`` profile on disk; else the historical hardcoded
        default. Never raises — this must be safe to read without a
        connection (e.g. before Connect is pressed).
        """
        if self._error_profile is not None:
            try:
                return float(
                    self._error_profile.get("slip_turn_extra", _SIM_SLIP_TURN_EXTRA)
                )
            except Exception:
                return _SIM_SLIP_TURN_EXTRA
        try:
            profile = sim_prefs.load_sim_error_profile()
            return float(profile.get("slip_turn_extra", _SIM_SLIP_TURN_EXTRA))
        except Exception:
            return _SIM_SLIP_TURN_EXTRA

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Load the sim lib, configure the error profile, and start the tick-thread.

        If the sim lib is missing, shows a warning and returns without
        connecting.  Idempotent — does nothing if already connected.

        ``_connected`` is set only after the tick-thread confirms ``Sim()``
        construction succeeded — NOT immediately after starting the thread
        (CR-15 item 4).  Before this fix an early ``command()``/``send()``
        call could race a not-yet-created ``Sim`` (or one that failed to
        construct), silently enqueuing commands nothing would ever drain.
        ``connect()`` waits (bounded by ``_SIM_READY_TIMEOUT_S``) on a
        ``threading.Event`` the tick-thread signals right after ``Sim()``
        construction completes, or on its own early-failure paths.
        """
        if self._connected:
            return

        lib_path = _sim_lib_path()
        if not lib_path.exists():
            msg = (
                f"Sim library not found: {lib_path}\n"
                f"Build it with:  python build.py\n"
                f"(run from tests/_infra/sim/)"
            )
            self._log(f"[ERROR] {msg}")
            _log.warning("SimTransport: lib missing at %s", lib_path)
            self._show_build_warning(str(lib_path))
            return

        self._stop_event.clear()
        self._sim_ready_event.clear()
        self._tick_thread = threading.Thread(
            target=self._tick_loop,
            name="sim-tick-thread",
            daemon=True,
        )
        self._tick_thread.start()

        ready = self._sim_ready_event.wait(timeout=_SIM_READY_TIMEOUT_S)
        self._connected = ready and self._sim is not None
        if self._connected:
            self._log("[INFO] SimTransport connected")
        else:
            _log.warning(
                "SimTransport: Sim() construction did not complete (ready=%s)",
                ready,
            )
            self._log("[ERROR] SimTransport failed to connect: simulator did not start")

    def disconnect(self) -> None:
        """Signal the tick-thread to stop and wait for it to exit."""
        self._stop_event.set()

        if self._tick_thread is not None and self._tick_thread.is_alive():
            if self._tick_thread is not threading.current_thread():
                self._tick_thread.join(timeout=3.0)
                if self._tick_thread.is_alive():
                    _log.warning("SimTransport tick-thread did not exit within 3 s")

        self._tick_thread = None
        self._connected = False
        self._sim = None
        self._log("[INFO] SimTransport disconnected")

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def send(self, line: str) -> None:
        """Fire-and-forget: enqueue a command for the tick-thread to execute."""
        if not self._connected:
            raise ConnectionError("SimTransport is not connected")
        self._cmd_queue.put((line, None, None))
        self._log(f"> {line}")

    def set_true_pose(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
        """Teleport the simulator plant (ground-truth) pose.

        In Sim mode the canvas avatar follows the plant ground truth
        (``sim.get_true_pose()``), NOT the firmware's belief.  The wire
        commands ``OZ`` and ``SI`` only reset the firmware's EKF estimate and
        the OTOS reference — they do NOT move the plant.  On real hardware the
        operator physically places the robot; in the sim there is no operator,
        so the plant must be teleported explicitly or the avatar snaps back to
        the plant's stale pose on the next ground-truth delivery.

        Enqueues a plant action on the tick-thread (the only thread allowed to
        touch the ``Sim`` object).  True wheel travel and velocity are also
        zeroed so encoder-based odometry restarts from a clean state.

        Parameters
        ----------
        x_cm, y_cm:
            Target plant position in centimetres (converted to mm for the sim).
        yaw_rad:
            Target plant heading in radians (0 = east), passed through as-is.
        """
        if not self._connected:
            return

        def _action(sim: "object") -> None:
            sim.set_true_pose(x_cm * 10.0, y_cm * 10.0, yaw_rad)  # type: ignore[attr-defined]
            # Zero true wheel travel and velocity so odom restarts clean.
            try:
                sim.set_true_wheel_travel(0.0, 0.0)  # type: ignore[attr-defined]
                sim.set_true_velocity(0.0, 0.0)      # type: ignore[attr-defined]
            except Exception:
                pass

        self._cmd_queue.put((_action, None, None))
        self._log(f"> [sim] set_true_pose({x_cm:.1f}cm, {y_cm:.1f}cm, {math.degrees(yaw_rad):.1f}°)")

    def command(self, line: str, read_ms: int = 200) -> str:
        """Send a command and return the synchronous reply string.

        Enqueues the command with a ``threading.Event`` and waits for the
        tick-thread to process it.  Timeout is derived from ``read_ms``.
        Returns an empty string on timeout or when not connected.  The
        reply is logged (and, if it parses as TLM, delivered to
        on_telemetry) by ``_drain_cmd_queue`` on the tick-thread — the one
        place both this method and ``send()`` funnel through — not here.
        """
        if not self._connected:
            return ""
        reply_list: list[str] = [""]
        done_evt = threading.Event()
        self._cmd_queue.put((line, reply_list, done_evt))
        self._log(f"> {line}")
        timeout_s = max(read_ms / 1000.0, 1.0)
        done_evt.wait(timeout=timeout_s)
        return reply_list[0]

    # ------------------------------------------------------------------
    # Background tick-thread
    # ------------------------------------------------------------------

    def _tick_loop(self) -> None:
        """Advance the sim at wall-clock rate; drain commands and async events.

        This is the only thread that touches the Sim object.  On entry it
        creates the Sim, configures the field-error profile, and sends
        ``STREAM 50`` to start TLM streaming.  On exit it destroys the Sim.
        """
        # Import here — only loaded after the lib is verified present.
        try:
            import sys as _sys
            _SIM_DIR = str(_sim_lib_path().parent.parent)
            if _SIM_DIR not in _sys.path:
                _sys.path.insert(0, _SIM_DIR)
            from firmware import Sim  # type: ignore[import]
        except Exception as exc:
            _log.error("SimTransport: failed to import Sim: %s", exc)
            self._log(f"[ERROR] Failed to load simulator: {exc}")
            self._connected = False
            self._sim_ready_event.set()  # unblock connect()'s wait — failed
            return

        try:
            with Sim() as sim:
                self._sim = sim
                # Sim() construction succeeded — unblock connect()'s wait.
                self._sim_ready_event.set()
                self._apply_field_profile(sim)
                # Send STREAM 50 so the firmware emits TLM every 50 ms.
                reply = sim.send_command("STREAM 50")
                self._log(f"[INFO] STREAM 50 → {reply.strip() if reply else 'OK'}")

                tick_count = 0
                while not self._stop_event.is_set():
                    t0 = time.monotonic()

                    # Drain commands from the queue.
                    self._drain_cmd_queue(sim)

                    # Advance simulation by one step.
                    sim.tick_for(_SIM_TICK_STEP_MS, step_ms=_SIM_TICK_STEP_MS)

                    # Drain async events (TLM/EVT lines) from the sim.
                    self._drain_async_evts(sim)

                    # Deliver ground-truth pose periodically.
                    tick_count += 1
                    if tick_count % _SIM_TRUTH_EVERY_N_TICKS == 0:
                        self._deliver_sim_truth(sim)

                    # Pace to wall-clock rate.
                    elapsed = time.monotonic() - t0
                    sleep_s = _SIM_TICK_SLEEP_S - elapsed
                    if sleep_s > 0:
                        self._stop_event.wait(timeout=sleep_s)
        except Exception as exc:
            _log.error("SimTransport tick-loop crashed: %s", exc)
            self._log(f"[ERROR] Sim tick-loop crashed: {exc}")
            # Sim() construction itself may have raised (before the inner
            # self._sim_ready_event.set() ran) — unblock connect()'s wait
            # either way so a failed construction is never mistaken for a
            # hang. Idempotent if already set.
            self._sim_ready_event.set()
        finally:
            self._sim = None

    def _drain_cmd_queue(self, sim: "object") -> None:
        """Drain all pending commands from the queue and execute them on sim.

        ``sim.send_command()`` returns its reply synchronously — unlike real
        hardware, where every wire reply flows through one shared reader
        regardless of whether the outbound side was ``send()`` (fire-and-
        forget) or ``command()`` (synchronous).  Logging and TLM-delivery
        happen here, for both call paths, so a fire-and-forget reply (e.g.
        ``SNAP``, used by idle-detection) is as visible in the console and
        reaches ``on_telemetry`` the same as a ``command()`` reply does —
        previously the fire-and-forget path silently discarded it.
        """
        # Import Sim type for isinstance check would be circular; use duck-typing.
        try:
            while True:
                item = self._cmd_queue.get_nowait()
                line, reply_list, done_evt = item
                try:
                    if callable(line):
                        # Plant action (e.g. set_true_pose) — run directly on
                        # the tick-thread which exclusively owns the Sim object.
                        line(sim)
                        reply = ""
                    else:
                        reply = sim.send_command(line)  # type: ignore[attr-defined]
                except Exception as exc:
                    reply = f"ERR sim: {exc}"
                if reply_list is not None:
                    reply_list[0] = reply
                if done_evt is not None:
                    done_evt.set()
                for raw_line in reply.split("\n"):
                    raw_line = raw_line.strip()
                    if not raw_line:
                        continue
                    self._log(f"< {raw_line}")
                    frame = parse_tlm(raw_line)
                    if frame is not None:
                        self._deliver_tlm(frame)
        except queue.Empty:
            pass

    def _drain_async_evts(self, sim: "object") -> None:
        """Drain accumulated async output, log every line, and deliver TLM frames.

        Mirrors ``_HardwareTransport``'s ``on_recv`` hook, which logs every
        raw wire line unconditionally.  The background ``STREAM 50`` started
        in ``connect()`` is what actually feeds the canvas pose trace, and it
        must be visible in the console like any other traffic — previously
        only the subset that happened to parse as TLM was even processed, and
        nothing from this path was ever logged, making the trace's data
        source invisible.
        """
        try:
            raw = sim.get_async_evts()  # type: ignore[attr-defined]
        except Exception:
            return
        if not raw:
            return
        for line in raw.split("\n"):
            line = line.strip()
            if not line:
                continue
            self._log(f"< {line}")
            frame = parse_tlm(line)
            if frame is not None:
                self._deliver_tlm(frame)

    def _deliver_sim_truth(self, sim: "object") -> None:
        """Read ground-truth pose from the sim and deliver to on_truth callback.

        Converts from simulator units (x_mm, y_mm, h_rad) to the callback
        convention (x_cm, y_cm, yaw_rad): x and y are divided by 10.
        Heading is already in radians and is passed through unchanged.
        """
        try:
            x_mm, y_mm, h_rad = sim.get_true_pose()  # type: ignore[attr-defined]
        except Exception:
            return
        x_cm = x_mm / 10.0
        y_cm = y_mm / 10.0
        self._deliver_truth((x_cm, y_cm, h_rad))

    def _apply_field_profile(self, sim: "object") -> None:
        """Load the persisted sim error profile and apply it to ``sim``.

        Called once from the tick-thread right after the ``Sim`` is created
        (before the tick loop starts), so the operator's Sim Errors panel
        settings (persisted via ``sim_prefs``) are live from the first tick.
        Falls back to ``sim_prefs.DEFAULT_PROFILE`` (the historical
        hardcoded 0.26 / 0.05 / 0.0 / 0.0 values) if the file is missing or
        corrupt — ``load_sim_error_profile()`` never raises, but this is
        belt-and-suspenders.
        """
        try:
            profile = sim_prefs.load_sim_error_profile()
        except Exception:
            profile = dict(sim_prefs.DEFAULT_PROFILE)
        self._apply_profile_to_sim(sim, profile)

    def _apply_profile_to_sim(self, sim: "object", profile: dict) -> None:
        """Apply all four sim error knobs in ``profile`` to ``sim``.

        Mirrors the ``sim_field_profile`` fixture from tests/conftest.py for
        the two historical knobs (turn-slip over-report, OTOS linear noise)
        and additionally wires the two previously-unused knobs (per-side
        encoder noise, OTOS yaw noise) — issue
        testgui-sim-error-profile-config.

        Each knob is applied in its own try/except so a missing sim method
        (e.g. a stale prebuilt lib without ``sim_set_encoder_noise``) never
        prevents the other three from applying. Stores the (attempted)
        profile on ``self._error_profile`` so ``turn_scrub_factor`` reflects
        it regardless of which individual knobs actually landed.
        """
        defaults = sim_prefs.DEFAULT_PROFILE
        slip_turn_extra = profile.get("slip_turn_extra", defaults["slip_turn_extra"])
        otos_linear_noise = profile.get("otos_linear_noise", defaults["otos_linear_noise"])
        otos_yaw_noise = profile.get("otos_yaw_noise", defaults["otos_yaw_noise"])
        encoder_noise_mm = profile.get("encoder_noise_mm", defaults["encoder_noise_mm"])

        try:
            sim.set_field_profile(  # type: ignore[attr-defined]
                slip_turn_extra=slip_turn_extra,
                fuse_otos=True,
            )
        except Exception as exc:
            _log.warning("SimTransport: could not apply slip_turn_extra: %s", exc)
        try:
            sim.set_otos_linear_noise(otos_linear_noise)  # type: ignore[attr-defined]
        except Exception as exc:
            _log.warning("SimTransport: could not apply otos_linear_noise: %s", exc)
        try:
            sim.set_otos_yaw_noise(otos_yaw_noise)  # type: ignore[attr-defined]
        except Exception as exc:
            _log.warning("SimTransport: could not apply otos_yaw_noise: %s", exc)
        try:
            sim.set_encoder_noise(0, encoder_noise_mm)  # type: ignore[attr-defined]
            sim.set_encoder_noise(1, encoder_noise_mm)  # type: ignore[attr-defined]
        except Exception as exc:
            _log.warning("SimTransport: could not apply encoder_noise_mm: %s", exc)

        self._error_profile = dict(profile)
        self._log(
            f"[INFO] Sim error profile applied "
            f"(encoder_noise_mm={encoder_noise_mm}, "
            f"slip_turn_extra={slip_turn_extra}, "
            f"otos_linear_noise={otos_linear_noise}, "
            f"otos_yaw_noise={otos_yaw_noise})"
        )

    def apply_error_profile(self, profile: dict) -> None:
        """Apply ``profile`` live to a connected sim (Sim Errors "Apply" button).

        Safe to call from the Qt GUI thread: the actual sim mutation is
        dispatched onto the tick-thread via the same command queue
        ``set_true_pose`` uses, since the tick-thread exclusively owns the
        ``Sim`` object.

        No-ops (after logging a warning) if not connected — there is no
        running sim to mutate. The profile is still persisted separately by
        the caller (``sim_prefs.save_sim_error_profile``) so it takes effect
        on the next Connect regardless.
        """
        if not self._connected or self._sim is None:
            _log.warning(
                "SimTransport.apply_error_profile: not connected, profile not "
                "applied live (will take effect on next Connect)"
            )
            self._log("[WARN] Sim error profile not applied: not connected")
            return

        def _action(sim: "object") -> None:
            self._apply_profile_to_sim(sim, profile)

        self._cmd_queue.put((_action, None, None))

    # ------------------------------------------------------------------
    # Qt warning helper
    # ------------------------------------------------------------------

    @staticmethod
    def _show_build_warning(lib_path: str) -> None:
        """Show a QMessageBox.warning if Qt is available; otherwise log only.

        Qt is optional — the transport module must be importable without
        PySide6.  This method attempts a deferred import and falls back
        silently when PySide6 is not installed.
        """
        try:
            from PySide6.QtWidgets import QApplication, QMessageBox  # type: ignore[import-untyped]
            app = QApplication.instance()
            if app is not None:
                QMessageBox.warning(
                    None,
                    "Build required",
                    f"Simulator library not found:\n{lib_path}\n\n"
                    "Run:  python build.py\n"
                    "(from tests/_infra/sim/)",
                )
        except ImportError:
            # PySide6 not installed — warning was already emitted via on_log.
            pass
        except Exception as exc:
            _log.debug("SimTransport: could not show QMessageBox: %s", exc)
