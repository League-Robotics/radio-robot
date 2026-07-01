"""robot_radio.testgui.transport — Transport ABC and Serial/Relay backends.

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

Both concrete backends:
- Start a TLM reader thread on connect() that reads lines from the serial
  connection's TLM/EVT queues, parses them via parse_tlm(), and invokes
  on_telemetry.
- Start a camera-truth polling thread on connect() that calls
  read_camera_pose() for tag 100 and invokes on_truth.  The aprilcam
  dependency is lazy / optional: if the daemon is not available the thread
  logs a warning and delivers None.
- Join all threads on disconnect().

Helper:
    list_ports() -> list[str]
        Enumerate USB modem serial ports (wraps serial_conn.list_serial_ports).
"""

from __future__ import annotations

import abc
import logging
import threading
import time
from typing import Callable

from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
from robot_radio.robot.protocol import TLMFrame, parse_tlm

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
            self._log(f"TX {line}")

        def _on_recv(line: str) -> None:
            self._log(f"RX {line}")

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
