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
        command(line, read_timeout) — send and collect reply lines joined as str.

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
- Start a TLM reader thread on connect() that drains the serial
  connection's binary telemetry queue (097-003), adapts each frame via
  TLMFrame.from_pb2(), and invokes on_telemetry.
- Start a camera-truth polling thread on connect() that calls
  read_camera_pose() for tag 100 and invokes on_truth.  The aprilcam
  dependency is lazy / optional: if the daemon is not available the thread
  logs a warning and delivers None.
- Join all threads on disconnect().

SimTransport()
    108-007: rewired onto ``robot_radio.io.sim_loop.SimLoop`` -- the real,
    19-symbol ``sim_ctypes.cpp`` ABI over ``TestSim::SimHarness``/
    ``TestSim::SimPlant`` (108-005/006), replacing the dead sprint-081/082
    ``SimConnection`` this class used to own (deleted at sprint 102 ticket
    005). ``SimLoop`` IS a ``TwistTransport`` directly (``twist()``/
    ``stop()``/``read_pending_binary_tlm_frames()``), so this class's
    ``.protocol`` property hands ``planner.tour.run_tour()`` the ``SimLoop``
    instance itself -- no adapter, exactly the same shape
    ``_HardwareTransport.protocol`` hands it a live ``NezhaProtocol``. Tour
    buttons are un-gated for Sim as of this ticket (see ``__main__.py``'s
    ``_tour_hw_tooltip()``/``_tour_sim_tooltip()`` and the (removed)
    ``is_sim_transport()`` tour-disable in ``_on_connect()``).

    ``SimLoop`` has NO generic wire/config-channel simulation surface at all
    (unlike ``SimConnection``, which was a drop-in ``SerialConnection``
    substitute) -- so ``send()``/``command()`` on this class cannot route an
    arbitrary text-v2 verb through ``binary_bridge.translate_command()`` the
    way ``_HardwareTransport`` does; SimTransport never touches
    ``binary_bridge`` at all (that module's ``translate_command()`` /
    ``segment``/``replace`` builders stay real-hardware-only -- see
    ``clasi/issues/binary-bridge-segment-replace-arms-deleted.md``).  A
    ``send()``/``command()`` call is accepted, logged, and is a no-op
    (returns ``""`` for ``command()``) -- driving the sim for real happens
    exclusively through ``.protocol``'s ``twist()``/``stop()`` surface (a
    tour, or ``KeyboardDriver``'s direct twist calls).

    Unit conversion: sim true-pose is (x, y, h) in (mm, mm, rad); on_truth receives
    (x_cm, y_cm, yaw_rad) — x and y are divided by 10; heading is passed
    through unchanged (already radians).

    Before connecting, if the sim lib
    (src/sim/build/libfirmware_host.{dylib,so}) is missing, a
    QMessageBox.warning is shown (when Qt is available) and connect() returns
    without connecting.

    A configurable field error profile is applied on connect, loaded via
    ``sim_prefs.load_sim_error_profile()`` and applied directly through
    ``SimLoop``'s fault-condition setters named in
    ``sim_prefs.PROFILE_TO_SIM_SETTER`` where a 1:1 mapping exists (108-007:
    the new 19-symbol ABI backs far fewer fault knobs than the deleted
    ~40-symbol ``SimConnection`` one did -- see ``_apply_profile_to_sim()``'s
    own docstring for the full, narrowed mapping and the "not supported in
    this sim" logging path for every profile key that has no ``SimLoop``
    setter). ``apply_error_profile(profile)`` re-applies live to a connected
    sim (the Sim Errors panel's Apply button).

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
import base64
import logging
import math
import pathlib
import sys
import threading
import time
from typing import Any, Callable

from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
from robot_radio.io.sim_loop import SimLoop
from robot_radio.robot import protocol
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame
from robot_radio.testgui import binary_bridge
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


def find_robot_serial_port(candidates: "list[str] | None" = None) -> "str | None":
    """Return the direct-USB ROBOT port for the Serial transport, or None.

    Resolution order (pure and Qt-free, like ``find_relay_port()`` below):

    1. ``config/devices.json`` -- mbdeploy's device registry (``mbdeploy
       probe`` refreshes it) -- entries whose ``role`` is ``NEZHA2``/
       ``ROBOT``, filtered to ports that are actually present right now
       (registry entries go stale as USB re-enumerates), preferring ones in
       ``candidates`` when given.
    2. Fallback for a registry that is missing/stale: the first candidate
       port that is NOT claimed by a ``RADIOBRIDGE`` registry entry -- so a
       bench with a relay dongle plugged in never auto-picks the relay as
       the robot.

    Parameters
    ----------
    candidates:
        Ordered port paths to choose among (``list_ports()`` when ``None``).
    """
    if candidates is None:
        candidates = list_ports()

    robot_ports: list[str] = []
    relay_ports: set[str] = set()
    registry = pathlib.Path(__file__).resolve().parents[4] / "config" / "devices.json"
    if registry.exists():
        try:
            import json as _json
            for entry in _json.loads(registry.read_text()).values():
                role = (entry.get("role") or "").upper()
                port = entry.get("port")
                if not port:
                    continue
                if role in ("NEZHA2", "ROBOT"):
                    robot_ports.append(port)
                elif role == "RADIOBRIDGE":
                    relay_ports.add(port)
        except Exception:  # noqa: BLE001 -- unreadable registry == no registry
            pass

    # 1. Registry robot ports, existence-checked, candidates-preferred.
    live_robot = [p for p in robot_ports if pathlib.Path(p).exists()]
    for port in candidates:
        if port in live_robot:
            return port
    if live_robot:
        return live_robot[0]

    # 2. First candidate not known to be a relay.
    for port in candidates:
        if port not in relay_ports:
            return port
    return None


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
        simulator overrides this with its injected ``body_rot_scrub``
        (083-001: ``slip_turn_extra`` no longer has a live ctypes effect —
        see ``sim_prefs``'s module docstring).
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
    def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
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
        # NezhaProtocol wrapping self._conn -- constructed on connect(),
        # torn down on disconnect(). command()/send() route every outbound
        # line through binary_bridge.translate_command(self._proto, line),
        # which is what actually builds/sends the binary CommandEnvelope
        # (the firmware's text plane is a 6-verb rump now; S/D/T/RT/SET/
        # GET/STREAM/... have no text form left to send).
        self._proto: NezhaProtocol | None = None

        # Background thread handles
        self._reader_thread: threading.Thread | None = None
        self._truth_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        # 107-003: raised by a caller (testgui's _TourRunner) that needs
        # EXCLUSIVE drain access to the shared binary TLM queue for the
        # duration of a tour -- see suspend_telemetry_reader()'s own
        # docstring for why unmanaged draining starves a tour's own
        # StreamingExecutor of fresh telemetry.
        self._reader_suspended = threading.Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open serial connection and start reader + truth threads."""
        if self._conn is not None and self._conn.is_open:
            return

        self._stop_event.clear()

        # Wire log callbacks through SerialConnection's on_send/on_recv
        # hooks. Both hooks receive the RAW wire line -- for a binary
        # command/reply that is the armored `*B<base64>` line (see
        # io/serial_conn.py's send_envelope()/`_reader_loop()` docstrings);
        # on_recv in particular fires for EVERY decoded line, including the
        # high-rate telemetry push stream, BEFORE any classification.
        # binary_bridge.render_log_line() (097, Goal 4) translates each
        # armored line to readable text, or returns None for a
        # ReplyEnvelope{tlm} push frame -- dropped from the log entirely
        # rather than flooding it with an opaque base64 blob every ~20-50ms.
        def _on_send(line: str) -> None:
            rendered = binary_bridge.render_log_line(line, outbound=True)
            if rendered is not None:
                self._log(f"> {rendered}")

        def _on_recv(line: str) -> None:
            rendered = binary_bridge.render_log_line(line, outbound=False)
            if rendered is not None:
                self._log(f"< {rendered}")

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

        self._proto = NezhaProtocol(self._conn)

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
        self._reader_suspended.clear()

        if self._conn is not None:
            try:
                self._conn.disconnect()
            except Exception:
                _log.exception("Error during SerialConnection.disconnect")
            self._conn = None
        self._proto = None

    # ------------------------------------------------------------------
    # Twist-surface accessor (107-003)
    # ------------------------------------------------------------------

    @property
    def protocol(self) -> NezhaProtocol | None:
        """The already-constructed ``NezhaProtocol`` wrapping this
        transport's connection, or ``None`` if not connected.

        Narrow, read-only accessor -- exposes exactly the slice
        ``planner/executor.py``'s ``TwistTransport`` structural ``Protocol``
        needs (``twist()``/``stop()``/``read_pending_binary_tlm_frames()``);
        a real ``NezhaProtocol`` already satisfies that protocol as-is, no
        adapter needed. Added 107-003 so ``testgui``'s ``_TourRunner`` can
        drive ``planner.tour.run_tour()`` directly against the live wire
        instead of routing each tour step through ``binary_bridge.
        translate_command()``'s dead ``segment``/``replace`` envelope arms
        (see ``planner/tour.py``'s own module docstring for that history).
        """
        return self._proto

    def suspend_telemetry_reader(self) -> None:
        """Pause ``_reader_loop()``'s drain of the shared binary TLM queue.

        107-003 (architecture-update.md Step 7, Open Question 1):
        ``_reader_loop()`` and ``NezhaProtocol.read_pending_binary_tlm_frames()``
        (called by ``planner.executor.StreamingExecutor`` during a tour) both
        ultimately drain the SAME underlying ``SerialConnection.
        _binary_tlm_queue`` -- one non-replayable queue, two independent
        consumers. ``_reader_loop()`` polls every ``_TLM_DRAIN_INTERVAL_S``
        (40ms), far faster than the executor's own ``streaming_interval``-
        paced ``tick()`` (~150ms default), so left unmanaged it wins almost
        every frame -- starving the executor's heading-feedback/fault-bit/
        overshoot checks of fresh telemetry for nearly the whole tour
        (confirmed on the bench, this ticket's own investigation).

        A caller (``testgui``'s ``_TourRunner``) calls this before handing
        ``self.protocol`` to ``run_tour()``, becoming the queue's SOLE
        consumer for the run's duration, and forwards each frame it drains
        back through ``on_telemetry`` itself (via ``run_tour()``'s own
        ``row_callback`` hook) so the canvas/avatar keeps tracking while
        ``_reader_loop()`` stands down. Pairs with
        ``resume_telemetry_reader()``, which the caller invokes in a
        ``finally``. Idempotent; safe to call whether or not a reader thread
        is currently running.
        """
        self._reader_suspended.set()

    def resume_telemetry_reader(self) -> None:
        """Undo ``suspend_telemetry_reader()`` -- ``_reader_loop()`` resumes
        draining the shared binary TLM queue on its own. Idempotent."""
        self._reader_suspended.clear()

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def send(self, line: str) -> None:
        """Fire-and-forget: translate ``line`` to binary and dispatch it.

        097: the firmware's text plane is a 6-verb rump now -- every
        motion/config line is translated to a binary ``CommandEnvelope``
        via ``binary_bridge.translate_command()`` (which performs its own
        request/reply round trip for a supported verb, or sends nothing at
        all for a verb with no binary arm yet). The reply string is
        discarded here (fire-and-forget contract), matching the
        ``send_fast()`` pre-migration behavior's "no reply read" shape --
        it is still logged, via ``translate_command``'s own
        ``send_envelope()``/``NezhaProtocol`` calls invoking
        ``SerialConnection``'s ``on_send``/``on_recv`` hooks the same way
        ``command()`` does.
        """
        if self._conn is None or not self._conn.is_open or self._proto is None:
            raise ConnectionError("Transport is not connected")
        binary_bridge.translate_command(self._proto, line)

    def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
        """Translate ``line`` to binary, send it, and return the rendered
        text-v2 reply line (097: see ``binary_bridge.translate_command()``).
        """
        if self._conn is None or not self._conn.is_open or self._proto is None:
            return ""
        return binary_bridge.translate_command(self._proto, line)

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
        """Drain the binary TLM queue and deliver TLMFrame objects to on_telemetry.

        097-003: telemetry is binary-only now (``NezhaProtocol.stream()``/
        ``.snap()``, whichever a caller uses to arm it, always send
        ``StreamControl{binary: true}``). The SerialConnection already has
        its own internal reader thread that fills ``_binary_tlm_queue``;
        this thread drains that queue (``drain_binary_tlm()``) and adapts
        each frame via ``TLMFrame.from_pb2()``, forwarding results to
        on_telemetry.
        """
        while not self._stop_event.is_set():
            if self._conn is None or not self._conn.is_open:
                break
            if self._reader_suspended.is_set():
                # 107-003: a tour owns the shared binary TLM queue right now
                # (see suspend_telemetry_reader()'s own docstring) -- skip
                # this iteration's drain entirely. Draining-and-discarding
                # would still steal frames from the tour's own executor, so
                # this thread must not touch the queue at all while
                # suspended.
                self._stop_event.wait(timeout=_TLM_DRAIN_INTERVAL_S)
                continue
            try:
                replies = self._conn.drain_binary_tlm()
            except Exception:
                break

            for reply in replies:
                self._deliver_tlm(TLMFrame.from_pb2(reply.tlm))

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


# Hot-reload override: the Test buttons rebuild the sim lib and load a FRESH
# copy at a unique path (dlopen caches by path, so the canonical path would
# return the stale already-mapped image). When set, connect() loads this
# instead of the canonical build path. See __main__.py's _run_sim_test().
_SIM_LIB_OVERRIDE: "pathlib.Path | None" = None


def set_sim_lib_override(path: "pathlib.Path | None") -> None:
    """Point the next SimTransport.connect() at `path` (a fresh dylib copy) for
    hot-reload, or clear the override with None."""
    global _SIM_LIB_OVERRIDE
    _SIM_LIB_OVERRIDE = path


def _sim_lib_path() -> pathlib.Path:
    """Return the path for the firmware host simulation library.

    Normally src/sim/build/ relative to the repo root, resolved from this
    file's location regardless of cwd. If a hot-reload override is set
    (set_sim_lib_override), that fresh-copy path is returned instead.
    """
    if _SIM_LIB_OVERRIDE is not None:
        return _SIM_LIB_OVERRIDE
    # transport.py is at src/host/robot_radio/testgui/transport.py
    # Repo root is four levels up.
    _here = pathlib.Path(__file__).parent   # testgui/
    _src = _here.parent.parent.parent       # src/
    _repo = _src.parent                     # repo root
    return _repo / "src" / "sim" / "build" / _sim_lib_name()


# ---------------------------------------------------------------------------
# SimTransport
# ---------------------------------------------------------------------------

# Tick step in milliseconds — the sim integration granularity.  At speed
# factor N the tick-thread advances N of these steps per wall-clock tick,
# so the physics step size (and firmware control tick) is identical at
# every speed — only wall-clock pacing changes.
_SIM_TICK_STEP_DURATION = 20  # [ms]
# Wall-clock sleep between ticks (real-time pacing at 1x speed factor).
_SIM_TICK_SLEEP_S = _SIM_TICK_STEP_DURATION / 1000.0
# Speed-factor bounds for set_speed_factor().  20x with STREAM 50 means
# ~400 TLM lines/s wall into the log pane — busy but workable; anything
# beyond that has no operator value.
_SIM_SPEED_MIN = 1
_SIM_SPEED_MAX = 20
# Ground-truth pose delivery rate (~5 Hz to match hardware truth polling).
_SIM_TRUTH_EVERY_N_TICKS = max(1, round(200 / _SIM_TICK_STEP_DURATION))

# How long connect() waits for SimLoop.connect() to load the lib and boot
# the sim before giving up. SimLoop.connect() itself is a synchronous ctypes
# call (sub-millisecond) -- this bound only protects against a genuinely
# wedged/misbehaving lib, not a steady-state expectation.
_SIM_READY_TIMEOUT_S = 5.0


# ---------------------------------------------------------------------------
# Sim-mode config path (109-002, Architecture Revision 1): SET/GET-shaped
# host needs route through typed ConfigDelta patches with REAL firmware
# consumers, constructed via the SAME NezhaProtocol.config()/wait_for_ack()
# hardware transports already use -- never through binary_bridge.
# translate_command() (a universal dead stub on every transport since
# legacy_render/legacy_verbs were deleted, see Architecture Revision 1).
# ---------------------------------------------------------------------------

# The ConfigDelta patch kinds RobotLoop::handleConfig() applies live
# (src/firm/app/robot_loop.cpp's handleConfig(): MOTOR, OTOS, and, as of
# 109-008, PLANNER -- every other patch_kind (DRIVETRAIN/WATCHDOG/NONE)
# still replies ACK_STATUS_ERR/ERR_UNIMPLEMENTED unconditionally, a
# documented scope boundary, not an oversight, per src/firm/app/DESIGN.md
# §3). Keys landing on MotorConfigPatch (pid.*/ml/mr) or PlannerConfigPatch
# (minSpeed/headingKp/headingKd) have a real consumer; DrivetrainConfigPatch's
# tw/rotSlip/ekfQ*/ekfR* and the bare watchdog sTimeout arm do not, on ANY
# transport, this sprint. This table is reused (not re-derived) from
# protocol.py's own key-vocabulary -- see that module's "Config key <->
# binary target/field mapping" header comment for the authoritative per-key
# target list.
_CONFIG_MOTOR_KEYS = frozenset(protocol._MOTOR_PID_KEYS) | {"ml", "mr"}
_CONFIG_PLANNER_KEYS = frozenset(protocol._PLANNER_KEYS)
_CONFIG_SUPPORTED_KEYS = _CONFIG_MOTOR_KEYS | _CONFIG_PLANNER_KEYS
_CONFIG_UNSUPPORTED_KEYS = frozenset(protocol._ALL_SET_KEYS) - _CONFIG_SUPPORTED_KEYS


class _SimConfigConn:
    """Duck-typed ``SerialConnection`` substitute so ``NezhaProtocol.
    config()`` can be reused VERBATIM against a ``SimLoop`` -- Architecture
    Revision 1's "one mechanism, not a Sim-specific fork": the exact same
    envelope-building/key-vocabulary code hardware transports use, just
    injected via ``SimLoop.inject_command()`` instead of a live serial
    write.

    Implements only ``send_envelope_fast()`` -- the one method
    ``NezhaProtocol.config()`` calls on ``self._conn`` (duck-typed, no
    ``isinstance`` check inside it). Deliberately does NOT implement
    ``wait_for_ack()``: ``NezhaProtocol.wait_for_ack()`` unconditionally
    re-wraps whatever ``self._conn.wait_for_ack()`` returns via
    ``AckEntry.from_pb2()``, which expects a RAW ``telemetry_pb2.AckEntry``
    (``.status``/``.corr_id``/``.err_code``) -- but ``SimLoop.
    read_pending_binary_tlm_frames()`` already returns adapted ``TLMFrame``/
    ``AckEntry`` dataclasses, one layer past that raw shape. Correlating the
    ack ring is this class's OWN job instead (``poll_ack()`` below), called
    directly by ``SimTransport`` rather than through
    ``NezhaProtocol.wait_for_ack()``.
    """

    def __init__(self, loop: SimLoop) -> None:
        self._loop = loop
        self._corr_counter = 0

    def send_envelope_fast(self, envelope: "envelope_pb2.CommandEnvelope") -> int:
        """Assign a corr_id (own counter -- this adapter is the only sender
        on this path, so no cross-source collision risk the way hardware's
        shared ``_corr_counter`` guards against), armor, and inject via
        ``SimLoop.inject_command()`` -- the exact ``*B<base64>`` shape
        ``SerialConnection.send_envelope_fast()`` writes to a real serial
        port (see that method's own docstring), minus the trailing
        newline framing a live serial stream needs and a direct
        ``inject_command()`` call does not (``FakeTransport::
        enqueueInbound()`` takes one already-delimited line per call)."""
        self._corr_counter += 1
        corr_id = self._corr_counter
        envelope.corr_id = corr_id
        armored = base64.b64encode(envelope.SerializeToString()).decode("ascii")
        self._loop.inject_command(f"*B{armored}")
        return corr_id

    def poll_ack(self, corr_id: int, timeout: int = 500,  # [ms]
                ) -> "protocol.AckEntry | None":
        """Poll ``SimLoop.read_pending_binary_tlm_frames()``'s ack ring for
        ``corr_id``, mirroring ``SerialConnection.wait_for_ack()``'s own
        re-delivery-tolerant matching (returns on the FIRST frame carrying a
        match) -- a small, Sim-local reimplementation rather than an import
        of that method's private ``_match_ack_in_frames()`` helper, since
        that helper matches against raw ``pb2.ReplyEnvelope`` objects
        (``reply.tlm.acks``) off ``drain_binary_tlm()``, not the already-
        adapted ``TLMFrame``/``AckEntry`` dataclasses ``SimLoop.
        read_pending_binary_tlm_frames()`` returns."""
        deadline = time.monotonic() + (timeout / 1000.0)
        while True:
            for frame in self._loop.read_pending_binary_tlm_frames():
                if not frame.acks:
                    continue
                for ack in frame.acks:
                    if ack.corr_id == corr_id:
                        return ack
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.01)


class SimTransport(Transport):
    """Transport backend that drives the real compiled firmware simulator
    (``src/sim/build/libfirmware_host.{dylib,so}``) via
    ``robot_radio.io.sim_loop.SimLoop`` (108-007 rewire -- see this class's
    own module-docstring entry above for the full reconciliation from the
    deleted ``SimConnection``).

    Ownership / thread safety
    --------------------------
    ``self._loop`` (a ``SimLoop``) is constructed and connected directly on
    the GUI/caller thread in ``connect()`` -- ``SimLoop.connect()`` itself
    starts and owns its OWN background tick-thread internally (see that
    class's own docstring), so this class does not need a tick-thread of
    its own the way the old ``SimConnection``-owning implementation did.
    ``disconnect()`` tears the ``SimLoop`` down, which joins its tick-thread.

    Lib build check
    ---------------
    ``connect()`` checks for the sim lib before constructing a ``SimLoop``.
    If the lib is missing, a ``QMessageBox.warning`` is shown (when Qt is
    available) and ``connect()`` returns without connecting.  If Qt is not
    available, a message is emitted via ``on_log`` instead.
    """

    def __init__(self) -> None:
        super().__init__()
        self._loop: SimLoop | None = None
        self._connected = False
        # The last error profile actually applied to a running sim (via
        # connect()'s _apply_field_profile or a live apply_error_profile()
        # call) — issue testgui-sim-error-profile-config. None until then.
        self._error_profile: dict | None = None
        # Fast-forward multiple: sim-time advanced per wall-clock tick.
        # Written from the GUI thread via set_speed_factor(); applied to the
        # connected SimLoop via its own set_speed_factor() (read once per
        # SimLoop tick-thread iteration -- a plain int attribute is atomic
        # under the GIL, no lock needed) and re-applied on every connect().
        self._speed_factor: int = 1
        # Background thread driving a direct-motion command (SEG/D/RT --
        # see command()'s own routing) through planner.tour, so command()
        # itself returns promptly instead of blocking the GUI thread for the
        # whole motion. One at a time -- a second direct-motion request
        # while one is in flight is rejected (logged), not queued or
        # interleaved, since two StreamingExecutors driving the same
        # SimLoop concurrently would race its single-consumer command queue.
        self._motion_thread: threading.Thread | None = None
        self._motion_stop_event = threading.Event()
        # 109-002: config path -- constructed in connect(), torn down in
        # disconnect(). ``_config_proto`` is a real ``NezhaProtocol`` wrapping
        # ``_config_conn`` (a ``_SimConfigConn``, see that class's own
        # docstring) so ``config()`` (the envelope-building/key-vocabulary
        # code) is reused verbatim, never reimplemented; ack correlation
        # goes straight through ``_config_conn.poll_ack()`` instead (see
        # that class's own docstring for why). ``_config_echo`` is the
        # host-side GET answer store (Architecture Revision 1: "GET is
        # answered from host-side state... not a new firmware query wire
        # arm") -- wire key -> last formatted value actually acked by the
        # firmware this session.
        self._config_conn: "_SimConfigConn | None" = None
        self._config_proto: NezhaProtocol | None = None
        self._config_echo: dict[str, str] = {}

    @property
    def protocol(self) -> SimLoop | None:
        """The connected ``SimLoop``, or ``None`` if not connected.

        ``SimLoop`` directly satisfies ``planner.executor.TwistTransport``
        (``twist()``/``stop()``/``read_pending_binary_tlm_frames()``) -- no
        adapter needed, mirroring ``_HardwareTransport.protocol``'s own
        contract exactly. This is what un-gates the Sim tour buttons: the
        GUI's ``_TourRunner.run()`` reads ``transport.protocol`` and hands
        it straight to ``planner.tour.run_tour()``.
        """
        return self._loop

    def firmware_version(self) -> "str | None":
        """Version compiled into the loaded sim library, or None if not
        connected -- surfaced in the GUI header so a stale still-running GUI
        (old dylib still mapped after a rebuild) is obvious at a glance."""
        if self._loop is None:
            return None
        try:
            return self._loop.firmware_version()
        except Exception:
            return None

    @property
    def turn_scrub_factor(self) -> float:
        """The ``body_rot_scrub`` factor the sim currently injects.

        108-007: ``SimLoop``'s 19-symbol ABI has NO setter backing
        ``body_rot_scrub`` at all (see ``_apply_profile_to_sim()``'s own
        docstring) -- this always returns the neutral ``1.0`` now (no
        scrub), regardless of any persisted profile value. Kept as a
        property (rather than removed) because ``Transport``'s base class
        declares it and callers (the Sim Errors panel's turn-scrub display)
        still read it unconditionally.
        """
        return 1.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Load the sim lib, construct/connect a ``SimLoop``, and apply the
        persisted error profile.

        If the sim lib is missing, shows a warning and returns without
        connecting.  Idempotent — does nothing if already connected.
        """
        if self._connected:
            return

        lib_path = _sim_lib_path()
        if not lib_path.exists():
            msg = (
                f"Sim library not found: {lib_path}\n"
                f"Build it with:  python build.py\n"
                f"(run from src/sim/)"
            )
            self._log(f"[ERROR] {msg}")
            _log.warning("SimTransport: lib missing at %s", lib_path)
            self._show_build_warning(str(lib_path))
            return

        try:
            profile = sim_prefs.load_sim_error_profile()
        except Exception:
            profile = dict(sim_prefs.DEFAULT_PROFILE)
        track_width = profile.get("trackwidth", sim_prefs.DEFAULT_PROFILE["trackwidth"])

        loop = SimLoop(track_width=float(track_width), lib_path=lib_path)
        loop.on_telemetry = self._deliver_tlm
        loop.on_truth = self._on_loop_truth

        try:
            loop.connect()
        except Exception as exc:
            _log.warning("SimTransport: SimLoop.connect() failed: %s", exc)
            self._log(f"[ERROR] SimTransport failed to connect: {exc}")
            return

        loop.set_speed_factor(self._speed_factor)

        self._loop = loop
        self._connected = True
        self._config_conn = _SimConfigConn(loop)
        self._config_proto = NezhaProtocol(self._config_conn)  # type: ignore[arg-type]
        self._config_echo = {}
        self._apply_profile_to_sim(loop, profile)
        self._log("[INFO] SimTransport connected")

    def disconnect(self) -> None:
        """Tear down the connected ``SimLoop`` (joins its tick-thread)."""
        self._motion_stop_event.set()
        if self._motion_thread is not None and self._motion_thread.is_alive():
            self._motion_thread.join(timeout=3.0)
        self._motion_thread = None
        if self._loop is not None:
            try:
                self._loop.disconnect()
            except Exception as exc:  # noqa: BLE001
                _log.warning("SimTransport: SimLoop.disconnect() raised: %s", exc)
        self._loop = None
        self._connected = False
        self._config_conn = None
        self._config_proto = None
        self._config_echo = {}
        self._log("[INFO] SimTransport disconnected")

    def suspend_telemetry_reader(self) -> None:
        """Delegate to the connected ``SimLoop`` -- see that class's own
        docstring; mirrors ``_HardwareTransport.suspend_telemetry_reader()``
        so ``_TourRunner`` can treat every transport identically."""
        if self._loop is not None:
            self._loop.suspend_telemetry_reader()

    def resume_telemetry_reader(self) -> None:
        """Delegate to the connected ``SimLoop`` -- see
        ``suspend_telemetry_reader()``."""
        if self._loop is not None:
            self._loop.resume_telemetry_reader()

    def _on_loop_truth(self, pose: tuple) -> None:
        """Adapt ``SimLoop.on_truth``'s ``(x, y, h)`` in (mm, mm, rad) to
        this class's ``on_truth`` convention: ``(x_cm, y_cm, yaw_rad)``."""
        x, y, h = pose
        self._deliver_truth((x / 10.0, y / 10.0, h))

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    #
    # 108-007 left send()/command() as accepted-and-logged no-ops --
    # ``SimLoop`` has no generic wire/config-channel simulation surface for
    # an arbitrary text-v2 line to translate onto the way
    # ``binary_bridge.translate_command()`` does for real hardware (and
    # neither method calls into that module -- SimTransport's own call
    # graph never reaches binary_bridge's segment/replace builders; see
    # this class's module-docstring entry and
    # clasi/issues/binary-bridge-segment-replace-arms-deleted.md). This
    # follow-up fix (TestGUI Sim command-surface) routes the handful of
    # wire verbs the GUI's OWN buttons actually send through command()/
    # send() -- STOP/X, the Turn buttons' "SEG 0 <cdeg>" pivots, the
    # COMMANDS panel's "D <l> <r> <mm>"/"RT <cdeg>" direct-motion rows, and
    # the pose-reset verbs _set_origin() sends (SI/OZ/ZERO enc) -- onto
    # ``self._loop`` so Sim mode's direct buttons are no longer silent.
    # Every other verb (S/T/R/TURN/G, and anything else) still has no sim
    # backing -- accepted-and-logged, just with a short message instead of
    # the old multi-line essay.

    # Verbs recognized by parse_tour() as-is: "D <l> <r> <mm>" (a straight
    # leg) and "RT <cdeg>" (a relative in-place turn) -- see
    # planner/tour.py's own parse_tour() docstring. Both are direct rows in
    # testgui/commands.py's COMMANDS schema.
    _MOTION_VERBS = ("D", "RT")

    # Unmanaged (direct-twist) motion defaults -- the open-loop primitive path.
    # Matched to the managed path's nominal cruise so the ONLY difference
    # between the two GUI columns is planner-vs-no-planner, not speed.
    _UNMANAGED_SPEED = 150.0      # [mm/s]
    _UNMANAGED_YAW_RATE = 2.0     # [rad/s]

    def send(self, line: str) -> None:
        """Fire-and-forget: routes recognized verbs into the sim; anything
        else is accepted and logged (see class docstring)."""
        if not self._connected:
            raise ConnectionError("SimTransport is not connected")
        self._log(f"> {line}")
        self._dispatch(line)

    def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
        """Routes recognized verbs into the sim. ``SET``/``GET`` (109-002)
        return a real reply string from the config path below (see
        ``_handle_config_set()``/``_handle_config_get()``); every other
        recognized/unrecognized verb is accepted and logged, returning
        ``""`` (see class docstring) -- there is no real reply to wait
        ``read_timeout`` for those."""
        if not self._connected:
            return ""
        self._log(f"> {line}")
        reply = self._dispatch(line)
        return reply if reply is not None else ""

    def _dispatch(self, line: str) -> "str | None":
        """Route one wire-verb ``line`` to whatever sim-side action it maps
        onto. Returns a reply string for verbs that have a real one
        (``SET``/``GET``, 109-002); ``None`` for every fire-and-forget verb
        (unchanged contract for those). Never raises -- a malformed line is
        logged and dropped/replied ``ERR``, same tolerance the real wire has
        for a bad line."""
        tokens = line.split()
        if not tokens:
            return None
        verb = tokens[0].upper()

        if verb in ("STOP", "X"):
            self._sim_stop()
        elif verb == "SEG" and len(tokens) == 3 and tokens[1] == "0":
            # In-place pivot: "SEG 0 <cdeg>" -- arc_length=0 means pure
            # rotation. parse_tour() doesn't know "SEG", but its own
            # "RT <cdeg>" verb is the exact same shape (both carry a signed
            # centidegree turn angle), so translate onto that.
            self._run_motion_async(f"RT {tokens[2]}")
        elif verb in self._MOTION_VERBS:
            self._run_motion_async(line)
        elif verb == "SI":
            self._sim_setpose(tokens)
        elif verb in ("OZ", "ZERO"):
            # OZ re-references the OTOS heading-zero to the robot's CURRENT
            # physical orientation; ZERO clears the encoder integrators.
            # Neither has a well-defined "teleport to origin" meaning on its
            # own (that's SI's job, and _set_origin() already calls
            # set_true_pose(0,0,0) directly before sending these) -- quiet
            # no-op so the caller's sequence doesn't error.
            _log.debug("SimTransport: %s accepted, no-op in sim", verb)
        elif verb == "SET" and len(tokens) == 2 and "=" in tokens[1]:
            return self._handle_config_set(tokens[1])
        elif verb == "GET" and len(tokens) == 2:
            return self._handle_config_get(tokens[1])
        elif verb in ("OL", "OA", "OI"):
            return self._handle_otos_patch(verb, tokens[1:])
        else:
            self._log(f"[INFO] SimTransport: {line!r} not supported in this sim")
        return None

    def _handle_config_set(self, kv: str) -> str:
        """``SET <key>=<value>`` (109-002): route through the SAME typed
        ``ConfigDelta`` patch mechanism hardware transports use
        (``NezhaProtocol.config()``), constructed by ``self._config_proto``
        and injected via ``SimLoop.inject_command()`` (see
        ``_SimConfigConn``). A key with no live firmware consumer
        (``_CONFIG_UNSUPPORTED_KEYS``) gets an explicit, immediate host-side
        "unsupported" error -- NO wire round trip is attempted for it
        (Architecture Revision 1, sprint.md: "no wire round trip, no silent
        no-op, no fabricated success")."""
        key, _, raw_value = kv.partition("=")
        if key not in protocol._ALL_SET_KEYS:
            msg = f"ERR badkey {key}"
            self._log(f"[WARN] SimTransport: {msg}")
            return msg
        if key in _CONFIG_UNSUPPORTED_KEYS:
            msg = (
                f"ERR unsupported {key} -- no live firmware consumer this "
                f"sprint (RobotLoop::handleConfig applies MotorConfigPatch/"
                f"OtosConfigPatch/PlannerConfigPatch only; see sprint 109's "
                f"Architecture Revision 1)"
            )
            self._log(f"[WARN] SimTransport: {msg}")
            return msg
        try:
            value = float(raw_value)
        except ValueError:
            msg = f"ERR badval {kv}"
            self._log(f"[WARN] SimTransport: {msg}")
            return msg

        assert self._config_proto is not None  # only reachable while connected
        assert self._config_conn is not None
        try:
            corr_id = self._config_proto.config(**{key: value})
        except ValueError as exc:
            msg = f"ERR {exc}"
            self._log(f"[WARN] SimTransport: {msg}")
            return msg

        ack = self._config_conn.poll_ack(corr_id, timeout=500)
        if ack is None:
            msg = f"ERR timeout {key}"
            self._log(f"[WARN] SimTransport: {msg}")
            return msg
        if not ack.ok:
            msg = f"ERR nak {key} err_code={ack.err_code}"
            self._log(f"[WARN] SimTransport: {msg}")
            return msg

        formatted = protocol._format_config_value(value)
        self._config_echo[key] = formatted
        msg = f"OK set {key}={formatted}"
        self._log(f"< {msg}")
        return msg

    def _handle_config_get(self, key: str) -> str:
        """``GET <key>`` (109-002): host-side echo of the last value THIS
        session itself pushed via ``SET`` -- never a new firmware query wire
        arm (Architecture Revision 1: "GET is answered from host-side
        state... the host-echo approach is sufficient for what this
        sprint's tests actually need")."""
        if key not in protocol._ALL_SET_KEYS:
            msg = f"ERR badkey {key}"
            self._log(f"[WARN] SimTransport: {msg}")
            return msg
        if key in _CONFIG_UNSUPPORTED_KEYS:
            msg = (
                f"ERR unsupported {key} -- no live firmware consumer this "
                f"sprint (see sprint 109's Architecture Revision 1)"
            )
            self._log(f"[WARN] SimTransport: {msg}")
            return msg
        if key not in self._config_echo:
            msg = f"ERR nodata {key} -- not SET this session"
            self._log(f"[WARN] SimTransport: {msg}")
            return msg
        msg = f"{key}={self._config_echo[key]}"
        self._log(f"< {msg}")
        return msg

    def _handle_otos_patch(self, verb: str, pos: list[str]) -> str:
        """``OL <scale>``/``OA <scale>``/``OI`` (109-004, Architecture
        Revision 1): route through the SAME direct-patch-send mechanism
        hardware transports use (``NezhaProtocol.otos_config()``,
        constructed by ``self._config_proto`` and injected via
        ``SimLoop.inject_command()`` -- see ``_SimConfigConn``), mirroring
        ``_handle_config_set()``'s own shape exactly. Unlike SET/GET, there
        is no unsupported-key gating here -- ``RobotLoop::handleConfig``
        DOES apply ``OtosConfigPatch`` live (see that method's own
        comment), so every one of these three verbs has a real firmware
        consumer."""
        try:
            if verb == "OL":
                if not pos:
                    msg = "ERR badarg OL requires <scale>"
                    self._log(f"[WARN] SimTransport: {msg}")
                    return msg
                kwargs: dict[str, Any] = {"linear_scale": float(pos[0])}
            elif verb == "OA":
                if not pos:
                    msg = "ERR badarg OA requires <scale>"
                    self._log(f"[WARN] SimTransport: {msg}")
                    return msg
                kwargs = {"angular_scale": float(pos[0])}
            else:  # OI
                kwargs = {"init": True}
        except ValueError:
            msg = f"ERR badarg {verb} {' '.join(pos)}"
            self._log(f"[WARN] SimTransport: {msg}")
            return msg

        assert self._config_proto is not None  # only reachable while connected
        assert self._config_conn is not None
        corr_id = self._config_proto.otos_config(**kwargs)

        ack = self._config_conn.poll_ack(corr_id, timeout=500)
        if ack is None:
            msg = f"ERR timeout {verb}"
            self._log(f"[WARN] SimTransport: {msg}")
            return msg
        if not ack.ok:
            msg = f"ERR nak {verb} err_code={ack.err_code}"
            self._log(f"[WARN] SimTransport: {msg}")
            return msg

        msg = f"OK {verb.lower()}"
        self._log(f"< {msg}")
        return msg

    def _sim_stop(self) -> None:
        """STOP/X: halt the sim immediately AND signal any in-flight
        direct-motion thread to abort at its next tick (``run_tour()``'s
        own ``should_stop`` poll -- see ``_run_motion_async()``)."""
        self._motion_stop_event.set()
        if self._loop is not None:
            try:
                self._loop.stop()
            except Exception as exc:  # noqa: BLE001
                _log.warning("SimTransport: loop.stop() raised: %s", exc)

    def _sim_setpose(self, tokens: list) -> None:
        """``SI <x_mm> <y_mm> <h_cdeg>`` -- teleports the plant to that pose
        (wire units per ``robot_radio.robot.sync_pose.pose_to_setpose_line``:
        x/y in mm, heading in centidegrees)."""
        if len(tokens) != 4:
            self._log(f"[WARN] SimTransport: malformed SI line: {' '.join(tokens)!r}")
            return
        try:
            x_mm, y_mm, h_cdeg = (float(t) for t in tokens[1:])
        except ValueError:
            self._log(f"[WARN] SimTransport: malformed SI line: {' '.join(tokens)!r}")
            return
        self.set_true_pose(x_mm / 10.0, y_mm / 10.0, math.radians(h_cdeg / 100.0))

    def run_unmanaged(self, *, distance_mm: float = 0.0, angle_deg: float = 0.0) -> None:
        """UNMANAGED (open-loop) primitive motion: turn the motors on at a
        fixed velocity for exactly the time to cover `distance_mm` OR
        `angle_deg`, then let the firmware deadman stop them. Goes straight
        through `twist` -> `Drive::setTwist` -- NO Motion::Executor / Ruckig
        (the un-managed counterpart to `_run_motion_async`'s D/RT path). A
        single `twist(v, omega, duration_ms)` whose deadman lease IS the motion
        duration: `RobotLoop::handleTwist` arms the deadman for `duration_ms`
        (no max clamp), the motors run that long, then neutralize -- a true
        "motors on for a time, then off" with no host-side timing loop.

        Exactly one of `distance_mm`/`angle_deg` is honored (distance wins if
        both are nonzero)."""
        if self._loop is None:
            return
        if distance_mm != 0.0:
            v_x = math.copysign(self._UNMANAGED_SPEED, distance_mm)
            omega = 0.0
            duration_ms = abs(distance_mm) / self._UNMANAGED_SPEED * 1000.0
            label = f"unmanaged drive {distance_mm:+.0f}mm @ {self._UNMANAGED_SPEED:.0f}mm/s"
        elif angle_deg != 0.0:
            v_x = 0.0
            omega = math.copysign(self._UNMANAGED_YAW_RATE, angle_deg)
            duration_ms = math.radians(abs(angle_deg)) / self._UNMANAGED_YAW_RATE * 1000.0
            label = f"unmanaged turn {angle_deg:+.0f}deg @ {self._UNMANAGED_YAW_RATE:.1f}rad/s"
        else:
            return
        self._motion_stop_event.clear()
        try:
            self._loop.twist(v_x, omega, duration_ms)
        except Exception as exc:  # noqa: BLE001
            self._log(f"[ERROR] SimTransport: {label} failed: {exc}")
            return
        self._log(f"[INFO] SimTransport: {label} (twist v_x={v_x:.0f} omega={omega:.2f}, deadman {duration_ms:.0f}ms)")

    def _run_motion_async(self, wire_step: str) -> None:
        """Drive ``wire_step`` (a single "D .../"RT ..." tour-shaped leg)
        through ``planner.tour.parse_tour()``/``run_tour()`` against
        ``self._loop`` on a background thread, mirroring ``__main__.py``'s
        own ``_TourRunner.run()`` construction (same ``PlannerParams``/
        ``HeadingCorrector`` recipe) -- so a Turn/Drive button press is a
        real profiled motion in Sim, not a silent no-op, without blocking
        the calling (GUI) thread for the motion's whole duration."""
        if self._loop is None:
            return
        if self._motion_thread is not None and self._motion_thread.is_alive():
            self._log(
                f"[WARN] SimTransport: motion already in progress, ignoring {wire_step!r}")
            return

        self._motion_stop_event.clear()
        loop = self._loop

        def _worker() -> None:
            from robot_radio.config.robot_config import get_robot_config
            from robot_radio.planner.heading import HeadingCorrector
            from robot_radio.planner.model import PlannerParams
            from robot_radio.planner.tour import parse_tour, run_tour

            try:
                legs = parse_tour([wire_step])
            except ValueError as exc:
                self._log(f"[ERROR] SimTransport: {exc}")
                return

            params = PlannerParams()
            heading = HeadingCorrector(params, robot_config=get_robot_config())

            try:
                result = run_tour(
                    loop, params, heading, legs,
                    should_stop=self._motion_stop_event.is_set,
                )
            except Exception as exc:  # noqa: BLE001
                self._log(f"[ERROR] SimTransport: {wire_step!r} failed: {exc}")
                return

            outcome = result.legs[0].outcome.value if result.legs else "unknown"
            self._log(f"[INFO] SimTransport: {wire_step!r} -> {outcome}")

        self._motion_thread = threading.Thread(
            target=_worker, name="sim-direct-motion", daemon=True)
        self._motion_thread.start()

    def set_true_pose(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
        """Teleport the sim plant to ``(x_cm, y_cm, yaw_rad)`` --
        ``SimLoop.set_true_pose()``'s own (mm, mm, rad) contract, so x/y are
        scaled by 10 here (same cm->mm convention every other SimTransport
        pose call uses -- see ``_on_loop_truth()``'s own x/10.0 the other
        direction). This is what makes the canvas avatar actually move on
        "Set Robot @ 0,0"/SI in Sim mode -- the avatar follows the plant's
        ground truth, and there is no operator to place the robot the way
        real hardware's reset workflow assumes.
        """
        if not self._connected or self._loop is None:
            return
        try:
            self._loop.set_true_pose(x_cm * 10.0, y_cm * 10.0, yaw_rad)
        except Exception as exc:  # noqa: BLE001
            _log.warning("SimTransport: set_true_pose() raised: %s", exc)
            self._log(f"[ERROR] SimTransport: set_true_pose failed: {exc}")
            return
        self._log(
            f"[INFO] SimTransport: teleported to ({x_cm:.1f}cm, {y_cm:.1f}cm, "
            f"{math.degrees(yaw_rad):.1f}°)"
        )

    def set_pid_enabled(self, enabled: bool) -> None:
        """Enable/disable the firmware velocity PID on both sim motors
        (TestGUI "PID" checkbox -- ``SimLoop.set_pid_enabled()`` ->
        ``sim_set_pid_enabled()`` -> ``NezhaMotor::setPidEnabled()``, both
        ports). Firmware default is enabled; a fresh connect (including the
        Test buttons' rebuild+reconnect) starts back at enabled, so the GUI
        re-applies its checkbox state after every connect. No-op (logged)
        when not connected."""
        if not self._connected or self._loop is None:
            self._log("[WARN] SimTransport: PID toggle ignored -- not connected")
            return
        try:
            self._loop.set_pid_enabled(bool(enabled))
        except Exception as exc:  # noqa: BLE001
            _log.warning("SimTransport: set_pid_enabled() raised: %s", exc)
            self._log(f"[ERROR] SimTransport: PID toggle failed: {exc}")
            return
        self._log(f"[INFO] SimTransport: velocity PID {'ENABLED' if enabled else 'DISABLED'}")

    def set_speed_factor(self, factor: int) -> None:
        """Set the sim fast-forward multiple (1 = real time).

        Clamped to [``_SIM_SPEED_MIN``, ``_SIM_SPEED_MAX``].  Safe to call
        at any time, connected or not -- delegates to the connected
        ``SimLoop.set_speed_factor()`` immediately (takes effect on its
        tick-thread's next iteration) and re-applied fresh on every
        ``connect()``.
        """
        clamped = max(_SIM_SPEED_MIN, min(_SIM_SPEED_MAX, int(factor)))
        if clamped == self._speed_factor:
            return
        self._speed_factor = clamped
        if self._loop is not None:
            self._loop.set_speed_factor(clamped)
        self._log(f"[INFO] Sim speed set to {clamped}x")

    # ------------------------------------------------------------------
    # Sim error profile
    # ------------------------------------------------------------------

    def _apply_profile_to_sim(self, loop: SimLoop, profile: dict) -> None:
        """Apply every sim error knob in ``profile`` to ``loop`` where a
        ``SimLoop`` fault setter actually exists; log a clear "not
        supported in this sim" for every knob that has none.

        108-007: ``sim_ctypes.cpp``'s new 19-symbol ABI backed FAR fewer
        fault-condition knobs than the deleted ~40-symbol ``SimConnection``
        one did -- ``SimLoop`` exposed exactly four fault setters:
        ``set_wheel_disconnected``/``set_wheel_freeze``/
        ``set_wheel_dropout_rate`` (none of which has a ``sim_prefs``
        profile-key equivalent yet -- no GUI control drives them this
        sprint) and ``set_otos_drift(x_drift, y_drift, heading_drift)``.
        109-002 added a fifth: ``set_enc_scale_err(port, fraction)`` --
        see below.

        ONE ``DEFAULT_PROFILE`` pairing maps onto the OTOS surface:
        ``otos_lin_drift``/``otos_yaw_drift`` -> a single
        ``loop.set_otos_drift(otos_lin_drift, 0.0, otos_yaw_drift)`` call
        (the old profile has no separate x/y drift terms -- ``otos_lin_drift``
        is applied to the x term, y left at its neutral 0.0). This is a
        special case -- excluded from ``sim_prefs.PROFILE_TO_SIM_SETTER``
        (still empty; see that module's own docstring) and handled
        explicitly here instead.

        ``enc_scale_err_l``/``enc_scale_err_r`` (109-002): each maps 1:1 onto
        ``loop.set_enc_scale_err(port, fraction)`` -- port 1=left, 2=right,
        matching every other port-keyed knob's convention
        (``set_wheel_disconnected``/``set_wheel_freeze``/
        ``set_wheel_dropout_rate``). Unlike the OTOS pairing above, these are
        two independent single-argument calls, not one combined call.

        ``otos_lin_scale_err``/``otos_ang_scale_err`` (109-007): combined
        into a single ``loop.set_otos_raw_scale_err(linear, angular)`` call
        -- models a physically mis-calibrated OTOS chip (fractional
        over/under-report, 0=perfect); a firmware-pushed OL/OA calibration
        scalar corrects the effect back out (see ``sim_plant.h``'s own
        ``SimPlant::setOtosRawScaleErr()``/``handleOtosWrite()`` comments).

        Every OTHER ``DEFAULT_PROFILE`` key (``encoder_noise``,
        ``slip_turn_extra``, ``otos_linear_noise``, ``otos_yaw_noise``,
        ``body_rot_scrub``, ``body_lin_scrub``, ``motor_offset_l/r``) has NO
        ``SimLoop`` setter at all -- applying is skipped outright, with a
        ``[WARN]`` logged if the profile value is away from that key's
        neutral default (mirrors the deleted implementation's own
        skip-and-warn treatment for its own three unsupported knobs, just
        widened to cover this ABI's narrower fault-knob surface).
        ``trackwidth`` is excluded from that warn loop -- it is applied at
        ``SimLoop`` CONSTRUCTION time (``connect()``'s own
        ``SimLoop(track_width=...)`` call), not live, so a live Apply with a
        changed trackwidth logs an explicit "takes effect on next Connect"
        note instead of a bare "not supported".

        Never raises -- each setter call is wrapped in its own try/except.
        Stores the (attempted) profile on ``self._error_profile`` so
        ``turn_scrub_factor``/diagnostics reflect what was attempted.
        """
        defaults = sim_prefs.DEFAULT_PROFILE

        # -- the one real mapping: otos_lin_drift/otos_yaw_drift -> set_otos_drift --
        lin_drift = profile.get("otos_lin_drift", defaults["otos_lin_drift"])
        yaw_drift = profile.get("otos_yaw_drift", defaults["otos_yaw_drift"])
        try:
            loop.set_otos_drift(float(lin_drift), 0.0, float(yaw_drift))
        except Exception as exc:
            _log.warning(
                "SimTransport: could not apply otos_lin_drift/otos_yaw_drift "
                "via loop.set_otos_drift(): %s", exc,
            )

        # -- enc_scale_err_l/r (109-002): 1:1 onto set_enc_scale_err(port, .) --
        enc_scale_err_l = profile.get("enc_scale_err_l", defaults["enc_scale_err_l"])
        enc_scale_err_r = profile.get("enc_scale_err_r", defaults["enc_scale_err_r"])
        try:
            loop.set_enc_scale_err(1, float(enc_scale_err_l))
            loop.set_enc_scale_err(2, float(enc_scale_err_r))
        except Exception as exc:
            _log.warning(
                "SimTransport: could not apply enc_scale_err_l/r "
                "via loop.set_enc_scale_err(): %s", exc,
            )

        # -- otos_lin_scale_err/otos_ang_scale_err (109-007): combined onto
        # set_otos_raw_scale_err(linear, angular) --
        otos_lin_scale_err = profile.get(
            "otos_lin_scale_err", defaults["otos_lin_scale_err"])
        otos_ang_scale_err = profile.get(
            "otos_ang_scale_err", defaults["otos_ang_scale_err"])
        try:
            loop.set_otos_raw_scale_err(
                float(otos_lin_scale_err), float(otos_ang_scale_err))
        except Exception as exc:
            _log.warning(
                "SimTransport: could not apply otos_lin_scale_err/"
                "otos_ang_scale_err via loop.set_otos_raw_scale_err(): %s", exc,
            )

        # -- trackwidth: construction-time only, not live --
        trackwidth = profile.get("trackwidth", defaults["trackwidth"])
        if trackwidth != defaults["trackwidth"]:
            msg = (
                f"trackwidth={trackwidth} takes effect on the NEXT Connect -- "
                "SimLoop's track_width is fixed at construction (sim_create()), "
                "not live-settable"
            )
            _log.info("SimTransport: %s", msg)
            self._log(f"[INFO] {msg}")

        # -- every other knob: no SimLoop ABI backing at all --
        _unsupported_keys = [
            key for key in defaults
            if key not in ("otos_lin_drift", "otos_yaw_drift", "trackwidth",
                           "enc_scale_err_l", "enc_scale_err_r",
                           "otos_lin_scale_err", "otos_ang_scale_err")
        ]
        for key in _unsupported_keys:
            value = profile.get(key, defaults[key])
            neutral = defaults[key]
            if value != neutral:
                msg = (
                    f"{key}={value} not supported in this sim -- sim_ctypes.cpp's "
                    "19-symbol ABI has no matching fault setter (see "
                    "sim_loop.py's SimLoop docstring and _apply_profile_to_sim()'s "
                    "own docstring for the full, narrowed mapping)"
                )
                _log.warning("SimTransport: %s", msg)
                self._log(f"[WARN] {msg}")

        self._error_profile = dict(profile)
        self._log(
            f"[INFO] Sim error profile applied "
            f"(otos_lin_drift={lin_drift}, otos_yaw_drift={yaw_drift}, "
            f"enc_scale_err_l={enc_scale_err_l}, enc_scale_err_r={enc_scale_err_r})"
        )

    def apply_error_profile(self, profile: dict) -> None:
        """Apply ``profile`` live to a connected sim (Sim Errors "Apply" button).

        No-ops (after logging a warning) if not connected — there is no
        running sim to mutate. The profile is still persisted separately by
        the caller (``sim_prefs.save_sim_error_profile``) so it takes effect
        on the next Connect regardless.
        """
        if not self._connected or self._loop is None:
            _log.warning(
                "SimTransport.apply_error_profile: not connected, profile not "
                "applied live (will take effect on next Connect)"
            )
            self._log("[WARN] Sim error profile not applied: not connected")
            return

        self._apply_profile_to_sim(self._loop, profile)

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
                    "(from src/sim/)",
                )
        except ImportError:
            # PySide6 not installed — warning was already emitted via on_log.
            pass
        except Exception as exc:
            _log.debug("SimTransport: could not show QMessageBox: %s", exc)
