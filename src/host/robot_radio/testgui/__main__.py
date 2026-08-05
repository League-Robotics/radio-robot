"""Entry point for ``python -m robot_radio.testgui``.

Launches the Robot Test GUI main window.  Requires PySide6 (install via
``uv sync --group gui``).

All PySide6 imports are kept inside this module so that the package itself can
be imported without PySide6 present.

Mode indicator (ticket 001)
----------------------------
- A ``QLabel`` with ``objectName="mode_label"`` is placed at the top of the
  right panel, above the playfield canvas.
- It reflects the currently selected transport:
  - "Sim"    → "SIM MODE"     (grey)
  - "Serial" → "BENCH MODE"   (blue)
  - "Relay"  → "PLAYFIELD MODE" (green)
- The pure, Qt-free helper ``transport_name_to_mode_label(name)`` maps a
  transport name to ``(text, stylesheet)`` and can be imported and tested
  without a ``QApplication``.

Wiring (ticket 008)
--------------------
- ``TraceModel`` (traces.py) accumulates four world-cm polylines.
- ``build_canvas()`` (canvas.py) renders the playfield QGraphicsView with
  trace paths and a robot marker (red front / blue back).
- Transport ``on_telemetry`` and ``on_truth`` callbacks are marshalled from
  background threads to the Qt main thread via ``QMetaObject.invokeMethod``
  before touching the TraceModel or canvas.
- The ops panel's ``clear_traces_cb`` is wired to ``TraceModel.clear()``
  followed by ``canvas_ctrl.refresh()``.
- The ops panel's ``refresh_playfield_cb(pixmap, origin_x, origin_y)`` is wired to
  ``canvas_ctrl.set_background(pixmap, origin_x=origin_x, origin_y=origin_y)`` so
  the world→pixel transform updates atomically with the background; world (0,0)
  lands on AprilTag 1 after refresh.

Live-view lifecycle (ticket 003; main-thread bridge + throttle, ticket 009)
-----------------------------------------------------------------------------
In PLAYFIELD MODE (Relay transport) a ``_LiveViewWorker`` runs on a ``QThread``
and streams camera frames at ~9-12 Hz.  ``frame_ready`` is connected to a
``_LiveFrameBridge`` (see :func:`build_live_frame_bridge`) constructed on the
GUI thread — NOT to a bare function — because a ``QueuedConnection`` to a
non-``QObject`` callable is delivered on the *emitting* (worker) thread in
this PySide build, and the worker's tight capture loop never returns to its
own event loop to process it (``testgui-playfield-not-live-updating.md``).
The bridge's ``on_frame`` slot calls ``canvas_ctrl.set_avatar_pose`` on
*every* frame (full worker rate, smooth marker) but converts the BGR ndarray
to a QPixmap and calls ``canvas_ctrl.set_background`` only on a throttled
subset (~3-4 fps) to limit GUI-thread work.

When live-view is active (``_state["live_view_active"] is True``), BOTH
``_TelemetryBridge`` slots leave the avatar marker alone — the camera live-view
worker (``canvas_ctrl.set_avatar_pose``) is the sole owner of the marker in
live view:

- ``on_truth_ready`` skips ``canvas_ctrl.refresh()`` entirely.  The green
  truth trace still accumulates in the TraceModel.
- ``on_frame_ready`` calls ``canvas_ctrl.refresh(update_marker=False)`` so
  the trace paths (including the magenta fused trace) still redraw at the
  faster TLM rate, but ``_update_marker`` is not invoked and the marker is
  not repositioned.

Outside live view (Sim/Serial, or Relay before live-view starts),
``on_frame_ready`` calls ``canvas_ctrl.refresh(avatar_yaw_rad)``, positioning
the marker from the latest pose -- 097: ``avatar_yaw_rad`` (and, via
``CanvasController._update_marker``'s own trace-source preference, the
marker's POSITION too) now prefer the host-side encoder dead-reckoning trace
(``TraceModel.encoder``/``encoder_yaw``) over the fused pose, which stays
pinned at the origin until sprint 098 wires a live ``PoseEstimator``. Falls
back to the fused pose once/if the encoder trace has none yet.

On relay disconnect (or window close) the worker is stopped, the thread joined,
and ``canvas_ctrl.restore_static_background()`` reverts the canvas to the grey
placeholder with the field-centre origin.

Sim and Serial transports do NOT start the live-view worker.

Record / Pause / Stop controls (ticket 005)
---------------------------------------------
Three ``QPushButton`` widgets — ``record_btn``, ``pause_btn``, ``stop_btn`` —
appear below the transport controls on the left panel.  They drive a
``SessionRecorder`` (Qt-free, ``testgui/recorder.py``) that writes every TX
command and every RX response/telemetry line to a JSONL file under
``recordings/``.

The tap point is ``_append_log(text, direction=None)``: any call with
``direction="TX"`` or ``direction="RX"`` is forwarded to the recorder.  GUI
status messages (connect/disconnect notices, etc.) call ``_append_log`` without
a direction and are NOT recorded.

Button enable/disable rules:
- Idle:      Record enabled; Pause and Stop disabled.
- Recording: Record disabled; Pause and Stop enabled.
- Paused:    Record (labelled "Resume") enabled; Pause disabled; Stop enabled.

Camera selection (ticket 063-008)
------------------------------------
A ``QComboBox`` (``objectName="camera_combo"``) sits in the left panel's
``selector_row``, alongside the transport and robot combos (relocated from
the right panel by ticket 075-001), listing cameras reported by the
aprilcam daemon's
``DaemonControl.list_cameras()``.  It is populated best-effort at window
build time — if the daemon is unreachable the combo is left empty; this must
never raise or block window construction.  The initial selection reflects
``camera_prefs.load_camera_pref()`` (falling back to the "3"/Arducam heuristic,
then the first available camera) via ``camera_prefs.select_camera()`` — the
same shared helper used by ``operations.py`` and ``live_view.py`` so all
camera-consuming code paths agree on which camera to use.  Changing the
selection calls ``camera_prefs.save_camera_pref()`` and immediately triggers
``ops_ctrl.trigger_live_grab()`` to refresh the playfield image from the
newly selected camera.
"""

from __future__ import annotations

import logging
import math
import sys
import time

_log = logging.getLogger(__name__)


def transport_name_to_mode_label(name: str) -> tuple[str, str]:
    """Map a transport name to a ``(text, stylesheet)`` pair for the mode label.

    This function is Qt-free and can be imported and tested without a
    ``QApplication`` instance.

    Args:
        name: The transport name as shown in the combo box ("Sim", "Serial",
              "Relay", or any other string).

    Returns:
        A tuple of ``(label_text, stylesheet)`` where ``label_text`` is the
        display string for the mode label and ``stylesheet`` is a CSS-style
        string suitable for ``QLabel.setStyleSheet()``.
    """
    _MAP: dict[str, tuple[str, str]] = {
        "Sim": ("SIM MODE", "color: #808080; font-weight: bold;"),
        "Serial": ("BENCH MODE", "color: #4080ff; font-weight: bold;"),
        "Relay": ("PLAYFIELD MODE", "color: #20c020; font-weight: bold;"),
    }
    return _MAP.get(name, ("UNKNOWN MODE", "color: #ff8000; font-weight: bold;"))


def build_live_frame_bridge(canvas_ctrl) -> object:
    """Build a ``_LiveFrameBridge`` bound to ``canvas_ctrl`` (ticket 063-009).

    Mirrors :func:`robot_radio.testgui.live_view.build_live_view_worker`'s
    factory pattern: PySide6 is imported lazily here so this module stays
    importable without PySide6 present, and the returned object is a real
    ``QObject`` that can be constructed directly in a headless test (pass a
    fake ``canvas_ctrl`` stand-in and call ``.on_frame(...)`` — no
    ``QApplication``-driven event loop or GUI wiring required).

    Root cause this bridges around (see
    ``testgui-playfield-not-live-updating.md`` and the ``_WorkerBridge``
    docstring below): connecting a worker ``QObject``'s signal directly to a
    bare function with ``Qt.ConnectionType.QueuedConnection`` does NOT marshal
    the call onto the GUI thread in this PySide build — with no ``QObject``
    receiver, the "queued" delivery instead runs on the *emitting* (worker)
    thread. The live-view worker's ``run()`` loop never returns to its own
    thread's Qt event loop (a tight ``while not self._stop`` loop with
    ``time.sleep()``), so such deliveries are never processed at all and the
    canvas never repaints. Constructing this bridge on the GUI thread and
    connecting ``frame_ready`` to its *bound-method* slot ensures delivery
    reliably happens on the GUI thread, exactly like ``_RXBridge`` /
    ``_TelemetryBridge`` / ``_WorkerBridge``.

    Stakeholder-decided throttling: the avatar pose (``set_avatar_pose``)
    updates on **every** frame the worker emits (full worker rate, ~9-12 Hz),
    so the marker stays smooth. The background pixmap conversion
    (``_bgr_ndarray_to_pixmap``) and ``set_background`` call are more
    expensive, so they are throttled to roughly 3-4 fps by only running on
    every ``BACKGROUND_THROTTLE_N``-th frame (frame-count modulo, not a
    wall-clock gate — simplest option that meets the target given the
    worker's steady ~9-12 Hz rate). A lower effective background rate under
    load is acceptable; tests assert the ratio invariant, not an exact fps.
    """
    from PySide6.QtCore import QObject, Slot  # type: ignore[import-untyped]

    class _LiveFrameBridge(QObject):
        """Marshals live-view worker frames onto the Qt GUI main thread.

        See :func:`build_live_frame_bridge` for the full rationale. In short:
        a ``QueuedConnection`` to a bare function is delivered on the
        *emitting* thread in this PySide build (the live-view worker never
        returns to its own event loop, so such deliveries are never
        processed — see ``testgui-playfield-not-live-updating.md``). This
        bridge is constructed on the GUI thread, so its bound-method slot
        runs on the GUI thread, exactly like the existing ``_WorkerBridge``.

        Avatar pose updates on every frame (full worker rate); the
        background ``QPixmap`` conversion + ``canvas_ctrl.set_background()``
        call is throttled to ~3-4 fps so as not to burn GUI-thread time on
        every ~80ms tick.
        """

        #: Convert+set the background every Nth frame (~9-12 Hz worker / 3 ≈ 3-4 fps).
        BACKGROUND_THROTTLE_N = 3

        def __init__(self, canvas_ctrl) -> None:
            super().__init__()
            self._canvas_ctrl = canvas_ctrl
            self._frame_count = 0

        @Slot(object, float, float, float, float, float)
        def on_frame(
            self,
            bgr: object,
            origin_x: float,
            origin_y: float,
            tx: float,
            ty: float,
            tyaw: float,
        ) -> None:
            """Main-thread slot: full-rate avatar, throttled background.

            Called via ``QueuedConnection`` from the live-view worker
            thread, but delivered on the GUI thread because this bridge is a
            ``QObject`` constructed there. ``QPixmap`` must be constructed
            here (GUI thread only).

            Parameters
            ----------
            bgr:
                BGR ndarray from the worker.
            origin_x, origin_y:
                A1 origin in cm from the deskewed frame — passed to
                set_background.
            tx, ty:
                Tag-100 world position in cm.
            tyaw:
                Tag-100 world heading in radians.
            """
            # Avatar: every frame, full worker rate (~9-12 Hz) — stakeholder
            # decision, keeps the marker smooth even though the background
            # is throttled.
            self._canvas_ctrl.set_avatar_pose(tx, ty, tyaw)

            # Background: throttled subset (~3-4 fps target). Frame-count
            # modulo is the simplest gate that meets the target at the
            # worker's steady rate; a time.monotonic() gate would be an
            # equally valid implementer's choice per the ticket.
            self._frame_count += 1
            if self._frame_count % self.BACKGROUND_THROTTLE_N != 0:
                return

            from robot_radio.testgui.operations import _bgr_ndarray_to_pixmap
            pm = _bgr_ndarray_to_pixmap(bgr)
            if pm is not None:
                self._canvas_ctrl.set_background(
                    pm, origin_x=origin_x, origin_y=origin_y
                )

    return _LiveFrameBridge(canvas_ctrl)


# When True (set by main() for the real GUI), _build_main_window redirects the
# process's C-level stdout/stderr (fd 1/2) into the console so the sim dylib's
# own cout/cerr -- and the Test-button rebuild output -- appear in TestGUI.
# Left False for headless tests so pytest's own capture is untouched.
_ENABLE_STDOUT_CAPTURE = False


def _install_stdout_capture(bridge) -> None:
    """Redirect process fd 1 and 2 into a pipe; a daemon reader tees each line
    back to the real terminal AND emits it to the GUI console via `bridge.line`
    (a Qt signal marshalled to the main thread). Installs once.

    NB: C++ ``std::cout`` to a pipe is fully buffered -- use ``std::endl`` (or
    ``<< std::flush``) in the sim to see output promptly.
    """
    import os
    import threading

    if getattr(_install_stdout_capture, "_installed", False):
        return
    _install_stdout_capture._installed = True

    saved_out = os.dup(1)   # keep the real terminal so output still shows there
    read_fd, write_fd = os.pipe()
    os.dup2(write_fd, 1)
    os.dup2(write_fd, 2)
    os.close(write_fd)

    def _reader() -> None:
        buf = b""
        while True:
            try:
                chunk = os.read(read_fd, 4096)
            except OSError:
                break
            if not chunk:
                break
            try:
                os.write(saved_out, chunk)   # tee to the launching terminal
            except OSError:
                pass
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                bridge.line.emit(line.decode("utf-8", "replace"))

    threading.Thread(target=_reader, name="sim-stdout-capture", daemon=True).start()


def _build_main_window():  # type: ignore[return]
    """Build and return the main QMainWindow with transport wiring.

    Layout (left-to-right via QSplitter):
    - Left panel: transport selector QComboBox, port QLineEdit, Connect
      button, the Managed/Unmanaged motion preset panel, tour buttons, and
      the Operations panel.
    - Right panel: QGraphicsView playfield canvas with trace paths and robot
      marker (top) and timestamped log pane QPlainTextEdit (bottom).

    The transport selector enables the port QLineEdit when Serial or Relay
    is selected.  Clicking Connect instantiates the selected Transport,
    calls transport.connect(), then sends ``STREAM 50``.  Clicking
    Disconnect calls transport.disconnect().

    The log pane receives all sent and received lines via the transport's
    on_log callback, delivered safely from background threads via
    QMetaObject.invokeMethod.

    128-004: the parameter-field command-row panel this docstring used to
    describe (schema-driven from ``robot_radio.testgui.commands.COMMANDS``,
    one Send button per row) is deleted -- it was already hidden and its
    verbs either had no binary-plane arm at all or were already superseded
    by the Managed/Unmanaged preset panel below (see ``commands.py``'s own
    module docstring).
    """
    # PySide6 imports are intentionally deferred here.
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QPlainTextEdit,
        QPushButton,
        QSpinBox,
        QSplitter,
        QStyle,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )
    from PySide6.QtCore import Qt, QMetaObject, Q_ARG  # type: ignore[import-untyped]

    from robot_radio.testgui.worker_session import WorkerSession
    from robot_radio.testgui.transport import (
        Transport,
        SerialTransport,
        RelayTransport,
        SimTransport,
        find_robot_serial_port,
        list_ports,
        effective_track_width as _effective_track_width,
    )
    from robot_radio.testgui.commands import (
        TOURS,
        goto_distance,
        goto_reached,
        tour_execution,
    )
    from robot_radio.testgui.operations import (
        build_panel as _build_ops_panel,
        build_setpose_command,
        is_sim_transport,
    )
    from robot_radio.testgui import traces as traces_mod
    from robot_radio.testgui.traces import TraceModel
    from robot_radio.testgui.canvas import build_canvas
    from robot_radio.testgui.turn_graphs import TurnGraphPanel
    from robot_radio.testgui.turn_control import TurnControlServer
    from robot_radio.testgui.telemetry_panel import (
        build_telemetry_panel,
        is_telemetry_log_line,
    )
    from robot_radio.testgui.recorder import SessionRecorder, direction_from_marker
    from robot_radio.testgui import camera_prefs
    from robot_radio.testgui import sim_prefs
    from robot_radio.robot.protocol import wheel_frozen_reason

    # QApplication must exist before any QWidget is created.  We create one
    # only if one does not already exist (e.g. during testing).
    app = QApplication.instance() or QApplication(sys.argv)

    # Active transport — kept in a mutable container so inner functions can
    # rebind it without 'nonlocal' limitations across closures.
    # live_view_active: True while a _LiveViewWorker is running (Relay only).
    # live_worker / live_thread: references held for clean shutdown.
    # live_bridge: the _LiveFrameBridge (ticket 063-009) kept alive for the
    # lifetime of the frame_ready connection — a dropped reference would
    # silently break delivery, exactly as documented for _WorkerBridge.
    _state: dict = {
        "transport": None,
        "live_view_active": False,
        "live_worker": None,
        "live_thread": None,
        "live_bridge": None,
        # Latest camera ground-truth pose (x_cm, y_cm, yaw_rad, monotonic_ts)
        # cached from the transport on_truth callback for the GOTO loop.
        "last_truth": None,
        # 119 ticket 001: tracks the LAST drained frame's own
        # fault_shaping_disabled bit, for edge-triggered logging in
        # _TelemetryBridge.on_frame_ready() -- see that method's own
        # docstring.
        "shaping_disabled_active": False,
        # 129-002: tracks the LAST drained frame's own wheel_frozen_reason()
        # (None while healthy, else "LEFT"/"RIGHT"/"LEFT + RIGHT"), for the
        # same edge-triggered logging + level-set banner pattern as
        # shaping_disabled_active above.
        "wheel_frozen_reason": None,
    }

    # Session recorder — Qt-free; accumulates TX/RX lines to a JSONL file.
    # Manual recorder: driven by the Record/Pause/Stop buttons, writes a
    # timestamped file the operator explicitly saves.
    recorder = SessionRecorder()
    # Always-on "latest" recorder: runs for the whole connected session and
    # overwrites recordings/latest.jsonl each connect, so there is ALWAYS a
    # capture even when the operator forgot to press Record.  Fed the same
    # TX/RX lines as the manual recorder, but independent of its state.
    latest_recorder = SessionRecorder()
    #: Fixed filename the latest-session capture is (over)written to.
    LATEST_RECORDING_NAME = "latest.jsonl"

    # ------------------------------------------------------------------ window
    window = QMainWindow()
    window.setWindowTitle("Robot Test GUI")
    # Open large so the playfield view gets real estate, clamped to the
    # screen (with an allowance for the title bar) so the window still
    # fits on smaller displays.
    win_w, win_h = 1920, 1110
    screen = app.primaryScreen()
    if screen is not None:
        avail = screen.availableGeometry()
        win_w = min(win_w, avail.width())
        win_h = min(win_h, avail.height() - 40)
    window.resize(win_w, win_h)

    # ------------------------------------------------------------ central widget
    central = QWidget()
    window.setCentralWidget(central)
    root_layout = QHBoxLayout(central)
    root_layout.setContentsMargins(4, 4, 4, 4)

    splitter = QSplitter(Qt.Orientation.Horizontal)
    root_layout.addWidget(splitter)

    # ---------------------------------------------------------------- left panel
    left_widget = QWidget()
    left_layout = QVBoxLayout(left_widget)
    left_layout.setContentsMargins(4, 4, 4, 4)

    # Transport selector
    transport_label = QLabel("Transport:")

    transport_combo = QComboBox()
    transport_combo.setObjectName("transport_combo")
    transport_combo.addItems(["Sim", "Serial", "Relay"])

    # Robot selector — pick which robot config is active.  Selecting a robot
    # rewrites active_robot.json and reloads that config (wired below, once
    # trace_model / _append_log are in scope).
    from robot_radio.config.robot_config import (
        _reset_robot_config,
        get_robot_config,
        list_robots,
        set_active_robot,
    )

    robot_label = QLabel("Robot:")

    robot_combo = QComboBox()
    robot_combo.setObjectName("robot_combo")
    _robot_choices = list_robots()  # list[(name, path)]
    for _name, _path in _robot_choices:
        robot_combo.addItem(_name, str(_path))
    # Preselect the currently-active robot, if resolvable.
    _active_cfg = get_robot_config()
    if _active_cfg is not None:
        _idx = robot_combo.findText(_active_cfg.robot_name)
        if _idx >= 0:
            robot_combo.setCurrentIndex(_idx)

    # Camera-selection pull-down (ticket 063-008; relocated into the
    # session-initiation selector row by ticket 075-001) — lists cameras
    # known to the aprilcam daemon; persisted across sessions via
    # camera_prefs.  Populated best-effort below (_populate_camera_combo);
    # selection change is wired further down, once ops_ctrl.trigger_live_grab
    # is available.
    camera_combo_label = QLabel("Camera:")
    camera_combo = QComboBox()
    camera_combo.setObjectName("camera_combo")
    camera_combo.setToolTip(
        "Select which aprilcam daemon camera to use for the playfield feed.\n"
        "The selection persists across sessions and triggers an immediate\n"
        "playfield refresh from the newly selected camera."
    )

    # Sim fast-forward selector — visible only for the Sim transport
    # (toggled in _on_transport_changed alongside the Sim Errors panel).
    # The sim clock is purely virtual; SimTransport's tick-thread paces it
    # to wall time, so a speed factor just advances more sim-steps per wall
    # tick (identical physics, compressed wall time).  Applied live via
    # SimTransport.set_speed_factor() on change and on connect.
    sim_speed_label = QLabel("Speed:")
    sim_speed_combo = QComboBox()
    sim_speed_combo.setObjectName("sim_speed_combo")
    for _mult in (1, 2, 5, 10, 20):
        sim_speed_combo.addItem(f"{_mult}×", _mult)
    sim_speed_combo.setToolTip(
        "Sim fast-forward: how much sim-time passes per wall-clock second.\n"
        "Physics integration step is unchanged — trajectories are identical\n"
        "at every speed. High factors make the console log very busy."
    )

    # Session-initiation selector strip (ticket 075-001): transport, robot,
    # and camera combos collapsed into one row.
    selector_row = QWidget()
    selector_layout = QHBoxLayout(selector_row)
    selector_layout.setContentsMargins(0, 0, 0, 0)
    selector_layout.addWidget(transport_label)
    selector_layout.addWidget(transport_combo, stretch=1)
    selector_layout.addWidget(robot_label)
    selector_layout.addWidget(robot_combo, stretch=1)
    selector_layout.addWidget(camera_combo_label)
    selector_layout.addWidget(camera_combo, stretch=1)
    selector_layout.addWidget(sim_speed_label)
    selector_layout.addWidget(sim_speed_combo)
    left_layout.addWidget(selector_row)

    # Port picker (enabled only for Serial / Relay)
    port_label = QLabel("Port:")
    left_layout.addWidget(port_label)

    port_edit = QLineEdit()
    port_edit.setObjectName("port_edit")
    port_edit.setPlaceholderText("/dev/cu.usbmodem…")
    port_edit.setEnabled(False)
    # Pre-populate with the ROBOT's port (device-registry role lookup, so a
    # plugged-in relay dongle is never pre-picked); fall back to the first
    # detected USB modem port.
    detected = list_ports()
    _robot_port = find_robot_serial_port(detected)
    if _robot_port is not None:
        port_edit.setText(_robot_port)
    elif detected:
        port_edit.setText(detected[0])
    left_layout.addWidget(port_edit)

    # Session-control buttons (ticket 075-001): Connect, Disconnect, Record,
    # Pause, Stop collapsed into one row, each carrying a QStyle standard
    # icon (Design Rationale Decision 3: affirmative/negative pair for
    # Connect/Disconnect, media-transport family for Record/Pause/Stop).
    connect_btn = QPushButton("Connect")
    connect_btn.setObjectName("connect_btn")
    connect_btn.setIcon(
        window.style().standardIcon(QStyle.StandardPixmap.SP_DialogYesButton)
    )
    disconnect_btn = QPushButton("Disconnect")
    disconnect_btn.setObjectName("disconnect_btn")
    disconnect_btn.setEnabled(False)
    disconnect_btn.setIcon(
        window.style().standardIcon(QStyle.StandardPixmap.SP_DialogNoButton)
    )

    # Record / Pause / Stop controls (ticket 005)
    record_btn = QPushButton("Record")
    record_btn.setObjectName("record_btn")
    record_btn.setIcon(
        window.style().standardIcon(QStyle.StandardPixmap.SP_MediaPlay)
    )
    pause_btn = QPushButton("Pause")
    pause_btn.setObjectName("pause_btn")
    pause_btn.setEnabled(False)
    pause_btn.setIcon(
        window.style().standardIcon(QStyle.StandardPixmap.SP_MediaPause)
    )
    stop_btn = QPushButton("Stop")
    stop_btn.setObjectName("stop_btn")
    stop_btn.setEnabled(False)
    stop_btn.setIcon(
        window.style().standardIcon(QStyle.StandardPixmap.SP_MediaStop)
    )

    session_btn_row = QWidget()
    session_btn_layout = QHBoxLayout(session_btn_row)
    session_btn_layout.setContentsMargins(0, 0, 0, 0)
    session_btn_layout.addWidget(connect_btn)
    session_btn_layout.addWidget(disconnect_btn)
    session_btn_layout.addWidget(record_btn)
    session_btn_layout.addWidget(pause_btn)
    session_btn_layout.addWidget(stop_btn)
    left_layout.addWidget(session_btn_row)

    # Send buttons — collected so every motion control (Managed/Unmanaged
    # preset buttons, tour buttons, etc.) enables/disables together on
    # connect/disconnect. 128-004: the parameter-field COMMANDS-schema
    # command-row panel that used to populate part of this list (Send
    # button per S/T/D/R/TURN/RT/G row, built by ``_build_command_row()``/
    # ``_wire_send_button()``) is deleted outright -- it was already hidden
    # (``setVisible(False)``, stakeholder 2026-07-17: "I don't need full
    # parameters, I just need buttons") and translated verbs with no
    # binary-plane arm on the current wire (``testgui/binary_bridge.py``'s
    # own module docstring has the full accounting). The Managed/Unmanaged
    # preset-button panel below (and, for D/RT specifically, its fixed
    # presets) is the live replacement; see ``commands.py``'s own module
    # docstring for the fuller history.
    _send_buttons: list[QPushButton] = []

    # Two-column motion panel (Unmanaged direct-twist | Managed MOVE-queue).
    # Each sends the binary SEG primitive (arc_length=0 => pure pivot), CCW+,
    # delta_heading in centidegrees: "SEG 0 <cdeg>" (binary_bridge translates
    # to CommandEnvelope{segment}). Enabled on connect via _send_buttons.
    # Shared by the buttons AND the turn-control socket (turn_control.py).
    def _send_seg_turn(deg_value: float) -> None:
        transport = _state.get("transport")
        if transport is None:
            _append_log("[WARN] Not connected")
            return
        cdeg = int(round(deg_value * 100))
        _append_log(f"[TURN] {deg_value:+g} deg  ->  SEG 0 {cdeg}")
        try:
            transport.command(f"SEG 0 {cdeg}", read_timeout=500)
        except Exception as exc:  # noqa: BLE001
            _append_log(f"[ERROR] {exc}")

    def _make_turn_handler(deg_value: int):
        return lambda: _send_seg_turn(deg_value)

    # --- Motion panel: two columns, SAME commands via DIFFERENT paths -------
    # LEFT  = Unmanaged: direct twist/stop, NO trajectory planner
    #         (SimTransport.run_unmanaged -> one twist, deadman-timed).
    # RIGHT = Managed:   D/RT -> planner.tour.run_tour -> MOVE queue.
    # Same distance/angle presets on both sides so the ONLY variable is the
    # path. Distance presets are [mm]; angle presets are [deg]; each fires
    # forward (+) and back/CCW-vs-CW (-).
    _DIST_PRESETS = (100, 500, 700)
    _ANGLE_PRESETS = (90, 180, 270, 360)

    def _unmanaged_dist(mm: float) -> None:
        t = _state.get("transport")
        if t is not None and hasattr(t, "run_unmanaged"):
            t.run_unmanaged(distance_mm=float(mm))

    def _unmanaged_ang(deg: float) -> None:
        t = _state.get("transport")
        if t is not None and hasattr(t, "run_unmanaged"):
            t.run_unmanaged(angle_deg=float(deg))

    def _managed_dist(mm: float) -> None:
        t = _state.get("transport")
        if t is not None:
            t.command(f"D 150 150 {int(mm)}", read_timeout=500)

    def _managed_ang(deg: float) -> None:
        t = _state.get("transport")
        if t is not None:
            t.command(f"RT {int(round(deg * 100))}", read_timeout=500)

    # Test buttons (stakeholder 2026-07-18/19) — the in-GUI edit->compile->
    # reload->run loop lives INSIDE each path column (not a separate row):
    # each RECOMPILES the sim lib, HOT-RELOADS the fresh dylib, resets the
    # avatar/pose/traces, then runs the column's own fixed motion —
    # managed = D/RT through Motion::Executor/Pilot; unmanaged = one
    # run_unmanaged() deadman-timed twist (no planner, no heading loop, no
    # retargets). Always clickable (they connect themselves); wired to
    # _run_sim_test() at the bottom of this function, once that (and
    # _on_connect/_set_origin) are defined.
    test_s_btn = QPushButton("S — drive 700mm")
    test_s_btn.setObjectName("test_s_btn")
    test_s_btn.setToolTip(
        "Rebuild + reload the sim, reset, then drive 700 mm through the "
        "managed path (Motion::Executor profile + Pilot heading loop)."
    )
    test_t_btn = QPushButton("T — turn 360°")
    test_t_btn.setObjectName("test_t_btn")
    test_t_btn.setToolTip(
        "Rebuild + reload the sim, reset, then turn 360° through the "
        "managed path (Motion::Executor profile + Pilot heading PD)."
    )
    test_us_btn = QPushButton("S — drive 700mm")
    test_us_btn.setObjectName("test_us_btn")
    test_us_btn.setToolTip(
        "Rebuild + reload the sim, reset, then drive 700 mm UNMANAGED: one "
        "direct twist at 150 mm/s, deadman-timed — no planner, no heading "
        "loop, no retargets."
    )
    test_ut_btn = QPushButton("T — turn 360°")
    test_ut_btn.setObjectName("test_ut_btn")
    test_ut_btn.setToolTip(
        "Rebuild + reload the sim, reset, then turn 360° UNMANAGED: one "
        "direct twist at 2 rad/s, deadman-timed — no planner, no heading "
        "loop, no retargets."
    )

    # Inner velocity-PID enable — ONE sim-wide state (NezhaMotor::
    # setPidEnabled() on both motors via SimTransport.set_pid_enabled), with
    # a checkbox in EACH column (stakeholder: "both of them should have a
    # way to turn off the PID"); the two stay mirrored via _on_pid_toggled.
    # Checked = PID on (firmware default). Unchecked, wheels run open-loop
    # feedforward (duty = kff × velocity target). A MANAGED motion still
    # closes the outer heading/executor loops either way — the unmanaged
    # column is the fully open-loop path. Re-applied after every connect
    # (incl. the Test buttons' rebuild+reconnect) in _on_connect(), since a
    # fresh sim always boots with PID enabled.
    pid_checkbox_u = QCheckBox("PID")
    pid_checkbox_u.setObjectName("pid_checkbox_unmanaged")
    pid_checkbox_m = QCheckBox("PID")
    pid_checkbox_m.setObjectName("pid_checkbox_managed")
    for _cb in (pid_checkbox_u, pid_checkbox_m):
        _cb.setChecked(True)
        _cb.setToolTip(
            "Inner velocity PID on/off (Sim only, one state — the two "
            "checkboxes mirror each other). Unchecked: wheels run open-loop "
            "feedforward (duty = kff × velocity target)."
        )

    def _on_pid_toggled(checked: bool) -> None:
        # Mirror the two column checkboxes (one sim, one inner-loop state)
        # without re-entrant toggling, then apply live when a sim is
        # connected; otherwise the state is picked up at the next Sim
        # connect (_on_connect).
        for _cb in (pid_checkbox_u, pid_checkbox_m):
            if _cb.isChecked() != checked:
                _cb.blockSignals(True)
                _cb.setChecked(checked)
                _cb.blockSignals(False)
        transport = _state.get("transport")
        if isinstance(transport, SimTransport):
            transport.set_pid_enabled(bool(checked))

    pid_checkbox_u.toggled.connect(_on_pid_toggled)
    pid_checkbox_m.toggled.connect(_on_pid_toggled)

    def _make_motion_column(title: str, key: str, dist_cb, ang_cb,
                            test_s: QPushButton, test_t: QPushButton,
                            pid_cb: QCheckBox) -> QGroupBox:
        """``key`` ("unmanaged"/"managed", stakeholder headless-button-
        acceptance-suite fix, 2026-07-22): object-names each preset button
        ``{key}_dist_btn_{signed:+d}``/``{key}_ang_btn_{signed:+d}`` so a
        headless test can ``findChild()`` and ``.click()`` the EXACT preset
        button an operator presses, instead of only calling the underlying
        callback. Mirrors the GOTO spinboxes' own object-name precedent
        (see their own creation comment above) — before this, these preset
        buttons were the one remaining gap in the button surface with no
        findChild()-able identity at all.
        """
        box = QGroupBox(title)
        col = QVBoxLayout(box)
        col.setContentsMargins(6, 4, 6, 4)
        col.setSpacing(3)
        for group_label, presets, cb, kind in (
            ("Distance [mm]", _DIST_PRESETS, dist_cb, "dist"),
            ("Angles [deg]", _ANGLE_PRESETS, ang_cb, "ang"),
        ):
            lbl = QLabel(group_label)
            lbl.setStyleSheet("font-weight: bold;")
            col.addWidget(lbl)
            for mag in presets:
                row = QWidget()
                h = QHBoxLayout(row)
                h.setContentsMargins(0, 0, 0, 0)
                h.setSpacing(3)
                for signed in (mag, -mag):
                    b = QPushButton(f"{signed:+d}")
                    b.setObjectName(f"{key}_{kind}_btn_{signed:+d}")
                    b.setEnabled(False)
                    b.setFixedWidth(64)
                    b.clicked.connect(lambda _checked=False, s=signed, f=cb: f(s))
                    h.addWidget(b)
                    _send_buttons.append(b)
                h.addStretch()
                col.addWidget(row)
        # This column's own Test section (rebuild+reload+run through THIS
        # column's path) + its PID checkbox — see the widgets' own creation
        # comments above.
        lbl = QLabel("Test")
        lbl.setStyleSheet("font-weight: bold;")
        col.addWidget(lbl)
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(3)
        h.addWidget(test_s)
        h.addWidget(test_t)
        h.addWidget(pid_cb)
        h.addStretch()
        col.addWidget(row)
        col.addStretch()
        return box

    motion_panel = QWidget()
    motion_panel_layout = QHBoxLayout(motion_panel)
    motion_panel_layout.setContentsMargins(0, 0, 0, 0)
    motion_panel_layout.setSpacing(8)
    motion_panel_layout.addWidget(
        _make_motion_column("Unmanaged — direct twist", "unmanaged", _unmanaged_dist, _unmanaged_ang,
                            test_us_btn, test_ut_btn, pid_checkbox_u))
    motion_panel_layout.addWidget(
        _make_motion_column("Managed — MOVE", "managed", _managed_dist, _managed_ang,
                            test_s_btn, test_t_btn, pid_checkbox_m))
    left_layout.addWidget(motion_panel)

    # Tour buttons — run a pre-programmed motion sequence (one per named tour).
    # Each button resets the robot to the origin, then sends the tour's moves
    # one at a time on a background thread, waiting for each to complete.
    #
    # Tour steps (D/RT) go straight to NezhaProtocol.move_twist()/
    # move_wheels() via transport.py's _dispatch_managed_move() -- never
    # through binary_bridge.py. _set_origin()'s own OZ/SI/ZERO calls stay
    # gated (generic ERR "unsupported" reply -- neither has a binary-plane
    # arm on the current wire, see ``testgui/binary_bridge.py``'s own module
    # docstring) and are therefore no-ops on the binary plane today, but
    # that no longer blocks the tour from running:
    # _set_origin() also halts (128-003: transport.halt(), estop) and resets
    # the DISPLAY (TraceModel
    # anchor/clear, canvas avatar) unconditionally, and the sim plant is
    # teleported to (0,0,0) separately (is_sim_transport() branch) -- the
    # tour then drives for real via D/RT, and the avatar tracks it via
    # host-side encoder dead reckoning (traces.py's EncoderDeadReckoner).
    tour_row = QWidget()
    tour_layout = QHBoxLayout(tour_row)
    tour_layout.setContentsMargins(0, 0, 0, 0)
    tour_layout.setSpacing(4)
    # 108-007: tours now run against BOTH backends -- SimTransport is
    # rewired onto robot_radio.io.sim_loop.SimLoop (the real, compiled
    # firmware simulator), which satisfies planner.executor.TwistTransport
    # directly, the same shape _HardwareTransport.protocol already exposed
    # for a live NezhaProtocol (see transport.py's SimTransport.protocol).
    # Tour buttons therefore enable identically for Sim and hardware
    # connections -- see _on_connect() below, which no longer disables them
    # for is_sim_transport(transport).
    # 134-005: the rest-to-rest tours ("Square") say so in their tooltip --
    # the sequencing, not just the geometry, is what the operator is
    # choosing between (see planner.tour.TourExecution).
    def _tour_execution_note(name: str) -> str:
        execution = tour_execution(name)
        if not getattr(execution, "sequential", False):
            # Pipelined by default -- say so, and point at the control that
            # changes it, so the operator can tell which path a press takes.
            return (
                " Pipelined by default (one-leg lookahead, never rests); "
                "tick 'Rest-to-rest' to run it the way the square tour was "
                "measured instead."
            )
        return (
            f" Rest-to-rest: one Move in flight at a time with a "
            f"{execution.settle:.1f}s PASSIVE settle at every boundary "
            "(nothing is sent during the dwell -- a wheels(0,0) lease "
            "would clear the firmware's heading ledger). This is the "
            "path ticket 134-004 measured at median 6.3 mm closure."
        )

    def _tour_hw_tooltip(name: str) -> str:
        return (
            f"Run {name}: resets to the origin (display-only pending "
            "sprint 098's OZ/SI binary arms), then drives the tour's "
            "geometry via planner.tour.run_tour() (streamed twist()s, "
            "closed-loop heading correction)."
            + _tour_execution_note(name)
        )

    def _tour_sim_tooltip(name: str) -> str:
        return (
            f"Run {name}: drives the tour's geometry against the compiled "
            "firmware simulator (SimLoop) via planner.tour.run_tour() "
            "(streamed twist()s, closed-loop heading correction) -- the "
            "same driver path as hardware."
            + _tour_execution_note(name)
        )

    _tour_buttons: list[tuple[QPushButton, str]] = []
    for _tour_name in TOURS:
        _tb = QPushButton(_tour_name)
        _tb.setObjectName(f"tour_btn_{_tour_name.lower().replace(' ', '_')}")
        _tb.setEnabled(False)
        _tb.setToolTip(_tour_hw_tooltip(_tour_name))
        tour_layout.addWidget(_tb)
        _tour_buttons.append((_tb, _tour_name))
        # 097: added to _send_buttons (below) so tour buttons enable on
        # connect like every other motion control -- the tour-RUNNING
        # disable (while a tour is in flight) is a SEPARATE, orthogonal
        # concern handled by _on_tour_clicked/_stop_tour/_on_tour_finished.
        _send_buttons.append(_tb)
    # Stop Tour — dedicated control to abort a running tour. Deliberately NOT
    # added to _send_buttons: unlike the tour buttons themselves (which enable
    # on connect), this button must stay disabled while idle even when
    # connected — it only enables while a tour is actively running (see
    # _on_tour_clicked / _stop_tour / _on_tour_finished below).
    stop_tour_btn = QPushButton("Stop Tour")
    stop_tour_btn.setObjectName("stop_tour_btn")
    stop_tour_btn.setEnabled(False)
    stop_tour_btn.setToolTip("Stop the currently running tour.")
    tour_layout.addWidget(stop_tour_btn)
    # Rest-to-rest opt-in (OOP 2026-08-05). Sprint 134 proved the
    # rest-to-rest execution on "Square" only; Tour 1 / Tour 2 stayed on the
    # pipelined lookahead and got none of it. This box is how they reach the
    # same path -- unchecked it changes nothing (both tours run exactly as
    # they always have), and it never alters "Square", whose own execution is
    # already sequential and measured good. Read at click time, so toggling
    # it mid-tour affects the NEXT run, not the one in flight.
    rest_to_rest_chk = QCheckBox("Rest-to-rest (align each corner)")
    rest_to_rest_chk.setObjectName("tour_rest_to_rest_chk")
    rest_to_rest_chk.setChecked(False)
    rest_to_rest_chk.setToolTip(
        "Run Tour 1 / Tour 2 the way sprint 134 proved the square tour: one "
        "Move in flight at a time, with a 1.2s PASSIVE settle at every "
        "segment boundary and 2.4 rad/s pivots (measured median 6.3 mm "
        "closure, ticket 134-004). Unchecked, those two tours keep the "
        "historical pipelined one-leg lookahead. 'Square' always runs "
        "rest-to-rest either way."
    )
    tour_layout.addWidget(rest_to_rest_chk)
    tour_layout.addStretch()
    left_layout.addWidget(tour_row)

    # GOTO — synthetic camera-based go-to: drive to a world (x, y) point by
    # repeatedly correcting the robot's pose from the camera and re-issuing G.
    #
    # This is the WORLD-ABSOLUTE, camera-closed-loop GOTO. This button stays
    # gated: its pursuit loop repeatedly snaps the robot's pose to camera
    # truth (SI) between re-issued G's, and neither SI nor G has a
    # binary-plane arm on the current wire (128-004: see
    # ``testgui/binary_bridge.py``'s own module docstring for the full
    # accounting -- the old COMMANDS schema's relative-open-loop "G" row
    # this comment used to point to as a working alternative is itself
    # deleted, for the same reason).
    goto_row = QWidget()
    goto_layout = QHBoxLayout(goto_row)
    goto_layout.setContentsMargins(0, 0, 0, 0)
    goto_layout.setSpacing(4)

    goto_btn = QPushButton("GOTO")
    goto_btn.setObjectName("goto_btn")
    goto_btn.setEnabled(False)
    goto_btn.setFixedWidth(52)
    goto_btn.setToolTip(
        "World-absolute camera GOTO requires sprint 098 -- the pursuit loop "
        "repeatedly re-anchors the robot's pose to camera truth (SI), which "
        "has no binary-plane arm on the current wire."
    )
    goto_layout.addWidget(goto_btn)

    _goto_verb = QLabel("GOTO")
    _goto_verb.setFixedWidth(44)
    goto_layout.addWidget(_goto_verb)

    def _make_goto_spin(name: str, default: int, lo: int, hi: int, unit: str):
        lbl = QLabel(f"{name}:")
        lbl.setFixedWidth(46)
        goto_layout.addWidget(lbl)
        sp = QSpinBox()
        # Object name (085-003): the tour buttons and Sim Errors spinboxes
        # already expose findChild()-able object names for headless GUI
        # tests (test_tour1_geometry.py's pattern); the GOTO spinboxes had
        # none, which made it impossible to drive a real end-to-end GOTO
        # test through the actual widgets — this was the one genuine gap
        # ticket 003's new test surfaced.
        sp.setObjectName(f"goto_spin_{name}")
        sp.setRange(lo, hi)
        sp.setValue(default)
        sp.setFixedWidth(80)
        goto_layout.addWidget(sp)
        # Unit label OUTSIDE the edit box (not a spin-box suffix).
        goto_layout.addWidget(QLabel(unit))
        return sp

    goto_x_spin = _make_goto_spin("x", 0, -10000, 10000, "mm")
    goto_y_spin = _make_goto_spin("y", 0, -10000, 10000, "mm")
    goto_eps_spin = _make_goto_spin("eps", 50, 1, 2000, "mm")
    goto_speed_spin = _make_goto_spin("speed", 200, 1, 1000, "mm/s")
    goto_layout.addStretch()
    left_layout.addWidget(goto_row)
    # 097: goto_btn is NOT added to _send_buttons -- GOTO's pursuit loop
    # needs SI/G, gated pending sprint 098, so it stays permanently
    # disabled instead of enabling on connect (see the tooltip above).

    # Sim Errors panel (issue testgui-sim-error-profile-config, extended to
    # the full SIMSET knob set by ticket 069-007) — makes every Sim-mode
    # injected plant/odometry error runtime-configurable instead of the
    # historical hardcoded constants. Backed by sim_prefs' persisted JSON
    # file; visible only when Sim is the selected transport (toggled in
    # _on_transport_changed below). Editable pre-connect: the normal
    # workflow is set errors, then Connect — SimTransport picks the saved
    # file up on connect via _apply_field_profile().
    sim_errors_group = QGroupBox("Sim Errors")
    sim_errors_group.setObjectName("sim_errors_group")
    sim_errors_layout = QVBoxLayout(sim_errors_group)
    sim_errors_layout.setContentsMargins(4, 4, 4, 4)
    sim_errors_layout.setSpacing(4)

    _sim_error_profile = sim_prefs.load_sim_error_profile()

    def _make_sim_err_spin(
        target_layout: QVBoxLayout, object_name: str, label: str, value: float,
        lo: float, hi: float, decimals: int,
    ) -> QDoubleSpinBox:
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(4)
        lbl = QLabel(label)
        lbl.setFixedWidth(96)
        row_layout.addWidget(lbl)
        spin = QDoubleSpinBox()
        spin.setObjectName(object_name)
        spin.setRange(lo, hi)
        spin.setDecimals(decimals)
        spin.setValue(value)
        spin.setFixedWidth(68)
        row_layout.addWidget(spin)
        row_layout.addStretch()
        target_layout.addWidget(row)
        return spin

    def _add_sim_err_section_label(target_layout: QVBoxLayout, title: str) -> None:
        """Bold sub-heading grouping the spin-box rows that follow it.

        Purely visual (069-007's suggested grouping: "Encoder Report Error",
        "Body-Truth Scrub", "Geometry & Actuation", "OTOS Error") — the
        panel's column QVBoxLayouts have no nested QGroupBoxes, so tests
        locate individual spin boxes by objectName regardless of grouping.
        """
        lbl = QLabel(title)
        bold_font = lbl.font()
        bold_font.setBold(True)
        lbl.setFont(bold_font)
        target_layout.addWidget(lbl)

    # Three side-by-side columns: LEFT = OTOS Error + Geometry & Actuation,
    # MIDDLE = Encoder Report Error, RIGHT = Body-Truth Scrub (069-007 /
    # architecture-update.md Decision 2).
    columns_row = QWidget()
    columns_layout = QHBoxLayout(columns_row)
    columns_layout.setContentsMargins(0, 0, 0, 0)
    columns_layout.setSpacing(8)

    col_left = QWidget()
    col_left_layout = QVBoxLayout(col_left)
    col_left_layout.setContentsMargins(0, 0, 0, 0)
    col_left_layout.setSpacing(4)
    columns_layout.addWidget(col_left)

    col_mid = QWidget()
    col_mid_layout = QVBoxLayout(col_mid)
    col_mid_layout.setContentsMargins(0, 0, 0, 0)
    col_mid_layout.setSpacing(4)
    columns_layout.addWidget(col_mid)

    col_right = QWidget()
    col_right_layout = QVBoxLayout(col_right)
    col_right_layout.setContentsMargins(0, 0, 0, 0)
    col_right_layout.setSpacing(4)
    columns_layout.addWidget(col_right)

    # -- LEFT column: OTOS Error --------------------------------------------
    _add_sim_err_section_label(col_left_layout, "OTOS Error")
    sim_err_otos_linear = _make_sim_err_spin(
        col_left_layout, "sim_err_otos_linear", "OTOS linear noise:",
        _sim_error_profile["otos_linear_noise"], 0.0, 2.0, 3,
    )
    sim_err_otos_yaw = _make_sim_err_spin(
        col_left_layout, "sim_err_otos_yaw", "OTOS yaw noise:",
        _sim_error_profile["otos_yaw_noise"], 0.0, 2.0, 3,
    )
    sim_err_otos_lin_scale = _make_sim_err_spin(
        col_left_layout, "sim_err_otos_lin_scale", "OTOS lin scale err:",
        _sim_error_profile["otos_lin_scale_err"], -0.5, 0.5, 3,
    )
    sim_err_otos_ang_scale = _make_sim_err_spin(
        col_left_layout, "sim_err_otos_ang_scale", "OTOS ang scale err:",
        _sim_error_profile["otos_ang_scale_err"], -0.5, 0.5, 3,
    )
    # 083-001: sim_set_otos_linear_drift/sim_set_otos_yaw_drift apply a
    # constant ADDITIVE term once per SimOdometer tick (confirmed by reading
    # source/hal/sim/sim_odometer.cpp's tick()) -- NOT the old SIMSET wire
    # protocol's per-SECOND rate (otosLinDriftMmS/otosYawDriftDegS), which no
    # longer exists.  Labels/ranges below reflect the per-tick unit the
    # ctypes setter actually takes (see sim_prefs.py's module docstring).
    sim_err_otos_lin_drift = _make_sim_err_spin(
        col_left_layout, "sim_err_otos_lin_drift", "OTOS lin drift (mm/tick):",
        _sim_error_profile["otos_lin_drift"], -5.0, 5.0, 3,
    )
    sim_err_otos_yaw_drift = _make_sim_err_spin(
        col_left_layout, "sim_err_otos_yaw_drift", "OTOS yaw drift (rad/tick):",
        _sim_error_profile["otos_yaw_drift"], -0.05, 0.05, 4,
    )

    # 083-001: motor_offset_l/r and slip_turn_extra (below) have NO ctypes
    # ABI entry point at all against the current sim_api.cpp surface
    # (Hal::PhysicsWorld::setOffsetFactor() is deliberately left unwrapped;
    # no turn-rate-dependent slip knob is wired -- see SimConnection's module
    # docstring). Stakeholder decision: these spin boxes stay VISIBLE (not
    # hidden/removed) so a persisted profile with a non-neutral value is
    # still visible to the operator; a tooltip flags that setting them has
    # no effect on a running Sim.
    _SIM_ERR_NOT_SUPPORTED_TOOLTIP = (
        "Not supported in sim: no ctypes ABI entry point backs this knob "
        "against the current sim_api.cpp surface. Setting this has no "
        "effect on a running Sim."
    )

    # -- LEFT column: Geometry & Actuation -----------------------------------
    _add_sim_err_section_label(col_left_layout, "Geometry & Actuation")
    sim_err_motor_offset_l = _make_sim_err_spin(
        col_left_layout, "sim_err_motor_offset_l", "motor offset L:",
        _sim_error_profile["motor_offset_l"], 0.0, 2.0, 3,
    )
    sim_err_motor_offset_l.setToolTip(_SIM_ERR_NOT_SUPPORTED_TOOLTIP)
    sim_err_motor_offset_r = _make_sim_err_spin(
        col_left_layout, "sim_err_motor_offset_r", "motor offset R:",
        _sim_error_profile["motor_offset_r"], 0.0, 2.0, 3,
    )
    sim_err_motor_offset_r.setToolTip(_SIM_ERR_NOT_SUPPORTED_TOOLTIP)
    # trackwidth has NO safe zero default (PhysicsWorld::update() divides
    # by it) — the spinbox range excludes 0 entirely, and the default is the
    # firmware config's trackwidth (128.0mm, what the sim seeds the plant
    # with at construction), not a sentinel. Every Apply unconditionally
    # sends this value; there is no "don't touch" case.
    sim_err_trackwidth = _make_sim_err_spin(
        col_left_layout, "sim_err_trackwidth", "trackwidth (mm):",
        _sim_error_profile["trackwidth"], 10.0, 500.0, 1,
    )

    # -- MIDDLE column: Encoder Report Error --------------------------------
    _add_sim_err_section_label(col_mid_layout, "Encoder Report Error")
    sim_err_encoder = _make_sim_err_spin(
        col_mid_layout, "sim_err_encoder_mm", "encoder noise (mm):",
        _sim_error_profile["encoder_noise"], 0.0, 50.0, 2,
    )
    sim_err_enc_scale_l = _make_sim_err_spin(
        col_mid_layout, "sim_err_enc_scale_l", "enc scale err L:",
        _sim_error_profile["enc_scale_err_l"], -0.5, 0.5, 3,
    )
    sim_err_enc_scale_r = _make_sim_err_spin(
        col_mid_layout, "sim_err_enc_scale_r", "enc scale err R:",
        _sim_error_profile["enc_scale_err_r"], -0.5, 0.5, 3,
    )
    col_mid_layout.addStretch()

    # -- RIGHT column: Body-Truth Scrub --------------------------------------
    _add_sim_err_section_label(col_right_layout, "Body-Truth Scrub")
    sim_err_slip_turn = _make_sim_err_spin(
        col_right_layout, "sim_err_slip_turn", "turn slip:",
        _sim_error_profile["slip_turn_extra"], 0.0, 2.0, 3,
    )
    sim_err_slip_turn.setToolTip(_SIM_ERR_NOT_SUPPORTED_TOOLTIP)
    sim_err_body_rot_scrub = _make_sim_err_spin(
        # 128-003 baseline fix: this field's window-build default is
        # resolve_calibration_defaults()'s reconciled value from the active
        # robot's calibration.rotational_slip -- data/robots/tovez.json now
        # carries 4 decimal digits (0.9117); 3 decimals rounded that to
        # 0.912 on setValue(), a silent precision loss that broke every
        # test asserting the spin box round-trips the resolved default
        # exactly. 4 decimals matches the measured calibration precision.
        col_right_layout, "sim_err_body_rot_scrub", "body rot scrub:",
        _sim_error_profile["body_rot_scrub"], 0.0, 1.0, 4,
    )
    sim_err_body_lin_scrub = _make_sim_err_spin(
        col_right_layout, "sim_err_body_lin_scrub", "body lin scrub:",
        _sim_error_profile["body_lin_scrub"], 0.0, 1.0, 3,
    )
    col_right_layout.addStretch()

    sim_errors_layout.addWidget(columns_row)

    sim_errors_apply_btn = QPushButton("Apply")
    sim_errors_apply_btn.setObjectName("sim_errors_apply_btn")
    sim_errors_from_cal_btn = QPushButton("From Calibration")
    sim_errors_from_cal_btn.setObjectName("sim_errors_from_cal_btn")
    sim_errors_btn_row = QWidget()
    sim_errors_btn_layout = QHBoxLayout(sim_errors_btn_row)
    sim_errors_btn_layout.setContentsMargins(0, 0, 0, 0)
    sim_errors_btn_layout.setSpacing(4)
    sim_errors_btn_layout.addWidget(sim_errors_apply_btn)
    sim_errors_btn_layout.addWidget(sim_errors_from_cal_btn)
    sim_errors_layout.addWidget(sim_errors_btn_row)

    def _on_sim_errors_apply() -> None:
        """Save the Sim Errors fields and, if connected to Sim, apply live."""
        profile = {
            "encoder_noise": sim_err_encoder.value(),
            "slip_turn_extra": sim_err_slip_turn.value(),
            "otos_linear_noise": sim_err_otos_linear.value(),
            "otos_yaw_noise": sim_err_otos_yaw.value(),
            "enc_scale_err_l": sim_err_enc_scale_l.value(),
            "enc_scale_err_r": sim_err_enc_scale_r.value(),
            "body_rot_scrub": sim_err_body_rot_scrub.value(),
            "body_lin_scrub": sim_err_body_lin_scrub.value(),
            "motor_offset_l": sim_err_motor_offset_l.value(),
            "motor_offset_r": sim_err_motor_offset_r.value(),
            "trackwidth": sim_err_trackwidth.value(),
            "otos_lin_scale_err": sim_err_otos_lin_scale.value(),
            "otos_ang_scale_err": sim_err_otos_ang_scale.value(),
            "otos_lin_drift": sim_err_otos_lin_drift.value(),
            "otos_yaw_drift": sim_err_otos_yaw_drift.value(),
        }
        sim_prefs.save_sim_error_profile(profile)
        _append_log(
            f"[INFO] Sim error profile saved "
            f"(encoder_noise={profile['encoder_noise']}, "
            f"slip_turn_extra={profile['slip_turn_extra']}, "
            f"otos_linear_noise={profile['otos_linear_noise']}, "
            f"otos_yaw_noise={profile['otos_yaw_noise']}, "
            f"...+{len(profile) - 4} more knobs)"
        )
        transport = _state.get("transport")
        if transport is not None and is_sim_transport(transport):
            try:
                transport.apply_error_profile(profile)
            except Exception as exc:
                _append_log(f"[ERROR] Failed to apply sim error profile live: {exc}")

    def _on_sim_errors_from_cal() -> None:
        """Populate the Sim Errors spin boxes with the inverse of the active
        robot's calibration, then save + live-apply via ``_on_sim_errors_apply``.

        Issue testgui-sim-errors-from-calibration-button / ticket 070-004: the
        sim firmware bakes the active robot's calibration into
        DefaultConfig.cpp (e.g. rotationalSlip=0.92, trackwidthMm=128.0), but
        the sim plant is ideal (zero scrub) — so PlannerBegin.cpp's rotSlip
        arc-inflation over-rotates RT commands against the ideal plant. This
        button injects a matching plant-side scrub (body rot scrub =
        rotational_slip, trackwidth = geometry.trackwidth) so the firmware's
        correction and the plant's scrub cancel out. Every other knob is set
        to its neutral value (mmPerDeg/OTOS-scale calibrations are inert in
        sim — see the issue's investigated mapping); the three noise fields
        (sim_err_encoder, sim_err_otos_linear, sim_err_otos_yaw) are never
        touched.

        Ticket 073-003: the lookup/fallback (``get_robot_config()`` ->
        ``cfg.geometry.rotational_slip`` / ``cfg.geometry.trackwidth``
        (132-014: moved off the retired ``calibration.rotational_slip``
        path), missing config/fields never crash the panel — they fall
        back to the
        neutral value for that knob and log a [WARN]) is now delegated to
        the shared ``sim_prefs.resolve_calibration_defaults()`` resolver
        (Design Rationale Decision 4), which also backs
        ``load_sim_error_profile()``'s factory-default fallback so a fresh
        TestGUI install reconciles calibration out of the box, not only on
        this button's manual click.
        """
        rot_slip, tw = sim_prefs.resolve_calibration_defaults(log=_append_log)

        sim_err_slip_turn.setValue(0.0)
        sim_err_body_rot_scrub.setValue(rot_slip)
        sim_err_body_lin_scrub.setValue(1.0)
        sim_err_motor_offset_l.setValue(1.0)
        sim_err_motor_offset_r.setValue(1.0)
        sim_err_trackwidth.setValue(tw)
        sim_err_enc_scale_l.setValue(0.0)
        sim_err_enc_scale_r.setValue(0.0)
        sim_err_otos_lin_scale.setValue(0.0)
        sim_err_otos_ang_scale.setValue(0.0)
        sim_err_otos_lin_drift.setValue(0.0)
        sim_err_otos_yaw_drift.setValue(0.0)

        _append_log(
            f"[INFO] Sim error profile set from inverse calibration "
            f"(body_rot_scrub={rot_slip}, trackwidth={tw}, all other "
            f"knobs neutral; noise fields untouched)"
        )

        _on_sim_errors_apply()

    sim_errors_apply_btn.clicked.connect(_on_sim_errors_apply)
    sim_errors_from_cal_btn.clicked.connect(_on_sim_errors_from_cal)
    left_layout.addWidget(sim_errors_group)

    left_layout.addStretch()
    splitter.addWidget(left_widget)

    # --------------------------------------------------------- TraceModel (Qt-free)
    # 097: seed the host-side encoder dead-reckoning trackwidth from the
    # active robot config (``_active_cfg``, resolved above for the robot
    # combo's initial selection) -- falls back to TraceModel's own default
    # (128 mm, the project's usual trackwidth) when no config is resolvable
    # or it carries no trackwidth. Kept live via ``trace_model.
    # set_trackwidth()`` in ``_on_robot_changed`` below.
    _initial_trackwidth = (
        _active_cfg.trackwidth
        if _active_cfg is not None and _active_cfg.trackwidth
        else traces_mod._DEFAULT_TRACKWIDTH
    )
    trace_model = TraceModel(trackwidth=_initial_trackwidth)

    # --------------------------------------------------------------- right panel
    right_widget = QWidget()
    right_layout = QVBoxLayout(right_widget)
    right_layout.setContentsMargins(4, 4, 4, 4)

    # Mode indicator label — updated by _on_transport_changed() below.
    mode_label = QLabel("SIM MODE")
    mode_label.setObjectName("mode_label")
    mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
    _init_text, _init_style = transport_name_to_mode_label(
        transport_combo.currentText()
    )
    mode_label.setText(_init_text)
    mode_label.setStyleSheet(_init_style)
    right_layout.addWidget(mode_label)

    # Shaping-disabled banner (119 ticket 001,
    # kill-the-silent-off-shaping-config-boundary.md) -- the loud off-state
    # indicator for flags bit 16 (kFlagFaultShapingDisabled /
    # TLMFrame.fault_shaping_disabled). Hidden by default; shown by
    # _TelemetryBridge.on_frame_ready() below whenever the freshest drained
    # frame carries the bit, hidden again the frame it clears. See that
    # method's own docstring for the edge-triggered log line this same bit
    # also drives.
    shaping_disabled_banner = QLabel("SHAPING DISABLED — MOVE running unshaped")
    shaping_disabled_banner.setObjectName("shaping_disabled_banner")
    shaping_disabled_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
    shaping_disabled_banner.setStyleSheet(
        "color: white; background-color: #c02020; font-weight: bold; padding: 2px;")
    shaping_disabled_banner.setVisible(False)
    right_layout.addWidget(shaping_disabled_banner)

    # Wheel-frozen banner (129-002, wheel-frozen-fault-flag-in-telemetry.md)
    # -- the stakeholder's own "the test program should be throwing big red
    # errors when it happens" ask. Loud off-state indicator for flags bits
    # 19/20 (kFlagFaultWheelFrozenLeft/Right / TLMFrame.
    # fault_wheel_frozen_left/right). Hidden by default; shown by
    # _TelemetryBridge.on_frame_ready() below, naming which wheel via
    # robot.protocol.wheel_frozen_reason() (the one shared helper both this
    # banner and planner.tour.run_tour()'s own abort-on-flag check read --
    # see that function's own docstring for why it lives in protocol.py, not
    # here).
    wheel_frozen_banner = QLabel("")
    wheel_frozen_banner.setObjectName("wheel_frozen_banner")
    wheel_frozen_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
    wheel_frozen_banner.setStyleSheet(
        "color: white; background-color: #c02020; font-weight: bold; padding: 2px;")
    wheel_frozen_banner.setVisible(False)
    right_layout.addWidget(wheel_frozen_banner)

    right_splitter = QSplitter(Qt.Orientation.Vertical)
    right_layout.addWidget(right_splitter)

    # Playfield canvas — replaces the ticket-005 placeholder QGraphicsView.
    # The image is aligned to the TOP of its viewport (was centred): the
    # playfield rides up under the mode label and any letterbox slack collects
    # below it, where the telemetry panel and console now live.
    canvas_widget, canvas_ctrl = build_canvas(trace_model)
    _canvas_view = canvas_widget.findChild(QWidget, "canvas_view")
    if _canvas_view is not None:
        _canvas_view.setAlignment(
            Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop
        )
    # Wrap the playfield canvas in a tabbed panel: "Playfield" tab + live
    # turn/drive graph tabs (Wheel speed / Wheel position / Heading /
    # Distance), plus a Clear button. Fed by the telemetry + camera hooks
    # below (graph_panel.add_tlm / add_camera). Inserted at splitter index 0
    # so the stretch factors / setSizes below still apply.
    graph_panel = TurnGraphPanel(playfield_widget=canvas_widget)
    right_splitter.addWidget(graph_panel)

    # Turn-control socket (turn_control.py) — lets an external process drive
    # turns / send config / pull the recorded traces THROUGH the running GUI
    # (single relay owner). send_line/clear/get are marshalled onto this Qt
    # main thread by the server's QObject bridge.
    def _control_send(wire_line: str) -> str:
        transport = _state.get("transport")
        if transport is None:
            return "not connected"
        try:
            return transport.command(wire_line, read_timeout=800) or "OK"
        except Exception as exc:  # noqa: BLE001
            return f"error: {exc}"

    _turn_control = TurnControlServer(
        send_line=_control_send,
        clear_fn=graph_panel.clear,
        get_series=lambda: graph_panel.recorder.series,
    )
    _control_port = _turn_control.start()
    _state["turn_control"] = _turn_control  # keep a reference alive
    if _control_port:
        _log.info("turn-control socket listening on 127.0.0.1:%d", _control_port)

    # Parsed-telemetry breakout panel — sits between the canvas and the
    # console.  Fed structured TLMFrames by on_frame_ready(); the raw TLM
    # wire lines are filtered out of the console (see _append_log).
    # 110-002: share the SAME TurnTraceRecorder the top graph tabs
    # (graph_panel) own -- the telemetry pane's rolling 10-second strip
    # charts are a windowed VIEW over it, not a second recorder/consumer.
    telemetry_widget, telemetry_ctrl = build_telemetry_panel(recorder=graph_panel.recorder)
    right_splitter.addWidget(telemetry_widget)

    # Log pane (QPlainTextEdit) — receives timestamped TX/RX lines, minus the
    # telemetry frames (those are broken out in the panel above).
    # "Serial" tab — timestamped TX/RX wire lines (the existing serial log).
    log_pane = QPlainTextEdit()
    log_pane.setObjectName("log_pane")
    log_pane.setReadOnly(True)
    log_pane.setPlaceholderText("(serial log will appear here)")

    # "Console" tab — the simulation program's OWN stdout/stderr (cout/cerr) and
    # the Test-button rebuild output, captured off fd 1/2 (_install_stdout_capture).
    sim_console = QPlainTextEdit()
    sim_console.setObjectName("sim_console")
    sim_console.setReadOnly(True)
    sim_console.setMaximumBlockCount(5000)
    sim_console.setPlaceholderText("(simulation stdout/stderr will appear here)")

    # Same spot as the old serial terminal, now a tab pair: Serial | Console
    # (stakeholder 2026-07-18 -- one panel, a tab to pick which stream).
    console_tabs = QTabWidget()
    console_tabs.setObjectName("console_tabs")
    console_tabs.addTab(log_pane, "Serial")
    console_tabs.addTab(sim_console, "Console")
    console_tabs.setMinimumHeight(160)
    right_splitter.addWidget(console_tabs)

    # Vertical proportions: playfield gets the top third-plus, the telemetry
    # panel takes its natural compact height, and the console tabs get a
    # generous share.  The canvas and console stretch on resize; telemetry fixed.
    right_splitter.setStretchFactor(0, 5)   # canvas
    right_splitter.setStretchFactor(1, 0)   # telemetry panel (fixed)
    right_splitter.setStretchFactor(2, 4)   # console tabs
    _tlm_h = telemetry_widget.sizeHint().height()
    _canvas_h = max(360, int((win_h - 60 - _tlm_h) * 0.58))
    _console_h = max(240, win_h - 60 - _tlm_h - _canvas_h)
    right_splitter.setSizes([_canvas_h, _tlm_h, _console_h])

    splitter.addWidget(right_widget)

    # Initial splitter proportions: the left controls column gets its
    # natural ~900 px (fits the sprint-075 three-column Sim Errors panel);
    # ALL remaining width goes to the playfield pane.  Stretch factors
    # ensure later window growth also flows to the playfield, not the
    # controls column.
    splitter.setStretchFactor(0, 0)
    splitter.setStretchFactor(1, 1)
    left_pane_w = 900
    splitter.setSizes([left_pane_w, max(400, win_w - left_pane_w)])

    # ---------------------------------------------------------------- wiring

    def _append_log(text: str, direction: str | None = None) -> None:
        """Append *text* to the log pane and optionally record it.

        Must be called from the Qt main thread.

        Parameters
        ----------
        text:
            The line to display in the log pane.
        direction:
            ``"TX"`` or ``"RX"`` to route this line through the session
            recorder.  Pass ``None`` (default) for internal GUI status messages
            that should not be recorded.
        """
        # Telemetry (TLM) frames are broken out in the telemetry panel; keep
        # them out of the console so command/reply traffic stays readable.
        # They are still fed to the recorders below so recordings stay complete.
        if not is_telemetry_log_line(text):
            log_pane.appendPlainText(text)
            # Auto-scroll to bottom.
            sb = log_pane.verticalScrollBar()
            sb.setValue(sb.maximum())
        # Route TX/RX lines to both recorders: the manual one (only records
        # when the operator started it) and the always-on latest capture.
        if direction is not None:
            recorder.append(direction, text)  # type: ignore[arg-type]
            latest_recorder.append(direction, text)  # type: ignore[arg-type]

    # ----------------------------------------------------------- recorder controls

    def _on_record_clicked() -> None:
        """Handle Record / Resume button click."""
        if recorder.state == "idle":
            path = recorder.start()
            _append_log(f"[REC] Recording started: {path}")
            record_btn.setEnabled(False)
            record_btn.setText("Record")
            pause_btn.setEnabled(True)
            stop_btn.setEnabled(True)
        elif recorder.state == "paused":
            recorder.resume()
            _append_log("[REC] Recording resumed")
            record_btn.setEnabled(False)
            record_btn.setText("Record")
            pause_btn.setEnabled(True)

    def _on_pause_clicked() -> None:
        """Handle Pause button click."""
        recorder.pause()
        _append_log("[REC] Recording paused")
        record_btn.setText("Resume")
        record_btn.setEnabled(True)
        pause_btn.setEnabled(False)

    def _on_stop_clicked() -> None:
        """Handle Stop button click — finalize the recording file."""
        path = recorder.stop()
        record_btn.setText("Record")
        record_btn.setEnabled(True)
        pause_btn.setEnabled(False)
        stop_btn.setEnabled(False)
        if path is not None:
            _append_log(f"[REC] Recording saved: {path}")

    record_btn.clicked.connect(_on_record_clicked)
    pause_btn.clicked.connect(_on_pause_clicked)
    stop_btn.clicked.connect(_on_stop_clicked)

    def _on_log_from_thread(text: str) -> None:
        """Thread-safe RX log delivery — marshals to the Qt main thread.

        Receives lines from background transport reader threads and posts them
        to the main thread via QMetaObject.invokeMethod with QueuedConnection.
        The actual log-pane append and recorder write happen on the main thread
        inside ``_append_log``.
        """
        # We cannot call _append_log directly here (wrong thread), so we
        # schedule a lambda to be called on the main thread.  Using
        # QMetaObject.invokeMethod on log_pane.appendPlainText is simpler for
        # the display, but we also need the recorder path.  The cleanest
        # solution is to use a dedicated bridge signal (see _RXBridge below).
        _rx_bridge.rx_line.emit(text)  # type: ignore[attr-defined]

    # ---------------------------------------------------------------- telemetry / truth wiring
    # Transport callbacks fire on background threads.  We must marshal to the
    # Qt main thread before touching TraceModel or canvas.

    # Thread-safe queue for TLMFrame objects crossing thread boundary.
    import queue as _queue_mod
    _pending_frames: "_queue_mod.Queue" = _queue_mod.Queue()

    # _pending_secondary/secondary_ready/on_secondary_ready/
    # _on_secondary_thread_v2 -- REMOVED (123-004): this queue-then-signal
    # plumbing existed solely to feed the telemetry panel's "loop" row from
    # TelemetrySecondary's cycle_busy/cycle_period (122-003, interim
    # placement); those fields now ride the primary TLMFrame (already
    # drained via _pending_frames/on_frame_ready above), so this parallel
    # path has no remaining consumer. `transport.on_telemetry_secondary`
    # (transport.py's own general Transport API) is untouched -- a future
    # caller wanting TelemetrySecondary's OTHER fields (cmd_vel/acc_*/
    # glitch_*/ts_*) can still wire it directly.

    # Use a QObject subclass with proper Qt signals to bridge the thread hop
    # safely.  QMetaObject.invokeMethod with a missing slot silently fails, so
    # we use dedicated signals connected with QueuedConnection instead.
    from PySide6.QtCore import QObject, Signal, Slot  # type: ignore[import-untyped]

    class _RXBridge(QObject):
        """Bridges background-thread RX log lines to the Qt main thread.

        The ``rx_line`` signal carries the raw formatted log string across the
        thread boundary; the ``on_rx_line`` slot handles it on the main thread.
        The line's direction (``TX``/``RX``/status) is inferred from its
        ``>``/``<`` marker so the recorder is fed with the correct ``dir`` and
        internal status lines are not recorded.
        """
        rx_line = Signal(str)

        def __init__(self) -> None:
            super().__init__()

        @Slot(str)
        def on_rx_line(self, text: str) -> None:
            """Process a transport log line on the Qt main thread."""
            _append_log(text, direction=direction_from_marker(text))

    _rx_bridge = _RXBridge()
    _rx_bridge.rx_line.connect(_rx_bridge.on_rx_line, Qt.ConnectionType.QueuedConnection)

    # Bridge for captured sim stdout/stderr (fd 1/2). The reader thread in
    # _install_stdout_capture emits `line`; the slot appends it to the console
    # on the Qt main thread. Activated only for the real GUI (main()).
    class _SimOutBridge(QObject):
        line = Signal(str)

        @Slot(str)
        def on_line(self, text: str) -> None:
            sim_console.appendPlainText(text)

    _sim_out_bridge = _SimOutBridge()
    _sim_out_bridge.line.connect(_sim_out_bridge.on_line, Qt.ConnectionType.QueuedConnection)
    if _ENABLE_STDOUT_CAPTURE:
        _install_stdout_capture(_sim_out_bridge)

    class _TelemetryBridge(QObject):
        """Bridges background-thread TLMFrame delivery to the Qt main thread.

        The ``frame_ready`` signal carries a sentinel (int) across the thread
        boundary; the actual frame is retrieved from a shared queue.
        """
        frame_ready = Signal()
        truth_ready = Signal(float, float, float)
        # secondary_ready -- REMOVED (123-004): see _pending_secondary's own
        # removal comment above.

        def __init__(self) -> None:
            super().__init__()

        @Slot()
        def on_frame_ready(self) -> None:
            """Process all pending TLMFrames queued from background threads.

            Always accumulates the fused AND encoder traces in the
            TraceModel (``trace_model.feed()`` -- 097: the latter now
            dead-reckons host-side from ``frame.enc`` when the frame has no
            ``encpose``, see that method's own docstring). In PLAYFIELD MODE
            (``live_view_active``) the avatar is driven by the camera
            live-view worker (:meth:`CanvasController.set_avatar_pose`), so
            the marker is left untouched here (``refresh(update_marker=False)``)
            to avoid fighting the worker for the marker's position — only the
            trace paths redraw at the (faster) TLM rate.

            097: the avatar heading passed to ``refresh()`` prefers
            ``trace_model.encoder_yaw`` (the SAME encoder dead-reckoning
            that now drives the avatar's POSITION, via ``CanvasController.
            _update_marker()``'s own trace-source change) over
            ``frame.pose``'s fused heading -- fused stays pinned at 0 until
            sprint 098, so a fused-only heading would never move either.
            Falls back to fused once the encoder trace has no data yet
            (matches the position fallback in canvas.py).

            110-003 (Sim speed-up factor stutter/breakage fix): ``feed()``/
            ``add_tlm()`` run for EVERY drained frame (cheap accumulation --
            ``TurnGraphPanel.add_tlm()`` only sets a dirty flag a separate
            150ms QTimer redraws from), but ``canvas_ctrl.refresh()`` is
            called AT MOST ONCE per drain, after the loop, from the LAST
            frame's state -- not once per frame inside it. At sim speed
            factor N, ``SimLoop``'s tick thread steps N cycles per
            wall-clock iteration and the (real, unmodified) firmware
            simulator emits one TLM line per cycle
            (``TestSim::SimHarness::step()``), so ``SimLoop.
            _drain_tlm_into_queue()`` delivers all N of a single iteration's
            frames to ``on_telemetry`` back to back before this slot next
            runs -- confirmed by this ticket's own harness
            (``test_sim_speed_factor.py``): burst size scales 1:1 with the
            selected multiplier. Refreshing here per frame meant
            ``canvas_ctrl.refresh()`` -- which REBUILDS every trace's full
            ``QPainterPath`` from scratch each call (``CanvasController.
            _update_traces()``, cost scaling with total accumulated trace
            length) -- ran up to N times per ~50ms iteration instead of
            once, the actual GUI-thread-congestion mechanism behind the
            reported 10x "herky-jerky"/20x "broken" symptom (not sim-side
            mistiming -- the sim's own pacing is unchanged by this fix; see
            the ticket file's own diagnosis). Coalescing to one refresh per
            drained burst removes the redundant, discarded intermediate
            redraws entirely -- the final on-screen state (drawn from the
            last frame's data, with every frame's data already fed into the
            TraceModel/graph panel first) is identical to before, just
            without repainting N-1 throwaway times.

            119 ticket 001 (kill-the-silent-off-shaping-config-boundary.md):
            every drained frame's ``fault_shaping_disabled`` (flags bit 16)
            is checked for an EDGE transition against
            ``_state["shaping_disabled_active"]`` -- logged via
            ``_append_log()`` only on the transition (set->clear or
            clear->set), never every frame, so a MOVE that legitimately
            runs unshaped for its whole duration does not flood the log at
            cycle rate. The ``shaping_disabled_banner`` label's visibility
            instead tracks the LAST drained frame's own state (a level, not
            an edge) -- an honest snapshot of "is this true right now" even
            if this burst never itself crossed an edge.

            129-002 (wheel-frozen-fault-flag-in-telemetry.md): every drained
            frame's ``wheel_frozen_reason()`` (flags bits 19/20) gets the
            SAME edge-triggered-log / level-set-banner treatment as
            ``fault_shaping_disabled`` above, via ``_state
            ["wheel_frozen_reason"]`` and ``wheel_frozen_banner``.
            """
            last_frame = None
            avatar_yaw_rad = None
            any_frame = False
            while True:
                try:
                    frame = _pending_frames.get_nowait()
                except Exception:
                    break
                any_frame = True
                last_frame = frame
                trace_model.feed(frame)
                graph_panel.add_tlm(time.monotonic(), frame)
                avatar_yaw_rad = trace_model.encoder_yaw
                if avatar_yaw_rad is None and frame.pose is not None:
                    avatar_yaw_rad = math.radians(frame.pose[2] / 100.0)
                # 119 ticket 001: loud off-state edge log -- see this
                # method's own docstring addition above.
                shaping_disabled_now = bool(getattr(frame, "fault_shaping_disabled", False))
                if shaping_disabled_now != _state.get("shaping_disabled_active", False):
                    _state["shaping_disabled_active"] = shaping_disabled_now
                    if shaping_disabled_now:
                        _append_log(
                            "[SHAPE] flags bit 16 (kFlagFaultShapingDisabled) SET -- MOVE "
                            "active with shaping/anticipation OFF on both axes; land-at-zero "
                            "cannot fire, threshold/timeout backstop is the only completion path"
                        )
                    else:
                        _append_log("[SHAPE] flags bit 16 cleared -- shaping active again")
                # 129-002: loud off-state edge log for a frozen wheel -- see
                # this method's own docstring addition above.
                wheel_frozen_now = wheel_frozen_reason(frame)
                if wheel_frozen_now != _state.get("wheel_frozen_reason"):
                    _state["wheel_frozen_reason"] = wheel_frozen_now
                    if wheel_frozen_now is not None:
                        _append_log(
                            f"[WHEEL] {wheel_frozen_now} wheel FROZEN -- commanded to move, "
                            "encoder not advancing (flags bit 19/20, gated wedge-suspect)"
                        )
                    else:
                        _append_log("[WHEEL] wheel-frozen fault cleared")
            if any_frame:
                shaping_disabled_banner.setVisible(_state.get("shaping_disabled_active", False))
                wheel_frozen_reason_now = _state.get("wheel_frozen_reason")
                if wheel_frozen_reason_now is not None:
                    wheel_frozen_banner.setText(f"WHEEL FROZEN: {wheel_frozen_reason_now}")
                wheel_frozen_banner.setVisible(wheel_frozen_reason_now is not None)
                if _state.get("live_view_active"):
                    canvas_ctrl.refresh(update_marker=False)
                else:
                    canvas_ctrl.refresh(avatar_yaw_rad)
            # Refresh the parsed-telemetry breakout with the freshest frame.
            if last_frame is not None:
                if last_frame.encpose is None:
                    # Binary TLM carries no encpose field (096-001's trim) --
                    # surface the SAME host-side dead-reckoned pose the
                    # canvas avatar draws (trace_model.feed() above just
                    # ingested this frame, so last_encpose is current), in
                    # the (mm, mm, cdeg) shape the wire field would have had.
                    # Until sprint 098 wires PoseEstimator, this row is the
                    # only live pose in the breakout: the fused pose= stays
                    # pinned at (0, 0, 0) by design.
                    last_frame.encpose = trace_model.last_encpose
                telemetry_ctrl.update_frame(last_frame)

        # on_secondary_ready() -- REMOVED (123-004): see
        # _pending_secondary's own removal comment above -- the telemetry
        # panel's loop-timing row now refreshes from on_frame_ready()'s own
        # telemetry_ctrl.update_frame(last_frame) call.

        @Slot(float, float, float)
        def on_truth_ready(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
            """Process a ground-truth pose update on the Qt main thread.

            Always accumulates the camera trace in the TraceModel.  In
            PLAYFIELD MODE (live_view_active) the avatar is driven by the
            camera live-view worker, so ``canvas_ctrl.refresh()`` is skipped
            to avoid a redundant redraw that would fight the worker's
            ``set_avatar_pose`` call.
            """
            trace_model.feed_truth(x_cm, y_cm, yaw_rad)
            graph_panel.add_camera(time.monotonic(), x_cm, y_cm, math.degrees(yaw_rad))
            if not _state.get("live_view_active"):
                canvas_ctrl.refresh()

    _bridge = _TelemetryBridge()
    _bridge.frame_ready.connect(_bridge.on_frame_ready, Qt.ConnectionType.QueuedConnection)
    _bridge.truth_ready.connect(_bridge.on_truth_ready, Qt.ConnectionType.QueuedConnection)

    class _WorkerBridge(QObject):
        """Marshals background-worker signals onto the Qt GUI main thread.

        A worker ``QObject`` running on its own ``QThread`` emits
        ``log_line(text, direction)`` and ``finished()``.  Those signals MUST
        be delivered to slots that run on the GUI thread — Qt widget access
        (e.g. ``QPlainTextEdit.appendPlainText``) from any other thread
        segfaults in Qt's text/layout engine.

        Connecting a worker signal directly to a plain Python function with
        ``QueuedConnection`` does NOT achieve that: with no ``QObject``
        receiver, PySide delivers the call on the *worker* thread.  This bridge
        is created on the GUI thread, so its *bound-method* slots — and the
        callbacks they invoke — run on the GUI thread, exactly like the
        existing ``_RXBridge`` / ``_TelemetryBridge`` pattern.

        A fresh bridge is created per worker run and kept alive in ``_state``
        (a dropped reference would silently break the connection).
        """

        def __init__(self, log_cb, finished_cb) -> None:
            super().__init__()
            self._log_cb = log_cb
            self._finished_cb = finished_cb

        @Slot(str, str)
        def on_log(self, text: str, direction: str) -> None:
            self._log_cb(text, direction or None)

        @Slot()
        def on_finished(self) -> None:
            self._finished_cb()

    class _TourRunner(QObject):
        """Runs a pre-programmed tour on a background thread.

        107-003: rewired onto ``planner.tour.run_tour()`` (ticket 002),
        driven directly against the connected ``_HardwareTransport``'s
        ``protocol`` accessor (a raw ``NezhaProtocol``, which satisfies
        ``executor.py``'s ``TwistTransport`` structural protocol as-is).
        No ``D``/``RT`` wire string, no ``binary_bridge.translate_command()``
        call — those verbs' ``segment``/``replace`` envelope arms no longer
        exist on the wire (see ``planner/tour.py``'s own module docstring
        for that history) — and no SNAP-poll idle detection: ``run_tour()``
        already knows synchronously, tick by tick, when a leg finishes.

        Telemetry-drain ownership (architecture-update.md Step 7, Open
        Question 1 — this ticket's own investigation, confirmed on the
        bench): ``_HardwareTransport._reader_loop()`` and ``run_tour()``'s
        own ``StreamingExecutor`` both ultimately drain the SAME underlying
        ``SerialConnection._binary_tlm_queue`` — one non-replayable queue,
        two independent consumers. Left unmanaged, ``_reader_loop()``'s
        faster ~40ms poll starves the executor of the fresh telemetry its
        heading-feedback/fault-bit/overshoot checks need for nearly the
        whole tour. ``run()`` therefore calls ``transport.
        suspend_telemetry_reader()`` before ``run_tour()`` — making the
        tour thread the queue's sole consumer for the run — and forwards
        each frame ``run_tour()``'s own ``row_callback`` hands it straight
        to ``transport.on_telemetry`` itself, the SAME Qt-bridge path
        ``_reader_loop()`` normally uses, so the canvas/avatar keeps
        tracking during a tour. ``transport.resume_telemetry_reader()``
        runs in a ``finally``, handing continuous draining back to
        ``_reader_loop()`` once the tour ends.

        Signals are marshalled to the Qt main thread via QueuedConnection:
        ``log_line(text, direction)`` carries ``[TOUR]`` status narration
        (direction ``""``, not recorded) — the raw ``>``/``<`` wire traffic
        is already logged by the transport itself, so this class must not
        echo it again or every step is duplicated in the console and the
        session recording.  ``finished()`` re-enables the button and joins
        the thread.  Public shape (signals + ``stop()``) is UNCHANGED from
        the pre-107-003 implementation — only ``run()``'s internals
        changed, so ``_make_tour_handler`` needs no changes.

        134-005 added ONE optional constructor argument, ``execution`` (a
        ``planner.tour.TourExecution``), saying how this tour is sequenced:
        the pipelined one-leg lookahead Tour 1 / Tour 2 have always used,
        or the rest-to-rest passive-settle execution "Square" needs. It
        defaults to ``None`` ("whatever ``run_tour()`` defaults to"), so
        every four-argument construction still behaves exactly as before;
        the signals and ``stop()`` are untouched.
        """

        log_line = Signal(str, str)
        finished = Signal()

        def __init__(
            self, transport: "object", state: dict, name: str, steps: list[str],
            execution: "object | None" = None,
        ) -> None:
            super().__init__()
            self._transport = transport
            self._state = state
            self._name = name
            self._steps = steps
            # 134-005: how this tour is SEQUENCED (a
            # ``planner.tour.TourExecution``), separate from its geometry --
            # "Square" is rest-to-rest with a passive dwell, Tour 1/2 keep
            # the one-leg lookahead. ``None`` means "whatever run_tour()
            # defaults to" (the pipelined path), which keeps every existing
            # caller and test double that constructs a _TourRunner with four
            # arguments working unchanged.
            self._execution = execution
            self._stop = False
            # --- per-segment truth, accumulated as the tour runs (OOP
            # 2026-08-05). Every number below is derived from what
            # ``run_tour()`` already hands its own ``on_leg``/``row_callback``
            # hooks -- ``TourLegResult.heading_before``/``heading_after`` for
            # angles, ``TLMFrame.pose`` (via the tour module's OWN
            # ``_frame_pose_rad()``) for positions. There is deliberately no
            # second measurement path: the closure this class reports at the
            # end is ``run_tour()``'s, and the per-leg lines are its inputs.
            self._leg_reports: list[dict] = []
            self._latest_pose: "tuple[float, float, float] | None" = None  # [mm, mm, rad]
            self._boundary_pose: "tuple[float, float, float] | None" = None  # [mm, mm, rad]
            self._heading_origin: float | None = None  # [rad] heading at leg 1's enqueue
            self._cumulative_command: float = 0.0  # [deg] commanded turn total so far
            self._frame_pose = None  # bound in run() to planner.tour._frame_pose_rad

        def stop(self) -> None:
            """Request the tour abort at the next safe point (thread-safe).

            Propagates to ``run_tour()``'s own ``should_stop`` hook (ticket
            002), polled once per tick — ``run_tour()`` stops the CURRENT
            leg via ``StreamingExecutor.stop_now()`` (immediate, no replan)
            and reports it as ``RunOutcome.STOPPED``; no further leg is
            attempted.
            """
            self._stop = True

        @Slot()
        def run(self) -> None:
            """Parse the tour's geometry and drive it via ``run_tour()``
            (runs on the worker thread)."""
            from robot_radio.config.robot_config import get_robot_config
            from robot_radio.planner.heading import HeadingCorrector
            from robot_radio.planner.model import PlannerParams
            import math as _math
            import time as _time

            try:
                # testgui-motion-paths-dead-after-move-cutover fix (2026-07-22
                # revival): planner.tour was dormant for a while -- its module
                # body used to raise AttributeError at import time
                # (referencing telemetry_pb2.ACK_STATUS_DONE, deleted by
                # 115-003's frame-v2 rewrite) -- now ported onto protocol
                # v4's Move/single-ack-slot shape (see that module's own file
                # header) and imports cleanly again. This import previously
                # sat ABOVE this try block, so the AttributeError propagated
                # out of run() uncaught -- finished() never emitted, the tour
                # button never re-enabled, and nothing was logged: a silent
                # worker-thread death. The guard stays (any FUTURE import
                # failure -- e.g. a broken checkout -- still fails a tour
                # press VISIBLY instead of silently, same as before) even
                # though the happy path now runs through it every time.
                try:
                    from robot_radio.planner.tour import (
                        _frame_pose_rad, parse_tour, run_tour,
                    )
                except Exception as exc:  # noqa: BLE001 -- see comment above
                    self.log_line.emit(
                        f"[TOUR] {self._name}: tour geometry unavailable "
                        f"({exc}) -- see clasi/issues/"
                        "testgui-motion-paths-dead-after-move-cutover.md",
                        "")
                    return

                protocol = getattr(self._transport, "protocol", None)
                if protocol is None:
                    self.log_line.emit(
                        f"[TOUR] {self._name}: transport has no live "
                        "protocol (not connected?) — aborting", "")
                    return

                try:
                    legs = parse_tour(self._steps)
                except ValueError as exc:
                    self.log_line.emit(
                        f"[TOUR] {self._name}: bad tour geometry: {exc}", "")
                    return

                if self._stop:
                    self.log_line.emit(f"[TOUR] {self._name} aborted", "")
                    return

                execution = self._execution
                sequential = bool(getattr(execution, "sequential", False))
                settle = float(getattr(execution, "settle", 0.0) or 0.0)
                omega_max = getattr(execution, "omega_max", None)

                self.log_line.emit(
                    f"[TOUR] {self._name} starting — {len(legs)} legs"
                    + (f", rest-to-rest ({settle:.1f}s passive settle per boundary)"
                       if sequential else ""), "")

                params = PlannerParams()
                heading = HeadingCorrector(params, robot_config=get_robot_config())

                # Per-segment accounting -- reset here so a re-run of the
                # same worker starts clean, and bind the tour module's own
                # pose reader (see __init__'s note on the single measurement
                # path).
                self._frame_pose = _frame_pose_rad
                self._leg_reports = []
                self._latest_pose = None
                self._boundary_pose = None
                self._heading_origin = None
                self._cumulative_command = 0.0
                tour_start = _time.monotonic()

                self._transport.suspend_telemetry_reader()
                try:
                    result = run_tour(
                        protocol, params, heading, legs,
                        sequential=sequential,
                        settle=settle,
                        omega_max=omega_max,
                        on_leg=self._on_leg,
                        row_callback=self._on_row,
                        should_stop=lambda: self._stop,
                    )
                except Exception as exc:  # noqa: BLE001
                    self.log_line.emit(f"[TOUR] {self._name} error: {exc}", "")
                    return
                finally:
                    self._transport.resume_telemetry_reader()

                if result.stopped_at is not None:
                    self.log_line.emit(
                        f"[TOUR] {self._name} stopped at leg "
                        f"{result.stopped_at + 1}/{len(legs)} "
                        f"({result.stopped_outcome.value})", "")
                else:
                    closure = result.closure
                    if closure.position_delta is not None:
                        self.log_line.emit(
                            f"[TOUR] {self._name} complete — closure: "
                            f"{closure.position_delta:.1f}mm, "
                            f"{_math.degrees(closure.heading_delta):.1f}deg", "")
                    else:
                        self.log_line.emit(f"[TOUR] {self._name} complete", "")

                # Whole-tour summary. Emitted for a stopped tour too -- the
                # legs that DID run are the ones worth reading when a tour
                # ends early, and suppressing them there would hide exactly
                # the run an operator most wants to look at.
                self._emit_summary(len(legs), result.closure,
                                   _time.monotonic() - tour_start)
            finally:
                self.finished.emit()

        def _on_leg(self, index, total, leg, leg_result) -> None:
            """``run_tour()``'s per-leg narration hook — fires synchronously
            on the worker thread as each leg completes.

            Reports THIS segment's own truth, not just that it retired: a
            straight leg's achieved length and cross-track deviation, a turn
            leg's achieved angle, error, and the cumulative heading residual
            that turn leaves behind. That is the per-segment attribution the
            bench scripts print (``src/tests/bench/planner_square_tour.py``'s
            "per-leg length / per-turn angle / cumulative residual" block)
            and it is what makes an imperfect closure diagnosable instead of
            merely reported — a tour that closes 30 mm because ONE corner
            under-turned looks nothing like one that bleeds 4 mm per leg,
            and the closure number alone cannot tell them apart.

            Angles come from ``TourLegResult.heading_before``/
            ``heading_after`` — ``run_tour()``'s own ``HeadingCorrector``
            readings, the same source it corrects against; positions come
            from ``TLMFrame.pose`` via the tour module's own
            ``_frame_pose_rad()``, the same field its closure uses.
            """
            import math as _math

            from robot_radio.testgui.commands import unwrap_angle_toward

            pose_before = self._boundary_pose
            pose_after = self._latest_pose
            self._boundary_pose = pose_after

            heading_before = leg_result.heading_before  # [rad] absolute since boot
            heading_after = leg_result.heading_after  # [rad] absolute since boot
            if self._heading_origin is None:
                self._heading_origin = heading_before

            commanded = float(leg.value)  # [mm] signed, or [deg] signed
            achieved = None
            error = None
            cross_track = None  # [mm] signed, distance legs only
            residual = None  # [deg] signed, turn legs only
            turned = None  # [deg] signed in-leg heading change, distance legs

            if leg.kind == "turn":
                self._cumulative_command += commanded
                if heading_before is not None and heading_after is not None:
                    achieved = unwrap_angle_toward(
                        _math.degrees(heading_after - heading_before), commanded)
                    error = achieved - commanded
                if heading_after is not None and self._heading_origin is not None:
                    swept = unwrap_angle_toward(
                        _math.degrees(heading_after - self._heading_origin),
                        self._cumulative_command)
                    residual = swept - self._cumulative_command
            else:
                # `pose_before == pose_after` means no telemetry pose landed
                # between this leg's own boundaries, so the pair measures
                # nothing -- report it as unmeasured rather than as a 0 mm
                # leg. It happens on leg 1, whose entry pose is seeded from
                # the first frame of the run (see `_on_row()`); `_emit_
                # summary()` re-bases that leg on `run_tour()`'s own closure
                # start pose, which is read before leg 1 is ever sent.
                measured = (pose_before is not None and pose_after is not None
                            and pose_before != pose_after)
                if measured:
                    achieved, cross_track = self._project(pose_before, pose_after)
                    error = achieved - commanded
                if heading_before is not None and heading_after is not None:
                    turned = unwrap_angle_toward(
                        _math.degrees(heading_after - heading_before), 0.0)

            self._leg_reports.append({
                "index": index, "kind": leg.kind, "commanded": commanded,
                "achieved": achieved, "error": error, "cross_track": cross_track,
                "residual": residual, "turned": turned,
                "pose_after": pose_after,
                "outcome": leg_result.outcome.value,
                "duration": leg_result.duration,
            })

            unit = "deg" if leg.kind == "turn" else "mm"
            detail = f"cmd {commanded:+.1f}{unit}"
            if achieved is None:
                detail += "  ach n/a (no telemetry pose yet)"
            else:
                detail += f"  ach {achieved:+.1f}{unit}  err {error:+.2f}{unit}"
                if leg.kind == "turn":
                    if residual is not None:
                        detail += f"  cum resid {residual:+.2f}deg"
                else:
                    detail += f"  xtrack {cross_track:+.1f}mm"
                    if turned is not None:
                        detail += f"  drift {turned:+.2f}deg"

            self.log_line.emit(
                f"[TOUR] {self._name} leg {index + 1}/{total}: "
                f"{leg.kind} {detail} "
                f"({leg_result.outcome.value}, {leg_result.duration:.2f}s, "
                f"{leg_result.tick_count} ticks)",
                "")

        @staticmethod
        def _project(entry_pose, exit_pose) -> "tuple[float, float]":
            """Signed along-track travel and perpendicular (cross-track)
            deviation [mm] between two poses, resolved in the heading the
            robot ENTERED the leg with.

            Along-track alone cannot tell a leg that ran long from one that
            arced; the pair can. Both are what the playfield rule asks a
            multi-segment run to report per leg.
            """
            import math as _math

            dx = exit_pose[0] - entry_pose[0]  # [mm]
            dy = exit_pose[1] - entry_pose[1]  # [mm]
            entry = entry_pose[2]  # [rad]
            return (dx * _math.cos(entry) + dy * _math.sin(entry),
                    -dx * _math.sin(entry) + dy * _math.cos(entry))

        def _emit_summary(self, total: int, closure, wall: float) -> None:
            """Compact end-of-tour block: every segment's number in one
            place, plus the whole-tour sweep and closure.

            Deliberately a handful of lines, not one line per leg again —
            the per-leg narration already streamed by as the tour ran; this
            is the shape an operator scans afterwards to see WHICH segment
            spent the error.
            """
            import math as _math

            reports = self._leg_reports
            if not reports:
                return

            # Re-base leg 1 on the tour's OWN start pose. `run_tour()` reads
            # that pose just before leg 1 is sent and only publishes it at
            # the end, so the streaming line for leg 1 had to make do with
            # the first frame of the run -- which on a starved first leg is
            # the SAME frame the leg ends on, and measures nothing. This is
            # the correct basis, and it is the same field the closure below
            # is computed from, so the two agree by construction.
            #
            # If leg 1's exit pose is still that same start pose, the leg has
            # no measurable span at all and stays `None` -> "n/a". That is
            # the honest reading, not a 0 mm leg: on the PIPELINED path leg
            # 1's terminal ack can land within a couple of polls (measured:
            # 0.05s / 2 ticks for a 345 mm leg), so its boundaries collapse
            # and the travel is really spent inside the next leg's window.
            # Printing "0.0 mm, err -345.0" there would invent a huge error
            # the robot did not make. Rest-to-rest runs do not have this
            # problem -- every boundary is a real rest.
            first = reports[0]
            start_pose = getattr(closure, "start_pose", None)
            if (start_pose is not None and first["kind"] == "distance"
                    and first["pose_after"] is not None
                    and first["pose_after"] != start_pose):
                achieved, cross_track = self._project(start_pose, first["pose_after"])
                first["achieved"] = achieved
                first["cross_track"] = cross_track
                first["error"] = achieved - first["commanded"]

            def _row(values, fmt) -> str:
                return " ".join("  n/a" if v is None else format(v, fmt) for v in values)

            def _command_note(values, fmt) -> str:
                if len({round(v, 3) for v in values}) == 1:
                    return f"cmd {format(values[0], fmt)} each"
                return "cmd " + " ".join(format(v, fmt) for v in values)

            name = self._name
            self.log_line.emit(
                f"[TOUR] {name} summary — {len(reports)}/{total} legs, {wall:.1f}s wall", "")

            distances = [r for r in reports if r["kind"] == "distance"]
            turns = [r for r in reports if r["kind"] == "turn"]

            if distances:
                self.log_line.emit(
                    f"[TOUR]   leg length [mm]:  {_row([r['achieved'] for r in distances], '7.1f')}"
                    f"   ({_command_note([r['commanded'] for r in distances], '.1f')};"
                    f" err {_row([r['error'] for r in distances], '+.1f')})", "")
                self.log_line.emit(
                    f"[TOUR]   leg cross-track [mm]:  "
                    f"{_row([r['cross_track'] for r in distances], '+7.1f')}", "")
                if any(r["achieved"] is None for r in distances):
                    self.log_line.emit(
                        "[TOUR]   (n/a = no telemetry pose landed inside that leg's own "
                        "boundaries — its travel is measured inside the next leg)", "")
            if turns:
                self.log_line.emit(
                    f"[TOUR]   turn angle [deg]:  {_row([r['achieved'] for r in turns], '+7.1f')}"
                    f"   ({_command_note([r['commanded'] for r in turns], '+.1f')};"
                    f" err {_row([r['error'] for r in turns], '+.2f')})", "")
                self.log_line.emit(
                    f"[TOUR]   cumulative heading residual per corner [deg]:  "
                    f"{_row([r['residual'] for r in turns], '+7.2f')}", "")
                swept = [r for r in turns if r["residual"] is not None]
                if swept:
                    total_command = sum(r["commanded"] for r in turns)  # [deg]
                    total_sweep = total_command + swept[-1]["residual"]  # [deg]
                    self.log_line.emit(
                        f"[TOUR]   heading sweep {total_sweep:+.1f}deg "
                        f"(cmd {total_command:+.1f}deg, "
                        f"err {swept[-1]['residual']:+.2f}deg)", "")

            if closure is not None and closure.position_delta is not None:
                start = closure.start_pose
                end = closure.end_pose
                self.log_line.emit(
                    f"[TOUR]   closure {closure.position_delta:.1f}mm "
                    f"(finish at {end[0] - start[0]:+.1f}, {end[1] - start[1]:+.1f}), "
                    f"heading delta {_math.degrees(closure.heading_delta):+.1f}deg", "")

        def _on_row(self, tick_index, leg_index, leg, tick_result, frame) -> None:
            """``run_tour()``'s per-tick hook — forwards the frame the
            executor just drained to the SAME ``on_telemetry`` Qt-bridge
            path ``_reader_loop()`` normally feeds, so the canvas/avatar
            keeps tracking while ``_reader_loop()`` itself is suspended
            (see this class's own docstring).

            Also keeps the freshest pose for ``_on_leg()``'s per-segment
            reporting. The pose captured on the FIRST tick stands in as leg
            1's own entry pose: ``run_tour()`` reads its closure start pose
            just before sending leg 1 and does not expose it until the tour
            ends, and one poll interval (~50 ms) into a leg is close enough
            to that instant to report leg 1 on the same terms as every
            later leg, which use a real boundary pose.
            """
            if frame is None:
                return
            if self._frame_pose is not None:
                pose = self._frame_pose(frame)
                if pose is not None:
                    self._latest_pose = pose
                    if self._boundary_pose is None:
                        self._boundary_pose = pose
            on_telemetry = self._transport.on_telemetry
            if on_telemetry is not None:
                try:
                    on_telemetry(frame)
                except Exception:  # noqa: BLE001
                    pass

    class _GotoRunner(QObject):
        """Camera-based GOTO — drives to a world point via repeated ``G`` moves.

        Each iteration reads the freshest cached camera ground-truth pose,
        checks whether the robot is within ``eps`` of the target (→ done, send
        ``STOP``), and otherwise snaps the robot's internal pose to the camera
        truth (``SI``) and re-issues a firmware ``G`` toward the fixed target.
        This is a camera-in-the-loop pure pursuit that corrects for odometry
        drift; the throttle keeps it from spamming the link.

        Runs on a background thread; ``log_line(text, direction)`` and
        ``finished()`` marshal to the Qt main thread (like ``_TourRunner``).
        Target/eps are in mm; speed in mm/s (matching the firmware ``G`` verb).
        """

        log_line = Signal(str, str)
        finished = Signal()

        #: Throttle (s) between pursuit iterations — "not as fast as it can".
        POLL_S = 0.3
        #: Max age (s) of a cached truth pose before it is considered stale.
        TRUTH_MAX_AGE_S = 2.0
        #: Overall timeout (s) before giving up.
        TIMEOUT_S = 60.0

        def __init__(
            self,
            transport: "object",
            state: dict,
            target_x: int,  # [mm]
            target_y: int,  # [mm]
            eps: int,  # [mm]
            speed: int,
        ) -> None:
            super().__init__()
            self._transport = transport
            self._state = state
            self._tx = target_x
            self._ty = target_y
            self._eps = eps
            self._speed = speed
            self._stop = False

        def stop(self) -> None:
            """Request the GOTO abort at the next safe point (thread-safe)."""
            self._stop = True

        @Slot()
        def run(self) -> None:
            """Run the pursuit loop (on the worker thread)."""
            import time

            self.log_line.emit(
                f"[GOTO] target=({self._tx}, {self._ty}) mm, eps={self._eps} mm, "
                f"speed={self._speed} mm/s",
                "",
            )
            deadline = time.monotonic() + self.TIMEOUT_S
            last_status = 0.0
            try:
                while not self._stop:
                    now = time.monotonic()
                    if now > deadline:
                        self.log_line.emit("[GOTO] timed out — aborting", "")
                        self._safe_stop()
                        return

                    truth = self._state.get("last_truth")
                    if truth is None or (now - truth[3]) > self.TRUTH_MAX_AGE_S:
                        self.log_line.emit(
                            "[GOTO] waiting for a fresh camera pose...", ""
                        )
                        time.sleep(self.POLL_S)
                        continue

                    x_cm, y_cm, yaw_rad, _ = truth
                    cur_x = x_cm * 10.0  # [mm]
                    cur_y = y_cm * 10.0  # [mm]

                    if goto_reached(self._tx, self._ty, cur_x, cur_y, self._eps):
                        self._safe_stop()
                        self.log_line.emit("[GOTO] reached target — complete", "")
                        return

                    # Correct the robot's internal pose to camera truth, then
                    # re-aim at the fixed world target.
                    si = build_setpose_command(x_cm, y_cm, yaw_rad)
                    g = f"G {self._tx} {self._ty} {self._speed}"
                    try:
                        self._transport.command(si, read_timeout=200)
                        self._transport.command(g, read_timeout=200)
                    except Exception as exc:  # noqa: BLE001
                        self.log_line.emit(f"[GOTO] send failed: {exc}", "")
                        return

                    # Throttled progress line (~1 Hz) — the raw SI/G traffic is
                    # visible via the transport, so we summarise here.
                    if now - last_status >= 1.0:
                        dist = goto_distance(
                            self._tx, self._ty, cur_x, cur_y
                        )
                        self.log_line.emit(f"[GOTO] dist={dist:.0f} mm", "")
                        last_status = now

                    time.sleep(self.POLL_S)
                # Loop exited due to stop request.
                self.log_line.emit("[GOTO] aborted", "")
                self._safe_stop()
            finally:
                self.finished.emit()

        def _safe_stop(self) -> None:
            """Halt-now (estop) the robot -- best-effort cleanup call from a
            worker thread (loop exit / exception path).

            128-003: rewired off the dead ``transport.send()`` call sending
            the wire verb ``STOP`` (routed through ``binary_bridge.
            translate_command()``'s unconditional stub on hardware, and
            even where it "worked" meant the PLANNED stop, not halt-now)
            onto ``transport.halt()`` directly. A raised failure is logged
            loudly -- never swallowed on faith -- but does not propagate
            past this cleanup call.
            """
            try:
                self._transport.halt()
                self.log_line.emit("[INFO] estop sent -- motion halted", "")
            except Exception as exc:  # noqa: BLE001
                self.log_line.emit(
                    f"[ERROR] HALT FAILED -- ROBOT MAY STILL BE MOVING: {exc}", "")

    def _on_telemetry_thread_v2(frame: "object") -> None:
        """Transport on_telemetry callback — fires on the reader/tick thread.

        Enqueues the frame and emits the bridge signal to wake the Qt main
        thread.  Also caches the freshest frame (with a monotonic timestamp)
        in ``_state["last_tlm"]`` so the tour worker can poll motion state
        without a synchronous ``SNAP``: a SNAP reply is a corr-id-less TLM
        frame that ``command()``'s reply queue never receives, but it *does*
        flow through this callback, so the tour reads mode from here instead.
        """
        import time as _time
        _state["last_tlm"] = (frame, _time.monotonic())
        _pending_frames.put(frame)
        _bridge.frame_ready.emit()  # type: ignore[attr-defined]

    # _on_secondary_thread_v2() -- REMOVED (123-004): see
    # _pending_secondary's own removal comment above.

    def _on_truth_thread(pose: "tuple | None") -> None:
        """Transport on_truth callback — fires on the truth/tick thread.

        Ignores ``None`` (camera not available); emits bridge signal for a
        valid pose.
        """
        if pose is not None:
            x_cm, y_cm, yaw_rad = pose
            # Cache the freshest truth pose (with a monotonic timestamp) so the
            # GOTO worker can read it without opening its own daemon session.
            import time as _time
            _state["last_truth"] = (x_cm, y_cm, yaw_rad, _time.monotonic())
            _bridge.truth_ready.emit(x_cm, y_cm, yaw_rad)  # type: ignore[attr-defined]

    def _stop_live_worker() -> None:
        """Stop the live-view worker and thread, then restore the static background.

        Safe to call when no worker is running (no-op in that case).
        """
        if not _state.get("live_view_active"):
            return
        worker = _state.get("live_worker")
        thread = _state.get("live_thread")
        if worker is not None:
            try:
                worker.stop()
            except Exception:
                pass
        if thread is not None:
            try:
                thread.quit()
                thread.wait(3000)
            except Exception:
                pass
        _state["live_worker"] = None
        _state["live_thread"] = None
        # Drop the _LiveFrameBridge reference alongside the worker/thread
        # (ticket 063-009) — nothing else holds it once the connection is
        # torn down.
        _state["live_bridge"] = None
        _state["live_view_active"] = False
        canvas_ctrl.restore_static_background()

    def _on_transport_changed(index: int) -> None:
        """Enable/disable port picker and update mode label for selected transport."""
        name = transport_combo.currentText()
        hardware = name in ("Serial", "Relay")
        port_edit.setEnabled(hardware)
        port_label.setEnabled(hardware)
        text, style = transport_name_to_mode_label(name)
        mode_label.setText(text)
        mode_label.setStyleSheet(style)
        # Sim Errors panel only makes sense for the Sim transport (issue
        # testgui-sim-error-profile-config).
        sim_errors_group.setVisible(name == "Sim")
        # Same for the sim fast-forward selector.
        sim_speed_label.setVisible(name == "Sim")
        sim_speed_combo.setVisible(name == "Sim")

    transport_combo.currentIndexChanged.connect(_on_transport_changed)
    # Trigger once to set initial state.
    _on_transport_changed(transport_combo.currentIndex())

    def _apply_sim_speed() -> None:
        """Push the selected fast-forward factor to a connected SimTransport.

        No-op for hardware transports (real robots run at real time) and
        when disconnected — the factor is re-applied on the next Sim
        connect, so a pre-connect selection is honored.
        """
        transport = _state.get("transport")
        if isinstance(transport, SimTransport):
            transport.set_speed_factor(int(sim_speed_combo.currentData()))

    sim_speed_combo.currentIndexChanged.connect(lambda _i: _apply_sim_speed())

    def _push_robot_calibration() -> None:
        """Push the active robot config's calibration to the connected robot.

        Selecting a robot must be authoritative: every calibration value the
        firmware exposes as a SET key is overwritten from the robot's JSON,
        so whatever calibration is compiled into the firmware
        (DefaultConfig.cpp — in Sim mode that is tovez's rotationalSlip=0.92)
        never silently leaks into a run.  An uncalibrated config ("tovez
        nocal") pushes NEUTRAL values — in particular ``SET rotSlip=0`` (the
        documented no-correction sentinel) — which is what makes
        "no-calibration robot + zero sim errors = geometry-pure turns" hold.
        Works identically for Sim and hardware transports.

        No-op when not connected; the push re-runs on the next Connect (and
        on every robot change while connected).

        113-006 (SUC-003): for a connected ``SimTransport``, this ALSO
        triggers the Tier-2 (boot-only motor fields -- travel_calib/
        fwd_sign/vel_filt_alpha; the planner half of this tier was DELETED,
        115-003, gut S1 motion-stack excision: nothing in the S1 minimal
        firmware reads a boot-loaded ``msg::PlannerConfig`` any more) push
        via ``transport.configure_from_robot(cfg)``
        after the per-command loop below completes. The per-command loop
        itself stays -- it is NOT redundant with ``configure_from_robot()``:
        it is what pushes the OTOS ``OI``/``OL``/``OA`` sequence (which
        neither Tier 1 nor Tier 2 of ``configure_from_robot()`` covers --
        109-004's OTOS calibration path is a separate mechanism, out of
        this ticket's scope, see sprint 113's own Out of Scope) and what
        populates the host-side ``GET`` echo cache
        (``SimTransport._config_echo``, read by ``_handle_config_get()``) --
        ``configure_from_robot()``'s own Tier-1 route goes straight through
        ``NezhaProtocol.set_config()``/``SimConfigConn``, bypassing
        ``SimTransport._handle_config_set()`` entirely, so it never touches
        that cache. The net effect is Tier 1 gets pushed twice for a Sim
        transport (once per-command here, once again inside
        ``configure_from_robot()``) -- harmless, repushing the same values
        (see this ticket's own Idempotency note), and the only way to add
        Tier 2 coverage without touching ``SimLoop``'s Tier-1/Tier-2-combined
        ``configure_from_robot()`` (out of this ticket's file scope).
        Hardware transports have no Tier 2/``configure_from_robot``
        equivalent (their boot config comes from their own compiled
        reflash) -- the extra call below is skipped for them, so this
        function's behavior for hardware is unchanged.
        """
        transport = _state.get("transport")
        if transport is None:
            return
        cfg = get_robot_config()
        if cfg is None:
            _append_log("[CAL] no active robot config — calibration push skipped")
            return
        from robot_radio.calibration.push import calibration_commands

        cmds = calibration_commands(cfg)
        n_bad = 0
        n_nodev = 0
        for cmd, read_timeout in cmds:
            try:
                reply = transport.command(cmd, read_timeout=read_timeout)
            except Exception as exc:  # noqa: BLE001 — log, don't kill the GUI
                _append_log(f"[CAL] push failed at {cmd!r}: {exc}")
                return
            upper = (reply or "").upper()
            if "NODEV" in upper:
                # Physical-device command (OI/OL/OA) on a target without that
                # device — normal in Sim mode, where the OTOS *model* is
                # configured via SIMSET instead. Skip quietly.
                n_nodev += 1
            elif "ERR" in upper:
                n_bad += 1
                _append_log(f"[CAL] {cmd!r} rejected: {(reply or '').strip()}")
        _append_log(
            f"[CAL] pushed {len(cmds) - n_bad - n_nodev}/{len(cmds)} "
            f"calibration values from robot '{cfg.robot_name}'"
            + (f" ({n_nodev} device cmds skipped: no device)" if n_nodev else "")
            + (f" ({n_bad} REJECTED)" if n_bad else "")
        )

        # wire-testgui-live-push-of-estimator-stop-lead: the nine estimator/
        # shaper fields (config.estimator.weight_heading_otos/
        # weight_omega_otos/staleness + config.planner.a_max/a_decel/
        # alpha_max/alpha_decel/jerk_max/yaw_jerk_max) live on TWO SEPARATE
        # ConfigGroupTargets with no SET key=value text form --
        # calibration_commands()/calibration_kwargs() above never covered
        # them, so neither of these two SET-key mechanisms would ever push
        # them. Unconditional, same as the Tier-2 push below (both
        # transports) -- see _push_estimator_config()'s own doc comment for
        # why every push here is now an EXPECTED, named rejection (132-014).
        _push_estimator_config(transport, cfg)

        if isinstance(transport, SimTransport):
            try:
                transport.configure_from_robot(cfg)
            except Exception as exc:  # noqa: BLE001 — log, don't kill the GUI
                _append_log(f"[CAL] configure_from_robot (Tier 2) failed: {exc}")
                return
            _append_log(f"[CAL] configured sim Tier 2 from robot '{cfg.robot_name}'")

    def _push_estimator_config(transport: "Transport", cfg: "RobotConfig") -> None:
        """Push *cfg*'s estimator fusion-weight + shaper-ceiling fields
        (``push.estimator_kwargs()``) to *transport*, one
        ``set_config_field()`` round trip per field.

        132-014 RETARGET: the single binary-only ``EstimatorConfigPatch``
        arm (``config.proto``) this function used to build is deleted
        (132-013) -- the same nine fields now split across TWO
        ``ConfigGroupTarget``s (``push.ESTIMATOR_FIELDS``/
        ``push.PLANNER_SHAPER_FIELDS``), addressed via
        ``NezhaProtocol.set_config_field()``. ``ESTIMATOR`` is an honest
        dead end (see ``push.estimator_kwargs()``'s own docstring): it
        decodes but ``install(ESTIMATOR)`` permanently returns
        ``ERR_UNIMPLEMENTED`` (``App::StateEstimator`` was already deleted
        as dead code before this sprint). ``PLANNER_SHAPER`` -- FIXED,
        132-017 (JSON reshape ticket, stakeholder-sanctioned mid-sprint
        scope addition): the six shaper-ceiling fields this function
        pushes now land LIVE again (closing
        ``clasi/issues/wire-testgui-live-push-of-estimator-stop-lead.md``
        for a second time), split out of the boot-only ``PLANNER`` group
        into their own re-appliable ``ConfigGroupTarget`` -- the temporary
        regression sprint 132's own schema unification introduced
        (132-002 through 132-013) is closed. Still pushed and logged every
        Connect/robot-select, both transports -- "selecting a robot must
        be authoritative" and "no silent no-ops" both still apply; a
        caller reading the log sees an honest, named rejection for
        ESTIMATOR and a real apply for PLANNER_SHAPER, instead of the old
        patch's silent "acks 0, lands nowhere."

        ``proto`` is resolved the SAME way for both transport kinds
        (``SimTransport._config_proto`` / ``_HardwareTransport.protocol``)
        -- both already expose ``set_config_field()`` directly (it dispatches
        on ``self._conn`` internally, duck-typed for ``_SimConfigConn`` vs.
        a real ``SerialConnection`` -- no separate ack-poll plumbing needed
        here any more). ``getattr(proto, "set_config_field", None)``, not a
        bare attribute reference: a duck-typed test double standing in for
        a ``NezhaProtocol`` (e.g. test_tour1_geometry.py's own
        ``_FakeTwistTransport``, which only implements the narrower
        ``TwistTransport`` surface) may not have it -- resolving it eagerly
        without this guard crashed ``_on_connect()`` for every test in that
        file, leaving the tour buttons disabled (the SAME hazard the old
        ``wait_for_ack`` guard here used to protect against).

        No-op (logged) when *cfg* carries none of the nine estimator/shaper
        fields, when the config channel is not ready, or when
        ``set_config_field`` is unavailable. Every push result -- applied
        or rejected -- is logged, per this fix's own acceptance: silent
        success/failure here is exactly what let the original gap go
        unnoticed.
        """
        from robot_radio.calibration.push import ESTIMATOR_FIELDS, estimator_kwargs
        from robot_radio.robot.pb2 import robot_config_pb2

        kwargs = estimator_kwargs(cfg)
        if not kwargs:
            _append_log(
                "[CAL] no estimator/shaper fields on active config — push skipped"
            )
            return

        if isinstance(transport, SimTransport):
            proto = transport._config_proto
            if proto is None:
                _append_log(
                    "[CAL] estimator/shaper push skipped — sim config "
                    "channel not ready"
                )
                return
        else:
            proto = transport.protocol
            if proto is None:
                _append_log(
                    "[CAL] estimator/shaper push skipped — no protocol "
                    "on transport"
                )
                return

        set_config_field = getattr(proto, "set_config_field", None)
        if set_config_field is None:
            _append_log(
                "[CAL] estimator/shaper push skipped — protocol has no "
                "set_config_field()"
            )
            return

        applied: "list[str]" = []
        rejected: "list[str]" = []
        for field_name, value in kwargs.items():
            target = (robot_config_pb2.ESTIMATOR if field_name in ESTIMATOR_FIELDS
                      else robot_config_pb2.PLANNER_SHAPER)
            try:
                ack = set_config_field(target, field_name, value)
            except Exception as exc:  # noqa: BLE001 — log, don't kill the GUI
                _append_log(f"[CAL] {field_name} push failed to send: {exc}")
                rejected.append(field_name)
                continue
            (applied if ack is not None else rejected).append(field_name)

        if rejected:
            _append_log(
                f"[CAL] pushed {len(applied)}/{len(kwargs)} estimator/shaper "
                f"fields from robot '{cfg.robot_name}' ({sorted(applied)}) — "
                f"{len(rejected)} rejected ({sorted(rejected)}): ESTIMATOR has "
                f"no live consumer (ERR_UNIMPLEMENTED) as of sprint 132's own "
                f"re-appliability table -- PLANNER_SHAPER fields rejecting "
                f"would be a regression, not expected (132-017)"
            )
        else:
            _append_log(
                f"[CAL] pushed {len(applied)}/{len(kwargs)} estimator/shaper "
                f"fields from robot '{cfg.robot_name}' ({sorted(applied)})"
            )

    def _check_firmware_version(transport: "Transport") -> None:
        """Compare the robot's `VER` firmware version against the host package.

        The 2026-07-03 bench-OTOS session lost hours to a robot silently
        running the previous day's firmware (which predated the sprint-074
        OTOS fusion fixes).  The host knows both sides: the robot answers
        `VER` with `fw=<version>` and the installed package version tracks
        the tree (`dotconfig version bump` keeps pyproject / Protocol.h in
        lockstep), so a mismatch is loud from now on.

        Best-effort: link drops or an unknown host version log an INFO/WARN
        line instead of blocking the connect.
        """
        import re as _re

        try:
            reply = transport.command("VER", read_timeout=600) or ""
        except Exception as exc:  # noqa: BLE001
            _append_log(f"[WARN] VER query failed: {exc}")
            return
        m = _re.search(r"fw=([0-9][^\s#]*)", reply)
        if m is None:
            _append_log(
                "[WARN] VER gave no parseable reply (link drop?) — firmware "
                "version unverified"
            )
            return
        fw = m.group(1)
        try:
            from importlib.metadata import version as _pkg_version

            host_ver = _pkg_version("microbit-v2-samples-tools")
        except Exception:  # noqa: BLE001
            _append_log(f"[INFO] robot firmware {fw} (host version unknown)")
            return
        if fw == host_ver:
            _append_log(f"[INFO] robot firmware {fw} matches host {host_ver}")
        else:
            _append_log(
                f"[WARN] *** ROBOT FIRMWARE {fw} != HOST {host_ver} — reflash "
                "before trusting bench/playfield behaviour (a stale flash "
                "masked the 2026-07-03 bench-OTOS bugs) ***"
            )

    def _on_robot_changed(index: int) -> None:
        """Load the robot selected in the dropdown (reloads on every change).

        When a transport is connected, the new robot's calibration is pushed
        immediately so the firmware always runs the SELECTED robot's values
        (see _push_robot_calibration).
        """
        path = robot_combo.itemData(index)
        if not path:
            return
        try:
            cfg = set_active_robot(path)
        except Exception as exc:  # noqa: BLE001 — surface load errors in the log
            _append_log(f"[ERROR] Failed to load robot: {exc}")
            return
        _append_log(
            f"[INFO] Loaded robot: {cfg.robot_name} "
            f"({cfg.hardware_model}, trackwidth={cfg.trackwidth}mm)"
        )
        # 097: keep the host-side encoder dead-reckoning trackwidth (the
        # avatar's own pose-integration geometry) in sync with the selected
        # robot's config.
        #
        # It must be the EFFECTIVE width -- what the firmware integrates its own
        # pose with -- NOT the raw caliper value `cfg.trackwidth` returns. See
        # effective_track_width()'s own docstring for the measured consequence.
        effective_track = _effective_track_width(cfg)
        if effective_track:
            trace_model.set_trackwidth(effective_track)
            # The graph panel keeps its OWN EncoderDeadReckoner for the
            # Heading/Distance strip charts, constructed with a hard-coded
            # 128.0 that nothing ever updated -- so those graphs carried the
            # same rotation-proportional error, for every robot, always. Same
            # geometry, same source, set together.
            graph_panel.recorder.set_trackwidth(effective_track)
        if _state.get("transport") is not None:
            _push_robot_calibration()

    robot_combo.currentIndexChanged.connect(_on_robot_changed)

    # ---------------------------------------------------------------- ops panel callbacks

    def _clear_playfield_traces() -> None:
        """Clear ONLY the playfield ``TraceModel`` polylines and refresh the
        canvas -- the low-level half of the unified clear (see
        ``_clear_traces()``'s docstring). Also handed to ``TurnGraphPanel``
        as ``on_clear_extra`` (at construction, above) so ITS "Clear
        traces" header button reaches the playfield too, without calling
        back into ``graph_panel.clear()`` (which would otherwise recurse)."""
        trace_model.clear()
        canvas_ctrl.refresh()

    def _clear_traces() -> None:
        """Clear all traces and refresh the canvas.

        OOP sim-motor-state fix (unify the two "Clear Traces" buttons): the
        ops-panel "Clear Traces" button used to clear ONLY the playfield
        ``TraceModel`` polylines, leaving the turn-graphs tab's four
        recorded traces (wheel speed/position, heading, distance) intact --
        confusingly different scope from the turn-graphs header's own
        "Clear traces" button (``graph_panel.clear()``), which cleared ONLY
        those. This now also clears ``graph_panel`` so either button clears
        everything -- ``graph_panel.clear()`` in turn calls
        ``_clear_playfield_traces()`` (its ``on_clear_extra`` hook) for the
        other direction; that helper does NOT call back into
        ``graph_panel.clear()``, so there is no clear<->clear recursion."""
        _clear_playfield_traces()
        graph_panel.clear()

    # Wire the OTHER direction: graph_panel's own "Clear traces" header
    # button also clears the playfield TraceModel (see _clear_traces()'s
    # docstring and _clear_playfield_traces()'s own docstring for why this
    # is a post-construction set_on_clear_extra() call rather than a
    # constructor argument -- this function is defined after graph_panel).
    graph_panel.set_on_clear_extra(_clear_playfield_traces)

    def _refresh_playfield(pixmap: "object", origin_x: float, origin_y: float) -> None:
        """Swap canvas background and update the A1 origin atomically.

        Both the deskewed pixmap and the daemon's A1 origin (cm, corner-origin
        frame) come from a single daemon read in OpsController so they always
        match.  Passing origin_x/origin_y to set_background ensures world (0,0)
        maps to tag 1's real pixel position in the new background.
        """
        canvas_ctrl.set_background(pixmap, origin_x=origin_x, origin_y=origin_y)

    def _set_origin() -> None:
        """Reset robot to world origin: wire commands + display reset.

        Operator workflow: physically place the robot at the playfield centre,
        then click "Set Robot @ 0,0" to reset everything to (0, 0, heading 0).

        Steps:
        0. Call ``transport.halt()`` (estop) to halt motors and abort any
           in-flight motion goal, so the pose reset starts from a truly idle
           robot.  In Sim mode this is essential: a plant teleport is
           overwritten by the next tick if the firmware is still driving
           toward an old goal (heading drifts back). 128-003: rewired off
           the dead ``transport.command()`` call sending the wire verb
           ``STOP`` (a no-op on hardware via
           ``binary_bridge.translate_command()``'s stub) onto
           ``transport.halt()`` directly -- a raised failure is logged
           loudly, never swallowed on faith.
        0b. In Sim mode only, teleport the plant ground-truth to (0, 0, 0°) via
           ``transport.set_true_pose`` — the sim avatar follows the plant, not
           the firmware belief, and there is no operator to place the robot.
        1. Send ``ZERO enc`` to clear wheel encoder integrators.
        2. Send ``OZ`` to zero the OTOS sensor's position and heading.
           This is essential: the firmware fuses the OTOS absolute heading
           every tick (``Odometry::correctEKF``), so without resetting the
           OTOS the heading snaps to 0 via ``SI`` then immediately drifts
           back toward the OTOS's stale reading.  ``OZ`` calls
           ``setPositionRaw(0, 0, 0)`` on the OTOS, re-referencing it to the
           robot's current physical orientation as the new heading-zero.
        3. Send ``SI 0 0 0`` (via build_setpose_command) to snap the
           firmware's fused/EKF pose to (0 mm, 0 mm, 0°).
        4. Re-anchor the TraceModel, clear trace polylines, and move the
           canvas avatar to the field centre with heading 0.

        If no transport is connected, steps 1–3 are skipped and a
        ``[WARN]`` message is logged.  The display reset (step 4) still runs
        so the GUI stays consistent.  In Sim mode a transport IS present, so
        all three wire commands are sent.

        ``ZERO``/``OZ``/``SI`` still have no binary-plane arm on the current
        wire (genuinely gated pending sprint 098's fused pose -- see
        ``testgui/binary_bridge.py``'s own module docstring) and are
        therefore no-ops on the wire today -- steps 1-3 are sent but change
        nothing firmware-side.
        This no longer blocks a tour from running, though: step 0's
        halt/Sim-plant-teleport and step 4's display reset (which also
        re-zeros the host-side ``EncoderDeadReckoner`` via ``TraceModel.
        anchor()``/``clear()``) are unconditional and sufficient for the
        open-loop D/RT tour steps that follow to start from a known state.
        """
        transport = _state.get("transport")
        if transport is not None:
            # 0. Halt-now (estop) and abort any in-flight motion goal
            #    (TURN/tour/GOTO) BEFORE resetting the pose.  This matters
            #    especially in Sim: the plant teleport below is overwritten
            #    by the next tick if the firmware is still driving the
            #    motors toward an old goal (the heading would drift straight
            #    back — the "jumps back to the angle it started with" bug).
            #    Halt first so PWM is zero when we teleport, and so the
            #    reset starts from a truly idle robot. Never log success on
            #    faith — a raised halt failure is loud, not silent.
            try:
                transport.halt()
                _append_log("[INFO] estop sent -- motion halted")
            except Exception as exc:  # noqa: BLE001
                _append_log(f"[ERROR] HALT FAILED -- ROBOT MAY STILL BE MOVING: {exc}")
            # 0b. Sim only: teleport the plant ground-truth to (0, 0, 0°).
            #    In Sim mode the avatar follows the plant ground truth, not the
            #    firmware's belief.  On real hardware the operator physically
            #    places the robot at centre; the sim has no operator, so without
            #    this the plant keeps its prior (e.g. turned) pose and the avatar
            #    snaps back to it on the next truth delivery — while OZ/SI below
            #    would re-reference the OTOS at a stale heading.  Teleport AFTER
            #    the halt (so it sticks) and before OZ so OZ zeroes at heading 0.
            if is_sim_transport(transport):
                transport.set_true_pose(0.0, 0.0, 0.0)
            # 1. Zero encoder counters so SI starts from a clean state.
            transport.command("ZERO enc", read_timeout=300)
            # 2. Zero the OTOS sensor (re-references heading to current orientation).
            transport.command("OZ", read_timeout=300)
            # 3. Snap the fused/EKF pose to (0, 0, heading 0°).
            si_cmd = build_setpose_command(0.0, 0.0, 0.0)
            transport.command(si_cmd, read_timeout=300)
        else:
            _append_log("[WARN] Set Robot @ 0,0: no robot connected — display only")

        # 4. Reset the display (unchanged from before).
        trace_model.anchor(0.0, 0.0, 0.0)
        trace_model.clear()
        canvas_ctrl.reset_avatar_to_center()
        canvas_ctrl.refresh()

    # ----------------------------------------------------------- tour controls

    def _on_tour_log(text: str, direction: str) -> None:
        """Main-thread slot for tour log/step lines (marshalled from worker)."""
        _append_log(text, direction=direction or None)

    def _settle_tour_ui() -> None:
        """Return the tour UI to idle. Called by WorkerSession on BOTH the
        explicit-stop and natural-completion paths, exactly once per run.

        097: tour buttons are re-enabled unconditionally -- tours are un-gated.
        Safe even when this runs via _on_disconnect(), which disables every
        send button later in that same function; the disconnect flow's own
        disable always wins the race, it just happens after this.
        """
        for _tb, _ in _tour_buttons:
            _tb.setEnabled(True)
        stop_tour_btn.setEnabled(False)
        # OOP sim-motor-state fix: un-ghost "Set Robot @ 0,0".
        ops_ctrl.set_tour_running(False)

    def _settle_goto_ui() -> None:
        """Return the GOTO UI to idle.

        097: goto_btn is deliberately NOT re-enabled -- it stays disabled
        pending the pursuit loop needing SI/G. Re-enabling it here would
        wrongly un-gate it on every disconnect, since this also runs from
        _on_disconnect().
        """

    _tour_session = WorkerSession("tour", on_settled=_settle_tour_ui)
    _goto_session = WorkerSession("goto", on_settled=_settle_goto_ui)

    def _stop_tour() -> None:
        """Stop a running tour worker and join its thread (safe if idle).

        Re-enables the tour buttons (and disables ``stop_tour_btn``)
        synchronously, right after the join, instead of relying on the
        worker's queued ``finished`` signal / ``_on_tour_finished``. That
        signal can be undelivered for the explicit-stop path: it fires
        *during* the blocking ``thread.wait()`` call below, but the queued
        slot cannot run until ``wait()`` returns — and by then this
        function has already dropped the only reference to the
        ``_WorkerBridge``, so the pending delivery is lost and the buttons
        would never re-enable. See ``testgui-tour-stop-reactivation.md`` for
        the full root-cause analysis.
        """
        _tour_session.stop()

    def _on_tour_finished() -> None:
        """Main-thread slot: tour ended — join the thread, re-enable buttons.

        This is the natural-completion path only (the worker finished on
        its own); the explicit-stop path is handled synchronously inside
        ``_stop_tour`` itself and does not depend on this slot running.
        """
        _tour_session.finished()

    def _make_tour_handler(name: str, steps: list[str]):
        def _on_tour_clicked() -> None:
            transport = _state.get("transport")
            if transport is None:
                _append_log("[WARN] Not connected")
                return
            if _tour_session.running:
                _append_log("[WARN] A tour is already running")
                return
            from robot_radio.testgui.operations import is_sim_transport

            if is_sim_transport(transport):
                # Tours remain allowed against the simulator (useful for
                # dry-runs), but the operator must never be confused about
                # the target — log it unambiguously.
                _append_log("[TOUR] running in SIM mode")
            _append_log(f"[TOUR] {name} starting — resetting to origin")
            # Origin reset runs on the main thread (wire commands + display).
            _set_origin()
            # Disable all tour buttons while one runs; enable Stop Tour.
            for _tb, _ in _tour_buttons:
                _tb.setEnabled(False)
            stop_tour_btn.setEnabled(True)
            # OOP sim-motor-state fix: ghost "Set Robot @ 0,0" for the
            # duration of the tour (see OpsController.set_tour_running()).
            ops_ctrl.set_tour_running(True)
            from PySide6.QtCore import QThread  # type: ignore[import-untyped]

            worker = _TourRunner(
                transport, _state, name, list(steps),
                tour_execution(name, rest_to_rest=rest_to_rest_chk.isChecked()))
            thread = QThread()
            worker.moveToThread(thread)
            # Marshal worker signals to the GUI thread via a main-thread bridge
            # (see _WorkerBridge — a direct connection to _on_tour_log would run
            # on the worker thread and segfault Qt).
            bridge = _WorkerBridge(_on_tour_log, _on_tour_finished)
            worker.log_line.connect(bridge.on_log, Qt.ConnectionType.QueuedConnection)
            worker.finished.connect(
                bridge.on_finished, Qt.ConnectionType.QueuedConnection
            )
            thread.started.connect(worker.run)
            _tour_session.start(worker, thread, bridge)

        return _on_tour_clicked

    for _tour_btn, _tour_name in _tour_buttons:
        _tour_btn.clicked.connect(_make_tour_handler(_tour_name, TOURS[_tour_name]))
    stop_tour_btn.clicked.connect(_stop_tour)

    # ----------------------------------------------------------- GOTO controls

    def _on_goto_log(text: str, direction: str) -> None:
        """Main-thread slot for GOTO log lines (marshalled from the worker)."""
        _append_log(text, direction=direction or None)

    def _stop_goto() -> None:
        """Stop a running GOTO worker and join its thread (safe if idle).

        Re-enables ``goto_btn`` synchronously right after the join, instead
        of relying on the worker's queued ``finished`` signal /
        ``_on_goto_finished`` — mirrors the ``_stop_tour`` fix (see
        ``testgui-tour-stop-reactivation.md``): that signal can be
        undelivered for the explicit-stop path because it fires during the
        blocking ``thread.wait()`` below, and by the time ``wait()``
        returns this function has already dropped the only reference to the
        ``_WorkerBridge``.
        """
        _goto_session.stop()
        # 097: goto_btn is NOT re-enabled here -- it is permanently disabled
        # pending sprint 098 (the pursuit loop needs SI/G). Pre-097 this
        # re-enabled it unconditionally whenever connected, which would have
        # wrongly un-gated it on every disconnect (this function also runs
        # from _on_disconnect()).

    def _stop_all_motion() -> None:
        """Cancel any running tour AND GOTO worker (used by the STOP button).

        Either may be re-issuing motion commands (``SI``/``G``/tour steps) on a
        background thread; cancelling both ensures a subsequent wire ``STOP``
        is not immediately overwritten.  Safe when nothing is running.
        """
        _stop_tour()
        _stop_goto()

    def _on_goto_finished() -> None:
        """Main-thread slot: GOTO ended — join thread, re-enable the button.

        This is the natural-completion path only (the worker finished on
        its own); the explicit-stop path is handled synchronously inside
        ``_stop_goto`` itself and does not depend on this slot running.
        """
        _goto_session.finished()
        # 097: goto_btn is NOT re-enabled here -- it is permanently disabled
        # pending sprint 098 (the pursuit loop needs SI/G). Pre-097 this
        # re-enabled it unconditionally whenever connected, which would have
        # wrongly un-gated it on every disconnect (this function also runs
        # from _on_disconnect()).

    def _on_goto_clicked() -> None:
        transport = _state.get("transport")
        if transport is None:
            _append_log("[WARN] Not connected")
            return
        if _goto_session.running:
            _append_log("[WARN] GOTO already running")
            return
        goto_btn.setEnabled(False)
        from PySide6.QtCore import QThread  # type: ignore[import-untyped]

        worker = _GotoRunner(
            transport,
            _state,
            goto_x_spin.value(),
            goto_y_spin.value(),
            goto_eps_spin.value(),
            goto_speed_spin.value(),
        )
        thread = QThread()
        worker.moveToThread(thread)
        # Marshal worker signals to the GUI thread via a main-thread bridge
        # (see _WorkerBridge).
        bridge = _WorkerBridge(_on_goto_log, _on_goto_finished)
        worker.log_line.connect(bridge.on_log, Qt.ConnectionType.QueuedConnection)
        worker.finished.connect(bridge.on_finished, Qt.ConnectionType.QueuedConnection)
        thread.started.connect(worker.run)
        _goto_session.start(worker, thread, bridge)

    goto_btn.clicked.connect(_on_goto_clicked)

    # Operations panel — built after _append_log is defined so the log callback
    # is live.
    ops_panel, ops_ctrl = _build_ops_panel(
        log_cb=_append_log,
        transport_ref=_state,
        clear_traces_cb=_clear_traces,
        refresh_playfield_cb=_refresh_playfield,
        set_origin_cb=_set_origin,
        stop_motion_cb=_stop_all_motion,
    )
    # Insert the ops panel before the addStretch() already added above.
    # Because addStretch() was called already, insert at the position before it.
    left_layout.insertWidget(left_layout.count() - 1, ops_panel)

    # ------------------------------------------------------------ camera combo

    def _populate_camera_combo() -> None:
        """Populate camera_combo from the aprilcam daemon's open-camera list.

        Best-effort: any daemon-connection or import failure degrades to an
        empty combo rather than raising, so window construction never blocks
        on daemon/hardware availability (matching the "no crash without
        hardware" convention used elsewhere in this file).

        Uses ``DaemonControl.list_cameras()`` (not ``enumerate_cameras()``) —
        see ``camera_prefs`` module docstring for the rationale: the combo's
        choices must match what the capture paths can actually use, and the
        capture paths only ever read already-open cameras.
        """
        cams: list[str] = []
        try:
            from aprilcam.config import Config  # type: ignore[import]
            from aprilcam.client.control import DaemonControl  # type: ignore[import]

            dc = DaemonControl.connect_default(Config.load())
            try:
                cams = dc.list_cameras()
            finally:
                try:
                    dc.close()
                except Exception:
                    pass
        except Exception as exc:
            _log.debug("camera_combo: daemon unreachable at startup: %s", exc)
            cams = []

        camera_combo.blockSignals(True)
        try:
            camera_combo.clear()
            camera_combo.addItems(cams)
            selected = camera_prefs.select_camera(cams, camera_prefs.load_camera_pref())
            if selected is not None:
                idx = camera_combo.findText(selected)
                if idx >= 0:
                    camera_combo.setCurrentIndex(idx)
        finally:
            camera_combo.blockSignals(False)

    _populate_camera_combo()

    def _on_camera_combo_changed(text: str) -> None:
        """Persist the newly selected camera and trigger an immediate refresh."""
        if not text:
            return
        camera_prefs.save_camera_pref(text)
        ops_ctrl.trigger_live_grab()

    camera_combo.currentTextChanged.connect(_on_camera_combo_changed)

    def _on_connect() -> None:
        """Instantiate the selected Transport, call connect(), send STREAM 50."""
        # Reload the active robot config from disk on every (re)connect so a
        # live edit to data/robots/*.json (e.g. adding calibration.rotational_slip)
        # is picked up without restarting the GUI. get_robot_config() caches a
        # singleton; without this, a still-running GUI keeps the config it read
        # at startup and reports stale "no calibration.*" fallbacks. The Test
        # S/T buttons also reconnect, so their runs pick up config edits too.
        _reset_robot_config()

        name = transport_combo.currentText()
        port = port_edit.text().strip()

        transport: Transport | None = None

        if name == "Serial":
            if not port:
                # Auto-detect the robot's direct-USB port (mirrors the Relay
                # branch's own auto-discovery below): mbdeploy's device
                # registry (config/devices.json) knows which usbmodem is the
                # NEZHA2 robot vs the RADIOBRIDGE relay dongle -- never
                # auto-pick the relay as the robot.
                _append_log("[INFO] Serial: no port specified -- auto-detecting robot...")
                port = find_robot_serial_port(list_ports())
                if port is None:
                    _append_log(
                        "[ERROR] No robot serial port found (no /dev/cu.usbmodem* "
                        "present -- is the robot plugged in? `mbdeploy probe` "
                        "refreshes the device registry)"
                    )
                    return
                _append_log(f"[INFO] Robot found on {port}")
                port_edit.setText(port)
            transport = SerialTransport(port)
        elif name == "Relay":
            from robot_radio.testgui.transport import find_relay_port, _relay_probe_banner
            _append_log("[INFO] Relay: scanning serial ports for relay...")
            discovered = find_relay_port(list_ports(), _relay_probe_banner)
            if discovered is None:
                # Fall back to port_edit if the user typed a port manually.
                discovered = port_edit.text().strip() or None
            if discovered is None:
                _append_log("[WARN] No relay found on any serial port")
                return
            _append_log(f"[INFO] Relay found on {discovered}")
            port_edit.setText(discovered)
            transport = RelayTransport(discovered)
        else:
            # Sim transport — backed by ctypes firmware simulator.
            transport = SimTransport()

        # Wire log callback.
        transport.on_log = _on_log_from_thread

        # Wire telemetry and truth callbacks — these fire on background threads;
        # the bridge marshals them safely to the Qt main thread.
        # transport.on_telemetry_secondary -- REMOVED (123-004): the
        # telemetry panel's loop-timing row moved to the primary frame; see
        # _pending_secondary's own removal comment above.
        transport.on_telemetry = _on_telemetry_thread_v2
        transport.on_truth = _on_truth_thread

        # Clear any stale trace data from a previous session.
        trace_model.clear()

        # 113-006: no explicit robot_config passed here -- SimTransport.
        # connect() resolves get_robot_config() itself when the (keyword-
        # only, optional) parameter is omitted, so a plain no-arg connect()
        # call still configures the sim from the just-reset-and-reloaded
        # active robot config (SUC-001), and this call site stays exactly
        # what it was for every OTHER caller: real hardware transports
        # (which ignore the parameter regardless), AND every existing
        # lightweight ``Transport`` test double across this test suite
        # (several fixtures define their own narrow ``connect(self) ->
        # None`` override with no ``robot_config`` parameter at all --
        # calling with a keyword argument here would raise ``TypeError`` on
        # those, silently short-circuiting the whole Connect flow for them).
        try:
            transport.connect()
        except Exception as exc:
            _append_log(f"[ERROR] Connect failed: {exc}")
            return

        # For SimTransport, connect() may return without connecting if the lib
        # is missing (it shows a QMessageBox and returns silently).  Check.
        if isinstance(transport, SimTransport) and not transport._connected:
            # Warning was already shown by connect() / _show_build_warning().
            return

        # Sim connected -- show the version compiled into the LOADED sim library
        # so a stale, still-running GUI (which keeps the old dylib mapped after a
        # rebuild) is obvious at a glance rather than looking like broken motion.
        if isinstance(transport, SimTransport):
            _sim_ver = transport.firmware_version()
            if _sim_ver:
                mode_label.setText(f"SIM MODE — v{_sim_ver}")

        # Start the always-on "latest" capture for this whole session. It
        # overwrites recordings/latest.jsonl each connect so there is always a
        # recording of the most recent session — even if the operator never
        # pressed Record. Started before STREAM 50 so setup traffic is captured.
        try:
            if latest_recorder.state != "idle":
                latest_recorder.stop()
            latest_path = latest_recorder.start(LATEST_RECORDING_NAME)
            _append_log(f"[REC] Latest-session capture started: {latest_path}")
        except Exception as exc:  # noqa: BLE001
            _append_log(f"[WARN] Could not start latest-session capture: {exc}")

        # For Sim transport, telemetry flows unconditionally every SimLoop
        # tick (no STREAM verb to send -- SimLoop has no wire/config-channel
        # simulation surface at all, see transport.py's SimTransport
        # docstring). For hardware transports, send STREAM 50 here.
        if not isinstance(transport, SimTransport):
            # Loudly flag a robot running stale firmware (hardware only —
            # the sim library is always built from this tree).
            _check_firmware_version(transport)
            try:
                reply = transport.command("STREAM 50", read_timeout=300)
                if reply:
                    _append_log(f"[INFO] STREAM 50 → {reply}")
                else:
                    _append_log("[INFO] STREAM 50 sent")
            except Exception as exc:
                _append_log(f"[WARN] STREAM 50 failed: {exc}")

        _state["transport"] = transport

        # Honor a fast-forward factor selected before Connect (Sim only —
        # no-op for hardware transports).
        _apply_sim_speed()

        # Push the active robot's calibration to the firmware so the selected
        # robot's values override whatever DefaultConfig.cpp baked in — an
        # uncalibrated robot pushes neutral values (SET rotSlip=0 etc.).
        # 113-006: for Sim, SimTransport.connect() (just above) already
        # configured the sim from this same active robot config (Tier 1 +
        # Tier 2), so this repeats the Tier-1 push -- a deliberate,
        # documented no-op-ish redundancy (see _push_robot_calibration()'s
        # own docstring): it is what populates the host-side GET echo cache
        # and pushes the OTOS OI/OL/OA sequence, neither of which
        # connect()'s own configure_from_robot() call touches. Skipping this
        # call for Sim would leave GET <key> answering "nodata" and OTOS
        # unconfigured after a bare Connect (no manual robot-select click).
        _push_robot_calibration()

        # Re-apply the PID checkbox state (Sim only; the two column
        # checkboxes are mirrored, so either one is authoritative): a fresh
        # sim always boots with the velocity PID enabled, so only an
        # UNCHECKED state needs pushing — this keeps the toggle sticky
        # across the Test buttons' rebuild+reconnect cycle without log
        # noise on the common (checked) path.
        if isinstance(transport, SimTransport) and not pid_checkbox_u.isChecked():
            transport.set_pid_enabled(False)

        # Serial = BENCH MODE: the robot is on the stand, where the real OTOS
        # optically tracks nothing yet often reads status-clean — the EKF then
        # fuses "stationary at the last anchor" and pins the fused pose while
        # the encoders move, and the heading-hold fights the phantom heading
        # (bench-OTOS diagnosis, 2026-07-03).  Swap in the bench OTOS (an
        # errored copy of measured wheel travel) so pose-dependent behaviour
        # works on the stand.  Relay (playfield) and Sim keep their odometer.
        if name == "Serial":
            try:
                reply = transport.command("DBG OTOS BENCH 1", read_timeout=500)
                _append_log(
                    "[BENCH] bench OTOS enabled (Serial = bench mode) → "
                    f"{(reply or '').strip() or '(no reply)'}"
                )
            except Exception as exc:  # noqa: BLE001
                _append_log(f"[WARN] Could not enable bench OTOS: {exc}")

        # Start the live-view worker for Relay (PLAYFIELD MODE) only.
        # Sim and Serial have no playfield camera.
        if name == "Relay":
            from PySide6.QtCore import QThread  # type: ignore[import-untyped]
            from robot_radio.testgui.live_view import build_live_view_worker
            try:
                live_worker = build_live_view_worker()
                live_thread = QThread()
                live_worker.moveToThread(live_thread)
                # Route frame_ready through a main-thread bridge (ticket
                # 063-009), NOT a bare function: a QueuedConnection to a
                # non-QObject callable is delivered on the *emitting*
                # (worker) thread in this PySide build, and the worker's
                # capture loop never returns to its own event loop to
                # process it — see build_live_frame_bridge's docstring and
                # testgui-playfield-not-live-updating.md.
                live_bridge = build_live_frame_bridge(canvas_ctrl)
                live_worker.frame_ready.connect(
                    live_bridge.on_frame, Qt.ConnectionType.QueuedConnection
                )
                live_thread.started.connect(live_worker.run)
                live_thread.start()
                _state["live_worker"] = live_worker
                _state["live_thread"] = live_thread
                _state["live_bridge"] = live_bridge
                _state["live_view_active"] = True
                _append_log("[INFO] Live-view worker started (PLAYFIELD MODE)")
            except Exception as exc:
                _append_log(f"[WARN] Could not start live-view worker: {exc}")

        # Update button states.
        connect_btn.setEnabled(False)
        disconnect_btn.setEnabled(True)
        transport_combo.setEnabled(False)
        port_edit.setEnabled(False)
        # Enable all Send buttons now that a transport is connected.
        for _sb in _send_buttons:
            _sb.setEnabled(True)
        # 108-007: tour buttons enable for Sim exactly like hardware now --
        # SimTransport.protocol exposes a live SimLoop (see the
        # tour-button-creation comment for the full rationale). Already
        # enabled by the generic loop above; only the tooltip differs.
        for _tb, _tour_name in _tour_buttons:
            if is_sim_transport(transport):
                _tb.setToolTip(_tour_sim_tooltip(_tour_name))
            else:
                _tb.setToolTip(_tour_hw_tooltip(_tour_name))
        # Enable operations panel buttons.
        ops_ctrl.set_connected(True, transport)
        desc = "Sim" if name == "Sim" else f"{name} on {port}"
        _append_log(f"[INFO] Connected via {desc}")

        # Auto-grab the live playfield image on hardware connect.  Sim has no
        # camera so skip it there — the grey placeholder is correct for sim.
        if not isinstance(transport, SimTransport):
            from PySide6.QtCore import QTimer  # type: ignore[import-untyped]
            QTimer.singleShot(200, ops_ctrl.trigger_live_grab)

    def _on_disconnect() -> None:
        """Call transport.disconnect() and clean up."""
        transport: Transport | None = _state.get("transport")
        if transport is None:
            return
        # Stop any running tour / GOTO before the transport goes away.
        _stop_tour()
        _stop_goto()
        _state["last_truth"] = None
        _state["last_tlm"] = None
        # Stop the live-view worker first so it doesn't race with cleanup.
        _stop_live_worker()
        # Best-effort: restore the real OTOS if this was a bench (Serial)
        # session, so a later playfield run doesn't inherit bench mode.
        if isinstance(transport, SerialTransport):
            try:
                transport.command("DBG OTOS BENCH 0", read_timeout=300)
            except Exception:  # noqa: BLE001
                pass
        try:
            transport.disconnect()
        except Exception as exc:
            _append_log(f"[WARN] Disconnect error: {exc}")
        _state["transport"] = None

        # Restore button/combo state.
        connect_btn.setEnabled(True)
        disconnect_btn.setEnabled(False)
        transport_combo.setEnabled(True)
        # Disable all Send buttons — no transport connected.
        for _sb in _send_buttons:
            _sb.setEnabled(False)
        # Disable operations panel buttons.
        ops_ctrl.set_connected(False)
        # Re-enable port field if a hardware transport was selected.
        _on_transport_changed(transport_combo.currentIndex())
        _append_log("[INFO] Disconnected")
        # Finalize the always-on latest-session capture.
        latest_path = latest_recorder.stop()
        if latest_path is not None:
            _append_log(f"[REC] Latest-session capture saved: {latest_path}")

    connect_btn.clicked.connect(_on_connect)
    disconnect_btn.clicked.connect(_on_disconnect)

    # ---- Test buttons: edit -> compile -> hot-reload -> reset -> run -------
    import os as _os
    import pathlib as _pathlib
    import shutil as _shutil
    import subprocess as _subprocess
    import tempfile as _tempfile
    import threading as _threading

    from robot_radio.testgui.transport import set_sim_lib_override as _set_sim_lib_override

    _repo_root = _pathlib.Path(__file__).resolve().parents[4]
    _reload_counter = {"n": 0}

    class _TestBridge(QObject):
        rebuilt = Signal(str, bool)   # (kind, ok) -- emitted by the worker thread

    _test_bridge = _TestBridge()

    def _finish_test(kind: str, ok: bool) -> None:
        """Main thread: after the rebuild, hot-reload the fresh dylib, reset the
        display, refresh the version stamp (done by _on_connect), and run."""
        try:
            if not ok:
                _append_log("[TEST] build FAILED — see console above; not reloading.")
                return
            fresh = _state.get("_fresh_sim_lib")
            if fresh is None:
                _append_log("[TEST] internal error: no fresh lib path")
                return
            if _state.get("transport") is not None:
                _on_disconnect()
            transport_combo.setCurrentText("Sim")
            _set_sim_lib_override(fresh)          # next connect loads the fresh copy
            try:
                _on_connect()                     # loads fresh dylib + updates version stamp
            finally:
                _set_sim_lib_override(None)
            if _state.get("transport") is None:
                _append_log("[TEST] reconnect failed — see console.")
                return
            _set_origin()                         # reset avatar / pose / traces
            transport = _state["transport"]
            if kind == "S":
                _append_log("[TEST] Test S (managed) -> D 150 150 700")
                transport.command("D 150 150 700", read_timeout=500)
            elif kind == "T":
                _append_log("[TEST] Test T (managed) -> RT 36000")
                transport.command("RT 36000", read_timeout=500)
            elif kind == "US":
                # Unmanaged: one direct twist, deadman-timed — no planner,
                # no heading loop, no retargets (SimTransport.run_unmanaged).
                _append_log("[TEST] Test S (unmanaged) -> direct twist, 700mm @ 150mm/s")
                transport.run_unmanaged(distance_mm=700.0)
            elif kind == "UT":
                _append_log("[TEST] Test T (unmanaged) -> direct twist, 360° @ 2rad/s")
                transport.run_unmanaged(angle_deg=360.0)
        finally:
            for _b in (test_s_btn, test_t_btn, test_us_btn, test_ut_btn):
                _b.setEnabled(True)

    _test_bridge.rebuilt.connect(_finish_test, Qt.ConnectionType.QueuedConnection)

    def _run_sim_test(kind: str) -> None:
        for _b in (test_s_btn, test_t_btn, test_us_btn, test_ut_btn):
            _b.setEnabled(False)

        def _worker() -> None:
            ok = True
            print(f"\n[TEST] ===== Test {kind}: rebuilding sim lib =====", flush=True)
            # Regenerate version + message headers, then build the sim lib. The
            # subprocesses inherit the redirected fd 1/2, so their output lands
            # in the console automatically.
            steps = [
                [sys.executable, "src/scripts/gen_version.py"],
                [sys.executable, "src/scripts/gen_messages.py"],
                ["cmake", "--build", "src/sim/build", "--parallel", "--target", "firmware_host"],
            ]
            for cmd in steps:
                try:
                    rc = _subprocess.run(cmd, cwd=str(_repo_root)).returncode
                except Exception as exc:  # noqa: BLE001
                    print(f"[TEST] step errored: {' '.join(cmd)}: {exc}", flush=True)
                    ok = False
                    break
                if rc != 0:
                    print(f"[TEST] step failed ({rc}): {' '.join(cmd)}", flush=True)
                    ok = False
                    break
            if ok:
                _reload_counter["n"] += 1
                suffix = ".dylib" if sys.platform == "darwin" else ".so"
                built = _repo_root / "src" / "sim" / "build" / f"libfirmware_host{suffix}"
                fresh = _pathlib.Path(_tempfile.gettempdir()) / f"sim_reload_{_os.getpid()}_{_reload_counter['n']}{suffix}"
                try:
                    _shutil.copy2(built, fresh)
                    _state["_fresh_sim_lib"] = fresh
                    print(f"[TEST] built OK -> reloading {fresh.name}", flush=True)
                except Exception as exc:  # noqa: BLE001
                    print(f"[TEST] copy failed: {exc}", flush=True)
                    ok = False
            _test_bridge.rebuilt.emit(kind, ok)

        _threading.Thread(target=_worker, name=f"sim-test-{kind}", daemon=True).start()

    test_s_btn.clicked.connect(lambda: _run_sim_test("S"))
    test_t_btn.clicked.connect(lambda: _run_sim_test("T"))
    test_us_btn.clicked.connect(lambda: _run_sim_test("US"))
    test_ut_btn.clicked.connect(lambda: _run_sim_test("UT"))

    # Stop the live-view worker and any running tour / GOTO on app quit.
    app.aboutToQuit.connect(_stop_live_worker)
    app.aboutToQuit.connect(_stop_tour)
    app.aboutToQuit.connect(_stop_goto)
    # Flush/close both recorders on quit so a quit-without-disconnect still
    # leaves a complete latest.jsonl on disk.
    app.aboutToQuit.connect(recorder.stop)
    app.aboutToQuit.connect(latest_recorder.stop)

    # -------------------------------------------------------------- startup grab
    # Trigger a best-effort live playfield grab shortly after the event loop
    # starts (200 ms delay gives Qt time to show the window and initialise the
    # viewport before we fire a background daemon call).  The grab runs on a
    # background thread; the grey placeholder remains visible until it completes.
    # In sim mode there is no camera so we skip the auto-grab entirely.
    from PySide6.QtCore import QTimer  # type: ignore[import-untyped]
    QTimer.singleShot(200, ops_ctrl.trigger_live_grab)

    return window, app


def main() -> None:
    """Launch the Robot Test GUI and block until the window is closed."""
    global _ENABLE_STDOUT_CAPTURE
    _ENABLE_STDOUT_CAPTURE = True   # real GUI: capture sim cout/cerr to the console
    window, app = _build_main_window()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
