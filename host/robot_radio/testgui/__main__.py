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
``on_frame_ready`` calls ``canvas_ctrl.refresh(fused_yaw_rad)`` as before,
positioning the marker from the latest fused pose.

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
A ``QComboBox`` (``objectName="camera_combo"``) sits in the right panel above
the playfield canvas, listing cameras reported by the aprilcam daemon's
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


def _build_main_window():  # type: ignore[return]
    """Build and return the main QMainWindow with transport wiring.

    Layout (left-to-right via QSplitter):
    - Left panel: transport selector QComboBox, port QLineEdit, Connect
      button, schema-driven command rows, and placeholder operations panel.
    - Right panel: QGraphicsView playfield canvas with trace paths and robot
      marker (top) and timestamped log pane QPlainTextEdit (bottom).

    The transport selector enables the port QLineEdit when Serial or Relay
    is selected.  Clicking Connect instantiates the selected Transport,
    calls transport.connect(), then sends ``STREAM 50``.  Clicking
    Disconnect calls transport.disconnect().

    The log pane receives all sent and received lines via the transport's
    on_log callback, delivered safely from background threads via
    QMetaObject.invokeMethod.

    Command rows are built from the ``COMMANDS`` schema in
    ``robot_radio.testgui.commands``.  Each row has a label, one labeled
    input field per parameter, and a Send button.  Send buttons are disabled
    when no transport is connected.  Clicking Send assembles the wire string
    via ``build_wire_string`` and calls ``transport.command(line)``.
    """
    # PySide6 imports are intentionally deferred here.
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QApplication,
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
        QVBoxLayout,
        QWidget,
    )
    from PySide6.QtCore import Qt, QMetaObject, Q_ARG  # type: ignore[import-untyped]

    from robot_radio.testgui.transport import (
        Transport,
        SerialTransport,
        RelayTransport,
        SimTransport,
        list_ports,
    )
    from robot_radio.testgui.commands import (
        COMMANDS,
        TOURS,
        build_wire_string,
        goto_distance_mm,
        goto_reached,
    )
    from robot_radio.testgui.operations import (
        build_panel as _build_ops_panel,
        build_setpose_command,
        is_sim_transport,
    )
    from robot_radio.testgui.traces import TraceModel
    from robot_radio.testgui.canvas import build_canvas
    from robot_radio.testgui.drive import KeyboardDriver
    from robot_radio.testgui.recorder import SessionRecorder, direction_from_marker
    from robot_radio.testgui import camera_prefs
    from robot_radio.testgui import sim_prefs

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
        "tour_worker": None,
        "tour_thread": None,
        "tour_bridge": None,
        "goto_worker": None,
        "goto_thread": None,
        "goto_bridge": None,
        # Latest camera ground-truth pose (x_cm, y_cm, yaw_rad, monotonic_ts)
        # cached from the transport on_truth callback for the GOTO loop.
        "last_truth": None,
    }

    # Keyboard driver — wires cursor-key VW driving onto the main window.
    _driver = KeyboardDriver()

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
    window.resize(1200, 700)

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
    left_layout.addWidget(transport_label)

    transport_combo = QComboBox()
    transport_combo.setObjectName("transport_combo")
    transport_combo.addItems(["Sim", "Serial", "Relay"])
    left_layout.addWidget(transport_combo)

    # Robot selector — pick which robot config is active.  Selecting a robot
    # rewrites active_robot.json and reloads that config (wired below, once
    # trace_model / _append_log are in scope).
    from robot_radio.config.robot_config import (
        get_robot_config,
        list_robots,
        set_active_robot,
    )

    robot_label = QLabel("Robot:")
    left_layout.addWidget(robot_label)

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
    left_layout.addWidget(robot_combo)

    # Port picker (enabled only for Serial / Relay)
    port_label = QLabel("Port:")
    left_layout.addWidget(port_label)

    port_edit = QLineEdit()
    port_edit.setObjectName("port_edit")
    port_edit.setPlaceholderText("/dev/cu.usbmodem…")
    port_edit.setEnabled(False)
    # Pre-populate with the first detected USB modem port if any.
    detected = list_ports()
    if detected:
        port_edit.setText(detected[0])
    left_layout.addWidget(port_edit)

    # Connect / Disconnect buttons in an HBox
    btn_row = QWidget()
    btn_layout = QHBoxLayout(btn_row)
    btn_layout.setContentsMargins(0, 0, 0, 0)

    connect_btn = QPushButton("Connect")
    connect_btn.setObjectName("connect_btn")
    disconnect_btn = QPushButton("Disconnect")
    disconnect_btn.setObjectName("disconnect_btn")
    disconnect_btn.setEnabled(False)

    btn_layout.addWidget(connect_btn)
    btn_layout.addWidget(disconnect_btn)
    left_layout.addWidget(btn_row)

    # Record / Pause / Stop controls (ticket 005)
    record_btn = QPushButton("Record")
    record_btn.setObjectName("record_btn")
    pause_btn = QPushButton("Pause")
    pause_btn.setObjectName("pause_btn")
    pause_btn.setEnabled(False)
    stop_btn = QPushButton("Stop")
    stop_btn.setObjectName("stop_btn")
    stop_btn.setEnabled(False)

    rec_row = QWidget()
    rec_layout = QHBoxLayout(rec_row)
    rec_layout.setContentsMargins(0, 0, 0, 0)
    rec_layout.addWidget(record_btn)
    rec_layout.addWidget(pause_btn)
    rec_layout.addWidget(stop_btn)
    left_layout.addWidget(rec_row)

    # Command rows — built from the COMMANDS schema.
    # Each row: Send button | verb label | field1 | field2 …
    # Send buttons and verb labels are fixed-width so they line up in columns.
    # All Send buttons are collected so we can enable/disable them together.
    cmd_rows_widget = QWidget()
    cmd_rows_widget.setObjectName("cmd_rows")
    cmd_rows_layout = QVBoxLayout(cmd_rows_widget)
    cmd_rows_layout.setContentsMargins(0, 4, 0, 4)
    cmd_rows_layout.setSpacing(4)

    # List of all Send buttons — disabled until a transport connects.
    _send_buttons: list[QPushButton] = []

    def _build_command_row(spec) -> tuple[QWidget, list]:
        """Build one command row widget from a CommandSpec.

        Returns (row_widget, field_getters) where ``field_getters`` is an
        ordered list of callables; each callable returns the current numeric
        value of the corresponding field.
        """
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(4)

        # Send button first so all Send buttons form a left-justified column.
        send_btn = QPushButton("Send")
        send_btn.setObjectName(f"send_btn_{spec['label'].lower()}")
        send_btn.setEnabled(False)
        send_btn.setFixedWidth(52)
        row_layout.addWidget(send_btn)

        # Command verb label (fixed-width so verb names align in their own column).
        verb_label = QLabel(spec["label"])
        verb_label.setFixedWidth(44)
        row_layout.addWidget(verb_label)

        field_getters: list = []

        for param in spec["params"]:
            name = param["name"]
            unit = param.get("unit", "")
            p_type = param.get("type", int)
            p_min = param.get("min", -10000)
            p_max = param.get("max", 10000)
            p_default = param.get("default", 0)

            # Short label above (or beside) the field.
            param_label = QLabel(f"{name}:")
            param_label.setFixedWidth(46)
            row_layout.addWidget(param_label)

            if p_type is float:
                spin = QDoubleSpinBox()
                spin.setRange(float(p_min), float(p_max))
                spin.setValue(float(p_default))
                spin.setDecimals(1)
                spin.setSuffix(f" {unit}" if unit else "")
                spin.setFixedWidth(80)
                row_layout.addWidget(spin)
                field_getters.append(lambda s=spin: s.value())
            else:
                spin = QSpinBox()
                spin.setRange(int(p_min), int(p_max))
                spin.setValue(int(p_default))
                spin.setSuffix(f" {unit}" if unit else "")
                spin.setFixedWidth(80)
                row_layout.addWidget(spin)
                field_getters.append(lambda s=spin: s.value())

        row_layout.addStretch()

        _send_buttons.append(send_btn)
        return row, field_getters

    # Wire up each row's Send button to build + dispatch the wire string.
    def _wire_send_button(btn: QPushButton, spec, getters: list) -> None:
        """Connect btn.clicked to a closure that builds + sends the wire string."""
        def _on_send():
            transport: Transport | None = _state.get("transport")
            if transport is None:
                _append_log("[WARN] Not connected")
                return
            values = {
                param["name"]: getter()
                for param, getter in zip(spec["params"], getters)
            }
            line = build_wire_string(spec, values)
            # The transport itself logs both the outbound line and its reply
            # via on_log (routed through _rx_bridge, which infers TX/RX from
            # the >/< marker for the recorder) — echoing them here too would
            # duplicate every line in the console and the recording.
            try:
                transport.command(line, read_ms=500)
            except Exception as exc:
                _append_log(f"[ERROR] {exc}")

        btn.clicked.connect(_on_send)

    # Build one row per command in the COMMANDS schema.
    _row_send_getters: list[tuple[QPushButton, "object", list]] = []
    for cmd_spec in COMMANDS:
        row_widget, getters = _build_command_row(cmd_spec)
        cmd_rows_layout.addWidget(row_widget)
        # Find the Send button just appended.
        btn = _send_buttons[-1]
        _row_send_getters.append((btn, cmd_spec, getters))

    left_layout.addWidget(cmd_rows_widget)

    # Tour buttons — run a pre-programmed motion sequence (one per named tour).
    # Each button resets the robot to the origin, then sends the tour's moves
    # one at a time on a background thread, waiting for each to complete.
    tour_row = QWidget()
    tour_layout = QHBoxLayout(tour_row)
    tour_layout.setContentsMargins(0, 0, 0, 0)
    tour_layout.setSpacing(4)
    _tour_buttons: list[tuple[QPushButton, str]] = []
    for _tour_name in TOURS:
        _tb = QPushButton(_tour_name)
        _tb.setObjectName(f"tour_btn_{_tour_name.lower().replace(' ', '_')}")
        _tb.setEnabled(False)
        _tb.setToolTip(
            f"Run {_tour_name}: reset to origin, then drive a fixed "
            "sequence, waiting for each move to complete."
        )
        tour_layout.addWidget(_tb)
        _tour_buttons.append((_tb, _tour_name))
        # Enable/disable together with the Send buttons on connect/disconnect.
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
    tour_layout.addStretch()
    left_layout.addWidget(tour_row)

    # GOTO — synthetic camera-based go-to: drive to a world (x, y) point by
    # repeatedly correcting the robot's pose from the camera and re-issuing G.
    goto_row = QWidget()
    goto_layout = QHBoxLayout(goto_row)
    goto_layout.setContentsMargins(0, 0, 0, 0)
    goto_layout.setSpacing(4)

    goto_btn = QPushButton("GOTO")
    goto_btn.setObjectName("goto_btn")
    goto_btn.setEnabled(False)
    goto_btn.setFixedWidth(52)
    goto_btn.setToolTip(
        "Camera-based go-to: repeatedly reads the camera pose, snaps the robot "
        "to it (SI), and drives toward (x, y) with G until within eps."
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
        sp.setRange(lo, hi)
        sp.setValue(default)
        sp.setSuffix(f" {unit}")
        sp.setFixedWidth(80)
        goto_layout.addWidget(sp)
        return sp

    goto_x_spin = _make_goto_spin("x", 0, -10000, 10000, "mm")
    goto_y_spin = _make_goto_spin("y", 0, -10000, 10000, "mm")
    goto_eps_spin = _make_goto_spin("eps", 50, 1, 2000, "mm")
    goto_speed_spin = _make_goto_spin("speed", 200, 1, 1000, "mm/s")
    goto_layout.addStretch()
    left_layout.addWidget(goto_row)
    # GOTO enables/disables with the Send buttons on connect/disconnect.
    _send_buttons.append(goto_btn)

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
        object_name: str, label: str, value: float,
        lo: float, hi: float, decimals: int,
    ) -> QDoubleSpinBox:
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(4)
        lbl = QLabel(label)
        lbl.setFixedWidth(120)
        row_layout.addWidget(lbl)
        spin = QDoubleSpinBox()
        spin.setObjectName(object_name)
        spin.setRange(lo, hi)
        spin.setDecimals(decimals)
        spin.setValue(value)
        spin.setFixedWidth(90)
        row_layout.addWidget(spin)
        row_layout.addStretch()
        sim_errors_layout.addWidget(row)
        return spin

    def _add_sim_err_section_label(title: str) -> None:
        """Bold sub-heading grouping the spin-box rows that follow it.

        Purely visual (069-007's suggested grouping: "Encoder Report Error",
        "Body-Truth Scrub", "Geometry & Actuation", "OTOS Error") — the
        panel's flat QVBoxLayout has no nested QGroupBoxes, so tests locate
        individual spin boxes by objectName regardless of grouping.
        """
        lbl = QLabel(title)
        bold_font = lbl.font()
        bold_font.setBold(True)
        lbl.setFont(bold_font)
        sim_errors_layout.addWidget(lbl)

    # -- Encoder Report Error --------------------------------------------
    _add_sim_err_section_label("Encoder Report Error")
    sim_err_encoder_mm = _make_sim_err_spin(
        "sim_err_encoder_mm", "encoder noise (mm):",
        _sim_error_profile["encoder_noise_mm"], 0.0, 50.0, 2,
    )
    sim_err_enc_scale_l = _make_sim_err_spin(
        "sim_err_enc_scale_l", "enc scale err L:",
        _sim_error_profile["enc_scale_err_l"], -0.5, 0.5, 3,
    )
    sim_err_enc_scale_r = _make_sim_err_spin(
        "sim_err_enc_scale_r", "enc scale err R:",
        _sim_error_profile["enc_scale_err_r"], -0.5, 0.5, 3,
    )

    # -- Body-Truth Scrub --------------------------------------------------
    _add_sim_err_section_label("Body-Truth Scrub")
    sim_err_slip_turn = _make_sim_err_spin(
        "sim_err_slip_turn", "turn slip:",
        _sim_error_profile["slip_turn_extra"], 0.0, 2.0, 3,
    )
    sim_err_body_rot_scrub = _make_sim_err_spin(
        "sim_err_body_rot_scrub", "body rot scrub:",
        _sim_error_profile["body_rot_scrub"], 0.0, 1.0, 3,
    )
    sim_err_body_lin_scrub = _make_sim_err_spin(
        "sim_err_body_lin_scrub", "body lin scrub:",
        _sim_error_profile["body_lin_scrub"], 0.0, 1.0, 3,
    )

    # -- Geometry & Actuation ----------------------------------------------
    _add_sim_err_section_label("Geometry & Actuation")
    sim_err_motor_offset_l = _make_sim_err_spin(
        "sim_err_motor_offset_l", "motor offset L:",
        _sim_error_profile["motor_offset_l"], 0.0, 2.0, 3,
    )
    sim_err_motor_offset_r = _make_sim_err_spin(
        "sim_err_motor_offset_r", "motor offset R:",
        _sim_error_profile["motor_offset_r"], 0.0, 2.0, 3,
    )
    # trackwidth_mm has NO safe zero default (PhysicsWorld::update() divides
    # by it) — the spinbox range excludes 0 entirely, and the default is the
    # firmware config's trackwidth (128.0mm, what the sim seeds the plant
    # with at construction), not a sentinel. Every Apply unconditionally
    # sends this value; there is no "don't touch" case.
    sim_err_trackwidth = _make_sim_err_spin(
        "sim_err_trackwidth", "trackwidth (mm):",
        _sim_error_profile["trackwidth_mm"], 10.0, 500.0, 1,
    )

    # -- OTOS Error ----------------------------------------------------------
    _add_sim_err_section_label("OTOS Error")
    sim_err_otos_linear = _make_sim_err_spin(
        "sim_err_otos_linear", "OTOS linear noise:",
        _sim_error_profile["otos_linear_noise"], 0.0, 2.0, 3,
    )
    sim_err_otos_yaw = _make_sim_err_spin(
        "sim_err_otos_yaw", "OTOS yaw noise:",
        _sim_error_profile["otos_yaw_noise"], 0.0, 2.0, 3,
    )
    sim_err_otos_lin_scale = _make_sim_err_spin(
        "sim_err_otos_lin_scale", "OTOS lin scale err:",
        _sim_error_profile["otos_lin_scale_err"], -0.5, 0.5, 3,
    )
    sim_err_otos_ang_scale = _make_sim_err_spin(
        "sim_err_otos_ang_scale", "OTOS ang scale err:",
        _sim_error_profile["otos_ang_scale_err"], -0.5, 0.5, 3,
    )
    sim_err_otos_lin_drift = _make_sim_err_spin(
        "sim_err_otos_lin_drift", "OTOS lin drift (mm/s):",
        _sim_error_profile["otos_lin_drift_mms"], -50.0, 50.0, 2,
    )
    sim_err_otos_yaw_drift = _make_sim_err_spin(
        "sim_err_otos_yaw_drift", "OTOS yaw drift (deg/s):",
        _sim_error_profile["otos_yaw_drift_degs"], -30.0, 30.0, 2,
    )

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
            "encoder_noise_mm": sim_err_encoder_mm.value(),
            "slip_turn_extra": sim_err_slip_turn.value(),
            "otos_linear_noise": sim_err_otos_linear.value(),
            "otos_yaw_noise": sim_err_otos_yaw.value(),
            "enc_scale_err_l": sim_err_enc_scale_l.value(),
            "enc_scale_err_r": sim_err_enc_scale_r.value(),
            "body_rot_scrub": sim_err_body_rot_scrub.value(),
            "body_lin_scrub": sim_err_body_lin_scrub.value(),
            "motor_offset_l": sim_err_motor_offset_l.value(),
            "motor_offset_r": sim_err_motor_offset_r.value(),
            "trackwidth_mm": sim_err_trackwidth.value(),
            "otos_lin_scale_err": sim_err_otos_lin_scale.value(),
            "otos_ang_scale_err": sim_err_otos_ang_scale.value(),
            "otos_lin_drift_mms": sim_err_otos_lin_drift.value(),
            "otos_yaw_drift_degs": sim_err_otos_yaw_drift.value(),
        }
        sim_prefs.save_sim_error_profile(profile)
        _append_log(
            f"[INFO] Sim error profile saved "
            f"(encoder_noise_mm={profile['encoder_noise_mm']}, "
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
        (sim_err_encoder_mm, sim_err_otos_linear, sim_err_otos_yaw) are never
        touched.

        Missing config / missing calibration fields never crash the panel —
        they fall back to the neutral value for that knob and log a [WARN].
        """
        cfg = get_robot_config()
        if cfg is None:
            rot_slip = 1.0
            tw = sim_prefs.DEFAULT_PROFILE["trackwidth_mm"]
            _append_log(
                "[WARN] From Calibration: no active robot config found — "
                f"falling back to neutral body_rot_scrub={rot_slip}, "
                f"trackwidth_mm={tw}"
            )
        else:
            if cfg.calibration.rotational_slip is not None:
                rot_slip = cfg.calibration.rotational_slip
            else:
                rot_slip = 1.0
                _append_log(
                    "[WARN] From Calibration: active robot config has no "
                    f"calibration.rotational_slip — falling back to neutral "
                    f"body_rot_scrub={rot_slip}"
                )
            if cfg.geometry.trackwidth is not None:
                tw = cfg.geometry.trackwidth
            else:
                tw = sim_prefs.DEFAULT_PROFILE["trackwidth_mm"]
                _append_log(
                    "[WARN] From Calibration: active robot config has no "
                    f"geometry.trackwidth — falling back to neutral "
                    f"trackwidth_mm={tw}"
                )

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
            f"(body_rot_scrub={rot_slip}, trackwidth_mm={tw}, all other "
            f"knobs neutral; noise fields untouched)"
        )

        _on_sim_errors_apply()

    sim_errors_apply_btn.clicked.connect(_on_sim_errors_apply)
    sim_errors_from_cal_btn.clicked.connect(_on_sim_errors_from_cal)
    left_layout.addWidget(sim_errors_group)

    left_layout.addStretch()
    splitter.addWidget(left_widget)

    # --------------------------------------------------------- TraceModel (Qt-free)
    trace_model = TraceModel()

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

    # Camera-selection pull-down (ticket 063-008) — lists cameras known to
    # the aprilcam daemon; persisted across sessions via camera_prefs.
    # Populated best-effort below (_populate_camera_combo); selection change
    # is wired further down, once ops_ctrl.trigger_live_grab is available.
    camera_row = QWidget()
    camera_row_layout = QHBoxLayout(camera_row)
    camera_row_layout.setContentsMargins(0, 0, 0, 0)
    camera_row_layout.setSpacing(4)
    camera_combo_label = QLabel("Camera:")
    camera_row_layout.addWidget(camera_combo_label)
    camera_combo = QComboBox()
    camera_combo.setObjectName("camera_combo")
    camera_combo.setToolTip(
        "Select which aprilcam daemon camera to use for the playfield feed.\n"
        "The selection persists across sessions and triggers an immediate\n"
        "playfield refresh from the newly selected camera."
    )
    camera_row_layout.addWidget(camera_combo, stretch=1)
    right_layout.addWidget(camera_row)

    right_splitter = QSplitter(Qt.Orientation.Vertical)
    right_layout.addWidget(right_splitter)

    # Playfield canvas — replaces the ticket-005 placeholder QGraphicsView.
    canvas_widget, canvas_ctrl = build_canvas(trace_model)
    right_splitter.addWidget(canvas_widget)

    # Log pane (QPlainTextEdit) — receives timestamped TX/RX lines
    log_pane = QPlainTextEdit()
    log_pane.setObjectName("log_pane")
    log_pane.setReadOnly(True)
    log_pane.setPlaceholderText("(log output will appear here)")
    log_pane.setMaximumHeight(200)
    right_splitter.addWidget(log_pane)

    splitter.addWidget(right_widget)

    # Reasonable initial splitter proportions: 30% left / 70% right
    splitter.setSizes([360, 840])

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

    class _TelemetryBridge(QObject):
        """Bridges background-thread TLMFrame delivery to the Qt main thread.

        The ``frame_ready`` signal carries a sentinel (int) across the thread
        boundary; the actual frame is retrieved from a shared queue.
        """
        frame_ready = Signal()
        truth_ready = Signal(float, float, float)

        def __init__(self) -> None:
            super().__init__()

        @Slot()
        def on_frame_ready(self) -> None:
            """Process all pending TLMFrames queued from background threads.

            Always accumulates the fused trace in the TraceModel.  In
            PLAYFIELD MODE (``live_view_active``) the avatar is driven by the
            camera live-view worker (:meth:`CanvasController.set_avatar_pose`),
            so the marker is left untouched here (``refresh(update_marker=False)``)
            to avoid fighting the worker for the marker's position — only the
            trace paths redraw at the (faster) TLM rate.
            """
            while True:
                try:
                    frame = _pending_frames.get_nowait()
                except Exception:
                    break
                trace_model.feed(frame)
                fused_yaw_rad = None
                if frame.pose is not None:
                    fused_yaw_rad = math.radians(frame.pose[2] / 100.0)
                if _state.get("live_view_active"):
                    canvas_ctrl.refresh(update_marker=False)
                else:
                    canvas_ctrl.refresh(fused_yaw_rad)

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

        Sends each wire string in ``steps`` via ``transport.command``, then
        waits for the robot to return to idle (``mode=I``) before sending the
        next.  Idle detection requests fresh telemetry with a fire-and-forget
        ``SNAP`` and reads the resulting frame from ``state["last_tlm"]`` (the
        transport reader delivers the SNAP's TLM reply there via
        ``on_telemetry``).  This is done instead of ``transport.command("SNAP")``
        because a SNAP reply is a corr-id-less TLM frame that ``command()``'s
        reply queue never receives — the call would always time out with an
        empty string, so ``mode=I`` was never seen and the tour could never
        advance past its first step.  On-demand SNAP (rather than a stream) is
        used because the radio relay drops asynchronous stream frames but
        answers SNAP reliably.

        Signals are marshalled to the Qt main thread via QueuedConnection:
        ``log_line(text, direction)`` carries ``[TOUR]`` status narration
        (direction ``""``, not recorded) — the raw ``>``/``<`` wire traffic
        for each step is already logged by the transport itself (see
        ``_GotoRunner``, which follows the same rule), so this class must not
        echo it again or every step is duplicated in the console and the
        session recording.  ``finished()`` re-enables the button and joins
        the thread.
        """

        log_line = Signal(str, str)
        finished = Signal()

        #: Delay (s) after a command before polling, so the move has started.
        SPINUP_S = 0.2
        #: Interval (s) between SNAP completion polls.
        POLL_S = 0.3
        #: Max wait (s) for a SNAP's TLM reply to reach state["last_tlm"].
        SNAP_REPLY_TIMEOUT_S = 0.8
        #: Per-move timeout (s) before giving up and aborting the tour.
        MOVE_TIMEOUT_S = 30.0

        def __init__(
            self, transport: "object", state: dict, name: str, steps: list[str]
        ) -> None:
            super().__init__()
            self._transport = transport
            self._state = state
            self._name = name
            self._steps = steps
            self._stop = False

        def stop(self) -> None:
            """Request the tour abort at the next safe point (thread-safe)."""
            self._stop = True

        @Slot()
        def run(self) -> None:
            """Execute the tour step-by-step (runs on the worker thread)."""
            import time

            total = len(self._steps)
            try:
                for i, cmd in enumerate(self._steps, 1):
                    if self._stop:
                        self.log_line.emit(f"[TOUR] {self._name} aborted", "")
                        return
                    self.log_line.emit(
                        f"[TOUR] {self._name} step {i}/{total}: {cmd}", ""
                    )
                    try:
                        self._transport.command(cmd, read_ms=500)
                    except Exception as exc:  # noqa: BLE001
                        self.log_line.emit(f"[TOUR] error sending {cmd!r}: {exc}", "")
                        return
                    if not self._wait_for_idle(time):
                        self.log_line.emit(
                            f"[TOUR] timed out waiting for '{cmd}' to complete — "
                            "aborting",
                            "",
                        )
                        return
                self.log_line.emit(f"[TOUR] {self._name} complete", "")
            finally:
                self.finished.emit()

        def _wait_for_idle(self, time_mod: "object") -> bool:
            """Wait until telemetry reports ``mode=I`` (idle) or timeout.

            Requests a fresh frame with a fire-and-forget ``SNAP`` and reads
            the mode from ``state["last_tlm"]`` (populated by the transport's
            ``on_telemetry`` callback — see the class docstring for why
            ``command("SNAP")`` cannot be used).  Only frames stamped after
            this wait began are accepted, so a stale pre-move idle frame does
            not end the wait early.

            Returns ``True`` when idle is observed (or on stop request),
            ``False`` on timeout.
            """
            time_mod.sleep(self.SPINUP_S)
            t_start = time_mod.monotonic()
            deadline = t_start + self.MOVE_TIMEOUT_S
            while time_mod.monotonic() < deadline:
                if self._stop:
                    return True
                # Request one fresh telemetry frame; its TLM reply is delivered
                # to state["last_tlm"] by the transport reader thread.
                try:
                    self._transport.send("SNAP")
                except Exception:  # noqa: BLE001
                    return False
                reply_deadline = time_mod.monotonic() + self.SNAP_REPLY_TIMEOUT_S
                while time_mod.monotonic() < reply_deadline:
                    if self._stop:
                        return True
                    cached = self._state.get("last_tlm")
                    if cached is not None:
                        frame, ts = cached
                        mode = (getattr(frame, "mode", None) or "").upper()
                        if ts >= t_start and mode == "I":
                            return True
                    time_mod.sleep(self.POLL_S)
            return False

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
            target_x_mm: int,
            target_y_mm: int,
            eps_mm: int,
            speed: int,
        ) -> None:
            super().__init__()
            self._transport = transport
            self._state = state
            self._tx = target_x_mm
            self._ty = target_y_mm
            self._eps = eps_mm
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
                    cur_x_mm = x_cm * 10.0
                    cur_y_mm = y_cm * 10.0

                    if goto_reached(self._tx, self._ty, cur_x_mm, cur_y_mm, self._eps):
                        self._safe_stop()
                        self.log_line.emit("[GOTO] reached target — complete", "")
                        return

                    # Correct the robot's internal pose to camera truth, then
                    # re-aim at the fixed world target.
                    si = build_setpose_command(x_cm, y_cm, yaw_rad)
                    g = f"G {self._tx} {self._ty} {self._speed}"
                    try:
                        self._transport.command(si, read_ms=200)
                        self._transport.command(g, read_ms=200)
                    except Exception as exc:  # noqa: BLE001
                        self.log_line.emit(f"[GOTO] send failed: {exc}", "")
                        return

                    # Throttled progress line (~1 Hz) — the raw SI/G traffic is
                    # visible via the transport, so we summarise here.
                    if now - last_status >= 1.0:
                        dist = goto_distance_mm(
                            self._tx, self._ty, cur_x_mm, cur_y_mm
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
            """Send a best-effort STOP to halt the robot."""
            try:
                self._transport.send("STOP")
            except Exception:  # noqa: BLE001
                pass

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

    transport_combo.currentIndexChanged.connect(_on_transport_changed)
    # Trigger once to set initial state.
    _on_transport_changed(transport_combo.currentIndex())

    def _on_robot_changed(index: int) -> None:
        """Load the robot selected in the dropdown (reloads on every change)."""
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

    robot_combo.currentIndexChanged.connect(_on_robot_changed)

    # Wire Send buttons — must happen after _append_log / _state are in scope.
    for _btn, _spec, _getters in _row_send_getters:
        _wire_send_button(_btn, _spec, _getters)

    # ---------------------------------------------------------------- ops panel callbacks

    def _clear_traces() -> None:
        """Clear all traces and refresh the canvas."""
        trace_model.clear()
        canvas_ctrl.refresh()

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
        0. Send ``STOP`` to halt motors and abort any in-flight motion goal,
           so the pose reset starts from a truly idle robot.  In Sim mode this
           is essential: a plant teleport is overwritten by the next tick if the
           firmware is still driving toward an old goal (heading drifts back).
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
        """
        transport = _state.get("transport")
        if transport is not None:
            # 0. Halt motors and abort any in-flight motion goal (TURN/tour/GOTO)
            #    BEFORE resetting the pose.  This matters especially in Sim: the
            #    plant teleport below is overwritten by the next tick if the
            #    firmware is still driving the motors toward an old goal (the
            #    heading would drift straight back — the "jumps back to the angle
            #    it started with" bug).  STOP first so PWM is zero when we
            #    teleport, and so the reset starts from a truly idle robot.
            transport.command("STOP", read_ms=300)
            # 0b. Sim only: teleport the plant ground-truth to (0, 0, 0°).
            #    In Sim mode the avatar follows the plant ground truth, not the
            #    firmware's belief.  On real hardware the operator physically
            #    places the robot at centre; the sim has no operator, so without
            #    this the plant keeps its prior (e.g. turned) pose and the avatar
            #    snaps back to it on the next truth delivery — while OZ/SI below
            #    would re-reference the OTOS at a stale heading.  Teleport AFTER
            #    STOP (so it sticks) and before OZ so OZ zeroes at heading 0.
            if is_sim_transport(transport):
                transport.set_true_pose(0.0, 0.0, 0.0)
            # 1. Zero encoder counters so SI starts from a clean state.
            transport.command("ZERO enc", read_ms=300)
            # 2. Zero the OTOS sensor (re-references heading to current orientation).
            transport.command("OZ", read_ms=300)
            # 3. Snap the fused/EKF pose to (0, 0, heading 0°).
            si_cmd = build_setpose_command(0.0, 0.0, 0.0)
            transport.command(si_cmd, read_ms=300)
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
        worker = _state.get("tour_worker")
        thread = _state.get("tour_thread")
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
        _state["tour_worker"] = None
        _state["tour_thread"] = None
        _state["tour_bridge"] = None
        if _state.get("transport") is not None:
            for _tb, _ in _tour_buttons:
                _tb.setEnabled(True)
        stop_tour_btn.setEnabled(False)

    def _on_tour_finished() -> None:
        """Main-thread slot: tour ended — join the thread, re-enable buttons.

        This is the natural-completion path only (the worker finished on
        its own); the explicit-stop path is handled synchronously inside
        ``_stop_tour`` itself and does not depend on this slot running.
        """
        thread = _state.get("tour_thread")
        if thread is not None:
            try:
                thread.quit()
                thread.wait(3000)
            except Exception:
                pass
        _state["tour_worker"] = None
        _state["tour_thread"] = None
        _state["tour_bridge"] = None
        if _state.get("transport") is not None:
            for _tb, _ in _tour_buttons:
                _tb.setEnabled(True)
        stop_tour_btn.setEnabled(False)

    def _make_tour_handler(name: str, steps: list[str]):
        def _on_tour_clicked() -> None:
            transport = _state.get("transport")
            if transport is None:
                _append_log("[WARN] Not connected")
                return
            if _state.get("tour_thread") is not None:
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
            from PySide6.QtCore import QThread  # type: ignore[import-untyped]

            worker = _TourRunner(transport, _state, name, list(steps))
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
            thread.start()
            _state["tour_worker"] = worker
            _state["tour_thread"] = thread
            _state["tour_bridge"] = bridge

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
        worker = _state.get("goto_worker")
        thread = _state.get("goto_thread")
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
        _state["goto_worker"] = None
        _state["goto_thread"] = None
        _state["goto_bridge"] = None
        if _state.get("transport") is not None:
            goto_btn.setEnabled(True)

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
        thread = _state.get("goto_thread")
        if thread is not None:
            try:
                thread.quit()
                thread.wait(3000)
            except Exception:
                pass
        _state["goto_worker"] = None
        _state["goto_thread"] = None
        _state["goto_bridge"] = None
        if _state.get("transport") is not None:
            goto_btn.setEnabled(True)

    def _on_goto_clicked() -> None:
        transport = _state.get("transport")
        if transport is None:
            _append_log("[WARN] Not connected")
            return
        if _state.get("goto_thread") is not None:
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
        thread.start()
        _state["goto_worker"] = worker
        _state["goto_thread"] = thread
        _state["goto_bridge"] = bridge

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
        name = transport_combo.currentText()
        port = port_edit.text().strip()

        transport: Transport | None = None

        if name == "Serial":
            if not port:
                _append_log("[ERROR] No port specified for Serial transport")
                return
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
        transport.on_telemetry = _on_telemetry_thread_v2
        transport.on_truth = _on_truth_thread

        # Clear any stale trace data from a previous session.
        trace_model.clear()

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

        # For Sim transport, STREAM 50 is sent internally by the tick-thread.
        # For hardware transports, send STREAM 50 here.
        if not isinstance(transport, SimTransport):
            try:
                reply = transport.command("STREAM 50", read_ms=300)
                if reply:
                    _append_log(f"[INFO] STREAM 50 → {reply}")
                else:
                    _append_log("[INFO] STREAM 50 sent")
            except Exception as exc:
                _append_log(f"[WARN] STREAM 50 failed: {exc}")

        _state["transport"] = transport

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

        # Attach cursor-key driving to the window.
        _driver.attach(window, transport)

        # Update button states.
        connect_btn.setEnabled(False)
        disconnect_btn.setEnabled(True)
        transport_combo.setEnabled(False)
        port_edit.setEnabled(False)
        # Enable all Send buttons now that a transport is connected.
        for _sb in _send_buttons:
            _sb.setEnabled(True)
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
        # Detach cursor-key driving before the transport goes away.
        _driver.detach()
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
    window, app = _build_main_window()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
