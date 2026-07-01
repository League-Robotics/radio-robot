"""Entry point for ``python -m robot_radio.testgui``.

Launches the Robot Test GUI main window.  Requires PySide6 (install via
``uv sync --group gui``).

All PySide6 imports are kept inside this module so that the package itself can
be imported without PySide6 present.

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
"""

from __future__ import annotations

import math
import sys


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
    from robot_radio.testgui.commands import COMMANDS, build_wire_string
    from robot_radio.testgui.operations import build_panel as _build_ops_panel
    from robot_radio.testgui.traces import TraceModel
    from robot_radio.testgui.canvas import build_canvas
    from robot_radio.testgui.drive import KeyboardDriver

    # QApplication must exist before any QWidget is created.  We create one
    # only if one does not already exist (e.g. during testing).
    app = QApplication.instance() or QApplication(sys.argv)

    # Active transport — kept in a mutable container so inner functions can
    # rebind it without 'nonlocal' limitations across closures.
    _state: dict = {"transport": None}

    # Keyboard driver — wires cursor-key VW driving onto the main window.
    _driver = KeyboardDriver()

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
            _append_log(f"TX {line}")
            try:
                reply = transport.command(line, read_ms=500)
                if reply:
                    _append_log(f"RX {reply.strip()}")
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

    left_layout.addStretch()
    splitter.addWidget(left_widget)

    # --------------------------------------------------------- TraceModel (Qt-free)
    trace_model = TraceModel()

    # --------------------------------------------------------------- right panel
    right_widget = QWidget()
    right_layout = QVBoxLayout(right_widget)
    right_layout.setContentsMargins(4, 4, 4, 4)

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

    def _append_log(text: str) -> None:
        """Append *text* to the log pane.  Must be called from the Qt main thread."""
        log_pane.appendPlainText(text)
        # Auto-scroll to bottom.
        sb = log_pane.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _on_log_from_thread(text: str) -> None:
        """Thread-safe log delivery via QMetaObject.invokeMethod."""
        QMetaObject.invokeMethod(
            log_pane,
            "appendPlainText",
            Qt.ConnectionType.QueuedConnection,
            Q_ARG(str, text),
        )

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
            """Process all pending TLMFrames queued from background threads."""
            while True:
                try:
                    frame = _pending_frames.get_nowait()
                except Exception:
                    break
                trace_model.feed(frame)
                fused_yaw_rad = None
                if frame.pose is not None:
                    fused_yaw_rad = math.radians(frame.pose[2] / 100.0)
                canvas_ctrl.refresh(fused_yaw_rad)

        @Slot(float, float, float)
        def on_truth_ready(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
            """Process a ground-truth pose update on the Qt main thread."""
            trace_model.feed_truth(x_cm, y_cm, yaw_rad)
            canvas_ctrl.refresh()

    _bridge = _TelemetryBridge()
    _bridge.frame_ready.connect(_bridge.on_frame_ready, Qt.ConnectionType.QueuedConnection)
    _bridge.truth_ready.connect(_bridge.on_truth_ready, Qt.ConnectionType.QueuedConnection)

    def _on_telemetry_thread_v2(frame: "object") -> None:
        """Transport on_telemetry callback — fires on the reader/tick thread.

        Enqueues the frame and emits the bridge signal to wake the Qt main
        thread.
        """
        _pending_frames.put(frame)
        _bridge.frame_ready.emit()  # type: ignore[attr-defined]

    def _on_truth_thread(pose: "tuple | None") -> None:
        """Transport on_truth callback — fires on the truth/tick thread.

        Ignores ``None`` (camera not available); emits bridge signal for a
        valid pose.
        """
        if pose is not None:
            x_cm, y_cm, yaw_rad = pose
            _bridge.truth_ready.emit(x_cm, y_cm, yaw_rad)  # type: ignore[attr-defined]

    def _on_transport_changed(index: int) -> None:
        """Enable/disable port picker depending on selected transport."""
        name = transport_combo.currentText()
        hardware = name in ("Serial", "Relay")
        port_edit.setEnabled(hardware)
        port_label.setEnabled(hardware)

    transport_combo.currentIndexChanged.connect(_on_transport_changed)
    # Trigger once to set initial state.
    _on_transport_changed(transport_combo.currentIndex())

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
        """Re-anchor the TraceModel to the current pose as world (0,0).

        Operator workflow: physically place the robot at the playfield centre,
        then click "Set Robot @ 0,0" to tell the GUI the robot is at (0,0).

        This re-anchors the TraceModel (so the next incoming pose delta
        starts from world 0,0), clears all trace polylines, and moves the
        canvas avatar to centre.  No motion command is sent.
        """
        # Re-anchor: current origin maps to (0, 0).  Use heading 0 (east) as
        # the new forward direction for the body-to-world transform.
        trace_model.anchor(0.0, 0.0, 0.0)
        trace_model.clear()
        canvas_ctrl.reset_avatar_to_center()
        canvas_ctrl.refresh()

    # Operations panel — built after _append_log is defined so the log callback
    # is live.
    ops_panel, ops_ctrl = _build_ops_panel(
        log_cb=_append_log,
        transport_ref=_state,
        clear_traces_cb=_clear_traces,
        refresh_playfield_cb=_refresh_playfield,
        set_origin_cb=_set_origin,
    )
    # Insert the ops panel before the addStretch() already added above.
    # Because addStretch() was called already, insert at the position before it.
    left_layout.insertWidget(left_layout.count() - 1, ops_panel)

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
            if not port:
                _append_log("[ERROR] No port specified for Relay transport")
                return
            transport = RelayTransport(port)
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

    connect_btn.clicked.connect(_on_connect)
    disconnect_btn.clicked.connect(_on_disconnect)

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
