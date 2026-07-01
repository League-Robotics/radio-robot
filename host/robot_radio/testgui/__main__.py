"""Entry point for ``python -m robot_radio.testgui``.

Launches the Robot Test GUI main window.  Requires PySide6 (install via
``uv sync --group gui``).

All PySide6 imports are kept inside this module so that the package itself can
be imported without PySide6 present.
"""

from __future__ import annotations

import sys


def _build_main_window():  # type: ignore[return]
    """Build and return the main QMainWindow with transport wiring.

    Layout (left-to-right via QSplitter):
    - Left panel: transport selector QComboBox, port QLineEdit, Connect
      button, placeholder command rows, and placeholder operations panel.
    - Right panel: placeholder QGraphicsView canvas (top) and timestamped
      log pane QPlainTextEdit (bottom).

    The transport selector enables the port QLineEdit when Serial or Relay
    is selected.  Clicking Connect instantiates the selected Transport,
    calls transport.connect(), then sends ``STREAM 50``.  Clicking
    Disconnect calls transport.disconnect().

    The log pane receives all sent and received lines via the transport's
    on_log callback, delivered safely from background threads via
    QMetaObject.invokeMethod.
    """
    # PySide6 imports are intentionally deferred here.
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QApplication,
        QComboBox,
        QGraphicsScene,
        QGraphicsView,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QPlainTextEdit,
        QPushButton,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )
    from PySide6.QtCore import Qt, QMetaObject, Q_ARG  # type: ignore[import-untyped]
    from PySide6.QtCore import Slot  # type: ignore[import-untyped]

    from robot_radio.testgui.transport import (
        Transport,
        SerialTransport,
        RelayTransport,
        SimTransport,
        list_ports,
    )

    # QApplication must exist before any QWidget is created.  We create one
    # only if one does not already exist (e.g. during testing).
    app = QApplication.instance() or QApplication(sys.argv)

    # Active transport — kept in a mutable container so inner functions can
    # rebind it without 'nonlocal' limitations across closures.
    _state: dict = {"transport": None}

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

    # Placeholder: command rows area
    cmd_placeholder = QWidget()
    cmd_placeholder.setObjectName("cmd_rows_placeholder")
    cmd_placeholder.setMinimumHeight(120)
    cmd_label = QLabel("(command rows — coming in later tickets)")
    cmd_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
    cmd_inner = QVBoxLayout(cmd_placeholder)
    cmd_inner.addWidget(cmd_label)
    left_layout.addWidget(cmd_placeholder)

    # Placeholder: operations panel
    ops_placeholder = QWidget()
    ops_placeholder.setObjectName("ops_panel_placeholder")
    ops_placeholder.setMinimumHeight(120)
    ops_label = QLabel("(operations panel — coming in later tickets)")
    ops_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
    ops_inner = QVBoxLayout(ops_placeholder)
    ops_inner.addWidget(ops_label)
    left_layout.addWidget(ops_placeholder)

    left_layout.addStretch()
    splitter.addWidget(left_widget)

    # --------------------------------------------------------------- right panel
    right_widget = QWidget()
    right_layout = QVBoxLayout(right_widget)
    right_layout.setContentsMargins(4, 4, 4, 4)

    right_splitter = QSplitter(Qt.Orientation.Vertical)
    right_layout.addWidget(right_splitter)

    # Placeholder: canvas (QGraphicsView)
    scene = QGraphicsScene()
    canvas = QGraphicsView(scene)
    canvas.setObjectName("canvas_view")
    canvas.setMinimumSize(400, 300)
    right_splitter.addWidget(canvas)

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

    def _on_transport_changed(index: int) -> None:
        """Enable/disable port picker depending on selected transport."""
        name = transport_combo.currentText()
        hardware = name in ("Serial", "Relay")
        port_edit.setEnabled(hardware)
        port_label.setEnabled(hardware)

    transport_combo.currentIndexChanged.connect(_on_transport_changed)
    # Trigger once to set initial state.
    _on_transport_changed(transport_combo.currentIndex())

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

        # Update button states.
        connect_btn.setEnabled(False)
        disconnect_btn.setEnabled(True)
        transport_combo.setEnabled(False)
        port_edit.setEnabled(False)
        desc = "Sim" if name == "Sim" else f"{name} on {port}"
        _append_log(f"[INFO] Connected via {desc}")

    def _on_disconnect() -> None:
        """Call transport.disconnect() and clean up."""
        transport: Transport | None = _state.get("transport")
        if transport is None:
            return
        try:
            transport.disconnect()
        except Exception as exc:
            _append_log(f"[WARN] Disconnect error: {exc}")
        _state["transport"] = None

        # Restore button/combo state.
        connect_btn.setEnabled(True)
        disconnect_btn.setEnabled(False)
        transport_combo.setEnabled(True)
        # Re-enable port field if a hardware transport was selected.
        _on_transport_changed(transport_combo.currentIndex())
        _append_log("[INFO] Disconnected")

    connect_btn.clicked.connect(_on_connect)
    disconnect_btn.clicked.connect(_on_disconnect)

    return window, app


def main() -> None:
    """Launch the Robot Test GUI and block until the window is closed."""
    window, app = _build_main_window()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
