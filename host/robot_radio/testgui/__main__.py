"""Entry point for ``python -m robot_radio.testgui``.

Launches the Robot Test GUI main window.  Requires PySide6 (install via
``uv sync --group gui``).

All PySide6 imports are kept inside this module so that the package itself can
be imported without PySide6 present.
"""

from __future__ import annotations

import sys


def _build_main_window():  # type: ignore[return]
    """Build and return a skeleton QMainWindow.

    Layout (left-to-right via QSplitter):
    - Left panel: transport selector QComboBox, placeholder command rows, and
      placeholder operations panel (QWidget).
    - Right panel: placeholder QGraphicsView canvas (top) and placeholder
      QPlainTextEdit log pane (bottom).
    """
    # PySide6 imports are intentionally deferred here.
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QApplication,
        QComboBox,
        QGraphicsScene,
        QGraphicsView,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPlainTextEdit,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )
    from PySide6.QtCore import Qt  # type: ignore[import-untyped]

    # QApplication must exist before any QWidget is created.  We create one
    # only if one does not already exist (e.g. during testing).
    app = QApplication.instance() or QApplication(sys.argv)

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

    # Placeholder: log pane (QPlainTextEdit)
    log_pane = QPlainTextEdit()
    log_pane.setObjectName("log_pane")
    log_pane.setReadOnly(True)
    log_pane.setPlaceholderText("(log output will appear here)")
    log_pane.setMaximumHeight(150)
    right_splitter.addWidget(log_pane)

    splitter.addWidget(right_widget)

    # Reasonable initial splitter proportions: 30% left / 70% right
    splitter.setSizes([360, 840])

    return window, app


def main() -> None:
    """Launch the Robot Test GUI and block until the window is closed."""
    window, app = _build_main_window()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
