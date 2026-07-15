"""tests/testgui/test_mode_indicator.py -- ticket 085-008: mode label test
port. Ported from ``tests_old/testgui/test_mode_indicator.py``.

Covers both the pure Qt-free helper ``transport_name_to_mode_label`` and the
live widget behaviour via ``_build_main_window()``.

No production code change: pure verification pass.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_mode_indicator.py -v
"""

from __future__ import annotations

import sys

import pytest


# ---------------------------------------------------------------------------
# qapp fixture -- ensure a QApplication exists (offscreen, module-scoped)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def qapp():
    """Ensure a QApplication exists for the whole module.

    ``QT_QPA_PLATFORM=offscreen`` is already set by conftest.py before this
    import runs.
    """
    # 107-004: turn a missing `gui` dependency group into a clean skip, not
    # a hard collection/run error -- see test_tour1_geometry.py's module
    # docstring for the full rationale.
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app
    # Do NOT call app.quit() -- other tests in the same session may need it.


# ---------------------------------------------------------------------------
# Qt-free unit tests -- no QApplication required
# ---------------------------------------------------------------------------


def test_transport_name_to_mode_label_sim():
    """'Sim' -> 'SIM MODE'."""
    from robot_radio.testgui.__main__ import transport_name_to_mode_label

    text, _ = transport_name_to_mode_label("Sim")
    assert text == "SIM MODE"


def test_transport_name_to_mode_label_serial():
    """'Serial' -> 'BENCH MODE'."""
    from robot_radio.testgui.__main__ import transport_name_to_mode_label

    text, _ = transport_name_to_mode_label("Serial")
    assert text == "BENCH MODE"


def test_transport_name_to_mode_label_relay():
    """'Relay' -> 'PLAYFIELD MODE'."""
    from robot_radio.testgui.__main__ import transport_name_to_mode_label

    text, _ = transport_name_to_mode_label("Relay")
    assert text == "PLAYFIELD MODE"


def test_transport_name_to_mode_label_unknown():
    """Unknown transport names return a safe fallback containing 'MODE'."""
    from robot_radio.testgui.__main__ import transport_name_to_mode_label

    text, _ = transport_name_to_mode_label("Bluetooth")
    assert "MODE" in text


def test_transport_name_to_mode_label_returns_tuple():
    """Return value is a 2-tuple of (str, str)."""
    from robot_radio.testgui.__main__ import transport_name_to_mode_label

    result = transport_name_to_mode_label("Sim")
    assert isinstance(result, tuple)
    assert len(result) == 2
    assert isinstance(result[0], str)
    assert isinstance(result[1], str)


def test_transport_name_to_mode_label_style_is_nonempty():
    """Style string is non-empty for all known transports."""
    from robot_radio.testgui.__main__ import transport_name_to_mode_label

    for name in ("Sim", "Serial", "Relay"):
        _, style = transport_name_to_mode_label(name)
        assert style, f"Expected non-empty style for transport '{name}'"


# ---------------------------------------------------------------------------
# Qt widget tests -- require QApplication via conftest offscreen fixture
# ---------------------------------------------------------------------------


class TestModeLabel:
    """Mode label widget updates correctly when the combo changes."""

    def test_mode_label_exists(self, qapp):
        """The right panel must have a QLabel with objectName 'mode_label'."""
        from robot_radio.testgui.__main__ import _build_main_window
        from PySide6.QtWidgets import QLabel  # type: ignore[import-untyped]

        window, _ = _build_main_window()
        label = window.findChild(QLabel, "mode_label")
        assert label is not None, "Expected a QLabel with objectName 'mode_label'"
        window.close()

    def test_mode_label_initial_text(self, qapp):
        """Initial combo selection is 'Sim', so label text must be 'SIM MODE'."""
        from robot_radio.testgui.__main__ import _build_main_window
        from PySide6.QtWidgets import QComboBox, QLabel  # type: ignore[import-untyped]

        window, _ = _build_main_window()
        combo = window.findChild(QComboBox, "transport_combo")
        label = window.findChild(QLabel, "mode_label")
        assert combo is not None
        assert label is not None
        assert combo.currentText() == "Sim"
        assert label.text() == "SIM MODE"
        window.close()

    def test_mode_label_updates_on_combo_change(self, qapp):
        """Changing combo selection immediately updates the mode label text."""
        from robot_radio.testgui.__main__ import _build_main_window
        from PySide6.QtWidgets import QComboBox, QLabel  # type: ignore[import-untyped]

        window, _ = _build_main_window()
        combo = window.findChild(QComboBox, "transport_combo")
        label = window.findChild(QLabel, "mode_label")
        assert combo is not None
        assert label is not None

        combo.setCurrentText("Relay")
        assert label.text() == "PLAYFIELD MODE"

        combo.setCurrentText("Serial")
        assert label.text() == "BENCH MODE"

        combo.setCurrentText("Sim")
        assert label.text() == "SIM MODE"

        window.close()
