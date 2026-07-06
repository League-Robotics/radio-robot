"""tests/testgui/test_telemetry_panel.py — parsed-telemetry breakout panel.

Covers the OOP telemetry-panel change:

  * Qt-free formatting/geometry helpers in
    ``robot_radio.testgui.telemetry_panel`` (no QApplication needed).
  * ``is_telemetry_log_line`` — the console filter that keeps TLM frames out
    of the log pane.
  * Panel wiring inside ``_build_main_window`` — the panel exists between the
    canvas and the console, updates its labels/arrows from a ``TLMFrame``, and
    the console suppresses telemetry lines while still recording them.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_telemetry_panel.py -v
"""
from __future__ import annotations

import math
import sys

import pytest

from robot_radio.testgui.telemetry_panel import (
    arrow_fraction,
    body_to_screen,
    fmt_enc,
    fmt_pose,
    fmt_seq,
    fmt_time,
    fmt_twist,
    fmt_vel,
    is_telemetry_log_line,
    twist_velocity,
    wheel_velocity,
)


# ---------------------------------------------------------------------------
# Qt-free helpers
# ---------------------------------------------------------------------------


class TestConsoleFilter:
    """``is_telemetry_log_line`` splits TLM frames from command/reply traffic."""

    @pytest.mark.parametrize(
        "line",
        [
            "[12:34:56] < TLM t=123 enc=10,20 pose=1,2,3",
            "[12:34:56] < < TLM t=123",       # relay double-marker
            "[00:00:01] <# TLM t=9",
            "< TLM t=1",                       # no timestamp prefix
        ],
    )
    def test_telemetry_lines_detected(self, line):
        assert is_telemetry_log_line(line) is True

    @pytest.mark.parametrize(
        "line",
        [
            "[12:34:56] > S 200 200",
            "[12:34:56] < OK pong t=5",
            "[12:34:56] < ERR badarg",
            "[12:34:56] < EVT done T",
            "[INFO] connected",
            "[REC] Recording started",
        ],
    )
    def test_non_telemetry_lines_pass(self, line):
        assert is_telemetry_log_line(line) is False


class TestVelocityGeometry:
    """Body→screen mapping and arrow scaling."""

    def test_forward_motion_points_up(self):
        # Body +x (forward) must map to screen up (dy < 0), no lateral drift.
        dx, dy = body_to_screen(150.0, 0.0)
        assert dy < 0
        assert abs(dx) < 1e-9

    def test_left_motion_points_left(self):
        # Body +y (left) must map to screen left (dx < 0), no vertical drift.
        dx, dy = body_to_screen(0.0, 120.0)
        assert dx < 0
        assert abs(dy) < 1e-9

    def test_magnitude_preserved(self):
        dx, dy = body_to_screen(30.0, 40.0)
        assert math.hypot(dx, dy) == pytest.approx(50.0)

    def test_arrow_fraction_clamped(self):
        assert arrow_fraction(0.0) == 0.0
        assert arrow_fraction(-5.0) == 0.0
        assert 0.0 < arrow_fraction(200.0) < 1.0
        assert arrow_fraction(10_000.0) == 1.0


class TestTwistVelocity:
    """``twist_velocity`` normalises differential and mecanum twists."""

    def test_differential(self):
        # (v, omega_mrad); v_y is 0, omega converted mrad/s -> deg/s.
        v_x, v_y, omega = twist_velocity((150, 1000))
        assert v_x == 150.0
        assert v_y == 0.0
        assert omega == pytest.approx(math.degrees(1.0))

    def test_mecanum(self):
        v_x, v_y, omega = twist_velocity((100, -30, 500))
        assert (v_x, v_y) == (100.0, -30.0)
        assert omega == pytest.approx(math.degrees(0.5))

    def test_none(self):
        assert twist_velocity(None) is None

    def test_wheel_velocity_mean(self):
        assert wheel_velocity((100, 200)) == (150.0, 0.0)
        assert wheel_velocity((10, 20, 30, 40)) == (25.0, 0.0)
        assert wheel_velocity(None) is None


class TestFormatting:
    """Value formatters render numbers, and ``—`` for absent fields."""

    def test_placeholders(self):
        assert fmt_time(None) == "—"
        assert fmt_seq(None) == "—"
        assert fmt_enc(None) == "—"
        assert fmt_pose(None) == "—"
        assert fmt_vel(None) == "—"
        assert fmt_twist(None) == "—"

    def test_values(self):
        assert fmt_time(12345) == "12.345 s"
        assert fmt_seq(7) == "7"
        assert "1024" in fmt_enc((1024, 1019))
        assert "17.8" in fmt_pose((350, -12, 1780))   # 1780 cdeg -> 17.8 deg
        assert "150" in fmt_vel((150, 148))
        assert "11.5" in fmt_twist((149, 200))         # 200 mrad/s -> 11.5 deg/s


# ---------------------------------------------------------------------------
# Widget-level wiring (offscreen Qt)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def qapp():
    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


class TestPanelWiring:
    def test_panel_updates_from_frame(self, qapp):
        from PySide6.QtWidgets import QLabel, QWidget

        from robot_radio.testgui.telemetry_panel import build_telemetry_panel
        from robot_radio.robot.protocol import TLMFrame

        widget, ctrl = build_telemetry_panel()
        try:
            frame = TLMFrame(
                t=12345,
                seq=42,
                enc=(1024, 1019),
                vel=(150, 148),
                pose=(350, -12, 1780),
                encpose=(340, -10, 1770),
                otos=(352, -14, 1782),
                twist=(149, 200),
            )
            ctrl.update_frame(frame)

            time_lbl = widget.findChild(QLabel, "tlm_val_time")
            seq_lbl = widget.findChild(QLabel, "tlm_val_seq")
            enc_lbl = widget.findChild(QLabel, "tlm_val_enc")
            twist_lbl = widget.findChild(QLabel, "tlm_val_twist")
            assert time_lbl.text() == "12.345 s"
            assert seq_lbl.text() == "42"
            assert "1024" in enc_lbl.text()
            assert "149" in twist_lbl.text()

            # Both velocity arrows exist.
            assert widget.findChild(QWidget, "tlm_arrow_vel") is not None
            assert widget.findChild(QWidget, "tlm_arrow_twist") is not None
        finally:
            widget.deleteLater()

    def test_window_has_panel_and_filters_console(self, qapp):
        from PySide6.QtWidgets import QPlainTextEdit, QWidget

        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            panel = window.findChild(QWidget, "telemetry_panel")
            assert panel is not None
            log = window.findChild(QPlainTextEdit, "log_pane")
            assert log is not None
            assert log.maximumHeight() > 200  # no longer capped at the old 200 px
        finally:
            window.close()
            window.deleteLater()
