"""tests/testgui/test_drive.py — headless unit tests for drive.py.

Tests are split into two groups:
1. Pure-function tests — assert key-combo → correct VW wire string.
   These need PySide6 (to resolve Qt.Key constants) but NOT a QApplication.
2. KeyboardDriver lifecycle tests — press/release/timer/STOP using a fake
   transport and simulated key events.  Requires QApplication (offscreen).

Run with:
    uv run python -m pytest tests/testgui -q

Requirements: PySide6 (uv sync --group gui).
The conftest sets QT_QPA_PLATFORM=offscreen before any PySide6 import.
"""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import pytest


# ---------------------------------------------------------------------------
# Fake transport
# ---------------------------------------------------------------------------


class FakeTransport:
    """Minimal fake Transport that records send() calls."""

    def __init__(self) -> None:
        self.sent: list[str] = []

    def send(self, line: str) -> None:
        self.sent.append(line)

    def command(self, line: str, read_ms: int = 200) -> str:
        return "OK"


# ---------------------------------------------------------------------------
# Helpers for constructing fake Qt key events
# ---------------------------------------------------------------------------


def _make_key_event(key, is_auto_repeat: bool = False) -> MagicMock:
    """Return a MagicMock that quacks like a QKeyEvent."""
    evt = MagicMock()
    evt.key.return_value = int(key)
    evt.isAutoRepeat.return_value = is_auto_repeat
    return evt


# ---------------------------------------------------------------------------
# Pure-function tests — no QApplication needed
# ---------------------------------------------------------------------------


class TestVwLineForKey:
    """vw_line_for_key() maps Qt.Key arrow constants to VW strings."""

    def test_up_returns_forward(self):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key, FWD_SPEED_MMS

        line = vw_line_for_key(int(Qt.Key.Key_Up))
        assert line == f"VW {FWD_SPEED_MMS} 0"

    def test_down_returns_back(self):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key, FWD_SPEED_MMS

        line = vw_line_for_key(int(Qt.Key.Key_Down))
        assert line == f"VW -{FWD_SPEED_MMS} 0"

    def test_left_returns_ccw(self):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key, ROTATE_OMEGA_MRADS

        line = vw_line_for_key(int(Qt.Key.Key_Left))
        assert line == f"VW 0 {ROTATE_OMEGA_MRADS}"

    def test_right_returns_cw(self):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key, ROTATE_OMEGA_MRADS

        line = vw_line_for_key(int(Qt.Key.Key_Right))
        assert line == f"VW 0 -{ROTATE_OMEGA_MRADS}"

    def test_non_arrow_key_returns_none(self):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key

        # Space bar is not an arrow key.
        line = vw_line_for_key(int(Qt.Key.Key_Space))
        assert line is None

    def test_default_speed_constants(self):
        """Named constants must match the ticket-specified defaults."""
        from robot_radio.testgui.drive import FWD_SPEED_MMS, ROTATE_OMEGA_MRADS

        assert FWD_SPEED_MMS == 200
        assert ROTATE_OMEGA_MRADS == 500

    def test_up_wire_string_literal(self):
        """Ticket literal: Up held → 'VW 200 0'."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key

        assert vw_line_for_key(int(Qt.Key.Key_Up)) == "VW 200 0"

    def test_down_wire_string_literal(self):
        """Ticket literal: Down held → 'VW -200 0'."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key

        assert vw_line_for_key(int(Qt.Key.Key_Down)) == "VW -200 0"

    def test_left_wire_string_literal(self):
        """Ticket literal: Left held → 'VW 0 500'."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key

        assert vw_line_for_key(int(Qt.Key.Key_Left)) == "VW 0 500"

    def test_right_wire_string_literal(self):
        """Ticket literal: Right held → 'VW 0 -500'."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key

        assert vw_line_for_key(int(Qt.Key.Key_Right)) == "VW 0 -500"


class TestVwLineForKeySet:
    """vw_line_for_key_set() maps a frozenset of held key ints to VW strings."""

    def test_single_up_key(self):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key_set, FWD_SPEED_MMS

        held = frozenset([int(Qt.Key.Key_Up)])
        assert vw_line_for_key_set(held) == f"VW {FWD_SPEED_MMS} 0"

    def test_single_left_key(self):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key_set, ROTATE_OMEGA_MRADS

        held = frozenset([int(Qt.Key.Key_Left)])
        assert vw_line_for_key_set(held) == f"VW 0 {ROTATE_OMEGA_MRADS}"

    def test_empty_set_returns_none(self):
        from robot_radio.testgui.drive import vw_line_for_key_set

        assert vw_line_for_key_set(frozenset()) is None

    def test_non_arrow_keys_return_none(self):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import vw_line_for_key_set

        held = frozenset([int(Qt.Key.Key_A), int(Qt.Key.Key_B)])
        assert vw_line_for_key_set(held) is None


# ---------------------------------------------------------------------------
# KeyboardDriver lifecycle tests — require QApplication
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def qapp():
    """Module-scoped QApplication (offscreen) for driver tests."""
    from PySide6.QtWidgets import QApplication
    import sys

    app = QApplication.instance() or QApplication(sys.argv)
    return app


@pytest.fixture()
def fake_window(qapp):
    """A minimal QMainWindow with no-op key event handlers."""
    from PySide6.QtWidgets import QMainWindow

    w = QMainWindow()
    return w


class TestKeyboardDriverAttachDetach:
    """attach() / detach() install and remove key event overrides."""

    def test_attach_installs_key_press_override(self, fake_window):
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        original_press = fake_window.keyPressEvent
        driver.attach(fake_window, transport)

        # The key press handler must have been replaced.
        assert fake_window.keyPressEvent is not original_press
        driver.detach()

    def test_detach_restores_original_key_press(self, fake_window):
        """After detach, key press events no longer route through the driver.

        PySide6 bound-method objects are transient (new wrapper each access),
        so we verify behavior: after detach, pressing Up sends nothing.
        """
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)
        driver.detach()

        # After detach the driver's handler is no longer installed.
        # Pressing Up via the driver's handler directly must not send VW.
        evt = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        driver._on_key_press(evt)
        vw_sent = [s for s in transport.sent if s.startswith("VW")]
        assert vw_sent == [], (
            "Driver sent VW after detach — transport reference not cleared"
        )

    def test_detach_without_attach_does_not_raise(self):
        from robot_radio.testgui.drive import KeyboardDriver

        driver = KeyboardDriver()
        driver.detach()  # must not raise

    def test_double_attach_detaches_first(self, fake_window, qapp):
        """Calling attach() twice replaces the previous attachment cleanly."""
        from robot_radio.testgui.drive import KeyboardDriver

        t1 = FakeTransport()
        t2 = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, t1)
        driver.attach(fake_window, t2)  # should detach t1 first
        assert driver._transport is t2
        driver.detach()


class TestKeyboardDriverKeyPress:
    """Key press events dispatch the correct VW command."""

    def test_up_press_sends_vw_forward(self, fake_window, qapp):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        evt = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(evt)
        driver.detach()

        assert "VW 200 0" in transport.sent

    def test_down_press_sends_vw_back(self, fake_window, qapp):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        evt = _make_key_event(Qt.Key.Key_Down, is_auto_repeat=False)
        fake_window.keyPressEvent(evt)
        driver.detach()

        assert "VW -200 0" in transport.sent

    def test_left_press_sends_vw_ccw(self, fake_window, qapp):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        evt = _make_key_event(Qt.Key.Key_Left, is_auto_repeat=False)
        fake_window.keyPressEvent(evt)
        driver.detach()

        assert "VW 0 500" in transport.sent

    def test_right_press_sends_vw_cw(self, fake_window, qapp):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        evt = _make_key_event(Qt.Key.Key_Right, is_auto_repeat=False)
        fake_window.keyPressEvent(evt)
        driver.detach()

        assert "VW 0 -500" in transport.sent

    def test_auto_repeat_press_is_ignored(self, fake_window, qapp):
        """Auto-repeat key events must not generate additional sends."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        # One real press, then three auto-repeats.
        evt_real = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        evt_repeat = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=True)
        fake_window.keyPressEvent(evt_real)
        count_after_press = len(transport.sent)
        fake_window.keyPressEvent(evt_repeat)
        fake_window.keyPressEvent(evt_repeat)
        fake_window.keyPressEvent(evt_repeat)
        driver.detach()

        # Auto-repeat events must not increment the send count.
        assert len(transport.sent) == count_after_press, (
            f"Auto-repeat events triggered extra sends: {transport.sent}"
        )

    def test_non_arrow_key_press_not_handled(self, fake_window, qapp):
        """Non-arrow keys must not generate VW sends.

        We call the driver's handler directly so a MagicMock QKeyEvent is not
        forwarded to the C++ original handler (which requires a real QKeyEvent).
        """
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        # Call driver's handler directly to avoid forwarding mock to C++.
        evt = _make_key_event(Qt.Key.Key_Space, is_auto_repeat=False)
        # Temporarily replace _orig_key_press so the forward doesn't call C++.
        driver._orig_key_press = lambda e: None
        driver._on_key_press(evt)
        driver.detach()

        vw_sent = [s for s in transport.sent if s.startswith("VW")]
        assert vw_sent == [], f"Non-arrow key triggered VW send: {transport.sent}"


class TestKeyboardDriverKeyRelease:
    """Key release events send STOP and stop the timer."""

    def test_release_sends_stop(self, fake_window, qapp):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        release = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        fake_window.keyReleaseEvent(release)
        driver.detach()

        assert "STOP" in transport.sent

    def test_release_stops_timer(self, fake_window, qapp):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        release = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        assert driver._timer is not None and driver._timer.isActive()

        fake_window.keyReleaseEvent(release)
        assert driver._timer is None or not driver._timer.isActive()
        driver.detach()

    def test_auto_repeat_release_is_ignored(self, fake_window, qapp):
        """Auto-repeat release events must not send STOP prematurely."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        release_repeat = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=True)
        fake_window.keyPressEvent(press)
        fake_window.keyReleaseEvent(release_repeat)

        # Auto-repeat release must NOT send STOP.
        assert "STOP" not in transport.sent
        driver.detach()

    def test_non_arrow_key_release_not_handled(self, fake_window, qapp):
        """Non-arrow key releases must not send STOP.

        We call the driver's handler directly so a MagicMock QKeyEvent is not
        forwarded to the C++ original handler (which requires a real QKeyEvent).
        """
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        evt = _make_key_event(Qt.Key.Key_Space, is_auto_repeat=False)
        # Replace _orig_key_release so forward doesn't call C++.
        driver._orig_key_release = lambda e: None
        driver._on_key_release(evt)
        driver.detach()

        assert "STOP" not in transport.sent


class TestKeyboardDriverKeepalive:
    """Timer fires repeatedly while key is held, sending the VW command."""

    def test_timer_starts_on_key_press(self, fake_window, qapp):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver(interval_ms=50)
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(press)

        assert driver._timer is not None
        assert driver._timer.isActive(), "Timer must be active while key is held"
        driver.detach()

    def test_timer_tick_resends_cmd(self, fake_window, qapp):
        """Simulating a timer tick sends the current VW command again."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver(interval_ms=100)
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Down, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        count_after_press = len(transport.sent)

        # Simulate timer tick directly.
        driver._on_timer_tick()
        driver._on_timer_tick()

        assert len(transport.sent) == count_after_press + 2
        assert transport.sent[-1] == "VW -200 0"
        driver.detach()

    def test_timer_stops_on_release(self, fake_window, qapp):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver(interval_ms=50)
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Left, is_auto_repeat=False)
        release = _make_key_event(Qt.Key.Key_Left, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        assert driver._timer is not None and driver._timer.isActive()

        fake_window.keyReleaseEvent(release)
        assert driver._timer is None or not driver._timer.isActive()
        driver.detach()


class TestKeyboardDriverNoTransport:
    """Driver ignores key events when no transport is attached."""

    def test_key_press_without_transport_does_not_raise(self, fake_window, qapp):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        driver = KeyboardDriver()
        driver.attach(fake_window, transport=None)  # type: ignore[arg-type]

        evt = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(evt)  # must not raise or send
        driver.detach()

    def test_key_release_without_transport_does_not_raise(self, fake_window, qapp):
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        driver = KeyboardDriver()
        driver.attach(fake_window, transport=None)  # type: ignore[arg-type]

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        release = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        fake_window.keyReleaseEvent(release)  # must not raise
        driver.detach()


class TestKeyboardDriverImportWithoutPySide6:
    """drive module is importable without a live QApplication.

    The constants FWD_SPEED_MMS and ROTATE_OMEGA_MRADS must be accessible at
    the module level (no lazy guard needed for them).
    """

    def test_constants_importable(self):
        from robot_radio.testgui.drive import FWD_SPEED_MMS, ROTATE_OMEGA_MRADS

        assert isinstance(FWD_SPEED_MMS, int)
        assert isinstance(ROTATE_OMEGA_MRADS, int)

    def test_keyboard_driver_class_importable(self):
        from robot_radio.testgui.drive import KeyboardDriver

        assert KeyboardDriver is not None

    def test_vw_line_for_key_importable(self):
        from robot_radio.testgui.drive import vw_line_for_key

        assert callable(vw_line_for_key)
