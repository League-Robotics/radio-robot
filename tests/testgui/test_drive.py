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
    """Minimal fake Transport that records send() and keepalive arm/disarm calls."""

    def __init__(self) -> None:
        self.sent: list[str] = []
        # "arm" / "disarm", in call order -- used to verify KeyboardDriver
        # correctly brackets a drive session (sprint 065, ticket 005).
        self.keepalive_events: list[str] = []

    def send(self, line: str) -> None:
        self.sent.append(line)

    def command(self, line: str, read_ms: int = 200) -> str:
        return "OK"

    def arm_keepalive(self) -> None:
        self.keepalive_events.append("arm")

    def disarm_keepalive(self) -> None:
        self.keepalive_events.append("disarm")


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

    def test_release_starts_deadman_timer_keeps_running(self, fake_window, qapp):
        """Release begins the bounded STOP deadman resend; the timer keeps
        running through the deadman window instead of stopping immediately
        (it stops once the deadman sequence completes -- see
        TestKeyboardDriverStopDeadman)."""
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
        assert driver._timer is not None and driver._timer.isActive(), (
            "Timer must stay active through the bounded STOP deadman window"
        )
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

    def test_timer_eventually_stops_after_release_deadman_completes(
        self, fake_window, qapp
    ):
        """The timer stops once the bounded STOP deadman sequence completes,
        not immediately on release (see TestKeyboardDriverStopDeadman for the
        exact resend-count assertions)."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver, STOP_RESEND_COUNT

        transport = FakeTransport()
        driver = KeyboardDriver(interval_ms=50)
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Left, is_auto_repeat=False)
        release = _make_key_event(Qt.Key.Key_Left, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        assert driver._timer is not None and driver._timer.isActive()

        fake_window.keyReleaseEvent(release)
        assert driver._timer is not None and driver._timer.isActive()

        # Drive the remaining deadman ticks directly.
        for _ in range(STOP_RESEND_COUNT - 1):
            driver._on_timer_tick()

        assert driver._timer is None or not driver._timer.isActive()
        driver.detach()


class TestKeyboardDriverStopDeadman:
    """STOP deadman resend (CR-04 / ticket 065-004): a dropped fire-and-
    forget STOP no longer leaves the robot driving forever.  Release
    resends STOP a bounded number of times (STOP_RESEND_COUNT) before the
    timer actually stops, and focus loss triggers the same sequence."""

    def test_release_resends_stop_bounded_count_then_stops(self, fake_window, qapp):
        """Release sends STOP immediately, then the timer resends it for
        STOP_RESEND_COUNT - 1 further ticks, then stops -- STOP_RESEND_COUNT
        total STOP sends, no more."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver, STOP_RESEND_COUNT

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        release = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        fake_window.keyReleaseEvent(release)

        # The release tick itself is the first STOP send.
        stops_after_release = [s for s in transport.sent if s == "STOP"]
        assert len(stops_after_release) == 1
        assert driver._timer is not None and driver._timer.isActive()

        # STOP_RESEND_COUNT - 1 further timer ticks each resend STOP.
        for i in range(STOP_RESEND_COUNT - 1):
            driver._on_timer_tick()
            stops_so_far = [s for s in transport.sent if s == "STOP"]
            assert len(stops_so_far) == 2 + i, (
                f"Expected {2 + i} STOP sends after tick {i + 1}, "
                f"got {len(stops_so_far)}"
            )

        # Total STOP sends == STOP_RESEND_COUNT; timer now stopped.
        total_stops = [s for s in transport.sent if s == "STOP"]
        assert len(total_stops) == STOP_RESEND_COUNT
        assert driver._timer is None or not driver._timer.isActive()

        # One more tick (simulating a straggling timer fire) must not send
        # another STOP -- the sequence is over.
        driver._on_timer_tick()
        total_stops_after_extra_tick = [s for s in transport.sent if s == "STOP"]
        assert len(total_stops_after_extra_tick) == STOP_RESEND_COUNT
        driver.detach()

    def test_dropped_first_stop_still_recovers_via_deadman_resend(
        self, fake_window, qapp
    ):
        """Simulate a dropped first STOP (fake transport raises once) --
        a subsequent deadman resend still gets through, and the timer still
        stops on schedule (the countdown advances even on a failed send)."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver, STOP_RESEND_COUNT

        class DropFirstStopTransport(FakeTransport):
            """Raises on the first STOP send only; behaves normally after."""

            def __init__(self) -> None:
                super().__init__()
                self._stop_sends_seen = 0

            def send(self, line: str) -> None:
                if line == "STOP":
                    self._stop_sends_seen += 1
                    if self._stop_sends_seen == 1:
                        raise RuntimeError("simulated dropped STOP line")
                super().send(line)

        transport = DropFirstStopTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        release = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        fake_window.keyReleaseEvent(release)  # first STOP send "dropped"

        # The dropped send must not have been recorded, but the deadman
        # sequence must still be running (timer active).
        assert "STOP" not in transport.sent
        assert driver._timer is not None and driver._timer.isActive()

        # Drive the remaining deadman ticks -- a subsequent resend must get
        # through despite the first one failing.
        for _ in range(STOP_RESEND_COUNT - 1):
            driver._on_timer_tick()

        assert "STOP" in transport.sent, (
            "A later deadman resend must still deliver STOP after the first "
            "send was dropped"
        )
        # The countdown must have advanced on the dropped send too, so the
        # timer still stops on schedule (not one tick late).
        assert driver._timer is None or not driver._timer.isActive()
        driver.detach()

    def test_focus_out_triggers_deadman_when_key_held(self, fake_window, qapp):
        """Losing window focus while an arrow key is held triggers the same
        bounded STOP deadman-resend sequence a real key-release would."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver, STOP_RESEND_COUNT

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)
        # Avoid forwarding a MagicMock focus event to the real C++ handler.
        driver._orig_focus_out = lambda e: None

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        assert "VW 200 0" in transport.sent
        assert "STOP" not in transport.sent

        focus_event = MagicMock()
        driver._on_focus_out(focus_event)

        # Focus loss must have started the deadman sequence exactly like a
        # real key release: one immediate STOP send, timer still active.
        assert "STOP" in transport.sent
        assert driver._timer is not None and driver._timer.isActive()

        for _ in range(STOP_RESEND_COUNT - 1):
            driver._on_timer_tick()

        total_stops = [s for s in transport.sent if s == "STOP"]
        assert len(total_stops) == STOP_RESEND_COUNT
        assert driver._timer is None or not driver._timer.isActive()
        driver.detach()

    def test_focus_out_does_nothing_when_no_key_held(self, fake_window, qapp):
        """Focus loss while idle (no key held, no deadman in progress) must
        not send a spurious STOP."""
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)
        driver._orig_focus_out = lambda e: None

        focus_event = MagicMock()
        driver._on_focus_out(focus_event)

        assert transport.sent == []
        driver.detach()

    def test_focus_out_forwards_to_original_handler(self, fake_window, qapp):
        """The original focusOutEvent handler must still be invoked (no
        double-handling, no silently dropped Qt behavior)."""
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        calls = []
        driver._orig_focus_out = lambda e: calls.append(e)

        focus_event = MagicMock()
        driver._on_focus_out(focus_event)

        assert calls == [focus_event]
        driver.detach()


class TestKeyboardDriverMultiKeyRelease:
    """CR-15 item 8: releasing one held arrow key while another is still
    held falls back to driving the remaining key instead of starting the
    STOP deadman sequence; the deadman still fires when the LAST held key
    is released."""

    def test_release_one_of_two_held_keys_continues_driving(self, fake_window, qapp):
        """Up + Left both held; releasing Up (Left still held) must NOT
        send STOP -- it must continue driving with Left's command."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        up_press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        left_press = _make_key_event(Qt.Key.Key_Left, is_auto_repeat=False)
        up_release = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)

        fake_window.keyPressEvent(up_press)
        fake_window.keyPressEvent(left_press)
        fake_window.keyReleaseEvent(up_release)

        assert "STOP" not in transport.sent, (
            f"Releasing one of two held keys must not send STOP: {transport.sent}"
        )
        assert transport.sent[-1] == "VW 0 500", (
            f"Expected fallback to Left's command after releasing Up: {transport.sent}"
        )
        assert driver._timer is not None and driver._timer.isActive()
        driver.detach()

    def test_release_last_held_key_starts_deadman(self, fake_window, qapp):
        """After the fallback key is also released (no keys left held), the
        bounded STOP deadman sequence must fire as usual."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver, STOP_RESEND_COUNT

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        up_press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        left_press = _make_key_event(Qt.Key.Key_Left, is_auto_repeat=False)
        up_release = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        left_release = _make_key_event(Qt.Key.Key_Left, is_auto_repeat=False)

        fake_window.keyPressEvent(up_press)
        fake_window.keyPressEvent(left_press)
        fake_window.keyReleaseEvent(up_release)
        assert "STOP" not in transport.sent

        fake_window.keyReleaseEvent(left_release)
        assert "STOP" in transport.sent, (
            f"Releasing the last held key must start the STOP deadman: {transport.sent}"
        )

        for _ in range(STOP_RESEND_COUNT - 1):
            driver._on_timer_tick()

        total_stops = [s for s in transport.sent if s == "STOP"]
        assert len(total_stops) == STOP_RESEND_COUNT
        assert driver._timer is None or not driver._timer.isActive()
        driver.detach()

    def test_held_keys_tracked_across_press_and_release(self, fake_window, qapp):
        """self._held_keys reflects exactly the keys currently held."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        up_key = int(Qt.Key.Key_Up)
        left_key = int(Qt.Key.Key_Left)

        fake_window.keyPressEvent(_make_key_event(Qt.Key.Key_Up, is_auto_repeat=False))
        assert driver._held_keys == {up_key}

        fake_window.keyPressEvent(_make_key_event(Qt.Key.Key_Left, is_auto_repeat=False))
        assert driver._held_keys == {up_key, left_key}

        fake_window.keyReleaseEvent(_make_key_event(Qt.Key.Key_Up, is_auto_repeat=False))
        assert driver._held_keys == {left_key}

        fake_window.keyReleaseEvent(_make_key_event(Qt.Key.Key_Left, is_auto_repeat=False))
        assert driver._held_keys == set()
        driver.detach()

    def test_focus_out_clears_held_keys(self, fake_window, qapp):
        """Focus loss must clear all tracked held keys, not just stop driving."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver, STOP_RESEND_COUNT

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)
        driver._orig_focus_out = lambda e: None

        fake_window.keyPressEvent(_make_key_event(Qt.Key.Key_Up, is_auto_repeat=False))
        fake_window.keyPressEvent(_make_key_event(Qt.Key.Key_Left, is_auto_repeat=False))
        assert driver._held_keys

        driver._on_focus_out(MagicMock())
        assert driver._held_keys == set()

        for _ in range(STOP_RESEND_COUNT - 1):
            driver._on_timer_tick()
        driver.detach()

    def test_release_non_held_key_falls_back_to_still_held_key(
        self, fake_window, qapp
    ):
        """A stray release of a key that was never pressed (e.g. Right, when
        only Up is held) must not send STOP -- Up is still logically held,
        so the fallback command keeps driving it."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        fake_window.keyPressEvent(_make_key_event(Qt.Key.Key_Up, is_auto_repeat=False))
        fake_window.keyReleaseEvent(
            _make_key_event(Qt.Key.Key_Right, is_auto_repeat=False)
        )

        assert "STOP" not in transport.sent
        driver.detach()


class TestKeyboardDriverKeepaliveArmDisarm:
    """Keepalive arm/disarm bracket a drive session (sprint 065, ticket 005).

    ``SerialConnection.connect()`` no longer arms the ambient '+' keepalive
    daemon automatically -- ``KeyboardDriver`` (the layer that owns
    open-ended VW motion sessions) now calls
    ``transport.arm_keepalive()`` on the first key press of a session and
    ``transport.disarm_keepalive()`` once the bounded STOP deadman sequence
    completes (or on ``detach()``, as a safety net).
    """

    def test_press_release_deadman_complete_brackets_correctly(
        self, fake_window, qapp
    ):
        """arm on first press; NOT disarmed merely on release (STOP still
        needs bounded resending); disarmed exactly once the deadman
        sequence completes."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver, STOP_RESEND_COUNT

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        release = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)

        fake_window.keyPressEvent(press)
        assert transport.keepalive_events == ["arm"], (
            "Keepalive must be armed on the first key press of a drive session"
        )

        fake_window.keyReleaseEvent(release)
        assert transport.keepalive_events == ["arm"], (
            "Keepalive must not be disarmed on release itself -- STOP still "
            "needs its bounded deadman resend"
        )

        for _ in range(STOP_RESEND_COUNT - 1):
            driver._on_timer_tick()

        assert transport.keepalive_events == ["arm", "disarm"], (
            "Keepalive must be disarmed exactly once, only after the "
            "deadman sequence completes"
        )

        # detach() after a clean disarm must not double-disarm.
        driver.detach()
        assert transport.keepalive_events == ["arm", "disarm"]

    def test_held_key_direction_change_does_not_rearm(self, fake_window, qapp):
        """Switching directions mid-drive (still holding a key, never
        released) must not call arm_keepalive() a second time -- the
        session is still active."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        up = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        down = _make_key_event(Qt.Key.Key_Down, is_auto_repeat=False)
        fake_window.keyPressEvent(up)
        fake_window.keyPressEvent(down)

        assert transport.keepalive_events == ["arm"], (
            f"arm_keepalive() must only be called once per session: "
            f"{transport.keepalive_events}"
        )
        driver.detach()

    def test_focus_out_deadman_disarms_like_release(self, fake_window, qapp):
        """Focus-loss triggers the same deadman sequence as a real release,
        including the disarm once it completes."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver, STOP_RESEND_COUNT

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)
        driver._orig_focus_out = lambda e: None

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        assert transport.keepalive_events == ["arm"]

        focus_event = MagicMock()
        driver._on_focus_out(focus_event)
        assert transport.keepalive_events == ["arm"]

        for _ in range(STOP_RESEND_COUNT - 1):
            driver._on_timer_tick()

        assert transport.keepalive_events == ["arm", "disarm"]
        driver.detach()

    def test_detach_mid_drive_disarms_as_safety_net(self, fake_window, qapp):
        """detach() (e.g. on transport.disconnect()) mid-drive, before the
        deadman sequence would otherwise complete, must still disarm."""
        from PySide6.QtCore import Qt
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        press = _make_key_event(Qt.Key.Key_Up, is_auto_repeat=False)
        fake_window.keyPressEvent(press)
        assert transport.keepalive_events == ["arm"]

        driver.detach()
        assert transport.keepalive_events == ["arm", "disarm"], (
            "detach() mid-drive must disarm the keepalive as a safety net"
        )

    def test_no_key_press_never_arms(self, fake_window, qapp):
        """Attaching (without pressing any key) must never touch the
        keepalive -- arming is tied strictly to an actual drive session."""
        from robot_radio.testgui.drive import KeyboardDriver

        transport = FakeTransport()
        driver = KeyboardDriver()
        driver.attach(fake_window, transport)

        assert transport.keepalive_events == []
        driver.detach()
        assert transport.keepalive_events == []


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
