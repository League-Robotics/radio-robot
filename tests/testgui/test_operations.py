"""tests/testgui/test_operations.py — Headless tests for operations.py.

Tests cover:
- Qt-free pure helpers: build_setpose_command, is_sim_transport.
- OpsController handler behavior with a fake transport.
- Button enable/disable state via set_connected().
- STREAM toggle text and command.
- Clear Traces and Refresh Playfield hook wiring.
- Graceful degradation when daemon not available.

Run with:
    QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui -q

Requirements: PySide6 (uv sync --group gui).
"""

from __future__ import annotations

import math
from typing import Any
from unittest.mock import MagicMock, patch

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class FakeTransport:
    """Minimal fake transport for operations tests."""

    def __init__(self, name: str = "FakeTransport") -> None:
        self._name = name
        self.sent_commands: list[str] = []
        self.sent_fire_forget: list[str] = []
        self.command_reply: str = "OK"

    def command(self, line: str, read_ms: int = 200) -> str:
        self.sent_commands.append(line)
        return self.command_reply

    def send(self, line: str) -> None:
        self.sent_fire_forget.append(line)


class FakeSimTransport(FakeTransport):
    """Fake transport whose class name is 'SimTransport'."""

    # Override __class__.__name__ so is_sim_transport() recognises it.
    pass


# Rename the class so isinstance checks on the name pass.
FakeSimTransport.__name__ = "SimTransport"
FakeSimTransport.__qualname__ = "SimTransport"


# ---------------------------------------------------------------------------
# Qt-free pure-helper tests
# ---------------------------------------------------------------------------


class TestBuildSetposeCommand:
    """build_setpose_command — pure SI wire string builder."""

    def test_basic_conversion(self):
        from robot_radio.testgui.operations import build_setpose_command

        line = build_setpose_command(x_cm=10.0, y_cm=20.0, yaw_rad=0.0)
        assert line == "SI 100 200 0", f"Unexpected: {line!r}"

    def test_heading_east(self):
        """East-facing robot: yaw_rad=0 → h_cdeg=0."""
        from robot_radio.testgui.operations import build_setpose_command

        line = build_setpose_command(0.0, 0.0, 0.0)
        assert line == "SI 0 0 0"

    def test_heading_north_90deg(self):
        """North-facing robot: yaw_rad=pi/2 → h_cdeg=9000."""
        from robot_radio.testgui.operations import build_setpose_command

        line = build_setpose_command(0.0, 0.0, math.pi / 2)
        # round(90.0 * 100) = 9000
        assert line == "SI 0 0 9000"

    def test_negative_coordinates(self):
        from robot_radio.testgui.operations import build_setpose_command

        line = build_setpose_command(-5.0, -10.0, math.pi)
        # x_mm = round(-5 * 10) = -50; y_mm = round(-10 * 10) = -100
        # h_cdeg = round(180 * 100) = 18000
        assert line == "SI -50 -100 18000"

    def test_fractional_cm_rounds(self):
        from robot_radio.testgui.operations import build_setpose_command

        line = build_setpose_command(1.5, 2.5, 0.0)
        # 1.5 cm * 10 = 15 mm; 2.5 cm * 10 = 25 mm
        assert line == "SI 15 25 0"

    def test_returns_string(self):
        from robot_radio.testgui.operations import build_setpose_command

        result = build_setpose_command(0.0, 0.0, 0.0)
        assert isinstance(result, str)


class TestIsSimTransport:
    """is_sim_transport — duck-type check on class name."""

    def test_fake_transport_is_not_sim(self):
        from robot_radio.testgui.operations import is_sim_transport

        t = FakeTransport()
        assert not is_sim_transport(t)

    def test_sim_named_transport_is_sim(self):
        from robot_radio.testgui.operations import is_sim_transport

        t = FakeSimTransport()
        assert is_sim_transport(t)

    def test_none_is_not_sim(self):
        from robot_radio.testgui.operations import is_sim_transport

        assert not is_sim_transport(None)  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# OpsController handler tests (headless — no QApplication required)
# ---------------------------------------------------------------------------


def _make_controller(
    transport: FakeTransport | None = None,
    clear_cb=None,
    refresh_cb=None,    # signature: (pixmap, origin_x, origin_y) -> None
    set_origin_cb=None,
) -> tuple["object", list[str], dict]:
    """Build an OpsController with fake widgets and optional transport."""
    from robot_radio.testgui.operations import OpsController

    log_entries: list[str] = []
    state = {"transport": transport}

    # Fake buttons (plain objects with setEnabled / setText / setChecked).
    class FakeBtn:
        def __init__(self, checked: bool = False, text: str = "") -> None:
            self.enabled = True
            self.checked = checked
            self.text_val = text
            self.tooltip = ""

        def setEnabled(self, v: bool) -> None:
            self.enabled = v

        def setToolTip(self, t: str) -> None:
            self.tooltip = t

        def setText(self, t: str) -> None:
            self.text_val = t

        def setChecked(self, v: bool) -> None:
            self.checked = v

    sync_btn = FakeBtn()
    zero_btn = FakeBtn()
    stop_btn = FakeBtn()
    clear_btn = FakeBtn()
    refresh_btn = FakeBtn()
    stream_btn = FakeBtn(text="STREAM: off")
    origin_btn = FakeBtn()

    ctrl = OpsController(
        transport_ref=state,
        log_cb=log_entries.append,
        sync_btn=sync_btn,
        zero_btn=zero_btn,
        stop_btn=stop_btn,
        clear_btn=clear_btn,
        refresh_btn=refresh_btn,
        stream_btn=stream_btn,
        origin_btn=origin_btn,
        # refresh_btn is NOT in transport_buttons — the camera is independent of
        # the robot transport, so Refresh Playfield is always enabled.
        transport_buttons=[sync_btn, zero_btn, stop_btn, stream_btn],
        clear_traces_cb=clear_cb,
        refresh_playfield_cb=refresh_cb,
        set_origin_cb=set_origin_cb,
    )
    return ctrl, log_entries, state


# --- Zero Encoders ---

class TestZeroEncoders:
    def test_sends_zero_enc(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)
        ctrl.on_zero_encoders()
        assert "ZERO enc" in t.sent_commands

    def test_logs_reply(self):
        t = FakeTransport()
        t.command_reply = "OK zero"
        ctrl, log, state = _make_controller(t)
        ctrl.on_zero_encoders()
        assert any("zero" in e.lower() or "Zero" in e for e in log)

    def test_no_transport_logs_warn(self):
        ctrl, log, state = _make_controller(None)
        ctrl.on_zero_encoders()
        assert any("WARN" in e or "not connected" in e for e in log)

    def test_no_transport_does_not_raise(self):
        ctrl, log, state = _make_controller(None)
        ctrl.on_zero_encoders()  # Must not raise


# --- STOP ---

class TestStop:
    def test_sends_stop(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)
        ctrl.on_stop()
        assert "STOP" in t.sent_fire_forget

    def test_logs_stop(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)
        ctrl.on_stop()
        assert any("STOP" in e for e in log)

    def test_no_transport_logs_warn(self):
        ctrl, log, state = _make_controller(None)
        ctrl.on_stop()
        assert any("WARN" in e or "not connected" in e for e in log)

    def test_no_transport_does_not_raise(self):
        ctrl, log, state = _make_controller(None)
        ctrl.on_stop()


# --- Clear Traces ---

class TestClearTraces:
    def test_calls_clear_cb(self):
        called = []
        ctrl, log, state = _make_controller(None, clear_cb=lambda: called.append(True))
        ctrl.on_clear_traces()
        assert called == [True]

    def test_no_cb_does_not_raise(self):
        ctrl, log, state = _make_controller(None, clear_cb=None)
        ctrl.on_clear_traces()  # Must not raise

    def test_logs_done(self):
        ctrl, log, state = _make_controller(None)
        ctrl.on_clear_traces()
        assert any("Clear Traces" in e or "done" in e.lower() for e in log)

    def test_works_without_transport(self):
        """Clear Traces must work even with no transport connected."""
        called = []
        ctrl, log, state = _make_controller(None, clear_cb=lambda: called.append(1))
        ctrl.on_clear_traces()
        assert called  # callback was invoked

    def test_cb_exception_logged_not_raised(self):
        def bad_cb():
            raise RuntimeError("test error")

        ctrl, log, state = _make_controller(None, clear_cb=bad_cb)
        ctrl.on_clear_traces()  # Must not raise
        assert any("ERROR" in e or "callback" in e.lower() for e in log)


# --- STREAM toggle ---

class TestStreamToggle:
    def test_toggle_on_sends_stream_50(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)
        ctrl.on_stream_toggled(True)
        assert any("STREAM 50" in c for c in t.sent_commands)

    def test_toggle_off_sends_stream_0(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)
        ctrl.on_stream_toggled(True)
        ctrl.on_stream_toggled(False)
        assert any("STREAM 0" in c for c in t.sent_commands)

    def test_toggle_on_updates_label(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)

        # Access the fake stream button to check its text.
        from robot_radio.testgui.operations import OpsController
        # The stream_btn is stored as ctrl._stream_btn
        ctrl.on_stream_toggled(True)
        assert ctrl._stream_btn.text_val == "STREAM: on"  # type: ignore[attr-defined]

    def test_toggle_off_updates_label(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)
        ctrl.on_stream_toggled(True)
        ctrl.on_stream_toggled(False)
        assert ctrl._stream_btn.text_val == "STREAM: off"  # type: ignore[attr-defined]

    def test_no_transport_logs_warn(self):
        ctrl, log, state = _make_controller(None)
        ctrl.on_stream_toggled(True)
        assert any("WARN" in e or "not connected" in e for e in log)


# --- set_connected ---

class TestSetConnected:
    def test_connected_enables_transport_buttons(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)

        # Force buttons to disabled first.
        for btn in ctrl._transport_buttons:
            btn.setEnabled(False)

        ctrl.set_connected(True, t)

        for btn in ctrl._transport_buttons:
            assert btn.enabled  # type: ignore[attr-defined]

    def test_disconnected_disables_transport_buttons(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)

        ctrl.set_connected(True, t)
        ctrl.set_connected(False)

        for btn in ctrl._transport_buttons:
            assert not btn.enabled  # type: ignore[attr-defined]

    def test_sim_mode_disables_sync_pose(self):
        t = FakeSimTransport()
        ctrl, log, state = _make_controller(t)

        ctrl.set_connected(True, t)

        # Sync pose button must be disabled in sim mode.
        assert not ctrl._sync_btn.enabled  # type: ignore[attr-defined]

    def test_hardware_mode_enables_sync_pose(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)

        ctrl.set_connected(True, t)

        # Sync pose must be enabled for hardware transport.
        assert ctrl._sync_btn.enabled  # type: ignore[attr-defined]

    def test_disconnect_resets_stream_toggle(self):
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)

        ctrl.set_connected(True, t)
        ctrl._stream_btn.setChecked(True)  # type: ignore[attr-defined]
        ctrl.set_connected(False)

        assert not ctrl._stream_btn.checked  # type: ignore[attr-defined]


# --- Sync Pose (daemon unavailable) ---

class TestSyncPoseDaemonUnavailable:
    def test_import_error_logs_warn_not_crash(self):
        """If aprilcam is not importable, logs a warning; does not crash."""
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)

        # Patch _read_daemon_pose to raise RuntimeError (daemon not available).
        def _fail():
            raise RuntimeError("aprilcam not installed")

        ctrl._read_daemon_pose = _fail
        ctrl.on_sync_pose()

        assert any("WARN" in e or "daemon" in e.lower() for e in log)
        # No command must have been sent.
        assert not t.sent_commands

    def test_tag_not_seen_logs_warn(self):
        """If tag 100 not seen (daemon returns None), logs warning."""
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)

        ctrl._read_daemon_pose = lambda: None
        ctrl.on_sync_pose()

        assert any(
            "not seen" in e.lower() or "WARN" in e or "tag 100" in e.lower()
            for e in log
        )
        assert not t.sent_commands

    def test_pose_sends_si_command(self):
        """When pose is available, sends SI wire string."""
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)

        # Inject a known pose.
        ctrl._read_daemon_pose = lambda: (10.0, 20.0, 0.0)
        ctrl.on_sync_pose()

        assert any("SI 100 200 0" in c for c in t.sent_commands)

    def test_pose_logs_values(self):
        """Log entry includes the pose values and the SI command."""
        t = FakeTransport()
        ctrl, log, state = _make_controller(t)

        ctrl._read_daemon_pose = lambda: (10.0, 20.0, math.pi / 2)
        ctrl.on_sync_pose()

        # Should log something containing the SI command.
        assert any("SI" in e for e in log)

    def test_no_transport_logs_warn(self):
        ctrl, log, state = _make_controller(None)
        ctrl.on_sync_pose()
        assert any("WARN" in e or "not connected" in e for e in log)


# --- Refresh Playfield (daemon unavailable) ---

class TestRefreshPlayfieldDaemonUnavailable:
    def test_no_transport_does_not_warn_not_connected(self):
        """on_refresh_playfield works without a transport — camera is independent."""
        ctrl, log, state = _make_controller(None)
        # Stub out the actual daemon call so the test is hermetic.
        ctrl._capture_playfield_frame_and_calib = lambda: None
        ctrl.on_refresh_playfield()
        # Must NOT log a "not connected" warning — refresh does not need transport.
        assert not any("not connected" in e.lower() for e in log), (
            f"Refresh Playfield must not warn 'not connected'; got: {log}"
        )

    def test_no_transport_attempts_capture(self):
        """on_refresh_playfield attempts capture even without a transport."""
        ctrl, log, state = _make_controller(None)
        called = []
        ctrl._capture_playfield_frame_and_calib = lambda: called.append(True) or None
        ctrl.on_refresh_playfield()
        assert called, "Capture must be attempted even without a robot transport"

    def test_capture_failure_logs_warn_not_crash(self):
        ctrl, log, state = _make_controller(None)

        def _fail():
            raise RuntimeError("daemon not available")

        # _capture_playfield_frame_and_calib replaced _capture_playfield_pixmap.
        ctrl._capture_playfield_frame_and_calib = _fail
        ctrl.on_refresh_playfield()

        assert any("WARN" in e or "capture" in e.lower() for e in log)

    def test_none_pixmap_logs_warn(self):
        ctrl, log, state = _make_controller(None)

        # Returning None signals no image from daemon.
        ctrl._capture_playfield_frame_and_calib = lambda: None
        ctrl.on_refresh_playfield()

        assert any("WARN" in e or "no image" in e.lower() for e in log)

    def test_pixmap_calls_refresh_cb(self):
        """refresh_playfield_cb is called with (pixmap, origin_x, origin_y)."""
        received: list = []
        ctrl, log, state = _make_controller(None, refresh_cb=lambda px, ox, oy: received.append((px, ox, oy)))

        fake_pixmap = object()
        # _capture_playfield_frame_and_calib returns (pixmap, origin_x, origin_y).
        ctrl._capture_playfield_frame_and_calib = lambda: (fake_pixmap, 12.5, 34.0)
        ctrl.on_refresh_playfield()

        assert len(received) == 1
        assert received[0][0] is fake_pixmap
        assert received[0][1] == pytest.approx(12.5)
        assert received[0][2] == pytest.approx(34.0)

    def test_pixmap_calls_refresh_cb_without_transport(self):
        """refresh_playfield_cb is called even without a robot transport."""
        received: list = []
        ctrl, log, state = _make_controller(None, refresh_cb=lambda px, ox, oy: received.append((px, ox, oy)))

        fake_pixmap = object()
        ctrl._capture_playfield_frame_and_calib = lambda: (fake_pixmap, 5.0, 8.0)
        ctrl.on_refresh_playfield()

        assert len(received) == 1, "Refresh callback must fire without transport"

    def test_refresh_cb_exception_logged(self):
        def bad_cb(px, ox, oy):
            raise ValueError("canvas error")

        ctrl, log, state = _make_controller(None, refresh_cb=bad_cb)
        ctrl._capture_playfield_frame_and_calib = lambda: (object(), 0.0, 0.0)
        ctrl.on_refresh_playfield()

        assert any("ERROR" in e or "callback" in e.lower() for e in log)


# ---------------------------------------------------------------------------
# trigger_live_grab — background-thread auto-grab
# ---------------------------------------------------------------------------


class TestTriggerLiveGrab:
    """trigger_live_grab fires capture on a background thread and marshals result
    back to the Qt main thread via a queued signal.
    """

    def _build_qapp(self):
        from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]
        import sys
        return QApplication.instance() or QApplication(sys.argv)

    def _pump_events(self, n: int = 5) -> None:
        """Process Qt events so queued signals are delivered."""
        from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]
        for _ in range(n):
            QApplication.processEvents()

    def test_trigger_calls_capture_on_background_thread(self):
        """Capture runs on a non-main thread."""
        import threading
        self._build_qapp()

        call_threads: list = []

        def _fake_capture():
            call_threads.append(threading.current_thread().name)
            return None

        ctrl, log, state = _make_controller(None)
        ctrl._capture_playfield_frame_and_calib = _fake_capture
        ctrl.trigger_live_grab()

        # Give the thread time to complete.
        import time
        time.sleep(0.2)
        self._pump_events(10)

        assert call_threads, "Capture was never called"
        main_thread = threading.main_thread().name
        assert call_threads[0] != main_thread, (
            f"Capture must run on a background thread, not {main_thread!r}"
        )

    def test_trigger_success_invokes_refresh_cb(self):
        """On success, refresh_playfield_cb is called (via queued signal)."""
        import time
        self._build_qapp()

        received: list = []

        def _fake_capture():
            return (object(), 10.0, 20.0)

        ctrl, log, state = _make_controller(
            None,
            refresh_cb=lambda px, ox, oy: received.append((px, ox, oy)),
        )
        ctrl._capture_playfield_frame_and_calib = _fake_capture
        ctrl.trigger_live_grab()

        # Wait for thread + signal delivery.
        time.sleep(0.2)
        self._pump_events(10)

        assert len(received) == 1, (
            f"refresh_playfield_cb should have been called once; calls={received}"
        )
        _, ox, oy = received[0]
        assert ox == pytest.approx(10.0)
        assert oy == pytest.approx(20.0)

    def test_trigger_daemon_absent_logs_placeholder_message(self):
        """When capture fails (daemon absent), logs a clear placeholder message."""
        import time
        self._build_qapp()

        def _fail():
            raise RuntimeError("aprilcam not available")

        ctrl, log, state = _make_controller(None)
        ctrl._capture_playfield_frame_and_calib = _fail
        ctrl.trigger_live_grab()

        time.sleep(0.2)
        self._pump_events(10)

        # Must log a message about no camera/placeholder — not crash.
        assert any(
            "aprilcam" in e.lower() or "placeholder" in e.lower() or "camera" in e.lower()
            for e in log
        ), f"Expected placeholder message in log; got: {log}"

    def test_trigger_daemon_absent_no_stale_image(self):
        """When capture fails, refresh_playfield_cb is NOT called (no stale image shown)."""
        import time
        self._build_qapp()

        received: list = []

        def _fail():
            raise RuntimeError("aprilcam not available")

        ctrl, log, state = _make_controller(
            None,
            refresh_cb=lambda px, ox, oy: received.append((px, ox, oy)),
        )
        ctrl._capture_playfield_frame_and_calib = _fail
        ctrl.trigger_live_grab()

        time.sleep(0.2)
        self._pump_events(10)

        assert not received, (
            "refresh_playfield_cb must NOT be called when capture fails "
            "(no stale image should be shown)"
        )

    def test_trigger_none_result_logs_placeholder_message(self):
        """When capture returns None, logs the placeholder message."""
        import time
        self._build_qapp()

        ctrl, log, state = _make_controller(None)
        ctrl._capture_playfield_frame_and_calib = lambda: None
        ctrl.trigger_live_grab()

        time.sleep(0.2)
        self._pump_events(10)

        assert any(
            "placeholder" in e.lower() or "camera" in e.lower() or "aprilcam" in e.lower()
            for e in log
        ), f"Expected placeholder message when capture returns None; got: {log}"

    def test_trigger_no_crash_when_no_refresh_cb(self):
        """trigger_live_grab must not crash when refresh_playfield_cb is None."""
        import time
        self._build_qapp()

        ctrl, log, state = _make_controller(None, refresh_cb=None)
        ctrl._capture_playfield_frame_and_calib = lambda: (object(), 5.0, 5.0)
        ctrl.trigger_live_grab()  # must not raise

        time.sleep(0.2)
        self._pump_events(10)
        # No assertion needed — just must not crash.


# ---------------------------------------------------------------------------
# Importability without PySide6
# ---------------------------------------------------------------------------


class TestOperationsImportability:
    """operations.py module-level code must be importable without PySide6."""

    def test_module_importable(self):
        import robot_radio.testgui.operations as ops
        assert hasattr(ops, "build_setpose_command")
        assert hasattr(ops, "is_sim_transport")
        assert hasattr(ops, "build_panel")
        assert hasattr(ops, "OpsController")

    def test_sync_pose_module_importable(self):
        import robot_radio.robot.sync_pose as sp
        assert hasattr(sp, "daemon_read_pose")
        assert hasattr(sp, "pose_to_setpose_line")


# ---------------------------------------------------------------------------
# build_panel smoke test (requires QApplication)
# ---------------------------------------------------------------------------


class TestBuildPanel:
    """build_panel returns a QGroupBox and an OpsController."""

    def test_build_panel_returns_panel_and_controller(self):
        from PySide6.QtWidgets import QApplication, QGroupBox  # type: ignore[import-untyped]
        import sys

        app = QApplication.instance() or QApplication(sys.argv)
        from robot_radio.testgui.operations import build_panel, OpsController

        state = {"transport": None}
        log_entries: list[str] = []
        panel, ctrl = build_panel(
            log_cb=log_entries.append,
            transport_ref=state,
        )

        assert isinstance(panel, QGroupBox)
        assert isinstance(ctrl, OpsController)

    def test_ops_panel_has_seven_buttons(self):
        from PySide6.QtWidgets import QApplication, QPushButton  # type: ignore[import-untyped]
        import sys

        app = QApplication.instance() or QApplication(sys.argv)
        from robot_radio.testgui.operations import build_panel

        state = {"transport": None}
        panel, ctrl = build_panel(log_cb=lambda _: None, transport_ref=state)

        buttons = panel.findChildren(QPushButton)
        assert len(buttons) == 7, (
            f"Expected 7 buttons in ops panel, found {len(buttons)}: "
            f"{[b.objectName() for b in buttons]}"
        )

    def test_ops_btn_object_names(self):
        from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]
        import sys

        app = QApplication.instance() or QApplication(sys.argv)
        from robot_radio.testgui.operations import build_panel

        state = {"transport": None}
        panel, ctrl = build_panel(log_cb=lambda _: None, transport_ref=state)

        expected_names = {
            "ops_btn_sync_pose",
            "ops_btn_zero_encoders",
            "ops_btn_stop",
            "ops_btn_clear_traces",
            "ops_btn_refresh_playfield",
            "ops_btn_stream",
            "ops_btn_set_origin",
        }
        from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]
        actual_names = {b.objectName() for b in panel.findChildren(QPushButton)}
        assert actual_names == expected_names, (
            f"Button names mismatch.\nExpected: {expected_names}\nGot: {actual_names}"
        )

    def test_transport_buttons_disabled_initially(self):
        from PySide6.QtWidgets import QApplication, QPushButton  # type: ignore[import-untyped]
        import sys

        app = QApplication.instance() or QApplication(sys.argv)
        from robot_radio.testgui.operations import build_panel

        state = {"transport": None}
        panel, ctrl = build_panel(log_cb=lambda _: None, transport_ref=state)

        # Transport-dependent buttons start disabled.
        # NOTE: refresh_playfield is NOT in this list — the camera is independent
        # of the robot transport and should always be enabled.
        disabled_names = {
            "ops_btn_sync_pose",
            "ops_btn_zero_encoders",
            "ops_btn_stop",
            "ops_btn_stream",
        }
        for btn in panel.findChildren(QPushButton):
            if btn.objectName() in disabled_names:
                assert not btn.isEnabled(), (
                    f"Button {btn.objectName()!r} should be disabled initially"
                )

    def test_refresh_playfield_btn_enabled_initially(self):
        """Refresh Playfield must be enabled without transport (camera is independent)."""
        from PySide6.QtWidgets import QApplication, QPushButton  # type: ignore[import-untyped]
        import sys

        app = QApplication.instance() or QApplication(sys.argv)
        from robot_radio.testgui.operations import build_panel

        state = {"transport": None}
        panel, ctrl = build_panel(log_cb=lambda _: None, transport_ref=state)

        refresh_btn = panel.findChild(QPushButton, "ops_btn_refresh_playfield")
        assert refresh_btn is not None, "ops_btn_refresh_playfield not found"
        assert refresh_btn.isEnabled(), (
            "Refresh Playfield must be enabled without transport "
            "(camera is independent of robot connection)"
        )

    def test_clear_traces_btn_enabled_initially(self):
        from PySide6.QtWidgets import QApplication, QPushButton  # type: ignore[import-untyped]
        import sys

        app = QApplication.instance() or QApplication(sys.argv)
        from robot_radio.testgui.operations import build_panel

        state = {"transport": None}
        panel, ctrl = build_panel(log_cb=lambda _: None, transport_ref=state)

        from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]
        clear_btn = panel.findChild(QPushButton, "ops_btn_clear_traces")
        assert clear_btn is not None, "ops_btn_clear_traces not found"
        assert clear_btn.isEnabled(), "Clear Traces button should be enabled without transport"

    def test_stream_btn_is_checkable(self):
        from PySide6.QtWidgets import QApplication, QPushButton  # type: ignore[import-untyped]
        import sys

        app = QApplication.instance() or QApplication(sys.argv)
        from robot_radio.testgui.operations import build_panel

        state = {"transport": None}
        panel, ctrl = build_panel(log_cb=lambda _: None, transport_ref=state)

        stream_btn = panel.findChild(QPushButton, "ops_btn_stream")
        assert stream_btn is not None
        assert stream_btn.isCheckable()

    def test_set_origin_btn_enabled_initially(self):
        """'Set Robot @ 0,0' button must be enabled without a transport."""
        from PySide6.QtWidgets import QApplication, QPushButton  # type: ignore[import-untyped]
        import sys

        app = QApplication.instance() or QApplication(sys.argv)
        from robot_radio.testgui.operations import build_panel

        state = {"transport": None}
        panel, ctrl = build_panel(log_cb=lambda _: None, transport_ref=state)

        origin_btn = panel.findChild(QPushButton, "ops_btn_set_origin")
        assert origin_btn is not None, "ops_btn_set_origin not found"
        assert origin_btn.isEnabled(), "'Set Robot @ 0,0' should be enabled without transport"


# ---------------------------------------------------------------------------
# Set Robot @ 0,0 (on_set_origin) — display-only, no wire command
# ---------------------------------------------------------------------------


class TestSetOrigin:
    """on_set_origin calls set_origin_cb; never touches the transport."""

    def test_calls_set_origin_cb(self):
        called = []
        ctrl, log, state = _make_controller(None, set_origin_cb=lambda: called.append(True))
        ctrl.on_set_origin()
        assert called == [True], "set_origin_cb should have been called once"

    def test_no_cb_does_not_raise(self):
        ctrl, log, state = _make_controller(None, set_origin_cb=None)
        ctrl.on_set_origin()  # must not raise

    def test_logs_done(self):
        ctrl, log, state = _make_controller(None)
        ctrl.on_set_origin()
        assert any("0,0" in e or "anchor" in e.lower() or "centre" in e.lower() for e in log)

    def test_works_without_transport(self):
        """Set Robot @ 0,0 must work with no transport connected."""
        called = []
        ctrl, log, state = _make_controller(None, set_origin_cb=lambda: called.append(1))
        ctrl.on_set_origin()
        assert called  # callback was invoked

    def test_sends_no_wire_command(self):
        """on_set_origin must not call transport.command() or transport.send()."""
        t = FakeTransport()
        ctrl, log, state = _make_controller(t, set_origin_cb=lambda: None)
        ctrl.on_set_origin()
        assert not t.sent_commands, (
            f"Set Robot @ 0,0 must send no wire commands; got: {t.sent_commands}"
        )
        assert not t.sent_fire_forget, (
            f"Set Robot @ 0,0 must send no fire-and-forget commands; got: {t.sent_fire_forget}"
        )

    def test_sends_no_command_with_connected_transport(self):
        """Even when a transport is connected, on_set_origin sends nothing."""
        t = FakeTransport()
        ctrl, log, state = _make_controller(t, set_origin_cb=lambda: None)
        # Simulate connected state.
        ctrl.set_connected(True, t)
        ctrl.on_set_origin()
        assert not t.sent_commands
        assert not t.sent_fire_forget

    def test_cb_exception_logged_not_raised(self):
        def bad_cb():
            raise RuntimeError("re-anchor error")

        ctrl, log, state = _make_controller(None, set_origin_cb=bad_cb)
        ctrl.on_set_origin()  # must not raise
        assert any("ERROR" in e or "callback" in e.lower() for e in log)

    def test_set_origin_transport_button_remains_enabled_after_set_origin(self):
        """After set_origin, transport buttons should be unaffected."""
        t = FakeTransport()
        ctrl, log, state = _make_controller(t, set_origin_cb=lambda: None)
        ctrl.set_connected(True, t)
        before = [btn.enabled for btn in ctrl._transport_buttons]
        ctrl.on_set_origin()
        after = [btn.enabled for btn in ctrl._transport_buttons]
        assert before == after, "set_origin must not change transport button enable state"
