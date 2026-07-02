"""tests/testgui/test_sim_errors_panel.py — headless tests for the "Sim Errors" panel.

Covers (issue testgui-sim-error-profile-config):
- The ``sim_errors_group`` QGroupBox and its four spin boxes
  (``sim_err_encoder_mm``, ``sim_err_slip_turn``, ``sim_err_otos_linear``,
  ``sim_err_otos_yaw``) exist and are populated from
  ``sim_prefs.load_sim_error_profile()`` at window build.
- Visibility toggles with the transport combo: visible for "Sim", hidden
  for "Serial" / "Relay".
- Clicking ``sim_errors_apply_btn`` saves the current field values via
  ``sim_prefs.save_sim_error_profile`` (monkeypatched to capture the call
  without touching the real ``data/`` directory).

``_build_main_window()`` returns only ``(window, app)`` — its internal
``_state`` dict is not exposed as a test seam (see the precedent in
test_live_frame_bridge.py's module docstring). This suite therefore covers
panel existence / visibility / defaults / save-on-apply directly against
window widgets, per the ticket's documented fallback for cases where
reaching ``_state`` from outside is impractical.

Run with:
    QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/test_sim_errors_panel.py -q

Requirements: PySide6 (uv sync --group gui).
"""

from __future__ import annotations

import pytest


@pytest.fixture(scope="session")
def qapp():
    """Ensure a QApplication exists for the whole test session (see test_smoke.py)."""
    import sys
    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


# ---------------------------------------------------------------------------
# Panel existence + defaults
# ---------------------------------------------------------------------------


class TestSimErrorsPanelExistence:
    def test_group_and_spin_boxes_exist(self, qapp, monkeypatch, tmp_path):
        from robot_radio.testgui import sim_prefs

        # Isolate from the real data/testgui/sim_error_profile.json.
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")

        from PySide6.QtWidgets import QGroupBox, QDoubleSpinBox, QPushButton  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            group = window.findChild(QGroupBox, "sim_errors_group")
            assert group is not None, "sim_errors_group not found"

            for object_name in (
                "sim_err_encoder_mm",
                "sim_err_slip_turn",
                "sim_err_otos_linear",
                "sim_err_otos_yaw",
            ):
                spin = window.findChild(QDoubleSpinBox, object_name)
                assert spin is not None, f"{object_name} spin box not found"

            apply_btn = window.findChild(QPushButton, "sim_errors_apply_btn")
            assert apply_btn is not None, "sim_errors_apply_btn not found"
        finally:
            window.hide()

    def test_spin_boxes_populated_from_defaults(self, qapp, monkeypatch, tmp_path):
        """With no persisted file, fields must show sim_prefs.DEFAULT_PROFILE."""
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")

        from PySide6.QtWidgets import QDoubleSpinBox  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            encoder_spin = window.findChild(QDoubleSpinBox, "sim_err_encoder_mm")
            slip_spin = window.findChild(QDoubleSpinBox, "sim_err_slip_turn")
            linear_spin = window.findChild(QDoubleSpinBox, "sim_err_otos_linear")
            yaw_spin = window.findChild(QDoubleSpinBox, "sim_err_otos_yaw")

            assert encoder_spin.value() == pytest.approx(0.0)
            assert slip_spin.value() == pytest.approx(0.26)
            assert linear_spin.value() == pytest.approx(0.05)
            assert yaw_spin.value() == pytest.approx(0.0)
        finally:
            window.hide()

    def test_spin_boxes_populated_from_persisted_file(self, qapp, monkeypatch, tmp_path):
        """A persisted profile must populate the fields at window build."""
        from robot_radio.testgui import sim_prefs

        prefs_path = tmp_path / "sim_error_profile.json"
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)
        sim_prefs.save_sim_error_profile(
            {
                "encoder_noise_mm": 4.0,
                "slip_turn_extra": 0.5,
                "otos_linear_noise": 0.3,
                "otos_yaw_noise": 0.07,
            }
        )

        from PySide6.QtWidgets import QDoubleSpinBox  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            encoder_spin = window.findChild(QDoubleSpinBox, "sim_err_encoder_mm")
            slip_spin = window.findChild(QDoubleSpinBox, "sim_err_slip_turn")
            linear_spin = window.findChild(QDoubleSpinBox, "sim_err_otos_linear")
            yaw_spin = window.findChild(QDoubleSpinBox, "sim_err_otos_yaw")

            assert encoder_spin.value() == pytest.approx(4.0)
            assert slip_spin.value() == pytest.approx(0.5)
            assert linear_spin.value() == pytest.approx(0.3)
            assert yaw_spin.value() == pytest.approx(0.07)
        finally:
            window.hide()


# ---------------------------------------------------------------------------
# Visibility toggling with the transport combo
# ---------------------------------------------------------------------------


class TestSimErrorsPanelVisibility:
    def test_visible_when_sim_selected(self, qapp, monkeypatch, tmp_path):
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")

        from PySide6.QtWidgets import QGroupBox, QComboBox  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            transport_combo = window.findChild(QComboBox, "transport_combo")
            group = window.findChild(QGroupBox, "sim_errors_group")

            transport_combo.setCurrentText("Sim")
            assert not group.isHidden(), "sim_errors_group must be visible for Sim"
        finally:
            window.hide()

    def test_hidden_for_serial_and_relay(self, qapp, monkeypatch, tmp_path):
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")

        from PySide6.QtWidgets import QGroupBox, QComboBox  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            transport_combo = window.findChild(QComboBox, "transport_combo")
            group = window.findChild(QGroupBox, "sim_errors_group")

            transport_combo.setCurrentText("Serial")
            assert group.isHidden(), "sim_errors_group must hide for Serial"

            transport_combo.setCurrentText("Relay")
            assert group.isHidden(), "sim_errors_group must hide for Relay"

            transport_combo.setCurrentText("Sim")
            assert not group.isHidden(), "sim_errors_group must reappear for Sim"
        finally:
            window.hide()

    def test_starts_visible_because_sim_is_default(self, qapp, monkeypatch, tmp_path):
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")

        from PySide6.QtWidgets import QGroupBox, QComboBox  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            transport_combo = window.findChild(QComboBox, "transport_combo")
            group = window.findChild(QGroupBox, "sim_errors_group")

            assert transport_combo.currentText() == "Sim"
            assert not group.isHidden()
        finally:
            window.hide()


# ---------------------------------------------------------------------------
# Apply button: saves the field values
# ---------------------------------------------------------------------------


class TestSimErrorsApplyButton:
    def test_apply_saves_field_values(self, qapp, monkeypatch, tmp_path):
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")

        saved: list[dict] = []
        monkeypatch.setattr(
            sim_prefs, "save_sim_error_profile", lambda profile: saved.append(profile)
        )

        from PySide6.QtWidgets import QDoubleSpinBox, QPushButton  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            encoder_spin = window.findChild(QDoubleSpinBox, "sim_err_encoder_mm")
            slip_spin = window.findChild(QDoubleSpinBox, "sim_err_slip_turn")
            linear_spin = window.findChild(QDoubleSpinBox, "sim_err_otos_linear")
            yaw_spin = window.findChild(QDoubleSpinBox, "sim_err_otos_yaw")
            apply_btn = window.findChild(QPushButton, "sim_errors_apply_btn")

            encoder_spin.setValue(6.0)
            slip_spin.setValue(0.6)
            linear_spin.setValue(0.4)
            yaw_spin.setValue(0.08)

            apply_btn.click()

            assert len(saved) == 1
            assert saved[0] == {
                "encoder_noise_mm": 6.0,
                "slip_turn_extra": 0.6,
                "otos_linear_noise": 0.4,
                "otos_yaw_noise": 0.08,
            }
        finally:
            window.hide()

    def test_apply_calls_live_apply_on_connected_sim_transport(
        self, qapp, monkeypatch, tmp_path
    ):
        """With a fake connected SimTransport, Apply must call
        transport.apply_error_profile(profile) with the field values.

        Since _state is internal to _build_main_window() (no test seam),
        this test monkeypatches robot_radio.testgui.transport.SimTransport
        BEFORE building the window, so the module-scoped `from
        robot_radio.testgui.transport import (..., SimTransport, ...)` inside
        _build_main_window() (a fresh import executed on every call) binds
        to the fake class, then drives a real Connect via the Connect
        button so _state["transport"] becomes an instance of it.
        """
        from robot_radio.testgui import sim_prefs, operations
        import robot_radio.testgui.transport as transport_module

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")
        monkeypatch.setattr(
            sim_prefs, "save_sim_error_profile", lambda profile: None
        )

        applied: list[dict] = []

        class FakeConnectedSimTransport(transport_module.Transport):
            """Fake whose class name is 'SimTransport' (duck-typed by
            operations.is_sim_transport) and that never touches real hardware
            or the ctypes sim.
            """

            def __init__(self) -> None:
                super().__init__()
                self._connected = False

            def connect(self) -> None:
                self._connected = True

            def disconnect(self) -> None:
                self._connected = False

            def send(self, line: str) -> None:
                pass

            def command(self, line: str, read_ms: int = 200) -> str:
                return "OK"

            def apply_error_profile(self, profile: dict) -> None:
                applied.append(profile)

        FakeConnectedSimTransport.__name__ = "SimTransport"
        FakeConnectedSimTransport.__qualname__ = "SimTransport"

        # Sanity: operations.is_sim_transport must recognize it by class name.
        assert operations.is_sim_transport(FakeConnectedSimTransport())

        monkeypatch.setattr(transport_module, "SimTransport", FakeConnectedSimTransport)

        from PySide6.QtWidgets import (  # type: ignore[import-untyped]
            QDoubleSpinBox,
            QPushButton,
            QComboBox,
        )
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            transport_combo = window.findChild(QComboBox, "transport_combo")
            connect_btn = window.findChild(QPushButton, "connect_btn")
            transport_combo.setCurrentText("Sim")
            connect_btn.click()

            encoder_spin = window.findChild(QDoubleSpinBox, "sim_err_encoder_mm")
            slip_spin = window.findChild(QDoubleSpinBox, "sim_err_slip_turn")
            linear_spin = window.findChild(QDoubleSpinBox, "sim_err_otos_linear")
            yaw_spin = window.findChild(QDoubleSpinBox, "sim_err_otos_yaw")
            apply_btn = window.findChild(QPushButton, "sim_errors_apply_btn")

            encoder_spin.setValue(9.0)
            slip_spin.setValue(0.9)
            linear_spin.setValue(0.9)
            yaw_spin.setValue(0.09)

            apply_btn.click()

            assert len(applied) == 1, (
                f"Expected apply_error_profile to be called once on the "
                f"connected fake SimTransport; got {applied}"
            )
            assert applied[0] == {
                "encoder_noise_mm": 9.0,
                "slip_turn_extra": 0.9,
                "otos_linear_noise": 0.9,
                "otos_yaw_noise": 0.09,
            }
        finally:
            try:
                disconnect_btn = window.findChild(QPushButton, "disconnect_btn")
                if disconnect_btn is not None and disconnect_btn.isEnabled():
                    disconnect_btn.click()
            except Exception:
                pass
            window.hide()
