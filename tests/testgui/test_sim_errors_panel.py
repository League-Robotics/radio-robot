"""tests/testgui/test_sim_errors_panel.py — headless tests for the "Sim Errors" panel.

Covers (issue testgui-sim-error-profile-config; extended to the full SIMSET
knob set by ticket 069-007):
- The ``sim_errors_group`` QGroupBox and its spin boxes — the historical
  four (``sim_err_encoder_mm``, ``sim_err_slip_turn``,
  ``sim_err_otos_linear``, ``sim_err_otos_yaw``) plus the eleven new 069-007
  knobs (see ``_ALL_SIM_ERR_SPIN_NAMES`` below) — exist and are populated
  from ``sim_prefs.load_sim_error_profile()`` at window build.
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

# objectName -> DEFAULT_PROFILE key, for every 069-007 spin box (the
# historical four are covered by their own dedicated assertions below since
# they predate this map and have their own long-standing tests).
_NEW_SIM_ERR_SPIN_TO_PROFILE_KEY = {
    "sim_err_enc_scale_l": "enc_scale_err_l",
    "sim_err_enc_scale_r": "enc_scale_err_r",
    "sim_err_body_rot_scrub": "body_rot_scrub",
    "sim_err_body_lin_scrub": "body_lin_scrub",
    "sim_err_motor_offset_l": "motor_offset_l",
    "sim_err_motor_offset_r": "motor_offset_r",
    "sim_err_trackwidth": "trackwidth_mm",
    "sim_err_otos_lin_scale": "otos_lin_scale_err",
    "sim_err_otos_ang_scale": "otos_ang_scale_err",
    "sim_err_otos_lin_drift": "otos_lin_drift_mms",
    "sim_err_otos_yaw_drift": "otos_yaw_drift_degs",
}


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
                *_NEW_SIM_ERR_SPIN_TO_PROFILE_KEY,
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
            assert slip_spin.value() == pytest.approx(0.0)  # 073-003: was 0.26
            assert linear_spin.value() == pytest.approx(0.05)
            assert yaw_spin.value() == pytest.approx(0.0)

            # 069-007: every new knob must reproduce DEFAULT_PROFILE's
            # value, including the multiplicative knobs at 1.0 (not 0.0)
            # and trackwidth_mm at the real 150.0 (not 0.0).
            for object_name, profile_key in _NEW_SIM_ERR_SPIN_TO_PROFILE_KEY.items():
                spin = window.findChild(QDoubleSpinBox, object_name)
                assert spin.value() == pytest.approx(sim_prefs.DEFAULT_PROFILE[profile_key]), (
                    f"{object_name} did not default to "
                    f"DEFAULT_PROFILE['{profile_key}']"
                )
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
                "enc_scale_err_l": 0.04,
                "enc_scale_err_r": -0.04,
                "body_rot_scrub": 0.8,
                "body_lin_scrub": 0.85,
                "motor_offset_l": 1.1,
                "motor_offset_r": 0.9,
                "trackwidth_mm": 148.0,
                "otos_lin_scale_err": 0.03,
                "otos_ang_scale_err": -0.03,
                "otos_lin_drift_mms": 2.0,
                "otos_yaw_drift_degs": -1.5,
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

            expected = {
                "sim_err_enc_scale_l": 0.04,
                "sim_err_enc_scale_r": -0.04,
                "sim_err_body_rot_scrub": 0.8,
                "sim_err_body_lin_scrub": 0.85,
                "sim_err_motor_offset_l": 1.1,
                "sim_err_motor_offset_r": 0.9,
                "sim_err_trackwidth": 148.0,
                "sim_err_otos_lin_scale": 0.03,
                "sim_err_otos_ang_scale": -0.03,
                "sim_err_otos_lin_drift": 2.0,
                "sim_err_otos_yaw_drift": -1.5,
            }
            for object_name, value in expected.items():
                spin = window.findChild(QDoubleSpinBox, object_name)
                assert spin.value() == pytest.approx(value), f"{object_name} mismatch"
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
            # Only the historical four spin boxes were changed above; every
            # 069-007 knob must be present in the saved profile at its
            # DEFAULT_PROFILE value (untouched spin boxes keep their
            # window-build default).
            assert saved[0] == {
                **sim_prefs.DEFAULT_PROFILE,
                "encoder_noise_mm": 6.0,
                "slip_turn_extra": 0.6,
                "otos_linear_noise": 0.4,
                "otos_yaw_noise": 0.08,
            }
        finally:
            window.hide()

    def test_apply_saves_all_new_knob_field_values(self, qapp, monkeypatch, tmp_path):
        """069-007: changing a new knob's spin box must be reflected in the
        saved profile dict, not silently dropped."""
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
            new_values = {
                "sim_err_enc_scale_l": 0.1,
                "sim_err_enc_scale_r": -0.1,
                "sim_err_body_rot_scrub": 0.7,
                "sim_err_body_lin_scrub": 0.75,
                "sim_err_motor_offset_l": 1.2,
                "sim_err_motor_offset_r": 0.8,
                "sim_err_trackwidth": 160.0,
                "sim_err_otos_lin_scale": 0.15,
                "sim_err_otos_ang_scale": -0.15,
                "sim_err_otos_lin_drift": 3.0,
                "sim_err_otos_yaw_drift": -2.0,
            }
            for object_name, value in new_values.items():
                spin = window.findChild(QDoubleSpinBox, object_name)
                spin.setValue(value)

            apply_btn = window.findChild(QPushButton, "sim_errors_apply_btn")
            apply_btn.click()

            assert len(saved) == 1
            expected = {
                **sim_prefs.DEFAULT_PROFILE,
                "enc_scale_err_l": 0.1,
                "enc_scale_err_r": -0.1,
                "body_rot_scrub": 0.7,
                "body_lin_scrub": 0.75,
                "motor_offset_l": 1.2,
                "motor_offset_r": 0.8,
                "trackwidth_mm": 160.0,
                "otos_lin_scale_err": 0.15,
                "otos_ang_scale_err": -0.15,
                "otos_lin_drift_mms": 3.0,
                "otos_yaw_drift_degs": -2.0,
            }
            assert saved[0] == pytest.approx(expected)
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
                **sim_prefs.DEFAULT_PROFILE,
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
