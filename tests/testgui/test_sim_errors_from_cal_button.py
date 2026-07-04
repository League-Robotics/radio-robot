"""tests/testgui/test_sim_errors_from_cal_button.py — headless tests for the
"Sim Errors" panel's "From Calibration" button (issue
testgui-sim-errors-from-calibration-button, ticket 070-004).

Covers:
- Button existence (``sim_errors_from_cal_btn``, objectName) alongside the
  existing ``sim_errors_apply_btn``.
- The inverse-calibration mapping: ``get_robot_config()`` monkeypatched to
  return a fake (but real ``RobotConfig``) config with known
  ``calibration.rotational_slip`` / ``geometry.trackwidth``, click the
  button, assert every mapped spin box's value.
- The noise-field exception: ``sim_err_encoder_mm``, ``sim_err_otos_linear``,
  ``sim_err_otos_yaw`` are pre-set to a nonzero value and must be unchanged
  after the click.
- The same-path invariant: clicking the button calls
  ``sim_prefs.save_sim_error_profile`` and (when a connected fake
  ``SimTransport`` is present) ``SimTransport.apply_error_profile``, exactly
  once each, with the mapped profile — i.e. it reuses
  ``_on_sim_errors_apply``'s save/apply path rather than a second,
  independently-written one.
- The missing-config fallback: ``get_robot_config()`` returning ``None``, and
  returning a config whose ``calibration.rotational_slip`` or
  ``geometry.trackwidth`` is ``None``, falls back to the neutral value for
  that knob and logs a ``[WARN]`` (never raises).

Run with:
    QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/test_sim_errors_from_cal_button.py -q

Requirements: PySide6 (uv sync --group gui).
"""

from __future__ import annotations

import pytest


def _expected_mapping(rotational_slip: float, trackwidth: float) -> dict:
    """objectName -> expected spin-box value for the 12 mapped knobs, given
    an active-robot config with the supplied ``rotational_slip``/
    ``trackwidth``. Deliberately uses non-neutral values (0.85, 140.0) in
    the "happy path" tests below — distinct from every fallback value (1.0 /
    128.0) — so a bug that silently used the fallback instead of the config
    value would be caught.
    """
    return {
        "sim_err_slip_turn": 0.0,
        "sim_err_body_rot_scrub": rotational_slip,
        "sim_err_body_lin_scrub": 1.0,
        "sim_err_motor_offset_l": 1.0,
        "sim_err_motor_offset_r": 1.0,
        "sim_err_trackwidth": trackwidth,
        "sim_err_enc_scale_l": 0.0,
        "sim_err_enc_scale_r": 0.0,
        "sim_err_otos_lin_scale": 0.0,
        "sim_err_otos_ang_scale": 0.0,
        "sim_err_otos_lin_drift": 0.0,
        "sim_err_otos_yaw_drift": 0.0,
    }


def _fake_robot_config(*, rotational_slip=0.85, trackwidth=140.0, robot_name="faketovez"):
    """A real (pydantic) ``RobotConfig`` with only the fields this ticket
    cares about overridden.

    Using the real model (rather than an unconfigured ``MagicMock``)
    exercises the real ``.calibration.rotational_slip`` / ``.geometry.
    trackwidth`` attribute paths, and gives a real ``str`` ``.robot_name`` —
    required because ``_build_main_window()``'s pre-existing robot-combo
    preselect logic calls ``QComboBox.findText(cfg.robot_name)``, which
    raises a ``TypeError`` if handed a non-``str`` (e.g. an unconfigured
    ``MagicMock`` attribute).
    """
    from robot_radio.config.robot_config import (
        CalibrationConfig,
        GeometryConfig,
        IdentityConfig,
        RobotConfig,
    )

    return RobotConfig(
        identity=IdentityConfig(robot_name=robot_name, uid="fake-uid"),
        geometry=GeometryConfig(trackwidth=trackwidth),
        calibration=CalibrationConfig(rotational_slip=rotational_slip),
    )


@pytest.fixture(scope="session")
def qapp():
    """Ensure a QApplication exists for the whole test session (see test_smoke.py)."""
    import sys
    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


# ---------------------------------------------------------------------------
# Button existence
# ---------------------------------------------------------------------------


class TestSimErrorsFromCalButtonExistence:
    def test_button_exists_next_to_apply(self, qapp, monkeypatch, tmp_path):
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")

        from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            apply_btn = window.findChild(QPushButton, "sim_errors_apply_btn")
            from_cal_btn = window.findChild(QPushButton, "sim_errors_from_cal_btn")
            assert apply_btn is not None, "sim_errors_apply_btn not found"
            assert from_cal_btn is not None, "sim_errors_from_cal_btn not found"
            assert from_cal_btn.text() == "From Calibration"
        finally:
            window.hide()


# ---------------------------------------------------------------------------
# Mapping correctness
# ---------------------------------------------------------------------------


class TestSimErrorsFromCalMapping:
    def test_mapping_from_active_robot_config(self, qapp, monkeypatch, tmp_path):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")
        monkeypatch.setattr(sim_prefs, "save_sim_error_profile", lambda profile: None)

        fake_cfg = _fake_robot_config(rotational_slip=0.85, trackwidth=140.0)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        from PySide6.QtWidgets import QDoubleSpinBox, QPushButton  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            btn = window.findChild(QPushButton, "sim_errors_from_cal_btn")
            btn.click()

            expected = _expected_mapping(0.85, 140.0)
            for object_name, value in expected.items():
                spin = window.findChild(QDoubleSpinBox, object_name)
                assert spin.value() == pytest.approx(value), (
                    f"{object_name} mismatch after From Calibration click "
                    f"(expected {value}, got {spin.value()})"
                )
        finally:
            window.hide()


# ---------------------------------------------------------------------------
# Noise-field exception: the three noise spins are never touched
# ---------------------------------------------------------------------------


class TestSimErrorsFromCalNoiseFieldsUntouched:
    def test_noise_fields_unchanged_by_from_cal(self, qapp, monkeypatch, tmp_path):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")
        monkeypatch.setattr(sim_prefs, "save_sim_error_profile", lambda profile: None)

        fake_cfg = _fake_robot_config(rotational_slip=0.85, trackwidth=140.0)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        from PySide6.QtWidgets import QDoubleSpinBox, QPushButton  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            encoder_spin = window.findChild(QDoubleSpinBox, "sim_err_encoder_mm")
            linear_spin = window.findChild(QDoubleSpinBox, "sim_err_otos_linear")
            yaw_spin = window.findChild(QDoubleSpinBox, "sim_err_otos_yaw")

            # Pre-set all three noise fields to a nonzero value.
            encoder_spin.setValue(7.0)
            linear_spin.setValue(0.6)
            yaw_spin.setValue(0.11)

            btn = window.findChild(QPushButton, "sim_errors_from_cal_btn")
            btn.click()

            assert encoder_spin.value() == pytest.approx(7.0), (
                "sim_err_encoder_mm must not be touched by From Calibration"
            )
            assert linear_spin.value() == pytest.approx(0.6), (
                "sim_err_otos_linear must not be touched by From Calibration"
            )
            assert yaw_spin.value() == pytest.approx(0.11), (
                "sim_err_otos_yaw must not be touched by From Calibration"
            )
        finally:
            window.hide()


# ---------------------------------------------------------------------------
# Same-path invariant: reuses _on_sim_errors_apply's save/live-apply, not a
# second, independently-written save/apply call.
# ---------------------------------------------------------------------------


class TestSimErrorsFromCalSamePath:
    def test_from_cal_saves_via_sim_prefs_save(self, qapp, monkeypatch, tmp_path):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")

        saved: list[dict] = []
        monkeypatch.setattr(
            sim_prefs, "save_sim_error_profile", lambda profile: saved.append(profile)
        )

        fake_cfg = _fake_robot_config(rotational_slip=0.85, trackwidth=140.0)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            btn = window.findChild(QPushButton, "sim_errors_from_cal_btn")
            btn.click()

            assert len(saved) == 1, (
                "From Calibration must persist exactly once via "
                "sim_prefs.save_sim_error_profile (the same call "
                "_on_sim_errors_apply makes)"
            )
            expected = {
                **sim_prefs.DEFAULT_PROFILE,
                "slip_turn_extra": 0.0,
                "body_rot_scrub": 0.85,
                "body_lin_scrub": 1.0,
                "motor_offset_l": 1.0,
                "motor_offset_r": 1.0,
                "trackwidth_mm": 140.0,
                "enc_scale_err_l": 0.0,
                "enc_scale_err_r": 0.0,
                "otos_lin_scale_err": 0.0,
                "otos_ang_scale_err": 0.0,
                "otos_lin_drift_mms": 0.0,
                "otos_yaw_drift_degs": 0.0,
            }
            assert saved[0] == pytest.approx(expected)
        finally:
            window.hide()

    def test_from_cal_calls_live_apply_on_connected_sim_transport(
        self, qapp, monkeypatch, tmp_path
    ):
        """With a fake connected SimTransport, From Calibration must call
        transport.apply_error_profile(profile) with the mapped profile —
        mirrors test_sim_errors_panel.py's
        TestSimErrorsApplyButton.test_apply_calls_live_apply_on_connected_sim_transport.
        """
        import robot_radio.config.robot_config as robot_config_module
        import robot_radio.testgui.transport as transport_module
        from robot_radio.testgui import operations, sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")
        monkeypatch.setattr(sim_prefs, "save_sim_error_profile", lambda profile: None)

        fake_cfg = _fake_robot_config(rotational_slip=0.85, trackwidth=140.0)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

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

            def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
                return "OK"

            def apply_error_profile(self, profile: dict) -> None:
                applied.append(profile)

        FakeConnectedSimTransport.__name__ = "SimTransport"
        FakeConnectedSimTransport.__qualname__ = "SimTransport"

        assert operations.is_sim_transport(FakeConnectedSimTransport())

        monkeypatch.setattr(transport_module, "SimTransport", FakeConnectedSimTransport)

        from PySide6.QtWidgets import QComboBox, QPushButton  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            transport_combo = window.findChild(QComboBox, "transport_combo")
            connect_btn = window.findChild(QPushButton, "connect_btn")
            transport_combo.setCurrentText("Sim")
            connect_btn.click()

            btn = window.findChild(QPushButton, "sim_errors_from_cal_btn")
            btn.click()

            assert len(applied) == 1, (
                f"Expected apply_error_profile to be called once on the "
                f"connected fake SimTransport; got {applied}"
            )
            expected = {
                **sim_prefs.DEFAULT_PROFILE,
                "slip_turn_extra": 0.0,
                "body_rot_scrub": 0.85,
                "body_lin_scrub": 1.0,
                "motor_offset_l": 1.0,
                "motor_offset_r": 1.0,
                "trackwidth_mm": 140.0,
                "enc_scale_err_l": 0.0,
                "enc_scale_err_r": 0.0,
                "otos_lin_scale_err": 0.0,
                "otos_ang_scale_err": 0.0,
                "otos_lin_drift_mms": 0.0,
                "otos_yaw_drift_degs": 0.0,
            }
            assert applied[0] == pytest.approx(expected)
        finally:
            try:
                disconnect_btn = window.findChild(QPushButton, "disconnect_btn")
                if disconnect_btn is not None and disconnect_btn.isEnabled():
                    disconnect_btn.click()
            except Exception:
                pass
            window.hide()


# ---------------------------------------------------------------------------
# Missing / partial config fallback — never raises, logs a [WARN]
# ---------------------------------------------------------------------------


class TestSimErrorsFromCalMissingConfigFallback:
    def test_none_config_falls_back_to_neutral_and_logs_warning(
        self, qapp, monkeypatch, tmp_path
    ):
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")
        monkeypatch.setattr(sim_prefs, "save_sim_error_profile", lambda profile: None)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: None)

        from PySide6.QtWidgets import (  # type: ignore[import-untyped]
            QDoubleSpinBox,
            QPlainTextEdit,
            QPushButton,
        )
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            btn = window.findChild(QPushButton, "sim_errors_from_cal_btn")
            btn.click()  # must not raise

            rot_spin = window.findChild(QDoubleSpinBox, "sim_err_body_rot_scrub")
            tw_spin = window.findChild(QDoubleSpinBox, "sim_err_trackwidth")

            assert rot_spin.value() == pytest.approx(1.0), (
                "body_rot_scrub must fall back to the neutral 1.0 when "
                "get_robot_config() returns None"
            )
            assert tw_spin.value() == pytest.approx(
                sim_prefs.DEFAULT_PROFILE["trackwidth_mm"]
            ), (
                "trackwidth must fall back to DEFAULT_PROFILE['trackwidth_mm'] "
                "when get_robot_config() returns None"
            )

            log_pane = window.findChild(QPlainTextEdit, "log_pane")
            assert "[WARN]" in log_pane.toPlainText(), (
                "a missing config must log a [WARN] line, not fail silently"
            )
        finally:
            window.hide()

    def test_none_rotational_slip_falls_back_to_neutral(self, qapp, monkeypatch, tmp_path):
        """geometry.trackwidth present, calibration.rotational_slip missing:
        only body_rot_scrub falls back; trackwidth still comes from config."""
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")
        monkeypatch.setattr(sim_prefs, "save_sim_error_profile", lambda profile: None)

        fake_cfg = _fake_robot_config(rotational_slip=None, trackwidth=140.0)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        from PySide6.QtWidgets import (  # type: ignore[import-untyped]
            QDoubleSpinBox,
            QPlainTextEdit,
            QPushButton,
        )
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            btn = window.findChild(QPushButton, "sim_errors_from_cal_btn")
            btn.click()

            rot_spin = window.findChild(QDoubleSpinBox, "sim_err_body_rot_scrub")
            tw_spin = window.findChild(QDoubleSpinBox, "sim_err_trackwidth")

            assert rot_spin.value() == pytest.approx(1.0), (
                "body_rot_scrub must fall back to 1.0 when "
                "calibration.rotational_slip is None"
            )
            assert tw_spin.value() == pytest.approx(140.0), (
                "trackwidth must still come from geometry.trackwidth when "
                "only rotational_slip is missing"
            )

            log_pane = window.findChild(QPlainTextEdit, "log_pane")
            assert "[WARN]" in log_pane.toPlainText()
        finally:
            window.hide()

    def test_none_trackwidth_falls_back_to_neutral(self, qapp, monkeypatch, tmp_path):
        """calibration.rotational_slip present, geometry.trackwidth missing:
        only trackwidth falls back; body_rot_scrub still comes from config."""
        import robot_radio.config.robot_config as robot_config_module
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")
        monkeypatch.setattr(sim_prefs, "save_sim_error_profile", lambda profile: None)

        fake_cfg = _fake_robot_config(rotational_slip=0.85, trackwidth=None)
        monkeypatch.setattr(robot_config_module, "get_robot_config", lambda: fake_cfg)

        from PySide6.QtWidgets import (  # type: ignore[import-untyped]
            QDoubleSpinBox,
            QPlainTextEdit,
            QPushButton,
        )
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            btn = window.findChild(QPushButton, "sim_errors_from_cal_btn")
            btn.click()

            rot_spin = window.findChild(QDoubleSpinBox, "sim_err_body_rot_scrub")
            tw_spin = window.findChild(QDoubleSpinBox, "sim_err_trackwidth")

            assert rot_spin.value() == pytest.approx(0.85), (
                "body_rot_scrub must still come from calibration.rotational_slip "
                "when only trackwidth is missing"
            )
            assert tw_spin.value() == pytest.approx(
                sim_prefs.DEFAULT_PROFILE["trackwidth_mm"]
            ), (
                "trackwidth must fall back to DEFAULT_PROFILE['trackwidth_mm'] "
                "when geometry.trackwidth is None"
            )

            log_pane = window.findChild(QPlainTextEdit, "log_pane")
            assert "[WARN]" in log_pane.toPlainText()
        finally:
            window.hide()
