"""tests/testgui/test_sim_errors_from_calibration.py -- ticket 085-006: "From
Calibration" button, driven against the REAL config loader and REAL
``data/robots/tovez.json``. Ported from
``tests_old/testgui/test_sim_errors_from_calibration.py``.

The Sim Errors panel's **From Calibration** button (ticket 070-004) populates
the error knobs with the INVERSE of the active robot's calibration: the sim
firmware bakes the robot's calibration into DefaultConfig.cpp (e.g.
``rotationalSlip=0.92``), but the sim plant is ideal (zero scrub) -- so the
firmware's correction over-rotates RT against the ideal plant. Loading the
inverse (``body rot scrub = calibration.rotational_slip``, ``trackwidth =
geometry.trackwidth``, every other knob neutral) makes the firmware's
correction and the plant's scrub cancel.

Stakeholder-defined invariant (2026-07-03): with a robot that has NO
calibration ("tovez nocal"), From Calibration must yield the ZERO-ERROR
panel -- there is no calibration to invert, so every knob lands at its
neutral value (additive 0.0, multiplicative 1.0, trackwidth = the robot's
geometry trackwidth) and the three noise fields are left untouched.

These tests drive the REAL headless GUI (offscreen Qt) and the REAL config
loader; the active robot is injected via the ``ROBOT_CONFIG`` env var
(resolution order #1 in ``get_robot_config``) plus a config-cache reset, so
they never depend on -- or disturb -- ``data/robots/active_robot.json``.
No sim connection is needed: From Calibration edits the panel and persists
via Apply regardless of connection state.

**Relationship to test_sim_errors_from_cal_button.py** (085-006 Open
Question 3): these two files test overlapping ground -- both assert the
rotational_slip/trackwidth -> body_rot_scrub/trackwidth mapping for a nocal
and a calibrated robot -- from two different historical eras. Neither fully
supersedes the other: ``test_sim_errors_from_cal_button.py`` is the
strictly broader suite (adds the missing-config-entirely fallback, the
missing-single-field fallback for each field independently, the
same-path-as-Apply invariant, and a fake connected SimTransport's
``apply_error_profile`` call) via a mocked ``get_robot_config()``; THIS file
is the only one that exercises the REAL config loader
(``ROBOT_CONFIG``/``load_robot_config``) end to end against the REAL
``data/robots/tovez.json`` values, which the mocked-config file does not
cover at all. Both are kept, per this ticket's instruction not to resolve
the overlap unilaterally.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_sim_errors_from_calibration.py -v
"""
from __future__ import annotations

import json
import pathlib
import time

import pytest

_REPO = pathlib.Path(__file__).resolve().parents[2]
_TOVEZ_JSON = _REPO / "data" / "robots" / "tovez.json"

#: Panel state expected for a robot with NO calibration: the zero-error case.
#: (objectName -> value).  The three noise fields are absent by design --
#: From Calibration never touches them.
_ZERO_ERROR_PANEL: dict[str, float] = {
    "sim_err_slip_turn": 0.0,
    "sim_err_body_rot_scrub": 1.0,
    "sim_err_body_lin_scrub": 1.0,
    "sim_err_motor_offset_l": 1.0,
    "sim_err_motor_offset_r": 1.0,
    "sim_err_trackwidth": 128.0,       # nocal keeps its geometry.trackwidth
    "sim_err_enc_scale_l": 0.0,
    "sim_err_enc_scale_r": 0.0,
    "sim_err_otos_lin_scale": 0.0,
    "sim_err_otos_ang_scale": 0.0,
    "sim_err_otos_lin_drift": 0.0,
    "sim_err_otos_yaw_drift": 0.0,
}

#: Sentinels proving the noise fields are left alone by From Calibration.
_NOISE_SENTINELS: dict[str, float] = {
    "sim_err_encoder_mm": 1.23,
    "sim_err_otos_linear": 0.45,
    "sim_err_otos_yaw": 0.067,
}


@pytest.fixture(scope="module")
def qapp():
    """QApplication for the module (offscreen platform set by conftest)."""
    import sys

    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


@pytest.fixture
def robot_config_env(monkeypatch, tmp_path):
    """Return a function that makes a given config dict the active robot.

    Uses the ``ROBOT_CONFIG`` env var (resolution order #1) plus the
    config-cache reset, and restores + re-resets on teardown so later tests
    resolve the repo's real active robot again.
    """
    from robot_radio.config import robot_config as rc_mod

    def _activate(cfg: dict, name: str) -> pathlib.Path:
        path = tmp_path / f"{name}.json"
        path.write_text(json.dumps(cfg))
        monkeypatch.setenv("ROBOT_CONFIG", str(path))
        rc_mod._reset_robot_config()
        return path

    yield _activate
    rc_mod._reset_robot_config()


def _build_window_with_tmp_prefs(monkeypatch, tmp_path):
    """Build the real main window with sim_prefs persistence redirected."""
    from robot_radio.testgui import sim_prefs
    import robot_radio.testgui.__main__ as gui_main

    monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)
    monkeypatch.setattr(
        sim_prefs, "_PREFS_PATH", tmp_path / "sim_error_profile.json"
    )
    window, _app = gui_main._build_main_window()
    return window


def _click_from_cal_and_read_panel(qapp, window) -> dict[str, float]:
    """Set noise sentinels, click From Calibration, return all spin values."""
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QDoubleSpinBox,
        QPushButton,
    )

    for name, value in _NOISE_SENTINELS.items():
        spin = window.findChild(QDoubleSpinBox, name)
        assert spin is not None, f"noise spinbox {name!r} not found"
        spin.setValue(value)

    btn = window.findChild(QPushButton, "sim_errors_from_cal_btn")
    assert btn is not None, "From Calibration button not found"
    btn.click()
    # The click handler is synchronous (panel edits + prefs save); a short
    # event-loop spin keeps queued log-pane updates from piling up.
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        qapp.processEvents()

    values: dict[str, float] = {}
    for name in list(_ZERO_ERROR_PANEL) + list(_NOISE_SENTINELS):
        spin = window.findChild(QDoubleSpinBox, name)
        assert spin is not None, f"spinbox {name!r} not found"
        values[name] = spin.value()
    return values


def _load_tovez() -> dict:
    return json.loads(_TOVEZ_JSON.read_text())


def test_nocal_robot_yields_zero_error_panel(
    qapp, monkeypatch, tmp_path, robot_config_env
):
    """A robot with NO calibration section -> From Calibration produces the
    zero-error panel (nothing to invert), noise fields untouched, and the
    zero-error profile is persisted (Apply runs as part of the click)."""
    from robot_radio.testgui import sim_prefs

    nocal = _load_tovez()
    nocal.pop("calibration", None)  # "tovez nocal": no calibration at all
    nocal["identity"]["robot_name"] = "tovez nocal"
    nocal["identity"]["uid"] = "tovez-nocal"
    robot_config_env(nocal, "tovez_nocal")

    window = _build_window_with_tmp_prefs(monkeypatch, tmp_path)
    try:
        values = _click_from_cal_and_read_panel(qapp, window)
    finally:
        window.hide()

    mismatches = [
        f"{name}: expected {expected}, panel shows {values[name]}"
        for name, expected in _ZERO_ERROR_PANEL.items()
        if values[name] != pytest.approx(expected, abs=1e-6)
    ]
    assert not mismatches, (
        "From Calibration with a nocal robot must yield the zero-error "
        "panel:\n  " + "\n  ".join(mismatches)
    )

    for name, sentinel in _NOISE_SENTINELS.items():
        assert values[name] == pytest.approx(sentinel, abs=1e-6), (
            f"From Calibration must not touch noise field {name!r}"
        )

    # The click ends in Apply -> the zero-error profile (plus the untouched
    # noise sentinels) must be persisted.
    saved = sim_prefs.load_sim_error_profile()
    assert saved["body_rot_scrub"] == pytest.approx(1.0)
    assert saved["trackwidth"] == pytest.approx(128.0)
    assert saved["slip_turn_extra"] == pytest.approx(0.0)
    assert saved["encoder_noise"] == pytest.approx(
        _NOISE_SENTINELS["sim_err_encoder_mm"]
    )


def test_calibrated_robot_yields_inverse_calibration(
    qapp, monkeypatch, tmp_path, robot_config_env
):
    """The real calibrated tovez config -> body rot scrub = rotational_slip
    (0.92), trackwidth = geometry.trackwidth (128), everything else neutral."""
    cfg = _load_tovez()
    expected_slip = cfg["calibration"]["rotational_slip"]
    expected_tw = cfg["geometry"]["trackwidth"]
    robot_config_env(cfg, "tovez_calibrated")

    window = _build_window_with_tmp_prefs(monkeypatch, tmp_path)
    try:
        values = _click_from_cal_and_read_panel(qapp, window)
    finally:
        window.hide()

    assert values["sim_err_body_rot_scrub"] == pytest.approx(
        expected_slip, abs=1e-6
    ), "body rot scrub must mirror calibration.rotational_slip"
    assert values["sim_err_trackwidth"] == pytest.approx(expected_tw, abs=1e-6)
    # Everything else stays neutral even for a calibrated robot (the other
    # calibration entries are inert in sim -- see the 070-004 mapping).
    for name in (
        "sim_err_slip_turn",
        "sim_err_enc_scale_l",
        "sim_err_enc_scale_r",
        "sim_err_otos_lin_scale",
        "sim_err_otos_ang_scale",
        "sim_err_otos_lin_drift",
        "sim_err_otos_yaw_drift",
    ):
        assert values[name] == pytest.approx(0.0, abs=1e-6), name
    for name in (
        "sim_err_body_lin_scrub",
        "sim_err_motor_offset_l",
        "sim_err_motor_offset_r",
    ):
        assert values[name] == pytest.approx(1.0, abs=1e-6), name
