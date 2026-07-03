"""tests/testgui/test_calibration_push_on_connect.py — robot select is authoritative.

Stakeholder contract (2026-07-03): opening a robot in the TestGUI must send
that robot's calibration values to the robot — what DefaultConfig.cpp baked
in at compile time must not matter.  In Sim mode the firmware bakes tovez's
calibration (rotationalSlip=0.92, trackwidth=128); selecting the
uncalibrated "tovez nocal" must therefore push neutral values over it
(``SET rotSlip=0``), so a nocal robot at zero sim errors runs geometry-pure.

Drives the REAL headless GUI (offscreen Qt) and the REAL ctypes firmware
sim: pin the active robot via ROBOT_CONFIG, click Connect, and read the
firmware's live config back through the transport (``GET rotSlip``).
"""
from __future__ import annotations

import json
import pathlib
import time

import pytest

_REPO = pathlib.Path(__file__).parent.parent.parent
_SIM_BUILD = _REPO / "tests" / "_infra" / "sim" / "build"

_LIB_PRESENT = any(
    (_SIM_BUILD / name).exists()
    for name in ("libfirmware_host.dylib", "libfirmware_host.so")
)

pytestmark = pytest.mark.skipif(
    not _LIB_PRESENT,
    reason="firmware sim lib not built (tests/_infra/sim: cmake --build build)",
)


@pytest.fixture(scope="module")
def qapp():
    import sys

    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


def _connect_gui_with_config(qapp, monkeypatch, tmp_path, cfg: dict):
    """Pin *cfg* as the active robot, build the GUI, click Connect.

    Returns (window, transport).  Caller must disconnect + hide.
    """
    from PySide6.QtWidgets import QComboBox, QPushButton  # type: ignore[import-untyped]

    import robot_radio.testgui.__main__ as gui_main
    from robot_radio.config import robot_config as rc_mod
    from robot_radio.testgui import sim_prefs
    from robot_radio.testgui import transport as transport_mod

    cfg_path = tmp_path / "active.json"
    cfg_path.write_text(json.dumps(cfg))
    monkeypatch.setenv("ROBOT_CONFIG", str(cfg_path))
    rc_mod._reset_robot_config()

    monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)
    monkeypatch.setattr(
        sim_prefs, "_PREFS_PATH", tmp_path / "sim_error_profile.json"
    )
    monkeypatch.setattr(transport_mod, "_SIM_TICK_SLEEP_S", 0.004)

    _RealSimTransport = transport_mod.SimTransport
    created: list = []

    class SimTransport(_RealSimTransport):  # noqa: N801 — name-checked by is_sim_transport
        def __init__(self) -> None:
            super().__init__()
            created.append(self)

    monkeypatch.setattr(transport_mod, "SimTransport", SimTransport)

    window, _app = gui_main._build_main_window()
    combo = window.findChild(QComboBox, "transport_combo")
    combo.setCurrentText("Sim")
    connect_btn = window.findChild(QPushButton, "connect_btn")
    connect_btn.click()

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        qapp.processEvents()
        time.sleep(0.01)
        if created and created[-1]._connected:
            break

    assert created and created[-1]._connected, "SimTransport failed to connect"
    return window, created[-1]


def _teardown(qapp, window):
    from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]

    from robot_radio.config import robot_config as rc_mod

    btn = window.findChild(QPushButton, "disconnect_btn")
    if btn is not None and btn.isEnabled():
        btn.click()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            qapp.processEvents()
            time.sleep(0.01)
    window.hide()
    rc_mod._reset_robot_config()


def _nocal_config() -> dict:
    return {
        "schema_version": 2,
        "identity": {"robot_name": "tovez nocal", "uid": "tovez-nocal"},
        "connection": {"device_announcement_name": "tovez"},
        "geometry": {"trackwidth": 128},
        "wheels": {"wheel_diameter_mm": 80.77},
    }


def test_connect_pushes_nocal_neutral_calibration_into_firmware(
    qapp, monkeypatch, tmp_path
):
    """Connect with an uncalibrated active robot -> the firmware's baked
    rotationalSlip=0.92 is overwritten with the neutral sentinel 0."""
    window, transport = _connect_gui_with_config(
        qapp, monkeypatch, tmp_path, _nocal_config()
    )
    try:
        reply = transport.command("GET rotSlip", read_ms=500)
        assert "rotSlip=0.000" in reply, (
            f"nocal connect must neutralize the baked rotationalSlip; "
            f"firmware reports {reply.strip()!r}"
        )
        assert "tw=128" in transport.command("GET tw", read_ms=500)
    finally:
        _teardown(qapp, window)


def test_connect_pushes_calibrated_values_into_firmware(
    qapp, monkeypatch, tmp_path
):
    """Connect with a calibrated active robot -> its values land verbatim
    (here deliberately different from the baked 0.92 to prove overwrite)."""
    cfg = _nocal_config()
    cfg["identity"] = {"robot_name": "tovez-custom", "uid": "tovez-custom"}
    cfg["calibration"] = {"rotational_slip": 0.85}
    window, transport = _connect_gui_with_config(qapp, monkeypatch, tmp_path, cfg)
    try:
        reply = transport.command("GET rotSlip", read_ms=500)
        assert "rotSlip=0.850" in reply, (
            f"connect must push the active robot's calibration; firmware "
            f"reports {reply.strip()!r}"
        )
    finally:
        _teardown(qapp, window)
