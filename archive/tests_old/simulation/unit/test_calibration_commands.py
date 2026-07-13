"""test_calibration_commands.py — the transport-agnostic calibration builder.

``robot_radio.calibration.push.calibration_commands`` builds the wire
sequence any transport (SerialConnection, NezhaProtocol, or the TestGUI's
Transport) pushes when a robot is opened/selected.  The stakeholder contract
(2026-07-03): selecting a robot must be AUTHORITATIVE — the config's
calibration overwrites whatever the firmware baked in at compile time
(DefaultConfig.cpp), and an UNCALIBRATED config pushes neutral values so
"no calibration" really means no calibration:

  * ``rotational_slip`` missing/null  ->  ``SET rotSlip=0`` (the documented
    no-correction sentinel; effectiveSlip() maps 0 -> 1.0).  This is the
    key that makes "tovez nocal + zero sim errors = geometry-pure turns"
    hold in the sim, where DefaultConfig bakes rotationalSlip=0.92.
"""
from __future__ import annotations

import json

import pytest

from robot_radio.calibration.push import calibration_commands
from robot_radio.config.robot_config import load_robot_config


def _load(tmp_path, cfg: dict):
    p = tmp_path / "cfg.json"
    p.write_text(json.dumps(cfg))
    return load_robot_config(p)


def _base_config() -> dict:
    return {
        "schema_version": 2,
        "identity": {"robot_name": "testbot", "uid": "testbot"},
        "connection": {"device_announcement_name": "testbot"},
        "geometry": {"trackwidth": 128},
        "wheels": {"wheel_diameter_mm": 80.77},
    }


def test_calibrated_config_pushes_its_rotational_slip(tmp_path):
    cfg_dict = _base_config()
    cfg_dict["calibration"] = {"rotational_slip": 0.92}
    cmds = [c for c, _ in calibration_commands(_load(tmp_path, cfg_dict))]

    assert "SET rotSlip=0.92" in cmds
    assert "SET tw=128" in cmds


def test_uncalibrated_config_pushes_neutral_rotslip_sentinel(tmp_path):
    """No calibration section -> SET rotSlip=0 is STILL sent (sentinel), so
    the firmware's compiled-in calibration cannot leak into a nocal run."""
    cmds = [c for c, _ in calibration_commands(_load(tmp_path, _base_config()))]

    assert "SET rotSlip=0" in cmds, (
        "an uncalibrated config must explicitly neutralize the firmware's "
        f"baked-in rotationalSlip; sent: {cmds}"
    )


def test_null_rotational_slip_also_pushes_sentinel(tmp_path):
    cfg_dict = _base_config()
    cfg_dict["calibration"] = {"rotational_slip": None, "otos_linear_scale": 1.05}
    cmds = [c for c, _ in calibration_commands(_load(tmp_path, cfg_dict))]

    assert "SET rotSlip=0" in cmds


def test_sequence_shape_and_otos_ordering(tmp_path):
    """ml/mr derive from wheel diameter when per-wheel values are absent;
    OI precedes the OL/OA scalar writes (hardware requirement)."""
    cmds = [c for c, _ in calibration_commands(_load(tmp_path, _base_config()))]

    # pi * 80.77 / 360 = 0.704851...
    assert any(c.startswith("SET ml=0.7048") for c in cmds)
    assert any(c.startswith("SET mr=0.7048") for c in cmds)
    oi = cmds.index("OI")
    ol = next(i for i, c in enumerate(cmds) if c.startswith("OL "))
    oa = next(i for i, c in enumerate(cmds) if c.startswith("OA "))
    assert oi < ol and oi < oa


def test_commands_land_in_firmware(sim, tmp_path):
    """End-to-end against the real sim: pushing an uncalibrated config
    overwrites the baked DefaultConfig rotationalSlip (0.92 -> 0) and
    trackwidth stays at the config's value."""
    reply = sim.send_command("GET rotSlip")
    assert "rotSlip=0.920" in reply, f"unexpected baked value: {reply!r}"

    for cmd, _read_ms in calibration_commands(_load(tmp_path, _base_config())):
        reply = sim.send_command(cmd)
        if "NODEV" in reply.upper():
            # OI/OL/OA talk to the physical OTOS device, which the sim does
            # not have (its OTOS *model* is configured via SIMSET) — an
            # expected, tolerated skip; the GUI push treats it the same way.
            assert cmd == "OI" or cmd.startswith(("OL ", "OA ")), (
                f"unexpected nodev for {cmd!r}"
            )
            continue
        assert "ERR" not in reply.upper(), f"{cmd!r} rejected: {reply!r}"

    assert "rotSlip=0.000" in sim.send_command("GET rotSlip")
    assert "tw=128" in sim.send_command("GET tw")
