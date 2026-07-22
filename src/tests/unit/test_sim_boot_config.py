"""src/tests/unit/test_sim_boot_config.py -- ticket 113-004, narrowed by
115-003 (gut-to-minimal-firmware S1 motion-stack excision).

``sim_boot_config.py`` computes the Tier-2 (boot-only) ``Devices::
MotorConfig`` scalar set that ``gen_boot_config.py`` bakes into a real
robot's ``boot_config.cpp``, but from an already-loaded ``RobotConfig`` (or
a raw robot-JSON dict) at sim-open time -- by CALLING
``gen_boot_config.py``'s own mapping functions, never re-deriving them
(sprint 113 Design Rationale Decision 2).

115-003 deleted ``sim_boot_config.py``'s ``msg::PlannerConfig`` half
(``planner_boot_config_for()``/``_heading_source_wire_value()``) wholesale
-- ``msg::PlannerConfig`` itself, and every ``gen_boot_config.py`` mapping
function it called (``motion_limits_for_config``/
``profile_rot_limits_for_config``/``min_speed_for_config``/
``heading_gains_for_config``/``arrive_dwell_for_config``/
``heading_source_for_config``/``heading_dwell_for_config``/
``lead_compensation_for_config``/``actuation_lag_for_config``/
``distance_gains_for_config``/``model_tau_for_config``), went with the
deleted ``App::Pilot``/``Motion::Executor`` subsystems (ticket 003's proto
surgery). Every ``planner_boot_config_for()`` test this file used to carry
is removed along with it -- only ``motor_boot_config_for()`` coverage
survives, UNCHANGED (it depends only on ``vel_gains_for_config()``/
``fwd_sign_for_ports()``, both still live).

Covers, per the ticket's own Testing section:
  1. ``motor_boot_config_for()`` against ``tovez.json`` AND
     ``tovez_nocal.json``, asserting each returned value equals what
     ``gen_boot_config.py``'s own functions independently compute for the
     SAME input -- a direct call-through comparison, never a hardcoded
     expected-value table, so this test can't silently drift from the
     generator it mirrors.
  2. A fallback case: a minimal cfg missing the "control"/"calibration"
     section entirely hard-fails (sprint 114 config-as-truth completion),
     not a silent fallback to old firmware defaults.
  3. The drive pair's mirror-mounted ``fwd_sign`` (088-002) genuinely reads
     through the ``RobotConfig`` path.

This module is Qt-free and sim-lib-free (pure function coverage only) --
collected under ``src/tests/unit/`` per ``pyproject.toml``'s ``testpaths``,
mirroring ``src/tests/unit/test_calibration_kwargs.py``'s own placement
(the Tier-1 sibling of this Tier-2 helper).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from robot_radio.calibration.sim_boot_config import motor_boot_config_for
from robot_radio.config.robot_config import (
    CalibrationConfig,
    ControlConfig,
    RobotConfig,
    IdentityConfig,
    load_robot_config,
)

# src/tests/unit/test_sim_boot_config.py -> unit -> tests -> src -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"
_SCRIPTS_DIR = _REPO_ROOT / "src" / "scripts"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)


_ROBOT_JSON_NAMES = ["tovez.json", "tovez_nocal.json"]


def _raw_cfg(name: str) -> dict:
    return json.loads((_ROBOTS_DIR / name).read_text())


# ---------------------------------------------------------------------------
# motor_boot_config_for() parity.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", _ROBOT_JSON_NAMES)
@pytest.mark.parametrize("port", [1, 2, 3, 4])
def test_motor_boot_config_for_matches_gen_boot_config(name, port):
    raw = _raw_cfg(name)
    robot_config = load_robot_config(_ROBOTS_DIR / name)

    result = motor_boot_config_for(robot_config, port)

    *_gains, expected_filt = gbc.vel_gains_for_config(raw)
    expected_fwd_signs = gbc.fwd_sign_for_ports(raw)

    assert result["vel_filt_alpha"] == expected_filt
    assert result["fwd_sign"] == expected_fwd_signs[port - 1]


def test_motor_boot_config_for_raises_with_no_calibration_or_control_section():
    """motor_boot_config_for() unconditionally resolves vel_gains_for_config()
    first (for vel_filt_alpha) -- with no control.vel_* keys at all, it must
    raise the same MissingRobotConfigKeyError gen_boot_config.py itself
    raises (sprint 114 config-as-truth completion), not silently return the
    old VEL_FILT_ALPHA/FWD_SIGN placeholder pair."""
    with pytest.raises(gbc.MissingRobotConfigKeyError):
        motor_boot_config_for({}, port=1)


# A fully-populated control block (sprint 114: vel_gains_for_config() has no
# fallback, so any motor_boot_config_for() call needs one) -- values are
# arbitrary/don't-care except where a specific test overrides fwd_sign via
# calibration below.
_FULL_CONTROL_FOR_MOTOR_TESTS = ControlConfig(
    vel_kp=0.002, vel_ki=0.0, vel_kff=0.002, vel_imax=0.0, vel_kaw=0.0, vel_filt=1.0,
)


def test_motor_boot_config_for_reads_fwd_sign_from_robot_config():
    """Proves the drive pair's mirror-mounted fwd_sign (088-002) genuinely
    reads through the RobotConfig path -- both shipped fixtures happen to
    set left=+1/right=-1, so this synthetic config picks the OPPOSITE signs
    to prove it isn't coincidentally matching a hardcoded default."""
    cfg = RobotConfig(
        identity=IdentityConfig(robot_name="r", uid="0"),
        calibration=CalibrationConfig(fwd_sign_left=-1, fwd_sign_right=1),
        control=_FULL_CONTROL_FOR_MOTOR_TESTS,
    )

    left = motor_boot_config_for(cfg, port=1)
    right = motor_boot_config_for(cfg, port=2)
    other = motor_boot_config_for(cfg, port=3)

    assert left["fwd_sign"] == -1
    assert right["fwd_sign"] == 1
    assert other["fwd_sign"] == gbc.FWD_SIGN
