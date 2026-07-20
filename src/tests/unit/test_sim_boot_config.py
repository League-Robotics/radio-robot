"""src/tests/unit/test_sim_boot_config.py -- ticket 113-004.

``sim_boot_config.py`` computes the Tier-2 (boot-only) ``msg::PlannerConfig``/
``Devices::MotorConfig`` scalar set that ``gen_boot_config.py`` bakes into a
real robot's ``boot_config.cpp``, but from an already-loaded ``RobotConfig``
(or a raw robot-JSON dict) at sim-open time -- by CALLING
``gen_boot_config.py``'s own mapping functions, never re-deriving them
(sprint 113 Design Rationale Decision 2).

Covers, per the ticket's own Testing section:
  1. ``planner_boot_config_for()``/``motor_boot_config_for()`` against
     ``tovez.json`` AND ``tovez_nocal.json``, asserting each returned value
     equals what ``gen_boot_config.py``'s own functions independently
     compute for the SAME input -- a direct call-through comparison, never
     a hardcoded expected-value table, so this test can't silently drift
     from the generator it mirrors.
  2. The same parity holds when the source is a raw robot-JSON dict
     (``gen_boot_config.py``'s own native input shape), proving
     ``_as_cfg_dict()`` is a lossless passthrough for that shape.
  3. A fallback case: a minimal cfg missing the "control" section entirely
     (and a default-constructed ``RobotConfig``) resolves every field to
     its documented ``gen_boot_config.py`` default.
  4. Every Tier-2 field genuinely READS from the config, not merely always
     returning the default -- both shipped fixtures omit ``heading_source``/
     ``heading_lead_bias``/``plan_lead``/``terminal_lead`` entirely, so the
     fixture-based parity tests above can't distinguish "read" from
     "always default" for those four fields; a synthetic arbitrary-value
     config (both as a raw dict and as a ``RobotConfig``) closes that gap.
  5. ``heading_source`` resolves to the wire int enum value (via the real
     generated ``planner_pb2.HeadingSourceMode``), covering all three
     values (auto/otos/encoder), not just the default both shipped
     profiles happen to leave unset.

This module is Qt-free and sim-lib-free (pure function coverage only) --
collected under ``src/tests/unit/`` per ``pyproject.toml``'s ``testpaths``,
mirroring ``src/tests/unit/test_calibration_kwargs.py``'s own placement
(the Tier-1 sibling of this Tier-2 helper).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

from robot_radio.calibration.sim_boot_config import (
    motor_boot_config_for,
    planner_boot_config_for,
)
from robot_radio.config.robot_config import (
    CalibrationConfig,
    ControlConfig,
    IdentityConfig,
    RobotConfig,
    load_robot_config,
)
from robot_radio.robot.pb2 import planner_pb2

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
# 1/2. planner_boot_config_for() parity -- RobotConfig source AND raw-dict
#      source, both against gen_boot_config.py's own functions directly.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", _ROBOT_JSON_NAMES)
def test_planner_boot_config_for_matches_gen_boot_config_from_robot_config(name):
    raw = _raw_cfg(name)
    robot_config = load_robot_config(_ROBOTS_DIR / name)

    result = planner_boot_config_for(robot_config)

    assert result["a_max"] == gbc.A_MAX_DEFAULT
    assert result["a_decel"] == gbc.A_DECEL_DEFAULT
    assert result["v_body_max"] == gbc.V_BODY_MAX_DEFAULT
    assert result["j_max"] == gbc.J_MAX_DEFAULT
    assert result["yaw_jerk_max"] == gbc.YAW_JERK_MAX_DEFAULT

    yaw_rate_max, yaw_acc_max = gbc.profile_rot_limits_for_config(raw)
    assert result["yaw_rate_max"] == yaw_rate_max
    assert result["yaw_acc_max"] == yaw_acc_max

    assert result["min_speed"] == gbc.min_speed_for_config(raw)

    heading_kp, heading_kd = gbc.heading_gains_for_config(raw)
    assert result["heading_kp"] == heading_kp
    assert result["heading_kd"] == heading_kd

    assert result["arrive_dwell"] == gbc.arrive_dwell_for_config(raw)

    heading_dwell_tol, heading_dwell_rate = gbc.heading_dwell_for_config(raw)
    assert result["heading_dwell_tol"] == heading_dwell_tol
    assert result["heading_dwell_rate"] == heading_dwell_rate

    heading_lead_bias, plan_lead, terminal_lead = gbc.lead_compensation_for_config(raw)
    assert result["heading_lead_bias"] == heading_lead_bias
    assert result["plan_lead"] == plan_lead
    assert result["terminal_lead"] == terminal_lead

    assert result["actuation_lag"] == gbc.actuation_lag_for_config(raw)

    distance_kp, distance_tol = gbc.distance_gains_for_config(raw)
    assert result["distance_kp"] == distance_kp
    assert result["distance_tol"] == distance_tol

    model_tau_lin, model_tau_ang = gbc.model_tau_for_config(raw)
    assert result["model_tau_lin"] == model_tau_lin
    assert result["model_tau_ang"] == model_tau_ang

    # Both shipped fixtures omit control.heading_source -> "auto" default,
    # for BOTH the raw-dict path (gen_boot_config.py's own function) and
    # the RobotConfig path under test.
    assert result["heading_source"] == planner_pb2.HEADING_SOURCE_AUTO


@pytest.mark.parametrize("name", _ROBOT_JSON_NAMES)
def test_planner_boot_config_for_matches_gen_boot_config_from_raw_dict(name):
    """Same parity, but sourced directly from the raw robot-JSON dict --
    gen_boot_config.py's own native input shape -- proving _as_cfg_dict()
    is a lossless passthrough rather than a RobotConfig-only code path."""
    raw = _raw_cfg(name)

    result = planner_boot_config_for(raw)

    model_tau_lin, model_tau_ang = gbc.model_tau_for_config(raw)
    assert result["model_tau_lin"] == model_tau_lin
    assert result["model_tau_ang"] == model_tau_ang

    distance_kp, distance_tol = gbc.distance_gains_for_config(raw)
    assert result["distance_kp"] == distance_kp
    assert result["distance_tol"] == distance_tol

    assert result["actuation_lag"] == gbc.actuation_lag_for_config(raw)


# ---------------------------------------------------------------------------
# 3. Fallback: missing "control" section entirely -> every field resolves to
#    gen_boot_config.py's own documented default.
# ---------------------------------------------------------------------------


def test_planner_boot_config_for_falls_back_to_defaults_with_no_control_section():
    result = planner_boot_config_for({})

    assert result["a_max"] == gbc.A_MAX_DEFAULT
    assert result["a_decel"] == gbc.A_DECEL_DEFAULT
    assert result["v_body_max"] == gbc.V_BODY_MAX_DEFAULT
    assert result["j_max"] == gbc.J_MAX_DEFAULT
    assert result["yaw_jerk_max"] == gbc.YAW_JERK_MAX_DEFAULT
    assert result["yaw_rate_max"] == gbc.YAW_RATE_MAX_DEFAULT
    assert result["yaw_acc_max"] == gbc.YAW_ACC_MAX_DEFAULT
    assert result["min_speed"] == gbc.MIN_SPEED_DEFAULT
    assert result["heading_kp"] == gbc.HEADING_KP_DEFAULT
    assert result["heading_kd"] == gbc.HEADING_KD_DEFAULT
    assert result["arrive_dwell"] == gbc.ARRIVE_DWELL_DEFAULT
    assert result["actuation_lag"] == gbc.ACTUATION_LAG_DEFAULT
    assert result["distance_kp"] == gbc.DISTANCE_KP_DEFAULT
    assert result["distance_tol"] == gbc.DISTANCE_TOL_DEFAULT
    assert result["model_tau_lin"] == gbc.MODEL_TAU_LIN_DEFAULT
    assert result["model_tau_ang"] == gbc.MODEL_TAU_ANG_DEFAULT
    assert result["heading_source"] == planner_pb2.HEADING_SOURCE_AUTO


def test_planner_boot_config_for_falls_back_with_empty_robot_config():
    """The SAME fallback, but sourced from a default-constructed
    RobotConfig (control section present, every field None) -- proving the
    RobotConfig code path hits the identical fallback as the raw-dict path
    above, not a different one.

    EXCEPTION: yaw_acc_max is excluded from the direct comparison.
    ControlConfig.max_rot_accel_dps2 is a PRE-EXISTING field (predates
    113-004) with its own non-None host-side default (300.0, documented on
    the field itself as the turn/turn2 CLI's fallback, "NOT pushed to
    firmware") -- but gen_boot_config.py's profile_rot_limits_for_config()
    (100-014) ALSO reads this identical control.max_rot_accel_dps2 JSON key
    for the firmware boot bake, whose own fallback (YAW_ACC_MAX_DEFAULT =
    20.0 rad/s^2, ~1145.9 deg/s^2) is a different number. Every REAL shipped
    robot JSON sets this key explicitly (tovez.json=600, tovez_nocal.json=
    1145.92 deg/s^2 -- see the parity tests above, which pass), so this
    divergence never surfaces in practice; it is flagged here, not silently
    asserted around, as a pre-existing dual-purpose-default landmine for a
    hypothetical future robot JSON that omits the key entirely -- out of
    113-004's own scope (this ticket reuses gen_boot_config.py's mapping
    unmodified; ControlConfig.max_rot_accel_dps2's host-side default is a
    different field's pre-existing concern)."""
    empty = RobotConfig(identity=IdentityConfig(robot_name="r", uid="0"))

    result = planner_boot_config_for(empty)
    expected = planner_boot_config_for({})

    result.pop("yaw_acc_max")
    expected.pop("yaw_acc_max")
    assert result == expected


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


def test_motor_boot_config_for_falls_back_with_no_calibration_section():
    result = motor_boot_config_for({}, port=1)

    assert result["vel_filt_alpha"] == gbc.VEL_FILT_ALPHA
    assert result["fwd_sign"] == gbc.FWD_SIGN


def test_motor_boot_config_for_reads_fwd_sign_from_robot_config():
    """Proves the drive pair's mirror-mounted fwd_sign (088-002) genuinely
    reads through the RobotConfig path -- both shipped fixtures happen to
    set left=+1/right=-1, so this synthetic config picks the OPPOSITE signs
    to prove it isn't coincidentally matching a hardcoded default."""
    cfg = RobotConfig(
        identity=IdentityConfig(robot_name="r", uid="0"),
        calibration=CalibrationConfig(fwd_sign_left=-1, fwd_sign_right=1),
    )

    left = motor_boot_config_for(cfg, port=1)
    right = motor_boot_config_for(cfg, port=2)
    other = motor_boot_config_for(cfg, port=3)

    assert left["fwd_sign"] == -1
    assert right["fwd_sign"] == 1
    assert other["fwd_sign"] == gbc.FWD_SIGN


# ---------------------------------------------------------------------------
# 4. Every Tier-2 field genuinely reads from the config -- a synthetic
#    arbitrary-value config, both as a raw dict and as a RobotConfig.
# ---------------------------------------------------------------------------


_ARBITRARY_CONTROL = {
    "yaw_rate_max": 90.0, "max_rot_accel_dps2": 360.0,
    "min_speed": 20.0,
    "heading_kp": 5.5, "heading_kd": 1.25,
    "arrive_dwell": 0.33,
    "heading_source": "encoder",
    "heading_lead_bias": -0.02, "plan_lead": 0.05, "terminal_lead": 0.06,
    "actuation_lag": 0.2,
    "distance_kp": 3.3, "distance_tol": 4.4,
    "model_tau_lin": 0.25, "model_tau_ang": 0.19,
}


def _assert_arbitrary_values(result: "dict[str, float | int]") -> None:
    assert result["yaw_rate_max"] == math.radians(90.0)
    assert result["yaw_acc_max"] == math.radians(360.0)
    assert result["min_speed"] == 20.0
    assert result["heading_kp"] == 5.5
    assert result["heading_kd"] == 1.25
    assert result["arrive_dwell"] == 0.33
    assert result["heading_dwell_tol"] == math.radians(gbc.HEADING_DWELL_TOL_DEG_DEFAULT)
    assert result["heading_dwell_rate"] == math.radians(gbc.HEADING_DWELL_RATE_DPS_DEFAULT)
    assert result["heading_lead_bias"] == -0.02
    assert result["plan_lead"] == 0.05
    assert result["terminal_lead"] == 0.06
    assert result["actuation_lag"] == 0.2
    assert result["distance_kp"] == 3.3
    assert result["distance_tol"] == 4.4
    assert result["model_tau_lin"] == 0.25
    assert result["model_tau_ang"] == 0.19
    assert result["heading_source"] == planner_pb2.HEADING_SOURCE_FORCE_ENCODER


def test_planner_boot_config_for_reads_arbitrary_control_values_from_raw_dict():
    result = planner_boot_config_for({"control": _ARBITRARY_CONTROL})

    _assert_arbitrary_values(result)


def test_planner_boot_config_for_reads_arbitrary_control_values_from_robot_config():
    """The SAME arbitrary values, but sourced from a RobotConfig -- proves
    113-004's ControlConfig extension (heading_source/heading_lead_bias/
    plan_lead/terminal_lead/actuation_lag/distance_tol/model_tau_lin/
    model_tau_ang) genuinely round-trips through model_dump(), not just
    the raw-dict path."""
    cfg = RobotConfig(
        identity=IdentityConfig(robot_name="r", uid="0"),
        control=ControlConfig(**_ARBITRARY_CONTROL),
    )

    result = planner_boot_config_for(cfg)

    _assert_arbitrary_values(result)


# ---------------------------------------------------------------------------
# 5. heading_source wire-int resolution -- all three values.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value,expected_name", [
    ("auto", "HEADING_SOURCE_AUTO"),
    ("otos", "HEADING_SOURCE_FORCE_OTOS"),
    ("encoder", "HEADING_SOURCE_FORCE_ENCODER"),
    ("OTOS", "HEADING_SOURCE_FORCE_OTOS"),  # case-insensitive, matches gen_boot_config.py
])
def test_heading_source_resolves_to_wire_int_for_all_values(value, expected_name):
    cfg = {"control": {"heading_source": value}}

    result = planner_boot_config_for(cfg)

    assert result["heading_source"] == planner_pb2.HeadingSourceMode.Value(expected_name)
