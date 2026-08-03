"""src/tests/unit/test_robot_config_generated_shape.py -- 132-020 (wire
robot_config_generated.py's model into robot_config.py — keep behaviour,
adopt the generated shape).

Light smoke coverage per this ticket's own Acceptance Criteria: importable,
and ``RobotConfig`` constructible from a minimal in-memory dict shaped to
the GENERATED model's END-STATE consumer-grouped layout (identity/
connection/vision/geometry/motors/drive/wheel_control/planner/otos/
estimator) — NOT a real robot JSON, which is still in the OLD 13-section
shape until ticket 017's reshape (same posture ticket 016 states for its
own extra='forbid' work). See ``robot_config.py``'s own module docstring
for the full "generated struct SHAPE, hand-written baking BEHAVIOR" split
this composes.

Collected under ``src/tests/unit/`` — ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by
default.
"""

import pytest
from pydantic import ValidationError

from robot_radio.config.robot_config import GeometryConfig, RobotConfig, get_robot_config, list_robots
from robot_radio.config.robot_config_generated import (
    Connection,
    Drive,
    Estimator,
    Geometry,
    Identity,
    Motors,
    Otos,
    Planner,
    Vision,
    WheelControl,
)

_MINIMAL_IDENTITY = {"robot_name": "test-bot", "uid": "test-bot"}

# A minimal dict shaped to the GENERATED model's own end-state grouped
# layout (robot_config.proto / robot_config.schema.json), one representative
# field per robot-config group -- not a real robot JSON.
_GENERATED_SHAPE_DICT = {
    "identity": _MINIMAL_IDENTITY,
    "geometry": {"trackwidth": 150.0, "rotational_slip": 0.9},
    "motors": {
        "travel_calib_left": 0.5,
        "travel_calib_right": 0.5,
        "fwd_sign_left": 1,
        "fwd_sign_right": -1,
    },
    "drive": {"duty_per_speed_left": 0.01, "duty_per_speed_right": 0.01},
    "wheel_control": {"pid_kp": 1.0},
    "planner": {"v_max": 500.0},
    "otos": {"linear_scale": 1.0, "angular_scale": 1.0},
    "estimator": {"weight_heading_otos": 0.5},
}


def test_public_api_importable():
    """RobotConfig/get_robot_config/list_robots are importable from
    robot_config.py's public surface (ticket's own verification command)."""
    assert RobotConfig is not None
    assert callable(get_robot_config)
    assert callable(list_robots)


def test_group_classes_come_from_generated_module():
    """robot_config.py's per-group model classes ARE
    robot_config_generated.py's classes (or, for Geometry, a subclass of
    one) -- not independently hand-declared duplicates."""
    assert RobotConfig.model_fields["identity"].annotation is Identity
    assert RobotConfig.model_fields["connection"].annotation is Connection
    assert RobotConfig.model_fields["vision"].annotation is Vision
    assert RobotConfig.model_fields["motors"].annotation is Motors
    assert RobotConfig.model_fields["drive"].annotation is Drive
    assert RobotConfig.model_fields["wheel_control"].annotation is WheelControl
    assert RobotConfig.model_fields["planner"].annotation is Planner
    assert RobotConfig.model_fields["otos"].annotation is Otos
    assert RobotConfig.model_fields["estimator"].annotation is Estimator
    assert issubclass(GeometryConfig, Geometry)
    assert RobotConfig.model_fields["geometry"].annotation is GeometryConfig


def test_constructs_from_minimal_generated_shape_dict():
    """RobotConfig is constructible from a minimal in-memory dict shaped
    to the GENERATED model's layout (per Acceptance Criteria — not a real
    robot JSON)."""
    cfg = RobotConfig.model_validate(_GENERATED_SHAPE_DICT)

    assert cfg.robot_name == "test-bot"
    assert cfg.geometry.trackwidth == 150.0
    assert cfg.motors.travel_calib_left == 0.5
    assert cfg.motors.fwd_sign_right == -1
    assert cfg.drive.duty_per_speed_left == 0.01
    assert cfg.wheel_control.pid_kp == 1.0
    assert cfg.planner.v_max == 500.0
    assert cfg.otos_linear_scale == 1.0
    assert cfg.estimator.weight_heading_otos == 0.5


def test_constructs_from_generated_shape_json_round_trip():
    """The same minimal dict round-trips through JSON (model_validate_json),
    matching how load_robot_config() reads real files from disk."""
    import json

    cfg = RobotConfig.model_validate_json(json.dumps(_GENERATED_SHAPE_DICT))
    assert cfg.robot_name == "test-bot"
    assert cfg.geometry.rotational_slip == 0.9


@pytest.mark.parametrize("value", [0.0, 0.5, 0.75, 1.0])
def test_rotational_slip_domain_accepts_sentinel_and_calibrated_range(value):
    """rotational_slip's non-contiguous {0} u [0.5, 1.0] domain — the one
    cross-field validator not mechanically re-derivable from the schema
    (robot_config.proto's own header comment) — is preserved and
    reattached to GeometryConfig, where the field now lives."""
    cfg = RobotConfig.model_validate(
        {"identity": _MINIMAL_IDENTITY, "geometry": {"rotational_slip": value}}
    )
    assert cfg.geometry.rotational_slip == value


@pytest.mark.parametrize("value", [0.1, 0.49, -0.5, 1.5])
def test_rotational_slip_domain_rejects_outside_values(value):
    """Values strictly between 0 and 0.5, or outside [0, 1] entirely, are
    rejected -- the domain is {0} u [0.5, 1.0], not [0.0, 1.0]."""
    with pytest.raises(ValidationError):
        RobotConfig.model_validate(
            {"identity": _MINIMAL_IDENTITY, "geometry": {"rotational_slip": value}}
        )


def test_derived_field_computation_preserved():
    """mm_per_tick derived-field computation (wheels/encoders — no
    generated counterpart, stays hand-written) is preserved."""
    cfg = RobotConfig.model_validate(
        {
            "identity": _MINIMAL_IDENTITY,
            "encoders": {"has_encoders": True},
            "wheels": {"wheel_diameter_mm": 60.0, "ticks_per_rev": 360.0},
        }
    )
    assert cfg.mm_per_tick is not None
    assert cfg.mm_per_tick == pytest.approx(1.0 / (360.0 / (3.141592653589793 * 60.0)))
