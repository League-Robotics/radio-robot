"""robot_radio.config — per-robot configuration subpackage.

Re-exports all public names from robot_config so callers can use either:
    from robot_radio.config import get_robot_config, RobotConfig
or the fully-qualified form:
    from robot_radio.config.robot_config import get_robot_config, RobotConfig
"""

from robot_radio.config.robot_config import (
    ConnectionConfig,
    EncodersConfig,
    GeometryConfig,
    GripperConfig,
    IdentityConfig,
    OffsetXY,
    RobotConfig,
    VisionConfig,
    WheelsConfig,
    _reset_robot_config,
    get_robot_config,
    load_robot_config,
)

# 132-020: CalibrationConfig/ControlConfig/(old-shape) DriveConfig/
# OffsetXYYaw are gone -- their fields now live on robot_config.py's
# generated-shape groups (Geometry/Motors/Drive/WheelControl/Planner/Otos/
# Estimator), not as standalone hand-written classes. See
# robot_config.py's own module docstring.

__all__ = [
    "ConnectionConfig",
    "EncodersConfig",
    "GeometryConfig",
    "GripperConfig",
    "IdentityConfig",
    "OffsetXY",
    "RobotConfig",
    "VisionConfig",
    "WheelsConfig",
    "_reset_robot_config",
    "get_robot_config",
    "load_robot_config",
]
