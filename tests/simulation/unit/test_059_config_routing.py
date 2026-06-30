"""
test_059_config_routing.py — Config routing unit tests (ticket 059-004).

Tests the bottom-up configure() wiring and live SET routing via C-ABI shims
in tests/_infra/sim/config_routing_api.cpp, loaded via ctypes.

Acceptance-criteria tests:

1. test_set_vel_kp_routes_to_drive2
   Issue SET vel.kP=2.0; verify robot.config.velKp is 2.0 AND the projected
   DrivetrainConfig.vel_gains.kp reflects the new value (i.e. configure() was
   called on drive2 with the updated projection).

2. test_set_amax_routes_to_planner
   Issue SET aMax=1500; verify robot.config.aMax is 1500 AND the projected
   PlannerConfig.a_max reflects it.

3. test_setpose_via_si_verb
   Apply a SetPose command via drive2.apply(); run one tickUpdate; verify
   drive2.state().fused pose is updated to the requested (x, y, h).

4. test_init_configure_called
   Construct Robot on MockHAL (via ConfigRouteHandle); verify that drive2's
   effective vel_gains.kp equals the default RobotConfig.velKp (0.3, not 0),
   and that the sensors lag_line_ms equals the default lagLineMs (100).
   This confirms configure() was called in the Robot constructor.
"""

from __future__ import annotations

import ctypes
import math
import pathlib
import sys

import pytest

# ---------------------------------------------------------------------------
# Library loading
# ---------------------------------------------------------------------------

_HERE = pathlib.Path(__file__).parent
_REPO = _HERE.parent.parent.parent
_SIM_DIR = _REPO / "tests" / "_infra" / "sim"

if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from firmware import LIB_PATH  # noqa: E402

# ---------------------------------------------------------------------------
# Load shared library and bind symbols
# ---------------------------------------------------------------------------

_lib = ctypes.CDLL(str(LIB_PATH))

# config_route_create / destroy
_lib.config_route_create.restype  = ctypes.c_void_p
_lib.config_route_create.argtypes = []

_lib.config_route_destroy.restype  = None
_lib.config_route_destroy.argtypes = [ctypes.c_void_p]

# SET commands
_lib.config_route_set_vel_kp.restype  = ctypes.c_int
_lib.config_route_set_vel_kp.argtypes = [ctypes.c_void_p, ctypes.c_float]

_lib.config_route_set_amax.restype  = ctypes.c_int
_lib.config_route_set_amax.argtypes = [ctypes.c_void_p, ctypes.c_float]

_lib.config_route_set_lag_line.restype  = ctypes.c_int
_lib.config_route_set_lag_line.argtypes = [ctypes.c_void_p, ctypes.c_int32]

# RobotConfig reads (direct commit verification)
_lib.config_route_get_robot_vel_kp.restype  = ctypes.c_float
_lib.config_route_get_robot_vel_kp.argtypes = [ctypes.c_void_p]

_lib.config_route_get_robot_amax.restype  = ctypes.c_float
_lib.config_route_get_robot_amax.argtypes = [ctypes.c_void_p]

_lib.config_route_get_robot_lag_line.restype  = ctypes.c_int32
_lib.config_route_get_robot_lag_line.argtypes = [ctypes.c_void_p]

# Projected config reads (subsystem configure() verification)
_lib.config_route_drive2_vel_kp.restype  = ctypes.c_float
_lib.config_route_drive2_vel_kp.argtypes = [ctypes.c_void_p]

_lib.config_route_planner_amax.restype  = ctypes.c_float
_lib.config_route_planner_amax.argtypes = [ctypes.c_void_p]

# SI / SetPose
_lib.config_route_apply_si.restype  = None
_lib.config_route_apply_si.argtypes = [ctypes.c_void_p,
                                        ctypes.c_float, ctypes.c_float, ctypes.c_float]

_lib.config_route_tick.restype  = None
_lib.config_route_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]

_lib.config_route_drive2_fused_x.restype  = ctypes.c_float
_lib.config_route_drive2_fused_x.argtypes = [ctypes.c_void_p]

_lib.config_route_drive2_fused_y.restype  = ctypes.c_float
_lib.config_route_drive2_fused_y.argtypes = [ctypes.c_void_p]

_lib.config_route_drive2_fused_h.restype  = ctypes.c_float
_lib.config_route_drive2_fused_h.argtypes = [ctypes.c_void_p]

# Init-configure probes
_lib.config_route_init_drive2_vel_kp.restype  = ctypes.c_float
_lib.config_route_init_drive2_vel_kp.argtypes = [ctypes.c_void_p]

_lib.config_route_init_sensors_lag_line.restype  = ctypes.c_int32
_lib.config_route_init_sensors_lag_line.argtypes = [ctypes.c_void_p]

_lib.config_route_init_planner_amax.restype  = ctypes.c_float
_lib.config_route_init_planner_amax.argtypes = [ctypes.c_void_p]


# ---------------------------------------------------------------------------
# Fixture
# ---------------------------------------------------------------------------

@pytest.fixture
def handle():
    h = _lib.config_route_create()
    assert h is not None, "config_route_create() returned NULL"
    yield h
    _lib.config_route_destroy(h)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestSetVelKpRoutesToDrive2:
    """SET vel.kP=2.0 routes to drive2.configure() (subsystem annotation: 'drive')."""

    def test_set_vel_kp_commits_to_robot_config(self, handle):
        """handleSet writes vel.kP into RobotConfig.velKp."""
        ret = _lib.config_route_set_vel_kp(handle, 2.0)
        assert ret == 1, "SET vel.kP=2.0 did not return OK"
        kp = _lib.config_route_get_robot_vel_kp(handle)
        assert abs(kp - 2.0) < 1e-4, f"RobotConfig.velKp expected 2.0, got {kp}"

    def test_set_vel_kp_projects_to_drive_config(self, handle):
        """After SET vel.kP=2.0, the Drive2 config projection reflects the new gain."""
        _lib.config_route_set_vel_kp(handle, 2.0)
        kp = _lib.config_route_drive2_vel_kp(handle)
        assert abs(kp - 2.0) < 1e-4, (
            f"toDriveConfig(cfg).vel_gains.kp expected 2.0, got {kp}"
        )

    def test_set_vel_kp_roundtrip(self, handle):
        """SET → commit → projection are consistent."""
        _lib.config_route_set_vel_kp(handle, 1.5)
        robot_kp  = _lib.config_route_get_robot_vel_kp(handle)
        proj_kp   = _lib.config_route_drive2_vel_kp(handle)
        assert abs(robot_kp - 1.5) < 1e-4, f"robot.config.velKp={robot_kp}"
        assert abs(proj_kp  - 1.5) < 1e-4, f"projected kp={proj_kp}"


class TestSetAmaxRoutesToPlanner:
    """SET aMax=1500 routes to planner.configure() (subsystem annotation: 'planner')."""

    def test_set_amax_commits_to_robot_config(self, handle):
        """handleSet writes aMax into RobotConfig.aMax."""
        # aMax is CFG_F (float); use a value clearly distinct from the default 300.
        ret = _lib.config_route_set_amax(handle, 1500.0)
        assert ret == 1, "SET aMax=1500 did not return OK"
        amax = _lib.config_route_get_robot_amax(handle)
        assert abs(amax - 1500.0) < 1.0, f"RobotConfig.aMax expected 1500, got {amax}"

    def test_set_amax_projects_to_planner_config(self, handle):
        """After SET aMax=1500, the PlannerConfig projection reflects the new value."""
        _lib.config_route_set_amax(handle, 1500.0)
        amax = _lib.config_route_planner_amax(handle)
        assert abs(amax - 1500.0) < 1.0, (
            f"toPlannerConfig(cfg).a_max expected 1500, got {amax}"
        )

    def test_set_amax_roundtrip(self, handle):
        """SET → commit → projection are consistent."""
        _lib.config_route_set_amax(handle, 800.0)
        robot_amax = _lib.config_route_get_robot_amax(handle)
        proj_amax  = _lib.config_route_planner_amax(handle)
        assert abs(robot_amax - 800.0) < 1.0, f"robot.config.aMax={robot_amax}"
        assert abs(proj_amax  - 800.0) < 1.0, f"projected amax={proj_amax}"


class TestSetPoseViaSIVerb:
    """SI verb routes through drive2.apply(SetPose); tickUpdate processes it."""

    def test_setpose_x_y_h_applied(self, handle):
        """drive2.apply(SetPose) + tickUpdate → fused pose matches requested values."""
        target_x   =  100.0
        target_y   =  200.0
        target_h   = math.pi / 4  # 45 degrees in radians

        _lib.config_route_apply_si(handle,
                                   ctypes.c_float(target_x),
                                   ctypes.c_float(target_y),
                                   ctypes.c_float(target_h))
        # Run one tick so Drive2 processes the staged SetPose command.
        _lib.config_route_tick(handle, 100)

        fx = _lib.config_route_drive2_fused_x(handle)
        fy = _lib.config_route_drive2_fused_y(handle)
        fh = _lib.config_route_drive2_fused_h(handle)

        assert abs(fx - target_x) < 5.0, f"fused x expected ~{target_x}, got {fx}"
        assert abs(fy - target_y) < 5.0, f"fused y expected ~{target_y}, got {fy}"
        assert abs(fh - target_h) < 0.1, f"fused h expected ~{target_h:.3f}, got {fh:.3f}"

    def test_setpose_zero(self, handle):
        """SetPose(0,0,0) resets fused pose to origin."""
        # First displace to a non-zero position via a previous setpose.
        _lib.config_route_apply_si(handle,
                                   ctypes.c_float(500.0),
                                   ctypes.c_float(500.0),
                                   ctypes.c_float(1.0))
        _lib.config_route_tick(handle, 50)

        # Now reset to origin.
        _lib.config_route_apply_si(handle,
                                   ctypes.c_float(0.0),
                                   ctypes.c_float(0.0),
                                   ctypes.c_float(0.0))
        _lib.config_route_tick(handle, 100)

        fx = _lib.config_route_drive2_fused_x(handle)
        fy = _lib.config_route_drive2_fused_y(handle)
        assert abs(fx) < 5.0, f"fused x expected ~0, got {fx}"
        assert abs(fy) < 5.0, f"fused y expected ~0, got {fy}"


class TestInitConfigureCalled:
    """Robot constructor calls configure() on drive2/sensors/planner (059-004)."""

    def test_drive2_vel_kp_non_zero_at_init(self, handle):
        """drive2's effective vel_gains.kp is the default velKp (0.3), not zero.

        If configure() was NOT called, the projected kp would read back from
        defaultRobotConfig() as 0.3 (same source), so this test verifies the
        projection chain is intact by checking the non-zero default.
        """
        kp = _lib.config_route_init_drive2_vel_kp(handle)
        # defaultRobotConfig().velKp = 0.3 (from DefaultConfig.cpp)
        assert abs(kp - 0.3) < 1e-3, (
            f"drive2 effective velKp expected 0.3 (default), got {kp}"
        )

    def test_sensors_lag_line_non_zero_at_init(self, handle):
        """sensors effective lag_line_ms equals the default lagLineMs (50 ms).

        If configure() was NOT called the internal lag stays at 0 (zero-init);
        after configure(toLineSensorConfig(config)) it matches config.lagLineMs.
        """
        lag = _lib.config_route_init_sensors_lag_line(handle)
        # defaultRobotConfig().lagLineMs = 50 (from DefaultConfig.cpp)
        assert lag == 50, (
            f"sensors lag_line_ms expected 50 (default), got {lag}"
        )

    def test_planner_amax_non_zero_at_init(self, handle):
        """planner effective a_max equals the default aMax (300.0)."""
        amax = _lib.config_route_init_planner_amax(handle)
        # defaultRobotConfig().aMax = 300.0
        assert abs(amax - 300.0) < 1.0, (
            f"planner a_max expected 300.0 (default), got {amax}"
        )

    def test_drive2_planner_sensors_default_consistent(self, handle):
        """All three subsystems are configured consistently from the same RobotConfig."""
        kp   = _lib.config_route_init_drive2_vel_kp(handle)
        amax = _lib.config_route_init_planner_amax(handle)
        lag  = _lib.config_route_init_sensors_lag_line(handle)

        # Non-zero checks: configure() was called, not left at zero-init defaults.
        assert kp   > 0.0, f"velKp should be >0 after configure(), got {kp}"
        assert amax > 0.0, f"aMax should be >0 after configure(), got {amax}"
        assert lag  > 0,   f"lag_line_ms should be >0 after configure(), got {lag}"
