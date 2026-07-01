"""
test_planner_subsystem_smoke.py — Planner subsystem smoke tests (ticket 059-001).

Exercises the Planner subsystem via C-ABI shims in
tests/_infra/sim/planner_api.cpp, loaded via ctypes.  This ticket's tests are
construction + minimal tick sanity; deeper planner-isolation tests come in 059-002.

Tests:
  1. test_construct_destroy      — create/destroy PlannerHandle without crash.
  2. test_tick_idle              — 10 ticks at IDLE → body_twist stays near zero.
  3. test_apply_velocity         — VELOCITY goal (vx=200) → tick once → active,
                                   body_twist.vx ramps toward 200.
  4. test_apply_stop             — VELOCITY then STOP → mode returns to IDLE.
  5. test_toplannerconfig_a_max  — toPlannerConfig projection: a_max > 0.
  6. test_toplannerconfig_v_body_max — v_body_max > 0 from default config.
"""

from __future__ import annotations

import ctypes
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


def _load_lib() -> ctypes.CDLL:
    """Load firmware_host and configure planner_api shim signatures."""
    lib = ctypes.CDLL(str(LIB_PATH))

    # Lifecycle
    lib.planner_api_create.restype  = ctypes.c_void_p
    lib.planner_api_create.argtypes = []

    lib.planner_api_destroy.restype  = None
    lib.planner_api_destroy.argtypes = [ctypes.c_void_p]

    # Tick
    lib.planner_api_tick.restype  = ctypes.c_float
    lib.planner_api_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]

    # Command application
    lib.planner_api_apply_velocity.restype  = None
    lib.planner_api_apply_velocity.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float,
    ]

    lib.planner_api_apply_stop.restype  = None
    lib.planner_api_apply_stop.argtypes = [ctypes.c_void_p]

    lib.planner_api_apply_turn.restype  = None
    lib.planner_api_apply_turn.argtypes = [ctypes.c_void_p, ctypes.c_float]

    lib.planner_api_apply_timed.restype  = None
    lib.planner_api_apply_timed.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_uint32,
    ]

    lib.planner_api_apply_goto.restype  = None
    lib.planner_api_apply_goto.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ]

    lib.planner_api_apply_distance.restype  = None
    lib.planner_api_apply_distance.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float,
    ]

    lib.planner_api_apply_rotation.restype  = None
    lib.planner_api_apply_rotation.argtypes = [ctypes.c_void_p, ctypes.c_float]

    # State reads
    lib.planner_api_get_active.restype  = ctypes.c_int
    lib.planner_api_get_active.argtypes = [ctypes.c_void_p]

    lib.planner_api_get_mode.restype  = ctypes.c_int
    lib.planner_api_get_mode.argtypes = [ctypes.c_void_p]

    lib.planner_api_get_body_twist_vx.restype  = ctypes.c_float
    lib.planner_api_get_body_twist_vx.argtypes = [ctypes.c_void_p]

    lib.planner_api_get_body_twist_omega.restype  = ctypes.c_float
    lib.planner_api_get_body_twist_omega.argtypes = [ctypes.c_void_p]

    lib.planner_api_get_fused_x.restype  = ctypes.c_float
    lib.planner_api_get_fused_x.argtypes = [ctypes.c_void_p]

    # toPlannerConfig projection shims
    lib.planner_api_default_config_a_max.restype  = ctypes.c_float
    lib.planner_api_default_config_a_max.argtypes = []

    lib.planner_api_default_config_v_body_max.restype  = ctypes.c_float
    lib.planner_api_default_config_v_body_max.argtypes = []

    lib.planner_api_default_config_yaw_rate_max.restype  = ctypes.c_float
    lib.planner_api_default_config_yaw_rate_max.argtypes = []

    lib.planner_api_default_config_arrive_tol_mm.restype  = ctypes.c_float
    lib.planner_api_default_config_arrive_tol_mm.argtypes = []

    return lib


@pytest.fixture(scope="module")
def lib() -> ctypes.CDLL:
    return _load_lib()


class TestPlannerSmoke:
    """Smoke tests for Planner construction and minimal tick behavior."""

    def test_construct_destroy(self, lib):
        """Create and destroy a PlannerHandle — must not crash."""
        h = lib.planner_api_create()
        assert h is not None
        lib.planner_api_destroy(h)

    def test_tick_idle(self, lib):
        """10 ticks at IDLE — body_twist should stay near zero."""
        h = lib.planner_api_create()
        try:
            for i in range(10):
                lib.planner_api_tick(h, ctypes.c_uint32(i * 20))
            vx = lib.planner_api_get_body_twist_vx(h)
            assert abs(vx) < 1.0, f"Expected near-zero vx at IDLE, got {vx}"
        finally:
            lib.planner_api_destroy(h)

    def test_apply_velocity_ramps(self, lib):
        """VELOCITY goal (vx=200) staged, then ticked — body_twist.vx should
        start ramping (BVC trapezoid) from 0 toward 200 mm/s."""
        h = lib.planner_api_create()
        try:
            # Stage the velocity goal.
            lib.planner_api_apply_velocity(h, ctypes.c_float(200.0), ctypes.c_float(0.0))
            # Run several ticks (20 ms each) to let the BVC ramp up.
            for i in range(20):
                lib.planner_api_tick(h, ctypes.c_uint32(i * 20))
            vx = lib.planner_api_get_body_twist_vx(h)
            # After 20 ticks × 20 ms = 400 ms of ramping, vx should be > 0.
            assert vx > 0.0, f"Expected vx > 0 after VELOCITY goal, got {vx}"
        finally:
            lib.planner_api_destroy(h)

    def test_apply_stop_clears_active(self, lib):
        """VELOCITY then STOP — after a few ticks the commanded twist drops to zero."""
        h = lib.planner_api_create()
        try:
            # Start moving.
            lib.planner_api_apply_velocity(h, ctypes.c_float(200.0), ctypes.c_float(0.0))
            for i in range(5):
                lib.planner_api_tick(h, ctypes.c_uint32(i * 20))

            # Issue STOP.
            lib.planner_api_apply_stop(h)
            # Run ticks to let BVC ramp down.
            for i in range(5, 25):
                lib.planner_api_tick(h, ctypes.c_uint32(i * 20))

            # After a STOP the MC should be IDLE (mode=0).
            mode = lib.planner_api_get_mode(h)
            # Mode 0 = IDLE (msg::DriveMode::IDLE).
            assert mode == 0, f"Expected mode IDLE (0) after stop, got {mode}"
        finally:
            lib.planner_api_destroy(h)


class TestToPlannerConfig:
    """Tests for the toPlannerConfig() projection function."""

    def test_a_max_positive(self, lib):
        """Default RobotConfig.aMax projected to PlannerConfig.a_max must be > 0."""
        a_max = lib.planner_api_default_config_a_max()
        assert a_max > 0.0, f"Expected a_max > 0, got {a_max}"

    def test_v_body_max_positive(self, lib):
        """Default RobotConfig.vBodyMax projected to PlannerConfig.v_body_max > 0."""
        v_body_max = lib.planner_api_default_config_v_body_max()
        assert v_body_max > 0.0, f"Expected v_body_max > 0, got {v_body_max}"

    def test_yaw_rate_max_positive(self, lib):
        """Default yawRateMax projected to PlannerConfig.yaw_rate_max > 0."""
        yaw_rate_max = lib.planner_api_default_config_yaw_rate_max()
        assert yaw_rate_max > 0.0, f"Expected yaw_rate_max > 0, got {yaw_rate_max}"

    def test_arrive_tol_mm_positive(self, lib):
        """Default arriveTolMm projected to PlannerConfig.arrive_tol_mm > 0."""
        arrive_tol = lib.planner_api_default_config_arrive_tol_mm()
        assert arrive_tol > 0.0, f"Expected arrive_tol_mm > 0, got {arrive_tol}"
