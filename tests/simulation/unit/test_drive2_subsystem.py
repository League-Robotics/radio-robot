"""
test_drive2_subsystem.py — Drive2 subsystem isolation tests (ticket 057-004).

Exercises the subsystems::Drive2 facade via C-ABI shims in
tests/_infra/sim/drive2_api.cpp, loaded via ctypes.  Tests construct ONLY
the Drive2 subsystem on SimHardware devices (no full Robot/CommandProcessor),
apply commands via the direct apply path, tick the two-phase contract, and
assert on state.

Four tests per the ticket acceptance criteria:
  1. test_twist_advances_pose       — twist (vx=200) → 20 ticks → fused x > 0.
  2. test_vy_reject_on_differential — twist (vy=50) on differential → holonomic=0,
                                      no x-motion (pose does not advance laterally).
  3. test_setpose_reanchor          — SetPose(50, 50, 0.5) → tickUpdate+tickAction →
                                      state fused x≈50, y≈50, h≈0.5.
  4. test_neutral_brake             — apply NEUTRAL(BRAKE) → tick once →
                                      state.connected still true; no crash.
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
    """Load firmware_host and configure drive2_api shim signatures."""
    lib = ctypes.CDLL(str(LIB_PATH))

    # Lifecycle
    lib.drive2_api_create.restype  = ctypes.c_void_p
    lib.drive2_api_create.argtypes = []

    lib.drive2_api_destroy.restype  = None
    lib.drive2_api_destroy.argtypes = [ctypes.c_void_p]

    # Commands
    lib.drive2_api_apply_twist.restype  = None
    lib.drive2_api_apply_twist.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ]

    lib.drive2_api_apply_neutral_brake.restype  = None
    lib.drive2_api_apply_neutral_brake.argtypes = [ctypes.c_void_p]

    lib.drive2_api_apply_setpose.restype  = None
    lib.drive2_api_apply_setpose.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ]

    # Tick
    lib.drive2_api_tick_update.restype  = None
    lib.drive2_api_tick_update.argtypes = [ctypes.c_void_p, ctypes.c_uint32]

    lib.drive2_api_tick_action.restype  = None
    lib.drive2_api_tick_action.argtypes = [ctypes.c_void_p, ctypes.c_uint32]

    # State reads — fused pose
    lib.drive2_api_get_fused_x.restype  = ctypes.c_float
    lib.drive2_api_get_fused_x.argtypes = [ctypes.c_void_p]

    lib.drive2_api_get_fused_y.restype  = ctypes.c_float
    lib.drive2_api_get_fused_y.argtypes = [ctypes.c_void_p]

    lib.drive2_api_get_fused_h.restype  = ctypes.c_float
    lib.drive2_api_get_fused_h.argtypes = [ctypes.c_void_p]

    lib.drive2_api_get_connected.restype  = ctypes.c_int
    lib.drive2_api_get_connected.argtypes = [ctypes.c_void_p]

    # Capabilities
    lib.drive2_api_capabilities_holonomic.restype  = ctypes.c_int
    lib.drive2_api_capabilities_holonomic.argtypes = [ctypes.c_void_p]

    return lib


# ---------------------------------------------------------------------------
# Fixture: shared library handle.
# build_lib (session-scoped autouse) in conftest.py ensures the library is
# built before any test in this session runs.
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def dlib(build_lib):  # noqa: ARG001
    """Return a ctypes handle to the firmware_host library with drive2 shims."""
    return _load_lib()


# ---------------------------------------------------------------------------
# Helper: managed handle so every test gets a fresh Drive2Handle.
# ---------------------------------------------------------------------------

class Drive2Ctx:
    """Thin RAII wrapper around drive2_api_create / drive2_api_destroy."""

    def __init__(self, lib: ctypes.CDLL) -> None:
        self._lib = lib
        self._h = lib.drive2_api_create()
        if not self._h:
            raise RuntimeError("drive2_api_create() returned NULL")
        self._now = 0

    def apply_twist(self, vx: float, vy: float = 0.0, omega: float = 0.0) -> "Drive2Ctx":
        self._lib.drive2_api_apply_twist(self._h,
                                          ctypes.c_float(vx),
                                          ctypes.c_float(vy),
                                          ctypes.c_float(omega))
        return self

    def apply_neutral_brake(self) -> "Drive2Ctx":
        self._lib.drive2_api_apply_neutral_brake(self._h)
        return self

    def apply_setpose(self, x: float, y: float, h: float) -> "Drive2Ctx":
        self._lib.drive2_api_apply_setpose(self._h,
                                            ctypes.c_float(x),
                                            ctypes.c_float(y),
                                            ctypes.c_float(h))
        return self

    def tick(self, dt_ms: int = 20) -> "Drive2Ctx":
        """One combined tickUpdate + tickAction step, advancing by dt_ms."""
        self._now += dt_ms
        self._lib.drive2_api_tick_update(self._h, self._now)
        self._lib.drive2_api_tick_action(self._h, self._now)
        return self

    def fused_x(self) -> float:
        return float(self._lib.drive2_api_get_fused_x(self._h))

    def fused_y(self) -> float:
        return float(self._lib.drive2_api_get_fused_y(self._h))

    def fused_h(self) -> float:
        return float(self._lib.drive2_api_get_fused_h(self._h))

    def connected(self) -> bool:
        return bool(self._lib.drive2_api_get_connected(self._h))

    def holonomic(self) -> bool:
        return bool(self._lib.drive2_api_capabilities_holonomic(self._h))

    def destroy(self) -> None:
        if self._h:
            self._lib.drive2_api_destroy(self._h)
            self._h = None

    def __enter__(self) -> "Drive2Ctx":
        return self

    def __exit__(self, *_) -> None:
        self.destroy()


# ---------------------------------------------------------------------------
# Test 1: test_twist_advances_pose
#
# Apply a forward twist command (vx=200 mm/s, vy=0, omega=0) and tick 20
# times.  The EKF encoder-predict path should integrate the motion so that
# the fused x position ends up > 0 (robot moved forward).
#
# This is a parity smoke test: the encoder dead-reckoning in tickUpdate
# integrates wheel deltas from positionMm() through addOdometryObservation
# (EKF predict).  The BVC profiler ramps up to the target speed, so forward
# motion accumulates over 20 ticks × ~20 ms = ~400 ms.
# ---------------------------------------------------------------------------

def test_twist_advances_pose(dlib):
    """Forward twist (vx=200) for 20 ticks → fused x > 0 (robot moved forward)."""
    with Drive2Ctx(dlib) as d:
        # Apply a forward twist command (staged only, no hardware yet).
        d.apply_twist(vx=200.0, vy=0.0, omega=0.0)

        # Tick 20 times: tickAction applies the command → BVC ramps speed →
        # MC drives motors → SimHardware integrates → tickUpdate reads encoders
        # → EKF predict accumulates x.
        for _ in range(20):
            d.tick(dt_ms=20)
            # Re-apply the twist each tick (apply stages; each tickAction consumes it).
            d.apply_twist(vx=200.0, vy=0.0, omega=0.0)

        x = d.fused_x()
        assert x > 0.0, (
            f"Expected fused x > 0 after 20 forward-twist ticks, got x={x:.3f} mm"
        )


# ---------------------------------------------------------------------------
# Test 2: test_vy_reject_on_differential
#
# On a differential build, capabilities().holonomic == false.
# Apply a pure lateral twist (vx=0, vy=50, omega=0): Drive2 must reject it
# (zero actuation output).  The test verifies:
#   (a) holonomic flag is 0 (differential build),
#   (b) tickAction does not crash (no exception path),
#   (c) fused x remains at 0 after the tick (no forward motion from a lateral cmd).
# ---------------------------------------------------------------------------

def test_vy_reject_on_differential(dlib):
    """Differential build: vy-only twist is rejected — no motion, no crash."""
    with Drive2Ctx(dlib) as d:
        assert not d.holonomic(), (
            "Expected holonomic=False on the differential (tovez) build"
        )

        # Apply a pure-lateral twist on a non-holonomic drivetrain.
        d.apply_twist(vx=0.0, vy=50.0, omega=0.0)
        # tick must not crash; tickAction should reject and zero the motors.
        d.tick(dt_ms=20)

        x = d.fused_x()
        # No forward motion expected: EKF predicts zero displacement.
        assert abs(x) < 1.0, (
            f"Expected near-zero x displacement after vy-reject, got x={x:.3f} mm"
        )


# ---------------------------------------------------------------------------
# Test 3: test_setpose_reanchor
#
# Apply SetPose(50.0, 50.0, 0.5) then one tickUpdate + tickAction.
# The resetPose() call re-anchors the fused estimate so state().fused.pose
# reads back approximately (50, 50, 0.5).
#
# The EKF is reset to the given pose; the subsequent addOdometryObservation
# with zero encoder delta should leave it essentially at (50, 50, 0.5).
# Tolerance: ±2 mm for position, ±0.1 rad for heading (floating-point + EKF
# process noise over one tick).
# ---------------------------------------------------------------------------

def test_setpose_reanchor(dlib):
    """SetPose(50, 50, 0.5) + one tick → fused x≈50, y≈50, h≈0.5."""
    with Drive2Ctx(dlib) as d:
        d.apply_setpose(x=50.0, y=50.0, h=0.5)
        d.tick(dt_ms=20)

        x = d.fused_x()
        y = d.fused_y()
        h = d.fused_h()

        assert abs(x - 50.0) < 2.0, (
            f"SetPose x-reanchor failed: expected ~50 mm, got {x:.3f} mm"
        )
        assert abs(y - 50.0) < 2.0, (
            f"SetPose y-reanchor failed: expected ~50 mm, got {y:.3f} mm"
        )
        assert abs(h - 0.5) < 0.1, (
            f"SetPose h-reanchor failed: expected ~0.5 rad, got {h:.4f} rad"
        )


# ---------------------------------------------------------------------------
# Test 4: test_neutral_brake
#
# Apply NEUTRAL(BRAKE) then tick once.  Verifies:
#   (a) No crash / no exception (the NEUTRAL path executes cleanly).
#   (b) state().connected is still true after the tick.
#
# The deeper motor-output assertion (pwm ≈ 0) is not directly readable through
# the current state() API (vel_mms is the ACTUAL velocity from the encoder, not
# the target); the connected flag suffices as a liveness check for this ticket.
# The EKF-fusion test (ticket 005) will verify more detailed output behavior.
# ---------------------------------------------------------------------------

def test_neutral_brake(dlib):
    """NEUTRAL(BRAKE) + one tick → no crash; state.connected still true."""
    with Drive2Ctx(dlib) as d:
        d.apply_neutral_brake()
        d.tick(dt_ms=20)

        assert d.connected(), (
            "state.connected must be true after a NEUTRAL(BRAKE) tick"
        )
