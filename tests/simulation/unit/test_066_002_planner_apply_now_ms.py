"""
test_066_002_planner_apply_now_ms.py — Planner::apply() now_ms guard test
(ticket 066-002 / CR-11).

Background
----------
Planner::apply() used to hard-code `const uint32_t now = 0;` and pass that
into every begin*() call, which becomes MotionCommand::start()'s
MotionBaseline.t0Ms baseline. A PlannerCommand-path TIMED goal staged at a
realistic (nonzero) uptime would then see:

    elapsed = now_ms (at tick time) - t0Ms (== 0) == now_ms

so a TIME stop (StopCondition::evaluate, Kind::TIME) with even a short
duration threshold fires on the very first tick() after apply(), instead of
after the goal's actual duration has elapsed. This is currently unreachable
in production (BusDrain's PLANNER verb is a no-op placeholder), but is a
landmine for whoever wires it up next.

This test uses tests/_infra/sim/planner_api.cpp's C-ABI shims directly (the
059-001/059-002 pattern also used by test_059_bus_drain.py /
test_059_config_routing.py) to apply a TIMED goal with a realistic nonzero
now_ms and assert:

  1. The commanded motion actually runs for close to its full duration
     (measured via fused_x — a stop firing on the very next tick barely
     moves the robot; a stop firing after the full duration covers most of
     the expected distance).
  2. The command does eventually finish (the TIME stop is not simply
     inert) once now_ms has advanced past t0Ms + duration_ms.
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
    """Load firmware_host and configure the planner_api shim signatures used here."""
    lib = ctypes.CDLL(str(LIB_PATH))

    lib.planner_api_create.restype = ctypes.c_void_p
    lib.planner_api_create.argtypes = []

    lib.planner_api_destroy.restype = None
    lib.planner_api_destroy.argtypes = [ctypes.c_void_p]

    lib.planner_api_tick.restype = ctypes.c_float
    lib.planner_api_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]

    # planner_api_apply_timed(h, vx, omega, duration_ms, now_ms) — 066-002:
    # now_ms threaded through as the 5th argument (was 3-arg pre-fix).
    lib.planner_api_apply_timed.restype = None
    lib.planner_api_apply_timed.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float,
        ctypes.c_uint32, ctypes.c_uint32,
    ]

    lib.planner_api_get_active.restype = ctypes.c_int
    lib.planner_api_get_active.argtypes = [ctypes.c_void_p]

    lib.planner_api_get_fused_x.restype = ctypes.c_float
    lib.planner_api_get_fused_x.argtypes = [ctypes.c_void_p]

    return lib


@pytest.fixture(scope="module")
def lib():
    return _load_lib()


@pytest.fixture
def handle(lib):
    h = lib.planner_api_create()
    assert h, "planner_api_create() returned NULL"
    yield h
    lib.planner_api_destroy(h)


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_timed_goal_runs_full_duration_at_realistic_now_ms(lib, handle):
    """A TIMED goal staged with a realistic nonzero now_ms must not stop early.

    Regression guard for CR-11: before the fix, apply() ignored now_ms and
    always baselined t0Ms=0, so the very first tick() (at now_ms=100000) saw
    elapsed = 100000 - 0 >> duration_ms and fired the TIME stop instantly.
    """
    now_start = 100_000  # realistic nonzero uptime (100 s), not 0
    duration_ms = 2000
    vx = 150.0  # mm/s

    lib.planner_api_apply_timed(
        handle, ctypes.c_float(vx), ctypes.c_float(0.0),
        ctypes.c_uint32(duration_ms), ctypes.c_uint32(now_start),
    )

    # Tick forward to just short of the goal's duration (well past the BVC's
    # ramp-up time: aMax=300 mm/s^2 => ~0.5s to reach 150 mm/s).
    now_ms = now_start
    end_before_duration = now_start + 1800
    while now_ms < end_before_duration:
        now_ms += 10
        lib.planner_api_tick(handle, ctypes.c_uint32(now_ms))

    assert lib.planner_api_get_active(handle) == 1, (
        "MotionCommand went inactive before the TIMED goal's duration "
        "elapsed — the TIME stop fired early (t0Ms baseline bug, CR-11)."
    )

    fused_x = lib.planner_api_get_fused_x(handle)
    # Under the bug, the SOFT ramp-down starts on the very first tick, so the
    # robot barely moves (well under 100 mm by t=1800ms). Fixed behaviour
    # covers close to vx * 1.8s minus ramp-up loss (~230 mm here).
    assert fused_x > 100.0, (
        f"fused_x={fused_x:.1f}mm after 1.8s of a {vx}mm/s TIMED goal — "
        f"expected close to full-speed travel; the TIME stop appears to "
        f"have fired early (t0Ms baseline bug, CR-11)."
    )

    # Tick well past duration_ms + the SOFT stop's 3s deadline cap so the
    # command is guaranteed to have finished (converged or deadline-forced) —
    # proves the TIME stop is not simply inert, only correctly delayed.
    end_after_deadline = now_start + duration_ms + 3000 + 500
    while now_ms < end_after_deadline:
        now_ms += 10
        lib.planner_api_tick(handle, ctypes.c_uint32(now_ms))

    assert lib.planner_api_get_active(handle) == 0, (
        "MotionCommand never completed even after now_ms passed "
        "t0Ms + duration_ms + the SOFT-stop deadline cap."
    )
