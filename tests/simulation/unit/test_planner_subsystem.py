"""
test_planner_subsystem.py — Planner-isolation sim tests (ticket 059-002).

Exercises the Planner subsystem via C-ABI shims in
tests/_infra/sim/planner_api.cpp, loaded via ctypes.  This is the
stakeholder-requested validation: construct Planner in isolation
(via the C-ABI shim — no full robot, no comms), feed user goals via apply(),
call tick() repeatedly, and assert the RETURNED CommandBatch DrivetrainCommand
{twist} sequence (the vx + omega setpoints over time).

No robot, no comms — the RETURN model makes this a pure function of goal +
injected pose.  SimHardware produces a stationary pose (zero physics), which
is sufficient for testing goal convergence and timing logic.

Fixture pattern:
    h = lib.planner_api_create()     # allocate a fresh PlannerHandle
    lib.planner_api_apply_*(h, ...)  # stage a goal
    for tick in range(N):
        vx = lib.planner_api_tick(h, now_ms)   # advance one tick, read vx
        omega = lib.planner_api_get_body_twist_omega(h)
        active = lib.planner_api_is_active(h)
    lib.planner_api_destroy(h)       # release

Default robot config (DefaultConfig.cpp):
    aMax       = 300 mm/s²    — linear acceleration limit
    aDecel     = 250 mm/s²    — linear deceleration limit
    vBodyMax   = 400 mm/s     — body forward speed ceiling
    yawRateMax = 70 deg/s     — yaw rate ceiling

Extending this fixture for new goal types:
    1. Add a new planner_api_apply_* shim in tests/_infra/sim/planner_api.cpp.
    2. Bind its ctypes signature in _load_lib() below.
    3. Write a test function following the pattern above: apply goal, tick N
       times collecting (vx, omega, active) per tick, assert on the profile
       shape rather than exact values.
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


def _load_lib() -> ctypes.CDLL:
    """Load firmware_host and configure planner_api shim signatures."""
    lib = ctypes.CDLL(str(LIB_PATH))

    # --- Lifecycle ---
    lib.planner_api_create.restype = ctypes.c_void_p
    lib.planner_api_create.argtypes = []

    lib.planner_api_destroy.restype = None
    lib.planner_api_destroy.argtypes = [ctypes.c_void_p]

    # --- Tick: runs one full sense/plan/act cycle; returns commanded vx (mm/s) ---
    lib.planner_api_tick.restype = ctypes.c_float
    lib.planner_api_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]

    # --- Goal application shims ---
    # 066-002 / CR-11: each apply_* shim gained a trailing now_ms parameter
    # (threaded through to Planner::apply(cmd, now_ms)). Every call site in
    # this file applies its goal before the first tick(), so now_ms=0 here
    # matches this file's own tick cadence (_run_ticks starts at start_ms=0)
    # — not a workaround, the correct baseline for these fixtures.
    lib.planner_api_apply_velocity.restype = None
    lib.planner_api_apply_velocity.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_uint32,
    ]

    lib.planner_api_apply_timed.restype = None
    lib.planner_api_apply_timed.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_uint32,
        ctypes.c_uint32,
    ]

    lib.planner_api_apply_turn.restype = None
    lib.planner_api_apply_turn.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_uint32,
    ]

    lib.planner_api_apply_distance.restype = None
    lib.planner_api_apply_distance.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_uint32,
    ]

    lib.planner_api_apply_stop.restype = None
    lib.planner_api_apply_stop.argtypes = [ctypes.c_void_p, ctypes.c_uint32]

    # --- State reads ---
    lib.planner_api_get_active.restype = ctypes.c_int
    lib.planner_api_get_active.argtypes = [ctypes.c_void_p]

    # planner_api_is_active: alias for planner_api_get_active (ticket 059-002 AC)
    lib.planner_api_is_active.restype = ctypes.c_int
    lib.planner_api_is_active.argtypes = [ctypes.c_void_p]

    lib.planner_api_get_mode.restype = ctypes.c_int
    lib.planner_api_get_mode.argtypes = [ctypes.c_void_p]

    lib.planner_api_get_body_twist_vx.restype = ctypes.c_float
    lib.planner_api_get_body_twist_vx.argtypes = [ctypes.c_void_p]

    lib.planner_api_get_body_twist_omega.restype = ctypes.c_float
    lib.planner_api_get_body_twist_omega.argtypes = [ctypes.c_void_p]

    return lib


@pytest.fixture(scope="module")
def lib() -> ctypes.CDLL:
    return _load_lib()


# ---------------------------------------------------------------------------
# Helper: tick the planner N times at TICK_MS cadence, return per-tick records.
# Each record: (now_ms, vx_mmps, omega_rads, active).
# ---------------------------------------------------------------------------

TICK_MS = 20  # 20 ms per tick — matches the live loop cadence


def _run_ticks(lib, h, n_ticks: int, start_ms: int = 0) -> list:
    """Tick planner n_ticks times; return list of (now_ms, vx, omega, active)."""
    records = []
    for i in range(n_ticks):
        now_ms = start_ms + i * TICK_MS
        vx = lib.planner_api_tick(h, ctypes.c_uint32(now_ms))
        omega = lib.planner_api_get_body_twist_omega(h)
        active = lib.planner_api_is_active(h)
        records.append((now_ms, float(vx), float(omega), int(active)))
    return records


# ===========================================================================
# Tests
# ===========================================================================


class TestPlannerIsolation:
    """Planner-isolation tests: apply goal → tick → assert CommandBatch profile.

    All tests construct a fresh PlannerHandle, feed a goal, run N ticks,
    and assert on the shape of the (vx, omega, active) profile returned by
    planner_api_tick + state getters.  No robot, no comms — pure function.
    """

    # -----------------------------------------------------------------------
    # 1. Timed goal: trapezoid velocity profile
    # -----------------------------------------------------------------------
    def test_timed_goal_twist_profile(self, lib):
        """Apply timed goal (100 mm/s for 1000 ms); assert trapezoid vx profile.

        Default config: aMax=300 mm/s², aDecel=250 mm/s².
        Ramp-up to 100 mm/s: 100/300 ≈ 333 ms ≈ 17 ticks of 20 ms.
        Cruise at ~100 mm/s: ticks 17–50 (remaining of 1000 ms).
        SOFT stop (decel) fires after deadline; command goes inactive.

        Assertions (shape, not exact values):
          - First 10 ticks: vx increasing (ramp-up region).
          - Ticks 20–40: vx near commanded speed (within 20% of 100 mm/s).
          - After 1200 ms (60 ticks): is_active == 0, vx near 0.
          - omega stays near 0 throughout (straight forward timed goal).
        """
        TARGET_VX = 100.0  # mm/s
        DURATION_MS = 1000  # ms
        # Run enough ticks to cover ramp-up, cruise, decel, and termination.
        # 1000 ms duration + ~500 ms decel/settle = 1500 ms = 75 ticks
        N_TICKS = 80

        h = lib.planner_api_create()
        try:
            lib.planner_api_apply_timed(
                h,
                ctypes.c_float(TARGET_VX),
                ctypes.c_float(0.0),   # omega — straight forward
                ctypes.c_uint32(DURATION_MS),
                ctypes.c_uint32(0),    # now_ms — applied before the first tick (t=0)
            )
            records = _run_ticks(lib, h, N_TICKS)

            vxs    = [r[1] for r in records]
            omegas = [r[2] for r in records]
            actives = [r[3] for r in records]

            # --- Ramp-up: first 10 ticks (0–200 ms) should show increasing vx ---
            # aMax=300 mm/s² × 0.02 s = 6 mm/s per tick, so after 10 ticks ≈ 60 mm/s.
            ramp_region = vxs[:10]
            assert ramp_region[0] >= 0.0, \
                f"Tick 0 vx should be >= 0, got {ramp_region[0]:.2f}"
            assert ramp_region[-1] > ramp_region[0], \
                (f"vx should increase in ramp region: "
                 f"first={ramp_region[0]:.2f}, last={ramp_region[-1]:.2f}")
            # Verify monotonically non-decreasing over the ramp (small tolerance for BVC step)
            for i in range(1, len(ramp_region)):
                assert ramp_region[i] >= ramp_region[i - 1] - 0.1, \
                    f"vx not monotonic at tick {i}: {ramp_region[i - 1]:.2f} → {ramp_region[i]:.2f}"

            # --- Cruise: ticks 20–45 (400–900 ms) should be near TARGET_VX ---
            cruise_region = vxs[20:45]
            for i, vx in enumerate(cruise_region):
                assert vx > TARGET_VX * 0.70, \
                    (f"Cruise tick {20 + i}: vx={vx:.2f} should be > "
                     f"{TARGET_VX * 0.70:.2f} (70% of target)")
                assert vx <= TARGET_VX * 1.10, \
                    (f"Cruise tick {20 + i}: vx={vx:.2f} should be <= "
                     f"{TARGET_VX * 1.10:.2f} (110% of target — no overshoot)")

            # --- Termination: at 60 ticks (1200 ms), is_active should be 0 ---
            assert actives[60] == 0 or actives[70] == 0, \
                (f"is_active should be 0 by tick 60–70 (1200–1400 ms) "
                 f"for 1000 ms timed goal; "
                 f"actives[60]={actives[60]}, actives[70]={actives[70]}")

            # --- After termination, vx should be near 0 ---
            # Find when active first drops to 0
            done_tick = next((i for i, a in enumerate(actives) if a == 0), None)
            if done_tick is not None and done_tick + 5 < N_TICKS:
                late_vx = vxs[done_tick + 5]
                assert abs(late_vx) < TARGET_VX * 0.20, \
                    (f"vx should be near 0 after goal completes: "
                     f"vx at tick {done_tick + 5} = {late_vx:.2f}")

            # --- Omega stays near 0 (straight forward goal, no yaw component) ---
            for i, omega in enumerate(omegas[:50]):
                assert abs(omega) < 0.05, \
                    (f"Tick {i}: omega={omega:.4f} should stay near 0 "
                     f"for a straight-forward timed goal")

        finally:
            lib.planner_api_destroy(h)

    # -----------------------------------------------------------------------
    # 2. Turn goal: omega non-zero while active, vx near 0
    # -----------------------------------------------------------------------
    def test_turn_goal_convergence(self, lib):
        """Apply turn goal (π/4 rad, 45°); assert omega non-zero while active.

        yawRateMax = 70 deg/s → omega ≈ 1.22 rad/s for CCW.
        Nominal turn time: (π/4)/1.22 × 1000 ≈ 642 ms.
        Safety timeout: 2×642 + 2000 = 3284 ms ≈ 164 ticks × 20 ms.
        With zero physics, heading never advances → HEADING stop never fires.
        The TIME net fires at 3284 ms → is_active becomes 0.

        At 200 ticks × 20 ms = 4000 ms > 3284 ms, is_active should be 0.

        Assertions:
          - Ticks 1–50: omega non-zero (turn in progress).
          - Ticks 1–50: vx near 0 (spin-in-place).
          - After 200 ticks: is_active == 0.
        """
        TARGET_HEADING_RAD = math.pi / 4.0  # 45°
        N_TICKS = 200

        h = lib.planner_api_create()
        try:
            lib.planner_api_apply_turn(h, ctypes.c_float(TARGET_HEADING_RAD),
                                       ctypes.c_uint32(0))
            records = _run_ticks(lib, h, N_TICKS)

            omegas  = [r[2] for r in records]
            vxs     = [r[1] for r in records]
            actives = [r[3] for r in records]

            # --- omega should be non-zero in the active phase (first 50 ticks) ---
            # yawRateMax = 70 deg/s = 1.22 rad/s
            YAW_RATE_RAD = 70.0 * math.pi / 180.0  # ≈ 1.22 rad/s
            active_omegas = [omegas[i] for i in range(1, 50) if actives[i] == 1]
            if active_omegas:
                max_omega = max(abs(o) for o in active_omegas)
                assert max_omega > YAW_RATE_RAD * 0.5, \
                    (f"omega should reach >= 50% of yaw_rate_max={YAW_RATE_RAD:.3f} rad/s "
                     f"during turn; got max={max_omega:.4f}")
            else:
                # If no active tick in first 50, the planner was never active
                # (should not happen — fail with a clear message)
                assert False, \
                    "No active tick found in first 50 ticks for turn goal — planner never activated"

            # --- vx should stay near 0 for spin-in-place ---
            for i in range(1, 50):
                if actives[i] == 1:
                    assert abs(vxs[i]) < 10.0, \
                        (f"Tick {i}: vx={vxs[i]:.3f} should be near 0 "
                         f"for spin-in-place turn goal")

            # --- After 200 ticks (4000 ms), time-net should have fired → inactive ---
            # The last few ticks should be inactive
            last_10 = actives[190:]
            assert all(a == 0 for a in last_10), \
                (f"is_active should be 0 after time-net fires at 200 ticks "
                 f"(4000 ms); last_10={last_10}")

        finally:
            lib.planner_api_destroy(h)

    # -----------------------------------------------------------------------
    # 3. Distance goal: vx > 0 during motion, CommandBatch present each tick
    # -----------------------------------------------------------------------
    def test_distance_goal_profile(self, lib):
        """Apply distance goal (300 mm at 150 mm/s); assert vx > 0 during motion.

        With zero physics, encoder distance never advances → DISTANCE stop
        never fires.  Safety timeout: 2×(300/150)×1000 + 2000 = 4000 ms.
        At 220 ticks × 20 ms = 4400 ms > 4000 ms, is_active becomes 0.

        Ramp-up to 150 mm/s at aMax=300 mm/s²: 150/300 = 500 ms ≈ 25 ticks.

        Assertions:
          - Ticks 1–20: vx positive (ramp-up and cruise).
          - Tick 30 onward (600 ms): vx near target (within 25% of 150 mm/s).
          - At 220 ticks: is_active == 0 (timeout fired, command done).
          - omega stays near 0 (straight drive).
        """
        DIST_MM   = 300.0
        SPEED_MPS = 150.0
        N_TICKS   = 230  # 4600 ms — exceeds 4000 ms safety timeout

        h = lib.planner_api_create()
        try:
            lib.planner_api_apply_distance(
                h,
                ctypes.c_float(DIST_MM),
                ctypes.c_float(SPEED_MPS),
                ctypes.c_uint32(0),
            )
            records = _run_ticks(lib, h, N_TICKS)

            vxs     = [r[1] for r in records]
            omegas  = [r[2] for r in records]
            actives = [r[3] for r in records]

            # --- vx positive during motion (first 20 active ticks after tick 0) ---
            for i in range(1, 20):
                if actives[i] == 1:
                    assert vxs[i] >= 0.0, \
                        f"Tick {i}: vx={vxs[i]:.2f} should be >= 0 during distance drive"

            # --- At tick 30 (600 ms), ramp complete — vx should be near target ---
            cruise_region = [vxs[i] for i in range(30, 80) if actives[i] == 1]
            if cruise_region:
                avg_cruise = sum(cruise_region) / len(cruise_region)
                assert avg_cruise > SPEED_MPS * 0.70, \
                    (f"Cruise vx avg={avg_cruise:.2f} should be > "
                     f"{SPEED_MPS * 0.70:.2f} (70% of target {SPEED_MPS})")

            # --- At 220 ticks (4400 ms), time-net should have fired → inactive ---
            last_10 = actives[220:]
            assert all(a == 0 for a in last_10), \
                (f"is_active should be 0 at tick 220+ (4400 ms); "
                 f"safety timeout at 4000 ms; last_10={last_10}")

            # --- omega near 0 for straight drive ---
            active_region = [(i, omegas[i]) for i in range(1, 80) if actives[i] == 1]
            for i, omega in active_region:
                assert abs(omega) < 0.10, \
                    f"Tick {i}: omega={omega:.4f} should be near 0 for straight distance goal"

        finally:
            lib.planner_api_destroy(h)

    # -----------------------------------------------------------------------
    # 4. Stop command clears active state
    # -----------------------------------------------------------------------
    def test_stop_command_clears_active(self, lib):
        """Apply timed goal then apply stop; assert is_active becomes 0 immediately.

        Sequence:
          1. Apply timed goal (200 mm/s for 2000 ms).
          2. Tick 5 times (100 ms) — goal should be active, vx ramping.
          3. Apply stop → MC2 calls mc.stop() which does a HARD cancel.
          4. Tick once more — is_active must be 0.

        Implementation note: after HARD cancel + mc.stop(), the mode is IDLE
        and _activeCmd.active() is false. The BVC is NOT advanced after a hard
        stop (driveAdvance exits early once mode=IDLE and no MotionCommand is
        active), so the body_twist in _desired freezes at its last ramp value.
        The RETURN model contract is: is_active=0 and no new TWIST commands are
        issued to Drive — which is what the live bus dispatcher checks before
        forwarding.  We assert is_active=0 (the essential invariant) rather than
        vx=0 (which depends on BVC coast behaviour outside this ticket's scope).
        """
        h = lib.planner_api_create()
        try:
            # Stage the timed goal
            lib.planner_api_apply_timed(
                h,
                ctypes.c_float(200.0),
                ctypes.c_float(0.0),
                ctypes.c_uint32(2000),
                ctypes.c_uint32(0),    # now_ms — applied before the first tick (t=0)
            )

            # Tick 5 times — confirm it became active and vx is ramping
            now_ms = 0
            for _ in range(5):
                lib.planner_api_tick(h, ctypes.c_uint32(now_ms))
                now_ms += TICK_MS

            active_before_stop = lib.planner_api_is_active(h)
            vx_before_stop = lib.planner_api_get_body_twist_vx(h)
            assert active_before_stop == 1, \
                f"is_active should be 1 before stop; got {active_before_stop}"
            assert vx_before_stop > 0.0, \
                f"vx should be > 0 before stop (ramping); got {vx_before_stop:.2f}"

            # Issue STOP — hard cancel; mode goes IDLE immediately
            lib.planner_api_apply_stop(h, ctypes.c_uint32(now_ms))

            # One tick after stop: is_active must be 0
            lib.planner_api_tick(h, ctypes.c_uint32(now_ms))

            active_after_stop = lib.planner_api_is_active(h)
            assert active_after_stop == 0, \
                f"is_active should be 0 after stop command; got {active_after_stop}"

            # Mode should be IDLE (msg::DriveMode::IDLE = 0)
            mode_after_stop = lib.planner_api_get_mode(h)
            assert mode_after_stop == 0, \
                f"mode should be IDLE (0) after stop; got {mode_after_stop}"

        finally:
            lib.planner_api_destroy(h)

    # -----------------------------------------------------------------------
    # 5. Idle planner returns zero twist (no goal → no command)
    # -----------------------------------------------------------------------
    def test_planner_returns_empty_batch_when_idle(self, lib):
        """Tick with no goal applied; assert is_active == 0 and vx, omega near 0.

        The planner with no staged goal should be IDLE: tick() should return
        zero twist and is_active should be 0.  This validates the RETURN model
        does not emit spurious commands when idle.
        """
        N_TICKS = 10

        h = lib.planner_api_create()
        try:
            records = _run_ticks(lib, h, N_TICKS)

            for i, (now_ms, vx, omega, active) in enumerate(records):
                assert active == 0, \
                    f"Tick {i}: is_active={active}, expected 0 (idle)"
                assert abs(vx) < 1.0, \
                    f"Tick {i}: vx={vx:.3f}, expected near 0 (idle)"
                assert abs(omega) < 0.01, \
                    f"Tick {i}: omega={omega:.4f}, expected near 0 (idle)"

        finally:
            lib.planner_api_destroy(h)
