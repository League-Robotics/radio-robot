"""
test_motor_slew.py — unit tests for MotorSlew.h / MotorSlew::clampStep()
(064-002, |ΔPWM| slew cap for Motor::setSpeed()).

MotorSlew.h is a dependency-free, CODAL-free header, so it compiles into
libfirmware_host and is exercised here via the sim_motor_clamp_slew() C-ABI
hook (tests/_infra/sim/sim_api.cpp), which forwards straight to
MotorSlew::clampStep(lastWritten, target, maxDelta).

Motor::setSpeed() itself (source/hal/real/Motor.cpp) is NOT reachable from
HOST_BUILD (hal/real/ is excluded from the sim library — see
tests/_infra/sim/CMakeLists.txt) — that call site is verified by code review
against this ticket's diff instead (see the ticket's Testing section). This
file tests the pure clamp arithmetic only. It intentionally does not test any
"pct == 0 stop" behavior: the pure helper has no stop concept — that
special-casing lives in the caller, Motor::setSpeed().
"""
import ctypes
import pathlib
import sys

# Locate the sim shared library (same path as firmware.py / test_argparse.py).
_HERE = pathlib.Path(__file__).parent
_REPO = _HERE.parent.parent.parent
_SIM_DIR = _REPO / "tests" / "_infra" / "sim"

sys.path.insert(0, str(_SIM_DIR))

from firmware import LIB_PATH  # noqa: E402 (after sys.path insert)


def _load_lib():
    lib = ctypes.CDLL(str(LIB_PATH))
    lib.sim_motor_clamp_slew.restype = ctypes.c_int
    lib.sim_motor_clamp_slew.argtypes = [
        ctypes.c_int,  # lastWritten
        ctypes.c_int,  # target
        ctypes.c_int,  # maxDelta
    ]
    return lib


_lib = _load_lib()


def clamp_step(last_written: int, target: int, max_delta: int) -> int:
    return _lib.sim_motor_clamp_slew(last_written, target, max_delta)


# ---------------------------------------------------------------------------
# No clamp needed when within cap
# ---------------------------------------------------------------------------

class TestNoClampNeeded:
    def test_small_positive_step_passes_through(self):
        assert clamp_step(0, 10, 25) == 10

    def test_small_negative_step_passes_through(self):
        assert clamp_step(0, -10, 25) == -10

    def test_step_exactly_at_cap_passes_through(self):
        # |target - lastWritten| == maxDelta is within the "<=" bound.
        assert clamp_step(0, 25, 25) == 25
        assert clamp_step(0, -25, 25) == -25

    def test_unchanged_target_passes_through(self):
        assert clamp_step(40, 40, 25) == 40

    def test_within_cap_from_nonzero_baseline(self):
        assert clamp_step(50, 60, 25) == 60
        assert clamp_step(-50, -60, 25) == -60


# ---------------------------------------------------------------------------
# Clamp toward target when exceeding cap, either direction
# ---------------------------------------------------------------------------

class TestClampsTowardTarget:
    def test_large_positive_step_clamped(self):
        # 0 -> 100 with cap 25 steps to 25.
        assert clamp_step(0, 100, 25) == 25

    def test_large_negative_step_clamped(self):
        assert clamp_step(0, -100, 25) == -25

    def test_full_reversal_first_step_clamped(self):
        # +100 -> -100 (the arm-5 stand-session trigger): first write must
        # step by at most 25, not slam the full 200-point swing.
        assert clamp_step(100, -100, 25) == 75

    def test_clamp_from_positive_baseline_toward_more_negative(self):
        assert clamp_step(50, -50, 25) == 25

    def test_clamp_respects_custom_max_delta(self):
        assert clamp_step(0, 100, 10) == 10
        assert clamp_step(0, -100, 40) == -40


# ---------------------------------------------------------------------------
# Multi-call convergence — repeatedly calling with the same target eventually
# reaches it, mirroring how Motor::setSpeed() feeds the clamped `written`
# value back in as `lastWritten` on the next call.
# ---------------------------------------------------------------------------

class TestMultiCallConvergence:
    def test_full_reversal_converges_over_multiple_calls(self):
        # +100 -> -100 with cap 25: 100,75,50,25,0,-25,-50,-75,-100 (8 steps).
        last = 100
        target = -100
        steps = [last]
        for _ in range(20):
            if last == target:
                break
            last = clamp_step(last, target, 25)
            steps.append(last)
        assert last == target
        assert len(steps) - 1 == 8
        # Each step moves by at most the cap.
        for a, b in zip(steps, steps[1:]):
            assert abs(b - a) <= 25

    def test_convergence_from_arbitrary_baseline(self):
        last = -37
        target = 64
        for _ in range(20):
            if last == target:
                break
            last = clamp_step(last, target, 25)
        assert last == target

    def test_convergence_is_monotonic_toward_target(self):
        last = 0
        target = 100
        prev_gap = abs(target - last)
        for _ in range(20):
            if last == target:
                break
            last = clamp_step(last, target, 25)
            gap = abs(target - last)
            assert gap < prev_gap
            prev_gap = gap
        assert last == target

    def test_single_call_suffices_when_already_within_cap(self):
        assert clamp_step(10, 15, 25) == 15


# ---------------------------------------------------------------------------
# Boundary / edge values
# ---------------------------------------------------------------------------

class TestBoundaries:
    def test_extreme_reversal_full_range(self):
        # int8_t range clamp is Motor::setSpeed()'s job, not clampStep()'s —
        # but the pure arithmetic must still behave sanely at ±100.
        assert clamp_step(-100, 100, 25) == -75

    def test_zero_max_delta_never_moves(self):
        assert clamp_step(10, 20, 0) == 10
        assert clamp_step(10, 0, 0) == 10

    def test_target_zero_reachable_via_clamp(self):
        # Not a "stop" in Motor::setSpeed()'s sense (that's handled by the
        # caller) — just target == 0 as an ordinary value for this pure
        # function's contract.
        assert clamp_step(20, 0, 25) == 0
        assert clamp_step(30, 0, 25) == 5
