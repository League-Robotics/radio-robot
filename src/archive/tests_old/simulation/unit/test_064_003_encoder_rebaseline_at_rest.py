"""
test_064_003_encoder_rebaseline_at_rest.py — 064-003 regression tests.

MotorController::resetEncoderAccumulators() must choose a software-only
rebaseline (Motor::rebaselineSoft() / SimMotor::rebaselineSoft()) instead of
the hardware atomic-read burst (Motor::resetEncoder() / SimMotor::
resetEncoder()) whenever the drivetrain is NOT at rest — firing the atomic
0x46 burst while the wheels are rotating latches the Nezha encoder readback
(see clasi/sprints/064-.../issues/encoder-reset-while-moving-latches-readback.md,
stress-matrix arm 3: 13 transient latches / 10 cycles from exactly this
mechanism).

Reproduces the arm-3 scenario in sim: start a D command, let the wheels reach
nonzero velocity, then issue a second D before the first completes
(preemption). Both Planner::beginDistance() and Robot::distanceDrive() call
MotorController::resetEncoderAccumulators() (a pre-existing, un-fixed
redundancy — see architecture-update.md Open Question 1), so a single D
command issued while genuinely at rest increments the hard-reset counter by
2 (not 1); this file asserts *deltas* and *which* counter moved, not exact
counts, to stay independent of that redundancy.

Also verifies the at-rest path (D from idle, ZERO enc at idle) is
unchanged: it must still take the hardware atomic re-prime, because that
at-rest hardware re-prime is ALSO the transient-wedge self-heal mechanism
relied on elsewhere.
"""
import ctypes

import pytest

from firmware import Sim


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _register_reset_count_hooks(lib) -> None:
    """Register argtypes/restype for the 064-003 sim hooks on this CDLL.

    ctypes caches argtypes/restype per (CDLL instance, function name); each
    fresh Sim() creates a new CDLL wrapper, so this must be (re)done for
    every test's ``sim._lib`` — cheap and idempotent.
    """
    for name in (
        "sim_get_motor_hard_reset_count_l",
        "sim_get_motor_hard_reset_count_r",
        "sim_get_motor_soft_reset_count_l",
        "sim_get_motor_soft_reset_count_r",
    ):
        fn = getattr(lib, name)
        fn.argtypes = [ctypes.c_void_p]
        fn.restype = ctypes.c_int


class _Counts:
    __slots__ = ("hard_l", "hard_r", "soft_l", "soft_r")

    def __init__(self, hard_l: int, hard_r: int, soft_l: int, soft_r: int):
        self.hard_l = hard_l
        self.hard_r = hard_r
        self.soft_l = soft_l
        self.soft_r = soft_r


def _get_counts(sim: Sim) -> _Counts:
    return _Counts(
        hard_l=int(sim._lib.sim_get_motor_hard_reset_count_l(sim._h)),
        hard_r=int(sim._lib.sim_get_motor_hard_reset_count_r(sim._h)),
        soft_l=int(sim._lib.sim_get_motor_soft_reset_count_l(sim._h)),
        soft_r=int(sim._lib.sim_get_motor_soft_reset_count_r(sim._h)),
    )


# ---------------------------------------------------------------------------
# Test 1: reset while moving (D preempted by D, stress-matrix arm 3) — the
# not-at-rest path must use the software rebaseline, not the hardware burst.
# ---------------------------------------------------------------------------

def test_d_preempted_by_d_mid_flight_uses_soft_rebaseline(sim):
    """D preempting a still-moving D: software rebaseline, no hardware reset."""
    _register_reset_count_hooks(sim._lib)

    # Start a long drive so the second D below still finds it in flight.
    r = sim.send_command("D 200 200 400")
    assert "OK" in r.upper(), f"D1 failed: {repr(r)}"

    # Let the wheels ramp up to a clearly non-zero velocity.
    sim.tick_for(200)

    vel_l = float(sim._lib.sim_get_vel_l(sim._h))
    vel_r = float(sim._lib.sim_get_vel_r(sim._h))
    assert abs(vel_l) > 20.0 and abs(vel_r) > 20.0, (
        f"test setup: expected wheels moving (> 20 mm/s) before preemption, "
        f"got vel_l={vel_l:.1f}, vel_r={vel_r:.1f}"
    )

    before = _get_counts(sim)
    sim.get_async_evts()  # clear D1's accumulated EVTs (not under test here)

    # Preempt with a second D — this is the arm-3 trigger: Planner::
    # beginDistance() -> resetEncoderAccumulators() fires while the previous
    # command's wheels are still rotating.
    r2 = sim.send_command("D 150 150 100")
    assert "OK" in r2.upper(), f"D2 (preempting) failed: {repr(r2)}"

    after = _get_counts(sim)

    # Software rebaseline was used for BOTH wheels.
    assert after.soft_l > before.soft_l, (
        f"softResetCount(L) did not increment on mid-motion reset: "
        f"before={before.soft_l}, after={after.soft_l}"
    )
    assert after.soft_r > before.soft_r, (
        f"softResetCount(R) did not increment on mid-motion reset: "
        f"before={before.soft_r}, after={after.soft_r}"
    )

    # The hardware atomic-read burst must NOT have fired during this
    # preemption — that is exactly the mechanism that latches the Nezha
    # readback while the wheels are rotating.
    assert after.hard_l == before.hard_l, (
        f"hardResetCount(L) incremented during a mid-motion reset (should "
        f"have taken the software path): before={before.hard_l}, "
        f"after={after.hard_l}"
    )
    assert after.hard_r == before.hard_r, (
        f"hardResetCount(R) incremented during a mid-motion reset (should "
        f"have taken the software path): before={before.hard_r}, "
        f"after={after.hard_r}"
    )

    # The baseline after the mid-motion reset must not jump to a large
    # spurious value -- a fresh read shortly after must be small.
    sim.tick_for(24)
    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))
    assert abs(enc_l) < 20.0, (
        f"enc_l={enc_l:.1f} mm shortly after the mid-motion soft rebaseline "
        f"-- expected a small value, not a spurious jump"
    )
    assert abs(enc_r) < 20.0, (
        f"enc_r={enc_r:.1f} mm shortly after the mid-motion soft rebaseline "
        f"-- expected a small value, not a spurious jump"
    )

    # D2 must still complete cleanly (correctness, not just counters).
    evts = sim.get_async_evts() + sim.get_async_evts()
    all_evts = evts + _tick_until_done(sim, 8000)
    assert "EVT done D" in all_evts, f"D2 never completed: {repr(all_evts)}"


def _tick_until_done(sim: Sim, total_ms: int) -> str:
    evts = ""
    step = 24
    elapsed = 0
    while elapsed < total_ms:
        sim.tick_for(step)
        elapsed += step
        chunk = sim.get_async_evts()
        evts += chunk
        if "EVT done D" in evts:
            break
    return evts


# ---------------------------------------------------------------------------
# Test 2: at-rest path is unchanged — D from idle must still use the
# hardware atomic re-prime (the transient-wedge self-heal mechanism).
# ---------------------------------------------------------------------------

def test_d_from_idle_uses_hard_reset(sim):
    """D issued from a genuinely idle drivetrain: hardware reset, no soft path."""
    _register_reset_count_hooks(sim._lib)

    before = _get_counts(sim)

    r = sim.send_command("D 200 200 200")
    assert "OK" in r.upper(), f"D failed: {repr(r)}"

    after = _get_counts(sim)

    assert after.hard_l > before.hard_l, (
        f"hardResetCount(L) did not increment for a D issued from idle: "
        f"before={before.hard_l}, after={after.hard_l}"
    )
    assert after.hard_r > before.hard_r, (
        f"hardResetCount(R) did not increment for a D issued from idle: "
        f"before={before.hard_r}, after={after.hard_r}"
    )
    # No software rebaseline should have been used -- the drivetrain was
    # genuinely at rest (fresh sim, never driven).
    assert after.soft_l == before.soft_l == 0, (
        f"softResetCount(L) unexpectedly nonzero for an at-rest reset: "
        f"before={before.soft_l}, after={after.soft_l}"
    )
    assert after.soft_r == before.soft_r == 0, (
        f"softResetCount(R) unexpectedly nonzero for an at-rest reset: "
        f"before={before.soft_r}, after={after.soft_r}"
    )

    # Let it complete normally -- correctness is unaffected.
    all_evts = _tick_until_done(sim, 8000)
    assert "EVT done D" in all_evts, f"D never completed: {repr(all_evts)}"


# ---------------------------------------------------------------------------
# Test 3: ZERO enc at idle is unchanged -- hardware re-prime, not software.
# ---------------------------------------------------------------------------

def test_zero_enc_at_idle_uses_hard_reset(sim):
    """ZERO enc while genuinely at rest: hardware reset (self-heal path)."""
    _register_reset_count_hooks(sim._lib)

    # Drive briefly, then stop, so there is nonzero prior travel to zero.
    r = sim.send_command("D 150 150 60")
    assert "OK" in r.upper(), f"D failed: {repr(r)}"
    all_evts = _tick_until_done(sim, 8000)
    assert "EVT done D" in all_evts, f"setup D never completed: {repr(all_evts)}"

    # Let velocity settle to zero after the D command completes.
    sim.tick_for(200)
    vel_l = float(sim._lib.sim_get_vel_l(sim._h))
    vel_r = float(sim._lib.sim_get_vel_r(sim._h))
    assert abs(vel_l) < 5.0 and abs(vel_r) < 5.0, (
        f"test setup: expected the drivetrain at rest before ZERO enc, "
        f"got vel_l={vel_l:.1f}, vel_r={vel_r:.1f}"
    )

    before = _get_counts(sim)

    r2 = sim.send_command("ZERO enc")
    assert "OK" in r2.upper(), f"ZERO enc failed: {repr(r2)}"

    after = _get_counts(sim)

    assert after.hard_l > before.hard_l, (
        f"hardResetCount(L) did not increment for ZERO enc at idle: "
        f"before={before.hard_l}, after={after.hard_l}"
    )
    assert after.hard_r > before.hard_r, (
        f"hardResetCount(R) did not increment for ZERO enc at idle: "
        f"before={before.hard_r}, after={after.hard_r}"
    )
    assert after.soft_l == before.soft_l, (
        f"softResetCount(L) unexpectedly incremented for ZERO enc at idle: "
        f"before={before.soft_l}, after={after.soft_l}"
    )
    assert after.soft_r == before.soft_r, (
        f"softResetCount(R) unexpectedly incremented for ZERO enc at idle: "
        f"before={before.soft_r}, after={after.soft_r}"
    )
