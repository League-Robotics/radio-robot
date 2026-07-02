"""test_064_004_wedge_blindspots.py — 064-004 regression tests.

MotorController's per-wheel wedge detector had two structural blind spots
that together caused it to miss ALL ~18 observed wedge episodes in the
2026-07-01/02 stand sessions (see clasi/sprints/064-.../issues/
encoder-reset-while-moving-latches-readback.md):

  1. Target==0 reset: the stuck counter zeroed every tick a wheel's target
     was 0 -- including the transient tick(s) at a command's own
     deceleration/stop boundary, exactly where the latch mechanism onsets.
  2. Arming grace (033-005d, _hasMovedL/R): counting did not start until the
     wheel had moved at least once since the current command started -- a
     wheel that enters a NEW command already frozen never "moves," so
     counting never started (Episode A: RT turn frozen for 14 TLM frames,
     zero EVT).

064-004 removes both blind spots (the per-wheel comparison is now
unconditional), adds a `wedge=<L>,<R>` TLM field (unconditional, not gated
by config.tlmFields), and adds a one-shot auto re-prime when a wedge is
detected while the drivetrain is at rest.

This file tests all three pieces at the sim level. See also
tests/simulation/system/test_033_005_wedge_hardening.py (the companion
frozen-from-start scenario at the Odometry/dTheta-suppression level) and
tests/simulation/unit/test_golden_tlm.py (wedge= in the byte-exact canary).
"""
import ctypes
import sys
from pathlib import Path

import pytest

from firmware import Sim

# host/ is on sys.path via tests/conftest.py, but be defensive in case this
# file is ever run standalone.
_HOST_DIR = Path(__file__).resolve().parents[3] / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))

from robot_radio.robot.protocol import parse_tlm  # noqa: E402

# ---------------------------------------------------------------------------
# Constants — must stay in sync with firmware source.
# ---------------------------------------------------------------------------

# kWedgeThreshold in MotorController.h
WEDGE_THRESHOLD = 10

TICK_STEP_MS = 24


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _tick_n(s: Sim, n: int, t0: int, step: int = TICK_STEP_MS) -> int:
    """Tick the sim n times; return the updated timestamp."""
    t = t0
    for _ in range(n):
        s._lib.sim_tick(s._h, t)
        t += step
    return t


def _freeze_right(s: Sim) -> None:
    """Freeze right encoder by zeroing its plant speed-offset factor."""
    s._lib.sim_set_motor_offset(s._h, 1, ctypes.c_float(0.0))


def _unfreeze_right(s: Sim) -> None:
    """Restore right encoder offset factor to 1.0."""
    s._lib.sim_set_motor_offset(s._h, 1, ctypes.c_float(1.0))


def _register_reset_count_hooks(lib) -> None:
    """Register argtypes/restype for the 064-003 reset-count sim hooks.

    ctypes caches argtypes/restype per (CDLL instance, function name); each
    fresh Sim() creates a new CDLL wrapper, so this must be (re)done for
    every test's ``sim._lib``.
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


def _hard_reset_count_r(sim: Sim) -> int:
    return int(sim._lib.sim_get_motor_hard_reset_count_r(sim._h))


def _soft_reset_count_r(sim: Sim) -> int:
    return int(sim._lib.sim_get_motor_soft_reset_count_r(sim._h))


# ---------------------------------------------------------------------------
# Blind spot 2: arming grace removed -- a wheel frozen from the START of a
# new command must still arm and fire (Episode A repro).
# ---------------------------------------------------------------------------

def test_frozen_from_start_fires_wedge(sim):
    """A wheel frozen BEFORE a new command starts must still wedge-fire.

    Reproduces Episode A: the right wheel is frozen (offsetFactor=0)
    BEFORE the RT command is even sent, so it never "moves" this episode.
    Prior to 064-004 the 033-005d arming grace (_hasMovedR) blocked
    counting entirely in this exact shape -- EVT enc_wedged fired for NONE
    of ~18 real episodes with this signature. 064-004 removes the grace, so
    counting starts on the very first comparison.
    """
    _freeze_right(sim)

    r = sim.send_command("RT 9000")
    assert "OK" in r.upper(), f"RT failed: {repr(r)}"

    t = 0
    fired_at = None
    for i in range(WEDGE_THRESHOLD + 20):
        t = _tick_n(sim, 1, t)
        if sim.get_wheel_wedged_r():
            fired_at = i
            break

    assert fired_at is not None, (
        f"wheelWedgedR() never fired within {WEDGE_THRESHOLD + 20} ticks of "
        f"R being frozen from the start of a new command -- the 033-005d "
        f"arming grace blind spot has regressed."
    )
    # Left wheel was never frozen -- must not have wedged.
    assert not sim.get_wheel_wedged_l(), (
        "wheelWedgedL() fired even though the left wheel was never frozen."
    )


# ---------------------------------------------------------------------------
# Blind spot 1: target==0 reset removed -- a per-wheel stuck streak must
# survive a boundary where THAT wheel's own commanded target hits 0 while
# the other wheel (and hence the drivetrain overall) keeps driving.
# ---------------------------------------------------------------------------

def test_target_zero_boundary_does_not_reset_streak(sim):
    """A wheel's stuck streak must NOT reset when its own target hits 0.

    Protocol: drive both wheels (S 200 200), freeze the right encoder mid
    -command (so it still counts as "commanded" under the old code), accrue
    part of the stuck streak while R's target is still nonzero, then retarget
    to S 200 0 -- R's OWN commanded target becomes exactly 0 while L (and
    therefore the drivetrain overall) keeps driving. This is exactly the
    "transient tick(s) at a command's own deceleration/stop boundary" blind
    spot: under the old code the tgtR==0.0f branch would have wiped
    _stuckCountR to 0 at that instant.

    Discriminating tick budget: freeze for (WEDGE_THRESHOLD - 3) ticks before
    the boundary, then only 3 more ticks after it -- totalling exactly
    WEDGE_THRESHOLD. If the old target==0 reset were still active, the
    post-boundary streak would restart from 0 and 3 ticks alone could not
    possibly reach WEDGE_THRESHOLD=10 (see the companion negative-control
    test below, which proves 3 ticks from a fresh baseline is insufficient).
    """
    r = sim.send_command("S 200 200")
    assert "OK" in r.upper(), f"S failed: {repr(r)}"

    t = 0
    t = _tick_n(sim, 6, t)   # warm-up: real matched movement on both wheels

    _freeze_right(sim)

    pre_boundary_ticks = WEDGE_THRESHOLD - 3
    t = _tick_n(sim, pre_boundary_ticks, t)
    assert not sim.get_wheel_wedged_r(), (
        f"R wedged after only {pre_boundary_ticks} frozen ticks -- test "
        f"setup issue (should be just under WEDGE_THRESHOLD={WEDGE_THRESHOLD})."
    )

    # Boundary: R's own target crosses to exactly 0 while L keeps driving
    # (driving stays True overall, so refreshedWheel stays 3 and the
    # wedge-check block keeps running every tick).
    r2 = sim.send_command("S 200 0")
    assert "OK" in r2.upper(), f"S 200 0 (boundary) failed: {repr(r2)}"

    t = _tick_n(sim, 3, t)   # exactly enough to complete the streak IF it
                             # was not reset at the boundary (7 + 3 = 10)

    assert sim.get_wheel_wedged_r(), (
        "wheelWedgedR() did not fire after the pre-boundary streak "
        f"({pre_boundary_ticks} ticks) plus 3 post-boundary ticks == "
        f"{WEDGE_THRESHOLD} -- the target==0 reset blind spot has regressed "
        "(the streak was wiped when R's own target crossed to 0)."
    )


def test_three_ticks_alone_cannot_reach_threshold(sim):
    """Negative control for the boundary test above.

    Freezing R with NO pre-boundary streak and ticking only 3 times must NOT
    fire the wedge -- proves that the boundary test's 3 post-boundary ticks
    could only reach WEDGE_THRESHOLD because the pre-boundary streak
    survived, not because 3 ticks alone are sufficient.
    """
    r = sim.send_command("S 200 200")
    assert "OK" in r.upper(), f"S failed: {repr(r)}"
    t = 0
    t = _tick_n(sim, 6, t)

    _freeze_right(sim)
    t = _tick_n(sim, 3, t)

    assert not sim.get_wheel_wedged_r(), (
        "wheelWedgedR() fired after only 3 frozen ticks with no prior "
        "streak -- WEDGE_THRESHOLD is not being respected, so the boundary "
        "test above is not discriminating."
    )


# ---------------------------------------------------------------------------
# wedge= TLM field: unconditional, L-then-R wire order.
# ---------------------------------------------------------------------------

def test_wedge_field_healthy(sim):
    """A fresh, healthy sim reports wedge=0,0 on SNAP."""
    reply = sim.send_command("SNAP")
    frame = parse_tlm(reply)
    assert frame is not None, f"SNAP did not parse as TLM: {repr(reply)}"
    assert frame.wedge == (0, 0), (
        f"expected wedge=(0, 0) for a healthy, idle sim; got {frame.wedge} "
        f"(raw: {repr(reply)})"
    )


def test_wedge_field_reflects_right_latch(sim):
    """Forcing a right-wheel wedge shows up as wedge=0,1 in the next SNAP."""
    r = sim.send_command("RT 9000")
    assert "OK" in r.upper(), f"RT failed: {repr(r)}"
    _freeze_right(sim)

    t = 0
    for _ in range(WEDGE_THRESHOLD + 20):
        t = _tick_n(sim, 1, t)
        if sim.get_wheel_wedged_r():
            break
    assert sim.get_wheel_wedged_r(), "setup: R never wedged"

    reply = sim.send_command("SNAP")
    frame = parse_tlm(reply)
    assert frame is not None, f"SNAP did not parse as TLM: {repr(reply)}"
    assert frame.wedge == (0, 1), (
        f"expected wedge=(0, 1) (left healthy, right latched); got "
        f"{frame.wedge} (raw: {repr(reply)})"
    )


# ---------------------------------------------------------------------------
# Auto re-prime at idle: one-shot resetEncoderAccumulators() when a wedge is
# detected while the drivetrain is at rest.
# ---------------------------------------------------------------------------

def test_auto_reprime_fires_once_at_idle_and_clears_latch(sim):
    """A wedge that persists into an idle drivetrain triggers exactly one
    automatic re-prime, which clears the observable latch.
    """
    _register_reset_count_hooks(sim._lib)

    r = sim.send_command("S 200 200")
    assert "OK" in r.upper(), f"S failed: {repr(r)}"
    t = 0
    t = _tick_n(sim, 6, t)

    _freeze_right(sim)
    t = _tick_n(sim, WEDGE_THRESHOLD + 1, t)
    assert sim.get_wheel_wedged_r(), "setup: R never wedged while driving"

    before_hard = _hard_reset_count_r(sim)
    before_soft = _soft_reset_count_r(sim)

    # Stop -- the drivetrain transitions to genuinely at rest.
    r2 = sim.send_command("X")
    assert "OK" in r2.upper(), f"X failed: {repr(r2)}"

    # A couple of idle ticks are enough for the at-rest decision's measured
    # (velocity) component to catch up to the commanded (target) component
    # -- see MotorController::computeAtRest().
    t = _tick_n(sim, 5, t)

    after_hard = _hard_reset_count_r(sim)
    after_soft = _soft_reset_count_r(sim)

    assert after_hard == before_hard + 1, (
        f"hardResetCount(R) expected to increment by exactly 1 after the "
        f"drivetrain went idle with a persistent wedge; before={before_hard}, "
        f"after={after_hard}"
    )
    assert after_soft == before_soft, (
        f"softResetCount(R) unexpectedly changed for an at-rest re-prime "
        f"(should take the hardware path): before={before_soft}, "
        f"after={after_soft}"
    )
    assert not sim.get_wheel_wedged_r(), (
        "wheelWedgedR() still latched after the auto re-prime -- the "
        "explicit resetStuckCounters() call is required because Drive's "
        "idle path never re-collects the encoder to observe the reset "
        "naturally."
    )

    # One-shot: further idle ticks must NOT attempt another re-prime.
    t = _tick_n(sim, 40, t)
    assert _hard_reset_count_r(sim) == after_hard, (
        "hardResetCount(R) incremented again during a continued idle period "
        "-- the auto re-prime must fire at most once per episode."
    )

    # Recovery sanity: clear the injected fault and confirm a fresh command
    # drives normally with no lingering latch.
    _unfreeze_right(sim)
    r3 = sim.send_command("S 150 150")
    assert "OK" in r3.upper(), f"S (recovery) failed: {repr(r3)}"
    t = _tick_n(sim, 10, t)
    assert not sim.get_wheel_wedged_r(), (
        "wheelWedgedR() latched again immediately after the fault was "
        "cleared and a fresh command issued -- recovery did not hold."
    )


def test_auto_reprime_does_not_fire_while_moving(sim):
    """A wedge that fires WHILE STILL DRIVING must not trigger a re-prime."""
    _register_reset_count_hooks(sim._lib)

    r = sim.send_command("S 200 200")
    assert "OK" in r.upper(), f"S failed: {repr(r)}"
    t = 0
    t = _tick_n(sim, 6, t)

    _freeze_right(sim)
    t = _tick_n(sim, WEDGE_THRESHOLD + 1, t)
    assert sim.get_wheel_wedged_r(), "setup: R never wedged while driving"

    before_hard = _hard_reset_count_r(sim)
    assert before_hard == 0, (
        f"test setup: expected no prior hard reset, got {before_hard}"
    )

    # Keep driving (both wheels still commanded nonzero) for a long window
    # while the wedge stays latched -- the drivetrain is never at rest.
    t = _tick_n(sim, 40, t)

    after_hard = _hard_reset_count_r(sim)
    assert after_hard == before_hard, (
        f"hardResetCount(R) incremented while the drivetrain was still "
        f"driving (never at rest) -- the auto re-prime gate "
        f"(MotorController::isAtRest()) is not being respected: "
        f"before={before_hard}, after={after_hard}"
    )
    assert sim.get_wheel_wedged_r(), (
        "wheelWedgedR() cleared while still driving with no re-prime "
        "attempted -- unexpected."
    )
