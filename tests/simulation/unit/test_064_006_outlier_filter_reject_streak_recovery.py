"""
test_064_006_outlier_filter_reject_streak_recovery.py — 064-006 regression tests.

CR-02 (clasi/issues/encoder-integrity-i2c-failures-and-outlier-filter-recovery.md,
part (b)): Drive::_runOutlierFilter() (source/subsystems/drive/Drive.cpp)
rejects any per-tick delta > max(40mm, 0.2*target) and holds the previous
value; _filterRejectStreakL/R incremented on each rejection but were NEVER
consumed -- kFilterRejectStreakThreshold (3) sat declared-but-unused since the
sprint-060 ordered-tick cutover. A persistent divergence (e.g. a wheel
hand-rolled while idle) was therefore rejected FOREVER: every fresh read
differed from the same stale baseline by the same large delta, so the filter
held _hw.encMm[] frozen indefinitely and downstream odometry froze while
commanded motion continued. The filter's whole block was also gated
`if (driving)`, so _hw.encMm[] was never refreshed while idle -- guaranteeing
the freeze on the very next command after a hand-roll.

The fix (this ticket):
  1. Reject-streak rebaseline: once kFilterRejectStreakThreshold consecutive
     rejections accumulate on a wheel, accept the already-computed fresh
     reading as the new baseline and reset the streak to 0.
  2. Idle refresh: while NOT driving, _hw.encMm[] is refreshed unconditionally
     (no outlier gate) every tick from positionMm() -- see
     architecture-update.md Design Rationale 5.

Test-harness note: sim_set_enc_l/r (the existing injection hook) ALSO syncs
Drive's private outlier-filter baseline (_hw.encMm[]) in the same call
(injectEncL/R), which trivially "fixes" any divergence without exercising
either of the above paths. These tests instead use the new
sim_set_reported_enc_l/r hook (064-006), which touches ONLY the plant's
reported-encoder accumulator -- exactly what SimMotor::tick() promotes into
positionMm(), mirroring the real Motor's continuously-refreshed
_lastPositionMm -- leaving _hw.encMm[] deliberately stale. That is the
"hand-rolled wheel" / "diverged sensor" precondition both fixes must recover
from.
"""
from __future__ import annotations

import ctypes
import math

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _enc(sim) -> tuple[float, float]:
    """Return (encL, encR) -- Drive's outlier-filtered encoder cache (mm)."""
    l = float(sim._lib.sim_get_enc_l(sim._h))
    r = float(sim._lib.sim_get_enc_r(sim._h))
    return (l, r)


def _tick_until_evt(sim, needle: str, total_ms: int, step_ms: int = 24) -> str:
    """Advance sim, draining async EVTs, until `needle` is seen or time runs out."""
    evts = ""
    elapsed = 0
    while elapsed < total_ms:
        sim.tick_for(step_ms, step_ms=step_ms)
        elapsed += step_ms
        evts += sim.get_async_evts()
        if needle in evts:
            break
    return evts


# ---------------------------------------------------------------------------
# Test 1: idle refresh absorbs a hand-roll BEFORE the next command starts.
# ---------------------------------------------------------------------------

def test_hand_roll_while_idle_absorbed_before_next_turn(sim):
    """A wheel hand-rolled while idle must not freeze the outlier filter (and
    therefore odometry) on the very next command.

    Regression for CR-02's idle-gap defect: _runOutlierFilter only ran
    `if (driving)`, so _hw.encMm[] was never refreshed while idle. A wheel
    hand-rolled while parked left a stale baseline more than kMaxDeltaMm away
    from the sensor's true position; the next command's every fresh read was
    rejected forever, freezing encoder-derived odometry while the commanded
    turn kept running (the "freakout" scenario in the filed issue).
    """
    sim.set_perfect()

    # Confirm genuinely idle (fresh sim, never driven) before injecting.
    vel_l = float(sim._lib.sim_get_vel_l(sim._h))
    vel_r = float(sim._lib.sim_get_vel_r(sim._h))
    assert abs(vel_l) < 1.0 and abs(vel_r) < 1.0, (
        f"test setup: expected the drivetrain idle, got vel_l={vel_l:.1f}, "
        f"vel_r={vel_r:.1f}"
    )
    enc_l0, enc_r0 = _enc(sim)

    # Simulate the operator hand-rolling the (unpowered) wheels while idle:
    # the physical sensor now reports a new position. Drive's cached baseline
    # (_hw.encMm[]) is deliberately left stale -- see module docstring on why
    # sim_set_enc_l/r cannot be used here.
    hand_rolled_l = enc_l0 + 250.0
    hand_rolled_r = enc_r0 - 180.0
    sim._lib.sim_set_reported_enc_l(sim._h, ctypes.c_float(hand_rolled_l))
    sim._lib.sim_set_reported_enc_r(sim._h, ctypes.c_float(hand_rolled_r))

    # Idle refresh must absorb the jump into the baseline on the very next
    # tick -- no command has been issued yet.
    sim.tick_for(24, step_ms=24)
    enc_l1, enc_r1 = _enc(sim)
    assert enc_l1 == pytest.approx(hand_rolled_l, abs=2.0), (
        f"idle refresh did not absorb the hand-rolled left position: "
        f"expected ~{hand_rolled_l:.1f}, got {enc_l1:.1f}"
    )
    assert enc_r1 == pytest.approx(hand_rolled_r, abs=2.0), (
        f"idle refresh did not absorb the hand-rolled right position: "
        f"expected ~{hand_rolled_r:.1f}, got {enc_r1:.1f}"
    )

    # Now command a TURN. With the baseline already synced pre-command, the
    # outlier filter's fresh reads at command start land close to the
    # (already-updated) baseline and are accepted immediately -- odometry
    # tracks the turn instead of freezing (the bug: every read rejected
    # forever, TURN spins to the TIME net at the wrong heading).
    r = sim.send_command("TURN 9000")  # 90 degrees
    assert "OK" in r.upper(), f"TURN failed: {repr(r)}"

    evts = _tick_until_evt(sim, "EVT done TURN", 8_000)
    assert "EVT done TURN" in evts, (
        f"TURN never completed -- looks like the encoder pipeline froze "
        f"instead of tracking the turn: {repr(evts)}"
    )

    final_h_rad = float(sim._lib.sim_get_pose_h(sim._h))
    assert final_h_rad > math.radians(45.0), (
        f"fused heading only reached {math.degrees(final_h_rad):.1f} deg "
        f"after 'EVT done TURN' -- the encoder pipeline looks frozen rather "
        f"than having tracked the turn."
    )


# ---------------------------------------------------------------------------
# Test 2: a persistent in-drive divergence rebaselines at exactly the
# kFilterRejectStreakThreshold-th consecutive rejection, not before.
# ---------------------------------------------------------------------------

def test_persistent_divergence_rebaselines_at_threshold_not_before(sim):
    """During an active command, 3 consecutive large-delta rejections on one
    wheel must rebaseline to the fresh reading; the first two must NOT.

    Regression for CR-02's main defect: _filterRejectStreakL/R incremented
    but were never consumed, so a persistent divergence (three-plus
    consecutive rejections) held the stale baseline forever instead of
    escaping via the already-declared kFilterRejectStreakThreshold (3).
    """
    sim.set_perfect()

    r = sim.send_command("VW 200 0")  # straight forward, both wheels ~200 mm/s
    assert "OK" in r.upper(), f"VW failed: {repr(r)}"
    sim.tick_for(240, step_ms=24)  # get up to speed, establish a tracking baseline

    enc_l0, enc_r0 = _enc(sim)

    # Inject a persistent large jump into the RIGHT wheel's REPORTED encoder
    # only (bypassing Drive's baseline) -- kMaxDeltaMm here is
    # max(40, 0.2*200) = 40mm, so 300mm is comfortably over the reject
    # threshold and stays there every tick (constant injected value + only
    # a few mm/tick of ongoing commanded travel).
    jump_r = enc_r0 + 300.0
    sim._lib.sim_set_reported_enc_r(sim._h, ctypes.c_float(jump_r))

    # Tick 1: first consecutive rejection -- filter holds the stale baseline.
    sim.tick_for(24, step_ms=24)
    enc_r_t1 = _enc(sim)[1]
    assert enc_r_t1 == pytest.approx(enc_r0, abs=5.0), (
        f"filter accepted the diverged reading on the very first rejection "
        f"(streak=1): enc_r0={enc_r0:.1f}, enc_r_t1={enc_r_t1:.1f}"
    )

    # Tick 2: second consecutive rejection -- still held (streak=2 < 3).
    sim.tick_for(24, step_ms=24)
    enc_r_t2 = _enc(sim)[1]
    assert enc_r_t2 == pytest.approx(enc_r0, abs=5.0), (
        f"filter rebaselined before reaching kFilterRejectStreakThreshold "
        f"(streak=2): enc_r0={enc_r0:.1f}, enc_r_t2={enc_r_t2:.1f}"
    )

    # Tick 3: THIRD consecutive rejection reaches kFilterRejectStreakThreshold
    # -- filter rebaselines to the fresh (diverged) reading.
    sim.tick_for(24, step_ms=24)
    enc_r_t3 = _enc(sim)[1]
    assert abs(enc_r_t3 - jump_r) < 50.0, (
        f"filter did not rebaseline at the 3rd consecutive rejection: "
        f"expected close to jump_r={jump_r:.1f}, got enc_r_t3={enc_r_t3:.1f} "
        f"(still near stale baseline enc_r0={enc_r0:.1f}?)"
    )

    # The healthy left wheel was never touched -- must have kept tracking
    # normally throughout (the fix is scoped to the diverged wheel only).
    enc_l3 = _enc(sim)[0]
    assert (enc_l3 - enc_l0) > 5.0, (
        f"left encoder (never injected) barely moved during the right-wheel "
        f"divergence: {enc_l0:.1f} -> {enc_l3:.1f}"
    )


# ---------------------------------------------------------------------------
# Test 3: the streak resets on an accepted tick -- two non-consecutive pairs
# of rejections (with a recovered tick in between) must never rebaseline.
# ---------------------------------------------------------------------------

def test_streak_resets_on_recovery_never_rebaselines_below_threshold(sim):
    """A recovered tick between two short (2-tick) divergence runs must reset
    the streak, so neither run alone reaches the rebaseline threshold.

    This is the "engages at exactly the threshold, not before" regression
    guard: without the streak reset, a cumulative count of rejections spread
    across two short runs could reach 3 and spuriously rebaseline; the streak
    must be reset by any accepted (non-rejected) tick in between.
    """
    sim.set_perfect()

    r = sim.send_command("VW 200 0")
    assert "OK" in r.upper(), f"VW failed: {repr(r)}"
    sim.tick_for(240, step_ms=24)

    enc_r0 = _enc(sim)[1]

    # First short divergence run: 2 consecutive rejections (streak -> 2, not
    # yet at threshold).
    jump_r1 = enc_r0 + 300.0
    sim._lib.sim_set_reported_enc_r(sim._h, ctypes.c_float(jump_r1))
    sim.tick_for(24, step_ms=24)
    sim.tick_for(24, step_ms=24)
    enc_r_after_run1 = _enc(sim)[1]
    assert enc_r_after_run1 == pytest.approx(enc_r0, abs=5.0), (
        f"test setup: expected the baseline still held after 2 consecutive "
        f"rejections, got {enc_r_after_run1:.1f} (baseline was {enc_r0:.1f})"
    )

    # Recovery tick: the sensor reports a value close to the CURRENT
    # baseline again (not a persistent divergence) -- the retry-then-hold
    # path accepts it, resetting the streak to 0.
    recovered_r = enc_r_after_run1 + 5.0
    sim._lib.sim_set_reported_enc_r(sim._h, ctypes.c_float(recovered_r))
    sim.tick_for(24, step_ms=24)
    enc_r_recovered = _enc(sim)[1]
    assert enc_r_recovered == pytest.approx(recovered_r, abs=5.0), (
        f"filter did not accept the recovered (in-range) reading: "
        f"expected ~{recovered_r:.1f}, got {enc_r_recovered:.1f}"
    )

    # Second short divergence run: 2 MORE consecutive rejections. If the
    # streak had not reset above, this would be consecutive rejection #4/#5
    # overall (well past threshold) and would already show a rebaseline. If
    # it reset correctly, this is only a fresh 2-tick run -- must still be
    # frozen, not rebaselined.
    jump_r2 = enc_r_recovered + 300.0
    sim._lib.sim_set_reported_enc_r(sim._h, ctypes.c_float(jump_r2))
    sim.tick_for(24, step_ms=24)
    sim.tick_for(24, step_ms=24)
    enc_r_after_run2 = _enc(sim)[1]
    assert enc_r_after_run2 == pytest.approx(enc_r_recovered, abs=5.0), (
        f"streak did not reset after the intervening accepted tick -- "
        f"rebaselined before completing a fresh 3-consecutive streak: "
        f"expected ~{enc_r_recovered:.1f}, got {enc_r_after_run2:.1f} "
        f"(jump target was {jump_r2:.1f})"
    )
