"""
test_072_001_stiction_d_drive_repro.py — ticket 072-001.

Reproduces `d-drive-terminal-instability-reversal-thrash.md`'s field failure
signature -- a `D` drive that decelerates cleanly, then lands 1-3 mm SHORT of
its target, stalls at near-zero speed, and only completes much later -- in
sim, running the ACTUAL firmware control code (`Planner`'s D-mode decel hook,
`VelocityController`, `StopCondition::DISTANCE`), configured via the new
`SIMSET stictionPwmL/R=...` knob this ticket adds to `PhysicsWorld`. This is
NOT the forced-encoder-cap harness the issue used for diagnosis (which
bypasses the plant and directly pins reported encoders each tick) -- this
test drives a real, scripted `D` command against a plant that can genuinely
fail to respond to small PWM, exactly the gap architecture-update.md Step 1c
identifies.

This test is the repro VEHICLE tickets 002/003 are validated against; ticket
004 later flips its "completes via the TIME safety net, short of target"
assertion once the signed-DISTANCE-stop (002) and terminal-completion (003)
fixes land. **The behavior asserted here is the CURRENT, PRE-FIX, KNOWN-BUGGY
firmware behavior** -- it is deliberately captured, not treated as correct.

Empirically determined (`SIMSET stictionPwmL=28 stictionPwmR=28`, deterministic
-- the sim has no wall-clock/real-RNG dependency for this scenario, confirmed
by running twice and diffing byte-for-byte): a scripted `D 200 200 500`
decelerates normally, then the mean encoder distance sticks at 498.91 mm (1.09
mm short of the 500 mm target -- squarely inside the field-recorded 1-3 mm
short range) with both wheel velocities pinned at EXACTLY 0 mm/s for ~2.16 s
(the velocity controller's PI output settles at a constant PWM below the
stiction threshold once the ratcheted BVC target enters `minWheelSpeed`'s
integrator-freeze deadband -- see architecture-update.md Step 1b's root-cause
chain). `StopCondition::DISTANCE`'s strict `>=` crossing never fires during
the stall; the command only completes because the generous TIME safety net
(2x nominal + 2 s) eventually times out (`EVT done D reason=time`), itself
still 1.09 mm short of the commanded distance. The identical scripted `D`
command with the stiction knob at its default (0, no-op) completes cleanly
via `reason=dist` in ~3.3 s with no stall at all -- proving the land-short/
stall signature is caused BY the configured stiction, not some other
property of the `D` command.
"""
from __future__ import annotations


def _drive_and_watch(sim, target_mm: float, total_ms: int, step_ms: int = 24) -> dict:
    """Run one already-issued `D` command to completion (or `total_ms`),
    watching for a stall (both wheels pinned at ~0 mm/s while still short of
    the target) and recording how/when the command actually completes.

    Returns a dict with:
      done_reason:  "dist" | "time" | "other:<evts>" | None (never completed)
      done_avg:     mean (enc_l+enc_r)/2 at the SAME tick the completion EVT
                    fired (captured before any post-completion reset).
      stall_avg:    the mean encoder distance during the longest observed
                    near-zero-velocity-while-short-of-target run, or None if
                    no such stall was observed.
      max_stall_ms: duration of that longest stall run, in ms.
    """
    t = 0
    done_reason = None
    done_avg = None
    stall_avg = None
    stall_ticks = 0
    max_stall_ticks = 0
    while t < total_ms:
        sim.tick_for(step_ms, step_ms=step_ms)
        t += step_ms

        enc_l = float(sim._lib.sim_get_enc_l(sim._h))
        enc_r = float(sim._lib.sim_get_enc_r(sim._h))
        avg = (enc_l + enc_r) * 0.5
        vel_l = float(sim._lib.sim_get_vel_l(sim._h))

        evts = sim.get_async_evts()

        if done_reason is None and abs(vel_l) < 0.001 and 0.0 < avg < target_mm:
            stall_ticks += 1
            if stall_ticks * step_ms >= max_stall_ticks * step_ms:
                stall_avg = avg
                max_stall_ticks = stall_ticks
        else:
            stall_ticks = 0

        if done_reason is None and "EVT done D" in evts:
            done_avg = avg
            if "reason=dist" in evts:
                done_reason = "dist"
            elif "reason=time" in evts:
                done_reason = "time"
            else:
                done_reason = f"other:{evts}"
            break

    return {
        "elapsed_ms": t,
        "done_reason": done_reason,
        "done_avg": done_avg,
        "stall_avg": stall_avg,
        "max_stall_ms": max_stall_ticks * step_ms,
    }


def test_baseline_no_stiction_completes_cleanly_no_stall(sim) -> None:
    """Control case: at the stiction default (0 = no-op), the SAME scripted
    `D 200 200 500` completes via a clean DISTANCE crossing (`reason=dist`),
    at/over the target, with no stall -- establishing that the stiction
    knob (not the `D` command itself, the sim tick granularity, or any other
    confound) is what causes the land-short/stall signature in the next test.
    """
    reply = sim.send_command("D 200 200 500")
    assert "OK drive" in reply, f"D command rejected: {reply!r}"

    result = _drive_and_watch(sim, target_mm=500.0, total_ms=6000)

    assert result["done_reason"] == "dist", (
        f"baseline (no stiction) should complete via a clean DISTANCE "
        f"crossing: {result}"
    )
    assert result["done_avg"] is not None and result["done_avg"] >= 500.0, (
        f"a clean DISTANCE crossing must land AT/OVER the target: {result}"
    )
    assert result["max_stall_ms"] < 200, (
        f"baseline should show no meaningful stall (this repro's whole point "
        f"is that a zero-stiction plant CANNOT stall short): {result}"
    )


def test_stiction_reproduces_d_drive_land_short_stall_signature(sim) -> None:
    """Main repro: with `stictionPwmL/R` configured above the PWM the D-mode
    decel hook commands in its final approach, the D drive lands measurably
    short of the target and stalls -- the exact field failure signature.

    CURRENT (pre-002/003) BUGGY BEHAVIOR, asserted deliberately: the
    DISTANCE stop's strict `>=` crossing never fires during the stall, so
    the command only completes via the TIME safety net, itself still short
    of the commanded distance. Ticket 002 (signed DISTANCE stop) and ticket
    003 (terminal-completion guarantee) are what ticket 004 will validate
    against a FLIPPED version of the completion-path assertions below --
    this test's stall-detection assertions (the actual repro) are expected
    to keep passing since they exercise the same stiction-gated plant.
    """
    reply = sim.send_command("SIMSET stictionPwmL=28 stictionPwmR=28")
    assert reply.upper().startswith("OK"), f"SIMSET rejected: {reply!r}"

    reply = sim.send_command("D 200 200 500")
    assert "OK drive" in reply, f"D command rejected: {reply!r}"

    result = _drive_and_watch(sim, target_mm=500.0, total_ms=8000)

    # --- The repro: a sustained stall, short of the target. -----------------
    assert result["max_stall_ms"] >= 1500, (
        f"expected a sustained (>= 1.5 s) stall at near-zero velocity while "
        f"still short of the 500 mm target -- the field-recorded "
        f"'stalls 0.3-0.5 s' signature (this sim repro's stall runs longer "
        f"since nothing here ever breaks the integrator-freeze deadband via "
        f"noise/disturbance): {result}"
    )
    assert result["stall_avg"] is not None and 480.0 < result["stall_avg"] < 499.9, (
        f"stall plateau should be measurably SHORT of the 500 mm target "
        f"(field data: 1-3 mm short) -- got {result}"
    )

    # --- Current buggy completion path: TIME net, not a clean crossing. -----
    assert result["done_reason"] == "time", (
        f"expected the CURRENT buggy behavior: the DISTANCE stop's strict "
        f">= crossing never fires during the stall, so completion only "
        f"happens via the generous TIME safety net -- ticket 002/003 fix "
        f"this; ticket 004 flips this assertion once they land: {result}"
    )
    assert result["done_avg"] is not None and result["done_avg"] < 499.9, (
        f"the TIME-net completion should still land measurably SHORT of the "
        f"500 mm target -- this IS the 'lands short' defect, not a "
        f"coincidence of when the timeout happens to fire: {result}"
    )
