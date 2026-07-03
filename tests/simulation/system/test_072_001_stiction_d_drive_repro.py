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

This test is the repro VEHICLE tickets 002/003 are validated against.

Empirically determined (`SIMSET stictionPwmL=28 stictionPwmR=28`, deterministic
-- the sim has no wall-clock/real-RNG dependency for this scenario, confirmed
by running twice and diffing byte-for-byte): against the ORIGINAL (pre-003)
control code, a scripted `D 200 200 500` decelerates normally, then the mean
encoder distance sticks at 498.91 mm (1.09 mm short of the 500 mm target --
squarely inside the field-recorded 1-3 mm short range) with both wheel
velocities pinned at EXACTLY 0 mm/s for ~2.16 s (the velocity controller's PI
output settles at a constant PWM below the stiction threshold once the
ratcheted BVC target enters `minWheelSpeed`'s integrator-freeze deadband --
see architecture-update.md Step 1b's root-cause chain). `StopCondition::
DISTANCE`'s strict `>=` crossing never fires during the stall; the command
only completes because the generous TIME safety net (2x nominal + 2 s)
eventually times out (`EVT done D reason=time`), itself still 1.09 mm short
of the commanded distance. The identical scripted `D` command with the
stiction knob at its default (0, no-op) completes cleanly via `reason=dist`
in ~3.3 s with no stall at all -- proving the land-short/stall signature is
caused BY the configured stiction, not some other property of the `D`
command.

**072-003 update:** `test_stiction_reproduces_d_drive_land_short_stall_
signature` below originally asserted the paragraph above's PRE-FIX, KNOWN-
BUGGY behavior verbatim (stall >= 1.5 s, `reason=time`) as a deliberately-
captured defect, with a note that ticket 004 would flip the assertion once
002/003 landed. Ticket 003's terminal-completion guarantee (v_cap floored at
`minWheelSpeed`; a stalled-short forced completion via `MotionCommand::
forceComplete()` once `d_remaining` sits inside `distArriveTol` with no
progress for `stallConfirm` ms) now fires WHILE this same file is still on
`in-progress` -- leaving the old assertions in place would fail the suite
immediately, not at some later ticket 004 checkpoint, so ticket 003 updates
them here as the minimal anticipated consequence (071-002's own precedent for
touching a pre-existing test file outside its nominal ticket boundary to stay
green). The stiction gate itself (this ticket's own repro vehicle) and the
plain no-stiction control case are UNCHANGED below; only the buggy-completion
assertions are updated to the now-fixed behavior. See ticket 072-003's own
new test file, `test_072_003_terminal_completion_guarantee.py`, for the full
before/after comparison and the no-reversal/no-thrash/well-before-TIME-net
assertions this summary-level update does not repeat.
"""
from __future__ import annotations


def _drive_and_watch(sim, target_mm: float, total_ms: int, step_ms: int = 24) -> dict:
    """Run one already-issued `D` command to completion (or `total_ms`),
    watching for a stall (both wheels pinned at ~0 mm/s while still short of
    the target) and recording how/when the command actually completes.

    Returns a dict with:
      done_reason:  "dist" | "time" | "arrive" | "other:<evts>" | None (never
                    completed) -- "arrive" is the 072-003 stalled-short
                    forced-completion token.
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
            elif "reason=arrive" in evts:
                done_reason = "arrive"
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
    short of the target -- the exact field failure signature -- but (072-003
    update) now completes CLEANLY instead of stalling for seconds and timing
    out.

    FIXED (072-003) behavior, asserted here: the terminal `v_cap` floor and
    the stalled-short forced-completion path (`MotionCommand::
    forceComplete()`, `EVT done D reason=arrive`) mean the drive never
    accumulates a multi-second stall before completing -- the stall-confirm
    debounce (default 300 ms) fires once `d_remaining` sits inside
    `distArriveTol` (default 5 mm) with no progress, well before the
    generous TIME net (7000 ms for this `D 200 200 500`) would ever have
    fired. See `test_072_003_terminal_completion_guarantee.py` for the full
    no-reversal/no-thrash/well-before-TIME-net proof; this test only
    confirms the stiction gate (this ticket's own repro vehicle) still
    reproduces the land-short signature and that it no longer stalls/times
    out to get there.
    """
    reply = sim.send_command("SIMSET stictionPwmL=28 stictionPwmR=28")
    assert reply.upper().startswith("OK"), f"SIMSET rejected: {reply!r}"

    reply = sim.send_command("D 200 200 500")
    assert "OK drive" in reply, f"D command rejected: {reply!r}"

    result = _drive_and_watch(sim, target_mm=500.0, total_ms=8000)

    # --- 072-003 fix: no more sustained (>= 1.5 s) stall. --------------------
    assert result["max_stall_ms"] < 1000, (
        f"072-003's stall-confirm debounce (default 300 ms) should force a "
        f"clean completion long before a multi-second stall could "
        f"accumulate: {result}"
    )

    # --- Clean forced completion: reason=arrive, still measurably short of ---
    # --- the target (the documented, bounded, intentional under-travel). ---
    assert result["done_reason"] == "arrive", (
        f"expected the 072-003 stalled-short forced-completion path "
        f"(reason=arrive), not a TIME-net completion or a strict crossing: "
        f"{result}"
    )
    assert result["done_avg"] is not None and 495.0 < result["done_avg"] < 500.0, (
        f"a forced completion should land measurably short of the 500 mm "
        f"target (field data: 1-3 mm short) but within distArriveTol "
        f"(default 5 mm): {result}"
    )
    assert result["elapsed_ms"] < 6000, (
        f"the forced completion should land well before the ~7000 ms TIME "
        f"net for this D 200 200 500: {result}"
    )
