"""
test_072_003_terminal_completion_guarantee.py — ticket 072-003.

D-mode terminal-completion guarantee: (a) floors the D-mode decel hook's
terminal `v_cap` at `minWheelSpeed` instead of letting it asymptote toward
zero as `d_remaining` -> 0 (`Planner.cpp driveAdvance`, DISTANCE branch); (b)
a stalled-short forced-completion path -- if `d_remaining` sits inside the
new `distArriveTol` (mm) and stops shrinking for `stallConfirm` (ms)
consecutive ticks, the drive completes NOW via the new
`MotionCommand::forceComplete()` entry point (`EVT done D reason=arrive`)
instead of leaving the down-only ratchet pinned indefinitely.

Validates against ticket 001's stiction repro
(`test_072_001_stiction_d_drive_repro.py`): the SAME
`SIMSET stictionPwmL=28 stictionPwmR=28` + `D 200 200 500` scenario --
which used to stall ~2.16 s at 498.91 mm and only complete via the
multi-second TIME net (`reason=time`, still short of target) -- must now
complete CLEANLY: within `distArriveTol` of the target, at rest, with NO
backward travel and NO thrash anywhere in the drive, well before the TIME
net.

Also proves the terminal-completion path is provably INERT against the
ORIGINAL zero-stiction plant (architecture-update.md Decision 6 / SUC-004's
postcondition): `d_remaining` reaches exactly zero via the strict DISTANCE
crossing before the stall-confirm window could ever elapse, because the
shrink-tracking mechanism (Planner.cpp driveAdvance DISTANCE branch) resets
its debounce timer every tick the robot makes ANY forward progress. A second
control test drives this home structurally (not just empirically for the
default tuning): even with an exaggerated `distArriveTol`/`stallConfirm`,
the mechanism still cannot misfire while progress continues.
"""
from __future__ import annotations

TICK_STEP_MS = 24


def _drive_and_trace(sim, total_ms: int, step_ms: int = TICK_STEP_MS) -> dict:
    """Run one already-issued `D` command, recording the FULL avg-encoder
    trace (for monotonicity / no-reversal checking) and how/when it
    completes.

    Returns a dict with:
      elapsed_ms:    total sim time advanced before completion (or total_ms
                     if it never completed).
      done_evt:      the full accumulated EVT string from the tick the
                     command terminated (EVT done D ... or EVT safety_stop
                     ...), or None if it never completed within total_ms.
      done_avg:      mean (enc_l+enc_r)/2 at the tick the terminal EVT fired.
      trace:         list of avg encoder distance, one entry per tick.
      vel_l_at_done: sim_get_vel_l() reading at the terminal tick (rest
                     check).
    """
    t = 0
    done_evt = None
    done_avg = None
    vel_l_at_done = None
    trace: list[float] = []

    while t < total_ms:
        sim.tick_for(step_ms, step_ms=step_ms)
        t += step_ms

        enc_l = float(sim._lib.sim_get_enc_l(sim._h))
        enc_r = float(sim._lib.sim_get_enc_r(sim._h))
        avg = (enc_l + enc_r) * 0.5
        trace.append(avg)

        evts = sim.get_async_evts()
        if done_evt is None and ("EVT done D" in evts or "EVT safety_stop" in evts):
            done_avg = avg
            vel_l_at_done = float(sim._lib.sim_get_vel_l(sim._h))
            done_evt = evts
            break

    return {
        "elapsed_ms": t,
        "done_evt": done_evt,
        "done_avg": done_avg,
        "trace": trace,
        "vel_l_at_done": vel_l_at_done,
    }


def _assert_no_backward_travel(trace: list[float], eps: float = 0.05) -> None:
    """Assert the avg-encoder trace never decreases tick-to-tick beyond
    float noise -- i.e. no reversal, no thrash, anywhere in the drive."""
    for i in range(1, len(trace)):
        assert trace[i] >= trace[i - 1] - eps, (
            f"backward travel detected at tick {i}: "
            f"{trace[i - 1]:.4f} -> {trace[i]:.4f} mm "
            f"(trace window: {trace[max(0, i - 5):i + 5]})"
        )


# ---------------------------------------------------------------------------
# Primary validation: ticket 001's stiction repro now completes cleanly.
# ---------------------------------------------------------------------------

def test_stiction_stall_now_completes_cleanly_no_reversal_no_thrash(sim) -> None:
    """The exact `SIMSET stictionPwmL=28 stictionPwmR=28` + `D 200 200 500`
    scenario from ticket 001's repro -- which used to stall ~2.16 s at
    498.91 mm and only complete via the TIME net (`reason=time`) -- must now
    complete cleanly: within `distArriveTol` of the target, at rest, with no
    backward travel anywhere in the trace, well before the ~7000 ms TIME
    net for this `D 200 200 500`.
    """
    reply = sim.send_command("SIMSET stictionPwmL=28 stictionPwmR=28")
    assert reply.upper().startswith("OK"), f"SIMSET rejected: {reply!r}"

    reply = sim.send_command("D 200 200 500")
    assert "OK drive" in reply, f"D command rejected: {reply!r}"

    result = _drive_and_trace(sim, total_ms=6000)

    assert result["done_evt"] is not None, (
        f"drive never completed within 6 s (TIME net is ~7 s for this "
        f"D 200 200 500): {result}"
    )
    assert "EVT safety_stop" not in result["done_evt"], (
        f"must not trip the runaway safety net: {result}"
    )
    assert "EVT done D" in result["done_evt"], (
        f"expected a normal EVT done D completion: {result}"
    )
    assert (
        "reason=arrive" in result["done_evt"] or "reason=dist" in result["done_evt"]
    ), (
        f"expected reason=arrive (the 072-003 stalled-short forced "
        f"completion) or reason=dist (a clean crossing, if the v_cap floor "
        f"alone broke the stiction) -- NOT reason=time: {result}"
    )
    assert result["elapsed_ms"] < 6000, (
        f"drive did not complete within the 6 s window (TIME net is ~7 s): "
        f"{result}"
    )

    # No backward ramp / reversal / thrash anywhere in the drive -- the
    # exact field failure (stall -> reverse -> thrash -> lunge) this ticket
    # fixes.
    _assert_no_backward_travel(result["trace"])

    # Lands at/near the target: within distArriveTol (default 5 mm) short,
    # or at/over it (a clean crossing).
    assert result["done_avg"] is not None and result["done_avg"] >= 495.0, (
        f"should land within distArriveTol of the 500 mm target: {result}"
    )

    # At rest: the wheel is not still commanding meaningful speed when the
    # completion EVT fires (SOFT ramp-down converges to (0,0) before EVT).
    assert (
        result["vel_l_at_done"] is not None and abs(result["vel_l_at_done"]) < 5.0
    ), f"completion should occur at/near rest, not mid-ramp: {result}"


# ---------------------------------------------------------------------------
# Control: the ORIGINAL zero-stiction plant is provably unaffected.
# ---------------------------------------------------------------------------

def test_zero_stiction_control_completes_via_strict_crossing_not_arrive(sim) -> None:
    """Against the ORIGINAL zero-stiction plant (no `SIMSET` stiction
    knobs), the SAME `D 200 200 500` must behave identically to before this
    sprint: `d_remaining` reaches exactly zero via the strict DISTANCE
    crossing (`reason=dist`) -- the stall-confirm window structurally
    cannot elapse first, because the shrink-tracking reset fires every tick
    the robot makes ANY forward progress, and a zero-stiction plant always
    does.
    """
    reply = sim.send_command("D 200 200 500")
    assert "OK drive" in reply, f"D command rejected: {reply!r}"

    result = _drive_and_trace(sim, total_ms=6000)

    assert result["done_evt"] is not None, f"drive never completed: {result}"
    assert "reason=dist" in result["done_evt"], (
        f"zero-stiction D must still complete via the strict crossing, "
        f"exactly as before this sprint -- the terminal-completion path "
        f"must be provably inert here: {result}"
    )
    assert "reason=arrive" not in result["done_evt"], (
        f"the stalled-short forced-completion path must never fire against "
        f"a plant that never stalls: {result}"
    )
    assert result["done_avg"] is not None and result["done_avg"] >= 500.0, (
        f"a clean DISTANCE crossing must land AT/OVER the target: {result}"
    )
    _assert_no_backward_travel(result["trace"])


def test_stall_confirm_cannot_misfire_while_progress_continues(sim) -> None:
    """Structural (not just empirical) proof that the "no progress" gate is
    load-bearing: even with an exaggerated `distArriveTol` (so the
    tolerance band is entered early, well before the robot would naturally
    decelerate) and a huge `stallConfirm`, a zero-stiction `D 200 200 500`
    still completes via the strict DISTANCE crossing (`reason=dist`) -- the
    shrink-tracking reset fires every tick `d_remaining` decreases, so the
    stall timer never accumulates while the robot keeps moving forward,
    regardless of how the two tunables are set.
    """
    reply = sim.send_command("SET distArriveTol=100 stallConfirm=100000")
    assert reply.upper().startswith("OK"), f"SET rejected: {reply!r}"

    reply = sim.send_command("D 200 200 500")
    assert "OK drive" in reply, f"D command rejected: {reply!r}"

    result = _drive_and_trace(sim, total_ms=6000)

    assert result["done_evt"] is not None and "reason=dist" in result["done_evt"], (
        f"a huge stallConfirm must not delay or replace the strict "
        f"crossing while the robot keeps making forward progress: {result}"
    )
    _assert_no_backward_travel(result["trace"])


# ---------------------------------------------------------------------------
# RobotConfig SET/GET round-trip for the two new fields.
# ---------------------------------------------------------------------------

def test_dist_arrive_tol_and_stall_confirm_set_get_roundtrip(sim) -> None:
    """`distArriveTol`/`stallConfirm` are new SET/GET-able RobotConfig
    fields (four-file coordinated edit: Config.h, DefaultConfig.cpp,
    ConfigRegistry.cpp, robot_config.schema.json)."""
    reply = sim.send_command("SET distArriveTol=8 stallConfirm=400")
    assert reply.upper().startswith("OK"), f"SET rejected: {reply!r}"

    reply = sim.send_command("GET distArriveTol stallConfirm")
    assert "distArriveTol=8.000" in reply, f"GET missing distArriveTol: {reply!r}"
    assert "stallConfirm=400.000" in reply, f"GET missing stallConfirm: {reply!r}"


def test_stall_confirm_rejects_negative(sim) -> None:
    """A negative `stallConfirm` is rejected by `validateConfig` -- it would
    make the debounce fire instantly on the very first tick inside
    `distArriveTol`, even while the robot is still moving normally."""
    reply = sim.send_command("SET stallConfirm=-1")
    assert "ERR badval" in reply, (
        f"expected SET to reject stallConfirm=-1: {reply!r}"
    )
