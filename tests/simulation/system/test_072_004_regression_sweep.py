"""
test_072_004_regression_sweep.py — ticket 072-004.

Sprint 072 closes three D-drive termination defects together (signed
DISTANCE/ROTATION stop + SAFETY_MARGIN safety net (ticket 002), the D-mode
terminal-completion guarantee (ticket 003), all validated against the new
sim stiction/breakaway plant (ticket 001)). Tickets 001-003 each already
ship real-binary integration tests proving their own scenario in detail:

  - tests/simulation/system/test_072_003_terminal_completion_guarantee.py
    ::test_stiction_stall_now_completes_cleanly_no_reversal_no_thrash — the
    exhaustive stiction-plus-terminal-completion proof (full encoder trace,
    no-reversal check, rest-at-completion check).
  - tests/simulation/unit/test_072_002_signed_stop_and_safety_margin.py
    ::test_safety_margin_fires_on_runaway_reverse_during_forward_d — the
    exhaustive SAFETY_MARGIN/HARD-teardown proof (exact firing tick, PWM
    zeroed).
  - tests/simulation/system/test_072_003_terminal_completion_guarantee.py
    ::test_zero_stiction_control_completes_via_strict_crossing_not_arrive —
    the zero-stiction control case.

This file is ticket 004's own consolidated regression sweep: one place that
exercises all three of the sprint's guarantees side by side, confirming they
coexist correctly in a single suite rather than only in isolation across
three separate files. It intentionally does not re-derive every assertion
those files already make (see them for the exhaustive proofs) — it is the
sprint-wide gate ticket 004 owns, per architecture-update.md Step 5
("Ticket 004") and this ticket's own Acceptance Criteria (integrated
D-drive-with-stiction terminal-completion regression; integrated
safety_stop-on-runaway regression).
"""
from __future__ import annotations

import ctypes

TICK_STEP_MS = 24


def _drive_and_trace(sim, total_ms: int, step_ms: int = TICK_STEP_MS) -> dict:
    """Run one already-issued `D` command, recording the avg-encoder trace
    and how/when it completes. Mirrors the helper in
    test_072_003_terminal_completion_guarantee.py."""
    t = 0
    done_evt = None
    done_avg = None
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
            done_evt = evts
            break

    return {"elapsed_ms": t, "done_evt": done_evt, "done_avg": done_avg, "trace": trace}


def _assert_no_backward_travel(trace: list[float], eps: float = 0.05) -> None:
    for i in range(1, len(trace)):
        assert trace[i] >= trace[i - 1] - eps, (
            f"backward travel detected at tick {i}: "
            f"{trace[i - 1]:.4f} -> {trace[i]:.4f} mm"
        )


# ---------------------------------------------------------------------------
# Guarantee 1: D-drive-with-stiction completes cleanly (tickets 001 + 003).
# ---------------------------------------------------------------------------

def test_sprint_072_regression_stiction_d_drive_completes_cleanly_no_reversal(sim) -> None:
    """Against ticket 001's stiction-configured plant, a `D 200 200 500`
    drive completes cleanly (no reversal, no thrash, no safety_stop) via
    ticket 003's terminal-completion guarantee -- and does so without ever
    tripping ticket 002's SAFETY_MARGIN net, proving the signed-stop,
    safety-net, and terminal-completion fixes all coexist without conflict
    during a legitimate (if difficult) drive.
    """
    reply = sim.send_command("SIMSET stictionPwmL=28 stictionPwmR=28")
    assert reply.upper().startswith("OK"), f"SIMSET rejected: {reply!r}"

    reply = sim.send_command("D 200 200 500")
    assert "OK drive" in reply, f"D command rejected: {reply!r}"

    result = _drive_and_trace(sim, total_ms=6000)

    assert result["done_evt"] is not None, (
        f"drive never completed within 6 s: {result}"
    )
    assert "EVT safety_stop" not in result["done_evt"], (
        f"a legitimate stiction-limited drive must never trip the runaway "
        f"safety net: {result}"
    )
    assert "EVT done D" in result["done_evt"], (
        f"expected a normal EVT done D completion: {result}"
    )
    assert (
        "reason=arrive" in result["done_evt"] or "reason=dist" in result["done_evt"]
    ), f"expected reason=arrive or reason=dist, not reason=time: {result}"
    assert result["done_avg"] is not None and result["done_avg"] >= 495.0, (
        f"should land within distArriveTol (default 5 mm) of the 500 mm "
        f"target: {result}"
    )
    _assert_no_backward_travel(result["trace"])


# ---------------------------------------------------------------------------
# Guarantee 2: safety_stop fires on runaway (ticket 002).
# ---------------------------------------------------------------------------

def test_sprint_072_regression_safety_stop_fires_on_runaway(sim) -> None:
    """A forward `D 200 200 500` forced to run backward past the safety
    margin aborts via HARD teardown and `EVT safety_stop reason=runaway`
    within one control tick of crossing the margin -- the sprint's
    wire-visible safety net (ticket 002), confirmed here alongside
    guarantee 1/3 as part of the sprint-wide regression sweep.
    """
    reply = sim.send_command("D 200 200 500")
    assert "OK drive" in reply, f"D command rejected: {reply!r}"

    evts = ""
    fired_at_tick = None
    # -10 mm/tick: crosses the default 50 mm safety margin at tick 5.
    for i in range(1, 21):
        mm = -10.0 * i
        sim._lib.sim_set_enc_l(sim._h, ctypes.c_float(mm))
        sim._lib.sim_set_enc_r(sim._h, ctypes.c_float(mm))
        sim.tick_for(TICK_STEP_MS, step_ms=TICK_STEP_MS)
        tick_evts = sim.get_async_evts()
        evts += tick_evts
        if fired_at_tick is None and "EVT safety_stop" in tick_evts:
            fired_at_tick = i
        if fired_at_tick is not None:
            break

    assert fired_at_tick is not None, (
        f"SAFETY_MARGIN never fired during a 200 mm backward runaway on a "
        f"forward D: {evts!r}"
    )
    assert fired_at_tick <= 6, (
        f"SAFETY_MARGIN should fire within one control tick of crossing the "
        f"50 mm margin (tick 5 at -10 mm/tick), fired at tick "
        f"{fired_at_tick} instead: {evts!r}"
    )
    assert "reason=runaway" in evts, (
        f"safety_stop from a runaway D must carry reason=runaway: {evts!r}"
    )
    assert "EVT done D" not in evts and "reason=dist" not in evts, (
        f"the runaway must not ALSO report a false dist completion: {evts!r}"
    )

    # HARD teardown: PWM drops to (near) zero promptly.
    for _ in range(3):
        sim.tick_for(TICK_STEP_MS, step_ms=TICK_STEP_MS)
    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    pwm_r = float(sim._lib.sim_get_pwm_r(sim._h))
    assert abs(pwm_l) < 5.0 and abs(pwm_r) < 5.0, (
        f"SAFETY_MARGIN should force an immediate HARD stop, not a SOFT "
        f"ramp: pwm_l={pwm_l}, pwm_r={pwm_r}"
    )


# ---------------------------------------------------------------------------
# Guarantee 3: nominal zero-stiction D still completes on reason=dist.
# ---------------------------------------------------------------------------

def test_sprint_072_regression_nominal_zero_stiction_d_completes_via_dist(sim) -> None:
    """Against the ORIGINAL zero-stiction plant (no `SIMSET` stiction
    knobs configured), a `D 200 200 500` drive behaves identically to
    before this sprint: a clean strict-crossing completion
    (`EVT done D reason=dist`), never `reason=arrive`, never
    `EVT safety_stop`, no backward travel anywhere in the trace -- proving
    none of this sprint's three fixes regress the common, already-working
    case.
    """
    reply = sim.send_command("D 200 200 500")
    assert "OK drive" in reply, f"D command rejected: {reply!r}"

    result = _drive_and_trace(sim, total_ms=6000)

    assert result["done_evt"] is not None, f"drive never completed: {result}"
    assert "reason=dist" in result["done_evt"], (
        f"nominal zero-stiction D must complete via the strict crossing, "
        f"exactly as before this sprint: {result}"
    )
    assert "reason=arrive" not in result["done_evt"], (
        f"the stalled-short forced-completion path must never fire against "
        f"a plant that never stalls: {result}"
    )
    assert "EVT safety_stop" not in result["done_evt"], (
        f"a legitimate forward drive must never trip the runaway safety "
        f"net: {result}"
    )
    assert result["done_avg"] is not None and result["done_avg"] >= 500.0, (
        f"a clean DISTANCE crossing must land AT/OVER the target: {result}"
    )
    _assert_no_backward_travel(result["trace"])
