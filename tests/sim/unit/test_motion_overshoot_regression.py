"""Regression harness for ticket 086-001 (SUC-001/SUC-002), reproducing the
086 issue's own root-caused, measured terminal-overshoot bug BEFORE any fix
lands: ``clasi/sprints/086-motion-terminal-overshoot-fix-real-hardware-otos-
driver-and-flip-flop-cadence/issues/motion-turn-drive-terminal-overshoot.md``.

**Root cause (issue's own writeup):** the motor velocity loop overshoots
clean through zero into a sustained reverse spin at the end of every turn
(and drive) -- the commanded yaw/wheel-speed ramps to zero far faster than
the real motor loop can track, so the loop drives the wheel PAST zero into a
reverse spin that lingers for hundreds of ms before settling, backtracking
the turn's heading and rolling a completed drive back off its stop point.

This ticket is test-only -- no ``source/`` changes. Both tests below assert
the TIGHT, CORRECT post-fix behavior (not the buggy behavior itself), so
they currently FAIL against today's firmware and are marked
``xfail(strict=True)``: ticket 086-002 (motor-velocity-loop fix) and 086-003
(terminal decel/coast anticipation) are expected to make the underlying
assertions pass, at which point ``strict=True`` turns an unexpected PASS
into a hard failure -- forcing the xfail marker's removal instead of letting
the regression silently go unnoticed.

Drives ``libfirmware_host`` through the full wire dispatch (``Sim.command()``
via the ``sim`` fixture, ``tests/sim/conftest.py``) exactly like
``test_motion_commands_arc_turn.py``/``test_planner.py`` -- ``vel(L,R)`` is
sampled via ``SNAP`` (synchronous reply, not the async drain -- see
``firmware.py``'s ``Sim.command()`` doc comment) at the sim's own ~24 ms
tick-step resolution across the stop transition, per the ticket's own
"Testing plan".
"""
from __future__ import annotations

import math

import pytest

# Matches the sim's own ~24 ms control-period tick convention (firmware.py's
# _DEFAULT_STEP) -- fine enough to resolve the reverse-spin shape tick by
# tick, not just a single post-stop sample.
_TICK_STEP = 24   # [ms]


def _parse_tlm(line: str) -> dict[str, str]:
    """Parse one "TLM t=... mode=... ..." wire line into a key->value dict.

    Local, small, deliberately duplicated per test file -- mirrors this
    directory's existing precedent (e.g. test_tlm_stream_snap.py's own
    ``_parse_tlm``) rather than a shared test-util module.
    """
    parts = line.strip().split()
    assert parts[0] == "TLM", f"not a TLM line: {line!r}"
    return dict(p.split("=", 1) for p in parts[1:])


def _snap(sim) -> dict[str, str]:
    """Issue SNAP and parse its reply -- the synchronous command path (NOT
    ``get_async_evts()``'s drain), exactly as the ticket specifies."""
    reply = sim.command("SNAP").strip()
    lines = reply.splitlines()
    assert len(lines) == 1, f"expected exactly one TLM line from SNAP, got: {reply!r}"
    return _parse_tlm(lines[0])


# ---------------------------------------------------------------------------
# 1. Turn reverse-spin: RT 9000 (+90 deg).
# ---------------------------------------------------------------------------

# Post-completion sampling window: the issue's own report samples "across the
# completion tick and ~800 ms after" -- matched here exactly.
_SETTLE_WINDOW = 800   # [ms] after EVT done RT

# Grace period before the tight bound applies: the wheel is (correctly, even
# post-fix) still coasting down through zero for a short beat right after the
# stop fires -- the bug is the SUSTAINED reverse-sign residual that follows,
# not that first coast-down instant. 200 ms comfortably covers a clean
# coast-down; today's measured behavior (see docstring below) is still very
# much unsettled well past this point.
_SETTLE_GRACE = 200   # [ms] after EVT done RT

# Tight bound: a wheel that has genuinely stopped reads near 0 mm/s. 2.0 mm/s
# is generous enough to absorb ordinary filter/quantization noise but far
# below the sustained ~4-7 mm/s reverse residual measured pre-fix (see below).
_RESIDUAL_BOUND = 2.0   # [mm/s]

_RT_BUDGET = 4000   # [ms] ample for the turn itself plus the full settle tail


@pytest.mark.xfail(
    strict=True,
    reason=(
        "086-001: reproduces the 086 motion terminal-overshoot bug (sustained "
        "reverse-spin residual after RT completes); ticket 086-002 (motor "
        "velocity-loop fix) and 086-003 (terminal decel anticipation) are "
        "expected to remove this xfail marker."
    ),
)
def test_rt_9000_settles_without_sustained_reverse_spin_residual(sim):
    """RT 9000 (+90 deg) must settle to near-zero per-wheel velocity shortly
    after ``EVT done RT`` fires and STAY there -- no sustained reverse-sign
    residual spin.

    Measured pre-fix behavior (captured against this ticket's own commit,
    2026-07-06, via a dense 24 ms-step SNAP trace, in this exact tick/SNAP/
    EVT order -- see the issue's own writeup for the same shape measured
    independently):

        during turn:        vel(L,R) ramps to, and holds, ~(-164,+167) mm/s,
                             mode=T
        EVT done RT tick:   vel(L,R) = ( -93,  +93) mm/s   mode=I
                             (fired at t=864ms elapsed) -- heading already
                             correct (true_h=97.61 deg vs the 90 deg target),
                             but the wheel is STILL spinning at more than
                             half turn-speed when the stop fires.
        +24ms:               vel(L,R) = ( -52,  +52)
        +48ms:               vel(L,R) = ( -17,  +17)
        +72ms:               vel(L,R) = (  -2,   +2)   -- crosses zero
        +96ms:               vel(L,R) = (  +4,   -4)   -- reverse-sign begins
        +120ms:               vel(L,R) = (  +7,   -7)   -- local reverse peak
        +200..800ms:         vel(L,R) keeps oscillating between roughly
                             (+2..+7, -2..-7) mm/s -- a SUSTAINED reverse-sign
                             residual; the worst-case magnitude measured in
                             that window is 7.0 mm/s (t=1128ms), nowhere near
                             settled -- a longer trace shows it only fully
                             decays to 0 around +1800-1900ms, long after this
                             test's own 800 ms observation window.
        heading:             still descending from 97.61 deg through 92.55
                             deg by +816ms (not yet even back down to the 90
                             deg target within this window) -- a longer trace
                             confirms it keeps backtracking to a final rest
                             near 89.5 deg, an ~8 deg total backtrack from the
                             97.61 deg peak, within the issue's own reported
                             4-10 deg range.

    The assertion below is the CORRECT post-fix expectation -- a tight
    residual-velocity bound over the back half of the 800 ms post-completion
    window (086-002/086-003's target) -- which the measured shape above
    fails today (worst-case 7.0 mm/s against a 2.0 mm/s bound).
    """
    reply = sim.command("RT 9000")
    assert reply.strip() == "OK rt rot=9000"

    samples: list[tuple[int, float, float]] = []
    done_at: int | None = None
    elapsed = 0
    while elapsed < _RT_BUDGET:
        sim.tick_for(_TICK_STEP, step=_TICK_STEP)
        elapsed += _TICK_STEP
        tlm = _snap(sim)
        evts = sim.get_async_evts()
        vel_l, vel_r = (float(v) for v in tlm["vel"].split(","))
        samples.append((elapsed, vel_l, vel_r))
        if done_at is None and "EVT done RT" in evts:
            done_at = elapsed
        if done_at is not None and elapsed >= done_at + _SETTLE_WINDOW:
            break

    assert done_at is not None, (
        f"RT 9000 never completed (no 'EVT done RT') within a {_RT_BUDGET}ms budget"
    )

    tail = [
        (t, vel_l, vel_r) for (t, vel_l, vel_r) in samples
        if done_at + _SETTLE_GRACE <= t <= done_at + _SETTLE_WINDOW
    ]
    assert tail, "no samples captured in the post-completion settle window"

    worst = max(max(abs(vel_l), abs(vel_r)) for _, vel_l, vel_r in tail)
    assert worst <= _RESIDUAL_BOUND, (
        f"expected per-wheel residual velocity <= {_RESIDUAL_BOUND} mm/s "
        f"{_SETTLE_GRACE}-{_SETTLE_WINDOW}ms after 'EVT done RT' (fired at "
        f"t={done_at}ms), but measured a worst-case {worst:.2f} mm/s in that "
        f"window -- sustained reverse-spin residual, the 086 issue's own bug. "
        f"Tail samples (t_ms, vel_l, vel_r): {tail}"
    )


# ---------------------------------------------------------------------------
# 2. Drive overshoot: D 200 200 500 (500 mm at 200 mm/s).
# ---------------------------------------------------------------------------

# Materially tighter than the issue's own measured ~7% pre-fix overshoot.
_DRIVE_TOLERANCE_FRACTION = 0.015   # 1.5% of the commanded 500 mm (7.5 mm)
_D_BUDGET = 4000   # [ms] ample for the drive itself plus completion


@pytest.mark.xfail(
    strict=True,
    reason=(
        "086-001: reproduces the 086 motion terminal-overshoot bug (D "
        "command overshoots its commanded distance on stop); ticket 086-002 "
        "(motor velocity-loop fix) and 086-003 (terminal decel anticipation) "
        "are expected to remove this xfail marker."
    ),
)
def test_d_200_200_500_stops_within_tight_tolerance_of_commanded_distance(sim):
    """D 200 200 500 (500 mm straight-line drive) must stop within a tight
    tolerance of the commanded 500 mm, measured (ground-truth pose) at the
    tick ``EVT done D`` fires.

    Measured pre-fix behavior (captured against this ticket's own commit,
    2026-07-06, via a dense 24 ms-step trace, in this exact tick/EVT order):
    the wheel velocity loop overshoots the same way RT's does -- true x =
    532.51 mm at the tick 'EVT done D reason=dist' fires (t=2496ms elapsed,
    +6.50% over the 500 mm target -- matching the issue's own reported
    ~7%/~535mm closely) -- then the same reverse-spin residual keeps rolling
    the robot BACKWARD well past that point (true x = 523.10mm at +240ms,
    510.05mm at +720ms, crossing back under the 500mm target around
    +1100-1200ms, down to 486.43mm by +1920ms and still trending down),
    which is an even more dramatic manifestation of the same root cause: the
    terminal velocity-loop overshoot corrupts the final resting position in
    BOTH directions depending on when it is sampled.

    The assertion below measures at the 'EVT done D' completion tick itself
    (matching how a real caller -- e.g. a Planner-chained tour leg --
    would observe "done") against a tight 1.5% tolerance, which the ~5.14%
    pre-fix overshoot fails.
    """
    reply = sim.command("D 200 200 500")
    assert reply.strip() == "OK drive l=200 r=200 mm=500"

    evts = ""
    elapsed = 0
    while elapsed < _D_BUDGET:
        sim.tick_for(_TICK_STEP, step=_TICK_STEP)
        elapsed += _TICK_STEP
        evts = sim.get_async_evts()
        if "EVT done D" in evts:
            break

    assert "EVT done D" in evts, f"D 200 200 500 never completed within a {_D_BUDGET}ms budget"

    x, y, _h = sim.true_pose()
    dist = math.hypot(x, y)
    tolerance = 500.0 * _DRIVE_TOLERANCE_FRACTION
    assert abs(dist - 500.0) <= tolerance, (
        f"expected final distance within {tolerance:.2f}mm ({_DRIVE_TOLERANCE_FRACTION * 100:.1f}%) "
        f"of the commanded 500mm at 'EVT done D', measured {dist:.2f}mm "
        f"({(dist - 500.0) / 500.0 * 100:.2f}% over) -- the 086 issue's own "
        f"terminal-overshoot bug (true_pose=({x:.2f}, {y:.2f}))"
    )
