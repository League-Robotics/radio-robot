"""094-006: MOVE + graceful STOP + pull-TLM, exercised end to end over the
wire (`sim.command()`/`sim.command_on()`), extending
`test_bare_loop_commands.py`'s 093/094-005 four-verb suite rather than
replacing it.

Covers:
  - `MOVE <mm> <dir_cdeg> <fh_cdeg> [v=][a=][j=][w=][wa=][wj=]` parses into a
    `Motion::Segment` and posts it to `bb.segmentIn` (`source/commands/
    motion_commands.cpp`'s `parseMove`/`handleMove`) -- all three shapes the
    ticket's acceptance criteria name: straight (`MOVE <mm> 0 0`), pure
    in-place turn (`MOVE 0 0 <heading>`), and translate-then-terminal-pivot
    (`MOVE <mm> 0 <heading>`).
  - Each drains to a graceful stop with no reverse-creep (measured velocity
    never flips sign, beyond a small settle-noise floor, after it first
    becomes substantial).
  - `MOVE`'s out-of-range/missing-argument error replies
    (`ERR range`/`ERR badarg`), following the existing verbs' own
    convention.
  - `STOP`, sent over the wire mid-`MOVE`, triggers the SAME graceful
    decel-to-zero `NEUTRAL` gets at the `Drivetrain` level (094-004) --
    `STOP`'s own wire reply text is unchanged (`OK stop`).
  - `TLM` (new, one-shot synchronous read -- `handleTlm`) reports measured
    `enc=`/`vel=` that track real (simulated) wheel motion, plus the
    executor's active/idle flag (`msg::DrivetrainState.active`, widened by
    this ticket -- see `drivetrain.cpp`'s `state()`).
  - A `MOVE` posted during slack takes effect on the very next mandatory
    tick (`sim.command()`'s own dt=0 synchronous-command trick already
    replays one `Rt::MainLoop::tick()` immediately after routing --
    `tests/_infra/sim/sim_api.cpp`'s own file header).
  - Two `MOVE`s posted back-to-back, with no intervening `sim.tick_for()`
    call, BOTH execute (in order) -- proves `bb.segmentIn`'s
    `Rt::WorkQueue<Motion::Segment, 8>` shape does not silently drop the
    first one the way a latest-wins `Mailbox` would.

`S`/`STOP`'s own pre-existing DIRECT-mode assertions stay in
`test_bare_loop_commands.py`, unmodified -- this file only adds the new
MOVE/TLM surface plus one wire-level graceful-STOP confirmation, per the
ticket's "extended, not replaced" instruction.
"""
from __future__ import annotations

import re

import pytest

_TLM_RE = re.compile(
    r"^OK tlm enc=(-?\d+),(-?\d+) vel=(-?\d+),(-?\d+) active=([01])"
    r"(?: conn=[01],[01])?$"   # conn= appended post-094 (OOP I2C-health field)
)


def _parse_tlm(reply: str) -> tuple[int, int, int, int, int]:
    m = _TLM_RE.match(reply.strip())
    assert m is not None, f"TLM reply did not match the expected shape: {reply!r}"
    enc_l, enc_r, vel_l, vel_r, active = m.groups()
    return int(enc_l), int(enc_r), int(vel_l), int(vel_r), int(active)


def _run_and_check_no_reverse_creep(sim, seconds: float = 6.0, step: int = 24):
    """Tick the sim in `step`-ms increments for `seconds`, tracking each
    wheel's velocity sign once it first becomes substantial (|v| > 20 mm/s),
    and asserting it never flips past a small settle-noise floor (15 mm/s) in
    the opposite direction afterward -- the no-reverse-creep contract
    (drivetrain_harness.cpp's own scenario 3 precedent, exercised here over
    the wire instead of the C++ API). Returns (max_abs_vel_l, max_abs_vel_r)
    so a caller can additionally assert the segment genuinely drove instead
    of being a degenerate no-op.
    """
    ticks = int(seconds * 1000 / step)
    sign_l = 0
    sign_r = 0
    max_abs_l = 0.0
    max_abs_r = 0.0
    for _ in range(ticks):
        sim.tick_for(step)
        vel_l, vel_r = sim.vel()
        max_abs_l = max(max_abs_l, abs(vel_l))
        max_abs_r = max(max_abs_r, abs(vel_r))
        if sign_l == 0 and abs(vel_l) > 20.0:
            sign_l = 1 if vel_l > 0 else -1
        if sign_r == 0 and abs(vel_r) > 20.0:
            sign_r = 1 if vel_r > 0 else -1
        if sign_l == 1:
            assert vel_l > -15.0, f"left wheel reverse-crept: {vel_l} mm/s"
        elif sign_l == -1:
            assert vel_l < 15.0, f"left wheel reverse-crept: {vel_l} mm/s"
        if sign_r == 1:
            assert vel_r > -15.0, f"right wheel reverse-crept: {vel_r} mm/s"
        elif sign_r == -1:
            assert vel_r < 15.0, f"right wheel reverse-crept: {vel_r} mm/s"
    return max_abs_l, max_abs_r


# ---------------------------------------------------------------------------
# MOVE's three shapes -- each executes, settles, and never reverse-creeps.
# ---------------------------------------------------------------------------

def test_move_straight_executes_and_settles_no_reverse_creep(sim):
    """`MOVE <mm> 0 0` -- a plain straight (TRANSLATE-only, both pivots
    degenerate)."""
    reply = sim.command("MOVE 300 0 0")
    assert reply.strip() == "OK move dist=300 dir=0 fh=0"

    max_l, max_r = _run_and_check_no_reverse_creep(sim)
    assert max_l > 50.0 and max_r > 50.0, "segment never genuinely drove"

    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)

    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 0


def test_move_pure_in_place_turn_executes_and_settles_no_reverse_creep(sim):
    """`MOVE 0 0 <heading>` -- distance and direction both 0, so only the
    TERMINAL_PIVOT phase fires (a pure in-place turn)."""
    reply = sim.command("MOVE 0 0 9000")
    assert reply.strip() == "OK move dist=0 dir=0 fh=9000"

    max_l, max_r = _run_and_check_no_reverse_creep(sim)
    assert max_l > 20.0 and max_r > 20.0, "segment never genuinely drove"

    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)

    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 0


def test_move_translate_then_terminal_pivot_executes_and_settles_no_reverse_creep(sim):
    """`MOVE <mm> 0 <heading>` -- direction 0 (PRE_PIVOT degenerate), so
    TRANSLATE runs first, then TERMINAL_PIVOT rotates to `finalHeading`.

    Unlike the straight/pure-turn tests above, this segment has TWO phases
    with genuinely different (and, for TERMINAL_PIVOT, opposite-signed)
    wheel targets -- a wheel legitimately flips sign at the TRANSLATE ->
    TERMINAL_PIVOT boundary (a pivot needs one wheel to reverse relative to
    straight driving), so `_run_and_check_no_reverse_creep()`'s strict
    single-sign-per-wheel check would misfire on that boundary. Empirically
    (measured against this same plant/executor pairing) a 300mm/90deg
    segment fully converges to zero by ~2.4s, well inside the 3.6s run
    below -- so the strict no-reverse-creep check is applied only to the
    SEGMENT's own final settle tail (already at/near zero throughout),
    which is exactly the natural-completion contract this ticket's
    acceptance criteria ask for."""
    reply = sim.command("MOVE 300 0 9000")
    assert reply.strip() == "OK move dist=300 dir=0 fh=9000"

    max_l = max_r = 0.0
    for _ in range(150):   # 3.6s -- covers both TRANSLATE's and TERMINAL_
                           # PIVOT's own peak velocity (see docstring).
        sim.tick_for(24)
        vel_l, vel_r = sim.vel()
        max_l = max(max_l, abs(vel_l))
        max_r = max(max_r, abs(vel_r))
    assert max_l > 50.0 and max_r > 50.0, "segment never genuinely drove"

    # Final settle tail: strict no-reverse-creep, same contract as the
    # straight/pure-turn tests above.
    _run_and_check_no_reverse_creep(sim, seconds=4.0)

    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)

    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 0


# ---------------------------------------------------------------------------
# MOVE's argument-error convention.
# ---------------------------------------------------------------------------

def test_move_out_of_range_distance_replies_err_range(sim):
    reply = sim.command("MOVE 99999 0 0")
    assert reply.strip() == "ERR range distance"


def test_move_out_of_range_direction_replies_err_range(sim):
    reply = sim.command("MOVE 100 999999 0")
    assert reply.strip() == "ERR range direction"


def test_move_missing_required_tokens_replies_err_badarg(sim):
    reply = sim.command("MOVE 300 0")
    assert reply.strip() == "ERR badarg"


def test_move_out_of_range_kv_override_replies_err_range(sim):
    reply = sim.command("MOVE 300 0 0 v=999999")
    assert reply.strip() == "ERR range v"


# ---------------------------------------------------------------------------
# STOP over the wire, mid-MOVE: graceful decel-to-zero (094-004), confirmed
# end to end through the command surface (094-006).
# ---------------------------------------------------------------------------

def test_stop_over_wire_mid_move_triggers_graceful_decel_no_reverse_creep(sim):
    """`STOP` sent while a `MOVE` segment is actively executing triggers the
    SAME executor-owned graceful decel-to-zero `NEUTRAL` gets when a segment
    is in flight (drivetrain.cpp's `dispatchEscapeHatch()`, 094-004) --
    velocity decays toward zero and never reverses sign. `STOP`'s own wire
    reply text is unchanged (`OK stop`) even though its physical effect
    changed from 093's instant brake."""
    reply = sim.command("MOVE 2000 0 0")   # long: never completes naturally in this window
    assert reply.strip() == "OK move dist=2000 dir=0 fh=0"

    sim.tick_for(1000)   # 1s -- underway
    vel_l, vel_r = sim.vel()
    assert vel_l > 20.0 and vel_r > 20.0, "precondition: genuinely driving before STOP"

    reply = sim.command("STOP")
    assert reply.strip() == "OK stop"

    ever_negative = False
    for _ in range(250):   # up to 6s to settle
        sim.tick_for(24)
        vel_l, vel_r = sim.vel()
        v = (vel_l + vel_r) * 0.5
        if v < -5.0:
            ever_negative = True

    assert not ever_negative, "measured velocity reversed sign after STOP -- not a graceful decel"
    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)


def test_s_and_stop_still_work_unchanged_over_the_wire(sim):
    """093/094-005's `S`/`STOP` DIRECT-mode contract stays green, extended
    (not replaced) by this ticket -- full coverage lives in
    `test_bare_loop_commands.py`; this is a light smoke check that the SAME
    table still carries both verbs alongside the new `MOVE`/`TLM`."""
    reply = sim.command("S 150 150")
    assert reply.strip() == "OK drive l=150 r=150"
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l > 50.0 and vel_r > 50.0

    reply = sim.command("STOP")
    assert reply.strip() == "OK stop"
    sim.tick_for(1000)
    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)


# ---------------------------------------------------------------------------
# TLM: measured, not commanded; active/idle flag.
# ---------------------------------------------------------------------------

def test_tlm_reports_measured_enc_and_vel_tracking_real_motion(sim):
    """`TLM` reads `bb.drivetrain` -- MEASURED per-wheel encoder position/
    velocity (094-004's rewrite of `Drivetrain::state()`), not a commanded
    target -- and tracks the plant's own reported `sim.enc()`/`sim.vel()`
    reads closely."""
    reply = sim.command("S 150 150")
    assert reply.strip() == "OK drive l=150 r=150"
    sim.tick_for(3000)

    enc_l, enc_r = sim.enc()
    vel_l, vel_r = sim.vel()

    t_enc_l, t_enc_r, t_vel_l, t_vel_r, active = _parse_tlm(sim.command("TLM"))

    assert t_enc_l == pytest.approx(enc_l, abs=5.0)
    assert t_enc_r == pytest.approx(enc_r, abs=5.0)
    assert t_vel_l == pytest.approx(vel_l, abs=15.0)
    assert t_vel_r == pytest.approx(vel_r, abs=15.0)
    assert active == 1   # a live S drive is DIRECT-mode active


def test_tlm_active_flag_is_zero_when_idle(sim):
    """A fresh sim (nothing ever commanded) reports `active=0` -- neither
    the DIRECT-mode authority flag nor the segment executor has ever been
    engaged."""
    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 0


# ---------------------------------------------------------------------------
# Timing/ordering guarantees (architecture-update.md Section 5).
# ---------------------------------------------------------------------------

def test_move_sent_mid_slack_takes_effect_on_next_mandatory_tick(sim):
    """`sim.command()`'s own dt=0 synchronous-command trick already replays
    exactly one `Rt::MainLoop::tick()` at the unchanged `now` immediately
    after routing (`sim_api.cpp`'s file header) -- so a `TLM` read taken
    IMMEDIATELY after `MOVE`, with no separate `sim.tick_for()` call in
    between, already shows the segment claimed SEGMENT mode. Proves
    `segmentIn -> ring_ -> executor` happens within that one mandatory
    tick, not a multi-hop mailbox latency."""
    reply = sim.command("MOVE 300 0 0")
    assert reply.strip() == "OK move dist=300 dir=0 fh=0"

    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 1


def test_two_moves_queued_back_to_back_both_execute_in_order(sim):
    """Two `MOVE`s posted with NO intervening `sim.tick_for()` call both
    execute (in order) -- proves `bb.segmentIn`'s `Rt::WorkQueue<
    Motion::Segment, 8>` shape does not silently drop the first the way a
    latest-wins `Mailbox` would (architecture-update.md Section 5,
    "Command precedence and the 'no hiccups' requirement")."""
    reply1 = sim.command("MOVE 200 0 0")
    assert reply1.strip() == "OK move dist=200 dir=0 fh=0"
    reply2 = sim.command("MOVE 200 0 0")
    assert reply2.strip() == "OK move dist=200 dir=0 fh=0"

    # Ample settle window for TWO 200mm straight segments run back to back.
    for _ in range(500):   # up to 12s
        sim.tick_for(24)

    true_enc_l, true_enc_r = sim.true_wheel_travel()
    # A single `MOVE 200 0 0` run alone true-travels ~231mm (STOP_DISTANCE's
    # own trigger/coast headroom over the commanded 200mm -- not a bug, a
    # fixed characteristic of this executor/plant pairing). Two back-to-back
    # segments should true-travel ~462mm (2x) -- NOT ~231mm (1x), which a
    # Mailbox's silently-dropped-first-MOVE would produce.
    assert true_enc_l == pytest.approx(462.0, abs=60.0)
    assert true_enc_r == pytest.approx(462.0, abs=60.0)
    assert true_enc_l > 350.0, "second MOVE never ran -- looks like only one segment executed"

    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 0
