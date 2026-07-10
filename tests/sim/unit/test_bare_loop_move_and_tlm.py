"""094-006/097-006: MOVE/MOVER + graceful STOP + pull-TLM, exercised end to
end over the wire, extending `test_bare_loop_commands.py`'s
093/094-005/097-006 STOP-plus-binary-drive suite rather than replacing it.

097-006 (architecture-update-r2.md Decision 9, pure-binary firmware):
`MOVE`/`MOVER`/`S` are DELETED from the text plane outright (not merely
unregistered) -- every send below that used to be a text verb line now
goes through the binary `segment`/`replace`/`drive` arm instead
(`source/commands/binary_channel.cpp`, 096, sim-exhaustive), fed the SAME
`Motion::Segment`/`WheelTargets` shape each text handler used to build, via
`host/robot_radio/robot/legacy_translate.py`'s `segment_for_move()`/
`segment_for_mover()`/`wheel_targets_for_drive()` -- the identical
translation `rogo`'s proxy (ticket 004) and `NezhaProtocol` (ticket 002)
use. `STOP`/`TLM`/`PING` are untouched text verbs throughout this file
(STOP/TLM are this sprint's own confirmed-live rump; TLM's deletion is
ticket 008's separate scope).

Covers:
  - `MOVE <mm> <dir_cdeg> <fh_cdeg> [v=][a=][j=][w=][wa=][wj=]`'s binary
    parity (`segment`) -- all three shapes the ticket's acceptance criteria
    name: straight (`MOVE <mm> 0 0`), pure in-place turn
    (`MOVE 0 0 <heading>`), and translate-then-terminal-pivot
    (`MOVE <mm> 0 <heading>`).
  - Each drains to a graceful stop with no reverse-creep (measured velocity
    never flips sign, beyond a small settle-noise floor, after it first
    becomes substantial).
  - `MOVE`'s out-of-range error replies (binary `Error{ERR_RANGE, field}`),
    following the existing binary-arm tests' own convention
    (`test_binary_channel.py`).
  - `STOP`, sent over the wire mid-`MOVE`, triggers the SAME graceful
    decel-to-zero `NEUTRAL` gets at the `Drivetrain` level (094-004) --
    `STOP`'s own wire reply text is unchanged (`OK stop`).
  - `TLM` (one-shot synchronous read -- `handleTlm`, untouched text verb)
    reports measured `enc=`/`vel=` that track real (simulated) wheel
    motion, plus the executor's active/idle flag
    (`msg::DrivetrainState.active`, widened by 094-006 -- see
    `drivetrain.cpp`'s `state()`).
  - A `MOVE` posted during slack takes effect on the very next mandatory
    tick (`_binary_envelope.send()`'s own dt=0 synchronous-command trick,
    the same one `sim.command()` already used for text -- `tests/_infra/
    sim/sim_api.cpp`'s own file header).
  - Two `MOVE`s posted back-to-back, with no intervening `sim.tick_for()`
    call, BOTH execute (in order) -- proves `bb.segmentIn`'s
    `Rt::WorkQueue<Motion::Segment, 8>` shape does not silently drop the
    first one the way a latest-wins `Mailbox` would.

`S`/`STOP`'s own pre-existing DIRECT-mode assertions stay in
`test_bare_loop_commands.py` -- this file only adds the MOVE/MOVER/TLM
surface plus one wire-level graceful-STOP confirmation, per 094-006's
original "extended, not replaced" instruction (097-006 re-points the
drive/segment/replace halves at their binary arms, in place).
"""
from __future__ import annotations

import re

import pytest

from _binary_envelope import ERR_RANGE, send_drive, send_replace, send_segment
from robot_radio.robot import legacy_translate

# Full post-094 TLM shape (OOP additions): cmd= (post-governor commanded
# wheel velocity), acc= (firmware-EMA measured acceleration), conn= (per-motor
# I2C health), glitch= (source-side rejected encoder samples), ts= (per-wheel
# sample instants, firmware ms -- smooth-telemetry fix). enc=/vel= carry 0.1
# resolution (one decimal).
_TLM_RE = re.compile(
    r"^OK tlm enc=(?P<enc_l>-?\d+\.\d),(?P<enc_r>-?\d+\.\d)"
    r" vel=(?P<vel_l>-?\d+\.\d),(?P<vel_r>-?\d+\.\d)"
    r" cmd=(?P<cmd_l>-?\d+),(?P<cmd_r>-?\d+)"
    r" acc=(?P<acc_l>-?\d+),(?P<acc_r>-?\d+)"
    r" active=(?P<active>[01])"
    r" conn=(?P<conn_l>[01]),(?P<conn_r>[01])"
    r" glitch=(?P<glitch_l>\d+),(?P<glitch_r>\d+)"
    r" ts=(?P<ts_l>\d+),(?P<ts_r>\d+)"
    r" now=(?P<now>\d+)$"
)


def _parse_tlm(reply: str) -> tuple[float, float, float, float, int]:
    """Returns the historical 5-tuple (enc_l, enc_r, vel_l, vel_r, active) --
    the extra fields are validated by the regex match itself; tests that need
    them use _parse_tlm_full()."""
    m = _TLM_RE.match(reply.strip())
    assert m is not None, f"TLM reply did not match the expected shape: {reply!r}"
    return (float(m["enc_l"]), float(m["enc_r"]),
            float(m["vel_l"]), float(m["vel_r"]), int(m["active"]))


def _parse_tlm_full(reply: str) -> dict:
    m = _TLM_RE.match(reply.strip())
    assert m is not None, f"TLM reply did not match the expected shape: {reply!r}"
    return {k: float(v) for k, v in m.groupdict().items()}


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
    degenerate). 097-006: sent as a binary `segment` (legacy_translate.
    segment_for_move() builds the SAME Motion::Segment shape handleMove()
    used to)."""
    reply = send_segment(sim, legacy_translate.segment_for_move(300, 0, 0))
    assert reply.WhichOneof("body") == "ok"

    max_l, max_r = _run_and_check_no_reverse_creep(sim)
    assert max_l > 50.0 and max_r > 50.0, "segment never genuinely drove"

    vel_l, vel_r = sim.vel()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)

    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 0


def test_move_pure_in_place_turn_executes_and_settles_no_reverse_creep(sim):
    """`MOVE 0 0 <heading>` -- distance and direction both 0, so only the
    TERMINAL_PIVOT phase fires (a pure in-place turn). 097-006: binary
    `segment`."""
    reply = send_segment(sim, legacy_translate.segment_for_move(0, 0, 9000))
    assert reply.WhichOneof("body") == "ok"

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
    097-006: binary `segment`.

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
    reply = send_segment(sim, legacy_translate.segment_for_move(300, 0, 9000))
    assert reply.WhichOneof("body") == "ok"

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
# MOVE's argument-error convention -- binary Error{ERR_RANGE, field}
# (097-006: re-pointed off the text ERR range/badarg reply strings).
# ---------------------------------------------------------------------------

def test_move_out_of_range_distance_replies_err_range(sim):
    reply = send_segment(sim, legacy_translate.segment_for_move(99999, 0, 0))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == ERR_RANGE
    assert reply.err.field == 1   # MotionSegment.distance's own field number


def test_move_out_of_range_direction_replies_err_range(sim):
    reply = send_segment(sim, legacy_translate.segment_for_move(100, 999999, 0))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == ERR_RANGE
    assert reply.err.field == 2   # MotionSegment.direction's own field number


# test_move_missing_required_tokens_replies_err_badarg -- DELETED (097-006):
# no binary equivalent. The deleted text scenario was parseMove() rejecting
# a line with fewer than 3 positional tokens (`MOVE 300 0`) with
# `ERR badarg`; MotionSegment's proto3 scalar fields have no "omitted"
# wire state distinct from an explicit 0 (binary_channel.cpp's own comment:
# "no hand parsing/range checks... the generated decoder's own bound
# checks"), so there is nothing a binary send could do to reproduce
# "argument never supplied" -- it is a text-grammar-only concept, not a
# semantic behavior with a binary parity arm to re-point to (per this
# ticket's own "no binary arm -> delete, don't force a mapping" instruction,
# applied here at the validation-behavior level for MOVE's badarg case
# specifically, even though MOVE the verb otherwise has a binary arm).


def test_move_out_of_range_kv_override_replies_err_range(sim):
    """097-006: the deleted text `v=` kv override maps onto MotionSegment's
    `speed_max` field (parseMove()'s own `v` -> `Motion::Segment.speedMax`
    assignment, motion_commands.cpp -- see protos/motion.proto's own field
    doc comment citing kMoveMaxSpeedMax)."""
    reply = send_segment(sim, legacy_translate.segment_for_move(300, 0, 0, speed_max=999999))
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == ERR_RANGE
    assert reply.err.field == 4   # MotionSegment.speed_max's own field number


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
    # long: never completes naturally in this window. 097-006: binary segment.
    reply = send_segment(sim, legacy_translate.segment_for_move(2000, 0, 0))
    assert reply.WhichOneof("body") == "ok"

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


def test_binary_drive_and_stop_still_work_together_over_the_wire(sim):
    """097-006: 093/094-005's `S`/`STOP` DIRECT-mode contract stays green,
    with `S` re-pointed to its binary `drive` parity (text `S` is deleted --
    see test_bare_loop_commands.py's own header comment); `STOP` is
    untouched, still text. Full coverage lives in
    `test_bare_loop_commands.py`; this is a light smoke check that the SAME
    table still carries `STOP`/`TLM` alongside `MOVE`/`MOVER`'s binary
    arms."""
    reply = send_drive(sim, 150, 150)
    assert reply.WhichOneof("body") == "ok"
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
    reads closely. 097-006: the precondition drive is now the binary
    `drive` arm (text `S` is deleted)."""
    reply = send_drive(sim, 150, 150)
    assert reply.WhichOneof("body") == "ok"
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


def test_tlm_active_clears_after_stop_settles(sim):
    """REGRESSION (busy-vs-authority latch, 2026-07-09): `STOP` -> NEUTRAL
    sets the AUTHORITY flag true (holding neutral IS governing the pair), so
    a TLM active= sourced from it latched 1 after the first STOP ever sent
    and could never mean "idle" again -- every hardware completion poll
    (notebook, bench demos) ran to timeout. active= now reports
    DrivetrainState.busy: it must return to 0 once the post-STOP decel has
    settled. 097-006: the precondition drive is now the binary `drive` arm
    (text `S` is deleted)."""
    assert send_drive(sim, 150, 150).WhichOneof("body") == "ok"
    sim.tick_for(500)
    assert sim.command("STOP").strip() == "OK stop"
    sim.tick_for(2000)   # ample settle for the graceful decel
    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 0, "TLM active= stayed 1 after STOP settled (authority-flag latch)"


def test_move_streaming_chains_at_speed(sim):
    """REGRESSION (streaming chain, OOP 2026-07-09): micro-MOVE segments
    streamed while the previous one executes must CHAIN at speed -- an
    unchained from-rest-to-rest 15mm segment caps peak speed at
    sqrt(a*d) ~= 110 mm/s regardless of send rate, so a sustained stream
    exceeding ~150 mm/s proves the executor retarget()s from its moving
    state. Draining the stream must still end in the graceful decel
    (settled, no reverse). 097-006: binary `segment`, `stream=True` (the
    `s=1` kv override's own binary parity)."""
    peak = 0.0
    for _ in range(20):
        seg = legacy_translate.segment_for_move(15, 0, 0, stream=True)
        r = send_segment(sim, seg)
        assert r.WhichOneof("body") == "ok"
        sim.tick_for(60)
        vel_l, vel_r = sim.vel()
        peak = max(peak, (vel_l + vel_r) / 2.0)
    assert peak > 150.0, f"streamed micro-segments did not chain (peak {peak:.0f} mm/s)"

    went_negative = False
    for _ in range(150):   # 3.6s drain window
        sim.tick_for(24)
        vel_l, vel_r = sim.vel()
        if (vel_l + vel_r) / 2.0 < -8.0:
            went_negative = True
    assert not went_negative, "stream drain reversed direction (not a graceful decel)"
    vel_l, vel_r = sim.vel()
    assert abs(vel_l) < 10.0 and abs(vel_r) < 10.0

    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 0


def test_mover_deadman_velocity(sim):
    """MOVER (deadman-velocity teleop, OOP 2026-07-09): time-bounded
    velocity segments REPLACE the in-flight motion, replanned from the
    current velocity. While refreshed before each t= window expires the
    robot cruises at the commanded velocity; when refreshes stop, the
    deadman fires and it decels gracefully (no reverse). 097-006: binary
    `replace` (legacy_translate.segment_for_mover() builds the SAME
    Motion::Segment shape handleMover() used to)."""
    mover_seg = legacy_translate.segment_for_mover(0, 0, 0, time=800, v=250, omega=0)
    r = send_replace(sim, mover_seg)
    assert r.WhichOneof("body") == "ok", r
    sim.tick_for(500)
    vel_l, vel_r = sim.vel()
    assert (vel_l + vel_r) / 2.0 > 180.0, f"never reached commanded velocity ({vel_l},{vel_r})"

    # Keep refreshing: velocity sustained well past the first window.
    for _ in range(4):
        send_replace(sim, legacy_translate.segment_for_mover(0, 0, 0, time=800, v=250, omega=0))
        sim.tick_for(400)
        vel_l, vel_r = sim.vel()
        assert (vel_l + vel_r) / 2.0 > 180.0, "velocity sagged between refreshes"

    # Stop refreshing: deadman fires within t= + decel; graceful, no reverse.
    went_negative = False
    for _ in range(120):   # 2.9s
        sim.tick_for(24)
        vel_l, vel_r = sim.vel()
        if (vel_l + vel_r) / 2.0 < -8.0:
            went_negative = True
    assert not went_negative, "deadman decel reversed direction"
    vel_l, vel_r = sim.vel()
    assert abs(vel_l) < 10.0 and abs(vel_r) < 10.0, "deadman never stopped the robot"

    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 0


# test_mover_rejects_time_plus_distance -- DELETED (097-006): no binary
# equivalent. The deleted text scenario was parseMover()'s own
# `t > 0.0f && distance != 0` mutual-exclusivity guard
# (motion_commands.cpp), rejected with `ERR badarg t+distance` -- a
# text-parse-time-only convenience. binary_channel.cpp's `replace` handler
# (`handleReplace()`) posts `toSegment(src)` to `bb.replaceIn`
# UNCONDITIONALLY (that function's own comment: "no hand parsing/range
# checks... the generated decoder's own bound checks") -- it never
# replicated this specific guard, so there is nothing on the binary path
# this test could re-point to; deleted per this ticket's "no binary
# behavior -> delete, don't force a mapping" instruction, applied here at
# the validation-behavior level (MOVER the verb otherwise has a binary
# `replace` arm, exercised by test_mover_deadman_velocity above).


def test_pivot_completes_promptly_single_peaked(sim):
    """REGRESSION (multi-hump pivot + STOP_TIME stall, 2026-07-09): an
    in-place turn must execute as ONE velocity peak (no decaying re-solve
    humps) and report idle promptly after its plan exhausts -- not sit out
    the ~2.5s STOP_TIME net. Single-peak check: once |vel_r| has exceeded
    60 mm/s and then fallen below 20 mm/s, it must never rise above 40 mm/s
    again. 097-006: binary `segment`."""
    reply = send_segment(sim, legacy_translate.segment_for_move(0, 0, 9000))
    assert reply.WhichOneof("body") == "ok"
    peaked = fallen = False
    idle_at = None
    for i in range(160):   # 3.84 s at 24 ms
        sim.tick_for(24)
        _, vel_r = sim.vel()
        if abs(vel_r) > 60.0:
            assert not fallen, f"second velocity hump at t={(i+1)*0.024:.2f}s (|vel_r|={vel_r})"
            peaked = True
        elif peaked and abs(vel_r) < 20.0:
            fallen = True
        if fallen and abs(vel_r) > 40.0:
            raise AssertionError(f"pivot re-accelerated after settling (|vel_r|={vel_r})")
        if idle_at is None and fallen:
            _, _, _, _, active = _parse_tlm(sim.command("TLM"))
            if active == 0:
                idle_at = (i + 1) * 0.024
    assert peaked, "pivot never drove"
    assert idle_at is not None and idle_at < 3.0, \
        f"pivot did not report idle promptly (idle_at={idle_at})"


# ---------------------------------------------------------------------------
# Timing/ordering guarantees (architecture-update.md Section 5).
# ---------------------------------------------------------------------------

def test_move_sent_mid_slack_takes_effect_on_next_mandatory_tick(sim):
    """`_binary_envelope.send()`'s own dt=0 synchronous-command trick
    already replays exactly one `Rt::MainLoop::tick()` at the unchanged
    `now` immediately after routing (`sim_api.cpp`'s file header) -- so a
    `TLM` read taken IMMEDIATELY after `MOVE`, with no separate
    `sim.tick_for()` call in between, already shows the segment claimed
    SEGMENT mode. Proves `segmentIn -> ring_ -> executor` happens within
    that one mandatory tick, not a multi-hop mailbox latency. 097-006:
    binary `segment`."""
    reply = send_segment(sim, legacy_translate.segment_for_move(300, 0, 0))
    assert reply.WhichOneof("body") == "ok"

    _, _, _, _, active = _parse_tlm(sim.command("TLM"))
    assert active == 1


def test_two_moves_queued_back_to_back_both_execute_in_order(sim):
    """Two `MOVE`s posted with NO intervening `sim.tick_for()` call both
    execute (in order) -- proves `bb.segmentIn`'s `Rt::WorkQueue<
    Motion::Segment, 8>` shape does not silently drop the first the way a
    latest-wins `Mailbox` would (architecture-update.md Section 5,
    "Command precedence and the 'no hiccups' requirement"). 097-006: binary
    `segment`."""
    reply1 = send_segment(sim, legacy_translate.segment_for_move(200, 0, 0))
    assert reply1.WhichOneof("body") == "ok"
    reply2 = send_segment(sim, legacy_translate.segment_for_move(200, 0, 0))
    assert reply2.WhichOneof("body") == "ok"

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
