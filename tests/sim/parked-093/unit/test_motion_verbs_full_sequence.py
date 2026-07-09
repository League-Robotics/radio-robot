"""Off-hardware acceptance proof for ticket 084-009 (SUC-001..004): the
CONSOLIDATED cross-verb pass over tickets 001-005's own verb families --
``D``/``RT``/``T``/``S``/``R``/``TURN``/``G``/``STOP`` plus ``stop=``
clauses and ``mode=`` -- driven together in single, continuing sim sessions
rather than the one-verb-per-fresh-``sim`` shape ``test_motion_commands.py``/
``test_motion_commands_arc_turn.py``/``test_motion_commands_goto.py``/
``test_mode_machine.py`` each use.

This file does not re-derive new tolerances -- per ticket 009's own
acceptance wording ("a consolidated re-run across the full verb set, not a
re-derivation of new tolerances"), every numeric bound below is taken
directly from those four files' own measured-plant-behavior comments, or
freshly measured here (dated) using the identical methodology (``sim.
true_pose()``/``SNAP`` against this exact build) where a genuinely new
cross-verb combination (chained verbs, two racing user ``stop=`` clauses,
sensor/color/line rejection restated across the WHOLE verb family rather
than just ``D``) has no single earlier ticket's test to borrow a number
from.

No production ``source/`` file is touched by this ticket -- test-only, per
ticket 009's own Implementation Plan.
"""

from __future__ import annotations

import math

import pytest


def _parse_tlm(line: str) -> dict[str, str]:
    """Parse one "TLM t=... mode=... ..." wire line into a key->value dict.

    Local, small, deliberately duplicated per test-file precedent (see
    test_tlm_stream_snap.py's/test_mode_machine.py's own copies)."""
    parts = line.strip().split()
    assert parts[0] == "TLM", f"not a TLM line: {line!r}"
    return dict(p.split("=", 1) for p in parts[1:])


def _mode(sim) -> str:
    """Issue SNAP and return just its mode= value -- test_mode_machine.py's
    own ``_mode()`` helper, duplicated here per this directory's
    precedent."""
    reply = sim.command("SNAP").strip()
    lines = reply.splitlines()
    assert len(lines) == 1, f"expected exactly one TLM line from SNAP, got: {reply!r}"
    return _parse_tlm(lines[0])["mode"]


# ---------------------------------------------------------------------------
# Bullets 1+2: D 200 200 500 (~500 mm, EVT done D reason=dist) chained
# directly into RT 9000 (~90 deg, EVT done RT reason=rot) in ONE continuing
# session -- proving RT's closed loop resolves correctly against whatever
# pose D left behind, not just from a pristine (0,0,0) boot state every
# single-verb test file exercises it from.
# ---------------------------------------------------------------------------


def test_d_then_rt_chained_in_one_session_moves_pose_and_emits_reason_dist_and_rot(sim):
    reply = sim.command("D 200 200 500")
    assert reply.strip() == "OK drive l=200 r=200 mm=500"

    sim.tick_for(3000)
    x, y, h = sim.true_pose()
    # Same +-100 mm / +-5 mm / +-0.05 rad bounds test_motion_commands.py's own
    # test_d_moves_true_pose_and_emits_done_dist() uses at boot -- unaffected
    # by chaining since this is the FIRST command in this session too.
    assert abs(x - 500.0) < 100.0, f"expected true x near 500 mm, got {x}"
    assert abs(y) < 5.0
    assert abs(h) < 0.05

    evts = sim.get_async_evts()
    assert "EVT done D reason=dist" in evts

    reply = sim.command("RT 9000")
    assert reply.strip() == "OK rt rot=9000"

    sim.tick_for(3000)
    _x2, _y2, h2 = sim.true_pose()
    # Measured plant behavior (2026-07-06, chained after the D above): RT's
    # in-place rotation is not perfectly stationary in x/y once chained after
    # a prior drive (a small lateral/longitudinal creep from the wheel-arc
    # discretization during the ramp) -- test_motion_commands_arc_turn.py's
    # own +-10 deg heading tolerance for a 90 deg RT is reused unchanged
    # here; x/y are deliberately not asserted precisely in this chained
    # context (that precision belongs to the boot-state RT tests in that
    # file), only that the heading delta itself still closes correctly.
    expected_delta = math.pi / 2.0
    delta = h2 - h  # h was ~0 after the D leg above
    assert abs(delta - expected_delta) < math.radians(10.0), (
        f"expected ~90 deg RT rotation on top of D's heading, got delta={math.degrees(delta):.2f} deg"
    )

    evts = sim.get_async_evts()
    assert "EVT done RT reason=rot" in evts


# ---------------------------------------------------------------------------
# Bullet 3: T/S/R/TURN/G/STOP each exercised in the SAME continuing session
# (chained onward from wherever the previous verb left the plant), each
# producing its own documented EVT reason= token (docs/protocol-v2.md
# section 10's own table) -- the cross-verb composability no single ticket
# 002-004 test file owns (each of those starts every test from a fresh,
# pristine sim).
# ---------------------------------------------------------------------------


def test_full_verb_family_chained_sequence_emits_expected_completion_reasons(sim):
    # T -- timed drive; reason=time (test_motion_commands.py's own margin).
    reply = sim.command("T 150 150 800")
    assert reply.strip() == "OK drive l=150 r=150 ms=800"
    sim.tick_for(200)
    assert "EVT done T" not in sim.get_async_evts(), "well short of the 800 ms duration"
    sim.tick_for(2000)
    assert "EVT done T reason=time" in sim.get_async_evts()

    # S -- open-ended streaming drive; STOP halts it with NO EVT at all
    # (test_motion_commands.py's own test_stop_halts_immediately_with_no_evt).
    reply = sim.command("S 120 120")
    assert reply.strip() == "OK drive l=120 r=120"
    sim.tick_for(200)
    assert "EVT done" not in sim.get_async_evts()
    reply = sim.command("STOP")
    assert reply.strip() == "OK stop"
    assert sim.get_async_evts() == ""

    # R -- a bounded arc (own stop=t: clause); reason=time
    # (test_motion_commands_arc_turn.py's own margin).
    reply = sim.command("R 200 500 stop=t:1500")
    assert reply.strip() == "OK arc speed=200 radius=500"
    sim.tick_for(2500)
    assert "EVT done R reason=time" in sim.get_async_evts()

    # TURN -- absolute-heading turn-in-place; reason=heading
    # (test_motion_commands_arc_turn.py's own margin). Target is whatever
    # heading TURN itself resolves the shortest path to -- this test does
    # not care which absolute heading, only that TURN's own built-in
    # HEADING stop still fires correctly after the R leg above.
    reply = sim.command("TURN 0")
    assert reply.strip() == "OK turn heading=0 eps=300"
    sim.tick_for(3000)
    assert "EVT done TURN reason=heading" in sim.get_async_evts()

    # G -- relative go-to from wherever the chain above left the plant;
    # reason=pos (test_motion_commands_goto.py's own margin).
    x0, y0, _h0 = sim.true_pose()
    reply = sim.command(f"G {int(x0) + 300} {int(y0)} 200")
    assert reply.strip() == f"OK goto x={int(x0) + 300} y={int(y0)} speed=200"
    sim.tick_for(6000)
    assert "EVT done G reason=pos" in sim.get_async_evts()


# ---------------------------------------------------------------------------
# Bullet 5: mode= returns to 'I' at completion of EVERY verb family, polled
# via SNAP alone -- this test NEVER calls sim.get_async_evts() anywhere,
# proving mode=I is sufficient for tour-completion detection completely
# independent of whether the host drains a single EVT line (SUC-004's own
# tour-runner motivation). test_mode_machine.py already proves this per verb
# from a pristine boot state; this is the first-class, whole-session version
# spanning the full family in one chained run (measured 2026-07-06 against
# this exact build).
# ---------------------------------------------------------------------------


def test_mode_returns_to_idle_across_full_multiverb_tour_polled_via_snap_only(sim):
    assert _mode(sim) == "I"

    sim.command("D 200 200 500")
    assert _mode(sim) == "D"
    sim.tick_for(3000)
    assert _mode(sim) == "I"

    sim.command("RT 9000")
    assert _mode(sim) == "T"
    sim.tick_for(3000)
    assert _mode(sim) == "I"

    sim.command("T 150 150 800")
    assert _mode(sim) == "T"
    sim.tick_for(200)
    assert _mode(sim) == "T"
    sim.tick_for(2000)
    assert _mode(sim) == "I"

    sim.command("S 120 120")
    assert _mode(sim) == "S"
    sim.tick_for(200)
    assert _mode(sim) == "S", "S is open-ended -- mode stays S until STOP"
    sim.command("STOP")
    assert _mode(sim) == "I"

    sim.command("R 200 500 stop=t:1500")
    assert _mode(sim) == "T", "a stop=-bearing R shares the TIMED bucket (Decision 6)"
    sim.tick_for(2500)
    assert _mode(sim) == "I"

    sim.command("TURN 0")
    assert _mode(sim) == "T"
    sim.tick_for(3000)
    assert _mode(sim) == "I"

    x0, y0, _h0 = sim.true_pose()
    sim.command(f"G {int(x0) + 300} {int(y0)} 200")
    assert _mode(sim) == "G"
    sim.tick_for(6000)
    assert _mode(sim) == "I"


# ---------------------------------------------------------------------------
# Bullet 4a: stop= clauses honored, OR-combined -- extended here to TWO
# user-supplied clauses of DIFFERENT kinds racing each other (not just one
# user clause racing a verb's own built-in stop, which is all tickets
# 002-003's own test files exercise) -- confirms the OR-combination is
# genuine across multiple clauses, not merely "one clause beats the
# built-in."
# ---------------------------------------------------------------------------


def test_two_user_stop_clauses_or_combined_first_to_fire_wins(sim):
    # stop=t:150 (~150 ms) fires well before stop=d:400 (400 mm at 500 mm/s
    # ~= 800 ms) and well before the 5000 mm built-in distance stop.
    reply = sim.command("D 500 500 5000 stop=t:150 stop=d:400")
    assert reply.strip() == "OK drive l=500 r=500 mm=5000"

    sim.tick_for(2000)
    evts = sim.get_async_evts()
    assert "EVT done D reason=time" in evts, (
        f"expected the faster stop=t:150 clause to win the OR-combination race, got {evts!r}"
    )


# ---------------------------------------------------------------------------
# Bullet 4b: sensor/color/line clauses (and the sensor= back-compat alias)
# reject with ERR badarg -- test_motion_commands.py's own
# test_stop_clause_sensor_color_line_rejected_with_badarg only exercises
# this against D; this consolidated pass restates it across the WHOLE
# open-loop verb family (docs/protocol-v2.md section 10's own list: VW, S,
# R, T, D, TURN, RT) confirming the rejection is uniform, not
# accidentally D-specific.
# ---------------------------------------------------------------------------
_SENSOR_REJECT_CASES = [
    ("S 100 100 stop=sensor:line0:ge:512", "S"),
    ("T 100 100 1000 stop=color:120:0.5:0.4:0.1", "T"),
    ("D 100 100 200 stop=line:ge:512", "D"),
    ("R 100 500 stop=sensor:line0:ge:512", "R"),
    ("TURN 9000 stop=color:120:0.5:0.4:0.1", "TURN"),
    ("RT 9000 stop=line:ge:512", "RT"),
    ("T 100 100 1000 sensor=line0:ge:512", "T (sensor= alias)"),
]


@pytest.mark.parametrize("command_line,label", _SENSOR_REJECT_CASES, ids=[c[1] for c in _SENSOR_REJECT_CASES])
def test_stop_clause_sensor_color_line_rejected_across_verb_family(sim, command_line, label):
    reply = sim.command(command_line)
    assert reply.strip().startswith("ERR badarg"), (
        f"expected ERR badarg for {label}'s unsupported sensor/color/line clause, got {reply!r}"
    )

    # None of the rejected commands should have staged a goal.
    x, y, h = sim.true_pose()
    assert (x, y, h) == (0.0, 0.0, 0.0), (
        f"{label}'s rejected clause must not have moved the robot, got pose=({x}, {y}, {h})"
    )
