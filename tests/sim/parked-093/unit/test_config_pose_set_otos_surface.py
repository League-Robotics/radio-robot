"""Off-hardware acceptance proof for ticket 084-009 (SUC-005..007): the
CONSOLIDATED cross-cutting pass over tickets 006 (``SET``/``GET``), 007
(``SI``/``ZERO enc``), and 008 (the seven ``OI``/``OZ``/``OR``/``OP``/``OV``/
``OL``/``OA`` verbs) -- exercised together within single, continuing sim
sessions rather than each ticket's own one-surface-per-fresh-``sim`` shape
(``test_config_registry.py``/``test_pose_commands.py``/
``test_otos_commands.py``).

Per ticket 009's own acceptance wording, this is a re-run/extension, not a
re-derivation: every numeric bound below either reuses a number already
established by one of those three files, or was freshly measured here
(dated) against this exact build using the identical methodology, for a
genuinely new cross-surface combination none of the three tickets' own test
files exercises on its own (a config change's effect on a CLOSED-LOOP motion
verb's real outcome; ``SI``'s effect readable through the OTOS ``OP`` verb
ticket 008 added after ticket 007 shipped; the full Decision-2 dropped-key
table, which ticket 006's own test only partially enumerated; ``ZERO enc``
chained after a real top-level drive verb rather than the raw ``DEV S``
ticket 007's own test drives with).

No production ``source/`` file is touched by this ticket -- test-only, per
ticket 009's own Implementation Plan.
"""

from __future__ import annotations

import math

import pytest


def _parse_tlm(line: str) -> dict[str, str]:
    """Parse one "TLM t=... mode=... ..." wire line into a key->value dict.

    Local, small, deliberately duplicated per test-file precedent (see
    test_tlm_stream_snap.py's/test_pose_commands.py's own copies)."""
    parts = line.strip().split()
    assert parts[0] == "TLM", f"not a TLM line: {line!r}"
    return dict(p.split("=", 1) for p in parts[1:])


def _snap(sim) -> dict[str, str]:
    """Issue SNAP and parse its reply (see test_pose_commands.py's own
    _snap() for the "exactly one TLM line" precondition)."""
    reply = sim.command("SNAP").strip()
    lines = reply.splitlines()
    assert len(lines) == 1, f"expected exactly one TLM line from SNAP, got: {reply!r}"
    return _parse_tlm(lines[0])


# ---------------------------------------------------------------------------
# SET tw=...-then-GET round-trips (ticket 006's own headline proof, via
# DEV DT STATE) AND visibly changes CLOSED-LOOP turn geometry -- a stronger,
# genuinely cross-cutting demonstration ticket 006's own test (a
# DEV-DT-VW/STATE inspection of pre-governor commanded wheel targets) does
# not reach: RT's own built-in ROTATION stop threshold
# (source/commands/motion_commands.cpp's handleRT: arc = |relAngle| *
# (trackwidth/2)) is computed from PoseEstimator's CONFIGURED trackwidth,
# while the sim plant's physical rotation rate is governed by the FIXED
# true trackwidth (Hal::PhysicsWorld::kDefaultTrackwidth = 150 mm, see
# test_config_registry.py's own boot-dump comment). Deliberately
# mis-calibrating tw below (100, well off the true 150) makes RT 9000's
# REAL achieved rotation visibly undershoot 90 deg -- proof the trackwidth
# change has a genuine physical consequence, not just an echoed number.
# ---------------------------------------------------------------------------


def test_set_tw_get_round_trip_and_visible_effect_on_closed_loop_turn_geometry(sim):
    reply = sim.command("SET tw=100")
    assert reply.strip() == "OK set tw=100"
    reply = sim.command("GET tw")
    assert reply.strip() == "CFG tw=100"

    reply = sim.command("RT 9000")
    assert reply.strip() == "OK rt rot=9000"
    sim.tick_for(3000)

    _x, _y, h = sim.true_pose()
    # Measured plant behavior (2026-07-06): with tw=100 (well under the true
    # 150 mm plant trackwidth), RT's ROTATION stop fires once the per-wheel
    # arc reaches |90 deg| * (100/2) = 78.5 mm -- but at the plant's TRUE
    # 150 mm trackwidth that same 78.5 mm of differential arc corresponds to
    # only ~60 deg of REAL rotation (78.5 / (150/2) rad), not 90. Measured
    # true heading: ~65.3 deg (60 deg plus the usual SMOOTH-stop coast
    # documented throughout test_motion_commands_arc_turn.py). This is
    # clearly outside that file's own +-10 deg window around 90 deg for an
    # UNCHANGED (default 150) trackwidth -- the deliberate point of this
    # test: SET tw= visibly, physically changes what a 90 deg RT actually
    # achieves.
    h_deg = math.degrees(h)
    assert 45.0 < h_deg < 80.0, (
        f"expected RT 9000 to visibly UNDERSHOOT 90 deg under a mis-set "
        f"tw=100 (true trackwidth is 150), got {h_deg:.2f} deg"
    )

    tlm = _snap(sim)
    # PoseEstimator's OWN encoder-only belief (encpose=), computed with the
    # SAME (mis-set) trackwidth it used to size the stop threshold, is
    # internally self-consistent and still reads close to the COMMANDED
    # 9000 cdeg -- it has no way to know its trackwidth belief disagrees
    # with the plant's true geometry. Generous bound covers the coast.
    encpose_h_cdeg = int(tlm["encpose"].split(",")[2])
    assert 7500 < encpose_h_cdeg < 11000, (
        f"expected encpose= heading near the commanded 9000 cdeg "
        f"(PoseEstimator's own, self-consistent belief), got {encpose_h_cdeg}"
    )


# ---------------------------------------------------------------------------
# Every key Decision 2 (architecture-update.md (084)) dropped -> ERR badkey
# on both SET and GET. test_config_registry.py's own _DROPPED_KEYS list
# (kff/klf/klb/krf/krb/adjThr/adjGain/distScale/turnScale/tick/tlmPeriod --
# 11 keys) omits six more rows the SAME Decision-2 table (docs/
# protocol-v2.md section 7's Named Key Table) also marks "superseded"/"not
# carried forward": pid.kd, pid.max, ekfQv, ekfQomega, ekfROtosV, ekfREncV.
# This is this consolidated ticket's own full re-run of the COMPLETE table,
# not a correction of a bug -- confirmed by inspection of
# source/commands/config_commands.cpp's kRegisteredKeys list (and its SET/
# GET key-match chains): none of these six is specially handled either;
# they fall through to the same generic "unregistered key" path every
# never-existed key does.
# ---------------------------------------------------------------------------
_ALL_DROPPED_KEYS = [
    "kff", "klf", "klb", "krf", "krb",
    "adjThr", "adjGain",
    "pid.kd", "pid.max",
    "distScale", "turnScale",
    "tick", "tlmPeriod",
    "ekfQv", "ekfQomega", "ekfROtosV", "ekfREncV",
]


@pytest.mark.parametrize("key", _ALL_DROPPED_KEYS)
def test_all_decision_2_dropped_keys_return_err_badkey_on_set(sim, key):
    assert sim.command(f"SET {key}=1").strip() == f"ERR badkey {key}"


@pytest.mark.parametrize("key", _ALL_DROPPED_KEYS)
def test_all_decision_2_dropped_keys_return_err_badkey_on_get(sim, key):
    assert f"ERR badkey {key}" in sim.command(f"GET {key}")


# ---------------------------------------------------------------------------
# SI teleports the fused pose (confirmed via SNAP) AND -- ticket 008's own
# odometer re-anchor closure -- the SAME teleport is immediately visible
# through the OTOS OP verb too (ticket 008 did not exist when ticket 007's
# own test_pose_commands.py was written; this is the first test to read
# the SI effect back through BOTH surfaces in one session).
# ---------------------------------------------------------------------------


def test_si_teleports_fused_pose_confirmed_via_snap_and_through_otos_op(sim):
    before = _snap(sim)
    assert before["pose"] == "0,0,0"
    assert sim.command("OP").strip() == "OK pos x=0 y=0 h=0"

    reply = sim.command("SI 1000 500 900")
    assert reply.strip() == "OK setpose x=1000 y=500 h=900"

    tlm = _snap(sim)
    assert tlm["pose"] == "1000,500,900"
    assert tlm["encpose"] == "1000,500,900"

    # Hal::SimOdometer::setPose() writes odomX_/odomY_/odomH_ synchronously
    # (no tick needed) -- OP's own cheap accessor read (CMD_NONE, no
    # tick()) reflects the SI-triggered internal SET_POSE arm immediately.
    assert sim.command("OP").strip() == "OK pos x=1000 y=500 h=900"


# ---------------------------------------------------------------------------
# ZERO enc rezeroes enc=/encpose= with no phantom-jump discontinuity --
# ticket 007's own test_pose_commands.py chains this after a raw `DEV S`/
# `DEV STOP` pair; this consolidated pass instead chains it after a REAL
# top-level closed-loop drive verb (D, tickets 002/084-001's own Planner
# path), proving the guarantee holds identically when reached through the
# wire surface an actual host uses, not just the DEV bench-test surface.
# ---------------------------------------------------------------------------


def test_zero_enc_after_a_real_drive_verb_rezeroes_with_no_phantom_jump(sim):
    reply = sim.command("D 200 200 500")
    assert reply.strip() == "OK drive l=200 r=200 mm=500"
    sim.tick_for(3000)
    assert "EVT done D reason=dist" in sim.get_async_evts()
    sim.tick_for(200)   # settle, matching test_pose_commands.py's own precedent

    before = _snap(sim)
    x_before, y_before, h_before = (int(v) for v in before["encpose"].split(","))
    assert x_before > 400, f"expected substantial accumulated travel before ZERO, got x={x_before}"

    sim.command("ZERO enc")
    # Hal::Motor::resetPosition() is staged (hal/capability/motor.h) -- one
    # real tick lands the leaf's own hardware zero, matching
    # test_pose_commands.py's own precedent.
    sim.tick_for(24)

    enc_l, enc_r = sim.enc()
    # Looser than test_pose_commands.py's own <1.0 mm bound (that file
    # chains after a raw DEV S + STOP + settle sequence with a slightly
    # different terminal-velocity profile than a completed D's own SMOOTH
    # stop-and-settle) -- still an unambiguous, order-of-magnitude rezero
    # from the >400 mm accumulated above.
    assert abs(enc_l) < 5.0 and abs(enc_r) < 5.0, (
        f"expected ZERO enc to rezero the reported encoders, got enc=({enc_l}, {enc_r})"
    )

    after = _snap(sim)
    x_after, y_after, h_after = (int(v) for v in after["encpose"].split(","))
    # No phantom jump: encpose= (the ACCUMULATED world pose, which ZERO enc
    # does not itself move -- only PoseEstimator's forward encoder-delta
    # BASELINE resyncs) must stay close to its pre-ZERO value.
    assert abs(x_after - x_before) < 10, (
        f"phantom jump detected: encpose x moved from {x_before} to {x_after} "
        f"across ZERO enc's first following tick"
    )
    assert abs(y_after - y_before) < 10
    assert abs(h_after - h_before) < 50


# ---------------------------------------------------------------------------
# All seven OTOS verbs ack against the sim, chained back-to-back in ONE
# session (ticket 008's own test_otos_commands.py exercises each verb from
# its own fresh sim) -- proving the OtosCommandState shadow (OL/OA) and the
# live hardware.odometer() resolution survive a full round of every verb
# firing in sequence with no state corruption between them.
# ---------------------------------------------------------------------------


def test_all_seven_otos_verbs_chained_in_one_session_ack_against_the_sim(sim):
    assert sim.command("OI").strip() == "OK oi"
    assert sim.command("OZ").strip() == "OK oz"
    assert sim.command("OR").strip() == "OK or"
    assert sim.command("OP").strip() == "OK pos x=0 y=0 h=0"
    assert sim.command("OV 100 200 300").strip() == "OK setpos x=100 y=200 h=300"
    assert sim.command("OP").strip() == "OK pos x=100 y=200 h=300"
    assert sim.command("OL 5").strip() == "OK linear scalar=5"
    assert sim.command("OL").strip() == "OK linear scalar=5"
    assert sim.command("OA -3").strip() == "OK angular scalar=-3"
    assert sim.command("OA").strip() == "OK angular scalar=-3"
