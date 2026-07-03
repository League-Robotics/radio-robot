"""
test_069_004_encoder_otos_knobs.py — ticket 069-004 behavioral isolation
tests for the newly-surfaced per-wheel encoder-report-error and OTOS-error
`SIMSET`/`SIMGET` keys.

`SIMSET`/`SIMGET` round-trip coverage (every new key readable/writable, plus
the `otosLinDriftMmS`/`otosYawDriftDegS` per-second<->per-tick unit
conversion) lives in `tests/simulation/unit/test_sim_commands_registry.py`.
This module covers the *behavioral* half of ticket 069-004's acceptance
criteria: each error dimension must be isolated to the pose estimator that
actually reads it, per SUC-002/SUC-005.

Empirically verified (see the two test docstrings below for the numbers)
rather than assumed:

  - `encScaleErrL` (a `PhysicsWorld` reported-ENCODER error, 058-001 lineage)
    perturbs `TLM enc=` (the raw per-wheel reading) AND, because the
    encoder-only dead-reckoning accumulator (`Odometry`'s `_encPoseX/Y/H`,
    wire field `encpose=`) is arc-integrated directly from those same
    per-wheel reads and is *never touched by the EKF* (see Odometry.cpp's
    own comment), it visibly diverges from the plant's true pose too. What
    stays close to true is `otos=` (`SimOdometer` samples `PhysicsWorld`
    ground truth directly — the encoder-report-error channel is entirely
    outside its input path) and, with OTOS fusion enabled, the EKF-fused
    `pose=` (corrected back toward the OTOS/true trajectory every tick).
  - `otosLinScaleErr` (a `SimOdometer` OTOS-report error) perturbs `otos=`'s
    reported distance relative to true, while leaving the encoder-only
    `encpose=`/`enc=` path completely untouched (`SimOdometer` and
    `PhysicsWorld`'s encoder-report-error model are disjoint inputs).
"""
from __future__ import annotations

import math

from robot_radio.robot.protocol import parse_tlm


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def test_enc_scale_error_diverges_encoder_path_not_otos_or_fused(sim) -> None:
    """`encScaleErrL` (only) diverges TLM `enc=` (raw per-wheel) and the
    encoder-only `encpose=` accumulator from the plant's true trajectory,
    while `otos=`/the EKF-fused `pose=` (OTOS fusion enabled) stay close to
    true.

    Empirical run (5% left-encoder over-report, `T 200 200 2000` = 2 s
    straight drive at 200 mm/s commanded on both wheels): the reported
    per-wheel scale error also perturbs the closed-loop velocity controller
    (it servos to REPORTED, not true, velocity), so the plant's TRUE path
    itself curves slightly -- true=(382.1, 19.3, 0.117 rad). `otos=` and
    `pose=` track that actual curved path almost exactly (both within
    ~0.1 mm / ~0.0001 rad of true), because neither reads the encoder-report
    channel. `encpose=` does NOT track it -- it accumulates the SAME biased
    per-wheel deltas independently of the EKF's OTOS correction and lands at
    (392.1, -8.3, -0.027 rad), ~29 mm / ~0.14 rad (~8°) away from true.
    """
    sim.enable_otos_model()
    sim.set_otos_fusion(True)

    reply = sim.send_command("SIMSET encScaleErrL=0.05")
    assert reply.upper().startswith("OK"), reply

    reply = sim.send_command("T 200 200 2000")
    assert reply.upper().startswith("OK"), reply
    sim.tick_for(2500)

    reply = sim.send_command("SNAP")
    frame = parse_tlm(reply)
    assert frame is not None, f"SNAP did not parse as a TLM frame: {reply!r}"
    assert frame.enc is not None, f"SNAP frame missing enc=: {reply!r}"
    enc_l, enc_r = frame.enc
    assert enc_l != enc_r, (
        f"encScaleErrL=0.05 (left only) should make TLM enc= diverge "
        f"left-vs-right after a straight drive; got enc={frame.enc}"
    )

    true_x, true_y, _true_h = sim.get_true_pose()
    otos_x, otos_y, _otos_h = sim.get_otos_pose()
    fused_x, fused_y, _fused_h = sim.get_pose()
    enc_x, enc_y, _enc_h = sim.get_enc_pose()

    true_xy = (true_x, true_y)
    otos_dist = _dist((otos_x, otos_y), true_xy)
    fused_dist = _dist((fused_x, fused_y), true_xy)
    enc_dist = _dist((enc_x, enc_y), true_xy)

    assert otos_dist < 3.0, (
        f"otos= should track the plant's true pose closely (it never reads "
        f"the encoder-report-error channel); got {otos_dist:.2f} mm off true"
    )
    assert fused_dist < 5.0, (
        f"EKF-fused pose= (OTOS fusion enabled) should be corrected back "
        f"toward true/otos every tick; got {fused_dist:.2f} mm off true"
    )
    assert enc_dist > 15.0, (
        f"encoder-only dead-reckoning (encpose=) should visibly diverge "
        f"from true when encScaleErrL is injected; got only "
        f"{enc_dist:.2f} mm off true"
    )


def test_otos_scale_error_perturbs_otos_not_encoder_path(sim) -> None:
    """`otosLinScaleErr` (only) changes `otos=`'s reported distance relative
    to the plant's true pose, without perturbing `encpose=`/`enc=` at all.

    Empirical run (10% OTOS linear scale error, `T 200 200 2000` straight
    drive): true=(390.3, 0, 0); otos=(429.4, 0, 0) — almost exactly
    true * 1.10, confirming the scale error is applied; encpose=(390.3, 0, 0)
    — bit-for-bit unaffected, because `SimOdometer`'s error model is a
    disjoint input path from `PhysicsWorld`'s encoder-report-error model.
    """
    sim.enable_otos_model()

    reply = sim.send_command("SIMSET otosLinScaleErr=0.10")
    assert reply.upper().startswith("OK"), reply

    reply = sim.send_command("T 200 200 2000")
    assert reply.upper().startswith("OK"), reply
    sim.tick_for(2500)

    true_x, true_y, _true_h = sim.get_true_pose()
    otos_x, otos_y, _otos_h = sim.get_otos_pose()
    enc_x, enc_y, _enc_h = sim.get_enc_pose()

    true_dist = math.hypot(true_x, true_y)
    otos_dist = math.hypot(otos_x, otos_y)

    assert otos_dist > true_dist * 1.05, (
        f"otosLinScaleErr=0.10 should inflate otos='s reported distance "
        f"relative to the plant's true pose; otos_dist={otos_dist:.2f} "
        f"true_dist={true_dist:.2f}"
    )
    assert abs(enc_x - true_x) < 1.0 and abs(enc_y - true_y) < 1.0, (
        f"an OTOS-only error must not perturb the encoder-only path "
        f"(encpose=); enc=({enc_x:.3f},{enc_y:.3f}) "
        f"true=({true_x:.3f},{true_y:.3f})"
    )
