"""Sim verification for ticket 082-005 (SUC-005): a drive sequence (straight
leg + turn leg, driven via ``DEV DT VW``) run through the compiled
``libfirmware_host`` shows ``TLM``'s ``pose=`` (EKF-fused) and ``encpose=``
(encoder-only dead-reckoning) tracking the ctypes ground-truth pose
(``sim_get_true_pose_*`` -- ``firmware.py``'s ``Sim.true_pose()``) within the
plant's OWN documented tolerance, for the WHOLE sequence (sampled every
tick via ``SNAP``), not just at rest.

Two different tolerance bands, by design (not sloppiness -- see the two
constants' own comments below):

  - ``pose=`` (``fusedPose()``) is corrected every tick by a fresh, accurate
    (zero-error-knob) OTOS observation (``Hal::SimOdometer::tick()`` samples
    the plant's true pose AFTER this pass's ``Hal::PhysicsWorld::update()``
    call -- see ``dev_loop.cpp``'s pose-estimation step, which calls
    ``odometer->tick(now)`` right before ``poseEstimator->tick()``) -- so it
    tracks true tightly.
  - ``encpose=`` (``encoderPose()``) is fed by ``Hal::Motor::position()``,
    which is ``Hal::SimMotor::position()`` -- a CACHED sample taken at the
    START of ``SimMotor::tick()``, i.e. BEFORE this pass's
    ``Hal::PhysicsWorld::update()`` call advances the plant (see
    ``sim_motor.cpp``'s ``tick()`` step-2 comment and ``sim_hardware.h``'s
    file header on the two-call-per-pass re-entry guard). This is the SAME
    "one-tick sample latency" ``firmware.py``'s own docstring documents for
    ``velocity()`` -- ``position()`` inherits the identical lag by
    construction, since both come from the same ``lastPosition_`` cache.
    ``encpose=`` therefore lags true by roughly one devLoopTick pass's worth
    of motion (~24 ms at the current wheel speed), which is a real,
    documented simulation-design artifact, not a defect -- see this file's
    empirically-measured tolerance constants below.

Tolerances were derived by running this exact drive script and recording
the observed max divergence (straight leg: pose xy <= ~3.1 mm, pose h == 0;
encpose xy <= ~7.1 mm, encpose h == 0 -- no heading change on a straight
run; turn leg: pose xy <= ~1.3 mm, pose h <= ~0.23 deg; encpose xy <=
~2.1 mm, encpose h <= ~3.6 deg during the velocity-ramp transient, settling
to ~2 deg), then applying a >=4x margin so the test is not flaky against
minor future PID-tuning or plant-default changes while still catching a
genuine correctness regression (e.g. a EKF wiring break that made ``pose=``
track as loosely as ``encpose=``, or an ``encpose=`` blow-up well past the
one-tick-latency order of magnitude).
"""
from __future__ import annotations

import math

CDEG_PER_RAD = 5729.5779513   # kAngleScale, tlm_frame.cpp -- centidegrees per radian

# Empirically-derived, margined tolerances (see file header for the
# measurements they are based on).
_POSE_XY_TOL = 15.0     # [mm] fused pose position -- observed max ~3.1mm
_POSE_H_TOL = math.radians(2.0)     # fused pose heading -- observed max ~0.23deg
_ENCPOSE_XY_TOL = 25.0  # [mm] encoder-only position -- observed max ~7.1mm
_ENCPOSE_H_TOL = math.radians(8.0)  # encoder-only heading -- observed max ~3.6deg


def _parse_tlm(line: str) -> dict[str, str]:
    """Parse one "TLM t=... mode=... ..." wire line into a key->value dict.

    Local, small, deliberately duplicated per test file rather than shared
    via conftest.py -- mirrors this directory's existing precedent of
    per-file helpers (e.g. ``_drive_straight`` duplicated across
    test_otos_error_injection.py / test_errored_observation.py) rather than
    a shared test-util module.
    """
    parts = line.strip().split()
    assert parts[0] == "TLM", f"not a TLM line: {line!r}"
    return dict(p.split("=", 1) for p in parts[1:])


def _snap(sim) -> dict[str, str]:
    """Issue SNAP and parse its (always exactly one line, since no test in
    this file ever enables STREAM) reply."""
    reply = sim.command("SNAP").strip()
    lines = reply.splitlines()
    assert len(lines) == 1, f"expected exactly one TLM line from SNAP, got: {reply!r}"
    return _parse_tlm(lines[0])


def _wrapped_deg_diff(a_rad: float, b_rad: float) -> float:
    """abs(a - b) in degrees, wrapped to (-180, 180] first."""
    diff = math.atan2(math.sin(a_rad - b_rad), math.cos(a_rad - b_rad))
    return abs(math.degrees(diff))


def test_pose_and_encpose_track_true_pose_across_straight_and_turn(sim):
    """Drive straight, then turn, sampling TLM via SNAP every tick, and
    assert pose=/encpose= track sim.true_pose() within their respective
    (differently-sized -- see file header) tolerances for the WHOLE
    sequence, not just a final snapshot."""
    sim.command("DEV DT PORTS 1 2")

    samples = []   # (leg, true_x, true_y, true_h, pose, encpose)

    # --- Leg 1: straight ---
    sim.command("DEV DT VW 150 0 0")
    for _ in range(20):
        sim.tick_for(48)
        tlm = _snap(sim)
        true_pose = sim.true_pose()
        samples.append(("straight", true_pose, tlm))

    # --- Leg 2: turn (in place) ---
    sim.command("DEV DT VW 0 0 1.2")
    for _ in range(20):
        sim.tick_for(48)
        tlm = _snap(sim)
        true_pose = sim.true_pose()
        samples.append(("turn", true_pose, tlm))

    assert len(samples) == 40, "sanity: the full sequence was sampled"

    max_pose_xy = 0.0
    max_pose_h = 0.0
    max_enc_xy = 0.0
    max_enc_h = 0.0

    for leg, (true_x, true_y, true_h), tlm in samples:
        px, py, ph_cdeg = (int(v) for v in tlm["pose"].split(","))
        ex, ey, eh_cdeg = (int(v) for v in tlm["encpose"].split(","))
        ph = ph_cdeg / CDEG_PER_RAD
        eh = eh_cdeg / CDEG_PER_RAD

        pose_xy_err = math.hypot(px - true_x, py - true_y)
        enc_xy_err = math.hypot(ex - true_x, ey - true_y)
        pose_h_err = _wrapped_deg_diff(ph, true_h)
        enc_h_err = _wrapped_deg_diff(eh, true_h)

        assert pose_xy_err <= _POSE_XY_TOL, (
            f"[{leg}] pose= diverged from true pose by {pose_xy_err:.2f}mm "
            f"(tol {_POSE_XY_TOL}mm): pose=({px},{py}) true=({true_x:.2f},{true_y:.2f})"
        )
        assert pose_h_err <= math.degrees(_POSE_H_TOL), (
            f"[{leg}] pose= heading diverged from true heading by {pose_h_err:.2f}deg "
            f"(tol {math.degrees(_POSE_H_TOL)}deg)"
        )
        assert enc_xy_err <= _ENCPOSE_XY_TOL, (
            f"[{leg}] encpose= diverged from true pose by {enc_xy_err:.2f}mm "
            f"(tol {_ENCPOSE_XY_TOL}mm): encpose=({ex},{ey}) true=({true_x:.2f},{true_y:.2f})"
        )
        assert enc_h_err <= math.degrees(_ENCPOSE_H_TOL), (
            f"[{leg}] encpose= heading diverged from true heading by {enc_h_err:.2f}deg "
            f"(tol {math.degrees(_ENCPOSE_H_TOL)}deg)"
        )

        max_pose_xy = max(max_pose_xy, pose_xy_err)
        max_pose_h = max(max_pose_h, pose_h_err)
        max_enc_xy = max(max_enc_xy, enc_xy_err)
        max_enc_h = max(max_enc_h, enc_h_err)

    # Sanity: the sequence actually moved the robot in both legs (not a
    # trivially-passing all-zero test) -- final true pose is well away from
    # the origin in both x and heading.
    final_true_x, _final_true_y, final_true_h = samples[-1][1]
    assert final_true_x > 100.0, "sanity: the straight leg actually drove forward"
    assert abs(final_true_h) > math.radians(10.0), "sanity: the turn leg actually rotated"

    # Sanity: pose= (EKF-fused) tracks true pose measurably tighter than
    # encpose= (encoder-only) does -- the whole point of the two-band
    # tolerance design above. If this ever failed it would mean the OTOS
    # correction step stopped actually correcting (a real regression, not a
    # tolerance-tuning issue).
    assert max_pose_xy < max_enc_xy, (
        f"pose= (fused, OTOS-corrected) should track true pose tighter than "
        f"encpose= (encoder-only) does: max_pose_xy={max_pose_xy:.2f} "
        f"max_enc_xy={max_enc_xy:.2f}"
    )
