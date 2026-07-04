"""
test_068_004_zero_error_three_pose_agreement.py — sprint 068's closing
regression (ticket 068-004).

With every sim error-injection knob zeroed, there is nothing left for the
three pose estimators (encoder-only, raw OTOS, EKF-fused) to disagree
about. This test drives a TOUR_1-shaped multi-leg maneuver (turns and
straight drives) and asserts that ALL FOUR of:

  - frame.encpose  (068-001: firmware encoder-only dead-reckoned pose)
  - frame.otos     (raw OTOS pose)
  - frame.pose     (EKF-fused pose)
  - sim.get_true_pose()  (plant ground truth)

agree within a tight numeric tolerance, at every collected TLM frame,
throughout the whole maneuver.

Its purpose is to catch a future regression in any ONE of the three pose
estimators, or in the TLM wire encoding of any of them, specifically and by
name -- not only when injected noise happens to be nonzero (see
clasi/sprints/068-three-world-poses-in-tlm-firmware-encpose/architecture-update.md,
SUC-003).

Zeroing "all sim error-injection knobs" requires two things, not one:

  1. The sim's plant-level error knobs (``Sim.set_field_profile``'s
     ``slip_turn_extra``, and SimOdometer's noise/drift/scale-error
     setters). ``slip_turn_extra=0.0`` disables the plant's simulated wheel
     scrub during turns. SimOdometer's own noise/drift/scale-error knobs
     all default to zero on a fresh ``Sim()`` (see
     ``source/hal/sim/SimOdometer.h``: "Every error setter defaults to a
     no-op, so a fresh SimOdometer is PERFECT") -- ``fuse_otos=True`` only
     enables the (already-perfect) OTOS sim model and EKF fusion, it does
     not introduce any error.
  2. ``RobotConfig::rotationalSlip`` (default 0.92, set via ``SET rotSlip``)
     is a *calibration constant*, not a sim/plant error-injection knob --
     but it feeds directly into ``Odometry::predict()``'s turn-arc
     correction (``dTheta = ((dR-dL)/trackwidth) * effectiveSlip(rotSlip)``,
     see source/control/Odometry.h). It exists to compensate for the REAL
     wheel scrub that ``slip_turn_extra`` models. With ``slip_turn_extra``
     zeroed, there is no scrub to correct for -- so a nonzero
     ``rotationalSlip`` would apply an unwarranted correction, making
     ``encpose=`` (and, via ``Planner::beginRotation()``, the commanded
     turn arc itself) diverge from truth/otos/pose by tens of mm and tens
     of degrees over a multi-turn tour. This was confirmed empirically
     while writing this test: leaving ``rotationalSlip`` at its 0.92
     default produced a 262 mm / 37 degree max disagreement for
     ``encpose=`` alone, while ``otos=``/``pose=`` (which read plant truth
     directly, not through a slip-corrected encoder-arc model) stayed
     within ~1.4 mm / ~0.01 degree of truth throughout. ``SET rotSlip=0``
     is this project's established sentinel for "no correction"
     (``effectiveSlip()`` maps 0 -> 1.0; see
     ``tests/simulation/unit/test_rt_slip.py``). Zeroing it here is the
     "equivalent zeroing" the ticket calls for beyond the sim's own
     error-injection module.
  3. (073-002) ``SimHandle``'s constructor now seeds the PLANT's own
     ``bodyRotScrub`` from the baked-in ``RobotConfig.rotationalSlip``
     (0.92 by default) so a fresh, zero-configuration ``Sim()`` genuinely
     scrubs rotation -- independently of ``SET rotSlip`` above, which only
     affects the FIRMWARE's encoder-arc correction, not the plant's true
     rotation. Sub-step A (encoder accumulation) is never scrubbed, so an
     un-reset construction-time plant scrub makes ``encpose=`` (which
     follows the un-scrubbed wheel arc) diverge from true/otos/pose (which
     read the now-genuinely-scrubbed plant) on any turn -- ``SIMSET
     bodyRotScrub=1.0`` resets the plant side of "zero error" exactly as
     ``SET rotSlip=0`` resets the firmware side (Design Rationale Decision
     3's own documented consequence: "the seed is a default, not a lock").
"""
from __future__ import annotations

import math

import pytest

from robot_radio.robot.protocol import TLMFrame, parse_tlm
from robot_radio.testgui.commands import TOUR_1

RAD_TO_CDEG = 18000.0 / math.pi

# Tight tolerances -- with every error-injection knob at zero, all four pose
# sources track the same underlying motion and should nearly coincide. The
# residual gap observed in practice is ~1.4 mm / ~1 centidegree, coming from
# the wire encoding's integer truncation (int mm, int centidegrees) and
# float32 accumulation -- these tolerances leave several times that margin
# without being loose enough to hide a real regression.
XY_TOL_MM = 5.0
H_TOL_CDEG = 5.0

STEP_MS = 24          # tick granularity, matches other sim system tests
LEG_BUDGET_MS = 8000  # generous per-leg ceiling; longest observed leg ~4.4 s

# Wire pose fields to check against plant truth on every frame.
_POSE_FIELDS = ("encpose", "otos", "pose")


def _wrap_cdeg(delta: float) -> float:
    """Wrap a centidegree difference into (-18000, 18000]."""
    return ((delta + 18000.0) % 36000.0) - 18000.0


def _configure_zero_error(s) -> None:
    """Zero every sim error-injection knob (plant slip + calibration slip).

    See the module docstring for why `slip_turn_extra` (plant/sim), `rotSlip`
    (firmware calibration constant, `SET rotSlip=0`), AND (073-002)
    `bodyRotScrub` (the plant's own construction-time-seeded scrub) must all
    be zeroed/neutralized for the three pose estimators to actually agree
    with zero error.
    """
    # slip_turn_extra=0.0: no plant-level encoder over-report on turns.
    # fuse_otos=True: enable the (already-perfect, noise/drift/scale all
    # default to zero) OTOS sim model + EKF fusion so otos=/pose= populate.
    s.set_field_profile(slip_turn_extra=0.0, fuse_otos=True)

    reply = s.send_command("SET rotSlip=0")
    assert "OK" in reply.upper(), f"SET rotSlip=0 rejected: {reply!r}"

    # 073-002: SimHandle's constructor seeds the plant's bodyRotScrub from
    # RobotConfig.rotationalSlip (0.92 by default) -- reset it to neutral
    # (1.0) so the plant's TRUE rotation is genuinely un-scrubbed, matching
    # encpose= (sub-step A, never scrubbed) exactly. `SET rotSlip=0` above
    # only zeroes the FIRMWARE's encoder-arc correction; it does not touch
    # this independent plant-truth knob (see architecture-update.md Decision
    # 3: "the seed is a default, not a lock").
    reply = s.send_command("SIMSET bodyRotScrub=1.0")
    assert "OK" in reply.upper(), f"SIMSET bodyRotScrub=1.0 rejected: {reply!r}"


def _drive_tour_collecting_frames(s, tour: list[str]) -> list[tuple[TLMFrame, tuple[float, float, float]]]:
    """Drive each leg of `tour` in sequence, returning (frame, true_pose) pairs.

    For each collected TLM frame, the plant's true pose is sampled
    immediately after the same simulation tick that produced it, so
    `sim.get_true_pose()` really is "at the same tick" as the frame (per
    the ticket's acceptance criteria) rather than sampled once at the end
    of a multi-tick window.

    Each leg is given a fixed `LEG_BUDGET_MS` ceiling and is confirmed to
    have returned to idle (`frame.mode == "I"`) within that budget -- this
    is a deterministic sim maneuver, so a fixed generous ceiling is not
    flaky, and asserting idle-by-budget catches a leg that stalls (e.g. a
    rejected/malformed command) rather than silently under-sampling it.
    """
    samples: list[tuple[TLMFrame, tuple[float, float, float]]] = []
    for cmd in tour:
        reply = s.send_command(cmd)
        assert "OK" in reply.upper(), f"command {cmd!r} was not accepted: {reply!r}"

        elapsed = 0
        idle_seen = False
        while elapsed < LEG_BUDGET_MS:
            frames = s.tick_collect_tlm(total_ms=STEP_MS, step_ms=STEP_MS)
            elapsed += STEP_MS
            if not frames:
                continue
            true_pose = s.get_true_pose()
            for line in frames:
                frame = parse_tlm(line)
                if frame is None:
                    continue
                samples.append((frame, true_pose))
                if frame.mode == "I":
                    idle_seen = True

        assert idle_seen, (
            f"command {cmd!r} did not return to idle (mode='I') within "
            f"{LEG_BUDGET_MS} ms"
        )

    return samples


def test_zero_error_three_pose_and_truth_agreement(sim):
    """encpose=/otos=/pose= and plant truth agree within a tight tolerance
    at every TLM frame collected over a TOUR_1-shaped multi-leg maneuver,
    when every sim error-injection knob is zeroed."""
    s = sim
    _configure_zero_error(s)

    reply = s.send_command("STREAM 100")
    assert "period=100" in reply, f"STREAM 100 rejected: {reply!r}"

    samples = _drive_tour_collecting_frames(s, TOUR_1)

    # Sanity: the tour is 10 legs (RT/D/TURN); expect a healthy number of
    # collected frames, not a degenerate near-empty run.
    assert len(samples) > 100, (
        f"expected many TLM frames across a {len(TOUR_1)}-leg tour, got "
        f"{len(samples)}"
    )

    # The maneuver must have actually moved the robot -- otherwise the
    # agreement assertion below would be trivially true at the origin.
    # TOUR_1 is a CLOSING tour since 2026-07-03 (returns to its start), so
    # assert on cumulative true PATH LENGTH, not final displacement.
    path_mm = 0.0
    prev_xy = None
    for _frame, (tx, ty, _th) in samples:
        if prev_xy is not None:
            path_mm += math.hypot(tx - prev_xy[0], ty - prev_xy[1])
        prev_xy = (tx, ty)
    assert path_mm > 1000.0, (
        f"plant barely moved over the tour: cumulative true path "
        f"{path_mm:.0f} mm"
    )

    for frame, (true_x, true_y, true_h_rad) in samples:
        true_h_cdeg = true_h_rad * RAD_TO_CDEG
        for field_name in _POSE_FIELDS:
            wire_pose = getattr(frame, field_name)
            assert wire_pose is not None, (
                f"frame.{field_name} missing (t={frame.t}, mode={frame.mode}) "
                "-- all three wire poses must be present on every frame "
                "once STREAM is bound and OTOS is enabled"
            )
            wx, wy, wh = wire_pose
            assert wx == pytest.approx(true_x, abs=XY_TOL_MM), (
                f"frame.{field_name}.x={wx} vs true_x={true_x:.1f} "
                f"(t={frame.t}, mode={frame.mode})"
            )
            assert wy == pytest.approx(true_y, abs=XY_TOL_MM), (
                f"frame.{field_name}.y={wy} vs true_y={true_y:.1f} "
                f"(t={frame.t}, mode={frame.mode})"
            )
            dh = _wrap_cdeg(wh - true_h_cdeg)
            assert abs(dh) <= H_TOL_CDEG, (
                f"frame.{field_name}.h={wh} vs true_h_cdeg={true_h_cdeg:.1f} "
                f"(dh={dh:.1f}, t={frame.t}, mode={frame.mode})"
            )
