"""
test_fusion_validation.py — Sprint 047-005 fusion-validation tests.

Proves that the three estimates (encoder, optical, fused) maintained by Sprint 047
are genuinely independent and that EKF fusion never corrupts the encoder path.

Tests
-----
test_encoder_not_overwritten_by_fusion
    Drives straight to accumulate ~200 mm of encoder travel, then injects an OTOS
    pose offset of +200 mm (total ~400 mm) with fusion enabled.  After 25 sim ticks
    the EKF P-inflation re-baseline fires (after 10 consecutive gate rejections) and
    the fused estimate snaps toward the injected OTOS value.  The encoder estimate
    must not have moved toward the injection.

    Threshold choice:
    - enc tolerance ±5 mm: robot is stopped; any encoder motion > 5 mm would indicate
      the enc path was corrupted by the EKF correction write path.
    - fused pulled > +50 mm from enc_before: conservative (the actual snap is ~200 mm
      after P-inflation); ensures the fused path genuinely changed.
    - optical within ±10 mm of injected target: the SimOdometer persistent injection
      is reflected immediately in the optical read captured before the EKF step.

test_est_dump_emits_three_lines
    Sends "DBG EST" and verifies that the reply contains all three EST lines
    (enc, otos, fuse) with the expected fields: x=, y=, h=, vx=, vy=, w=, age=, v=.
"""
import pytest


# ---------------------------------------------------------------------------
# Test 1: Encoder path is NOT corrupted by OTOS-EKF fusion
# ---------------------------------------------------------------------------

def test_encoder_not_overwritten_by_fusion(sim):
    """Encoder estimate must not be pulled toward an OTOS-injected offset.

    Scenario:
      1. Drive straight at 200 mm/s for 2 s — encoder accumulates ~200 mm in X.
      2. Stop the robot.
      3. Inject a persistent OTOS pose at enc_x_before + 200 mm (a large offset that
         is well outside the Mahalanobis gate).
      4. Enable OTOS→EKF fusion and tick for 25 steps (25 × 24 ms = 600 ms).
         After 10 consecutive gate rejections the EKF performs P-inflation and the
         fused estimate snaps to the injected OTOS value on the 11th tick.
      5. Assert the three diverge as required.

    Thresholds:
      - enc unchanged: ±5 mm.  Robot is stopped; any drift > 5 mm implies the
        encoder accumulator was written by the fusion path (a regression).
      - fused pulled: fused_x > enc_x_before + 50 mm.  The actual snap is ~200 mm;
        50 mm is conservative and sufficient to prove the fused path changed.
      - optical reflects injection: within ±10 mm of enc_x_before + 200 mm.
    """
    # Extend watchdog (fixture already sets 60000 but be explicit).
    sim.send_command("SET sTimeout=60000")

    # ---- Phase 1: accumulate encoder travel ----
    # Drive 200 mm/s straight for 2000 ms → ~200 mm forward.
    r = sim.send_command("VW 200 0")
    assert "OK" in r.upper(), f"VW command failed: {r!r}"
    sim.tick_for(2000, step_ms=24)

    # Stop the robot so encoder does not keep advancing during the injection phase.
    sim.send_command("X")
    sim.tick_for(48, step_ms=24)  # two settle ticks

    enc_x_before, enc_y_before, enc_h_before = sim.get_enc_pose()

    # Sanity: the robot actually drove.
    assert enc_x_before > 100.0, (
        f"Expected encoder x > 100 mm after 2 s VW drive, got {enc_x_before:.1f} mm"
    )

    # ---- Phase 2: inject OTOS offset and enable fusion ----
    # Place OTOS at enc_x_before + 200 mm — well outside the 50 mm Mahalanobis gate.
    # The SimOdometer holds this as a persistent injection (returned every tick).
    otos_target_x = enc_x_before + 200.0
    sim.set_otos_pose(otos_target_x, 0.0, 0.0)

    # Enable OTOS→EKF fusion.  This also marks the SimOdometer as initialized so
    # Robot::otosCorrect() does not early-return.
    sim.set_otos_fusion(True)

    # ---- Phase 3: tick for 25 steps (10 rejections → P-inflation → snap) ----
    # After 10 consecutive position gate rejections the EKF inflates P to 1e6 mm²
    # (kRebaselineP) so the Kalman gain K ≈ 1 and the fused estimate snaps to the
    # injected OTOS value on the 11th tick.  25 ticks gives ample margin.
    sim.tick_for(25 * 24, step_ms=24)

    enc_x_after, enc_y_after, enc_h_after = sim.get_enc_pose()
    fused_x_after, fused_y_after, fused_h_after = sim.get_fused_pose()
    optical_x_after, optical_y_after, optical_h_after = sim.get_optical_pose()

    # ---- Assertions ----

    # Encoder path must NOT have moved toward the OTOS offset.
    enc_drift = abs(enc_x_after - enc_x_before)
    assert enc_drift < 5.0, (
        f"Encoder pose was corrupted by fusion: "
        f"enc_x_before={enc_x_before:.1f} mm, enc_x_after={enc_x_after:.1f} mm, "
        f"drift={enc_drift:.2f} mm (threshold 5 mm)"
    )

    # Fused pose MUST have been pulled significantly toward the OTOS offset.
    fused_pull = fused_x_after - enc_x_before
    assert fused_pull > 50.0, (
        f"Fused pose was not pulled toward the OTOS injection: "
        f"enc_x_before={enc_x_before:.1f} mm, fused_x_after={fused_x_after:.1f} mm, "
        f"pull={fused_pull:.1f} mm (threshold 50 mm)"
    )

    # Optical (raw OTOS snapshot) must reflect the injected value.
    optical_error = abs(optical_x_after - otos_target_x)
    assert optical_error < 10.0, (
        f"Optical pose does not reflect the injection: "
        f"expected ~{otos_target_x:.1f} mm, got {optical_x_after:.1f} mm "
        f"(error={optical_error:.2f} mm, threshold 10 mm)"
    )

    # All three estimates must be distinct (the fundamental property of Sprint 047).
    assert enc_x_after != fused_x_after, (
        f"Encoder and fused must differ after OTOS injection "
        f"(both = {enc_x_after:.1f} mm)"
    )
    assert enc_x_after != optical_x_after, (
        f"Encoder and optical must differ after OTOS injection "
        f"(enc={enc_x_after:.1f} mm, optical={optical_x_after:.1f} mm)"
    )


# ---------------------------------------------------------------------------
# Test 2: DBG EST command emits three EST lines with all required fields
# ---------------------------------------------------------------------------

def test_est_dump_emits_three_lines(sim):
    """DBG EST must emit three EST lines covering enc, otos, fuse.

    The reply from "DBG EST" must contain:
      EST enc   x=<n> y=<n> h=<n> vx=<n> vy=<n> w=<n> age=<n> v=<n>
      EST otos  x=<n> y=<n> h=<n> vx=<n> vy=<n> w=<n> age=<n> v=<n>
      EST fuse  x=<n> y=<n> h=<n> vx=<n> vy=<n> w=<n> age=<n> v=<n>

    All field keys (x=, y=, h=, vx=, vy=, w=, age=, v=) must appear on each line.
    """
    # Tick briefly so the firmware has a non-zero clock and each estimate
    # has been populated at least once before the dump.
    sim.tick_for(24, step_ms=24)

    reply = sim.send_command("DBG EST")

    # Each of the three source labels must appear.
    assert "EST enc" in reply, f"Missing EST enc line in reply: {reply!r}"
    assert "EST otos" in reply, f"Missing EST otos line in reply: {reply!r}"
    assert "EST fuse" in reply, f"Missing EST fuse line in reply: {reply!r}"

    # Each line must contain all required fields.
    required_fields = ("x=", "y=", "h=", "vx=", "vy=", "w=", "age=", "v=")
    for label in ("enc", "otos", "fuse"):
        matching = [ln for ln in reply.splitlines() if f"EST {label}" in ln]
        assert matching, f"Could not find any line with 'EST {label}' in: {reply!r}"
        line = matching[0]
        for field in required_fields:
            assert field in line, (
                f"EST {label} line is missing field '{field}': {line!r}"
            )


# ---------------------------------------------------------------------------
# Test 3: DBG EST reply text is byte-identical to the pre-refactor format
#
# 070-002 converted EstimateDump::source from a raw const char* to an
# EstimateSource enum (rendered via a single toString() at the emit point).
# This test pins the exact wire text — including the "EST %-4s" padding
# (two spaces after "enc", one after "otos"/"fuse") — so a future change to
# either the enum's toString() mapping or the snprintf format string cannot
# silently alter DBG EST's byte layout.
# ---------------------------------------------------------------------------

def test_est_dump_byte_identical_format(sim):
    """DBG EST's exact reply text on a freshly-reset robot must match the
    pre-refactor (raw const char* source) byte layout exactly."""
    sim.tick_for(24, step_ms=24)

    reply = sim.send_command("DBG EST")

    expected = (
        "EST enc  x=0 y=0 h=0 vx=0 vy=0 w=0 age=9999999 v=0\n"
        "EST otos x=0 y=0 h=0 vx=0 vy=0 w=0 age=9999999 v=0\n"
        "EST fuse x=0 y=0 h=0 vx=0 vy=0 w=0 age=9999999 v=0\n"
        "OK dbg est\n"
    )
    assert reply == expected, f"DBG EST reply text changed: {reply!r}"
