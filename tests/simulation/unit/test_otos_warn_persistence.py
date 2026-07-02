"""
test_otos_warn_persistence.py — Ticket 065-006 (CR-06).

`Robot::otosCorrect()` documented a two-tier D9 gate (READABLE vs HEALTHY)
that a 2026-06-17 change collapsed to `healthy = poseOk`, so a persistently
degraded-but-readable OTOS reading (lifted robot, robot on the stand, freshly
placed robot -- all of which hold `warnOpticalTracking`) got fused every
tick. `EKFTiny`'s own gate-recovery force-snaps the fused pose to that frozen
reading after 10 consecutive Mahalanobis-gate rejections, reopening the
"spin on placement" failure the original gate existed to prevent.

The fix restores the persistence distinction upstream of `EKFTiny` (itself
untouched): a WARNING-bit streak is fused through for <= kOtosWarnPersistK
(3) ticks (transient), blocked once it persists past that, and re-admitted
after kOtosCleanReadmitN (5) consecutive clean ticks. The live fusion path
exercised by these tests is `Drive::tickUpdate`'s STEP 5 (via
`loopTickOnce`) -- the sole OTOS->EKF path since the 060 ordered-tick
cutover; `Robot::otosCorrect()` carries the identical gate for API parity
but has no caller in the live loop.

Tests
-----
test_persistent_warn_blocks_fusion_no_snap
    Warn bit set before driving starts and held for the whole run (well past
    both kOtosWarnPersistK and EKFTiny's own 10-consecutive-rejection
    gate-recovery threshold): fused pose must track encoder-derived
    odometry, not snap toward the frozen (near-zero) OTOS pose.

test_transient_warn_blip_does_not_interrupt_fusion
    A 1-2 tick warn blip (<= kOtosWarnPersistK) followed by a return to
    clean readings must not block fusion -- the existing OTOS-injection
    gate-recovery snap (proven by test_fusion_validation.py's baseline
    scenario) must still occur on schedule.

test_clean_streak_readmits_fusion_after_block
    After a persistent-warn block engages, kOtosCleanReadmitN consecutive
    clean ticks must re-admit fusion -- verified by injecting a large OTOS
    offset only once the warn bit clears and confirming the fused pose is
    subsequently pulled toward it.
"""


def test_persistent_warn_blocks_fusion_no_snap(sim):
    """Persistent WARNING bit: fused pose tracks encoders, no snap to OTOS."""
    sim.send_command("SET sTimeout=60000")

    # Mark the SimOdometer initialised/fusing and set the WARNING bit BEFORE
    # any driving starts -- models "robot lifted/on the stand", the exact
    # scenario the issue describes as the first thing a stakeholder HITL
    # -tests against this gate.
    sim.set_otos_fusion(True)
    sim.set_otos_warn(True)

    r = sim.send_command("VW 200 0")
    assert "OK" in r.upper(), f"VW command failed: {r!r}"

    # 40 ticks x 24 ms = 960 ms -- well past kOtosWarnPersistK (3 ticks) and
    # past EKFTiny's own 10-consecutive-rejection gate-recovery threshold
    # (the force-snap this gate exists to prevent).
    sim.tick_for(40 * 24, step_ms=24)

    enc_x, enc_y, enc_h = sim.get_enc_pose()
    fused_x, fused_y, fused_h = sim.get_fused_pose()

    assert enc_x > 100.0, (
        f"Expected the robot to have driven forward, got enc_x={enc_x:.1f} mm"
    )

    # Fused pose must track the encoder estimate closely -- no snap toward
    # the frozen (near-origin) OTOS pose.
    drift = abs(fused_x - enc_x)
    assert drift < 20.0, (
        f"Fused pose diverged from encoder estimate -- looks like a snap to "
        f"the frozen OTOS pose: enc_x={enc_x:.1f} mm, fused_x={fused_x:.1f} mm "
        f"(drift={drift:.1f} mm, threshold 20 mm)"
    )


def test_transient_warn_blip_does_not_interrupt_fusion(sim):
    """A 1-2 tick warn blip must not block the normal OTOS fusion path."""
    sim.send_command("SET sTimeout=60000")

    # Same setup as test_fusion_validation.py's baseline snap scenario:
    # drive straight, stop, inject a large persistent OTOS offset.
    r = sim.send_command("VW 200 0")
    assert "OK" in r.upper(), f"VW command failed: {r!r}"
    sim.tick_for(2000, step_ms=24)

    sim.send_command("X")
    sim.tick_for(48, step_ms=24)

    enc_x_before, _, _ = sim.get_enc_pose()
    assert enc_x_before > 100.0, (
        f"Expected encoder x > 100 mm after 2 s VW drive, got {enc_x_before:.1f} mm"
    )

    otos_target_x = enc_x_before + 200.0
    sim.set_otos_pose(otos_target_x, 0.0, 0.0)
    sim.set_otos_fusion(True)

    # Transient warn blip -- 2 ticks, within kOtosWarnPersistK (3) -- must be
    # fused through, not blocked.
    sim.set_otos_warn(True)
    sim.tick_for(2 * 24, step_ms=24)
    sim.set_otos_warn(False)

    # Same 25-tick budget test_fusion_validation.py uses for the gate-
    # recovery snap to complete; the blip must not have delayed or blocked
    # it.
    sim.tick_for(25 * 24, step_ms=24)

    fused_x_after, _, _ = sim.get_fused_pose()
    fused_pull = fused_x_after - enc_x_before
    assert fused_pull > 50.0, (
        f"Transient warn blip incorrectly blocked/delayed OTOS fusion: "
        f"enc_x_before={enc_x_before:.1f} mm, fused_x_after={fused_x_after:.1f} mm "
        f"(pull={fused_pull:.1f} mm, expected > 50 mm)"
    )


def test_clean_streak_readmits_fusion_after_block(sim):
    """kOtosCleanReadmitN consecutive clean ticks re-admit fusion after a block."""
    sim.send_command("SET sTimeout=60000")

    # ---- Phase 1: engage a persistent-warn block while driving ----
    sim.set_otos_fusion(True)
    sim.set_otos_warn(True)
    r = sim.send_command("VW 200 0")
    assert "OK" in r.upper(), f"VW command failed: {r!r}"

    # 10 ticks is well past kOtosWarnPersistK (3) -- fusion should now be
    # blocked.
    sim.tick_for(10 * 24, step_ms=24)

    enc_x_mid, _, _ = sim.get_enc_pose()
    fused_x_mid, _, _ = sim.get_fused_pose()
    assert abs(fused_x_mid - enc_x_mid) < 20.0, (
        f"Fusion should be blocked while the warn bit persists (fused pose "
        f"should track encoders): enc_x_mid={enc_x_mid:.1f} mm, "
        f"fused_x_mid={fused_x_mid:.1f} mm"
    )

    # ---- Phase 2: stop, inject a large OTOS offset, clear the warn bit ----
    sim.send_command("X")
    sim.tick_for(48, step_ms=24)

    enc_x_before, _, _ = sim.get_enc_pose()
    otos_target_x = enc_x_before + 200.0
    sim.set_otos_pose(otos_target_x, 0.0, 0.0)
    sim.set_otos_warn(False)

    # ---- Phase 3: kOtosCleanReadmitN (5) clean ticks re-admit fusion, then
    # give the existing gate-recovery mechanism its usual budget to snap
    # toward the injected offset. ----
    sim.tick_for(30 * 24, step_ms=24)

    fused_x_after, _, _ = sim.get_fused_pose()
    fused_pull = fused_x_after - enc_x_before
    assert fused_pull > 50.0, (
        f"Clean streak did not re-admit OTOS fusion after the warn block: "
        f"enc_x_before={enc_x_before:.1f} mm, fused_x_after={fused_x_after:.1f} mm "
        f"(pull={fused_pull:.1f} mm, expected > 50 mm)"
    )
