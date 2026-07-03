"""
test_otos_stuck_value_gate.py — Ticket 074-003.

`Drive::_updateOtosFusionGate` (CR-06, sprint 065) already re-admits fusion
after a run of clean ticks -- proven correct and regression-tested by
`test_otos_warn_persistence.py`. Its only "is this reading healthy" input was
the OTOS chip's self-reported STATUS byte, which has no way to catch a
reading that is READABLE, reports a clean STATUS byte, and simply stops
updating. That is the field recording's exact signature: `otos=` frozen for
an entire session while `ekf_rej` climbed almost every tick -- proof the old
gate did not catch this case, since a *blocked* gate would stop feeding the
EKF and `ekf_rej` would stop climbing.

The fix (Drive.h/Drive.cpp) adds a per-tick "is the newly read pose unchanged
from the previous tick's pose AND does the robot show encoder-evidenced
motion this tick" check and ORs it into the existing `warnBit` passed to
`_updateOtosFusionGate`. This is an additional input to the same,
already-tested state machine -- not a new gate, not a rewrite. The streak
counters (`kOtosWarnPersistK`/`kOtosCleanReadmitN`) and the STATUS-bit path
are untouched (see `test_otos_warn_persistence.py`, still green, unmodified).

Tests
-----
test_stuck_value_blocks_fusion_and_readmits_after_stop
    A fixed OTOS pose is injected once (STATUS stays clean; the value never
    updates) while the robot drives. `ekf_rej` climbs during the initial
    persist-K ramp (the stuck value is still being fused and disagrees with
    the encoder prediction), then stops climbing once the block engages, and
    the fused pose tracks the encoder estimate instead of the frozen value.
    Stopping the robot drops encoder-evidenced motion to zero, which disarms
    the staleness check regardless of the still-frozen pose, so the existing
    clean-streak re-admission (kOtosCleanReadmitN) proceeds exactly as it
    does for a STATUS-bit block; injecting a fresh offset once re-admitted
    pulls the fused pose toward it (mirrors
    `test_clean_streak_readmits_fusion_after_block`'s phase-2/3 structure).

test_stationary_frozen_otos_never_flagged_stuck
    A robot that never drives (zero encoder-evidenced motion the whole test)
    with a completely static OTOS reading must NEVER be flagged stuck,
    however long the value has been static -- encMotion gates the whole
    check. Fusion must behave exactly as it would for a healthy sensor: the
    estimate converges to and stays at the injected (in-gate) reading rather
    than being blocked and left at the encoder-only dead-reckoning value.
"""


def test_stuck_value_blocks_fusion_and_readmits_after_stop(sim):
    """Stuck-but-STATUS-clean OTOS reading while moving: fusion blocks and
    ekf_rej stops climbing; stopping + a fresh value re-admits fusion."""
    sim.send_command("SET sTimeout=60000")

    sim.set_otos_fusion(True)
    # A fixed pose that never changes -- STATUS stays clean (readStatus is
    # untouched by this injection), only the VALUE is stuck. Chosen far
    # enough from the robot's actual trajectory that every fused tick before
    # the block engages disagrees with the encoder-predicted pose and is
    # rejected -- mirroring the field recording's "ekf_rej climbs almost
    # every tick" symptom.
    sim.set_otos_pose(500.0, 500.0, 0.0)

    rej_start = sim.get_ekf_rej_count()

    r = sim.send_command("VW 200 0")
    assert "OK" in r.upper(), f"VW command failed: {r!r}"

    # ---- Window A: spans the persist-K ramp-up while the stuck value is
    # still being fused (not yet blocked) -- ekf_rej must climb here. ----
    sim.tick_for(10 * 24, step_ms=24)
    rej_a = sim.get_ekf_rej_count()

    # Continue driving well past kOtosWarnPersistK (3) so the block is long
    # since engaged and stable before the second window is measured.
    sim.tick_for(40 * 24, step_ms=24)

    # ---- Window B: later, equal-length window -- block has long since
    # engaged, so ekf_rej must NOT climb (the stuck value is no longer
    # fused). ----
    rej_b0 = sim.get_ekf_rej_count()
    sim.tick_for(10 * 24, step_ms=24)
    rej_b1 = sim.get_ekf_rej_count()

    delta_a = rej_a - rej_start
    delta_b = rej_b1 - rej_b0
    assert delta_a > 0, (
        f"Expected ekf_rej to climb while the stuck value is still being "
        f"fused (before the block engages), got delta_a={delta_a}"
    )
    assert delta_b == 0, (
        f"ekf_rej kept climbing well after the block should have engaged -- "
        f"the stuck value is still being fused every tick: delta_b={delta_b} "
        f"(window rej {rej_b0}->{rej_b1}), unlike the initial climb "
        f"delta_a={delta_a}"
    )

    enc_x, _, _ = sim.get_enc_pose()
    fused_x, _, _ = sim.get_fused_pose()
    drift = abs(fused_x - enc_x)
    assert drift < 20.0, (
        f"Fused pose diverged from encoder estimate -- looks like the stuck "
        f"value is still being fused: enc_x={enc_x:.1f} mm, "
        f"fused_x={fused_x:.1f} mm (drift={drift:.1f} mm, threshold 20 mm)"
    )

    # ---- Recovery phase: stop -- this drops encoder-evidenced motion to
    # zero, which disarms the staleness check regardless of the still-frozen
    # injected pose, so _otosWarnStreak resets and _otosCleanStreak
    # accumulates every stopped tick (mirrors
    # test_clean_streak_readmits_fusion_after_block's exact phase-2/3
    # structure). ----
    sim.send_command("X")
    sim.tick_for(48, step_ms=24)

    enc_x_before, _, _ = sim.get_enc_pose()
    otos_target_x = enc_x_before + 200.0
    sim.set_otos_pose(otos_target_x, 0.0, 0.0)

    # kOtosCleanReadmitN (5) clean ticks re-admit fusion, then the existing
    # gate-recovery mechanism's usual budget to snap toward the injected
    # offset (same 30-tick budget test_clean_streak_readmits_fusion_after_block
    # uses).
    sim.tick_for(30 * 24, step_ms=24)

    fused_x_after, _, _ = sim.get_fused_pose()
    fused_pull = fused_x_after - enc_x_before
    assert fused_pull > 50.0, (
        f"Stuck-value block did not re-admit OTOS fusion once encoder-"
        f"evidenced motion stopped: enc_x_before={enc_x_before:.1f} mm, "
        f"fused_x_after={fused_x_after:.1f} mm (pull={fused_pull:.1f} mm, "
        f"expected > 50 mm)"
    )


def test_stationary_frozen_otos_never_flagged_stuck(sim):
    """A stationary robot with a frozen OTOS reading is never flagged stuck,
    however long the value has been static -- encMotion gates the check."""
    sim.send_command("SET sTimeout=60000")

    sim.set_otos_fusion(True)
    # Robot never drives -- zero encoder-evidenced motion for the whole
    # test. A small, in-gate static offset from the true (0,0,0) rest pose
    # lets the test positively observe continuous fusion (the estimate
    # should converge to and STAY at the injected value) rather than merely
    # inferring "no drift", which would look identical whether fusion is
    # blocked or not for a value that never differs from true.
    sim.set_otos_pose(20.0, 0.0, 0.0)

    # Hundreds of ticks -- far longer than kOtosWarnPersistK (3) or
    # kOtosCleanReadmitN (5) -- with the SAME frozen value the entire time
    # and zero encoder motion throughout.
    sim.tick_for(200 * 24, step_ms=24)

    fused_x, _, _ = sim.get_fused_pose()
    # If the stuck-value gate incorrectly fired here, fusion would be
    # blocked and the estimate would sit at the encoder-only dead-reckoning
    # value (~0 mm at rest) instead of the injected 20 mm offset.
    assert abs(fused_x - 20.0) < 5.0, (
        f"Fused pose did not converge to the injected static OTOS reading "
        f"-- the stationary robot appears to have been incorrectly flagged "
        f"'stuck' and fusion was blocked: fused_x={fused_x:.2f} mm "
        f"(expected ~20.0 mm)"
    )

    # A healthy sensor holding a converged, matching (in-gate) reading
    # produces no further rejections -- behaving exactly as a healthy sensor
    # would, per the acceptance criteria.
    rej_before = sim.get_ekf_rej_count()
    sim.tick_for(50 * 24, step_ms=24)
    rej_after = sim.get_ekf_rej_count()
    assert rej_after == rej_before, (
        f"ekf_rej grew ({rej_before}->{rej_after}) after convergence with a "
        f"static, unmoving robot -- not the behaviour of a healthy, "
        f"continuously-fused sensor"
    )
