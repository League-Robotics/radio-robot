"""test_067_003_ekf_setnoise.py — noise-only EKF setNoise() path (sprint 067-003).

`SET ekfRHead=<x>` replied OK and updated RobotConfig, but nothing consumed
the live value: PhysicalStateEstimate::initEKF() -> EKFTiny::init() ran
exactly once, from Drive's constructor. Naively "fixing" this by re-invoking
initEKF()/init() from Drive::configure() on every relevant SET would be a
regression in its own right, because init()'s contract is "set noise
parameters AND reset state/covariance" (EKFTiny.cpp) -- a live re-init would
teleport the robot's in-flight fused pose back to the origin mid-mission.

This file proves both halves of the fix:

  1. test_set_ekfrhead_does_not_reset_fused_pose -- SET ekfRHead after
     driving to a non-origin pose must NOT disturb the fused pose/velocity/
     covariance (proves Drive::configure() calls the noise-only setNoise()
     path, not initEKF()/init()).
  2. test_set_ekfrhead_changes_heading_correction_strength -- SET ekfRHead
     to a smaller vs. larger value must change how strongly a subsequent
     OTOS heading disagreement is corrected (proves the noise update
     actually reaches EKFTiny's live _rOtosXy/_Q state used by
     updateHeading(), not just RobotConfig storage).

Both tests were confirmed to FAIL when Drive::configure() was temporarily
changed to call the (theoretically simpler) initEKF()/init() re-init path
instead of setNoise() -- test 1 fails outright (pose resets to the origin);
test 2 becomes vacuous evidence of a bug, not proof of the fix.
"""
import pytest

from firmware import Sim


# ---------------------------------------------------------------------------
# Test 1: SET ekfRHead does not reset the fused pose/velocity/covariance.
# ---------------------------------------------------------------------------

def test_set_ekfrhead_does_not_reset_fused_pose(sim):
    """Drive to a non-origin pose, SET ekfRHead, and read back immediately.

    The fused pose, velocity, and full EKF covariance diagonal must be
    bit-for-bit unchanged immediately after the SET (no tick elapses between
    the "before" read and the SET) -- proving Drive::configure()'s EKF-noise
    push does not touch _ekf.x[]/_ekf.P[].
    """
    sim.send_command("SET sTimeout=60000")

    # Drive an asymmetric D so the robot ends at a non-trivial, non-origin
    # pose (advances both position and heading away from the origin) --
    # mirrors test_fusion_validation.py / test_estimator_command_paths.py's
    # "asymmetric D curves the pose" pattern.
    r = sim.send_command("D 100 250 300")
    assert "OK" in r.upper(), f"D command failed: {r!r}"
    sim.tick_for(3000, step_ms=24)

    x_before, y_before, h_before = sim.get_fused_pose()
    v_before = sim.get_fused_v()
    omega_before = sim.get_fused_omega()
    p_before = [sim.get_ekf_p_diag(i) for i in range(5)]

    # Sanity: the robot actually reached a non-trivial pose (not still at
    # the origin -- otherwise a reset-to-origin regression would be invisible).
    assert abs(x_before) > 20.0 or abs(y_before) > 20.0 or abs(h_before) > 0.1, (
        f"D command did not reach a non-trivial pose: "
        f"({x_before:.2f}, {y_before:.2f}, {h_before:.4f})"
    )

    # SET ekfRHead to a value different from the firmware default (0.01).
    set_reply = sim.send_command("SET ekfRHead=0.5")
    assert "OK" in set_reply.upper(), f"SET ekfRHead failed: {set_reply!r}"

    # Confirm the registry value actually moved (sanity on the SET itself).
    get_reply = sim.send_command("GET ekfRHead")
    assert "0.5" in get_reply, f"GET ekfRHead did not reflect the SET: {get_reply!r}"

    # Immediately (no further ticks) re-read the fused belief -- must be
    # unchanged. No sim tick has run between the "before" reads and here, so
    # the values must be exactly identical if setNoise() left x[]/P[] alone.
    x_after, y_after, h_after = sim.get_fused_pose()
    v_after = sim.get_fused_v()
    omega_after = sim.get_fused_omega()
    p_after = [sim.get_ekf_p_diag(i) for i in range(5)]

    assert x_after == x_before, (
        f"fused x reset by SET ekfRHead: {x_before!r} -> {x_after!r}"
    )
    assert y_after == y_before, (
        f"fused y reset by SET ekfRHead: {y_before!r} -> {y_after!r}"
    )
    assert h_after == h_before, (
        f"fused heading reset by SET ekfRHead: {h_before!r} -> {h_after!r}"
    )
    assert v_after == v_before, (
        f"fused v reset by SET ekfRHead: {v_before!r} -> {v_after!r}"
    )
    assert omega_after == omega_before, (
        f"fused omega reset by SET ekfRHead: {omega_before!r} -> {omega_after!r}"
    )
    for i in range(5):
        assert p_after[i] == p_before[i], (
            f"EKF covariance P[{i}][{i}] reset by SET ekfRHead: "
            f"{p_before[i]!r} -> {p_after[i]!r}"
        )

    # Belt-and-suspenders: tick a little further and confirm the fused pose
    # continues from where it was rather than teleporting toward the origin.
    # (A reset x[]=0/P[]=0 would not be visible in the HardwareState-cached
    # fused.pose fields until the *next* predict() call writes EKF output
    # back into HardwareState -- this catches exactly that delayed symptom,
    # which is the concrete failure mode the ticket describes: "teleport the
    # robot's actual, in-flight fused pose back to the origin mid-mission".)
    sim.tick_for(200, step_ms=24)
    x_later, y_later, h_later = sim.get_fused_pose()
    dist_from_pre_set_pose = ((x_later - x_before) ** 2 + (y_later - y_before) ** 2) ** 0.5
    assert dist_from_pre_set_pose < 20.0, (
        f"fused pose drifted far from its pre-SET value after a few more "
        f"ticks -- looks like a delayed reset-to-origin: "
        f"pre-SET=({x_before:.2f}, {y_before:.2f}), "
        f"post-tick=({x_later:.2f}, {y_later:.2f}), "
        f"distance={dist_from_pre_set_pose:.2f} mm"
    )


# ---------------------------------------------------------------------------
# Test 2: SET ekfRHead changes the live heading-correction strength.
# ---------------------------------------------------------------------------

def _heading_correction_after_one_tick(ekf_r_head: float) -> float:
    """Fresh Sim(): SET ekfRHead, inject a fixed OTOS heading disagreement,
    enable fusion, tick exactly one control step, and return the magnitude
    the fused heading moved toward the injected OTOS heading.

    A fresh Sim() per value (rather than reusing one instance across
    measurements) mirrors this sprint's own fix to test_rt_slip.py's
    cross-measurement state-accumulation bug -- no state must carry over
    between the low-R and high-R measurements.
    """
    with Sim() as sim:
        sim.send_command("SET sTimeout=60000")
        sim.send_command(f"SET ekfRHead={ekf_r_head}")

        # Let P[2][2] (heading covariance) grow off its zero-at-boot floor
        # via ordinary predict-step process noise so the Kalman gain is
        # non-degenerate (K = P/(P+R) is 0 regardless of R when P=0).
        sim.tick_for(500, step_ms=24)

        enc_x, enc_y, enc_h = sim.get_enc_pose()
        h_before = sim.get_fused_pose()[2]

        # Inject an OTOS reading that agrees on position but disagrees on
        # heading by a fixed, deliberate 0.2 rad (~11.5 deg) -- comfortably
        # inside the chi-square 1-DOF gate (3.84) for every R tested below,
        # so every run performs a real (non-rejected) update.
        otos_h = enc_h + 0.2
        sim.set_otos_pose(enc_x, enc_y, otos_h)
        sim.set_otos_fusion(True)

        sim.tick_for(24, step_ms=24)  # exactly one control tick

        h_after = sim.get_fused_pose()[2]
        rejects = sim.get_ekf_rej_count()

    assert rejects == 0, (
        f"heading update was gate-rejected at ekfRHead={ekf_r_head} "
        f"(rej_count={rejects}) -- test setup assumption violated"
    )
    return abs(h_after - h_before)


def test_set_ekfrhead_changes_heading_correction_strength():
    """A smaller ekfRHead (less measurement noise assumed) must produce a
    markedly LARGER single-tick heading correction toward a fixed OTOS
    disagreement than a larger ekfRHead (more measurement noise assumed).

    This is the live-consumer half of the fix: it proves Drive::configure()'s
    SET-triggered setNoise() call actually reaches EKFTiny's measurement-noise
    state consumed by updateHeading(), not just RobotConfig storage.

    Does not use the `sim` fixture -- the helper creates its own fresh Sim()
    instances for isolation -- but still benefits from the session-scoped
    autouse `build_lib` fixture that ensures the library is built.
    """
    correction_low_r = _heading_correction_after_one_tick(0.001)
    correction_high_r = _heading_correction_after_one_tick(5.0)

    assert correction_low_r > 0.01, (
        f"low-R correction too small to be a meaningful measurement: "
        f"{correction_low_r:.5f} rad"
    )
    assert correction_low_r > correction_high_r * 1.5, (
        f"SET ekfRHead did not change heading-correction strength: "
        f"low-R (0.001) correction={correction_low_r:.5f} rad, "
        f"high-R (5.0) correction={correction_high_r:.5f} rad "
        f"(expected low-R correction to be markedly larger)"
    )
