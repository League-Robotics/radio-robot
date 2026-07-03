"""test_070_003_physicalstateestimate_dethreading.py — regression tests for
ticket 070-003: PhysicalStateEstimate/Odometry de-threading (remove
HardwareState parameter threading).

Background (see clasi/sprints/070-fixme-cleanup-omnibus-.../
architecture-update.md Step 5 "PhysicalStateEstimate de-threading" and
Design Rationale Decision 3):

  `PhysicalStateEstimate`/`Odometry` used to take a `HardwareState&` on
  nearly every method even though each method only reads or writes a small,
  specific sub-piece of that struct. This ticket narrows every method to
  take exactly the inputs it reads (encoder readings, OTOS readings) and
  exactly the `PoseEstimate&` output(s) it writes. Trackwidth/rotational
  slip (previously passed as explicit `predict()` parameters) move to a new
  `setKinematics()` setter, called every tick from `Drive::tickUpdate()` --
  preserving sprint 067's live-`SET`-reaches-the-estimator guarantee exactly
  (Drive still reads `_robCfg` live; it just calls `setKinematics()`
  immediately before `addOdometryObservation()` instead of passing the two
  floats as call parameters).

This file covers the three ticket-mandated new-test areas:

  1. test_set_tw_and_rotslip_together_reach_predict_next_tick -- proves a
     SINGLE `setKinematics(trackwidthMm, rotationalSlip)` call correctly
     feeds BOTH values into `Odometry::predict()`'s next tick (mirrors
     sprint 067's own methodology: fresh Sim(), inject an encoder
     differential directly, tick once, read the resulting heading).
  2. test_set_tw_and_rotslip_do_not_reset_fused_pose -- proves the NEW
     `setKinematics()` method only writes `_trackwidthMm`/`_rotationalSlip`,
     not `_ekf.x[]`/`_ekf.P[]` (mirrors sprint 067-003's
     test_set_ekfrhead_does_not_reset_fused_pose, applied to the two
     kinematics keys this ticket's setKinematics() now carries).
  3. test_getpose_reads_ground_truth_fused_pose -- proves the narrowed
     `getPose(const PoseEstimate&, ...)` (used by
     `Planner::getPoseFloat()`) returns the SAME position an independent
     oracle (`sim.get_fused_pose()`, which reads `Drive::_hw.fused` through
     a wholly separate C API, not through `PhysicalStateEstimate::getPose`
     at all) reports -- a goto-to-world-origin command computed from that
     independent reading must actually converge on world (0, 0).
"""
import math

import pytest

from firmware import Sim


# ---------------------------------------------------------------------------
# 1. setKinematics() feeds BOTH trackwidth and rotationalSlip together.
# ---------------------------------------------------------------------------

def _ekf_predict_heading(tw_value: float, rotslip_value: float,
                          enc_r_mm: float = 200.0) -> float:
    """Isolate Drive::tickUpdate() STEP 4 (EKF predict): inject an encoder
    differential directly into the plant, tick exactly once, and return the
    resulting fused heading (radians).

    dTheta = ((dR - dL) / trackwidthMm) * effectiveSlip(rotationalSlip)

    Setting both tw and rotSlip in the same measurement (rather than pinning
    one, as test_067_002 does) proves Drive's single
    `_est.setKinematics(trackwidth, rotSlip)` call carries both live values
    through to the same predict() call -- not just one of the two.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        r = s.send_command(f"SET tw={tw_value}")
        assert "OK" in r.upper(), f"SET tw={tw_value} -> {r!r}"
        r = s.send_command(f"SET rotSlip={rotslip_value}")
        assert "OK" in r.upper(), f"SET rotSlip={rotslip_value} -> {r!r}"

        s._lib.sim_set_enc_l(s._h, 0.0)
        s._lib.sim_set_enc_r(s._h, enc_r_mm)
        s.tick_for(24, step_ms=24)
        return float(s._lib.sim_get_pose_h(s._h))


def test_set_tw_and_rotslip_together_reach_predict_next_tick():
    """A single SET tw=<x> + SET rotSlip=<y> pair must change
    Odometry::predict()'s next-tick heading by the combined analytic factor,
    proving setKinematics() carries both live values through together.

    Case A: tw=64,  rotSlip=1.0 (effectiveSlip -> 1.0) -> dTheta ~ (dR-dL)/64
    Case B: tw=128, rotSlip=0.5 (effectiveSlip -> 0.5) -> dTheta ~ (dR-dL)/256

    dTheta_B / dTheta_A = 64/256 = 0.25 -- a combined-parameter ratio that
    could only be produced if BOTH SET values reached the same predict()
    call (either alone reaching but not the other would produce 0.5 or 2.0,
    not 0.25).
    """
    h_a = _ekf_predict_heading(tw_value=64, rotslip_value=1.0)
    h_b = _ekf_predict_heading(tw_value=128, rotslip_value=0.5)

    assert abs(h_a) > 0.05, f"baseline heading too small to be meaningful: {h_a!r}"
    ratio = h_b / h_a
    assert ratio == pytest.approx(0.25, rel=0.1), (
        f"combined tw+rotSlip heading ratio should be ~0.25 "
        f"(64/256 trackwidth ratio * 1.0/0.5... -> 0.25 combined): "
        f"h_a(tw=64,slip=1.0)={h_a:.5f}, h_b(tw=128,slip=0.5)={h_b:.5f}, "
        f"ratio={ratio:.4f}"
    )


# ---------------------------------------------------------------------------
# 2. setKinematics() does not reset the fused pose/velocity/covariance.
# ---------------------------------------------------------------------------

def test_set_tw_and_rotslip_do_not_reset_fused_pose(sim):
    """Drive to a non-origin pose, SET tw= and SET rotSlip=, and read back
    immediately -- the fused pose, velocity, and full EKF covariance
    diagonal must be bit-for-bit unchanged (no tick elapses between the
    "before" reads and the SETs), proving Odometry::setKinematics() writes
    only _trackwidthMm/_rotationalSlip and never touches _ekf.x[]/_ekf.P[].

    Mirrors test_067_003_ekf_setnoise.py's
    test_set_ekfrhead_does_not_reset_fused_pose, applied to the two
    kinematics keys this ticket's setKinematics() now carries (previously
    passed as addOdometryObservation()/predict() call parameters, which
    could never have "reset" anything since they were plain floats with no
    setter of their own -- this is a genuinely new regression surface
    introduced by adding setKinematics() as a mutator).
    """
    sim.send_command("SET sTimeout=60000")

    r = sim.send_command("D 100 250 300")
    assert "OK" in r.upper(), f"D command failed: {r!r}"
    sim.tick_for(3000, step_ms=24)

    x_before, y_before, h_before = sim.get_fused_pose()
    v_before = sim.get_fused_v()
    omega_before = sim.get_fused_omega()
    p_before = [sim.get_ekf_p_diag(i) for i in range(5)]

    assert abs(x_before) > 20.0 or abs(y_before) > 20.0 or abs(h_before) > 0.1, (
        f"D command did not reach a non-trivial pose: "
        f"({x_before:.2f}, {y_before:.2f}, {h_before:.4f})"
    )

    set_reply = sim.send_command("SET tw=200")
    assert "OK" in set_reply.upper(), f"SET tw=200 failed: {set_reply!r}"
    set_reply2 = sim.send_command("SET rotSlip=0.5")
    assert "OK" in set_reply2.upper(), f"SET rotSlip=0.5 failed: {set_reply2!r}"

    get_reply = sim.send_command("GET tw")
    assert "200" in get_reply, f"GET tw did not reflect the SET: {get_reply!r}"
    get_reply2 = sim.send_command("GET rotSlip")
    assert "0.5" in get_reply2, f"GET rotSlip did not reflect the SET: {get_reply2!r}"

    x_after, y_after, h_after = sim.get_fused_pose()
    v_after = sim.get_fused_v()
    omega_after = sim.get_fused_omega()
    p_after = [sim.get_ekf_p_diag(i) for i in range(5)]

    assert x_after == x_before, f"fused x reset by SET tw/rotSlip: {x_before!r} -> {x_after!r}"
    assert y_after == y_before, f"fused y reset by SET tw/rotSlip: {y_before!r} -> {y_after!r}"
    assert h_after == h_before, f"fused heading reset by SET tw/rotSlip: {h_before!r} -> {h_after!r}"
    assert v_after == v_before, f"fused v reset by SET tw/rotSlip: {v_before!r} -> {v_after!r}"
    assert omega_after == omega_before, (
        f"fused omega reset by SET tw/rotSlip: {omega_before!r} -> {omega_after!r}"
    )
    for i in range(5):
        assert p_after[i] == p_before[i], (
            f"EKF covariance P[{i}][{i}] reset by SET tw/rotSlip: "
            f"{p_before[i]!r} -> {p_after[i]!r}"
        )


# ---------------------------------------------------------------------------
# 3. Narrowed getPose(const PoseEstimate&, ...) reads the ground-truth fused
#    pose, verified against an independent oracle.
# ---------------------------------------------------------------------------

def test_getpose_reads_ground_truth_fused_pose(sim):
    """Planner::getPoseFloat() -> PhysicalStateEstimate::getPose(const
    PoseEstimate&, ...) (070-003: narrowed from the whole HardwareState to
    the one PoseEstimate sub-struct it reads) must return the actual fused
    world pose.

    Verified against `sim.get_fused_pose()` -- an INDEPENDENT oracle that
    reads Drive::_hw.fused through a wholly separate C API
    (sim_get_pose_x/y/h), never touching PhysicalStateEstimate::getPose at
    all. Drive to a non-origin pose, read that independent fused pose, use
    it to compute the robot-relative offset that should drive the robot back
    to world (0, 0), issue that G command, and confirm the robot actually
    ends up near world (0, 0) -- if getPoseFloat()/getPose() had read a
    stale value, the wrong PoseEstimate struct, or the wrong field, the
    computed offset would be wrong and the robot would NOT converge on the
    origin.
    """
    sim.send_command("SET sTimeout=60000")

    r = sim.send_command("D 120 260 350")
    assert "OK" in r.upper(), f"D command failed: {r!r}"
    sim.tick_for(3000, step_ms=24)

    x0, y0, h0 = sim.get_fused_pose()
    assert abs(x0) > 20.0 or abs(y0) > 20.0, (
        f"D command did not reach a non-trivial pose: ({x0:.2f}, {y0:.2f})"
    )

    # Robot-relative offset (tx, ty) that drives the robot to world (0, 0),
    # computed independently of getPoseFloat()/getPose() -- pure trig on the
    # ground-truth (x0, y0, h0) read above.
    dx_world = 0.0 - x0
    dy_world = 0.0 - y0
    tx = dx_world * math.cos(h0) + dy_world * math.sin(h0)
    ty = -dx_world * math.sin(h0) + dy_world * math.cos(h0)

    r = sim.send_command(f"G {tx:.1f} {ty:.1f} 150")
    assert "OK" in r.upper(), f"G {tx:.1f} {ty:.1f} 150 -> {r!r}"

    sim.tick_for(6000, step_ms=24)

    x1, y1, _h1 = sim.get_fused_pose()
    dist_from_origin = math.sqrt(x1 * x1 + y1 * y1)
    assert dist_from_origin < 40.0, (
        f"G command computed from an independently-read fused pose should "
        f"converge on world (0, 0); ended at ({x1:.2f}, {y1:.2f}), "
        f"{dist_from_origin:.2f} mm from origin -- getPoseFloat()/getPose() "
        f"likely read a stale or wrong PoseEstimate"
    )
