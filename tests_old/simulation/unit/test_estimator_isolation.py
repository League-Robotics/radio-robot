"""
test_estimator_isolation.py — estimator-only isolation tests (040-005, Test 3).

Verifies that the firmware EKF estimate tracks plant truth and rejects bad OTOS.
This is the "estimator-only" row of the §7 verification matrix: it uses the new
estimation_error() ABI (WorldView crossing the plant/estimate boundary) to assert
the firmware's fused/dead-reckoned pose against the plant ground truth.

Scenarios:
  - Straight drive, no OTOS: the EKF tracks dead reckoning within tolerance
    (estimation_error()[0] small, heading error small).
  - OTOS fusion: enabling the OTOS model + fusion keeps the estimate tracking
    truth (fusion does not degrade a clean drive).
  - Bad OTOS injection: a wildly out-of-gate OTOS reading is rejected by the
    Mahalanobis gate (ekf_rej increments, the estimate barely moves, plant
    truth stays correct).
  - Recovery: after the bad reading is replaced by a truth-consistent OTOS
    reading, the estimation error returns within tolerance.
"""
import math

import pytest

from firmware import Sim


# ---------------------------------------------------------------------------
# Straight drive, no OTOS — EKF tracks dead reckoning
# ---------------------------------------------------------------------------

def test_clean_drive_estimation_error_small(sim):
    """After a clean straight drive (no OTOS), the EKF tracks truth within TOL.

    With perfect sensors and no slip, the firmware dead-reckoning estimate must
    match the plant truth closely (< 5 mm position, < 0.05 rad heading).
    """
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()

    sim.send_command("D 200 200 300")      # drive 300 mm forward
    sim.tick_for(8000, step_ms=24)
    assert "EVT done D" in sim.get_async_evts(), "D command did not complete"

    err_xy, err_h = sim.estimation_error()
    assert err_xy < 5.0, (
        f"estimation_error xy = {err_xy:.3f} mm after a clean drive — the EKF "
        f"is not tracking dead reckoning (expected < 5 mm)."
    )
    assert abs(err_h) < 0.05, (
        f"estimation_error h = {err_h:.4f} rad after a clean drive (expected "
        f"< 0.05 rad)."
    )
    # Sanity: the robot actually drove (otherwise error is trivially zero).
    assert sim.get_true_pose()[0] > 250.0, "plant did not reach the D target"


def test_turn_estimation_error_small(sim_field_profile):
    """After a TURN under field conditions the EKF heading tracks plant truth.

    The field profile's turn-slip model makes the plant body-rotation agree with
    the firmware's trackwidth-based estimate (a clean no-slip spin overshoots the
    plant heading on stop ramp-down, which is plant dynamics, not estimator
    drift — so this scenario uses the field fixture where the two agree)."""
    s = sim_field_profile
    s.send_command("TURN 9000")            # +90°
    s.tick_for(8000, step_ms=24)
    assert "EVT done TURN" in s.get_async_evts(), "TURN did not complete"

    err_xy, err_h = s.estimation_error()
    assert abs(err_h) < 0.1, (
        f"estimation_error h = {err_h:.4f} rad after a TURN (expected < 0.1 rad)."
    )
    assert err_xy < 20.0, f"estimation_error xy = {err_xy:.3f} mm after a TURN"


# ---------------------------------------------------------------------------
# OTOS fusion — fusion does not degrade a clean estimate
# ---------------------------------------------------------------------------

def test_otos_fusion_keeps_estimate_accurate(sim):
    """Enabling OTOS model + fusion keeps the estimate tracking truth on a drive."""
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()
    sim.enable_otos_model()
    sim.set_otos_fusion(True)

    sim.send_command("D 200 200 300")
    sim.tick_for(8000, step_ms=24)
    assert "EVT done D" in sim.get_async_evts(), "D command did not complete"

    err_xy, err_h = sim.estimation_error()
    assert err_xy < 5.0, (
        f"estimation_error xy = {err_xy:.3f} mm with OTOS fusion on a clean "
        f"drive (expected < 5 mm — fusion must not degrade a clean estimate)."
    )
    assert abs(err_h) < 0.05, f"estimation_error h = {err_h:.4f} rad with fusion"


# ---------------------------------------------------------------------------
# Bad OTOS injection — Mahalanobis gate rejects the outlier
# ---------------------------------------------------------------------------

def test_bad_otos_is_rejected(sim_field_profile):
    """A wildly out-of-gate OTOS reading is rejected: ekf_rej increments, the
    estimate barely moves, and the plant truth stays correct."""
    s = sim_field_profile
    s.send_command("D 200 200 300")
    s.tick_for(8000, step_ms=24)
    assert "EVT done D" in s.get_async_evts(), "D command did not complete"

    rej0 = s.get_ekf_rej_count()
    est0 = s.get_pose()
    true0 = s.get_true_pose()

    # Inject a wildly bad OTOS reading for a few ticks.
    for _ in range(3):
        s.set_otos_pose(9999.0, 9999.0, 99.0)
        s.tick_for(24, step_ms=24)

    rej1 = s.get_ekf_rej_count()
    est1 = s.get_pose()
    true1 = s.get_true_pose()

    # The Mahalanobis gate rejected the outlier reading(s).
    assert rej1 > rej0, (
        f"ekf_rej did not increase ({rej0}->{rej1}) — the bad OTOS reading was "
        f"NOT rejected by the gate."
    )
    # The estimate barely moved (it was not dragged toward 9999).
    est_jump = math.hypot(est1[0] - est0[0], est1[1] - est0[1])
    assert est_jump < 20.0, (
        f"estimate jumped {est_jump:.1f} mm on a rejected bad OTOS reading — the "
        f"gate did not protect the estimate (est {est0[:2]} -> {est1[:2]})."
    )
    # The plant truth is unaffected by a sensor read (sensors don't move trucks).
    assert true1[0] == pytest.approx(true0[0], abs=1.0), (
        f"plant true x changed on a sensor injection ({true0[0]}->{true1[0]})."
    )


# ---------------------------------------------------------------------------
# Recovery — truth-consistent OTOS restores the estimate after a bad burst
# ---------------------------------------------------------------------------

def test_estimator_recovers_after_bad_otos(sim_field_profile):
    """After a bad-OTOS burst, feeding a truth-consistent OTOS reading brings
    the estimation error back within tolerance."""
    s = sim_field_profile
    s.send_command("D 200 200 300")
    s.tick_for(8000, step_ms=24)
    assert "EVT done D" in s.get_async_evts(), "D command did not complete"

    # Bad burst (rejected).
    for _ in range(3):
        s.set_otos_pose(9999.0, 9999.0, 99.0)
        s.tick_for(24, step_ms=24)

    # Recovery: re-inject the OTOS pose at the (still-correct) plant truth and
    # let the EKF settle.  The estimation error must return within tolerance.
    tx, ty, th = s.get_true_pose()
    for _ in range(60):
        s.set_otos_pose(tx, ty, th)
        s.tick_for(24, step_ms=24)

    err_xy, err_h = s.estimation_error()
    assert err_xy < 20.0, (
        f"estimation_error xy = {err_xy:.3f} mm after recovery — the EKF did not "
        f"recover after the bad-OTOS burst was replaced with a good reading."
    )
    assert abs(err_h) < 0.15, f"estimation_error h = {err_h:.4f} rad after recovery"
