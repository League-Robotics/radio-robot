"""
test_observation_models.py — observation-only isolation tests (040-005, Test 2).

Verifies that the observation models (SimOdometer, SimMotor) correctly reflect
the configured sensor error WITHOUT changing the plant truth.  This is the
"observation-only" row of the §7 verification matrix: it isolates the sensor
read path from the chassis dynamics and from the firmware estimator.

Scenarios:
  - Perfect odometer (default): the OTOS sim-model accumulator tracks plant
    truth when no error is configured.
  - Read-failure dropout: with the OTOS read failure asserted, the firmware
    skips fusion (no EKF jump toward a bad injected reading, no gate rejection);
    clearing the failure resumes fusion on the next tick.
  - Frozen encoder: a frozen drive wheel (offset factor 0) stops advancing while
    the plant keeps integrating the other wheel — the observation model reports
    the frozen value, the plant truth keeps moving.

Yaw-drift (SimOdometer::setYawDriftRadsPerSec) is DEFERRED: no C ABI entry point
exists for it in this sprint (see ticket §"Yaw drift" note), so it is not
exercised here.
"""
import ctypes
import math

import pytest

from firmware import Sim

# Drive wheel side indices for sim_set_motor_offset (matches SimMotor::Side).
SIDE_LEFT = 0
SIDE_RIGHT = 1


# ---------------------------------------------------------------------------
# Perfect odometer: OTOS accumulator tracks plant truth (no error configured)
# ---------------------------------------------------------------------------

def test_perfect_otos_tracks_truth_on_straight_drive(sim):
    """With no error configured, the OTOS odom accumulator equals plant truth.

    The SimOdometer sim-model integrates the SAME true per-wheel velocity arc as
    the plant, so with zero noise its accumulated pose equals the plant truth.
    """
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()
    sim.enable_otos_model()
    sim.set_otos_fusion(True)

    sim.send_command("VW 200 0")           # 200 mm/s straight forward
    sim.tick_for(1000, step_ms=24)

    ox, oy, oh = sim.get_otos_pose()
    tx, ty, th = sim.get_true_pose()

    # Odometer reads truth (within tight tolerance) when no error is configured.
    assert ox == pytest.approx(tx, abs=1.0), f"OTOS x={ox} vs true x={tx}"
    assert oy == pytest.approx(ty, abs=1.0), f"OTOS y={oy} vs true y={ty}"
    assert oh == pytest.approx(th, abs=1e-2), f"OTOS h={oh} vs true h={th}"
    # And it actually moved (otherwise the equality is trivially true).
    assert tx > 50.0, f"plant did not advance: true x={tx}"


def test_perfect_otos_tracks_turn(sim):
    """The OTOS heading accumulator tracks plant truth through a spin."""
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()
    sim.enable_otos_model()
    sim.set_otos_fusion(True)

    sim.send_command("VW 0 200")           # spin CCW
    sim.tick_for(800, step_ms=24)

    oh = sim.get_otos_pose()[2]
    th = sim.get_true_pose()[2]
    assert oh == pytest.approx(th, abs=2e-2), f"OTOS h={oh} vs true h={th}"
    assert abs(th) > 0.05, f"plant did not rotate: true h={th}"


# ---------------------------------------------------------------------------
# Read-failure dropout: fusion skipped, no EKF jump
# ---------------------------------------------------------------------------

def test_read_failure_skips_fusion_no_jump(sim):
    """OTOS read failure → fusion skipped: the estimate does not jump to a bad
    injected reading and the gate rejection count does not increase."""
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()
    sim.enable_otos_model()
    sim.set_otos_fusion(True)

    sim.send_command("VW 200 0")
    sim.tick_for(600, step_ms=24)

    # Inject a wildly bad OTOS reading, but assert read failure so the firmware
    # never even evaluates the OTOS sample (lastReadOk() == False).
    sim.set_otos_pose(9999.0, 9999.0, 99.0)
    sim.set_otos_read_failure(True)

    est_x0 = sim.get_pose()[0]
    rej0 = sim.get_ekf_rej_count()

    sim.tick_for(120, step_ms=24)          # 5 ticks under read failure

    est_x1 = sim.get_pose()[0]
    rej1 = sim.get_ekf_rej_count()

    # Estimate must NOT jump toward 9999 (it continues normal dead reckoning).
    assert est_x1 < 1000.0, (
        f"estimate jumped to {est_x1} during OTOS read failure — fusion was not "
        f"skipped (the bad 9999 reading leaked into the EKF)."
    )
    # Read failure means the sample is never gated, so no rejection is counted.
    assert rej1 == rej0, (
        f"ekf_rej went {rej0}->{rej1} during read failure — the bad sample was "
        f"evaluated when it should have been dropped before the gate."
    )


def test_read_failure_clear_resumes_fusion(sim):
    """After clearing the read failure, the firmware resumes evaluating OTOS.

    The still-bad injected reading is now gated and rejected — proving fusion
    resumed (the sample reaches the Mahalanobis gate again)."""
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()
    sim.enable_otos_model()
    sim.set_otos_fusion(True)

    sim.send_command("VW 200 0")
    sim.tick_for(600, step_ms=24)

    sim.set_otos_pose(9999.0, 9999.0, 99.0)
    sim.set_otos_read_failure(True)
    sim.tick_for(120, step_ms=24)
    rej_during_fail = sim.get_ekf_rej_count()

    # Clear the failure — fusion resumes; the bad sample now reaches the gate.
    sim.set_otos_read_failure(False)
    sim.tick_for(120, step_ms=24)
    rej_after_clear = sim.get_ekf_rej_count()

    assert rej_after_clear > rej_during_fail, (
        f"ekf_rej did not increase after clearing the read failure "
        f"({rej_during_fail}->{rej_after_clear}) — fusion did not resume."
    )
    # The estimate still must not have jumped to the bad value (it was rejected).
    assert sim.get_pose()[0] < 1000.0, "estimate jumped despite gate rejection"


# ---------------------------------------------------------------------------
# Frozen encoder: observation freezes while plant truth advances
# ---------------------------------------------------------------------------

def test_frozen_encoder_holds_while_plant_advances(sim):
    """A frozen drive wheel (offset 0) stops reporting travel while the plant
    keeps moving the other wheel — observation freeze does not freeze truth."""
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()

    # Spin so the two wheels travel in opposite directions and differentiate.
    sim.send_command("VW 0 200")
    sim.tick_for(300, step_ms=24)

    enc_r0 = float(sim._lib.sim_get_enc_r(sim._h))
    enc_l0 = float(sim._lib.sim_get_enc_l(sim._h))

    # Freeze the right wheel's reported encoder (offset factor 0 → no advance).
    sim._lib.sim_set_motor_offset(sim._h, SIDE_RIGHT, ctypes.c_float(0.0))

    sim.tick_for(600, step_ms=24)

    enc_r1 = float(sim._lib.sim_get_enc_r(sim._h))
    enc_l1 = float(sim._lib.sim_get_enc_l(sim._h))

    # Right encoder reading is frozen (held at its pre-freeze value).
    assert enc_r1 == pytest.approx(enc_r0, abs=0.5), (
        f"frozen right encoder advanced {enc_r0:.2f}->{enc_r1:.2f} mm — the "
        f"observation should hold while frozen."
    )
    # The left wheel kept advancing — the plant did NOT freeze.
    assert abs(enc_l1 - enc_l0) > 5.0, (
        f"left encoder barely moved ({enc_l0:.2f}->{enc_l1:.2f}) while right was "
        f"frozen — the plant truth should keep advancing the other wheel."
    )


def test_frozen_encoder_does_not_corrupt_other_side(sim):
    """Freezing one wheel must not change the OTHER wheel's reported travel."""
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()

    sim.send_command("VW 200 0")           # straight: both wheels advance
    sim.tick_for(300, step_ms=24)

    # Freeze LEFT this time.
    sim._lib.sim_set_motor_offset(sim._h, SIDE_LEFT, ctypes.c_float(0.0))
    enc_l0 = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r0 = float(sim._lib.sim_get_enc_r(sim._h))

    sim.tick_for(400, step_ms=24)
    enc_l1 = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r1 = float(sim._lib.sim_get_enc_r(sim._h))

    assert enc_l1 == pytest.approx(enc_l0, abs=0.5), "frozen left encoder advanced"
    assert (enc_r1 - enc_r0) > 5.0, (
        f"right encoder did not advance ({enc_r0:.2f}->{enc_r1:.2f}) while only "
        f"the left was frozen."
    )
