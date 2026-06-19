"""
test_estimator_command_paths.py — coverage-focused estimator/motion command
isolation tests (040-005, coverage lever for the 85% goal).

These exercise firmware-logic paths reachable in sim that the plant/observation
split makes assertable against ground truth, but which the existing suite did
not cover:

  - The OTOS odometer command handlers (OI / OZ / OR / OP / OV / OL / OA) in
    Odometry.cpp / the OTOS command surface — init, zero, reset, print, set-pose,
    and linear/angular scalar configuration.
  - Asymmetric / curved drives that drive the MotorController per-wheel ratio
    seeding path (startDrive with fasterIsRight ≠ symmetric), which symmetric
    straight-line tests never reach.

Every assertion checks real behaviour (reply wire format, true-pose effect),
not merely "the line executed".
"""
import math

import pytest

from firmware import Sim


@pytest.fixture
def sim_otos(sim):
    """A sim with the OTOS odometer model enabled (so OTOS commands find a device).

    Without an enabled odometer the OTOS command handlers reply 'ERR nodev'; the
    sim-model enable + begin() makes them operate on the SimOdometer.
    """
    sim.send_command("SET sTimeout=60000")
    sim.enable_otos_model()
    sim.set_otos_fusion(True)
    return sim


# ---------------------------------------------------------------------------
# OTOS odometer command handlers (OI / OZ / OR / OP / OV / OL / OA)
# ---------------------------------------------------------------------------

def test_otos_init_zero_reset_ok(sim_otos):
    """OI / OZ / OR are accepted (odometer init / zero / reset)."""
    assert "OK" in sim_otos.send_command("OI").upper()
    assert "OK" in sim_otos.send_command("OZ").upper()
    assert "OK" in sim_otos.send_command("OR").upper()


def test_otos_print_reports_pose(sim_otos):
    """OP prints the current OTOS pose as x= y= h= fields."""
    reply = sim_otos.send_command("OP")
    assert reply.upper().startswith("OK")
    for field in ("x=", "y=", "h="):
        assert field in reply, f"OP reply missing {field!r}: {reply!r}"


def test_otos_setpose_roundtrips(sim_otos):
    """OV <x> <y> <h> sets the OTOS pose; the handler echoes it back."""
    reply = sim_otos.send_command("OV 100 200 50")
    assert reply.upper().startswith("OK")
    assert "x=100" in reply and "y=200" in reply and "h=50" in reply, (
        f"OV reply did not echo the set pose: {reply!r}"
    )
    # Zero it again and confirm the handler accepts the second set.
    reply2 = sim_otos.send_command("OV 0 0 0")
    assert "x=0" in reply2 and "y=0" in reply2 and "h=0" in reply2


def test_otos_setpose_badarg(sim_otos):
    """OV with fewer than three args → ERR badarg (parse guard path)."""
    reply = sim_otos.send_command("OV 100 200")
    assert reply.upper().startswith("ERR"), f"OV with 2 args should ERR: {reply!r}"


def test_otos_linear_scalar_set_and_default(sim_otos):
    """OL reads the linear scalar; OL <val> sets it."""
    assert "scalar=0" in sim_otos.send_command("OL"), "default linear scalar should be 0"
    reply = sim_otos.send_command("OL 5")
    assert "scalar=5" in reply, f"OL 5 did not set scalar: {reply!r}"


def test_otos_angular_scalar_set_and_default(sim_otos):
    """OA reads the angular scalar; OA <val> sets it."""
    assert "scalar=0" in sim_otos.send_command("OA"), "default angular scalar should be 0"
    reply = sim_otos.send_command("OA 3")
    assert "scalar=3" in reply, f"OA 3 did not set scalar: {reply!r}"


# ---------------------------------------------------------------------------
# Asymmetric / curved drives — MotorController per-wheel ratio seeding
# ---------------------------------------------------------------------------

def test_curved_drive_advances_and_rotates(sim):
    """VW with a non-zero omega drives a curved path: the plant advances AND
    rotates (asymmetric wheel velocities exercise the ratio-seeding path)."""
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()

    sim.send_command("VW 200 100")         # forward + CCW rotation
    sim.tick_for(1200, step_ms=24)

    tx, ty, th = sim.get_true_pose()
    assert tx > 30.0, f"curved drive did not advance: true_x={tx:.1f}"
    assert abs(th) > 0.05, f"curved drive did not rotate: true_h={th:.3f}"
    # CCW rotation with forward motion curves toward +y.
    assert ty > 0.0, f"curved drive curved the wrong way: true_y={ty:.1f}"


def test_streaming_recommand_changes_ratio(sim):
    """Re-issuing VW mid-drive with a new ratio (startDrive path) keeps the
    plant moving without a discontinuity — the seed-from-prior-delta branch."""
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()

    sim.send_command("VW 200 100")
    sim.tick_for(800, step_ms=24)
    pose_mid = sim.get_true_pose()

    # New asymmetric command with a different ratio → startDrive re-seed.
    sim.send_command("VW 100 250")
    sim.tick_for(800, step_ms=24)
    pose_end = sim.get_true_pose()

    # The robot kept moving across the re-command (no stall / no reset to origin).
    moved = math.hypot(pose_end[0] - pose_mid[0], pose_end[1] - pose_mid[1])
    assert moved > 10.0, (
        f"plant did not advance across the VW re-command "
        f"({pose_mid[:2]} -> {pose_end[:2]}, moved {moved:.1f} mm)."
    )
    # Heading kept increasing (both commands rotate CCW).
    assert pose_end[2] > pose_mid[2], (
        f"heading did not advance across the re-command "
        f"({pose_mid[2]:.3f} -> {pose_end[2]:.3f})."
    )


def test_asymmetric_d_curves_true_pose(sim):
    """An asymmetric D (vL ≠ vR) drives a curved path in the true pose."""
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()

    # Left slower than right → curve toward +y (left).
    sim.send_command("D 100 200 200")
    sim.tick_for(8000, step_ms=24)
    assert "EVT done D" in sim.get_async_evts(), "asymmetric D did not complete"

    tx, ty, th = sim.get_true_pose()
    assert tx > 20.0, f"asymmetric D did not advance: true_x={tx:.1f}"
    # Right wheel faster → robot curves left (+y) and rotates CCW (+h).
    assert th > 0.02, f"asymmetric D did not rotate CCW: true_h={th:.3f}"
