"""
test_whole_robot_plant_correctness.py — whole-robot isolation tests
(040-005, Test 4 — system tier).

Verifies end-to-end robot motion from command to TRUE chassis pose.  This is the
"whole-robot" row of the §7 verification matrix and the capability that the
physical-plant split unlocks: asserting the final TRUE pose (from the plant,
via sim_get_true_pose_*) against the planned target — an assertion that is
IMPOSSIBLE on real hardware (there is no ground-truth oracle on the robot) and
is now possible in sim via PhysicsWorld.

Scenarios (all under the field profile — realistic turn slip + OTOS fusion):
  - D command (distance drive): D 200 200 200 → true_pose_x within 20 mm of 200.
  - G command (robot-relative go-to): G 200 0 200 → true pose within 25 mm of
    the (200, 0) target.
  - TURN command: TURN 9000 (90°) → true_pose_h within 0.1 rad of π/2.
  - A multi-step D/G/TURN plan, asserting the TRUE pose at each step plus the
    estimation_error gate (the EKF tracks the true pose within 20 mm).
"""
import math

import pytest

from firmware import Sim


def _drive(s, cmd, evt_tag, settle_ms=10000):
    """Send a motion command, tick until its EVT done, assert completion."""
    s.get_async_evts()                     # clear any prior events
    reply = s.send_command(cmd)
    assert reply.upper().startswith("OK") or "OK" in reply.upper(), (
        f"command {cmd!r} was not accepted: {reply!r}"
    )
    s.tick_for(settle_ms, step_ms=24)
    evts = s.get_async_evts()
    assert f"EVT done {evt_tag}" in evts, (
        f"command {cmd!r} did not complete (no 'EVT done {evt_tag}'): {evts!r}"
    )


# ---------------------------------------------------------------------------
# Single-command true-pose correctness
# ---------------------------------------------------------------------------

def test_d_command_true_pose(sim_field_profile):
    """D 200 200 200 → final TRUE pose_x within 20 mm of the 200 mm target."""
    s = sim_field_profile
    _drive(s, "D 200 200 200", "D")

    tx, ty, th = s.get_true_pose()
    assert tx > 150.0, f"true_pose_x = {tx:.1f} mm — robot did not drive the D"
    assert tx == pytest.approx(200.0, abs=20.0), (
        f"true_pose_x = {tx:.1f} mm, expected ~200 ± 20 (D distance target)."
    )
    # Straight drive: minimal lateral / heading drift in the true pose.
    assert abs(ty) < 25.0, f"true_pose_y = {ty:.1f} mm drift on a straight D"

    err_xy, _ = s.estimation_error()
    assert err_xy < 20.0, (
        f"estimation_error = {err_xy:.1f} mm after D — EKF lost the true pose."
    )


def test_g_command_true_pose(sim_field_profile):
    """G 200 0 200 (robot-relative) → TRUE pose within 25 mm of (200, 0)."""
    s = sim_field_profile
    _drive(s, "G 200 0 200", "G")

    tx, ty, th = s.get_true_pose()
    assert tx == pytest.approx(200.0, abs=25.0), (
        f"true_pose_x = {tx:.1f} mm, expected ~200 ± 25 (G x target)."
    )
    assert ty == pytest.approx(0.0, abs=25.0), (
        f"true_pose_y = {ty:.1f} mm, expected ~0 ± 25 (G y target)."
    )

    err_xy, _ = s.estimation_error()
    assert err_xy < 20.0, f"estimation_error = {err_xy:.1f} mm after G"


def test_turn_command_true_pose(sim_field_profile):
    """TURN 9000 (90°) → final TRUE pose_h within 0.1 rad of π/2."""
    s = sim_field_profile
    _drive(s, "TURN 9000", "TURN")

    th = s.get_true_pose()[2]
    th_wrapped = math.atan2(math.sin(th), math.cos(th))
    assert th_wrapped == pytest.approx(math.pi / 2.0, abs=0.1), (
        f"true_pose_h = {th_wrapped:.4f} rad, expected ~π/2 ± 0.1 (TURN 90°)."
    )

    _, err_h = s.estimation_error()
    assert abs(err_h) < 0.1, (
        f"estimation_error h = {err_h:.4f} rad after TURN — EKF lost true heading."
    )


# ---------------------------------------------------------------------------
# Multi-step D / G / TURN plan — true pose tracked at every step
# ---------------------------------------------------------------------------

def test_d_turn_g_plan_true_pose(sim_field_profile):
    """A D → TURN → G plan: assert the TRUE pose and the estimation_error gate
    after each step (the whole-robot, true-pose, end-to-end assertion)."""
    s = sim_field_profile

    # Step 1: drive forward 200 mm.
    _drive(s, "D 200 200 200", "D")
    tx, ty, th = s.get_true_pose()
    assert tx == pytest.approx(200.0, abs=25.0), f"after D: true_x={tx:.1f}"
    err_xy, _ = s.estimation_error()
    assert err_xy < 20.0, f"after D: estimation_error={err_xy:.1f} mm"

    # Step 2: turn in place to +90°.  The true heading should be ~π/2 and the
    # position should be roughly preserved (spot turn, minimal translation).
    x_before_turn = s.get_true_pose()[0]
    _drive(s, "TURN 9000", "TURN")
    tx, ty, th = s.get_true_pose()
    th_wrapped = math.atan2(math.sin(th), math.cos(th))
    assert th_wrapped == pytest.approx(math.pi / 2.0, abs=0.15), (
        f"after TURN: true_h={th_wrapped:.4f} rad, expected ~π/2"
    )
    assert tx == pytest.approx(x_before_turn, abs=40.0), (
        f"after TURN: true_x drifted {x_before_turn:.1f}->{tx:.1f} on a spot turn"
    )
    err_xy, err_h = s.estimation_error()
    assert err_xy < 25.0, f"after TURN: estimation_error={err_xy:.1f} mm"
    assert abs(err_h) < 0.15, f"after TURN: estimation_error_h={err_h:.4f} rad"

    # Step 3: go-to a robot-relative forward point.  After facing +90° (world
    # +y), driving 150 mm forward should advance the WORLD y by ~150 mm.
    y_before_g = s.get_true_pose()[1]
    _drive(s, "G 150 0 200", "G")
    tx, ty, th = s.get_true_pose()
    assert (ty - y_before_g) > 100.0, (
        f"after G 150 0 (facing +90°): world y advanced only "
        f"{ty - y_before_g:.1f} mm (expected ~150)."
    )
    err_xy, _ = s.estimation_error()
    assert err_xy < 30.0, f"after G: estimation_error={err_xy:.1f} mm"


def test_estimation_error_gate_holds_across_plan(sim_field_profile):
    """The EKF estimate tracks the TRUE pose within 20 mm across a two-leg drive.

    Drives two consecutive D legs and asserts the estimation_error gate after
    each — the firmware never loses the true pose during a normal plan."""
    s = sim_field_profile

    _drive(s, "D 200 200 200", "D")
    err1, _ = s.estimation_error()
    assert err1 < 20.0, f"leg 1: estimation_error={err1:.1f} mm"
    assert s.get_true_pose()[0] > 150.0, "leg 1: robot did not move"

    _drive(s, "D 200 200 150", "D")
    err2, _ = s.estimation_error()
    assert err2 < 20.0, f"leg 2: estimation_error={err2:.1f} mm"
    # Two forward legs accumulate (~200 + ~150 = ~350 mm true x).
    assert s.get_true_pose()[0] > 300.0, (
        f"after two D legs true_x={s.get_true_pose()[0]:.1f} (expected ~350)"
    )
