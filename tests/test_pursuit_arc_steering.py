#!/usr/bin/env python3
"""test_pursuit_arc_steering.py — Unit tests for pursuit-arc steering law (011-002).

Pure Python implementation of the receding-horizon curvature steering law from:
  docs/kinematics-model.md §1.5
  source/control/DriveController.cpp (PURSUE branch)

Tests verify:
  - World-to-robot-frame goal transform
  - Curvature law: κ = 2·dy/(dx²+dy²)
  - Zero-distance guard: d² ≤ 0.1 → κ = 0 (no divide-by-zero)
  - Straight-ahead: κ = 0, ω = 0, vL = vR = _gSpeed
  - 45° left offset: κ = 0.01, ω = _gSpeed · 0.01
  - 90° left: κ = 0.02
  - Goal at origin: d² guard fires, κ = 0
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Pure Python mirrors of the C++ pursuit-arc steering law
# ---------------------------------------------------------------------------

def world_to_robot_goal(
    gx_world: float,
    gy_world: float,
    robot_x: float,
    robot_y: float,
    robot_h_rad: float,
) -> tuple[float, float]:
    """Transform a world-frame goal into robot frame.

    C++ equivalent in DriveController::tick() PURSUE branch:
        dxW = _gTargetXWorld - x
        dyW = _gTargetYWorld - y
        dx  =  dxW * cosf(h_rad) + dyW * sinf(h_rad)
        dy  = -dxW * sinf(h_rad) + dyW * cosf(h_rad)
    """
    dxW = gx_world - robot_x
    dyW = gy_world - robot_y
    dx =  dxW * math.cos(robot_h_rad) + dyW * math.sin(robot_h_rad)
    dy = -dxW * math.sin(robot_h_rad) + dyW * math.cos(robot_h_rad)
    return dx, dy


def compute_kappa(dx: float, dy: float) -> float:
    """Pursuit-arc curvature: κ = 2·dy/(dx²+dy²), with d²≤0.1 guard → 0.

    C++ equivalent in DriveController::tick() PURSUE branch:
        float d2    = dx * dx + dy * dy;
        float kappa = (d2 > 0.1f) ? (2.0f * dy / d2) : 0.0f;
    """
    d2 = dx * dx + dy * dy
    if d2 > 0.1:
        return 2.0 * dy / d2
    return 0.0


def beginGoTo_world_goal(
    tx: float,
    ty: float,
    robot_x: float,
    robot_y: float,
    robot_h_rad: float,
) -> tuple[float, float]:
    """Transform robot-relative (tx, ty) goal to world frame at beginGoTo() time.

    C++ equivalent in DriveController::beginGoTo():
        _gTargetXWorld = x + tx * cosf(h_rad) - ty * sinf(h_rad)
        _gTargetYWorld = y + tx * sinf(h_rad) + ty * cosf(h_rad)
    """
    gx = robot_x + tx * math.cos(robot_h_rad) - ty * math.sin(robot_h_rad)
    gy = robot_y + tx * math.sin(robot_h_rad) + ty * math.cos(robot_h_rad)
    return gx, gy


def bk_inverse(v: float, omega: float, b: float) -> tuple[float, float]:
    """vL = v - omega*(b/2), vR = v + omega*(b/2)."""
    half_b = b / 2.0
    vL = v - omega * half_b
    vR = v + omega * half_b
    return vL, vR


# ---------------------------------------------------------------------------
# Tests — curvature formula κ = 2·dy/(dx²+dy²)
# ---------------------------------------------------------------------------

class TestCurvatureLaw:
    """Verify κ = 2·dy/(dx²+dy²) covers all acceptance-criteria cases."""

    def test_straight_ahead_kappa_zero(self):
        """AC: goal (dx=300, dy=0) → κ = 0."""
        kappa = compute_kappa(dx=300.0, dy=0.0)
        assert kappa == pytest.approx(0.0, abs=1e-9)

    def test_45_deg_left(self):
        """AC: goal (dx=100, dy=100) → κ = 2·100/(100²+100²) = 0.01."""
        # d² = 10000 + 10000 = 20000; κ = 200/20000 = 0.01
        kappa = compute_kappa(dx=100.0, dy=100.0)
        assert kappa == pytest.approx(0.01, rel=1e-6)

    def test_90_deg_left(self):
        """AC: goal (dx=0, dy=100) → κ = 2·100/(0+10000) = 0.02."""
        kappa = compute_kappa(dx=0.0, dy=100.0)
        assert kappa == pytest.approx(0.02, rel=1e-6)

    def test_zero_distance_guard(self):
        """AC: goal (dx=0, dy=0) → d²=0 ≤ 0.1 guard fires, κ = 0."""
        kappa = compute_kappa(dx=0.0, dy=0.0)
        assert kappa == pytest.approx(0.0, abs=1e-9)

    def test_guard_threshold_boundary(self):
        """d² exactly at boundary (0.1): guard fires, κ = 0."""
        # dx=sqrt(0.05), dy=sqrt(0.05): d²=0.1 (not > 0.1)
        dx = math.sqrt(0.05)
        dy = math.sqrt(0.05)
        kappa = compute_kappa(dx=dx, dy=dy)
        assert kappa == pytest.approx(0.0, abs=1e-9)

    def test_guard_just_above_threshold(self):
        """d² just above 0.1: guard does NOT fire, κ = 2·dy/d²."""
        dx = 0.0
        dy = math.sqrt(0.1001)  # d² ≈ 0.1001 > 0.1
        kappa = compute_kappa(dx=dx, dy=dy)
        d2 = dx * dx + dy * dy
        expected = 2.0 * dy / d2
        assert kappa == pytest.approx(expected, rel=1e-5)

    def test_right_turn_negative_kappa(self):
        """Negative dy (goal to right) → negative κ → right turn."""
        kappa = compute_kappa(dx=100.0, dy=-100.0)
        assert kappa == pytest.approx(-0.01, rel=1e-6)

    def test_behind_robot(self):
        """Goal directly behind (dx<0, dy=0): κ = 0 (no lateral offset)."""
        kappa = compute_kappa(dx=-200.0, dy=0.0)
        assert kappa == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Tests — straight-ahead steering (vL = vR = v)
# ---------------------------------------------------------------------------

class TestStraightAheadSteering:
    """Verify full steering chain for goal directly ahead."""

    TRACK_WIDTH = 120.0   # mm
    SPEED       = 200.0   # mm/s

    def test_straight_ahead_no_steering(self):
        """AC: goal (300, 0), robot at origin facing +x → κ=0, ω=0, vL=vR=speed."""
        # beginGoTo(): robot at (0,0), heading=0; goal (300,0) robot-relative
        #   → world goal = (300, 0)
        # tick(): robot at (0,0), heading=0
        #   dx=300, dy=0 in robot frame
        dx, dy = 300.0, 0.0
        kappa = compute_kappa(dx, dy)
        omega = self.SPEED * kappa
        vL, vR = bk_inverse(self.SPEED, omega, self.TRACK_WIDTH)

        assert kappa == pytest.approx(0.0, abs=1e-9)
        assert omega == pytest.approx(0.0, abs=1e-9)
        assert vL == pytest.approx(self.SPEED)
        assert vR == pytest.approx(self.SPEED)

    def test_omega_proportional_to_kappa(self):
        """ω = v · κ: for goal (100, 100), ω = 200 · 0.01 = 2.0 rad/s."""
        dx, dy = 100.0, 100.0
        kappa = compute_kappa(dx, dy)
        omega = self.SPEED * kappa
        assert kappa == pytest.approx(0.01, rel=1e-6)
        assert omega == pytest.approx(self.SPEED * 0.01, rel=1e-6)


# ---------------------------------------------------------------------------
# Tests — world-to-robot-frame transform
# ---------------------------------------------------------------------------

class TestWorldToRobotTransform:
    """Verify world→robot-frame goal projection used in each PURSUE tick."""

    def test_robot_at_origin_facing_right(self):
        """Robot at (0,0,0): world goal = robot goal (identity transform)."""
        dx, dy = world_to_robot_goal(
            gx_world=300.0, gy_world=0.0,
            robot_x=0.0, robot_y=0.0, robot_h_rad=0.0
        )
        assert dx == pytest.approx(300.0, rel=1e-6)
        assert dy == pytest.approx(0.0, abs=1e-6)

    def test_robot_facing_90deg_ccw(self):
        """Robot at (0,0) facing 90° CCW (+y): world goal (300,0) → robot frame (0,-300).

        Robot's +x axis points in world +y direction.
        World goal at (300, 0) is therefore behind and right of robot:
          dx = 300*cos(90°) + 0*sin(90°) = 0
          dy = -300*sin(90°) + 0*cos(90°) = -300
        """
        h_rad = math.pi / 2.0
        dx, dy = world_to_robot_goal(
            gx_world=300.0, gy_world=0.0,
            robot_x=0.0, robot_y=0.0, robot_h_rad=h_rad
        )
        # world goal is at 90° right of robot's forward direction
        assert dx == pytest.approx(0.0, abs=1e-5)
        assert dy == pytest.approx(-300.0, rel=1e-5)

    def test_robot_translated(self):
        """Robot at (100,200,0): world goal (400,200) → robot frame (300, 0)."""
        dx, dy = world_to_robot_goal(
            gx_world=400.0, gy_world=200.0,
            robot_x=100.0, robot_y=200.0, robot_h_rad=0.0
        )
        assert dx == pytest.approx(300.0, rel=1e-6)
        assert dy == pytest.approx(0.0, abs=1e-6)

    def test_beginGoTo_roundtrip_zero_pose(self):
        """beginGoTo() world transform then tick() inverse → original robot-relative goal.

        Robot at (0,0,0): beginGoTo with (tx=300, ty=100) stores world goal.
        On first tick (robot still at 0,0,0), world_to_robot recovers (300, 100).
        """
        tx, ty = 300.0, 100.0
        gx, gy = beginGoTo_world_goal(tx, ty, robot_x=0.0, robot_y=0.0, robot_h_rad=0.0)
        dx, dy = world_to_robot_goal(gx, gy, robot_x=0.0, robot_y=0.0, robot_h_rad=0.0)
        assert dx == pytest.approx(tx, rel=1e-6)
        assert dy == pytest.approx(ty, rel=1e-6)

    def test_beginGoTo_roundtrip_nonzero_pose(self):
        """beginGoTo() + tick() at same pose recovers original goal: rotated pose."""
        tx, ty = 200.0, 150.0
        h_rad  = math.pi / 6.0   # 30° CCW
        rx, ry = 50.0, 75.0      # robot position

        gx, gy = beginGoTo_world_goal(tx, ty, robot_x=rx, robot_y=ry, robot_h_rad=h_rad)
        dx, dy = world_to_robot_goal(gx, gy, robot_x=rx, robot_y=ry, robot_h_rad=h_rad)
        assert dx == pytest.approx(tx, abs=1e-4)
        assert dy == pytest.approx(ty, abs=1e-4)

    def test_goal_directly_ahead_kappa_zero(self):
        """After transform: goal directly ahead in robot frame → κ = 0."""
        # Robot at (100, 200) facing 0: goal at world (400, 200)
        dx, dy = world_to_robot_goal(
            gx_world=400.0, gy_world=200.0,
            robot_x=100.0, robot_y=200.0, robot_h_rad=0.0
        )
        kappa = compute_kappa(dx, dy)
        assert kappa == pytest.approx(0.0, abs=1e-9)

    def test_goal_to_left_positive_kappa(self):
        """Goal to the left of robot's direction → positive κ (CCW turn)."""
        # Robot at (0,0) facing 0; goal at world (0, 300) → 90° to the left
        dx, dy = world_to_robot_goal(
            gx_world=0.0, gy_world=300.0,
            robot_x=0.0, robot_y=0.0, robot_h_rad=0.0
        )
        kappa = compute_kappa(dx, dy)
        # dy=300 > 0, so kappa > 0
        assert kappa > 0.0

    def test_goal_to_right_negative_kappa(self):
        """Goal to the right of robot's direction → negative κ (CW turn)."""
        # Robot at (0,0) facing 0; goal at world (0, -300) → 90° to the right
        dx, dy = world_to_robot_goal(
            gx_world=0.0, gy_world=-300.0,
            robot_x=0.0, robot_y=0.0, robot_h_rad=0.0
        )
        kappa = compute_kappa(dx, dy)
        assert kappa < 0.0
