"""Tests for the extended RobotState and Nezha back-compat properties.

Verifies:
1. _apply_tlm populates all RobotState fields from a full TLMFrame.
2. Partial-frame handling: absent fields retain prior values.
3. Back-compat properties (encoders, otos_pose, line_sensor, color) agree
   with state fields.
4. state.stamp is recent (within 1 second of time.monotonic()).

No hardware needed — tests use MagicMock SerialConnection directly.
"""

from __future__ import annotations

import math
import time
from unittest.mock import MagicMock

import pytest

from robot_radio.robot.protocol import NezhaProtocol, TLMFrame
from robot_radio.robot.nezha import Nezha
from robot_radio.robot.robot_state import RobotState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mock_conn() -> MagicMock:
    conn = MagicMock()
    conn.is_open = True
    conn.mode = "relay"
    conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": []}
    conn.send_fast.return_value = None
    conn.read_lines.return_value = []
    return conn


def _make_robot() -> Nezha:
    proto = NezhaProtocol(_mock_conn())
    return Nezha(proto)


# ---------------------------------------------------------------------------
# test_apply_tlm_populates_state
# ---------------------------------------------------------------------------

class TestApplyTlmPopulatesState:
    """_apply_tlm with a full TLMFrame populates all RobotState fields."""

    def test_encoders_populated(self) -> None:
        robot = _make_robot()
        tlm = TLMFrame(enc=(123, 119), pose=(100, 50, 9000),
                       twist=(200, 314), line=(10, 20, 30, 40),
                       color=(255, 128, 0, 200))
        robot._apply_tlm(tlm)
        assert robot.state.encoders == (123, 119)

    def test_pose_x_y_populated(self) -> None:
        robot = _make_robot()
        tlm = TLMFrame(pose=(100, 50, 9000))
        robot._apply_tlm(tlm)
        assert robot.state.pose.x == pytest.approx(100.0)
        assert robot.state.pose.y == pytest.approx(50.0)

    def test_pose_heading_in_radians(self) -> None:
        """9000 cdeg = 90 degrees = pi/2 radians."""
        robot = _make_robot()
        tlm = TLMFrame(pose=(0, 0, 9000))
        robot._apply_tlm(tlm)
        assert robot.state.pose.heading == pytest.approx(math.pi / 2, rel=1e-4)

    def test_twist_populated(self) -> None:
        robot = _make_robot()
        tlm = TLMFrame(twist=(200, 314))
        robot._apply_tlm(tlm)
        assert robot.state.twist == (200, 314)

    def test_line_populated(self) -> None:
        robot = _make_robot()
        tlm = TLMFrame(line=(10, 20, 30, 40))
        robot._apply_tlm(tlm)
        assert robot.state.line == (10, 20, 30, 40)

    def test_color_populated(self) -> None:
        robot = _make_robot()
        tlm = TLMFrame(color=(255, 128, 0, 200))
        robot._apply_tlm(tlm)
        assert robot.state.color == (255, 128, 0, 200)

    def test_v_omega_from_twist(self) -> None:
        """v and omega are derived from twist field (mrad/s -> rad/s)."""
        robot = _make_robot()
        tlm = TLMFrame(pose=(0, 0, 0), twist=(300, 1000))
        robot._apply_tlm(tlm)
        assert robot.state.v == pytest.approx(300.0)
        assert robot.state.omega == pytest.approx(1.0, rel=1e-4)  # 1000 mrad/s = 1 rad/s


# ---------------------------------------------------------------------------
# test_apply_tlm_partial_frame
# ---------------------------------------------------------------------------

class TestApplyTlmPartialFrame:
    """_apply_tlm with a partial TLMFrame retains prior values for absent fields."""

    def test_enc_update_preserves_line_color(self) -> None:
        """After setting line/color, an enc-only frame must not wipe them."""
        robot = _make_robot()
        # First frame: set line and color
        robot._apply_tlm(TLMFrame(line=(10, 20, 30, 40), color=(1, 2, 3, 4)))
        # Second frame: only enc
        robot._apply_tlm(TLMFrame(enc=(50, 48)))
        assert robot.state.encoders == (50, 48)
        assert robot.state.line == (10, 20, 30, 40)
        assert robot.state.color == (1, 2, 3, 4)

    def test_pose_update_preserves_encoders(self) -> None:
        """After setting encoders, a pose-only frame must not wipe them."""
        robot = _make_robot()
        robot._apply_tlm(TLMFrame(enc=(77, 75)))
        robot._apply_tlm(TLMFrame(pose=(200, 100, 4500)))
        assert robot.state.encoders == (77, 75)
        assert robot.state.pose.x == pytest.approx(200.0)

    def test_absent_twist_retains_prior_v_omega(self) -> None:
        """A frame without twist= retains v/omega from the previous state."""
        robot = _make_robot()
        robot._apply_tlm(TLMFrame(pose=(0, 0, 0), twist=(500, 2000)))
        assert robot.state.v == pytest.approx(500.0)
        # Second frame: no twist
        robot._apply_tlm(TLMFrame(enc=(10, 10)))
        assert robot.state.v == pytest.approx(500.0)
        assert robot.state.omega == pytest.approx(2.0, rel=1e-4)

    def test_world_pose_preserved_across_updates(self) -> None:
        """world_pose is never touched by _apply_tlm — it stays None by default."""
        robot = _make_robot()
        robot._apply_tlm(TLMFrame(enc=(1, 1)))
        assert robot.state.world_pose is None


# ---------------------------------------------------------------------------
# test_back_compat_properties
# ---------------------------------------------------------------------------

class TestBackCompatProperties:
    """Back-compat properties return values consistent with state."""

    def test_encoders_property_matches_state(self) -> None:
        robot = _make_robot()
        robot._apply_tlm(TLMFrame(enc=(42, 41)))
        assert robot.encoders == robot.state.encoders
        assert robot.encoders == (42, 41)

    def test_otos_pose_property_matches_state_pose(self) -> None:
        robot = _make_robot()
        robot._apply_tlm(TLMFrame(pose=(300, 150, 18000)))
        x, y, h = robot.otos_pose
        assert x == pytest.approx(robot.state.pose.x)
        assert y == pytest.approx(robot.state.pose.y)
        assert h == pytest.approx(robot.state.pose.heading)

    def test_otos_pose_heading_radians(self) -> None:
        """18000 cdeg = pi radians."""
        robot = _make_robot()
        robot._apply_tlm(TLMFrame(pose=(0, 0, 18000)))
        _, _, h = robot.otos_pose
        assert h == pytest.approx(math.pi, rel=1e-5)

    def test_line_sensor_property_matches_state(self) -> None:
        robot = _make_robot()
        robot._apply_tlm(TLMFrame(line=(5, 10, 15, 20)))
        assert robot.line_sensor == robot.state.line
        assert robot.line_sensor == (5, 10, 15, 20)

    def test_color_property_matches_state(self) -> None:
        robot = _make_robot()
        robot._apply_tlm(TLMFrame(color=(100, 200, 50, 255)))
        assert robot.color == robot.state.color
        assert robot.color == (100, 200, 50, 255)

    def test_encoders_default_before_tlm(self) -> None:
        """Before any TLM frame, encoders property returns (0, 0)."""
        robot = _make_robot()
        assert robot.encoders == (0, 0)

    def test_otos_pose_default_before_tlm(self) -> None:
        """Before any TLM frame, otos_pose returns (0.0, 0.0, 0.0)."""
        robot = _make_robot()
        assert robot.otos_pose == (0.0, 0.0, 0.0)

    def test_line_sensor_default_before_tlm(self) -> None:
        """Before any TLM frame, line_sensor returns (255, 255, 255, 255)."""
        robot = _make_robot()
        assert robot.line_sensor == (255, 255, 255, 255)

    def test_color_default_before_tlm(self) -> None:
        """Before any TLM frame, color returns (0, 0, 0, 0)."""
        robot = _make_robot()
        assert robot.color == (0, 0, 0, 0)


# ---------------------------------------------------------------------------
# test_state_stamp_recent
# ---------------------------------------------------------------------------

class TestStateStampRecent:
    """state.stamp is within 1 second of time.monotonic() after _apply_tlm."""

    def test_initial_stamp_recent(self) -> None:
        """The initial state stamp (from __init__) is recent."""
        before = time.monotonic()
        robot = _make_robot()
        after = time.monotonic()
        assert before <= robot.state.stamp <= after + 0.1

    def test_stamp_updated_on_apply_tlm(self) -> None:
        """state.stamp is refreshed by each _apply_tlm call."""
        robot = _make_robot()
        before = time.monotonic()
        robot._apply_tlm(TLMFrame(enc=(1, 1)))
        after = time.monotonic()
        assert before <= robot.state.stamp <= after + 0.1

    def test_stamp_within_one_second(self) -> None:
        """state.stamp is within 1 second of time.monotonic()."""
        robot = _make_robot()
        robot._apply_tlm(TLMFrame(enc=(5, 5)))
        assert abs(robot.state.stamp - time.monotonic()) < 1.0
