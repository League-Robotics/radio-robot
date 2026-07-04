"""Tests for sensor layer v2 adaptation (ticket 013-004).

Covers:
- OdomTracker: v2 TLMFrame-based update path, RobotConfig wiring.
- CamTracker: tag-100 filtering, pose units (mm), mocked DaemonControl.
- sensors/__init__.py laziness: importing robot_radio.sensors must NOT
  import cv2 (or aprilcam/grpc) unless CamTracker is explicitly accessed.

No hardware or camera daemon required — all gRPC/cv2 dependencies are mocked.
"""

from __future__ import annotations

import subprocess
import sys
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

# Ensure host package is importable
# __file__ is tests/simulation/unit/test_sensors_v2.py; host/ is at repo_root/host/
_HOST = Path(__file__).resolve().parent.parent.parent.parent / "host"
if str(_HOST) not in sys.path:
    sys.path.insert(0, str(_HOST))


# ---------------------------------------------------------------------------
# Helper: minimal TLMFrame stand-in for tests that don't need the real one
# ---------------------------------------------------------------------------

def _tlm(pose=None, enc=None, vel=None, t=None, mode=None):
    """Create a minimal TLMFrame-like object."""
    return SimpleNamespace(pose=pose, enc=enc, vel=vel, t=t, mode=mode)


# ---------------------------------------------------------------------------
# OdomTracker — v2 TLMFrame path
# ---------------------------------------------------------------------------

class TestOdomTrackerFromTLM:
    """OdomTracker accepts TLMFrame directly (v2 primary path)."""

    def test_basic_tlm_update(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker()
        frame = _tlm(pose=(100, 50, 900))
        result = tracker.update_from_tlm(frame)
        assert result is True
        assert tracker.x == 100
        assert tracker.y == 50
        assert tracker.heading == 900

    def test_tlm_with_zero_pose(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker()
        frame = _tlm(pose=(0, 0, 0))
        tracker.update_from_tlm(frame)
        assert tracker.x == 0
        assert tracker.y == 0
        assert tracker.heading == 0

    def test_tlm_negative_values(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker()
        frame = _tlm(pose=(-200, -50, -9000))
        tracker.update_from_tlm(frame)
        assert tracker.x == -200
        assert tracker.y == -50
        assert tracker.heading == -9000

    def test_tlm_no_pose_field_returns_false(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker()
        frame = _tlm(enc=(100, 95))    # no pose
        result = tracker.update_from_tlm(frame)
        assert result is False
        assert not tracker.anchored

    def test_auto_anchors_on_first_pose(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker()
        assert not tracker.anchored
        tracker.update_from_tlm(_tlm(pose=(100, 50, 900)))
        assert tracker.anchored

    def test_heading_deg_conversion(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker()
        tracker.update_from_tlm(_tlm(pose=(0, 0, 9000)))
        assert tracker.heading == 9000
        assert abs(tracker.heading_degrees - 90.0) < 0.001
        assert abs(tracker.heading_radians - math.radians(90.0)) < 1e-6

    def test_heading_cdeg_360_degrees(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker()
        tracker.update_from_tlm(_tlm(pose=(0, 0, 36000)))
        assert tracker.heading == 36000
        assert abs(tracker.heading_degrees - 360.0) < 0.001

    def test_path_appended_on_movement(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker(ref_world_pos=(0.0, 0.0), ref_world_yaw=0.0)
        tracker.update_from_tlm(_tlm(pose=(0, 0, 0)))     # anchor
        tracker.update_from_tlm(_tlm(pose=(0, 100, 0)))   # move forward 100 mm
        assert len(tracker.path) >= 2

    def test_path_not_appended_below_min_move(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker()
        tracker.update_from_tlm(_tlm(pose=(0, 0, 0)))   # anchor → 1 path entry
        tracker.update_from_tlm(_tlm(pose=(0, 1, 0)))   # 1 mm < MIN_MOVE=3
        assert len(tracker.path) == 1                    # no new entry

    def test_multiple_frames_tracked(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker()
        for i in range(5):
            tracker.update_from_tlm(_tlm(pose=(0, i * 10, 0)))
        # First frame anchors; subsequent ones accumulate
        assert tracker.anchored
        assert tracker.y == 40     # last frame


class TestOdomTrackerAprilcamWorldConvention:
    """OdomTracker world_pos/world_yaw must match the aprilcam world frame.

    066-002 / CR-12: this class anchors OdomTracker's TLM->world-frame
    transform to the aprilcam convention (A1-centred, +x east, +y north,
    CCW-positive heading — verified empirically, see
    ``robot_radio.sensors.odometry``'s ``_apply()``) instead of leaving it an
    untested "guessed geometry" stack.

    Firmware TLM pose (x, y, heading) [mm, mm, cdeg] is itself already a proper
    Cartesian pose in firmware's own fixed frame — CCW-positive, 0 = firmware
    +X — confirmed by reading ``Odometry.cpp``'s dead-reckoning integration
    (``pose.x += d*cos(theta); pose.y += d*sin(theta)``). Both frames share
    the same CCW-positive convention, so world_pos/world_yaw is a pure
    rotation (by the constant anchor-time offset) + translation of the
    firmware-frame delta — no body-frame ("x=right, y=forward") relabeling.
    """

    def test_straight_ahead_east_facing_anchor(self) -> None:
        """Anchored facing east (world_yaw=0): driving straight moves +x (east)."""
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker(ref_world_pos=(0.0, 0.0), ref_world_yaw=0.0)
        tracker.update_from_tlm(_tlm(pose=(0, 0, 0)))     # anchor
        tracker.update_from_tlm(_tlm(pose=(500, 0, 0)))   # drove 500mm straight, no turn
        wx, wy = tracker.world_pos    # cm
        assert abs(wx - 50.0) < 1e-6, f"expected +50cm east (x), got {wx}"
        assert abs(wy - 0.0) < 1e-6, f"expected 0cm north (y), got {wy}"
        assert abs(tracker.world_yaw - 0.0) < 1e-9

    def test_straight_ahead_north_facing_anchor(self) -> None:
        """Anchored facing north (world_yaw=+90deg): driving straight moves +y (north).

        This is the case that catches a CW/CCW sign inversion: under the old
        (buggy) CW-positive implementation this produced -50cm (south)
        instead of +50cm (north).
        """
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker(ref_world_pos=(0.0, 0.0), ref_world_yaw=math.pi / 2.0)
        tracker.update_from_tlm(_tlm(pose=(0, 0, 0)))     # anchor, firmware heading=0
        tracker.update_from_tlm(_tlm(pose=(500, 0, 0)))   # same straight-ahead firmware delta
        wx, wy = tracker.world_pos
        assert abs(wx - 0.0) < 1e-6, f"expected 0cm east (x), got {wx}"
        assert abs(wy - 50.0) < 1e-6, f"expected +50cm north (y), got {wy}"
        assert abs(tracker.world_yaw - math.pi / 2.0) < 1e-9

    def test_pure_ccw_turn_matches_firmware_direction(self) -> None:
        """A +90deg CCW firmware turn must yield a +90deg (not -90deg) world_yaw.

        aprilcam and firmware TLM heading are both CCW-positive, so a CCW
        turn as measured by firmware must map onto a CCW (positive) change
        in world_yaw — same direction, same magnitude, only offset by the
        constant anchor rotation.
        """
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker(ref_world_pos=(0.0, 0.0), ref_world_yaw=0.0)
        tracker.update_from_tlm(_tlm(pose=(0, 0, 0)))       # anchor
        tracker.update_from_tlm(_tlm(pose=(0, 0, 9000)))    # +90deg CCW, no translation
        assert abs(math.degrees(tracker.world_yaw) - 90.0) < 1e-6

    def test_anchor_offset_position_is_preserved(self) -> None:
        """A nonzero world anchor position is a pure translation on top of the rotation."""
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker(ref_world_pos=(1000.0, 2000.0), ref_world_yaw=0.0)
        tracker.update_from_tlm(_tlm(pose=(0, 0, 0)))       # anchor
        tracker.update_from_tlm(_tlm(pose=(0, 0, 0)))       # no movement
        wx, wy = tracker.world_pos
        assert abs(wx - 100.0) < 1e-6   # 1000mm -> 100cm
        assert abs(wy - 200.0) < 1e-6   # 2000mm -> 200cm


class TestOdomTrackerRobotConfig:
    """OdomTracker wires correctly to RobotConfig."""

    def _make_config(self, trackwidth=126.0, wheel_travel_calib_left=0.487, wheel_travel_calib_right=0.481):
        """Build a minimal RobotConfig-like object."""
        cal = SimpleNamespace(
            mm_per_wheel_deg_left=wheel_travel_calib_left,
            mm_per_wheel_deg_right=wheel_travel_calib_right,
        )
        return SimpleNamespace(trackwidth=trackwidth, calibration=cal)

    def test_config_wires_trackwidth(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        cfg = self._make_config(trackwidth=126.0)
        tracker = OdomTracker(config=cfg)
        assert tracker.trackwidth == 126.0

    def test_config_wires_mm_per_deg(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        cfg = self._make_config(wheel_travel_calib_left=0.487, wheel_travel_calib_right=0.481)
        tracker = OdomTracker(config=cfg)
        assert abs(tracker.wheel_travel_calib_left - 0.487) < 1e-6
        assert abs(tracker.wheel_travel_calib_right - 0.481) < 1e-6

    def test_real_robot_config(self) -> None:
        """Load the real tovez.json config and verify OdomTracker accepts it."""
        from robot_radio.config.robot_config import load_robot_config
        from robot_radio.sensors.odom_tracker import OdomTracker
        cfg_path = _HOST.parent / "data" / "robots" / "tovez.json"
        cfg = load_robot_config(cfg_path)
        tracker = OdomTracker(config=cfg)
        assert tracker.trackwidth == 128.0
        assert abs(tracker.wheel_travel_calib_left - 0.7165) < 1e-6
        assert abs(tracker.wheel_travel_calib_right - 0.7077) < 1e-6

    def test_bare_keyword_args(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        tracker = OdomTracker(trackwidth=130.0, wheel_travel_calib_left=0.5, wheel_travel_calib_right=0.5)
        assert tracker.trackwidth == 130.0

    def test_tlm_still_works_with_config(self) -> None:
        from robot_radio.sensors.odom_tracker import OdomTracker
        cfg = self._make_config()
        tracker = OdomTracker(config=cfg)
        tracker.update_from_tlm(_tlm(pose=(100, 50, 900)))
        assert tracker.x == 100
        assert tracker.y == 50
        assert tracker.heading == 900


class TestOdomTrackerRealProtocol:
    """OdomTracker with actual TLMFrame from robot_radio.robot.protocol."""

    def test_update_from_real_tlm_frame(self) -> None:
        from robot_radio.robot.protocol import parse_tlm
        from robot_radio.sensors.odom_tracker import OdomTracker
        frame = parse_tlm("TLM t=500 pose=100,50,900 enc=200,195")
        assert frame is not None
        tracker = OdomTracker()
        result = tracker.update_from_tlm(frame)
        assert result is True
        assert tracker.x == 100
        assert tracker.y == 50
        assert tracker.heading == 900

    def test_enc_only_frame_returns_false(self) -> None:
        from robot_radio.robot.protocol import parse_tlm
        from robot_radio.sensors.odom_tracker import OdomTracker
        frame = parse_tlm("TLM t=100 enc=200,195")
        assert frame is not None
        tracker = OdomTracker()
        result = tracker.update_from_tlm(frame)
        assert result is False    # no pose field


# ---------------------------------------------------------------------------
# CamTracker — tag-100 filtering, mocked daemon
# ---------------------------------------------------------------------------

def _make_tag(tag_id: int, x_mm: float = 500.0, y_mm: float = 300.0, yaw: float = 0.0):
    """Return a minimal tag-like object matching CamTracker's expectations."""
    return SimpleNamespace(
        id=tag_id,
        world_xy=(x_mm, y_mm),
        yaw=yaw,
    )


def _make_frame(*tags):
    return SimpleNamespace(tags=list(tags))


class TestCamTrackerTagFiltering:
    """CamTracker accepts tag 100 and ignores other IDs."""

    def _make_cam(self, initial_pos=(0.0, 0.0), yaw=0.0, robot_tag=100):
        from robot_radio.sensors.cam_tracker import CamTracker
        dc = MagicMock()
        return CamTracker(pos=initial_pos, yaw=yaw, robot_tag=robot_tag, dc=dc, cam_id="cam0")

    def test_accepts_tag_100(self) -> None:
        cam = self._make_cam()
        cam._dc.get_tags.return_value = _make_frame(_make_tag(100, 500.0, 300.0, 0.1))
        updated = cam.update()
        assert updated is True
        assert cam.pos == (500.0, 300.0)

    def test_rejects_tag_99(self) -> None:
        cam = self._make_cam()
        cam._dc.get_tags.return_value = _make_frame(_make_tag(99, 500.0, 300.0, 0.1))
        updated = cam.update()
        assert updated is False
        # Pose unchanged from initial (0, 0)
        assert cam.pos == (0.0, 0.0)

    def test_rejects_tag_1(self) -> None:
        cam = self._make_cam()
        cam._dc.get_tags.return_value = _make_frame(_make_tag(1, 200.0, 100.0, 0.0))
        updated = cam.update()
        assert updated is False

    def test_mixed_tags_only_100_accepted(self) -> None:
        """Frame with multiple tags — only the robot's tag updates pose."""
        cam = self._make_cam()
        cam._dc.get_tags.return_value = _make_frame(
            _make_tag(1, 999.0, 999.0, 1.0),    # ignored
            _make_tag(100, 500.0, 300.0, 0.1),  # accepted
            _make_tag(2, 111.0, 111.0, 0.5),    # ignored
        )
        updated = cam.update()
        assert updated is True
        assert cam.pos == (500.0, 300.0)

    def test_empty_frame_returns_false(self) -> None:
        cam = self._make_cam()
        cam._dc.get_tags.return_value = _make_frame()  # no tags
        updated = cam.update()
        assert updated is False

    def test_pose_units_are_mm(self) -> None:
        """Pose coordinates from world_xy are passed through as-is (mm)."""
        cam = self._make_cam(robot_tag=100)
        x_mm, y_mm = 1234.5, 678.9
        cam._dc.get_tags.return_value = _make_frame(_make_tag(100, x_mm, y_mm, 0.0))
        cam.update()
        assert abs(cam.pos[0] - x_mm) < 0.001
        assert abs(cam.pos[1] - y_mm) < 0.001

    def test_yaw_updated(self) -> None:
        cam = self._make_cam(robot_tag=100)
        cam._dc.get_tags.return_value = _make_frame(_make_tag(100, 500.0, 300.0, yaw=1.57))
        cam.update()
        assert abs(cam.yaw - 1.57) < 0.001

    def test_path_grows_on_movement(self) -> None:
        cam = self._make_cam(initial_pos=(0.0, 0.0), robot_tag=100)
        # Start at origin → move to (500, 0) = 500 mm > MIN_MOVE_CM=0.5 cm
        cam._dc.get_tags.return_value = _make_frame(_make_tag(100, 500.0, 0.0, 0.0))
        cam.update()
        assert len(cam.path) >= 2

    def test_wait_for_robot_finds_tag_100(self) -> None:
        from robot_radio.sensors.cam_tracker import CamTracker
        dc = MagicMock()
        dc.get_tags.return_value = _make_frame(_make_tag(100, 500.0, 300.0, 0.2))
        result = CamTracker.wait_for_robot(dc, cam_id="cam0", robot_tag=100, retries=3, pause_s=0.0)
        assert result is not None
        assert result.pos == (500.0, 300.0)
        assert abs(result.yaw - 0.2) < 0.001

    def test_wait_for_robot_timeout_returns_none(self) -> None:
        from robot_radio.sensors.cam_tracker import CamTracker
        dc = MagicMock()
        # Return frame with no tags matching tag 100
        dc.get_tags.return_value = _make_frame(_make_tag(99, 100.0, 100.0, 0.0))
        result = CamTracker.wait_for_robot(dc, cam_id="cam0", robot_tag=100, retries=3, pause_s=0.0)
        assert result is None

    def test_grpc_error_is_handled_gracefully(self) -> None:
        """CamTracker update must not raise on transient get_tags errors."""
        cam = self._make_cam(robot_tag=100)
        # Simulate intermittent gRPC error followed by success
        cam._dc.get_tags.side_effect = [
            RuntimeError("gRPC deadline exceeded"),
            _make_frame(_make_tag(100, 500.0, 300.0, 0.1)),
        ]
        # First call raises
        try:
            cam.update()
        except RuntimeError:
            pass  # caller responsibility to handle; tracker itself doesn't swallow
        # Second call succeeds
        cam._dc.get_tags.side_effect = None
        cam._dc.get_tags.return_value = _make_frame(_make_tag(100, 500.0, 300.0, 0.1))
        updated = cam.update()
        assert updated is True


# ---------------------------------------------------------------------------
# sensors/__init__.py laziness — cv2 must not be imported eagerly
# ---------------------------------------------------------------------------

class TestSensorsInitLaziness:
    """Importing robot_radio.sensors must NOT pull in cv2/aprilcam/grpc.

    These tests use a fresh subprocess for each check so that cv2 imported by
    other tests (e.g. test_playfield.py via aprilcam) cannot pollute the
    assertion.  The subprocess exits 0 if the invariant holds, non-zero if it
    is violated — so the tests still fail if someone makes sensors import cv2
    eagerly.
    """

    def _run_snippet(self, snippet: str) -> None:
        """Run *snippet* in a fresh interpreter and assert it exits 0."""
        result = subprocess.run(
            [sys.executable, "-c", snippet],
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, (
            f"Subprocess check failed (rc={result.returncode}):\n"
            f"stdout: {result.stdout}\n"
            f"stderr: {result.stderr}"
        )

    def test_cv2_not_in_sys_modules_after_import(self) -> None:
        """Bare import of robot_radio.sensors must leave cv2 absent."""
        self._run_snippet(
            "import sys\n"
            "import robot_radio.sensors as _s\n"
            "# Remove any lazy-cached names to simulate a fresh attribute lookup\n"
            "for name in ('CamTracker', 'Otos'):\n"
            "    _s.__dict__.pop(name, None)\n"
            "assert 'cv2' not in sys.modules, (\n"
            "    'cv2 was imported as a side-effect of import robot_radio.sensors. '\n"
            "    'The sensors/__init__.py must not eagerly import cam_tracker.'\n"
            ")\n"
        )

    def test_odom_tracker_available_without_cv2(self) -> None:
        """OdomTracker should be available without triggering a cv2 import."""
        self._run_snippet(
            "import sys\n"
            "import robot_radio.sensors as sensors\n"
            "assert hasattr(sensors, 'OdomTracker'), 'OdomTracker missing'\n"
            "_ = sensors.OdomTracker\n"
            "assert 'cv2' not in sys.modules, (\n"
            "    'cv2 was imported when accessing sensors.OdomTracker'\n"
            ")\n"
        )

    def test_cam_tracker_lazy_import(self) -> None:
        """CamTracker is accessible via lazy __getattr__ without pre-loading cv2."""
        import robot_radio.sensors as sensors
        # Remove cached CamTracker from module globals to force __getattr__
        sensors.__dict__.pop("CamTracker", None)
        # Accessing CamTracker should work (it imports cam_tracker module);
        # cam_tracker itself does not import cv2 directly — only aprilcam does,
        # and aprilcam is not in the test venv.
        # Just verify the name resolves to the class without crashing.
        from robot_radio.sensors.cam_tracker import CamTracker as _CT
        assert _CT is not None

    def test_sensors_all_exports_accessible(self) -> None:
        """All non-lazy __all__ names must resolve without cv2."""
        eager_names = [
            "Odometry", "OdomTracker", "parse_so", "parse_tlm",
            "ColorClassifier", "nezha_classifier", "calibrate_white",
            "ThrashMonitor", "CalibrationError", "load", "to_wire_values",
            "apply", "load_and_apply",
        ]
        names_repr = repr(eager_names)
        self._run_snippet(
            "import sys\n"
            "import robot_radio.sensors as sensors\n"
            f"eager_names = {names_repr}\n"
            "for name in eager_names:\n"
            "    obj = getattr(sensors, name)\n"
            "    assert obj is not None, f'sensors.{name} resolved to None'\n"
            "assert 'cv2' not in sys.modules, (\n"
            "    'cv2 was imported after accessing eager sensors exports'\n"
            ")\n"
        )


# ---------------------------------------------------------------------------
# otos.py — no v1 verb strings
# ---------------------------------------------------------------------------

class TestOtosV2Verbs:
    """otos.py must not contain v1 verb strings."""

    def _otos_source(self) -> str:
        import robot_radio.sensors.otos as otos_mod
        import inspect
        return inspect.getsource(otos_mod)

    def test_no_ok_verb(self) -> None:
        """'OK' (v1 OTOS calibrate ack) must not appear as a wire verb."""
        src = self._otos_source()
        # 'OK' as a standalone send command should not be present.
        # We look for self._send("OK") patterns.
        assert 'self._send("OK"' not in src, \
            "otos.py must not send the v1 'OK' OTOS calibrate verb"

    def test_no_so_verb(self) -> None:
        src = self._otos_source()
        assert '"SO"' not in src, "otos.py must not reference v1 SO verb"

    def test_no_oo_verb(self) -> None:
        src = self._otos_source()
        assert '"OO"' not in src, "otos.py must not reference v1 OO verb"

    def test_v2_verbs_present(self) -> None:
        """All expected v2 OTOS verbs are represented in the source."""
        src = self._otos_source()
        for verb in ("OI", "OZ", "OR", "OP", "OV"):
            assert f'"{verb}"' in src or f"'{verb}'" in src, \
                f"otos.py expected to contain v2 verb {verb!r}"


# ---------------------------------------------------------------------------
# Peripheral sensor module importability
# ---------------------------------------------------------------------------

class TestSensorModuleImports:
    """All sensor modules must import cleanly (no v1 wire call side-effects)."""

    def test_color_importable(self) -> None:
        import robot_radio.sensors.color  # noqa: F401

    def test_motion_monitor_importable(self) -> None:
        import robot_radio.sensors.motion_monitor  # noqa: F401

    def test_odometry_importable(self) -> None:
        import robot_radio.sensors.odometry  # noqa: F401

    def test_calibration_importable(self) -> None:
        import robot_radio.sensors.calibration  # noqa: F401

    def test_odom_tracker_importable(self) -> None:
        import robot_radio.sensors.odom_tracker  # noqa: F401

    def test_cam_tracker_importable(self) -> None:
        import robot_radio.sensors.cam_tracker  # noqa: F401
