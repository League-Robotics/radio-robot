"""src/tests/unit/test_pose_fix_convergence_pure.py -- 099-009 (SUC-005).

Exercises ONLY the pure-math helpers in ``src/tests/playfield/
pose_fix_convergence.py`` (the aprilcam end-to-end bench/playfield script
for the sprint's delayed camera-fix capability) -- no serial port, no
camera daemon, no robot. Covers:

- ``wrap_deg()`` -- angle wrapping to (-180, 180].
- ``camera_pose_to_pose_fix_kwargs()`` -- aprilcam world pose (cm, cm, rad)
  -> ``PoseFix`` wire kwargs (mm, mm, rad, ms).
- ``pose_fix_target_mm_cdeg()`` -- aprilcam world pose -> the SAME
  (x_mm, y_mm, h_cdeg) shape ``TLMFrame.pose`` reports.
- ``pose_error()`` -- Euclidean position error (mm) + wrapped heading error
  (deg) between two ``TLMFrame.pose``-shaped 3-tuples.
- ``pose_converged()`` -- tolerance check.
- ``geofence_from_playfield()`` / ``in_fence()`` -- ArUco-corner-derived
  geofence math (ported from ``tests_old/bench/world_goto_chart.py``, logic
  unchanged).

``src/tests/playfield/`` itself is NOT pytest-collected (``pyproject.toml``'s
``testpaths`` = ``tests/sim``, ``tests/unit``, ``tests/testgui`` only,
per ``tests/CLAUDE.md``'s three-domain split -- ``src/tests/playfield/`` is HITL
CLI tooling, not a pytest suite) -- this file lives under ``src/tests/unit/``
and reaches the script module by inserting ``tests/playfield`` onto
``sys.path`` directly, the SAME pattern ``src/tests/sim/unit/
test_pose_fix_end_to_end.py`` uses for its own sibling ``src/sim``
import. Importing the script module touches no hardware -- every
camera/robot call in it lives inside a function, never at module scope
(verified separately: ``uv run python -c "import ast; ast.parse(...)"`` and
a bare ``import pose_fix_convergence``, both hardware-free).
"""

from __future__ import annotations

import math
import pathlib
import sys

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_PLAYFIELD_DIR = _REPO_ROOT / "src" / "tests" / "playfield"
if str(_PLAYFIELD_DIR) not in sys.path:
    sys.path.insert(0, str(_PLAYFIELD_DIR))

import pose_fix_convergence as pfc  # noqa: E402


# ---------------------------------------------------------------------------
# wrap_deg()
# ---------------------------------------------------------------------------

class TestWrapDeg:
    def test_already_in_range(self) -> None:
        assert pfc.wrap_deg(45.0) == pytest.approx(45.0)

    def test_wraps_above_180(self) -> None:
        assert pfc.wrap_deg(270.0) == pytest.approx(-90.0)

    def test_wraps_below_negative_180(self) -> None:
        assert pfc.wrap_deg(-270.0) == pytest.approx(90.0)

    def test_exact_180_wraps_to_negative_180(self) -> None:
        # [-180, 180) convention (matches playfield_camera_run.py's wrap()).
        assert pfc.wrap_deg(180.0) == pytest.approx(-180.0)

    def test_zero(self) -> None:
        assert pfc.wrap_deg(0.0) == pytest.approx(0.0)

    def test_full_turn_wraps_to_zero(self) -> None:
        assert pfc.wrap_deg(360.0) == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# camera_pose_to_pose_fix_kwargs()
# ---------------------------------------------------------------------------

class TestCameraPoseToPoseFixKwargs:
    def test_scales_cm_to_mm(self) -> None:
        kwargs = pfc.camera_pose_to_pose_fix_kwargs(12.5, -30.0, 0.0, 1000.0)
        assert kwargs["x"] == pytest.approx(125.0)
        assert kwargs["y"] == pytest.approx(-300.0)

    def test_heading_radians_unchanged(self) -> None:
        kwargs = pfc.camera_pose_to_pose_fix_kwargs(0.0, 0.0, 1.2345, 0.0)
        assert kwargs["h"] == pytest.approx(1.2345)

    def test_t_rounded_to_int(self) -> None:
        kwargs = pfc.camera_pose_to_pose_fix_kwargs(0.0, 0.0, 0.0, 1234.6)
        assert kwargs["t"] == 1235
        assert isinstance(kwargs["t"], int)

    def test_t_rounds_down_below_half(self) -> None:
        kwargs = pfc.camera_pose_to_pose_fix_kwargs(0.0, 0.0, 0.0, 1234.4)
        assert kwargs["t"] == 1234

    def test_zero_pose(self) -> None:
        kwargs = pfc.camera_pose_to_pose_fix_kwargs(0.0, 0.0, 0.0, 0.0)
        assert kwargs == {"x": 0.0, "y": 0.0, "h": 0.0, "t": 0}

    def test_negative_values(self) -> None:
        kwargs = pfc.camera_pose_to_pose_fix_kwargs(-5.0, -10.0, -math.pi / 2, 500.0)
        assert kwargs["x"] == pytest.approx(-50.0)
        assert kwargs["y"] == pytest.approx(-100.0)
        assert kwargs["h"] == pytest.approx(-math.pi / 2)


# ---------------------------------------------------------------------------
# pose_fix_target_mm_cdeg()
# ---------------------------------------------------------------------------

class TestPoseFixTargetMmCdeg:
    def test_scales_and_converts_heading(self) -> None:
        target = pfc.pose_fix_target_mm_cdeg(10.0, 20.0, math.pi / 2)
        x_mm, y_mm, h_cdeg = target
        assert x_mm == pytest.approx(100.0)
        assert y_mm == pytest.approx(200.0)
        assert h_cdeg == pytest.approx(9000.0)  # 90 deg * 100

    def test_zero_heading(self) -> None:
        _x, _y, h_cdeg = pfc.pose_fix_target_mm_cdeg(0.0, 0.0, 0.0)
        assert h_cdeg == pytest.approx(0.0)

    def test_negative_heading(self) -> None:
        _x, _y, h_cdeg = pfc.pose_fix_target_mm_cdeg(0.0, 0.0, -math.pi)
        assert h_cdeg == pytest.approx(-18000.0)


# ---------------------------------------------------------------------------
# pose_error()
# ---------------------------------------------------------------------------

class TestPoseError:
    def test_identical_poses_zero_error(self) -> None:
        target = (100.0, 200.0, 4500.0)
        distance, heading_error = pfc.pose_error(target, target)
        assert distance == pytest.approx(0.0)
        assert heading_error == pytest.approx(0.0)

    def test_position_error_pythagorean(self) -> None:
        observed = (0.0, 0.0, 0.0)
        target = (30.0, 40.0, 0.0)   # 3-4-5 triangle -> 50mm
        distance, _ = pfc.pose_error(observed, target)
        assert distance == pytest.approx(50.0)

    def test_heading_error_simple(self) -> None:
        observed = (0.0, 0.0, 1000.0)   # 10 deg
        target = (0.0, 0.0, 2500.0)     # 25 deg
        _, heading_error = pfc.pose_error(observed, target)
        assert heading_error == pytest.approx(15.0)

    def test_heading_error_wraps_across_180(self) -> None:
        observed = (0.0, 0.0, 17900.0)   # 179 deg
        target = (0.0, 0.0, -17900.0)    # -179 deg
        _, heading_error = pfc.pose_error(observed, target)
        # True angular separation is 2 deg, not 358 deg.
        assert heading_error == pytest.approx(2.0)

    def test_heading_error_always_nonnegative(self) -> None:
        observed = (0.0, 0.0, -9000.0)
        target = (0.0, 0.0, 9000.0)
        _, heading_error = pfc.pose_error(observed, target)
        assert heading_error >= 0.0


# ---------------------------------------------------------------------------
# pose_converged()
# ---------------------------------------------------------------------------

class TestPoseConverged:
    def test_within_both_tolerances(self) -> None:
        assert pfc.pose_converged(10.0, 1.0, tol_pos=30.0, tol_heading=3.0) is True

    def test_exactly_at_tolerance_boundary(self) -> None:
        assert pfc.pose_converged(30.0, 3.0, tol_pos=30.0, tol_heading=3.0) is True

    def test_position_exceeds_tolerance(self) -> None:
        assert pfc.pose_converged(31.0, 1.0, tol_pos=30.0, tol_heading=3.0) is False

    def test_heading_exceeds_tolerance(self) -> None:
        assert pfc.pose_converged(10.0, 3.1, tol_pos=30.0, tol_heading=3.0) is False

    def test_both_exceed_tolerance(self) -> None:
        assert pfc.pose_converged(100.0, 45.0, tol_pos=30.0, tol_heading=3.0) is False


# ---------------------------------------------------------------------------
# geofence_from_playfield() / in_fence()
# ---------------------------------------------------------------------------

_PLAYFIELD = {
    "aruco_tags": [
        {"x": -67.0, "y": -44.65},
        {"x": 67.0, "y": -44.65},
        {"x": 67.0, "y": 44.65},
        {"x": -67.0, "y": 44.65},
    ],
}


class TestGeofenceFromPlayfield:
    def test_insets_by_margin(self) -> None:
        fence = pfc.geofence_from_playfield(_PLAYFIELD, margin=14.0)
        assert fence == pytest.approx((-53.0, 53.0, -30.65, 30.65))

    def test_zero_margin_matches_extent(self) -> None:
        fence = pfc.geofence_from_playfield(_PLAYFIELD, margin=0.0)
        assert fence == pytest.approx((-67.0, 67.0, -44.65, 44.65))

    def test_no_aruco_tags_raises(self) -> None:
        with pytest.raises(ValueError):
            pfc.geofence_from_playfield({"aruco_tags": []}, margin=14.0)

    def test_missing_aruco_tags_key_raises(self) -> None:
        with pytest.raises(ValueError):
            pfc.geofence_from_playfield({}, margin=14.0)


class TestInFence:
    def test_origin_is_inside(self) -> None:
        fence = pfc.geofence_from_playfield(_PLAYFIELD, margin=14.0)
        assert pfc.in_fence(0.0, 0.0, fence) is True

    def test_beyond_x_is_outside(self) -> None:
        fence = pfc.geofence_from_playfield(_PLAYFIELD, margin=14.0)
        assert pfc.in_fence(60.0, 0.0, fence) is False

    def test_beyond_y_is_outside(self) -> None:
        fence = pfc.geofence_from_playfield(_PLAYFIELD, margin=14.0)
        assert pfc.in_fence(0.0, 35.0, fence) is False

    def test_exactly_on_boundary_is_inside(self) -> None:
        fence = pfc.geofence_from_playfield(_PLAYFIELD, margin=14.0)
        x_lo, _x_hi, _y_lo, _y_hi = fence
        assert pfc.in_fence(x_lo, 0.0, fence) is True
