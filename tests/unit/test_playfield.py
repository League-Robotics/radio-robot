"""Tests for robot_radio.field.Playfield — tags, objects, pixel-world, paths.

All tests mock DaemonControl — no live daemon or camera required.

Test plan (8 cases from the ticket):
1. test_get_tag_found
2. test_get_tag_absent
3. test_get_tag_no_world_xy
4. test_get_object_from_fixture
5. test_world_to_pixel_round_trip  (includes explicit y-flip + origin assertion)
6. test_add_path_publishes_overlay
7. test_clear_paths_publishes_overlay
8. test_add_path_publishes_via_grpc
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from robot_radio.field.playfield import Feature, Playfield, Tag


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# A simple non-trivial 3×3 homography (roughly: 1 cm = 10 pixels, no shear).
# H maps [u, v, 1] pixel → [raw_x, raw_y, 1] cm (approx).
# We use a known invertible matrix so we can assert round-trip accuracy.
_H_DATA = [
    [0.1, 0.0, 5.0],
    [0.0, 0.1, 3.0],
    [0.0, 0.0, 1.0],
]


def _make_tag_frame(
    tags: list | None = None,
    homography: list[list[float]] | None = None,
) -> MagicMock:
    """Build a mock TagFrame."""
    tf = MagicMock()
    tf.tags = tags or []
    tf.homography = homography
    # Implement by_id as a real lookup.
    tf.by_id = lambda tag_id: next(
        (t for t in tf.tags if t.id == tag_id), None
    )
    return tf


def _make_tag_record(
    tag_id: int,
    world_xy: tuple[float, float] | None,
    yaw: float = 0.0,
) -> MagicMock:
    """Build a mock TagRecord."""
    rec = MagicMock()
    rec.id = tag_id
    rec.world_xy = world_xy
    rec.yaw = yaw
    return rec


def _make_dc(tag_frame: MagicMock | None = None) -> MagicMock:
    """Build a mock DaemonControl."""
    dc = MagicMock()
    if tag_frame is not None:
        dc.get_tags.return_value = tag_frame
    dc.where_is.return_value = {"status": "not_found", "matches": []}
    dc.close.return_value = None
    return dc


def _make_playfield(
    dc: MagicMock,
    tmp_path: Path,
    *,
    playfield_json: Path | None = None,
) -> Playfield:
    """Build a Playfield with a temp paths.json directory."""
    paths_json = tmp_path / "paths.json"
    return Playfield._from_parts(
        dc,
        "test-cam",
        paths_json=paths_json,
        playfield_json=playfield_json,
    )


def _inject_homography(pf: Playfield, h_data: list[list[float]]) -> None:
    """Directly inject a homography matrix into a Playfield (bypasses daemon)."""
    H = np.array(h_data, dtype=float)
    pf._H = H
    pf._H_inv = np.linalg.inv(H)


# ---------------------------------------------------------------------------
# 1. test_get_tag_found
# ---------------------------------------------------------------------------


class TestGetTagFound:
    """get_tag returns a Tag with correct x, y, yaw from a mocked TagRecord."""

    def test_returns_tag_dataclass(self, tmp_path: Path) -> None:
        rec = _make_tag_record(100, world_xy=(12.5, -3.0), yaw=0.785)
        tf = _make_tag_frame(tags=[rec])
        dc = _make_dc(tag_frame=tf)

        pf = _make_playfield(dc, tmp_path)
        tag = pf.get_tag(100)

        assert tag is not None
        assert isinstance(tag, Tag)
        assert tag.id == 100
        assert tag.x == pytest.approx(12.5)
        assert tag.y == pytest.approx(-3.0)
        assert tag.yaw == pytest.approx(0.785)

    def test_get_tags_called_with_cam_name(self, tmp_path: Path) -> None:
        rec = _make_tag_record(5, world_xy=(1.0, 2.0))
        tf = _make_tag_frame(tags=[rec])
        dc = _make_dc(tag_frame=tf)

        pf = _make_playfield(dc, tmp_path)
        pf.get_tag(5)

        dc.get_tags.assert_called_with("test-cam")


# ---------------------------------------------------------------------------
# 2. test_get_tag_absent
# ---------------------------------------------------------------------------


class TestGetTagAbsent:
    """get_tag returns None when the tag id is not in the frame."""

    def test_returns_none_when_id_missing(self, tmp_path: Path) -> None:
        tf = _make_tag_frame(tags=[])
        dc = _make_dc(tag_frame=tf)

        pf = _make_playfield(dc, tmp_path)
        assert pf.get_tag(999) is None

    def test_returns_none_when_different_id_present(self, tmp_path: Path) -> None:
        rec = _make_tag_record(1, world_xy=(0.0, 0.0))
        tf = _make_tag_frame(tags=[rec])
        dc = _make_dc(tag_frame=tf)

        pf = _make_playfield(dc, tmp_path)
        assert pf.get_tag(42) is None


# ---------------------------------------------------------------------------
# 3. test_get_tag_no_world_xy
# ---------------------------------------------------------------------------


class TestGetTagNoWorldXy:
    """get_tag returns None when world_xy is None (uncalibrated playfield)."""

    def test_returns_none_when_world_xy_none(self, tmp_path: Path) -> None:
        rec = _make_tag_record(7, world_xy=None, yaw=1.0)
        tf = _make_tag_frame(tags=[rec])
        dc = _make_dc(tag_frame=tf)

        pf = _make_playfield(dc, tmp_path)
        assert pf.get_tag(7) is None


# ---------------------------------------------------------------------------
# 4. test_get_object_from_fixture
# ---------------------------------------------------------------------------

_FIXTURE_PLAYFIELD = {
    "playfield": {"width_cm": 100.0, "height_cm": 80.0, "origin": "apriltag-center-a1"},
    "rectangles": [
        {
            "slug": "rect-northwest-purple",
            "type": "rectangle",
            "color": "purple",
            "cardinal": "northwest",
            "x": -35.0,
            "y": 24.0,
        }
    ],
    "dots": [],
    "april_tags": [],
    "aruco_tags": [],
}


class TestGetObjectFromFixture:
    """get_object falls back to playfield.json scanning when where_is fails."""

    @pytest.fixture
    def fixture_pf_path(self, tmp_path: Path) -> Path:
        path = tmp_path / "playfield.json"
        path.write_text(json.dumps(_FIXTURE_PLAYFIELD))
        return path

    def test_returns_feature_by_slug(
        self, tmp_path: Path, fixture_pf_path: Path
    ) -> None:
        dc = _make_dc()
        # where_is returns not_found so fallback scans playfield.json.
        dc.where_is.return_value = {"status": "not_found", "matches": []}

        pf = _make_playfield(dc, tmp_path, playfield_json=fixture_pf_path)
        feat = pf.get_object("rect-northwest-purple")

        assert feat is not None
        assert isinstance(feat, Feature)
        assert feat.slug == "rect-northwest-purple"
        assert feat.type == "rectangle"
        assert feat.color == "purple"
        assert feat.x == pytest.approx(-35.0)
        assert feat.y == pytest.approx(24.0)

    def test_returns_none_for_unknown_slug(
        self, tmp_path: Path, fixture_pf_path: Path
    ) -> None:
        dc = _make_dc()
        dc.where_is.return_value = {"status": "not_found", "matches": []}

        pf = _make_playfield(dc, tmp_path, playfield_json=fixture_pf_path)
        assert pf.get_object("does-not-exist") is None

    def test_where_is_ok_takes_precedence(
        self, tmp_path: Path, fixture_pf_path: Path
    ) -> None:
        dc = _make_dc()
        dc.where_is.return_value = {
            "status": "ok",
            "matches": [
                {
                    "slug": "rect-northwest-purple",
                    "type": "rectangle",
                    "location": {"x": -35.0, "y": 24.0},
                    "record": {"color": "purple"},
                }
            ],
        }

        pf = _make_playfield(dc, tmp_path, playfield_json=fixture_pf_path)
        feat = pf.get_object("purple rectangle")

        assert feat is not None
        assert feat.x == pytest.approx(-35.0)
        assert feat.y == pytest.approx(24.0)
        assert feat.color == "purple"


# ---------------------------------------------------------------------------
# 5. test_world_to_pixel_round_trip
# ---------------------------------------------------------------------------


class TestPixelWorldTransform:
    """world_to_pixel / pixel_to_world with known H and explicit transform check."""

    @pytest.fixture
    def pf_with_h(self, tmp_path: Path) -> Playfield:
        dc = _make_dc()
        pf = _make_playfield(dc, tmp_path)
        pf._origin_x = 50.0
        pf._origin_y = 40.0
        _inject_homography(pf, _H_DATA)
        return pf

    def test_round_trip_within_tolerance(self, pf_with_h: Playfield) -> None:
        x0, y0 = 10.0, 5.0
        pixel = pf_with_h.world_to_pixel(x0, y0)
        assert pixel is not None
        world_back = pf_with_h.pixel_to_world(*pixel)
        assert world_back is not None
        assert world_back[0] == pytest.approx(x0, abs=1e-6)
        assert world_back[1] == pytest.approx(y0, abs=1e-6)

    def test_world_to_pixel_uses_y_flip_and_origin_offset(
        self, pf_with_h: Playfield
    ) -> None:
        """Verify that world_to_pixel applies [x+ox, oy-y, 1], not [x, y, 1].

        We compute the pixel expected from the CORRECT transform and from the
        naive (no offset / no flip) transform, then assert the result matches
        the correct one and NOT the naive one.
        """
        H_inv = np.linalg.inv(np.array(_H_DATA, dtype=float))
        ox, oy = 50.0, 40.0
        x, y = 10.0, 5.0

        # Correct transform: raw = [x + ox, oy - y, 1]
        raw_correct = np.array([x + ox, oy - y, 1.0])
        px_h_correct = H_inv @ raw_correct
        u_correct = px_h_correct[0] / px_h_correct[2]
        v_correct = px_h_correct[1] / px_h_correct[2]

        # Naive (wrong) transform: raw = [x, y, 1]
        raw_naive = np.array([x, y, 1.0])
        px_h_naive = H_inv @ raw_naive
        u_naive = px_h_naive[0] / px_h_naive[2]
        v_naive = px_h_naive[1] / px_h_naive[2]

        actual = pf_with_h.world_to_pixel(x, y)
        assert actual is not None

        # Must match the correct transform.
        assert actual[0] == pytest.approx(u_correct, abs=1e-10)
        assert actual[1] == pytest.approx(v_correct, abs=1e-10)

        # Must NOT match the naive transform (sanity-check the test itself).
        assert abs(u_correct - u_naive) > 1.0 or abs(v_correct - v_naive) > 1.0, (
            "The correct and naive transforms produced identical results — "
            "the test fixture does not distinguish the two code paths."
        )

    def test_returns_none_without_homography(self, tmp_path: Path) -> None:
        dc = _make_dc()
        pf = _make_playfield(dc, tmp_path)
        # No homography injected.
        assert pf.world_to_pixel(0.0, 0.0) is None
        assert pf.pixel_to_world(0.0, 0.0) is None

    def test_round_trip_negative_coords(self, pf_with_h: Playfield) -> None:
        """Negative world coordinates (south-west) also round-trip cleanly."""
        x0, y0 = -20.0, -15.0
        pixel = pf_with_h.world_to_pixel(x0, y0)
        assert pixel is not None
        world_back = pf_with_h.pixel_to_world(*pixel)
        assert world_back is not None
        assert world_back[0] == pytest.approx(x0, abs=1e-6)
        assert world_back[1] == pytest.approx(y0, abs=1e-6)

    def test_homography_updated_from_tag_frame(self, tmp_path: Path) -> None:
        """Homography is cached when a tag frame carries a non-None homography."""
        tf = _make_tag_frame(tags=[], homography=_H_DATA)
        dc = _make_dc(tag_frame=tf)

        pf = _make_playfield(dc, tmp_path)
        # Before get_tag, homography is None.
        assert pf.homography is None
        pf.get_tag(1)  # triggers get_tags → _update_homography
        assert pf.homography is not None
        assert pf.homography.shape == (3, 3)


# ---------------------------------------------------------------------------
# 6. test_add_path_publishes_overlay
# ---------------------------------------------------------------------------


def _last_overlay_elements(dc: MagicMock) -> list[dict]:
    """Return the ``elements`` arg of the most recent publish_overlay call."""
    assert dc.publish_overlay.called, "publish_overlay was never called"
    args, kwargs = dc.publish_overlay.call_args
    # signature: publish_overlay(cam, elements, ttl=600.0)
    cam = args[0] if len(args) > 0 else kwargs.get("cam")
    assert cam == "test-cam", f"Expected cam 'test-cam', got {cam!r}"
    return args[1] if len(args) > 1 else kwargs["elements"]


class TestAddPath:
    """add_path mutates in-memory _paths and publishes a daemon overlay."""

    def test_writes_one_path_entry(self, tmp_path: Path) -> None:
        dc = _make_dc()
        pf = _make_playfield(dc, tmp_path)

        pf.add_path("track", [(1.0, 2.0), (3.0, 4.0)])

        # publish_overlay called with this camera.
        dc.publish_overlay.assert_called_once()
        elements = _last_overlay_elements(dc)
        # A 2-waypoint path → one polyline + two points.
        polylines = [e for e in elements if e["type"] == "polyline"]
        points = [e for e in elements if e["type"] == "point"]
        assert len(polylines) == 1
        assert polylines[0]["params"] == pytest.approx([1.0, 2.0, 3.0, 4.0])
        assert len(points) == 2

        # In-memory _paths holds the path_id / waypoint structure.
        assert len(pf._paths) == 1
        entry = pf._paths[0]
        assert entry["path_id"] == "track"
        assert entry["playfield_id"] == "test-cam"
        wps = entry["waypoints"]
        assert len(wps) == 2
        assert wps[0]["x"] == pytest.approx(1.0)
        assert wps[0]["y"] == pytest.approx(2.0)
        assert wps[1]["x"] == pytest.approx(3.0)
        assert wps[1]["y"] == pytest.approx(4.0)

    def test_waypoint_schema_fields(self, tmp_path: Path) -> None:
        dc = _make_dc()
        pf = _make_playfield(dc, tmp_path)

        pf.add_path("cam", [(1.0, 2.0)], symbol="x", color=(255, 0, 0), size_cm=0.8)

        # In-memory waypoint carries the full schema.
        wp = pf._paths[0]["waypoints"][0]
        assert "x" in wp
        assert "y" in wp
        assert "size_cm" in wp
        assert "symbol" in wp
        assert "symbol_color" in wp
        assert "line_color" in wp
        assert wp["symbol"] == "x"
        assert wp["symbol_color"] == [255, 0, 0]
        assert wp["size_cm"] == pytest.approx(0.8)

        # The published overlay carries the size and symbol colour through.
        elements = _last_overlay_elements(dc)
        points = [e for e in elements if e["type"] == "point"]
        assert len(points) == 1
        assert points[0]["params"] == pytest.approx([1.0, 2.0, 0.8])
        assert points[0]["color"] == [255, 0, 0]

    def test_replace_existing_path_id(self, tmp_path: Path) -> None:
        dc = _make_dc()
        pf = _make_playfield(dc, tmp_path)

        pf.add_path("track", [(1.0, 2.0)])
        pf.add_path("track", [(9.0, 8.0)])  # replace

        assert len(pf._paths) == 1
        assert pf._paths[0]["waypoints"][0]["x"] == pytest.approx(9.0)

        # Latest overlay reflects the replacement.
        elements = _last_overlay_elements(dc)
        points = [e for e in elements if e["type"] == "point"]
        assert len(points) == 1
        assert points[0]["params"][:2] == pytest.approx([9.0, 8.0])

    def test_multiple_paths_preserved(self, tmp_path: Path) -> None:
        dc = _make_dc()
        pf = _make_playfield(dc, tmp_path)

        pf.add_path("a", [(0.0, 0.0)])
        pf.add_path("b", [(1.0, 1.0)])

        ids = {e["path_id"] for e in pf._paths}
        assert ids == {"a", "b"}

        # The final overlay carries both paths (one point each).
        elements = _last_overlay_elements(dc)
        points = [e for e in elements if e["type"] == "point"]
        assert len(points) == 2


# ---------------------------------------------------------------------------
# 7. test_clear_paths_publishes_overlay
# ---------------------------------------------------------------------------


class TestClearPaths:
    """clear_paths empties _paths and publishes an empty overlay."""

    def test_clears_after_add_path(self, tmp_path: Path) -> None:
        dc = _make_dc()
        pf = _make_playfield(dc, tmp_path)

        pf.add_path("track", [(1.0, 2.0)])
        pf.clear_paths()

        assert pf._paths == []
        # Last publish_overlay call clears the overlay (empty element list).
        elements = _last_overlay_elements(dc)
        assert elements == []

    def test_clear_without_prior_write(self, tmp_path: Path) -> None:
        dc = _make_dc()
        pf = _make_playfield(dc, tmp_path)

        pf.clear_paths()

        assert pf._paths == []
        dc.publish_overlay.assert_called_once()
        elements = _last_overlay_elements(dc)
        assert elements == []


# ---------------------------------------------------------------------------
# 8. test_add_path_publishes_via_grpc
# ---------------------------------------------------------------------------


class TestAddPathAtomic:
    """The path side-effect goes through the daemon gRPC publish_overlay API.

    (The old atomic tmp-file + os.replace write mechanism is gone; paths are now
    published as a live daemon overlay.)
    """

    def test_tmp_file_and_os_replace(self, tmp_path: Path) -> None:
        dc = _make_dc()
        pf = _make_playfield(dc, tmp_path)

        pf.add_path("cam", [(1.0, 2.0)])

        # No file is written; the publish goes over the daemon API.
        assert not (tmp_path / "paths.json").exists()
        assert not (tmp_path / "paths.json.tmp").exists()
        dc.publish_overlay.assert_called_once()
        _, kwargs = dc.publish_overlay.call_args
        assert kwargs.get("ttl") == pytest.approx(600.0)

    def test_clear_paths_also_atomic(self, tmp_path: Path) -> None:
        dc = _make_dc()
        pf = _make_playfield(dc, tmp_path)

        pf.clear_paths()

        assert not (tmp_path / "paths.json").exists()
        dc.publish_overlay.assert_called_once()
        _, kwargs = dc.publish_overlay.call_args
        assert kwargs.get("ttl") == pytest.approx(600.0)
