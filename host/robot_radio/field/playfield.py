"""robot_radio.field.playfield — Playfield class wrapping the AprilCam daemon.

Coordinate convention
---------------------
All world coordinates are A1-centred, y-up, in centimetres.  The AprilTag at
position A1 is the origin (0, 0).  X increases east; Y increases north.

Pixel ↔ world transform
-----------------------
The daemon homography H maps *source-pixel space* to *raw corner-origin world
cm*, where the corner origin is at the physical top-left of the field
(not A1).  The A1 centre sits at ``(origin_x, origin_y)`` in raw cm, where::

    origin_x = field_width_cm / 2
    origin_y = field_height_cm / 2

The transform therefore requires an origin offset AND a y-axis flip (the raw
corner-origin frame has y increasing downward; the world frame has y increasing
northward / upward).

This matches ``aprilcam/ui/display.py:411-447`` exactly:

    world_to_pixel(x, y):
        raw = [x + origin_x,  origin_y - y,  1]
        px_h = inv(H) @ raw
        return (px_h[0] / px_h[2],  px_h[1] / px_h[2])

    pixel_to_world(u, v):
        raw_h = H @ [u, v, 1]
        rx, ry = raw_h[0]/raw_h[2], raw_h[1]/raw_h[2]
        return (rx - origin_x,  origin_y - ry)

paths.json schema
-----------------
[{"path_id": "...", "playfield_id": "...", "waypoints": [
    {"x": 1.0, "y": 2.0, "size_cm": 1.2,
     "symbol": "filled_circle",
     "symbol_color": [0, 200, 255],
     "line_color": [0, 200, 255]}
]}]

Written atomically: ``<path>.tmp`` → ``os.replace``.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from aprilcam.client.control import DaemonControl
    from aprilcam.client.models import TagFrame


# ---------------------------------------------------------------------------
# Public dataclasses
# ---------------------------------------------------------------------------


@dataclass
class Tag:
    """A detected AprilTag with world-frame position and yaw.

    Attributes:
        id:  Marker id.
        x:   World-frame x position, cm (A1-centred, east-positive).
        y:   World-frame y position, cm (A1-centred, north-positive).
        yaw: Tag yaw in radians (CCW-positive).
    """

    id: int
    x: float
    y: float
    yaw: float


@dataclass
class Feature:
    """A static playfield feature from playfield.json.

    Attributes:
        slug:  Machine-readable identifier, e.g. ``"rect-northwest-purple"``.
        type:  Feature type: ``"rectangle"``, ``"dot"``, ``"april_tag"``, etc.
        color: Color name or None.
        x:     World-frame x position, cm.
        y:     World-frame y position, cm.
    """

    slug: str
    type: str
    color: str | None
    x: float
    y: float


# ---------------------------------------------------------------------------
# Playfield
# ---------------------------------------------------------------------------

#: Category keys in playfield.json that contain feature records.
_FEATURE_CATEGORIES = ("rectangles", "dots", "april_tags", "aruco_tags")


class Playfield:
    """Camera-backed playfield: tags, objects, pixel↔world, and path overlays.

    Do NOT instantiate directly — use :meth:`open` or the test-friendly
    constructor :meth:`_from_parts`.

    All daemon I/O is lazy (inside methods) so that importing this module
    does not connect to the daemon.  ``aprilcam`` is imported inside methods
    that need it so that ``import robot_radio`` works in environments without
    a live daemon.
    """

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(
        self,
        _dc: "DaemonControl",
        _cam: str,
        *,
        paths_json: Path | str | None = None,
        playfield_json: Path | str | None = None,
    ) -> None:
        """Internal constructor.  Use :meth:`open` in production code.

        Args:
            _dc:            Connected :class:`DaemonControl` instance.
            _cam:           Camera name as returned by ``list_cameras()``.
            paths_json:     Override the default paths.json location (for
                            testing or non-standard data layouts).
            playfield_json: Override the playfield.json path (for testing).
        """
        self._dc = _dc
        self._cam = _cam

        # Homography cache (updated on each get_tags call).
        self._H: np.ndarray | None = None
        self._H_inv: np.ndarray | None = None

        # Field origin (half-dimensions in cm).
        self._origin_x: float = 0.0
        self._origin_y: float = 0.0

        # In-memory path list (list of path dicts for paths.json).
        self._paths: list[dict] = []

        # Resolve paths.json location.
        self._paths_json: Path | None = (
            Path(paths_json) if paths_json is not None else None
        )

        # Load playfield.json for static feature lookups.
        self._playfield_data: dict = {}
        if playfield_json is not None:
            self._load_playfield(Path(playfield_json))
        else:
            self._load_playfield_from_config()

    @classmethod
    def open(cls, cam_name: str | None = None) -> "Playfield":
        """Connect to the running AprilCam daemon and return a Playfield.

        Args:
            cam_name: Camera name to use.  If ``None``, the first camera
                      returned by ``list_cameras()`` is used.

        Returns:
            A connected :class:`Playfield` instance.

        Raises:
            RuntimeError: If the daemon is not running or no cameras are open.
        """
        from aprilcam.client.control import DaemonControl
        from aprilcam.config import Config

        dc = DaemonControl.connect_default(Config.load())
        cameras = dc.list_cameras()
        if not cameras:
            raise RuntimeError("No cameras are currently open in the AprilCam daemon.")
        cam = cam_name if cam_name is not None else cameras[0]
        return cls(dc, cam)

    @classmethod
    def _from_parts(
        cls,
        dc: "DaemonControl",
        cam: str,
        *,
        paths_json: Path | str | None = None,
        playfield_json: Path | str | None = None,
    ) -> "Playfield":
        """Test-friendly factory — accepts a pre-built (mocked) DaemonControl."""
        return cls(dc, cam, paths_json=paths_json, playfield_json=playfield_json)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _load_playfield(self, path: Path) -> None:
        """Load playfield.json from *path* and extract origin dimensions."""
        try:
            with open(path) as fh:
                self._playfield_data = json.load(fh)
        except (OSError, json.JSONDecodeError):
            self._playfield_data = {}
            return

        pf_block = self._playfield_data.get("playfield", {})
        width = float(pf_block.get("width_cm", 0.0))
        height = float(pf_block.get("height_cm", 0.0))
        # The "origin" key in playfield.json is a STRING slug
        # (e.g. "apriltag-center-a1"), NOT numeric coordinates.
        # Therefore always derive origin as the field half-dimensions.
        self._origin_x = width / 2.0
        self._origin_y = height / 2.0

    def _load_playfield_from_config(self) -> None:
        """Load playfield.json from the default AprilCam data directory."""
        try:
            from aprilcam.config import Config

            cfg = Config.load()
            pj_path = Path(cfg.data_dir) / "aprilcam" / "playfield.json"
            if pj_path.exists():
                self._load_playfield(pj_path)
        except Exception:
            pass

    def _default_paths_json(self) -> Path | None:
        """Return the default paths.json path derived from the AprilCam config."""
        if self._paths_json is not None:
            return self._paths_json
        try:
            from aprilcam.config import Config

            cfg = Config.load()
            return Path(cfg.data_dir) / "cameras" / self._cam / "paths.json"
        except Exception:
            return None

    def _update_homography(self, tf: "TagFrame") -> None:
        """Refresh the cached H / H_inv from a tag frame if it carries one."""
        if tf.homography is None:
            return
        H = np.array(tf.homography, dtype=float)
        if H.shape == (3, 3):
            self._H = H
            self._H_inv = np.linalg.inv(H)

    def _write_paths(self) -> None:
        """Atomically write the current in-memory path list to paths.json."""
        dest = self._default_paths_json()
        if dest is None:
            return
        dest.parent.mkdir(parents=True, exist_ok=True)
        tmp = str(dest) + ".tmp"
        with open(tmp, "w") as fh:
            fh.write(json.dumps(self._paths))
        os.replace(tmp, str(dest))

    # ------------------------------------------------------------------
    # Tags
    # ------------------------------------------------------------------

    def get_tag(self, tag_id: int) -> Tag | None:
        """Return the live tag with the given *tag_id*, or ``None``.

        Returns ``None`` when the tag is not currently visible or when the
        daemon has not calibrated world coordinates for it (``world_xy`` is
        ``None``).

        Args:
            tag_id: The AprilTag / ArUco marker id to look up.

        Returns:
            :class:`Tag` with world-frame ``x``, ``y``, and ``yaw``, or
            ``None``.
        """
        tf = self._dc.get_tags(self._cam)
        self._update_homography(tf)
        rec = tf.by_id(tag_id)
        if rec is None or rec.world_xy is None:
            return None
        return Tag(
            id=rec.id,
            x=float(rec.world_xy[0]),
            y=float(rec.world_xy[1]),
            yaw=float(rec.yaw),
        )

    def tags(self) -> dict[int, Tag]:
        """Return all currently visible, world-calibrated tags.

        Returns:
            Dict mapping tag id → :class:`Tag` for every tag with a valid
            ``world_xy``.
        """
        tf = self._dc.get_tags(self._cam)
        self._update_homography(tf)
        result: dict[int, Tag] = {}
        for rec in tf.tags:
            if rec.world_xy is not None:
                result[rec.id] = Tag(
                    id=rec.id,
                    x=float(rec.world_xy[0]),
                    y=float(rec.world_xy[1]),
                    yaw=float(rec.yaw),
                )
        return result

    # ------------------------------------------------------------------
    # Objects / features
    # ------------------------------------------------------------------

    def get_object(self, slug_or_query: str) -> Feature | None:
        """Return a static playfield :class:`Feature` by slug or keyword query.

        Tries ``where_is()`` on the daemon first.  Falls back to scanning
        ``playfield.json`` categories directly.

        Args:
            slug_or_query: Exact slug (e.g. ``"rect-northwest-purple"``) or a
                           natural-language query (e.g. ``"blue dot"``).

        Returns:
            :class:`Feature` with world-frame ``x`` and ``y``, or ``None``.
        """
        # Try the daemon's where_is RPC first.
        try:
            result = self._dc.where_is(slug_or_query, self._cam)
            if result.get("status") == "ok":
                matches = result.get("matches", [])
                if matches:
                    m = matches[0]
                    loc = m.get("location")
                    rec = m.get("record", {})
                    x = float(loc["x"]) if loc else float(rec.get("x", 0.0))
                    y = float(loc["y"]) if loc else float(rec.get("y", 0.0))
                    return Feature(
                        slug=m.get("slug", slug_or_query),
                        type=m.get("type", "unknown"),
                        color=rec.get("color"),
                        x=x,
                        y=y,
                    )
        except Exception:
            pass

        # Fallback: scan playfield.json categories.
        for cat in _FEATURE_CATEGORIES:
            for entry in self._playfield_data.get(cat, []):
                if entry.get("slug") == slug_or_query:
                    return Feature(
                        slug=entry["slug"],
                        type=entry.get("type", cat.rstrip("s")),
                        color=entry.get("color"),
                        x=float(entry.get("x", 0.0)),
                        y=float(entry.get("y", 0.0)),
                    )
        return None

    # ------------------------------------------------------------------
    # Pixel ↔ world transform
    # ------------------------------------------------------------------

    @property
    def homography(self) -> np.ndarray | None:
        """The current 3×3 homography matrix, or ``None`` if not yet received."""
        return self._H

    def world_to_pixel(self, x_cm: float, y_cm: float) -> tuple[float, float] | None:
        """Convert A1-centred world coordinates to source-pixel coordinates.

        Transform steps (replicating ``aprilcam/ui/display.py:437-440``):

        1. A1-centred world cm → raw corner-origin cm:
               raw = [x_cm + origin_x,  origin_y - y_cm,  1]
           The y-flip (``origin_y - y_cm``) converts from y-up (world) to
           y-down (raw corner-origin / pixel) coordinates.
        2. Raw corner-origin cm → source pixel (apply H_inv):
               px_h = H_inv @ raw
        3. Homogeneous normalise:
               (px_h[0] / px_h[2],  px_h[1] / px_h[2])

        Args:
            x_cm: World-frame x, cm.
            y_cm: World-frame y, cm.

        Returns:
            ``(u, v)`` pixel coordinates, or ``None`` if the homography has
            not been received yet.
        """
        if self._H_inv is None:
            return None
        raw = np.array([x_cm + self._origin_x, self._origin_y - y_cm, 1.0])
        px_h = self._H_inv @ raw
        return (float(px_h[0] / px_h[2]), float(px_h[1] / px_h[2]))

    def pixel_to_world(self, u: float, v: float) -> tuple[float, float] | None:
        """Convert source-pixel coordinates to A1-centred world coordinates.

        Inverse of :meth:`world_to_pixel`:

        1. Apply H to source pixel:
               raw_h = H @ [u, v, 1]
        2. Homogeneous normalise:
               rx = raw_h[0] / raw_h[2],  ry = raw_h[1] / raw_h[2]
        3. Raw corner-origin cm → A1-centred world cm:
               x = rx - origin_x,  y = origin_y - ry

        Args:
            u: Pixel column coordinate.
            v: Pixel row coordinate.

        Returns:
            ``(x_cm, y_cm)`` world coordinates, or ``None`` if the homography
            has not been received yet.
        """
        if self._H is None:
            return None
        raw_h = self._H @ np.array([u, v, 1.0])
        rx = float(raw_h[0] / raw_h[2])
        ry = float(raw_h[1] / raw_h[2])
        return (rx - self._origin_x, self._origin_y - ry)

    # ------------------------------------------------------------------
    # Path / overlay management
    # ------------------------------------------------------------------

    def add_path(
        self,
        path_id: str,
        points: list[tuple[float, float]],
        *,
        symbol: str = "filled_circle",
        color: tuple[int, int, int] = (0, 200, 255),
        size_cm: float = 1.2,
    ) -> None:
        """Add or replace a named path in paths.json.

        Args:
            path_id: Unique identifier for this path (replaces any existing
                     entry with the same id).
            points:  List of ``(x_cm, y_cm)`` world-frame waypoints.
            symbol:  Waypoint symbol name (default ``"filled_circle"``).
            color:   RGB colour tuple for both symbol and line (default cyan).
            size_cm: Waypoint marker radius in cm (default 1.2).
        """
        waypoints = [
            {
                "x": float(x),
                "y": float(y),
                "size_cm": float(size_cm),
                "symbol": symbol,
                "symbol_color": list(color),
                "line_color": list(color),
            }
            for x, y in points
        ]
        entry = {
            "path_id": path_id,
            "playfield_id": self._cam,
            "waypoints": waypoints,
        }
        # Replace existing entry with the same path_id, or append.
        for i, p in enumerate(self._paths):
            if p.get("path_id") == path_id:
                self._paths[i] = entry
                break
        else:
            self._paths.append(entry)
        self._write_paths()

    def add_symbol(
        self,
        x_cm: float,
        y_cm: float,
        *,
        symbol: str = "x",
        color: tuple[int, int, int] = (255, 235, 130),
        size_cm: float = 1.0,
    ) -> None:
        """Append a single-point marker to the special ``"symbols"`` path.

        Args:
            x_cm:   World-frame x, cm.
            y_cm:   World-frame y, cm.
            symbol: Symbol name (default ``"x"``).
            color:  RGB colour tuple (default light yellow).
            size_cm: Marker size in cm (default 1.0).
        """
        # Find existing symbols path, or create it.
        for p in self._paths:
            if p.get("path_id") == "symbols":
                p["waypoints"].append(
                    {
                        "x": float(x_cm),
                        "y": float(y_cm),
                        "size_cm": float(size_cm),
                        "symbol": symbol,
                        "symbol_color": list(color),
                        "line_color": list(color),
                    }
                )
                self._write_paths()
                return
        # No "symbols" path yet — create one.
        self.add_path("symbols", [(x_cm, y_cm)], symbol=symbol, color=color, size_cm=size_cm)

    def clear_paths(self) -> None:
        """Remove all paths and write an empty list to paths.json atomically."""
        self._paths = []
        self._write_paths()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Close the underlying DaemonControl gRPC channel."""
        self._dc.close()
