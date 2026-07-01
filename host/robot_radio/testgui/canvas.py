"""robot_radio.testgui.canvas — Playfield QGraphicsView canvas with trace paths.

Provides :class:`PlayfieldCanvas`, a ``QWidget`` wrapping a ``QGraphicsView`` /
``QGraphicsScene`` that displays:

- A background ``QPixmap`` of the physical playfield (deskewed via homography).
- Four ``QPainterPath`` trace polylines coloured by sensor:
    - green   — camera / ground-truth
    - orange  — encoder odometry
    - cyan    — OTOS odometry
    - magenta — fused EKF pose
- A robot marker at the current fused pose: a rectangle split into a **red
  front half** and a **blue back half**, rotated to the pose heading so the
  red half always faces the robot's forward direction.
- Per-trace visibility checkboxes in a column to the right of the canvas.

World-cm to pixel mapping
--------------------------
The background image is produced by deskewing ``playfield.jpg`` through the
homography stored in::

    tests/old/playfield_tour/playfield_calibration.json

at a fixed resolution of ``_PIXELS_PER_CM = 8.0`` px/cm.  The resulting
rectified image has size ``output_size = (field_w_cm * ppc, field_h_cm * ppc)``
and world (0, 0) sits exactly at the image centre.

Field-centred world→pixel transform::

    px = (field_w_cm / 2 + x_cm) * ppc
    py = (field_h_cm / 2 - y_cm) * ppc   # y-flip: +y north = up

This places world (0, 0) at the exact image centre.  All three of
{background, traces, robot marker} use this same ``ppc`` and formula so they
align perfectly.

Fallback
--------
If ``cv2`` is unavailable or the calibration/image files are missing the
canvas falls back to loading ``playfield_deskewed.jpg`` if present, or a plain
grey rectangle at ``output_size``.  The geometry (field-centred transform) is
preserved regardless of which fallback fires, so traces and the robot marker
always align with the background.

PySide6 import policy
---------------------
All PySide6 imports are deferred inside methods and factory functions so that
``import robot_radio.testgui.canvas`` succeeds without PySide6 installed
(unit tests and static analysis).

Asset resolution
----------------
The playfield assets are resolved relative to this file's location:

    pathlib.Path(__file__).parents[4] / "tests" / "old" / "playfield_tour" / ...

``__file__`` is ``host/robot_radio/testgui/canvas.py``; ``parents[3]`` is the
repo root when installed editable (``uv sync``).  If the assets are missing,
the canvas degrades gracefully.
"""

from __future__ import annotations

import json
import logging
import pathlib
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_radio.testgui.traces import TraceModel

_log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Asset paths (relative to package, editable install)
# ---------------------------------------------------------------------------

# This file: host/robot_radio/testgui/canvas.py
# parents[0] = testgui/
# parents[1] = robot_radio/
# parents[2] = host/
# parents[3] = repo root
_HERE = pathlib.Path(__file__).parent        # host/robot_radio/testgui/
_HOST = _HERE.parent.parent                  # host/
_REPO = _HOST.parent                         # repo root

_PLAYFIELD_IMAGE = _REPO / "tests" / "old" / "playfield_tour" / "playfield.jpg"
_PLAYFIELD_DESKEWED = _REPO / "tests" / "old" / "playfield_tour" / "playfield_deskewed.jpg"
_PLAYFIELD_CALIB = _REPO / "tests" / "old" / "playfield_tour" / "playfield_calibration.json"

# Default field dimensions (cm) — used if calibration file is missing.
_FIELD_WIDTH_CM_DEFAULT = 134.0
_FIELD_HEIGHT_CM_DEFAULT = 89.3

# Fixed pixels-per-cm for the rectified playfield image (matches movie.py cam-3 default).
_PIXELS_PER_CM = 8.0

# Robot marker physical dimensions (cm): 80 mm long (heading direction) × 50 mm wide.
_MARKER_LENGTH_CM = 8.0   # forward/heading direction (80 mm)
_MARKER_WIDTH_CM = 5.0    # lateral (50 mm)

# Trace colours (R, G, B, A) as Qt-compatible tuples.
_TRACE_COLORS = {
    "camera":  (60, 220, 90, 255),     # green
    "encoder": (255, 150, 0, 255),     # orange
    "otos":    (0, 190, 255, 255),     # cyan
    "fused":   (255, 69, 160, 255),    # magenta
}

_TRACE_LABELS = {
    "camera":  "Camera (truth)",
    "encoder": "Encoder",
    "otos":    "OTOS",
    "fused":   "Fused",
}


# ---------------------------------------------------------------------------
# Coordinate helpers (Qt-free)
# ---------------------------------------------------------------------------

def _load_calibration() -> tuple[float, float]:
    """Return (field_width_cm, field_height_cm) from the calibration JSON.

    Falls back to default field dimensions if the file is missing or malformed.
    """
    try:
        data = json.loads(_PLAYFIELD_CALIB.read_text())
        pf = data["playfield"]
        return float(pf["width"]), float(pf["height"])
    except Exception as exc:
        _log.debug("Calibration load failed (%s); using defaults", exc)
        return _FIELD_WIDTH_CM_DEFAULT, _FIELD_HEIGHT_CM_DEFAULT


def _build_playfield_calibration() -> "object | None":
    """Build a ``PlayfieldCalibration`` from the playfield tour calibration JSON.

    Returns ``None`` if the calibration file is missing, malformed, or if
    ``numpy`` is unavailable.  The returned object has ``warp_matrix(ppc)``,
    ``output_size(ppc)``, ``camera_matrix``, and ``dist_coeffs`` attributes
    matching ``robot_radio.media.movie.PlayfieldCalibration``.
    """
    try:
        import numpy as np
        from robot_radio.media.movie import PlayfieldCalibration
    except ImportError as exc:
        _log.debug("numpy/movie import failed (%s); deskew unavailable", exc)
        return None

    try:
        data = json.loads(_PLAYFIELD_CALIB.read_text())
        pf = data["playfield"]
        field_w = float(pf["width"])
        field_h = float(pf["height"])
        homography = np.array(data["homography"], dtype=float)
        camera_matrix_raw = data.get("camera_matrix")
        dist_coeffs_raw = data.get("dist_coeffs")
        camera_matrix = np.array(camera_matrix_raw, dtype=float) if camera_matrix_raw else None
        dist_coeffs = np.array(dist_coeffs_raw, dtype=float) if dist_coeffs_raw else None
        device_name = data.get("device_name", "playfield_tour")
        return PlayfieldCalibration(
            camera_name=device_name,
            field_width_cm=field_w,
            field_height_cm=field_h,
            homography=homography,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )
    except Exception as exc:
        _log.debug("_build_playfield_calibration failed (%s)", exc)
        return None


def _deskew_to_bgr(calib: "object", ppc: float) -> "object | None":
    """Deskew ``playfield.jpg`` using *calib* and return a BGR numpy ndarray.

    Returns ``None`` if ``cv2`` is unavailable or the image is missing.
    """
    try:
        import cv2
        from robot_radio.media.movie import _deskew_frame
    except ImportError as exc:
        _log.debug("cv2/movie import failed (%s); deskew unavailable", exc)
        return None

    if not _PLAYFIELD_IMAGE.exists():
        _log.debug("Playfield image not found: %s", _PLAYFIELD_IMAGE)
        return None

    frame = cv2.imread(str(_PLAYFIELD_IMAGE))
    if frame is None:
        _log.debug("cv2.imread returned None for %s", _PLAYFIELD_IMAGE)
        return None

    try:
        return _deskew_frame(frame, calib, ppc)
    except Exception as exc:
        _log.debug("_deskew_frame failed (%s)", exc)
        return None


def _bgr_to_pixmap(bgr: "object") -> "object | None":
    """Convert a BGR numpy ndarray to a ``QPixmap``.

    Returns ``None`` on failure or if PySide6 is unavailable.
    """
    try:
        import numpy as np
        from PySide6.QtGui import QImage, QPixmap  # type: ignore[import-untyped]
        bgr_arr = np.ascontiguousarray(bgr)
        h, w, ch = bgr_arr.shape
        # Convert BGR → RGB for Qt.
        rgb = bgr_arr[:, :, ::-1].copy()
        qi = QImage(rgb.data, w, h, w * ch, QImage.Format.Format_RGB888)
        pm = QPixmap.fromImage(qi)
        if pm.isNull():
            return None
        return pm
    except Exception as exc:
        _log.debug("_bgr_to_pixmap failed: %s", exc)
        return None


def _make_world_to_px(field_w_cm: float, field_h_cm: float, ppc: float):
    """Return a field-centred world→pixel callable.

    World (0, 0) maps to the image centre.  +x is east (right), +y is north
    (up, i.e. *decreasing* pixel y).

    Parameters
    ----------
    field_w_cm, field_h_cm:
        Full playfield dimensions in cm.
    ppc:
        Pixels per cm.

    Returns
    -------
    callable ``(x_cm, y_cm) -> (px, py)``
    """
    half_w = field_w_cm / 2.0
    half_h = field_h_cm / 2.0

    def world_to_px(x_cm: float, y_cm: float) -> tuple[float, float]:
        """Convert world-cm to scene pixel coordinates.

        World origin (0, 0) is the playfield centre.
        +x is east (right in the image), +y is north (up in the image,
        i.e. *decreasing* pixel y).
        """
        px = (half_w + x_cm) * ppc
        py = (half_h - y_cm) * ppc
        return px, py

    return world_to_px


# ---------------------------------------------------------------------------
# PlayfieldCanvas widget
# ---------------------------------------------------------------------------

def build_canvas(trace_model: "TraceModel") -> "tuple[object, object]":
    """Build the playfield canvas widget and return ``(widget, controller)``.

    Parameters
    ----------
    trace_model:
        The :class:`~robot_radio.testgui.traces.TraceModel` whose traces
        are rendered.  The controller subscribes to trace updates via
        :meth:`CanvasController.refresh`.

    Returns
    -------
    tuple[QWidget, CanvasController]
        ``widget`` — the ``QWidget`` to embed in the main window.
        ``controller`` — :class:`CanvasController` for update calls.
    """
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QCheckBox,
        QGraphicsPathItem,
        QGraphicsPixmapItem,
        QGraphicsRectItem,
        QGraphicsScene,
        QGraphicsView,
        QHBoxLayout,
        QSizePolicy,
        QVBoxLayout,
        QWidget,
    )
    from PySide6.QtCore import Qt  # type: ignore[import-untyped]
    from PySide6.QtGui import (  # type: ignore[import-untyped]
        QBrush,
        QColor,
        QPainterPath,
        QPen,
        QPixmap,
    )

    field_w_cm, field_h_cm = _load_calibration()
    ppc = _PIXELS_PER_CM

    # Rectified image size in pixels.
    img_w = int(round(field_w_cm * ppc))
    img_h = int(round(field_h_cm * ppc))

    # ------------------------------------------------------------------ Scene
    scene = QGraphicsScene()
    scene.setBackgroundBrush(QBrush(QColor(40, 40, 40)))

    # ------------------------------------------------------------------ View
    view = QGraphicsView(scene)
    view.setObjectName("canvas_view")
    view.setMinimumSize(400, 280)
    view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
    view.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
    view.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)

    # ------------------------------------------------------------------ Background pixmap
    bg_pixmap = _load_deskewed_bg_pixmap(img_w, img_h)

    bg_item = QGraphicsPixmapItem(bg_pixmap)
    bg_item.setZValue(-1)
    scene.addItem(bg_item)
    scene.setSceneRect(0, 0, img_w, img_h)

    # ------------------------------------------------------------------ Coordinate transform
    world_to_px = _make_world_to_px(field_w_cm, field_h_cm, ppc)

    # ------------------------------------------------------------------ Trace path items
    trace_items: dict[str, object] = {}
    for name, (r, g, b, a) in _TRACE_COLORS.items():
        pen = QPen(QColor(r, g, b, a), 2.0)
        pen.setCosmetic(True)   # constant width regardless of zoom
        path_item = QGraphicsPathItem()
        path_item.setPen(pen)
        path_item.setBrush(QBrush(Qt.BrushStyle.NoBrush))
        path_item.setZValue(1)
        scene.addItem(path_item)
        trace_items[name] = path_item

    # ------------------------------------------------------------------ Robot marker
    # The marker is TWO rectangles: front half (red) and back half (blue).
    # Both are children of a parent QGraphicsItemGroup used as transform origin.
    # The marker is rotated around its own centre to match the fused heading.
    from PySide6.QtWidgets import QGraphicsItemGroup  # type: ignore[import-untyped]

    # Physical dimensions converted to pixels.
    # Length = 8.0 cm (80 mm, heading direction); width = 5.0 cm (50 mm).
    ml_px = _MARKER_LENGTH_CM * ppc   # length in pixels (heading direction)
    mw_px = _MARKER_WIDTH_CM * ppc    # width in pixels (lateral)
    half_l = ml_px / 2.0

    marker_group = QGraphicsItemGroup()
    scene.addItem(marker_group)
    marker_group.setZValue(2)

    # Front half (red) — occupies the top half of the marker rect (y < 0 in item coords).
    front_rect = QGraphicsRectItem(-mw_px / 2, -ml_px / 2, mw_px, half_l)
    front_rect.setBrush(QBrush(QColor(220, 30, 30, 200)))
    front_rect.setPen(QPen(Qt.PenStyle.NoPen))
    marker_group.addToGroup(front_rect)

    # Back half (blue) — occupies the bottom half of the marker rect (y >= 0 in item coords).
    back_rect = QGraphicsRectItem(-mw_px / 2, 0, mw_px, half_l)
    back_rect.setBrush(QBrush(QColor(30, 80, 220, 200)))
    back_rect.setPen(QPen(Qt.PenStyle.NoPen))
    marker_group.addToGroup(back_rect)

    # Show avatar at world (0, 0) center immediately — before any telemetry.
    cx, cy = world_to_px(0.0, 0.0)
    marker_group.setPos(cx, cy)
    marker_group.setRotation(90.0)   # heading 0 = east; 90-0=90 deg Qt rotation
    marker_group.setVisible(True)

    # ------------------------------------------------------------------ Checkboxes
    checkbox_widget = QWidget()
    checkbox_layout = QVBoxLayout(checkbox_widget)
    checkbox_layout.setContentsMargins(4, 4, 4, 4)
    checkbox_layout.setSpacing(4)

    checkboxes: dict[str, QCheckBox] = {}
    for name in ("camera", "encoder", "otos", "fused"):
        r, g, b, _ = _TRACE_COLORS[name]
        label = _TRACE_LABELS[name]
        cb = QCheckBox(label)
        cb.setObjectName(f"trace_cb_{name}")
        cb.setChecked(True)
        cb.setStyleSheet(
            f"QCheckBox {{ color: rgb({r},{g},{b}); font-weight: bold; }}"
        )
        checkboxes[name] = cb
        checkbox_layout.addWidget(cb)
    checkbox_layout.addStretch()

    # ------------------------------------------------------------------ Outer layout
    outer_widget = QWidget()
    outer_layout = QHBoxLayout(outer_widget)
    outer_layout.setContentsMargins(0, 0, 0, 0)
    outer_layout.setSpacing(4)
    outer_layout.addWidget(view, stretch=1)
    outer_layout.addWidget(checkbox_widget)

    # ------------------------------------------------------------------ CanvasController
    controller = CanvasController(
        scene=scene,
        view=view,
        bg_item=bg_item,
        trace_items=trace_items,
        marker_group=marker_group,
        checkboxes=checkboxes,
        trace_model=trace_model,
        world_to_px=world_to_px,
        ppc=ppc,
        field_w_cm=field_w_cm,
        field_h_cm=field_h_cm,
        img_w=img_w,
        img_h=img_h,
    )

    # Wire checkboxes to trace visibility.
    for name, cb in checkboxes.items():
        cb.toggled.connect(lambda checked, n=name: controller.on_trace_toggled(n, checked))

    # Fit the view to the scene on first show.
    view.fitInView(scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    return outer_widget, controller


# ---------------------------------------------------------------------------
# CanvasController
# ---------------------------------------------------------------------------


class CanvasController:
    """Holds all update logic for the playfield canvas.

    Call :meth:`refresh` from the Qt main thread after every
    :meth:`~TraceModel.feed` or :meth:`~TraceModel.feed_truth` call.

    Parameters
    ----------
    scene, view, bg_item, trace_items, marker_group, checkboxes:
        Qt objects created in :func:`build_canvas`.
    trace_model:
        The :class:`~robot_radio.testgui.traces.TraceModel` to read.
    world_to_px:
        Callable ``(x_cm, y_cm) -> (px, py)`` for coordinate conversion.
    ppc:
        Pixels per cm (fixed at ``_PIXELS_PER_CM`` = 8.0).
    field_w_cm, field_h_cm:
        Playfield dimensions in cm (from calibration JSON).
    img_w, img_h:
        Rectified background image size in pixels.
    """

    def __init__(
        self,
        *,
        scene: "object",
        view: "object",
        bg_item: "object",
        trace_items: dict,
        marker_group: "object",
        checkboxes: dict,
        trace_model: "TraceModel",
        world_to_px,
        ppc: float,
        field_w_cm: float,
        field_h_cm: float,
        img_w: int,
        img_h: int,
    ) -> None:
        self._scene = scene
        self._view = view
        self._bg_item = bg_item
        self._trace_items = trace_items
        self._marker_group = marker_group
        self._checkboxes = checkboxes
        self._trace_model = trace_model
        self._world_to_px = world_to_px
        self._ppc = ppc
        self._field_w_cm = field_w_cm
        self._field_h_cm = field_h_cm
        self._img_w = img_w
        self._img_h = img_h

        # Expose for backward compatibility with tests that read _px_per_cm_x/_px_per_cm_y.
        self._px_per_cm_x = ppc
        self._px_per_cm_y = ppc

        # Track the last fused pose for the robot marker.
        self._last_fused_pose: tuple[float, float, float] | None = None  # (x_cm, y_cm, yaw_rad)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def refresh(self, fused_yaw_rad: float | None = None) -> None:
        """Rebuild all trace paths from the TraceModel and update the marker.

        Parameters
        ----------
        fused_yaw_rad:
            Current fused heading in radians (from the latest TLMFrame.pose[2]
            converted from centidegrees).  If ``None``, the marker heading is
            unchanged.
        """
        self._update_traces()
        self._update_marker(fused_yaw_rad)
        self._scene.update()  # type: ignore[attr-defined]

    def set_background(self, pixmap: "object") -> None:
        """Replace the canvas background with a new ``QPixmap`` (already deskewed).

        Parameters
        ----------
        pixmap:
            A deskewed ``QPixmap`` at the canonical ``_PIXELS_PER_CM`` resolution.
            If ``None`` or not a valid pixmap, the call is ignored.
        """
        try:
            from PySide6.QtGui import QPixmap  # type: ignore[import-untyped]
            if pixmap is None or not isinstance(pixmap, QPixmap) or pixmap.isNull():
                return
            self._bg_item.setPixmap(pixmap)  # type: ignore[attr-defined]
            new_w = pixmap.width()
            new_h = pixmap.height()
            self._scene.setSceneRect(0, 0, new_w, new_h)  # type: ignore[attr-defined]
            self._img_w = new_w
            self._img_h = new_h
            # The deskewed image is always at _PIXELS_PER_CM resolution; ppc unchanged.
            self.refresh()
        except Exception:
            _log.debug("set_background failed", exc_info=True)

    def on_trace_toggled(self, name: str, checked: bool) -> None:
        """Show or hide the trace path item and update the model's enabled flag.

        Parameters
        ----------
        name:
            Trace name: ``"camera"``, ``"encoder"``, ``"otos"``, or ``"fused"``.
        checked:
            ``True`` to show, ``False`` to hide.
        """
        self._trace_model.enabled[name] = checked
        item = self._trace_items.get(name)
        if item is not None:
            item.setVisible(checked)  # type: ignore[attr-defined]

    def reset_avatar_to_center(self) -> None:
        """Move the robot avatar to world (0, 0) and make it visible.

        Display-only: no motion command is sent.  Used by the "Set Robot @ 0,0"
        button after the operator has physically placed the robot at the
        playfield centre.
        """
        cx, cy = self._world_to_px(0.0, 0.0)
        self._marker_group.setPos(cx, cy)  # type: ignore[attr-defined]
        self._marker_group.setRotation(90.0)  # type: ignore[attr-defined]
        self._marker_group.setVisible(True)   # type: ignore[attr-defined]
        self._scene.update()  # type: ignore[attr-defined]

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _update_traces(self) -> None:
        """Rebuild all four QPainterPath objects from the current TraceModel."""
        from PySide6.QtGui import QPainterPath  # type: ignore[import-untyped]

        trace_data = {
            "camera":  self._trace_model.camera,
            "encoder": self._trace_model.encoder,
            "otos":    self._trace_model.otos,
            "fused":   self._trace_model.fused,
        }

        for name, points in trace_data.items():
            item = self._trace_items[name]
            path = QPainterPath()
            if len(points) >= 1:
                px, py = self._world_to_px(points[0][0], points[0][1])
                path.moveTo(px, py)
                for pt in points[1:]:
                    px, py = self._world_to_px(pt[0], pt[1])
                    path.lineTo(px, py)
            item.setPath(path)  # type: ignore[attr-defined]
            item.setVisible(self._trace_model.enabled[name])  # type: ignore[attr-defined]

    def _update_marker(self, fused_yaw_rad: float | None) -> None:
        """Position and rotate the robot marker at the latest fused pose.

        Falls back to world (0, 0) when no fused points are available (avatar
        is always visible).

        Parameters
        ----------
        fused_yaw_rad:
            Heading in radians.  ``None`` = no update.
        """
        fused_pts = self._trace_model.fused
        if fused_pts:
            x_cm, y_cm = fused_pts[-1]
            px, py = self._world_to_px(x_cm, y_cm)
            self._marker_group.setPos(px, py)  # type: ignore[attr-defined]
            self._marker_group.setVisible(True)  # type: ignore[attr-defined]
        else:
            # No data yet — keep avatar at world (0, 0) center, visible.
            cx, cy = self._world_to_px(0.0, 0.0)
            self._marker_group.setPos(cx, cy)  # type: ignore[attr-defined]
            self._marker_group.setVisible(True)  # type: ignore[attr-defined]

        if fused_yaw_rad is not None:
            # Qt rotation is clockwise degrees.  In world space, yaw=0 is east
            # (+x direction), which in pixel space is also rightward (+x).
            # Qt's default "up" in item space is -y (towards top of screen).
            # The marker's "front half" is placed at negative y in item coords
            # (top of the rect), so to make it face east (yaw=0) we rotate by:
            #   -yaw_deg (CCW world → CW Qt)
            #   + 90° to align item-north with screen-right (east)
            # Net: rotation_deg = 90 - degrees(yaw_rad)
            rotation_deg = 90.0 - math.degrees(fused_yaw_rad)
            self._marker_group.setRotation(rotation_deg)  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# Asset loading helpers
# ---------------------------------------------------------------------------

def _load_deskewed_bg_pixmap(img_w: int, img_h: int) -> "object":
    """Load or generate the deskewed playfield background as a ``QPixmap``.

    Attempts in order:
    1. Deskew ``playfield.jpg`` via homography (requires ``cv2`` + calibration).
    2. Load ``playfield_deskewed.jpg`` (pre-rectified fallback).
    3. Return a grey ``QPixmap`` of size ``(img_w, img_h)``.

    Always returns a valid (non-null) ``QPixmap``.
    """
    from PySide6.QtGui import QPixmap, QColor  # type: ignore[import-untyped]

    # Attempt 1: full deskew via homography.
    calib = _build_playfield_calibration()
    if calib is not None:
        bgr = _deskew_to_bgr(calib, _PIXELS_PER_CM)
        if bgr is not None:
            pm = _bgr_to_pixmap(bgr)
            if pm is not None and not pm.isNull():
                _log.debug(
                    "Loaded deskewed playfield image %dx%d px (via homography)",
                    pm.width(), pm.height(),
                )
                return pm

    # Attempt 2: pre-rectified JPEG fallback.
    if _PLAYFIELD_DESKEWED.exists():
        pm = QPixmap(str(_PLAYFIELD_DESKEWED))
        if not pm.isNull():
            _log.debug(
                "Loaded pre-deskewed playfield image %dx%d px from %s",
                pm.width(), pm.height(), _PLAYFIELD_DESKEWED,
            )
            return pm

    # Attempt 3: grey placeholder.
    _log.debug("Playfield image unavailable; using %dx%d grey background", img_w, img_h)
    placeholder = QPixmap(img_w, img_h)
    placeholder.fill(QColor(80, 80, 80))
    return placeholder


def _load_bg_pixmap() -> "object | None":
    """Load the default playfield image as a raw (un-deskewed) ``QPixmap``.

    Kept for backward compatibility.  Prefer ``_load_deskewed_bg_pixmap``.

    Returns ``None`` if PySide6 is not available or the file is missing.
    """
    try:
        from PySide6.QtGui import QPixmap  # type: ignore[import-untyped]
        if not _PLAYFIELD_IMAGE.exists():
            _log.debug("Playfield image not found: %s", _PLAYFIELD_IMAGE)
            return None
        pm = QPixmap(str(_PLAYFIELD_IMAGE))
        if pm.isNull():
            _log.debug("QPixmap.load failed for %s", _PLAYFIELD_IMAGE)
            return None
        _log.debug("Loaded playfield image %dx%d px from %s", pm.width(), pm.height(), _PLAYFIELD_IMAGE)
        return pm
    except Exception as exc:
        _log.debug("_load_bg_pixmap failed: %s", exc)
        return None
