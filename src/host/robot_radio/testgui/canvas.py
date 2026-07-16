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
The canvas supports two world→pixel origins:

**A1-centred (live camera path)** — the aprilcam daemon's world frame uses
AprilTag 1 as the origin.  ``get_tags()`` provides ``.origin_x`` / ``.origin_y``
(tag 1's position in the corner-origin cm frame) and the 3×3 homography H.

A1-centred world→pixel transform::

    px = ppc * (x_cm + origin_x)
    py = ppc * (origin_y - y_cm)   # y-flip: +y north = up

So world (0, 0) → (ppc * origin_x, ppc * origin_y) = tag 1's real pixel
position in the deskewed image.  origin_x and origin_y come from the daemon
and are NOT assumed to equal field_w/2 and field_h/2.

**Sim / static fallback** — when no daemon is available, or in simulation mode,
falls back to a field-centred origin: origin_x = field_w/2, origin_y = field_h/2
so world (0, 0) maps to the image centre (matches simulation's true start pose).

The transform formula is identical in both cases; only origin_x / origin_y
differ.

Deskew pipeline (live camera path)
-----------------------------------
Given daemon homography H (3×3, raw-camera-pixel → corner-origin cm):

    warp = diag(ppc, ppc, 1) @ H
    output_size = (round(fw*ppc), round(fh*ppc))
    cv2.warpPerspective(raw_bgr, warp, output_size)

This matches ``PlayfieldCalibration.warp_matrix(ppc)`` from ``movie.py``.

The background and the world→pixel transform are always set TOGETHER from a
single daemon read; ``set_background`` accepts the calibration params so the
transform always matches the background that was warped.

PySide6 import policy
---------------------
All PySide6 imports are deferred inside methods and factory functions so that
``import robot_radio.testgui.canvas`` succeeds without PySide6 installed
(unit tests and static analysis).

Startup background policy
--------------------------
On launch the canvas always shows a **neutral grey placeholder** — the bundled
test-fixture images in ``tests_old/old/playfield_tour/`` are NEVER loaded for
live display.  A live grab (via the aprilcam daemon) is triggered automatically
on window show and again on each hardware connect; the grey placeholder is
replaced once the grab succeeds.  In sim mode (no camera) the grey placeholder
is permanent.

The bundled calibration JSON
(``tests_old/old/playfield_tour/playfield_calibration.json``) is still read
for field dimensions (cm) so the placeholder is the correct aspect ratio.

Debug override
--------------
Setting the environment variable ``TESTGUI_LOAD_STATIC_PLAYFIELD=1`` re-enables
the old behaviour that loads the bundled deskewed JPEG as the startup background.
This is **OFF by default** and intended only for debugging when no camera is
available.
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

# This file: src/host/robot_radio/testgui/canvas.py
# parents[0] = testgui/
# parents[1] = robot_radio/
# parents[2] = host/
# parents[3] = src/
# parents[4] = repo root
_HERE = pathlib.Path(__file__).parent        # src/host/robot_radio/testgui/
_SRC = _HERE.parent.parent.parent            # src/
_REPO = _SRC.parent                          # repo root

# 107-004: repointed under archive/tests_old/ -- the reorg that parked
# tests_old/ under archive/ (commit ea9b3e28, `{tests_old => archive/tests_old}`)
# never updated these three constants, so every load silently fell back to
# the hardcoded defaults again (tests/testgui/test_canvas.py, dropped from
# testpaths at 102, went stale and never caught it). Re-adding tests/testgui/
# to testpaths (this ticket) surfaced it.
_PLAYFIELD_IMAGE = _REPO / "src" / "archive" / "tests_old" / "old" / "playfield_tour" / "playfield.jpg"
_PLAYFIELD_DESKEWED = (
    _REPO / "src" / "archive" / "tests_old" / "old" / "playfield_tour" / "playfield_deskewed.jpg"
)
_PLAYFIELD_CALIB = (
    _REPO / "src" / "archive" / "tests_old" / "old" / "playfield_tour" / "playfield_calibration.json"
)

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
    "camera":  "Camera / Truth",
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


def _make_world_to_px(origin_x: float, origin_y: float, ppc: float):
    """Return an A1-centred world→pixel callable.

    World (0, 0) maps to pixel (ppc * origin_x, ppc * origin_y).
    +x is east (right), +y is north (up, i.e. *decreasing* pixel y).

    For the live-camera path, origin_x and origin_y come from the daemon's
    TagFrame (.origin_x, .origin_y), which is AprilTag 1's position in the
    corner-origin cm frame.  For the sim/static fallback, they are set to
    field_w/2 and field_h/2 so world (0,0) maps to the image centre.

    Parameters
    ----------
    origin_x, origin_y:
        A1 offset in cm (corner-origin frame).  world (0,0) → (ppc*ox, ppc*oy).
    ppc:
        Pixels per cm.

    Returns
    -------
    callable ``(x_cm, y_cm) -> (px, py)``
    """
    def world_to_px(x_cm: float, y_cm: float) -> tuple[float, float]:
        """Convert world-cm to scene pixel coordinates.

        World origin (0, 0) is AprilTag 1 (live) or playfield centre (sim).
        +x is east (right in the image), +y is north (up in the image,
        i.e. *decreasing* pixel y).
        """
        px = ppc * (x_cm + origin_x)
        py = ppc * (origin_y - y_cm)
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

    # Sim/static fallback origin: field centre (world 0,0 = image centre).
    # The live-camera path overwrites this via set_background(pixmap, origin_x, origin_y).
    _origin_x = field_w_cm / 2.0
    _origin_y = field_h_cm / 2.0

    # ------------------------------------------------------------------ Scene
    scene = QGraphicsScene()
    scene.setBackgroundBrush(QBrush(QColor(40, 40, 40)))

    # ------------------------------------------------------------------ FitView subclass
    # Defined lazily inside build_canvas so PySide6 stays a deferred import.
    class _FitView(QGraphicsView):
        """QGraphicsView that always fits the scene in the viewport.

        Overrides resizeEvent and showEvent so the scene rect is re-fitted
        whenever the widget is shown or resized — the build-time fitInView call
        happens before the viewport has its real size, so it must be repeated.
        Scrollbars and scroll-hand drag are disabled so the user can never pan.
        """

        def resizeEvent(self, event: "object") -> None:  # type: ignore[override]
            super().resizeEvent(event)
            self.fitInView(self.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

        def showEvent(self, event: "object") -> None:  # type: ignore[override]
            super().showEvent(event)
            self.fitInView(self.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    # ------------------------------------------------------------------ View
    view = _FitView(scene)
    view.setObjectName("canvas_view")
    view.setMinimumSize(400, 280)
    view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
    view.setDragMode(QGraphicsView.DragMode.NoDrag)
    view.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
    view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

    # ------------------------------------------------------------------ Background pixmap
    # Always start with a grey placeholder.  The live-camera grab (triggered by
    # the window show-event or a hardware connect) will replace it.  The bundled
    # test images in tests_old/old/ are NEVER shown for live display; they only
    # exist as unit-test fixtures.
    # Debug override: TESTGUI_LOAD_STATIC_PLAYFIELD=1 re-enables the old path.
    import os as _os
    if _os.environ.get("TESTGUI_LOAD_STATIC_PLAYFIELD") == "1":
        bg_pixmap = _load_deskewed_bg_pixmap(img_w, img_h)
        _log.debug("TESTGUI_LOAD_STATIC_PLAYFIELD=1: loaded static background")
    else:
        bg_pixmap = _make_grey_placeholder(img_w, img_h)
        _log.debug("Startup: grey placeholder %dx%d px (live grab pending)", img_w, img_h)

    bg_item = QGraphicsPixmapItem(bg_pixmap)
    bg_item.setZValue(-1)
    scene.addItem(bg_item)
    scene.setSceneRect(0, 0, img_w, img_h)

    # ------------------------------------------------------------------ Coordinate transform
    # Sim/static-fallback origin: field centre → world (0,0) = image centre.
    world_to_px = _make_world_to_px(_origin_x, _origin_y, ppc)

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

    # Show avatar at world (0, 0) immediately — before any telemetry.
    # With the A1 origin, world (0,0) maps to tag 1's pixel position.
    cx, cy = world_to_px(0.0, 0.0)
    marker_group.setPos(cx, cy)
    marker_group.setRotation(90.0)   # heading 0 = east; rotation = 90 - 0 = 90
    marker_group.setVisible(True)

    # ------------------------------------------------------------------ Checkboxes
    checkbox_widget = QWidget()
    checkbox_layout = QVBoxLayout(checkbox_widget)
    checkbox_layout.setContentsMargins(4, 4, 4, 4)
    checkbox_layout.setSpacing(4)

    checkboxes: dict[str, QCheckBox] = {}
    for name in ("camera", "fused", "otos", "encoder"):
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
        origin_x=_origin_x,
        origin_y=_origin_y,
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

    In PLAYFIELD MODE the live-view worker drives the avatar via
    :meth:`set_avatar_pose` instead of :meth:`refresh`.  Once
    :meth:`set_avatar_pose` has been called, the avatar is LOCKED to that
    camera-derived pose (position AND yaw): subsequent background swaps via
    :meth:`set_background` re-apply the locked pose through the new
    world→pixel transform instead of falling back to the fused trace / centre
    — only a fresh :meth:`set_avatar_pose` call (a new tag read) may move the
    avatar while the lock is held.  On relay disconnect call
    :meth:`restore_static_background` to revert to the grey placeholder,
    reset the origin to the field-centre fallback, AND clear the lock so
    SIM/BENCH fused-driven marker behaviour returns.

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
    origin_x, origin_y:
        A1 offset in cm (corner-origin frame).  For the live-camera path
        these come from the daemon's TagFrame; for the sim/static fallback
        they are field_w/2 and field_h/2.
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
        origin_x: float | None = None,
        origin_y: float | None = None,
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
        # A1 origin (cm, corner-origin frame): where world (0,0) sits in the deskewed image.
        # Defaults to field centre for sim/static fallback.
        self._origin_x = field_w_cm / 2.0 if origin_x is None else origin_x
        self._origin_y = field_h_cm / 2.0 if origin_y is None else origin_y

        # Expose for backward compatibility with tests that read _px_per_cm_x/_px_per_cm_y.
        self._px_per_cm_x = ppc
        self._px_per_cm_y = ppc

        # Track the last fused pose for the robot marker.
        self._last_fused_pose: tuple[float, float, float] | None = None  # (x_cm, y_cm, yaw_rad)

        # Live-view lock: when set (via set_avatar_pose), the avatar is locked
        # to this camera-derived pose and set_background must re-apply it
        # through the new transform instead of falling back to the fused
        # trace / centre.  Cleared by restore_static_background so SIM/BENCH
        # fused-driven marker behaviour returns on relay disconnect.
        self._live_pose: tuple[float, float, float] | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def refresh(
        self,
        fused_yaw_rad: float | None = None,
        *,
        update_marker: bool = True,
    ) -> None:
        """Rebuild all trace paths from the TraceModel and (optionally) the marker.

        Parameters
        ----------
        fused_yaw_rad:
            Current fused heading in radians (from the latest TLMFrame.pose[2]
            converted from centidegrees).  If ``None``, the marker heading is
            unchanged.
        update_marker:
            When ``True`` (default), reposition/rotate the robot marker from
            the latest fused pose via :meth:`_update_marker` — preserves the
            behaviour of every existing caller.  When ``False``, only the
            trace paths are rebuilt and the marker is left untouched; used in
            PLAYFIELD MODE live view where the camera bridge
            (:meth:`set_avatar_pose`) owns the avatar and a TLM-rate refresh
            must not fight it for the marker's position.
        """
        self._update_traces()
        if update_marker:
            self._update_marker(fused_yaw_rad)
        self._scene.update()  # type: ignore[attr-defined]

    def set_background(
        self,
        pixmap: "object",
        *,
        origin_x: float | None = None,
        origin_y: float | None = None,
    ) -> None:
        """Replace the canvas background and (optionally) update the A1 origin.

        Background and transform are updated atomically so that traces and the
        robot avatar always align with the displayed background image.

        In live view (after :meth:`set_avatar_pose` has been called at least
        once), the avatar is LOCKED to the last camera-derived pose: this
        method does not re-derive the marker from the fused trace on every
        swap.  Instead it rebuilds the world→pixel transform first, then
        re-applies the locked live pose through that new transform, so the
        avatar stays glued to the tag pixel-accurately even though the origin
        may have shifted slightly between frames.  Only a fresh
        :meth:`set_avatar_pose` call (a new camera pose) can move the avatar
        while a live pose is locked.  Before any :meth:`set_avatar_pose` call
        (SIM/BENCH, or live view before the first camera fix), behaviour is
        unchanged: the marker follows the fused trace / centre fallback via
        :meth:`refresh`.

        Parameters
        ----------
        pixmap:
            A deskewed ``QPixmap`` at the canonical ``_PIXELS_PER_CM`` resolution.
            If ``None`` or not a valid pixmap, the call is ignored.
        origin_x, origin_y:
            A1 offset in cm (corner-origin frame) from the daemon's TagFrame.
            When provided, world (0,0) will map to pixel (ppc*origin_x, ppc*origin_y)
            in the new background — i.e. the avatar will sit on AprilTag 1.
            When ``None``, the existing origin is preserved (or the sim fallback
            field_w/2, field_h/2 remains in effect).
        """
        try:
            from PySide6.QtGui import QPixmap  # type: ignore[import-untyped]
            if pixmap is None or not isinstance(pixmap, QPixmap) or pixmap.isNull():
                return
            # Update origin BEFORE rebuilding world_to_px so the transform
            # always matches the background that is about to be displayed.
            if origin_x is not None:
                self._origin_x = origin_x
            if origin_y is not None:
                self._origin_y = origin_y
            # Rebuild the world→pixel callable from the (possibly new) origin.
            self._world_to_px = _make_world_to_px(self._origin_x, self._origin_y, self._ppc)
            self._bg_item.setPixmap(pixmap)  # type: ignore[attr-defined]
            new_w = pixmap.width()
            new_h = pixmap.height()
            self._scene.setSceneRect(0, 0, new_w, new_h)  # type: ignore[attr-defined]
            self._img_w = new_w
            self._img_h = new_h
            # Refresh traces always; only fall back to the fused-driven marker
            # update when no live (camera) pose is locked in.  When a live
            # pose IS locked, re-apply it through the just-rebuilt transform
            # instead — this is what keeps the avatar glued to the tag across
            # the live worker's frequent background swaps (see issue
            # testgui-set-background-yanks-avatar).
            self.refresh(update_marker=(self._live_pose is None))
            if self._live_pose is not None:
                self.set_avatar_pose(*self._live_pose)
            # Re-fit the view so the new background fills the viewport correctly.
            try:
                from PySide6.QtCore import Qt  # type: ignore[import-untyped]
                self._view.fitInView(  # type: ignore[attr-defined]
                    self._scene.sceneRect(),  # type: ignore[attr-defined]
                    Qt.AspectRatioMode.KeepAspectRatio,
                )
            except Exception:
                _log.debug("set_background re-fit failed", exc_info=True)
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

    def set_avatar_pose(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
        """Position and rotate the avatar at explicit world coordinates.

        Does not consult ``trace_model.fused``.  Used in PLAYFIELD MODE where
        the camera tag drives the avatar instead of fused telemetry.

        Stores ``(x_cm, y_cm, yaw_rad)`` as the locked live pose so that
        :meth:`set_background` can re-apply it (through a possibly-updated
        transform) instead of letting its internal refresh reposition the
        marker from the fused trace / centre fallback.  This lock persists
        until :meth:`restore_static_background` clears it.

        Parameters
        ----------
        x_cm, y_cm:
            World position in centimetres (A1-centred frame).
        yaw_rad:
            Robot heading in radians.  Converted to Qt rotation via
            ``rotation = 90 - degrees(yaw_rad)``.
        """
        self._live_pose = (x_cm, y_cm, yaw_rad)
        px, py = self._world_to_px(x_cm, y_cm)
        self._marker_group.setPos(px, py)           # type: ignore[attr-defined]
        rotation = 90.0 - math.degrees(yaw_rad)  # [deg]
        self._marker_group.setRotation(rotation)  # type: ignore[attr-defined]
        self._marker_group.setVisible(True)           # type: ignore[attr-defined]
        self._scene.update()                          # type: ignore[attr-defined]

    def restore_static_background(self) -> None:
        """Replace the live camera background with a grey placeholder.

        Resets the world→pixel origin to the field-centre fallback
        ``(field_w/2, field_h/2)`` so that world (0, 0) maps to the image
        centre again — correct for Sim and for "no camera" states.  Clears
        the live-pose lock set by :meth:`set_avatar_pose` so the fused-driven
        marker behaviour (SIM/BENCH) is fully restored.  Calls :meth:`refresh`
        so traces (and the now-unlocked marker) re-render with the restored
        transform.

        Call this after stopping the live-view worker on relay disconnect.
        """
        self._origin_x = self._field_w_cm / 2.0
        self._origin_y = self._field_h_cm / 2.0
        self._world_to_px = _make_world_to_px(self._origin_x, self._origin_y, self._ppc)
        self._live_pose = None
        placeholder = _make_grey_placeholder(self._img_w, self._img_h)
        self._bg_item.setPixmap(placeholder)          # type: ignore[attr-defined]
        self.refresh()

    def reset_avatar_to_center(self) -> None:
        """Move the robot avatar to world (0, 0) and reset heading to 0° (east).

        Display-only: no motion command is sent.  Used by the "Set Robot @ 0,0"
        button after the operator has physically placed the robot at the
        A1 origin (tag 1).

        Heading is reset to 0° (east, red-front pointing +x / right) so the
        avatar orientation matches the assumed starting pose.  The Qt rotation
        formula ``rotation = 90 - degrees(yaw_rad)`` gives 90 - 0 = 90°.
        """
        cx, cy = self._world_to_px(0.0, 0.0)
        self._marker_group.setPos(cx, cy)       # type: ignore[attr-defined]
        self._marker_group.setRotation(90.0)    # yaw=0 east → 90° Qt rotation  # type: ignore[attr-defined]
        self._marker_group.setVisible(True)     # type: ignore[attr-defined]
        self._scene.update()                    # type: ignore[attr-defined]

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
        """Position and rotate the robot marker at the latest known pose.

        097: position prefers the ``encoder`` trace (host-side dead
        reckoning from ``enc``, or firmware ``encpose`` if a future build
        ever adds it back — see ``traces.py``'s ``TraceModel.feed()``
        docstring) over ``fused`` (pinned at the anchor until sprint 098
        wires ``PoseEstimator::tick()`` — a fused-only avatar would never
        move today). Falls back to ``fused``, then world (0, 0), exactly
        as before whenever the encoder trace has no points yet (avatar is
        always visible).

        Parameters
        ----------
        fused_yaw_rad:
            Heading in radians — despite the name, as of 097 the caller
            (``__main__.py``'s ``on_frame_ready``) passes the SAME
            encoder-dead-reckoning heading (``TraceModel.encoder_yaw``)
            that now drives the position above, falling back to the fused
            heading only once the encoder trace has one too. ``None`` = no
            update.
        """
        pts = self._trace_model.encoder or self._trace_model.fused
        if pts:
            x_cm, y_cm = pts[-1]
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
            #   -yaw in degrees (CCW world → CW Qt)
            #   + 90° to align item-north with screen-right (east)
            # Net: rotation = 90 - degrees(yaw_rad)
            rotation = 90.0 - math.degrees(fused_yaw_rad)  # [deg]
            self._marker_group.setRotation(rotation)  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# Asset loading helpers
# ---------------------------------------------------------------------------

def _make_grey_placeholder(img_w: int, img_h: int) -> "object":
    """Return a neutral grey ``QPixmap`` of size ``(img_w, img_h)``.

    Used as the startup background so the canvas has the correct aspect ratio
    while waiting for the first live camera grab.  Never returns a null pixmap.
    """
    from PySide6.QtGui import QColor, QPixmap  # type: ignore[import-untyped]
    pm = QPixmap(img_w, img_h)
    pm.fill(QColor(80, 80, 80))
    return pm


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
