"""robot_radio.testgui.canvas — Playfield QGraphicsView canvas with trace paths.

Provides :class:`PlayfieldCanvas`, a ``QWidget`` wrapping a ``QGraphicsView`` /
``QGraphicsScene`` that displays:

- A background ``QPixmap`` of the physical playfield.
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
Pixels are derived from the playfield calibration JSON:

    tests/old/playfield_tour/playfield_calibration.json

The JSON has ``"playfield": {"width": 134.0, "height": 89.3}`` (cm).  The
default background image (``playfield.jpg``) is 1280 × 800 pixels.

Linear scale::

    px_per_cm_x = image_width_px  / field_width_cm
    px_per_cm_y = image_height_px / field_height_cm

World point (x_cm, y_cm) maps to::

    px  = x_cm * px_per_cm_x
    py  = y_cm * px_per_cm_y     (y increases downward in pixel space)

This is a simple linear scale anchored at the top-left of the image (world
origin 0,0 ≡ pixel origin UL).  The AprilTag coordinate system for this
playfield is A1-centred (+x east, +y north), which means +y in world space
maps to *decreasing* pixel y (upward in the image).  The transform accounts
for this sign flip:

    py = field_height_px - y_cm * px_per_cm_y

If the calibration file is not found, the canvas falls back to the raw field
dimensions (134 × 89.3 cm) with a best-guess pixel scale of 6.0 px/cm and a
grey background rectangle.

PySide6 import policy
---------------------
All PySide6 imports are deferred inside methods and factory functions so that
``import robot_radio.testgui.canvas`` succeeds without PySide6 installed
(unit tests and static analysis).

OQ-2 resolution
---------------
The playfield assets are resolved relative to this file's location:

    pathlib.Path(__file__).parents[4] / "tests" / "old" / "playfield_tour" / ...

``__file__`` is ``host/robot_radio/testgui/canvas.py``; ``parents[4]`` is the
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
# parents[3] = <repo-root>/    (when installed editable from host/)
# Actually: host is the package root, so __file__ parents:
#   [0] = testgui dir
#   [1] = robot_radio dir
#   [2] = host dir
#   [3] = repo root
_HERE = pathlib.Path(__file__).parent        # host/robot_radio/testgui/
_HOST = _HERE.parent.parent                  # host/
_REPO = _HOST.parent                         # repo root

_PLAYFIELD_IMAGE = _REPO / "tests" / "old" / "playfield_tour" / "playfield.jpg"
_PLAYFIELD_CALIB = _REPO / "tests" / "old" / "playfield_tour" / "playfield_calibration.json"

# Default field dimensions (cm) — used if calibration file is missing.
_FIELD_WIDTH_CM_DEFAULT = 134.0
_FIELD_HEIGHT_CM_DEFAULT = 89.3

# Robot marker dimensions (in field cm, scaled to px).
_MARKER_WIDTH_CM = 20.0    # robot body width approximation
_MARKER_HEIGHT_CM = 24.0   # robot body length approximation

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
    bg_pixmap = _load_bg_pixmap()
    if bg_pixmap is not None:
        img_w = bg_pixmap.width()
        img_h = bg_pixmap.height()
    else:
        # Fallback: grey rectangle sized at 6 px/cm.
        img_w = int(field_w_cm * 6)
        img_h = int(field_h_cm * 6)
        _log.debug("Playfield image not found; using %dx%d grey background", img_w, img_h)

    bg_item = QGraphicsPixmapItem(bg_pixmap if bg_pixmap is not None else QPixmap(img_w, img_h))
    if bg_pixmap is None:
        # Paint the placeholder grey.
        from PySide6.QtGui import QPainter  # type: ignore[import-untyped]
        placeholder = QPixmap(img_w, img_h)
        placeholder.fill(QColor(80, 80, 80))
        bg_item = QGraphicsPixmapItem(placeholder)
    bg_item.setZValue(-1)
    scene.addItem(bg_item)
    scene.setSceneRect(0, 0, img_w, img_h)

    # ------------------------------------------------------------------ Scale factors
    px_per_cm_x = img_w / field_w_cm
    px_per_cm_y = img_h / field_h_cm

    def world_to_px(x_cm: float, y_cm: float) -> tuple[float, float]:
        """Convert world-cm to scene pixel coordinates.

        World origin (0,0) is the top-left corner of the playfield image
        (A1-centre).  +x is east (right in the image), +y is north (up in
        the image, i.e., *decreasing* pixel y).
        """
        px = x_cm * px_per_cm_x
        py = img_h - y_cm * px_per_cm_y   # flip y-axis: north = up
        return px, py

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
    # Both are children of a parent QGraphicsRectItem used as a transform origin.
    # The marker is rotated around its own centre to match the fused heading.
    from PySide6.QtWidgets import QGraphicsItemGroup  # type: ignore[import-untyped]

    mw_px = _MARKER_WIDTH_CM * px_per_cm_x
    mh_px = _MARKER_HEIGHT_CM * px_per_cm_y
    half_h = mh_px / 2.0

    marker_group = QGraphicsItemGroup()
    scene.addItem(marker_group)
    marker_group.setZValue(2)

    # Front half (red) — occupies the top half of the marker rect.
    front_rect = QGraphicsRectItem(-mw_px / 2, -mh_px / 2, mw_px, half_h)
    front_rect.setBrush(QBrush(QColor(220, 30, 30, 200)))
    front_rect.setPen(QPen(Qt.PenStyle.NoPen))
    marker_group.addToGroup(front_rect)

    # Back half (blue) — occupies the bottom half of the marker rect.
    back_rect = QGraphicsRectItem(-mw_px / 2, 0, mw_px, half_h)
    back_rect.setBrush(QBrush(QColor(30, 80, 220, 200)))
    back_rect.setPen(QPen(Qt.PenStyle.NoPen))
    marker_group.addToGroup(back_rect)

    # Start hidden — no pose yet.
    marker_group.setVisible(False)

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
        px_per_cm_x=px_per_cm_x,
        px_per_cm_y=px_per_cm_y,
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
        px_per_cm_x: float,
        px_per_cm_y: float,
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
        self._px_per_cm_x = px_per_cm_x
        self._px_per_cm_y = px_per_cm_y
        self._img_w = img_w
        self._img_h = img_h

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
        """Replace the canvas background with a new ``QPixmap``.

        Parameters
        ----------
        pixmap:
            A ``QPixmap`` — received from the ``refresh_playfield_cb`` hook.
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
            # Rescale using the same field dimensions.
            from robot_radio.testgui.canvas import _load_calibration  # self-import ok
            field_w_cm, field_h_cm = _load_calibration()
            self._px_per_cm_x = new_w / field_w_cm
            self._px_per_cm_y = new_h / field_h_cm

            def _new_w2p(x_cm: float, y_cm: float) -> tuple[float, float]:
                px = x_cm * self._px_per_cm_x
                py = new_h - y_cm * self._px_per_cm_y
                return px, py

            self._world_to_px = _new_w2p
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

        Parameters
        ----------
        fused_yaw_rad:
            Heading in radians.  ``None`` = no update.
        """
        fused_pts = self._trace_model.fused
        if not fused_pts:
            self._marker_group.setVisible(False)  # type: ignore[attr-defined]
            return

        x_cm, y_cm = fused_pts[-1]
        px, py = self._world_to_px(x_cm, y_cm)
        self._marker_group.setPos(px, py)  # type: ignore[attr-defined]
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

def _load_bg_pixmap() -> "object | None":
    """Load the default playfield image as a ``QPixmap``.

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
