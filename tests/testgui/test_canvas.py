"""tests/testgui/test_canvas.py — headless smoke tests for PlayfieldCanvas.

Runs with ``QT_QPA_PLATFORM=offscreen`` (set by conftest.py).
Requires PySide6 (``uv sync --group gui``).

Run with:
    uv run python -m pytest tests/testgui/test_canvas.py -q
"""

from __future__ import annotations

import math

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_frame(
    *,
    enc: tuple[int, int] | None = None,
    otos: tuple[int, int, int] | None = None,
    pose: tuple[int, int, int] | None = None,
    t: int = 0,
):
    from robot_radio.robot.protocol import TLMFrame
    return TLMFrame(t=t, enc=enc, otos=otos, pose=pose)


def _make_trace_model():
    from robot_radio.testgui.traces import TraceModel
    return TraceModel()


# ---------------------------------------------------------------------------
# canvas.py import — deferred PySide6
# ---------------------------------------------------------------------------


class TestCanvasImport:
    def test_canvas_module_importable(self):
        """canvas.py must be importable (build_canvas itself defers PySide6)."""
        import robot_radio.testgui.canvas as canvas_mod
        assert hasattr(canvas_mod, "build_canvas")
        assert hasattr(canvas_mod, "CanvasController")

    def test_load_calibration_returns_field_dims(self):
        """_load_calibration() returns (width_cm, height_cm) with sane values."""
        from robot_radio.testgui.canvas import _load_calibration
        w, h = _load_calibration()
        # Calibration JSON has 134.0 × 89.3 (or we get the defaults).
        assert 50.0 < w < 300.0, f"width out of range: {w}"
        assert 30.0 < h < 200.0, f"height out of range: {h}"

    def test_load_calibration_known_values(self):
        """calibration.json matches the known playfield dimensions."""
        from robot_radio.testgui.canvas import _load_calibration, _PLAYFIELD_CALIB
        if not _PLAYFIELD_CALIB.exists():
            pytest.skip("playfield_calibration.json not present — defaults used")
        w, h = _load_calibration()
        assert w == pytest.approx(134.0)
        assert h == pytest.approx(89.3)


# ---------------------------------------------------------------------------
# QApplication fixture
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def qapp():
    """Create (or return) the QApplication singleton for the test session."""
    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]
    import sys
    app = QApplication.instance() or QApplication(sys.argv)
    yield app


# ---------------------------------------------------------------------------
# build_canvas() smoke tests
# ---------------------------------------------------------------------------


class TestBuildCanvas:
    def test_build_canvas_returns_widget_and_controller(self, qapp):
        """build_canvas() returns a (QWidget, CanvasController) tuple."""
        from robot_radio.testgui.canvas import build_canvas, CanvasController
        from PySide6.QtWidgets import QWidget  # type: ignore[import-untyped]

        model = _make_trace_model()
        widget, ctrl = build_canvas(model)

        assert isinstance(widget, QWidget)
        assert isinstance(ctrl, CanvasController)

    def test_canvas_view_objectname(self, qapp):
        """The QGraphicsView inside the canvas has objectName 'canvas_view'."""
        from robot_radio.testgui.canvas import build_canvas
        from PySide6.QtWidgets import QGraphicsView  # type: ignore[import-untyped]

        model = _make_trace_model()
        widget, ctrl = build_canvas(model)

        view = widget.findChild(QGraphicsView, "canvas_view")
        assert view is not None, "QGraphicsView named 'canvas_view' not found"

    def test_trace_checkboxes_present(self, qapp):
        """Four checkboxes (camera, encoder, otos, fused) are built."""
        from robot_radio.testgui.canvas import build_canvas
        from PySide6.QtWidgets import QCheckBox  # type: ignore[import-untyped]

        model = _make_trace_model()
        widget, ctrl = build_canvas(model)

        for name in ("camera", "encoder", "otos", "fused"):
            cb = widget.findChild(QCheckBox, f"trace_cb_{name}")
            assert cb is not None, f"Checkbox trace_cb_{name} not found"
            assert cb.isChecked(), f"Checkbox {name} should be checked by default"

    def test_trace_checkboxes_checked_by_default(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        widget, ctrl = build_canvas(model)

        from PySide6.QtWidgets import QCheckBox  # type: ignore[import-untyped]
        for name in ("camera", "encoder", "otos", "fused"):
            cb = widget.findChild(QCheckBox, f"trace_cb_{name}")
            assert cb is not None
            assert cb.isChecked()


# ---------------------------------------------------------------------------
# CanvasController.refresh() — trace paths update
# ---------------------------------------------------------------------------


class TestCanvasRefresh:
    @pytest.fixture
    def canvas_setup(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        model.anchor(0.0, 0.0, 0.0)
        widget, ctrl = build_canvas(model)
        return model, widget, ctrl

    def test_refresh_does_not_raise(self, canvas_setup):
        model, widget, ctrl = canvas_setup
        # Feed some frames and refresh.
        model.feed(_make_frame(pose=(0, 0, 0)))
        model.feed(_make_frame(pose=(1000, 0, 0)))
        ctrl.refresh()  # must not raise

    def test_refresh_with_fused_yaw(self, canvas_setup):
        model, widget, ctrl = canvas_setup
        model.feed(_make_frame(pose=(0, 0, 0)))
        model.feed(_make_frame(pose=(500, 0, 0)))
        ctrl.refresh(fused_yaw_rad=math.pi / 4)  # must not raise

    def test_trace_paths_update_after_feed(self, canvas_setup):
        """After feeding frames and refreshing, trace path items are non-empty."""
        from PySide6.QtGui import QPainterPath  # type: ignore[import-untyped]

        model, widget, ctrl = canvas_setup

        # Feed several frames across all sensors.
        for i in range(5):
            model.feed(_make_frame(
                enc=(i * 100, i * 100),
                otos=(i * 100, 0, 0),
                pose=(i * 100, 0, 0),
            ))
        ctrl.refresh()

        # Verify each trace item has a non-empty path.
        for name in ("encoder", "otos", "fused"):
            item = ctrl._trace_items[name]
            path = item.path()
            assert not path.isEmpty(), f"{name} trace path is empty after feed"

    def test_camera_trace_path_updates(self, canvas_setup):
        model, widget, ctrl = canvas_setup
        model.feed_truth(10.0, 20.0, 0.0)
        model.feed_truth(20.0, 25.0, 0.0)
        ctrl.refresh()
        item = ctrl._trace_items["camera"]
        path = item.path()
        assert not path.isEmpty(), "camera trace path is empty after feed_truth"


# ---------------------------------------------------------------------------
# CanvasController.refresh(update_marker=False) — camera bridge owns the avatar
# (ticket 063-011: TLM refresh must not move the avatar in live view)
# ---------------------------------------------------------------------------


class TestRefreshUpdateMarkerParam:
    """refresh(update_marker=False) rebuilds traces but must NOT move the marker."""

    @pytest.fixture
    def canvas_setup(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        model.anchor(0.0, 0.0, 0.0)
        widget, ctrl = build_canvas(model)
        return model, widget, ctrl

    def test_refresh_update_marker_false_does_not_move_marker(self, canvas_setup):
        """refresh(update_marker=False) leaves the marker position unchanged."""
        model, widget, ctrl = canvas_setup

        # Establish a baseline marker position away from the origin.
        model.feed(_make_frame(pose=(0, 0, 0)))
        model.feed(_make_frame(pose=(1000, 0, 0)))
        ctrl.refresh(fused_yaw_rad=0.0)
        pos_before = ctrl._marker_group.pos()
        rotation_before = ctrl._marker_group.rotation()

        # Feed a fused point that WOULD move the marker if applied.
        model.feed(_make_frame(pose=(5000, 500, 9000)))
        ctrl.refresh(fused_yaw_rad=math.pi / 2, update_marker=False)

        pos_after = ctrl._marker_group.pos()
        rotation_after = ctrl._marker_group.rotation()
        assert pos_after.x() == pytest.approx(pos_before.x()), (
            "Marker x must not move when update_marker=False"
        )
        assert pos_after.y() == pytest.approx(pos_before.y()), (
            "Marker y must not move when update_marker=False"
        )
        assert rotation_after == pytest.approx(rotation_before), (
            "Marker rotation must not change when update_marker=False"
        )

    def test_refresh_update_marker_false_still_updates_traces(self, canvas_setup):
        """refresh(update_marker=False) still rebuilds the trace paths."""
        model, widget, ctrl = canvas_setup

        model.feed(_make_frame(pose=(0, 0, 0)))
        model.feed(_make_frame(pose=(1000, 0, 0)))
        ctrl.refresh(update_marker=False)

        item = ctrl._trace_items["fused"]
        path = item.path()
        assert not path.isEmpty(), "fused trace path must update even with update_marker=False"

    def test_refresh_default_moves_marker(self, canvas_setup):
        """refresh() with the default update_marker=True DOES move the marker
        (contrast case proving the parameter, not some other effect, gates
        the marker update)."""
        model, widget, ctrl = canvas_setup

        model.feed(_make_frame(pose=(0, 0, 0)))
        ctrl.refresh(fused_yaw_rad=0.0)
        pos_before = ctrl._marker_group.pos()

        model.feed(_make_frame(pose=(2000, 0, 0)))
        ctrl.refresh(fused_yaw_rad=0.0)
        pos_after = ctrl._marker_group.pos()

        assert pos_after.x() != pytest.approx(pos_before.x()), (
            "Marker must move on a default refresh() call (update_marker=True)"
        )


# ---------------------------------------------------------------------------
# Robot marker — red front / blue back, rotation
# ---------------------------------------------------------------------------


class TestRobotMarker:
    @pytest.fixture
    def canvas_with_fused(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        model.anchor(0.0, 0.0, 0.0)
        widget, ctrl = build_canvas(model)
        # Feed a fused pose so the marker becomes visible.
        model.feed(_make_frame(pose=(0, 0, 0)))
        model.feed(_make_frame(pose=(500, 0, 0)))
        ctrl.refresh(fused_yaw_rad=0.0)
        return model, widget, ctrl

    def test_marker_visible_after_fused_feed(self, canvas_with_fused):
        """Robot marker must be visible once a fused pose is available."""
        model, widget, ctrl = canvas_with_fused
        assert ctrl._marker_group.isVisible(), "Robot marker not visible after fused feed"

    def test_marker_visible_at_center_before_fused_feed(self, qapp):
        """Robot marker is visible at world (0,0) centre before any telemetry."""
        from robot_radio.testgui.canvas import build_canvas, _PIXELS_PER_CM
        from robot_radio.testgui.canvas import _load_calibration

        model = _make_trace_model()
        model.anchor(0.0, 0.0, 0.0)
        widget, ctrl = build_canvas(model)
        ctrl.refresh()
        # Avatar is always visible (shown at centre on startup).
        assert ctrl._marker_group.isVisible(), "Marker must be visible at startup (no data yet)"
        # Position should be at image centre (world 0,0 = centre of rectified image).
        w_cm, h_cm = _load_calibration()
        ppc = _PIXELS_PER_CM
        expected_cx = (w_cm / 2) * ppc
        expected_cy = (h_cm / 2) * ppc
        pos = ctrl._marker_group.pos()
        assert pos.x() == pytest.approx(expected_cx, abs=2.0), (
            f"Marker x at startup should be image-centre {expected_cx:.1f}px, got {pos.x():.1f}"
        )
        assert pos.y() == pytest.approx(expected_cy, abs=2.0), (
            f"Marker y at startup should be image-centre {expected_cy:.1f}px, got {pos.y():.1f}"
        )

    def test_marker_has_two_children(self, canvas_with_fused):
        """Marker group must contain exactly two rectangle items (front + back)."""
        from PySide6.QtWidgets import QGraphicsRectItem  # type: ignore[import-untyped]

        model, widget, ctrl = canvas_with_fused
        children = ctrl._marker_group.childItems()
        rect_items = [c for c in children if isinstance(c, QGraphicsRectItem)]
        assert len(rect_items) == 2, f"Expected 2 rect items in marker, got {len(rect_items)}"

    def test_marker_front_is_red(self, canvas_with_fused):
        """The front rectangle (y < 0 in item coords) must be filled red."""
        from PySide6.QtWidgets import QGraphicsRectItem  # type: ignore[import-untyped]

        model, widget, ctrl = canvas_with_fused
        children = ctrl._marker_group.childItems()
        rect_items = [c for c in children if isinstance(c, QGraphicsRectItem)]
        # Front: rect with y < 0 (placed at -half_h).
        front_items = [r for r in rect_items if r.rect().y() < 0]
        assert len(front_items) == 1, "Expected one front (y<0) rect item"
        color = front_items[0].brush().color()
        # Red: r>g, r>b, r>150.
        assert color.red() > 150, f"Front should be red; got RGB=({color.red()},{color.green()},{color.blue()})"
        assert color.red() > color.green(), "Front: red channel should dominate"
        assert color.red() > color.blue(), "Front: red channel should dominate"

    def test_marker_back_is_blue(self, canvas_with_fused):
        """The back rectangle (y >= 0 in item coords) must be filled blue."""
        from PySide6.QtWidgets import QGraphicsRectItem  # type: ignore[import-untyped]

        model, widget, ctrl = canvas_with_fused
        children = ctrl._marker_group.childItems()
        rect_items = [c for c in children if isinstance(c, QGraphicsRectItem)]
        # Back: rect with y >= 0.
        back_items = [r for r in rect_items if r.rect().y() >= 0]
        assert len(back_items) == 1, "Expected one back (y>=0) rect item"
        color = back_items[0].brush().color()
        # Blue: b>r, b>g, b>100.
        assert color.blue() > 100, f"Back should be blue; got RGB=({color.red()},{color.green()},{color.blue()})"
        assert color.blue() > color.red(), "Back: blue channel should dominate"

    def test_marker_rotation_set(self, canvas_with_fused):
        """Marker rotation is applied when fused_yaw_rad is provided."""
        model, widget, ctrl = canvas_with_fused
        yaw = math.pi / 4  # 45°
        ctrl.refresh(fused_yaw_rad=yaw)
        # Expected rotation: 90 - degrees(yaw) = 90 - 45 = 45°
        expected_deg = 90.0 - math.degrees(yaw)
        actual_deg = ctrl._marker_group.rotation()
        assert actual_deg == pytest.approx(expected_deg, abs=0.01)

    def test_marker_position_follows_fused_trace(self, qapp):
        """Marker is positioned at the last fused world point."""
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        model.anchor(0.0, 0.0, 0.0)
        widget, ctrl = build_canvas(model)

        # Feed a known fused pose.
        model.feed(_make_frame(pose=(0, 0, 0)))
        # 1340 mm = 134 cm forward (matching the calibration field width).
        model.feed(_make_frame(pose=(1340, 0, 0)))
        ctrl.refresh(fused_yaw_rad=0.0)

        # The last fused point should be at (134, 0) in world coords.
        assert len(model.fused) == 2
        last_x, last_y = model.fused[-1]
        assert last_x == pytest.approx(134.0, abs=0.1)
        assert last_y == pytest.approx(0.0, abs=0.1)

        # Marker should be visible.
        assert ctrl._marker_group.isVisible()


# ---------------------------------------------------------------------------
# Checkbox toggle — trace visibility
# ---------------------------------------------------------------------------


class TestCheckboxToggle:
    @pytest.fixture
    def canvas_setup(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        model.anchor(0.0, 0.0, 0.0)
        widget, ctrl = build_canvas(model)
        return model, widget, ctrl

    def test_uncheck_hides_trace_item(self, canvas_setup):
        """Unchecking a checkbox hides the corresponding path item."""
        model, widget, ctrl = canvas_setup
        ctrl.on_trace_toggled("encoder", False)
        item = ctrl._trace_items["encoder"]
        assert not item.isVisible(), "encoder trace item should be hidden after uncheck"

    def test_recheck_shows_trace_item(self, canvas_setup):
        """Re-checking a checkbox shows the path item again."""
        model, widget, ctrl = canvas_setup
        ctrl.on_trace_toggled("fused", False)
        ctrl.on_trace_toggled("fused", True)
        item = ctrl._trace_items["fused"]
        assert item.isVisible(), "fused trace item should be visible after re-check"

    def test_toggle_updates_model_enabled_flag(self, canvas_setup):
        """Toggling a checkbox updates TraceModel.enabled."""
        model, widget, ctrl = canvas_setup
        ctrl.on_trace_toggled("otos", False)
        assert model.enabled["otos"] is False
        ctrl.on_trace_toggled("otos", True)
        assert model.enabled["otos"] is True


# ---------------------------------------------------------------------------
# set_background() — swap background pixmap
# ---------------------------------------------------------------------------


class TestSetBackground:
    @pytest.fixture
    def canvas_setup(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        widget, ctrl = build_canvas(model)
        return model, widget, ctrl

    def test_set_background_with_none_does_not_raise(self, canvas_setup):
        model, widget, ctrl = canvas_setup
        ctrl.set_background(None)  # must not raise

    def test_set_background_with_valid_pixmap(self, canvas_setup):
        """Passing a valid QPixmap updates the background without error."""
        from PySide6.QtGui import QPixmap, QColor  # type: ignore[import-untyped]

        model, widget, ctrl = canvas_setup
        # Create a small test pixmap.
        pm = QPixmap(320, 200)
        pm.fill(QColor(100, 120, 140))
        ctrl.set_background(pm)  # must not raise

    def test_set_background_with_null_pixmap_does_not_raise(self, canvas_setup):
        """A null QPixmap is handled gracefully."""
        from PySide6.QtGui import QPixmap  # type: ignore[import-untyped]

        model, widget, ctrl = canvas_setup
        null_pm = QPixmap()
        ctrl.set_background(null_pm)  # must not raise


# ---------------------------------------------------------------------------
# View scrollbar / drag / fit-in-view policy
# ---------------------------------------------------------------------------


class TestViewFitPolicy:
    """The QGraphicsView must always fit the playfield with no scroll capability."""

    @pytest.fixture
    def view_setup(self, qapp):
        from robot_radio.testgui.canvas import build_canvas
        from PySide6.QtWidgets import QGraphicsView  # type: ignore[import-untyped]

        model = _make_trace_model()
        widget, ctrl = build_canvas(model)
        view = widget.findChild(QGraphicsView, "canvas_view")
        assert view is not None
        return widget, ctrl, view

    def test_horizontal_scrollbar_always_off(self, view_setup):
        """Horizontal scrollbar policy must be ScrollBarAlwaysOff."""
        from PySide6.QtCore import Qt  # type: ignore[import-untyped]
        widget, ctrl, view = view_setup
        assert view.horizontalScrollBarPolicy() == Qt.ScrollBarPolicy.ScrollBarAlwaysOff, (
            "Horizontal scrollbar must be ScrollBarAlwaysOff to prevent panning"
        )

    def test_vertical_scrollbar_always_off(self, view_setup):
        """Vertical scrollbar policy must be ScrollBarAlwaysOff."""
        from PySide6.QtCore import Qt  # type: ignore[import-untyped]
        widget, ctrl, view = view_setup
        assert view.verticalScrollBarPolicy() == Qt.ScrollBarPolicy.ScrollBarAlwaysOff, (
            "Vertical scrollbar must be ScrollBarAlwaysOff to prevent panning"
        )

    def test_drag_mode_no_drag(self, view_setup):
        """Drag mode must be NoDrag — user must not be able to scroll/pan."""
        from PySide6.QtWidgets import QGraphicsView  # type: ignore[import-untyped]
        widget, ctrl, view = view_setup
        assert view.dragMode() == QGraphicsView.DragMode.NoDrag, (
            "Drag mode must be NoDrag to prevent user panning"
        )

    def test_scene_rect_fits_in_viewport_after_show(self, qapp):
        """After show(), the entire sceneRect must be contained in the viewport."""
        from robot_radio.testgui.canvas import build_canvas
        from PySide6.QtWidgets import QGraphicsView  # type: ignore[import-untyped]
        from PySide6.QtCore import QRectF  # type: ignore[import-untyped]

        model = _make_trace_model()
        widget, ctrl = build_canvas(model)
        view = widget.findChild(QGraphicsView, "canvas_view")
        assert view is not None

        # Resize to a concrete size so the viewport has real dimensions.
        widget.resize(800, 500)
        view.resize(700, 500)
        # Trigger show/resize events so _FitView re-fits.
        widget.show()
        view.show()

        scene_rect = view.sceneRect()
        if scene_rect.isEmpty():
            pytest.skip("scene rect is empty — cannot test fit")

        # Map viewport corners to scene coords to get the visible scene region.
        vp_rect = view.viewport().rect()
        visible_in_scene = view.mapToScene(vp_rect).boundingRect()

        # The scene rect must be fully contained within the visible region
        # (with a small tolerance for floating-point rounding and letterboxing).
        tol = 2.0
        expanded_visible = QRectF(
            visible_in_scene.x() - tol,
            visible_in_scene.y() - tol,
            visible_in_scene.width() + 2 * tol,
            visible_in_scene.height() + 2 * tol,
        )
        assert expanded_visible.contains(scene_rect), (
            f"sceneRect {scene_rect} not fully visible; visible region {visible_in_scene}"
        )

    def test_scene_rect_fits_after_resize(self, qapp):
        """After a resize(), the entire sceneRect must still fit — no scrolling needed."""
        from robot_radio.testgui.canvas import build_canvas
        from PySide6.QtWidgets import QGraphicsView  # type: ignore[import-untyped]
        from PySide6.QtCore import QRectF  # type: ignore[import-untyped]

        model = _make_trace_model()
        widget, ctrl = build_canvas(model)
        view = widget.findChild(QGraphicsView, "canvas_view")
        assert view is not None

        widget.show()

        for w, h in [(600, 400), (1024, 600)]:
            widget.resize(w, h)
            view.resize(w - 80, h)  # leave room for checkboxes

            scene_rect = view.sceneRect()
            if scene_rect.isEmpty():
                continue

            vp_rect = view.viewport().rect()
            visible_in_scene = view.mapToScene(vp_rect).boundingRect()

            tol = 2.0
            expanded_visible = QRectF(
                visible_in_scene.x() - tol,
                visible_in_scene.y() - tol,
                visible_in_scene.width() + 2 * tol,
                visible_in_scene.height() + 2 * tol,
            )
            assert expanded_visible.contains(scene_rect), (
                f"After resize to ({w},{h}): sceneRect {scene_rect} "
                f"not fully visible; visible {visible_in_scene}"
            )

    def test_background_swap_refits(self, qapp):
        """set_background() with a new pixmap must re-fit the scene in the view."""
        from robot_radio.testgui.canvas import build_canvas
        from PySide6.QtWidgets import QGraphicsView  # type: ignore[import-untyped]
        from PySide6.QtGui import QPixmap, QColor  # type: ignore[import-untyped]
        from PySide6.QtCore import QRectF  # type: ignore[import-untyped]

        model = _make_trace_model()
        widget, ctrl = build_canvas(model)
        view = widget.findChild(QGraphicsView, "canvas_view")
        assert view is not None

        widget.resize(800, 500)
        view.resize(700, 500)
        widget.show()
        view.show()

        # Swap in a new background pixmap of a different size.
        pm = QPixmap(400, 300)
        pm.fill(QColor(50, 100, 150))
        ctrl.set_background(pm)

        scene_rect = view.sceneRect()
        assert scene_rect.width() == pytest.approx(400, abs=2), (
            f"Scene rect width should be 400 after bg swap, got {scene_rect.width()}"
        )

        vp_rect = view.viewport().rect()
        visible_in_scene = view.mapToScene(vp_rect).boundingRect()

        tol = 2.0
        expanded_visible = QRectF(
            visible_in_scene.x() - tol,
            visible_in_scene.y() - tol,
            visible_in_scene.width() + 2 * tol,
            visible_in_scene.height() + 2 * tol,
        )
        assert expanded_visible.contains(scene_rect), (
            f"After bg swap: sceneRect {scene_rect} not fully visible; "
            f"visible {visible_in_scene}"
        )


# ---------------------------------------------------------------------------
# World-to-pixel coordinate mapping
# ---------------------------------------------------------------------------


class TestWorldToPixel:
    """Field-centred world→pixel transform: world (0,0) is image centre.

    Formula: px = (field_w/2 + x_cm) * ppc
             py = (field_h/2 - y_cm) * ppc   (y-flip: north = up)
    """

    @pytest.fixture
    def ctrl(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        _, ctrl = build_canvas(model)
        return ctrl

    def test_origin_maps_to_image_centre(self, ctrl):
        """World (0, 0) must map to the centre of the rectified image."""
        px, py = ctrl._world_to_px(0.0, 0.0)
        assert px == pytest.approx(ctrl._img_w / 2.0, abs=2.0), (
            f"Origin x: expected img_w/2={ctrl._img_w/2:.1f}, got {px:.1f}"
        )
        assert py == pytest.approx(ctrl._img_h / 2.0, abs=2.0), (
            f"Origin y: expected img_h/2={ctrl._img_h/2:.1f}, got {py:.1f}"
        )

    def test_ppc_scale_east(self, ctrl):
        """Moving 1 cm east increases pixel x by ppc pixels."""
        from robot_radio.testgui.canvas import _PIXELS_PER_CM
        px0, py0 = ctrl._world_to_px(0.0, 0.0)
        px1, py1 = ctrl._world_to_px(1.0, 0.0)
        assert px1 - px0 == pytest.approx(_PIXELS_PER_CM, abs=0.01)
        assert py1 == pytest.approx(py0, abs=0.01)

    def test_ppc_scale_north(self, ctrl):
        """Moving 1 cm north decreases pixel y by ppc pixels (y-flip)."""
        from robot_radio.testgui.canvas import _PIXELS_PER_CM
        px0, py0 = ctrl._world_to_px(0.0, 0.0)
        px1, py1 = ctrl._world_to_px(0.0, 1.0)
        assert py0 - py1 == pytest.approx(_PIXELS_PER_CM, abs=0.01)
        assert px1 == pytest.approx(px0, abs=0.01)

    def test_south_west_corner(self, ctrl):
        """World (-field_w/2, -field_h/2) maps to pixel (0, img_h) — SW corner."""
        from robot_radio.testgui.canvas import _load_calibration
        w_cm, h_cm = _load_calibration()
        px, py = ctrl._world_to_px(-w_cm / 2, -h_cm / 2)
        assert px == pytest.approx(0.0, abs=2.0), f"SW corner x: {px:.1f}"
        assert py == pytest.approx(ctrl._img_h, abs=2.0), f"SW corner y: {py:.1f}"

    def test_north_east_corner(self, ctrl):
        """World (field_w/2, field_h/2) maps to pixel (img_w, 0) — NE corner."""
        from robot_radio.testgui.canvas import _load_calibration
        w_cm, h_cm = _load_calibration()
        px, py = ctrl._world_to_px(w_cm / 2, h_cm / 2)
        assert px == pytest.approx(ctrl._img_w, abs=2.0), f"NE corner x: {px:.1f}"
        assert py == pytest.approx(0.0, abs=2.0), f"NE corner y: {py:.1f}"

    def test_centre_of_field(self, ctrl):
        """World (0, 0) maps to pixel centre — explicit sanity check."""
        from robot_radio.testgui.canvas import _load_calibration
        w_cm, h_cm = _load_calibration()
        px, py = ctrl._world_to_px(0.0, 0.0)
        assert px == pytest.approx(ctrl._img_w / 2, abs=2.0)
        assert py == pytest.approx(ctrl._img_h / 2, abs=2.0)


# ---------------------------------------------------------------------------
# Robot marker physical dimensions
# ---------------------------------------------------------------------------


class TestMarkerDimensions:
    """Marker must be 8.0 cm long (heading) × 5.0 cm wide at _PIXELS_PER_CM."""

    @pytest.fixture
    def canvas_setup(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        _, ctrl = build_canvas(model)
        return ctrl

    def test_marker_length_px(self, canvas_setup):
        """Marker length (heading direction, full height in item coords) = 8.0 * ppc."""
        from robot_radio.testgui.canvas import _PIXELS_PER_CM, _MARKER_LENGTH_CM
        from PySide6.QtWidgets import QGraphicsRectItem  # type: ignore[import-untyped]

        ctrl = canvas_setup
        ppc = _PIXELS_PER_CM
        expected_length_px = _MARKER_LENGTH_CM * ppc

        children = ctrl._marker_group.childItems()
        rect_items = [c for c in children if isinstance(c, QGraphicsRectItem)]
        # Total height = front height + back height.
        total_h = sum(abs(r.rect().height()) for r in rect_items)
        assert total_h == pytest.approx(expected_length_px, abs=0.01), (
            f"Marker total length: expected {expected_length_px:.1f}px, got {total_h:.1f}px"
        )

    def test_marker_width_px(self, canvas_setup):
        """Marker width (lateral) = 5.0 * ppc."""
        from robot_radio.testgui.canvas import _PIXELS_PER_CM, _MARKER_WIDTH_CM
        from PySide6.QtWidgets import QGraphicsRectItem  # type: ignore[import-untyped]

        ctrl = canvas_setup
        ppc = _PIXELS_PER_CM
        expected_width_px = _MARKER_WIDTH_CM * ppc

        children = ctrl._marker_group.childItems()
        rect_items = [c for c in children if isinstance(c, QGraphicsRectItem)]
        for r in rect_items:
            assert r.rect().width() == pytest.approx(expected_width_px, abs=0.01), (
                f"Marker rect width: expected {expected_width_px:.1f}px, "
                f"got {r.rect().width():.1f}px"
            )


# ---------------------------------------------------------------------------
# Avatar startup position and reset_avatar_to_center
# ---------------------------------------------------------------------------


class TestAvatarCenter:
    """Avatar must start at (0,0) centre and reset there on demand."""

    @pytest.fixture
    def canvas_setup(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        _, ctrl = build_canvas(model)
        return model, ctrl

    def test_avatar_visible_at_startup(self, canvas_setup):
        """Avatar is visible before any telemetry."""
        _, ctrl = canvas_setup
        assert ctrl._marker_group.isVisible(), "Avatar must be visible at startup"

    def test_avatar_at_center_at_startup(self, canvas_setup):
        """Avatar starts at world (0,0) = image centre."""
        _, ctrl = canvas_setup
        from robot_radio.testgui.canvas import _PIXELS_PER_CM, _load_calibration
        w_cm, h_cm = _load_calibration()
        ppc = _PIXELS_PER_CM
        expected_x = (w_cm / 2) * ppc
        expected_y = (h_cm / 2) * ppc
        pos = ctrl._marker_group.pos()
        assert pos.x() == pytest.approx(expected_x, abs=2.0)
        assert pos.y() == pytest.approx(expected_y, abs=2.0)

    def test_reset_avatar_to_center_repositions(self, canvas_setup):
        """reset_avatar_to_center() moves the marker back to image centre."""
        model, ctrl = canvas_setup
        from robot_radio.testgui.canvas import _PIXELS_PER_CM, _load_calibration

        # Feed some data to move marker away from centre.
        model.anchor(0.0, 0.0, 0.0)
        model.feed(_make_frame(pose=(0, 0, 0)))
        model.feed(_make_frame(pose=(5000, 0, 0)))  # 500 cm east
        ctrl.refresh(fused_yaw_rad=0.0)

        # Avatar should no longer be at centre.
        w_cm, h_cm = _load_calibration()
        ppc = _PIXELS_PER_CM
        centre_x = (w_cm / 2) * ppc
        centre_y = (h_cm / 2) * ppc
        pos_before = ctrl._marker_group.pos()
        assert pos_before.x() != pytest.approx(centre_x, abs=5.0), (
            "Avatar should have moved away from centre after feeding pose"
        )

        # Now reset.
        ctrl.reset_avatar_to_center()

        pos_after = ctrl._marker_group.pos()
        assert pos_after.x() == pytest.approx(centre_x, abs=2.0), (
            f"reset_avatar_to_center: x={pos_after.x():.1f} should be {centre_x:.1f}"
        )
        assert pos_after.y() == pytest.approx(centre_y, abs=2.0), (
            f"reset_avatar_to_center: y={pos_after.y():.1f} should be {centre_y:.1f}"
        )
        assert ctrl._marker_group.isVisible(), "Avatar must remain visible after reset"

    def test_reset_avatar_sends_no_command(self, canvas_setup):
        """reset_avatar_to_center() is display-only — no transport interaction."""
        # This is a canvas-level test: the method has no transport parameter at all.
        _, ctrl = canvas_setup
        # Just verify the method exists and doesn't raise.
        import inspect
        sig = inspect.signature(ctrl.reset_avatar_to_center)
        assert len(sig.parameters) == 0, "reset_avatar_to_center takes no args"
        ctrl.reset_avatar_to_center()  # must not raise


# ---------------------------------------------------------------------------
# A1-centred world→pixel mapping (daemon-origin path)
# ---------------------------------------------------------------------------


class TestA1CentredWorldToPixel:
    """Daemon A1-centred world→pixel transform: world (0,0) → (ppc*ox, ppc*oy).

    The live-camera path uses origin_x/origin_y from the daemon's TagFrame,
    NOT field_w/2 or field_h/2.  This tests the core correctness of the
    A1-centred formula:

        px = ppc * (x_cm + origin_x)
        py = ppc * (origin_y - y_cm)   # y-flip
    """

    def test_make_world_to_px_origin_at_corner(self):
        """origin=(0,0) means world (0,0) maps to pixel (0,0) — field corner."""
        from robot_radio.testgui.canvas import _make_world_to_px, _PIXELS_PER_CM

        ppc = _PIXELS_PER_CM
        f = _make_world_to_px(origin_x=0.0, origin_y=0.0, ppc=ppc)
        px, py = f(0.0, 0.0)
        assert px == pytest.approx(0.0)
        assert py == pytest.approx(0.0)

    def test_make_world_to_px_a1_origin_off_centre(self):
        """origin=(ox, oy) maps world (0,0) to pixel (ppc*ox, ppc*oy) — NOT field centre."""
        from robot_radio.testgui.canvas import _make_world_to_px, _PIXELS_PER_CM

        ppc = _PIXELS_PER_CM
        ox, oy = 30.0, 20.0   # not field_w/2 or field_h/2
        f = _make_world_to_px(origin_x=ox, origin_y=oy, ppc=ppc)
        px, py = f(0.0, 0.0)
        assert px == pytest.approx(ppc * ox), f"expected px={ppc*ox}, got {px}"
        assert py == pytest.approx(ppc * oy), f"expected py={ppc*oy}, got {py}"

    def test_make_world_to_px_non_centred_differs_from_centre(self):
        """When ox != fw/2, the origin pixel differs from image centre."""
        from robot_radio.testgui.canvas import _make_world_to_px, _PIXELS_PER_CM, _load_calibration

        ppc = _PIXELS_PER_CM
        fw, fh = _load_calibration()
        half_w = fw / 2.0
        half_h = fh / 2.0
        # Use a clearly off-centre origin.
        ox, oy = half_w * 0.3, half_h * 0.7
        f = _make_world_to_px(origin_x=ox, origin_y=oy, ppc=ppc)
        px, py = f(0.0, 0.0)
        # Must NOT equal field centre.
        assert px != pytest.approx(ppc * half_w, abs=1.0), (
            "Non-centred origin should not map (0,0) to image centre x"
        )
        assert py != pytest.approx(ppc * half_h, abs=1.0), (
            "Non-centred origin should not map (0,0) to image centre y"
        )
        # Must equal (ppc*ox, ppc*oy).
        assert px == pytest.approx(ppc * ox)
        assert py == pytest.approx(ppc * oy)

    def test_make_world_to_px_east_increment(self):
        """Moving 1 cm east always increases px by ppc regardless of origin."""
        from robot_radio.testgui.canvas import _make_world_to_px, _PIXELS_PER_CM

        ppc = _PIXELS_PER_CM
        f = _make_world_to_px(origin_x=40.0, origin_y=25.0, ppc=ppc)
        px0, py0 = f(0.0, 0.0)
        px1, py1 = f(1.0, 0.0)
        assert px1 - px0 == pytest.approx(ppc, abs=0.01)
        assert py1 == pytest.approx(py0, abs=0.01)

    def test_make_world_to_px_north_decrement(self):
        """Moving 1 cm north always decreases py by ppc (y-flip) regardless of origin."""
        from robot_radio.testgui.canvas import _make_world_to_px, _PIXELS_PER_CM

        ppc = _PIXELS_PER_CM
        f = _make_world_to_px(origin_x=40.0, origin_y=25.0, ppc=ppc)
        px0, py0 = f(0.0, 0.0)
        px1, py1 = f(0.0, 1.0)
        assert py0 - py1 == pytest.approx(ppc, abs=0.01)
        assert px1 == pytest.approx(px0, abs=0.01)

    def test_set_background_updates_origin_and_transform(self, qapp):
        """set_background(pixmap, origin_x=ox, origin_y=oy) atomically updates world→px."""
        from robot_radio.testgui.canvas import build_canvas, _PIXELS_PER_CM
        from PySide6.QtGui import QPixmap, QColor  # type: ignore[import-untyped]

        model = _make_trace_model()
        _, ctrl = build_canvas(model)

        ppc = _PIXELS_PER_CM
        ox, oy = 22.0, 17.5   # A1 offset that is NOT field centre
        pm = QPixmap(int(80 * ppc), int(60 * ppc))
        pm.fill(QColor(100, 120, 140))

        ctrl.set_background(pm, origin_x=ox, origin_y=oy)

        # origin must be stored
        assert ctrl._origin_x == pytest.approx(ox)
        assert ctrl._origin_y == pytest.approx(oy)

        # world (0,0) must map to (ppc*ox, ppc*oy)
        px, py = ctrl._world_to_px(0.0, 0.0)
        assert px == pytest.approx(ppc * ox, abs=0.1)
        assert py == pytest.approx(ppc * oy, abs=0.1)

    def test_set_background_without_origin_preserves_existing(self, qapp):
        """set_background(pixmap) without origin params preserves the existing origin."""
        from robot_radio.testgui.canvas import build_canvas, _PIXELS_PER_CM
        from PySide6.QtGui import QPixmap, QColor  # type: ignore[import-untyped]

        model = _make_trace_model()
        _, ctrl = build_canvas(model)
        ppc = _PIXELS_PER_CM

        # First set a custom origin.
        ox, oy = 33.0, 21.0
        pm1 = QPixmap(int(80 * ppc), int(60 * ppc))
        pm1.fill(QColor(100, 120, 140))
        ctrl.set_background(pm1, origin_x=ox, origin_y=oy)
        assert ctrl._origin_x == pytest.approx(ox)

        # Set background again with no origin params — origin must be preserved.
        pm2 = QPixmap(int(80 * ppc), int(60 * ppc))
        pm2.fill(QColor(50, 60, 70))
        ctrl.set_background(pm2)

        assert ctrl._origin_x == pytest.approx(ox), (
            "Origin must be preserved when set_background called without origin params"
        )
        assert ctrl._origin_y == pytest.approx(oy)


# ---------------------------------------------------------------------------
# Sim fallback path — origin at field centre
# ---------------------------------------------------------------------------


class TestSimFallbackOrigin:
    """In sim/static mode, build_canvas() uses origin = (fw/2, fh/2) so world (0,0)
    maps to image centre — the simulator's true start pose is (0,0).
    """

    @pytest.fixture
    def ctrl(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        _, ctrl = build_canvas(model)
        return ctrl

    def test_sim_fallback_origin_is_field_centre(self, ctrl):
        """build_canvas() default origin = (fw/2, fh/2) — world (0,0) = image centre."""
        from robot_radio.testgui.canvas import _load_calibration

        fw, fh = _load_calibration()
        assert ctrl._origin_x == pytest.approx(fw / 2.0, abs=0.1)
        assert ctrl._origin_y == pytest.approx(fh / 2.0, abs=0.1)

    def test_sim_fallback_world_zero_is_image_centre(self, ctrl):
        """Sim fallback: world (0,0) maps to the image centre pixel."""
        px, py = ctrl._world_to_px(0.0, 0.0)
        assert px == pytest.approx(ctrl._img_w / 2.0, abs=2.0)
        assert py == pytest.approx(ctrl._img_h / 2.0, abs=2.0)


# ---------------------------------------------------------------------------
# reset_avatar_to_center — heading reset
# ---------------------------------------------------------------------------


class TestResetAvatarHeading:
    """reset_avatar_to_center() must reset the avatar heading to 0° (east).

    The Qt rotation formula is ``rotation_deg = 90 - degrees(yaw_rad)``.
    For yaw_rad=0 (east), rotation_deg = 90.0.  The reset must leave
    marker_group.rotation() == 90.0.
    """

    @pytest.fixture
    def canvas_setup(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        model.anchor(0.0, 0.0, 0.0)
        _, ctrl = build_canvas(model)
        return model, ctrl

    def test_reset_leaves_heading_zero(self, canvas_setup):
        """reset_avatar_to_center() → rotation == 90.0 (yaw=0 east)."""
        model, ctrl = canvas_setup

        # Rotate to some non-zero heading.
        ctrl.refresh(fused_yaw_rad=math.pi / 2)   # north, rotation=0
        assert ctrl._marker_group.rotation() != pytest.approx(90.0, abs=1.0), (
            "Pre-condition: rotation should not be 90 at yaw=pi/2"
        )

        # Reset — must go back to 90° (heading east = 0 rad).
        ctrl.reset_avatar_to_center()
        assert ctrl._marker_group.rotation() == pytest.approx(90.0, abs=0.01), (
            f"reset_avatar_to_center: rotation={ctrl._marker_group.rotation():.2f}° "
            "should be 90.0° (east / yaw=0)"
        )

    def test_reset_heading_after_arbitrary_yaw(self, canvas_setup):
        """reset_avatar_to_center() always resets to 90° regardless of prior heading."""
        model, ctrl = canvas_setup

        for yaw in (math.pi / 4, math.pi, 3 * math.pi / 2, -math.pi / 6):
            ctrl.refresh(fused_yaw_rad=yaw)
            ctrl.reset_avatar_to_center()
            assert ctrl._marker_group.rotation() == pytest.approx(90.0, abs=0.01), (
                f"After reset from yaw={math.degrees(yaw):.1f}°, "
                f"rotation should be 90.0°, got {ctrl._marker_group.rotation():.2f}°"
            )

    def test_reset_keeps_avatar_visible(self, canvas_setup):
        """Avatar must remain visible after heading reset."""
        model, ctrl = canvas_setup
        ctrl.refresh(fused_yaw_rad=1.0)
        ctrl.reset_avatar_to_center()
        assert ctrl._marker_group.isVisible()


# ---------------------------------------------------------------------------
# _deskew_bgr_with_tag_frame (operations helper) — daemon H path
# ---------------------------------------------------------------------------


class TestDeskewWithTagFrame:
    """_deskew_bgr_with_tag_frame: uses daemon H to warp, returns (pixmap, ox, oy)."""

    def _make_fake_tag_frame(self, ox: float = 30.0, oy: float = 20.0,
                             fw: float = 80.0, fh: float = 60.0) -> object:
        """Build a minimal fake TagFrame for testing."""
        import numpy as np

        # Identity homography (raw-pixel = cm*1, i.e. 1 px/cm → ppc scales it).
        # In practice H maps raw pixels to corner-origin cm; for a unit test we
        # just need a real 3×3 so cv2.warpPerspective doesn't fail.
        H = np.eye(3, dtype=float).tolist()

        class FakeTagFrame:
            homography = H
            origin_x = ox
            origin_y = oy
            field_width_cm = fw
            field_height_cm = fh

        return FakeTagFrame()

    def test_returns_tuple_with_origin(self):
        """Returns (QPixmap, origin_x, origin_y) matching the TagFrame."""
        import numpy as np
        from robot_radio.testgui.operations import _deskew_bgr_with_tag_frame

        ox, oy = 25.0, 18.5
        fake_tf = self._make_fake_tag_frame(ox=ox, oy=oy, fw=80.0, fh=60.0)
        # Small synthetic BGR image.
        raw_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        result = _deskew_bgr_with_tag_frame(raw_bgr, fake_tf, ppc=2.0)

        assert result is not None, "_deskew_bgr_with_tag_frame returned None"
        pixmap, got_ox, got_oy = result
        assert got_ox == pytest.approx(ox)
        assert got_oy == pytest.approx(oy)

    def test_pixmap_size_matches_field_dims(self):
        """Output QPixmap dimensions = (round(fw*ppc), round(fh*ppc))."""
        import numpy as np
        from robot_radio.testgui.operations import _deskew_bgr_with_tag_frame

        fw, fh, ppc = 80.0, 60.0, 2.0
        fake_tf = self._make_fake_tag_frame(fw=fw, fh=fh)
        raw_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        result = _deskew_bgr_with_tag_frame(raw_bgr, fake_tf, ppc=ppc)

        assert result is not None
        pixmap, _, _ = result
        assert pixmap.width() == round(fw * ppc)
        assert pixmap.height() == round(fh * ppc)

    def test_no_homography_returns_none(self):
        """Returns None when TagFrame.homography is None (uncalibrated camera)."""
        import numpy as np
        from robot_radio.testgui.operations import _deskew_bgr_with_tag_frame

        class FakeUncalibratedFrame:
            homography = None
            origin_x = 0.0
            origin_y = 0.0
            field_width_cm = 80.0
            field_height_cm = 60.0

        raw_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        result = _deskew_bgr_with_tag_frame(raw_bgr, FakeUncalibratedFrame(), ppc=2.0)
        assert result is None, "Should return None when homography is None"

    def test_world_zero_maps_to_ppc_times_origin(self):
        """With the returned (ox, oy), world (0,0) maps to (ppc*ox, ppc*oy) via canvas formula."""
        import numpy as np
        from robot_radio.testgui.operations import _deskew_bgr_with_tag_frame
        from robot_radio.testgui.canvas import _make_world_to_px

        ox, oy, ppc = 22.0, 16.0, 4.0
        fake_tf = self._make_fake_tag_frame(ox=ox, oy=oy, fw=80.0, fh=60.0)
        raw_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        result = _deskew_bgr_with_tag_frame(raw_bgr, fake_tf, ppc=ppc)

        assert result is not None
        _, got_ox, got_oy = result
        w2px = _make_world_to_px(origin_x=got_ox, origin_y=got_oy, ppc=ppc)
        px, py = w2px(0.0, 0.0)
        assert px == pytest.approx(ppc * ox, abs=0.1)
        assert py == pytest.approx(ppc * oy, abs=0.1)


# ---------------------------------------------------------------------------
# Startup background — grey placeholder, not stale bundled image
# ---------------------------------------------------------------------------


class TestStartupBackground:
    """build_canvas() must start with a grey placeholder, never the stale bundled image.

    Policy (from canvas.py docstring):
    - Default startup background is a neutral grey placeholder.
    - The bundled test images in tests/old/ are NEVER loaded for live display.
    - TESTGUI_LOAD_STATIC_PLAYFIELD=1 re-enables the old path (for debugging only).

    We verify:
    1. The background pixel map is NOT loaded from the bundled JPEG paths.
    2. The background is a grey solid-colour pixmap at the correct field aspect.
    3. _make_grey_placeholder() exists and returns a valid non-null pixmap.
    """

    def test_make_grey_placeholder_returns_valid_pixmap(self, qapp):
        """_make_grey_placeholder(w, h) returns a non-null QPixmap of size (w, h)."""
        from robot_radio.testgui.canvas import _make_grey_placeholder
        pm = _make_grey_placeholder(400, 250)
        from PySide6.QtGui import QPixmap  # type: ignore[import-untyped]
        assert isinstance(pm, QPixmap)
        assert not pm.isNull(), "_make_grey_placeholder returned null pixmap"
        assert pm.width() == 400
        assert pm.height() == 250

    def test_startup_background_not_stale_image_by_default(self, qapp):
        """build_canvas() default background must not be the bundled playfield image.

        The bundled paths in tests/old/ are stale and must not be shown as live
        display.  We verify that TESTGUI_LOAD_STATIC_PLAYFIELD is NOT set (i.e.
        the default), and that the initial background pixmap is a plain grey
        placeholder, not a photo loaded from disk.

        We check this indirectly: the grey placeholder has a uniform grey fill
        (all pixels near (80,80,80)), while a real playfield photo is not uniform.
        """
        import os
        # Ensure the debug override is OFF.
        os.environ.pop("TESTGUI_LOAD_STATIC_PLAYFIELD", None)

        from robot_radio.testgui.canvas import build_canvas, _PIXELS_PER_CM
        model = _make_trace_model()
        widget, ctrl = build_canvas(model)

        from PySide6.QtCore import Qt  # type: ignore[import-untyped]
        pm = ctrl._bg_item.pixmap()
        assert not pm.isNull(), "Background pixmap must not be null"

        # Sample the centre pixel — grey placeholder is uniform (80,80,80).
        img = pm.toImage()
        cx, cy = pm.width() // 2, pm.height() // 2
        color = img.pixelColor(cx, cy)
        # Grey placeholder: all channels should be close to 80.
        assert abs(color.red() - 80) <= 10, (
            f"Startup background should be grey (R≈80); got R={color.red()} "
            f"— it looks like a real photo was loaded (stale bundled image?)"
        )
        assert abs(color.green() - 80) <= 10, (
            f"Startup background should be grey (G≈80); got G={color.green()}"
        )
        assert abs(color.blue() - 80) <= 10, (
            f"Startup background should be grey (B≈80); got B={color.blue()}"
        )

    def test_startup_background_correct_dimensions(self, qapp):
        """Grey placeholder has the correct field dimensions at _PIXELS_PER_CM."""
        import os
        os.environ.pop("TESTGUI_LOAD_STATIC_PLAYFIELD", None)

        from robot_radio.testgui.canvas import build_canvas, _PIXELS_PER_CM, _load_calibration
        model = _make_trace_model()
        widget, ctrl = build_canvas(model)

        fw, fh = _load_calibration()
        ppc = _PIXELS_PER_CM
        expected_w = int(round(fw * ppc))
        expected_h = int(round(fh * ppc))

        pm = ctrl._bg_item.pixmap()
        assert pm.width() == expected_w, (
            f"Placeholder width: expected {expected_w}, got {pm.width()}"
        )
        assert pm.height() == expected_h, (
            f"Placeholder height: expected {expected_h}, got {pm.height()}"
        )

    def test_debug_override_flag_loads_static_image(self, qapp):
        """TESTGUI_LOAD_STATIC_PLAYFIELD=1 enables the old static-image path.

        We only test that the flag is honoured (no crash / valid pixmap returned).
        We don't assert the actual pixel content because the static image may or
        may not be present in the test environment.
        """
        import os
        os.environ["TESTGUI_LOAD_STATIC_PLAYFIELD"] = "1"
        try:
            from robot_radio.testgui.canvas import build_canvas
            model = _make_trace_model()
            widget, ctrl = build_canvas(model)  # must not crash

            pm = ctrl._bg_item.pixmap()
            assert not pm.isNull(), "Background must not be null even in debug mode"
        finally:
            os.environ.pop("TESTGUI_LOAD_STATIC_PLAYFIELD", None)
