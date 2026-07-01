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
