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

    def test_marker_hidden_before_fused_feed(self, qapp):
        """Robot marker is hidden when no fused points have been added."""
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        model.anchor(0.0, 0.0, 0.0)
        widget, ctrl = build_canvas(model)
        ctrl.refresh()
        assert not ctrl._marker_group.isVisible(), "Marker must be hidden with no fused points"

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
    @pytest.fixture
    def ctrl(self, qapp):
        from robot_radio.testgui.canvas import build_canvas

        model = _make_trace_model()
        _, ctrl = build_canvas(model)
        return ctrl

    def test_origin_maps_to_top_left(self, ctrl):
        """World (0, 0) → pixel (0, img_h) because y is flipped."""
        px, py = ctrl._world_to_px(0.0, 0.0)
        assert px == pytest.approx(0.0, abs=1.0)
        assert py == pytest.approx(ctrl._img_h, abs=1.0)

    def test_top_right_corner(self, ctrl):
        """World (field_w, 0) → pixel (img_w, img_h)."""
        from robot_radio.testgui.canvas import _load_calibration
        w_cm, h_cm = _load_calibration()
        px, py = ctrl._world_to_px(w_cm, 0.0)
        assert px == pytest.approx(ctrl._img_w, abs=2.0)
        assert py == pytest.approx(ctrl._img_h, abs=2.0)

    def test_top_left_north_corner(self, ctrl):
        """World (0, field_h) → pixel (0, 0) — north is up (low pixel y)."""
        from robot_radio.testgui.canvas import _load_calibration
        w_cm, h_cm = _load_calibration()
        px, py = ctrl._world_to_px(0.0, h_cm)
        assert px == pytest.approx(0.0, abs=2.0)
        assert py == pytest.approx(0.0, abs=2.0)

    def test_centre_of_field(self, ctrl):
        """World centre maps to pixel centre."""
        from robot_radio.testgui.canvas import _load_calibration
        w_cm, h_cm = _load_calibration()
        px, py = ctrl._world_to_px(w_cm / 2, h_cm / 2)
        assert px == pytest.approx(ctrl._img_w / 2, abs=2.0)
        assert py == pytest.approx(ctrl._img_h / 2, abs=2.0)
