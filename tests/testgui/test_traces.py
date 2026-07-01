"""tests/testgui/test_traces.py — headless unit tests for TraceModel.

No PySide6 required.  Tests exercise the Qt-free ``TraceModel`` class directly.

Run with:
    uv run python -m pytest tests/testgui/test_traces.py -q

(Or include in the full testgui run:  uv run python -m pytest tests/testgui -q)
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
    """Build a minimal TLMFrame for testing without hitting firmware parse."""
    from robot_radio.robot.protocol import TLMFrame

    return TLMFrame(t=t, enc=enc, otos=otos, pose=pose)


# ---------------------------------------------------------------------------
# TraceModel import — Qt-free
# ---------------------------------------------------------------------------


class TestTraceModelImport:
    """Verify that traces.py is importable without PySide6."""

    def test_import_without_pyside6(self):
        """Importing TraceModel must not require PySide6."""
        # This test must pass even if PySide6 is not installed.
        from robot_radio.testgui.traces import TraceModel  # noqa: F401
        assert TraceModel is not None

    def test_trace_names(self):
        from robot_radio.testgui.traces import TraceModel

        assert set(TraceModel.TRACE_NAMES) == {"camera", "encoder", "otos", "fused"}


# ---------------------------------------------------------------------------
# Initial state
# ---------------------------------------------------------------------------


class TestTraceModelInit:
    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()

    def test_all_traces_empty_at_init(self):
        m = self.model
        assert m.camera == []
        assert m.encoder == []
        assert m.otos == []
        assert m.fused == []

    def test_all_traces_enabled_at_init(self):
        m = self.model
        assert m.enabled["camera"] is True
        assert m.enabled["encoder"] is True
        assert m.enabled["otos"] is True
        assert m.enabled["fused"] is True

    def test_anchor_defaults_to_origin_on_first_feed(self):
        """If anchor() is not called, the first feed() anchors at (0, 0, 0)."""
        m = self.model
        frame = _make_frame(pose=(0, 0, 0))
        m.feed(frame)
        # Anchor was set implicitly; fused trace should have the anchor point.
        assert len(m.fused) >= 1
        x, y = m.fused[0]
        assert abs(x) < 1e-6
        assert abs(y) < 1e-6


# ---------------------------------------------------------------------------
# anchor()
# ---------------------------------------------------------------------------


class TestAnchor:
    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()

    def test_anchor_set(self):
        self.model.anchor(10.0, 20.0, 0.5)
        assert self.model._anchor_x == pytest.approx(10.0)
        assert self.model._anchor_y == pytest.approx(20.0)
        assert self.model._anchor_h == pytest.approx(0.5)

    def test_anchor_resets_baselines(self):
        """Calling anchor() after a feed() must reset baselines."""
        m = self.model
        frame = _make_frame(enc=(100, 200), pose=(1000, 500, 0))
        m.feed(frame)
        assert m._enc_baseline is not None
        m.anchor(0.0, 0.0, 0.0)
        assert m._enc_baseline is None
        assert m._pose_baseline is None

    def test_anchor_does_not_clear_traces(self):
        """anchor() must NOT clear accumulated traces."""
        m = self.model
        m.anchor(0.0, 0.0, 0.0)
        m.feed_truth(5.0, 3.0, 0.0)
        m.anchor(0.0, 0.0, 0.0)
        assert len(m.camera) == 1


# ---------------------------------------------------------------------------
# feed() — encoder trace
# ---------------------------------------------------------------------------


class TestFeedEncoder:
    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()
        self.model.anchor(0.0, 0.0, 0.0)

    def test_encoder_baseline_on_first_frame(self):
        """First enc frame sets the baseline; a single anchor point is emitted."""
        m = self.model
        m.feed(_make_frame(enc=(0, 0)))
        # One anchor point appended.
        assert len(m.encoder) == 1
        assert m.encoder[0] == pytest.approx((0.0, 0.0), abs=1e-6)

    def test_encoder_straight_forward(self):
        """Driving straight: both encoders advance by the same amount."""
        m = self.model
        # Baseline at 0.
        m.feed(_make_frame(enc=(0, 0)))
        # Advance 500 mm (50 cm) straight.
        m.feed(_make_frame(enc=(500, 500)))
        assert len(m.encoder) == 2
        # At heading=0 (east), straight forward → world x+50 cm, y≈0.
        x, y = m.encoder[-1]
        assert x == pytest.approx(50.0, abs=0.5)
        assert abs(y) < 0.5

    def test_encoder_no_motion(self):
        """Feeding the same encoder value twice produces the same point."""
        m = self.model
        m.feed(_make_frame(enc=(1000, 1000)))
        m.feed(_make_frame(enc=(1000, 1000)))
        assert len(m.encoder) == 2
        # Both points should be at the same location (the anchor).
        assert m.encoder[0] == pytest.approx(m.encoder[1], abs=1e-6)

    def test_encoder_jump_guard(self):
        """Large encoder jump (>5000 mm) is re-baselined; no garbage point appended."""
        m = self.model
        m.feed(_make_frame(enc=(0, 0)))
        initial_len = len(m.encoder)
        # Jump by 6000 mm — should trigger re-baseline, not append a point.
        m.feed(_make_frame(enc=(6000, 6000)))
        assert len(m.encoder) == initial_len

    def test_encoder_with_anchor_heading(self):
        """At heading=90° (north), straight forward → world y+."""
        m = self.model
        m.anchor(0.0, 0.0, math.pi / 2)
        m.feed(_make_frame(enc=(0, 0)))
        m.feed(_make_frame(enc=(500, 500)))  # 50 cm forward
        x, y = m.encoder[-1]
        # North: x≈0, y≈+50
        assert abs(x) < 0.5
        assert y == pytest.approx(50.0, abs=0.5)

    def test_encoder_with_world_anchor(self):
        """World offset from anchor is correctly added to encoder trace."""
        m = self.model
        m.anchor(10.0, 20.0, 0.0)
        m.feed(_make_frame(enc=(0, 0)))
        m.feed(_make_frame(enc=(1000, 1000)))  # 100 cm straight
        x, y = m.encoder[-1]
        # Anchor at (10, 20); forward 100 cm at heading 0 → (110, 20).
        assert x == pytest.approx(110.0, abs=0.5)
        assert y == pytest.approx(20.0, abs=0.5)


# ---------------------------------------------------------------------------
# feed() — OTOS trace
# ---------------------------------------------------------------------------


class TestFeedOTOS:
    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()
        self.model.anchor(0.0, 0.0, 0.0)

    def test_otos_baseline_on_first_frame(self):
        m = self.model
        m.feed(_make_frame(otos=(0, 0, 0)))
        assert len(m.otos) == 1
        assert m.otos[0] == pytest.approx((0.0, 0.0), abs=1e-6)

    def test_otos_x_displacement(self):
        m = self.model
        m.feed(_make_frame(otos=(0, 0, 0)))
        m.feed(_make_frame(otos=(500, 0, 0)))  # 500 mm = 50 cm in body x
        x, y = m.otos[-1]
        # At heading=0, body x+ → world x+50 cm.
        assert x == pytest.approx(50.0, abs=0.1)
        assert abs(y) < 0.1

    def test_otos_y_displacement(self):
        m = self.model
        m.feed(_make_frame(otos=(0, 0, 0)))
        m.feed(_make_frame(otos=(0, 300, 0)))  # 300 mm = 30 cm in body y
        x, y = m.otos[-1]
        # At heading=0, body y+ → world y+30 cm.
        assert abs(x) < 0.1
        assert y == pytest.approx(30.0, abs=0.1)

    def test_otos_with_north_heading(self):
        """At heading=90°, body x becomes world y."""
        m = self.model
        m.anchor(0.0, 0.0, math.pi / 2)
        m.feed(_make_frame(otos=(0, 0, 0)))
        m.feed(_make_frame(otos=(500, 0, 0)))  # 50 cm forward (body x)
        x, y = m.otos[-1]
        assert abs(x) < 0.5
        assert y == pytest.approx(50.0, abs=0.5)

    def test_otos_world_offset(self):
        """World offset from anchor is correctly applied."""
        m = self.model
        m.anchor(5.0, 7.0, 0.0)
        m.feed(_make_frame(otos=(0, 0, 0)))
        m.feed(_make_frame(otos=(200, 0, 0)))  # 20 cm
        x, y = m.otos[-1]
        assert x == pytest.approx(25.0, abs=0.1)
        assert y == pytest.approx(7.0, abs=0.1)


# ---------------------------------------------------------------------------
# feed() — fused trace
# ---------------------------------------------------------------------------


class TestFeedFused:
    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()
        self.model.anchor(0.0, 0.0, 0.0)

    def test_fused_baseline_on_first_frame(self):
        m = self.model
        m.feed(_make_frame(pose=(0, 0, 0)))
        assert len(m.fused) == 1
        assert m.fused[0] == pytest.approx((0.0, 0.0), abs=1e-6)

    def test_fused_x_displacement(self):
        m = self.model
        m.feed(_make_frame(pose=(0, 0, 0)))
        m.feed(_make_frame(pose=(1000, 0, 0)))  # 1000 mm = 100 cm body x
        x, y = m.fused[-1]
        assert x == pytest.approx(100.0, abs=0.1)
        assert abs(y) < 0.1

    def test_fused_y_displacement(self):
        m = self.model
        m.feed(_make_frame(pose=(0, 0, 0)))
        m.feed(_make_frame(pose=(0, 500, 0)))  # 500 mm = 50 cm body y
        x, y = m.fused[-1]
        assert abs(x) < 0.1
        assert y == pytest.approx(50.0, abs=0.1)

    def test_fused_accumulates(self):
        """Each frame appends one point; n frames → n+1 points (baseline + n-1 updates)."""
        m = self.model
        for i in range(5):
            m.feed(_make_frame(pose=(i * 100, 0, 0)))
        assert len(m.fused) == 5


# ---------------------------------------------------------------------------
# feed_truth()
# ---------------------------------------------------------------------------


class TestFeedTruth:
    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()

    def test_feed_truth_appends_world_point(self):
        m = self.model
        m.feed_truth(10.0, 20.0, 0.5)
        assert len(m.camera) == 1
        assert m.camera[0] == pytest.approx((10.0, 20.0), abs=1e-6)

    def test_feed_truth_multiple(self):
        m = self.model
        for i in range(5):
            m.feed_truth(float(i), float(i * 2), 0.0)
        assert len(m.camera) == 5
        assert m.camera[4] == pytest.approx((4.0, 8.0), abs=1e-6)

    def test_feed_truth_does_not_require_anchor(self):
        """feed_truth() is independent of the body-to-world transform."""
        m = self.model
        # No anchor() call — should not raise.
        m.feed_truth(99.0, 88.0, 1.0)
        assert m.camera == [(99.0, 88.0)]


# ---------------------------------------------------------------------------
# clear()
# ---------------------------------------------------------------------------


class TestClear:
    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()
        self.model.anchor(5.0, 5.0, 0.0)

    def test_clear_empties_all_traces(self):
        m = self.model
        m.feed(_make_frame(enc=(0, 0), otos=(0, 0, 0), pose=(0, 0, 0)))
        m.feed(_make_frame(enc=(100, 100), otos=(100, 0, 0), pose=(100, 0, 0)))
        m.feed_truth(1.0, 2.0, 0.0)
        assert len(m.camera) > 0
        assert len(m.encoder) > 0
        assert len(m.otos) > 0
        assert len(m.fused) > 0
        m.clear()
        assert m.camera == []
        assert m.encoder == []
        assert m.otos == []
        assert m.fused == []

    def test_clear_resets_baselines(self):
        m = self.model
        m.feed(_make_frame(enc=(1000, 1000)))
        assert m._enc_baseline is not None
        m.clear()
        assert m._enc_baseline is None
        assert m._pose_baseline is None
        assert m._otos_baseline is None

    def test_clear_preserves_anchor(self):
        """clear() must NOT reset the anchor pose."""
        m = self.model
        m.clear()
        assert m._anchor_x == pytest.approx(5.0)
        assert m._anchor_y == pytest.approx(5.0)

    def test_clear_and_refeed(self):
        """After clear(), feeding new frames works correctly from zero displacement."""
        m = self.model
        # Feed, clear, then re-feed from scratch.
        m.feed(_make_frame(pose=(0, 0, 0)))
        m.feed(_make_frame(pose=(500, 0, 0)))
        m.clear()
        m.feed(_make_frame(pose=(0, 0, 0)))
        m.feed(_make_frame(pose=(200, 0, 0)))  # 20 cm from new baseline
        assert len(m.fused) == 2
        x, y = m.fused[-1]
        # Anchor at (5, 5), heading 0, forward 20 cm → (25, 5).
        assert x == pytest.approx(25.0, abs=0.2)
        assert y == pytest.approx(5.0, abs=0.2)


# ---------------------------------------------------------------------------
# enabled flag — gates rendering, not accumulation
# ---------------------------------------------------------------------------


class TestEnabledFlag:
    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()
        self.model.anchor(0.0, 0.0, 0.0)

    def test_feed_appends_regardless_of_enabled_flag(self):
        """Disabling a trace does not prevent accumulation."""
        m = self.model
        m.enabled["fused"] = False
        m.feed(_make_frame(pose=(0, 0, 0)))
        m.feed(_make_frame(pose=(1000, 0, 0)))
        # Points are still accumulated.
        assert len(m.fused) == 2

    def test_enabled_flag_is_per_trace(self):
        m = self.model
        m.enabled["camera"] = False
        assert m.enabled["camera"] is False
        assert m.enabled["encoder"] is True


# ---------------------------------------------------------------------------
# tw() body-to-world transform
# ---------------------------------------------------------------------------


class TestBodyToWorldTransform:
    """Direct tests of the internal _tw() transform."""

    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()

    def test_tw_heading_zero_identity(self):
        """At heading=0, forward=east: (bx, 0) → (bx, 0) + anchor offset."""
        m = self.model
        m.anchor(0.0, 0.0, 0.0)
        wx, wy = m._tw(10.0, 0.0)
        assert wx == pytest.approx(10.0)
        assert wy == pytest.approx(0.0)

    def test_tw_heading_90_north(self):
        """At heading=90° (north), forward (bx,0) → (0, bx) in world."""
        m = self.model
        m.anchor(0.0, 0.0, math.pi / 2)
        wx, wy = m._tw(10.0, 0.0)
        assert wx == pytest.approx(0.0, abs=1e-6)
        assert wy == pytest.approx(10.0, abs=1e-6)

    def test_tw_heading_180_west(self):
        """At heading=180° (west), forward (bx,0) → (-bx, 0) in world."""
        m = self.model
        m.anchor(0.0, 0.0, math.pi)
        wx, wy = m._tw(10.0, 0.0)
        assert wx == pytest.approx(-10.0, abs=1e-6)
        assert wy == pytest.approx(0.0, abs=1e-6)

    def test_tw_with_anchor_offset(self):
        """Anchor world offset is correctly added."""
        m = self.model
        m.anchor(5.0, 3.0, 0.0)
        wx, wy = m._tw(0.0, 0.0)
        assert wx == pytest.approx(5.0)
        assert wy == pytest.approx(3.0)

    def test_tw_45_degree_heading(self):
        """At heading=45°, forward is NE: both x and y increase equally."""
        m = self.model
        m.anchor(0.0, 0.0, math.pi / 4)
        dist = 10.0
        wx, wy = m._tw(dist, 0.0)
        # sqrt(2)/2 ≈ 0.7071
        expected = dist * math.sqrt(2) / 2
        assert wx == pytest.approx(expected, abs=1e-5)
        assert wy == pytest.approx(expected, abs=1e-5)
