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
    encpose: tuple[int, int, int] | None = None,
    otos: tuple[int, int, int] | None = None,
    pose: tuple[int, int, int] | None = None,
    t: int = 0,
):
    """Build a minimal TLMFrame for testing without hitting firmware parse."""
    from robot_radio.robot.protocol import TLMFrame

    return TLMFrame(t=t, encpose=encpose, otos=otos, pose=pose)


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
        frame = _make_frame(encpose=(100, 200, 0), pose=(1000, 500, 0))
        m.feed(frame)
        assert m._encpose_baseline is not None
        m.anchor(0.0, 0.0, 0.0)
        assert m._encpose_baseline is None
        assert m._pose_baseline is None

    def test_anchor_does_not_clear_traces(self):
        """anchor() must NOT clear accumulated traces."""
        m = self.model
        m.anchor(0.0, 0.0, 0.0)
        m.feed_truth(5.0, 3.0, 0.0)
        m.anchor(0.0, 0.0, 0.0)
        assert len(m.camera) == 1


# ---------------------------------------------------------------------------
# feed() — encoder trace (068-003: fed from firmware encpose=, structurally
# identical to _feed_otos()/_feed_fused() — no host-side re-integration, no
# reset-detection heuristic; see architecture-update.md Decision 4)
# ---------------------------------------------------------------------------


class TestFeedEncpose:
    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()
        self.model.anchor(0.0, 0.0, 0.0)

    def test_encpose_baseline_on_first_frame(self):
        m = self.model
        m.feed(_make_frame(encpose=(0, 0, 0)))
        assert len(m.encoder) == 1
        assert m.encoder[0] == pytest.approx((0.0, 0.0), abs=1e-6)

    def test_encpose_x_displacement(self):
        m = self.model
        m.feed(_make_frame(encpose=(0, 0, 0)))
        m.feed(_make_frame(encpose=(500, 0, 0)))  # 500 mm = 50 cm in body x
        x, y = m.encoder[-1]
        # At heading=0, body x+ → world x+50 cm.
        assert x == pytest.approx(50.0, abs=0.1)
        assert abs(y) < 0.1

    def test_encpose_y_displacement(self):
        m = self.model
        m.feed(_make_frame(encpose=(0, 0, 0)))
        m.feed(_make_frame(encpose=(0, 300, 0)))  # 300 mm = 30 cm in body y
        x, y = m.encoder[-1]
        # At heading=0, body y+ → world y+30 cm.
        assert abs(x) < 0.1
        assert y == pytest.approx(30.0, abs=0.1)

    def test_encpose_absent_field_skips_without_crash(self):
        """frame.encpose is None (e.g. talking to pre-068 firmware): skip,
        no trace point appended, no crash — same as an absent otos/pose."""
        m = self.model
        m.feed(_make_frame(encpose=None, pose=(0, 0, 0)))
        assert m.encoder == []
        assert len(m.fused) == 1

    def test_encpose_with_anchor_heading(self):
        """At heading=90° (north), straight forward → world y+."""
        m = self.model
        m.anchor(0.0, 0.0, math.pi / 2)
        m.feed(_make_frame(encpose=(0, 0, 0)))
        m.feed(_make_frame(encpose=(500, 0, 0)))  # 50 cm forward (body x)
        x, y = m.encoder[-1]
        assert abs(x) < 0.5
        assert y == pytest.approx(50.0, abs=0.5)

    def test_encpose_world_offset(self):
        """World offset from anchor is correctly applied."""
        m = self.model
        m.anchor(5.0, 7.0, 0.0)
        m.feed(_make_frame(encpose=(0, 0, 0)))
        m.feed(_make_frame(encpose=(200, 0, 0)))  # 20 cm
        x, y = m.encoder[-1]
        assert x == pytest.approx(25.0, abs=0.1)
        assert y == pytest.approx(7.0, abs=0.1)

    def test_encpose_mid_session_anchor_rotation(self):
        """Anchoring mid-session (nonzero firmware heading at baseline time)
        must rotate the encpose delta by (anchor_yaw - baseline_hdg) — same
        CR-10 fix already applied to otos/fused (see
        TestFeedOTOS.test_otos_mid_session_anchor_rotation)."""
        m = self.model
        m.anchor(0.0, 0.0, 0.0)  # camera: robot faces east at anchor time
        # Baseline: firmware heading is 90° (9000 cdeg) at anchor time.
        m.feed(_make_frame(encpose=(1000, 2000, 9000)))
        # Straight ahead in the firmware's own frame: +500 mm along firmware-y.
        m.feed(_make_frame(encpose=(1000, 2500, 9000)))
        x, y = m.encoder[-1]
        # Correct (rotated by -90°): east by 50 cm, not north.
        assert x == pytest.approx(50.0, abs=0.5)
        assert abs(y) < 0.5

    def test_encpose_does_not_use_trackwidth_or_scrub(self):
        """The deleted _feed_encoder() re-integration required a trackwidth
        and turn-scrub knob; _feed_encpose() plots the firmware's own
        already-computed pose directly, so TraceModel has no such methods."""
        assert not hasattr(self.model, "set_trackwidth_mm")
        assert not hasattr(self.model, "set_turn_scrub_factor")
        assert not hasattr(self.model, "notify_reset_pending")
        assert not hasattr(self.model, "_feed_encoder")


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

    def test_otos_mid_session_anchor_rotation(self):
        """Anchoring mid-session (nonzero firmware heading at baseline time)
        must rotate the otos delta by (anchor_yaw - baseline_hdg), aligning
        it with the camera trace — not by the anchor heading alone.

        Regression for CR-10: the camera says the robot faces east
        (anchor heading 0) at the moment of a mid-session anchor, but the
        firmware's own (never-rezeroed) OTOS heading reads 90° at that same
        instant — a fixed 90° offset between the firmware's persistent
        world frame and the camera/anchor frame.  The robot then drives
        straight (no turn) in the firmware's own frame; the raw firmware
        delta is expressed in the firmware frame (here: +y, since the
        firmware's heading is 90°) and must be rotated by -90° to land where
        the camera would actually see the robot (east of the anchor).
        """
        m = self.model
        m.anchor(0.0, 0.0, 0.0)  # camera: robot faces east at anchor time
        # Baseline: firmware heading is 90° (9000 cdeg) — NOT zero — at the
        # instant of this mid-session anchor.
        m.feed(_make_frame(otos=(1000, 2000, 9000)))
        # Straight ahead in the firmware's own frame (heading unchanged at
        # 90°): +500 mm along firmware-y.
        m.feed(_make_frame(otos=(1000, 2500, 9000)))
        x, y = m.otos[-1]
        # Correct (rotated by -90°): east by 50 cm, not north.
        assert x == pytest.approx(50.0, abs=0.5)
        assert abs(y) < 0.5


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

    def test_fused_mid_session_anchor_rotation(self):
        """Same CR-10 rotation fix as otos, applied to the fused trace.

        See TestFeedOTOS.test_otos_mid_session_anchor_rotation for the full
        scenario rationale.
        """
        m = self.model
        m.anchor(0.0, 0.0, 0.0)  # camera: robot faces east at anchor time
        m.feed(_make_frame(pose=(1000, 2000, 9000)))  # baseline hdg=90°
        m.feed(_make_frame(pose=(1000, 2500, 9000)))  # +500 mm firmware-y
        x, y = m.fused[-1]
        assert x == pytest.approx(50.0, abs=0.5)
        assert abs(y) < 0.5


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
        m.feed(_make_frame(encpose=(0, 0, 0), otos=(0, 0, 0), pose=(0, 0, 0)))
        m.feed(_make_frame(encpose=(100, 0, 0), otos=(100, 0, 0), pose=(100, 0, 0)))
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
        m.feed(_make_frame(encpose=(1000, 1000, 0)))
        assert m._encpose_baseline is not None
        m.clear()
        assert m._encpose_baseline is None
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


# ---------------------------------------------------------------------------
# _rw() firmware-world-frame rotation transform (CR-10)
# ---------------------------------------------------------------------------


class TestRotateWorldTransform:
    """Direct tests of the internal _rw() transform used by otos/fused."""

    def setup_method(self):
        from robot_radio.testgui.traces import TraceModel
        self.model = TraceModel()

    def test_rw_zero_rotation_identity(self):
        """rot=0 behaves exactly like _tw (fresh-anchor case, baseline hdg=anchor_h)."""
        m = self.model
        m.anchor(0.0, 0.0, 0.0)
        assert m._rw(10.0, 5.0, 0.0) == pytest.approx(m._tw(10.0, 5.0))

    def test_rw_90_degree_rotation(self):
        """A +90° rotation maps firmware +x to world +y."""
        m = self.model
        m.anchor(0.0, 0.0, 0.0)
        wx, wy = m._rw(10.0, 0.0, math.pi / 2)
        assert wx == pytest.approx(0.0, abs=1e-6)
        assert wy == pytest.approx(10.0, abs=1e-6)

    def test_rw_negative_90_degree_rotation(self):
        """A -90° rotation maps firmware +y to world +x."""
        m = self.model
        m.anchor(0.0, 0.0, 0.0)
        wx, wy = m._rw(0.0, 10.0, -math.pi / 2)
        assert wx == pytest.approx(10.0, abs=1e-6)
        assert wy == pytest.approx(0.0, abs=1e-6)

    def test_rw_with_anchor_offset(self):
        """Anchor world offset is added after rotation."""
        m = self.model
        m.anchor(5.0, 3.0, 0.0)
        wx, wy = m._rw(0.0, 0.0, math.pi / 2)
        assert wx == pytest.approx(5.0)
        assert wy == pytest.approx(3.0)
