"""src/tests/testgui/test_shaping_disabled_indicator.py -- 119 ticket 001
(kill-the-silent-off-shaping-config-boundary.md): the TestGUI status-bar
banner + log line for flags bit 16 (``kFlagFaultShapingDisabled`` /
``TLMFrame.fault_shaping_disabled``) -- the loud off-state for a feature
that used to have a silent, invisible off state.

``__main__.py``'s ``_TelemetryBridge.on_frame_ready`` is a closure inside
``_build_main_window()`` with no test seam -- per the established pattern in
``test_telemetry_gating.py``/``test_set_origin.py``/``test_tour_stop.py``,
the edge-detection/banner-visibility/log-line logic is re-implemented inline
here (Qt-free) and verified against real ``TLMFrame`` objects with the real
``fault_shaping_disabled`` property, mirroring the production closure's own
docstring exactly. A separate offscreen-Qt class confirms the real banner
widget exists in the built window with the right objectName and starts
hidden (mirrors ``test_mode_indicator.py``'s own ``test_mode_label_exists``
shape).

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_shaping_disabled_indicator.py -v
"""
from __future__ import annotations

import queue

import pytest


# ---------------------------------------------------------------------------
# Fake doubles (Qt-free)
# ---------------------------------------------------------------------------


class _FakeBanner:
    """Stand-in for the QLabel -- records every setVisible() call."""

    def __init__(self) -> None:
        self.visible_calls: "list[bool]" = []

    def setVisible(self, visible: bool) -> None:  # noqa: N802 -- mirrors Qt's own method name
        self.visible_calls.append(visible)

    @property
    def visible(self) -> "bool | None":
        return self.visible_calls[-1] if self.visible_calls else None


def _make_frame(fault_shaping_disabled: bool = False):
    """A minimal duck-typed TLMFrame stand-in -- only the one attribute this
    logic reads matters here (mirrors test_telemetry_gating.py's own
    ``_make_frame()`` shape, which likewise builds a bare, mostly-empty
    frame for a gating test that only cares about one field)."""
    from types import SimpleNamespace
    return SimpleNamespace(fault_shaping_disabled=fault_shaping_disabled)


def _make_shaping_disabled_check(state: dict, banner, append_log, pending_frames):
    """Re-implements the shaping-disabled slice of
    ``_TelemetryBridge.on_frame_ready``'s per-frame loop + its
    post-loop banner update -- mirrors the production code in
    ``src/host/robot_radio/testgui/__main__.py::_TelemetryBridge.on_frame_ready``
    exactly (edge-triggered log line via ``_state["shaping_disabled_active"]``,
    level-set banner visibility from the LAST drained frame)."""

    def run() -> None:
        any_frame = False
        while True:
            try:
                frame = pending_frames.get_nowait()
            except Exception:
                break
            any_frame = True
            shaping_disabled_now = bool(getattr(frame, "fault_shaping_disabled", False))
            if shaping_disabled_now != state.get("shaping_disabled_active", False):
                state["shaping_disabled_active"] = shaping_disabled_now
                if shaping_disabled_now:
                    append_log(
                        "[SHAPE] flags bit 16 (kFlagFaultShapingDisabled) SET -- MOVE "
                        "active with shaping/anticipation OFF on both axes; land-at-zero "
                        "cannot fire, threshold/timeout backstop is the only completion path"
                    )
                else:
                    append_log("[SHAPE] flags bit 16 cleared -- shaping active again")
        if any_frame:
            banner.setVisible(state.get("shaping_disabled_active", False))

    return run


# ---------------------------------------------------------------------------
# Edge-triggered logging -- never logs every frame, only the transition
# ---------------------------------------------------------------------------


class TestEdgeTriggeredLogging:
    def test_no_log_line_when_bit_never_asserts(self):
        state = {"shaping_disabled_active": False}
        banner = _FakeBanner()
        log: "list[str]" = []
        pending: "queue.Queue" = queue.Queue()
        for _ in range(5):
            pending.put(_make_frame(fault_shaping_disabled=False))

        run = _make_shaping_disabled_check(state, banner, log.append, pending)
        run()

        assert log == [], f"expected no log lines with the bit never set, got {log}"
        assert banner.visible is False

    def test_single_log_line_on_rising_edge_even_across_a_burst(self):
        """A whole burst of frames where the bit is set on every one of them
        (a MOVE that has been running unshaped for a while) logs exactly
        ONE line, not one per frame -- flooding the log at cycle rate is
        exactly the failure mode this design avoids."""
        state = {"shaping_disabled_active": False}
        banner = _FakeBanner()
        log: "list[str]" = []
        pending: "queue.Queue" = queue.Queue()
        for _ in range(20):
            pending.put(_make_frame(fault_shaping_disabled=True))

        run = _make_shaping_disabled_check(state, banner, log.append, pending)
        run()

        assert len(log) == 1, f"expected exactly one rising-edge log line, got {log}"
        assert "SET" in log[0]
        assert banner.visible is True

    def test_log_line_on_falling_edge(self):
        state = {"shaping_disabled_active": True}  # already set from a prior drain
        banner = _FakeBanner()
        log: "list[str]" = []
        pending: "queue.Queue" = queue.Queue()
        pending.put(_make_frame(fault_shaping_disabled=False))

        run = _make_shaping_disabled_check(state, banner, log.append, pending)
        run()

        assert len(log) == 1
        assert "cleared" in log[0]
        assert banner.visible is False

    def test_set_then_clear_within_one_burst_logs_both_edges(self):
        state = {"shaping_disabled_active": False}
        banner = _FakeBanner()
        log: "list[str]" = []
        pending: "queue.Queue" = queue.Queue()
        pending.put(_make_frame(fault_shaping_disabled=True))
        pending.put(_make_frame(fault_shaping_disabled=True))
        pending.put(_make_frame(fault_shaping_disabled=False))

        run = _make_shaping_disabled_check(state, banner, log.append, pending)
        run()

        assert len(log) == 2
        assert "SET" in log[0]
        assert "cleared" in log[1]
        # Banner reflects the LAST frame's state (level), which is clear here.
        assert banner.visible is False

    def test_no_frames_drained_leaves_banner_untouched(self):
        """An empty drain (on_frame_ready fired with nothing queued) must
        not touch the banner at all -- mirrors the production `if
        any_frame:` guard exactly (the canvas-refresh gate uses the same
        guard for the same reason)."""
        state = {"shaping_disabled_active": True}
        banner = _FakeBanner()
        log: "list[str]" = []
        pending: "queue.Queue" = queue.Queue()

        run = _make_shaping_disabled_check(state, banner, log.append, pending)
        run()

        assert log == []
        assert banner.visible_calls == [], "banner must not be touched on an empty drain"

    def test_missing_state_key_defaults_to_not_disabled(self):
        """No 'shaping_disabled_active' key at all (fresh _state dict, first
        frame ever) behaves as if it were False -- a first frame WITH the
        bit set is still a rising edge and logs once."""
        state: dict = {}
        banner = _FakeBanner()
        log: "list[str]" = []
        pending: "queue.Queue" = queue.Queue()
        pending.put(_make_frame(fault_shaping_disabled=True))

        run = _make_shaping_disabled_check(state, banner, log.append, pending)
        run()

        assert len(log) == 1 and "SET" in log[0]
        assert banner.visible is True


# ---------------------------------------------------------------------------
# Real production code -- TLMFrame actually exposes fault_shaping_disabled
# ---------------------------------------------------------------------------


class TestRealTLMFrameExposesTheProperty:
    """Sanity check against the real TLMFrame (not the duck-typed fake
    above) so the gating tests stay honest about what production code
    actually exposes."""

    def test_fault_shaping_disabled_property_exists_and_defaults_false(self):
        from robot_radio.robot.protocol import TLMFrame

        frame = TLMFrame()
        assert frame.fault_shaping_disabled is False

    def test_fault_shaping_disabled_reads_bit_16(self):
        from robot_radio.robot.protocol import TLMFrame

        frame = TLMFrame(flags=1 << 16)
        assert frame.fault_shaping_disabled is True

    def test_fault_shaping_disabled_independent_of_other_bits(self):
        """``active`` is a plain dataclass field populated only by
        ``from_pb2()`` (not flags-derived, unlike the fault/event
        properties) -- ``fault_move_timeout`` (bit 15) is the adjacent
        flags-derived property this test compares bit 16 against."""
        from robot_radio.robot.protocol import TLMFrame

        # bit 15 (fault_move_timeout) set, bit 16 clear.
        frame = TLMFrame(flags=(1 << 15))
        assert frame.fault_move_timeout is True
        assert frame.fault_shaping_disabled is False

        # bit 16 set, bit 15 clear -- the two do not shadow each other.
        frame2 = TLMFrame(flags=(1 << 16))
        assert frame2.fault_move_timeout is False
        assert frame2.fault_shaping_disabled is True


# ---------------------------------------------------------------------------
# Qt widget test -- the real banner exists, hidden by default
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def qapp():
    """Ensure a QApplication exists for the whole module.

    ``QT_QPA_PLATFORM=offscreen`` is already set by conftest.py before this
    import runs.
    """
    pytest.importorskip("PySide6")
    import sys

    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


class TestShapingDisabledBannerWidget:
    def test_banner_exists_with_expected_object_name(self, qapp):
        from PySide6.QtWidgets import QLabel  # type: ignore[import-untyped]

        from robot_radio.testgui.__main__ import _build_main_window

        window, _ = _build_main_window()
        banner = window.findChild(QLabel, "shaping_disabled_banner")
        assert banner is not None, (
            "expected a QLabel with objectName 'shaping_disabled_banner'"
        )
        window.close()

    def test_banner_hidden_by_default(self, qapp):
        """``isHidden()``, not ``isVisible()`` -- the window built by
        ``_build_main_window()`` in this test is never ``show()``n, so
        EVERY child widget's ``isVisible()`` would read False regardless of
        the banner's own explicit state (Qt: visibility requires every
        ancestor to be shown too). ``isHidden()`` reflects the WIDGET's own
        explicit ``setVisible(False)``/``hide()`` state independent of
        ancestor visibility -- the actual thing production code sets."""
        from PySide6.QtWidgets import QLabel  # type: ignore[import-untyped]

        from robot_radio.testgui.__main__ import _build_main_window

        window, _ = _build_main_window()
        banner = window.findChild(QLabel, "shaping_disabled_banner")
        assert banner is not None
        assert banner.isHidden() is True, (
            "the shaping-disabled banner must start hidden -- shown only once a "
            "frame actually carries the bit"
        )
        window.close()
