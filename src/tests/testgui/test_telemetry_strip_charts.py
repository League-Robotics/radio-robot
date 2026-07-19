"""src/tests/testgui/test_telemetry_strip_charts.py -- ticket 110-002:
rolling 10-second strip charts in the telemetry pane.

Covers:
  * ``turn_graphs.TurnTraceRecorder.latest_t()`` -- the Qt-free "now"
    reference a trailing-window view anchors its cutoff to.
  * ``turn_graphs.StripChartCanvas`` -- a windowing FILTER over the SAME
    recorder the (full-history) top graphs read: feeding more than 10 s of
    synthetic frames leaves only the trailing 10 s on the strip-chart
    canvas's own plotted lines, while ``recorder.series`` itself (and by
    extension the unwindowed top-graph view) still has every point.
  * ``build_telemetry_panel(recorder=...)`` wiring: the strip-chart tabs
    read the recorder passed in by the caller, not a private one -- no
    second recorder, no second telemetry-consumption path.

Run with::

    QT_QPA_PLATFORM=offscreen uv run python -m pytest src/tests/testgui/test_telemetry_strip_charts.py -v
"""
from __future__ import annotations

import sys

import pytest


@pytest.fixture(scope="session")
def qapp():
    # See test_telemetry_panel.py's identical fixture docstring: turns a
    # missing `gui` dependency group into a clean skip, not a hard
    # collection/run error.
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


# ---------------------------------------------------------------------------
# Qt-free: TurnTraceRecorder.latest_t()
# ---------------------------------------------------------------------------


def test_latest_t_is_none_for_empty_recorder():
    from robot_radio.testgui.turn_graphs import TurnTraceRecorder

    rec = TurnTraceRecorder()
    assert rec.latest_t() is None


def test_latest_t_is_max_across_all_series():
    """``latest_t()`` reports the recorder's own ELAPSED time (``t - t0``,
    the same x-axis every series/graph already plots against), not the raw
    wall-clock ``now`` passed to ``add_tlm`` -- the first ``add_tlm`` call
    establishes ``t0``, so the final elapsed value is ``now - t0``."""
    from robot_radio.robot.protocol import TLMFrame
    from robot_radio.testgui.turn_graphs import TurnTraceRecorder

    rec = TurnTraceRecorder()
    now = 0.0
    t0 = None
    for i in range(5):
        now += 1.0
        if t0 is None:
            t0 = now
        rec.add_tlm(now, TLMFrame(active=True, vel=(50.0, 50.0), enc=(i * 2.0, i * 2.0)))

    assert rec.latest_t() == pytest.approx(now - t0)


# ---------------------------------------------------------------------------
# StripChartCanvas windowing
# ---------------------------------------------------------------------------


def _feed_moving_frames(recorder, count: int, dt: float, start_t: float = 0.0):
    """Feed `count` synthetic moving TLM frames `dt` seconds apart, starting
    at `start_t`.  Returns the final elapsed time."""
    from robot_radio.robot.protocol import TLMFrame

    t = start_t
    for i in range(count):
        t += dt
        recorder.add_tlm(
            t, TLMFrame(active=True, vel=(100.0 + i, 100.0 + i), enc=(i * 5.0, i * 5.0)))
    return t


class TestStripChartWindowing:
    def test_strip_chart_only_plots_trailing_window_while_recorder_keeps_everything(self, qapp):
        from robot_radio.testgui.turn_graphs import (
            WHEEL_SPEED, StripChartCanvas, TurnTraceRecorder,
        )

        rec = TurnTraceRecorder()
        # 30 frames, 0.5s apart -> 15s of history, well past the 10s window.
        _feed_moving_frames(rec, count=30, dt=0.5)

        # The (Qt-free) recorder itself has EVERY point -- the unaffected
        # top-graph view's own data source.
        assert len(rec.series["vel_l"]) == 30

        canvas = StripChartCanvas("Wheel speed", "mm/s", WHEEL_SPEED, window=10.0)
        try:
            canvas.redraw(rec)

            # Every plotted line on the strip-chart canvas must only carry
            # points from the trailing 10s -- none older.
            now = rec.latest_t()
            cutoff = now - 10.0
            assert canvas._ax.lines, "expected at least one plotted series"
            for line in canvas._ax.lines:
                xs = line.get_xdata()
                assert len(xs) > 0
                assert all(x >= cutoff - 1e-9 for x in xs), (
                    f"strip chart plotted a point older than the 10s window: {xs}")
                # And the window actually excluded SOMETHING (not a no-op) --
                # the full recorder series is strictly longer than what's
                # plotted for the same key.
            plotted_lens = [len(line.get_xdata()) for line in canvas._ax.lines]
            assert max(plotted_lens) < 30, (
                "strip chart should have windowed out older points, but "
                f"plotted as many as the full recorder history: {plotted_lens}")
        finally:
            canvas.deleteLater()

    def test_strip_chart_with_less_than_window_seconds_shows_everything(self, qapp):
        from robot_radio.testgui.turn_graphs import (
            WHEEL_SPEED, StripChartCanvas, TurnTraceRecorder,
        )

        rec = TurnTraceRecorder()
        _feed_moving_frames(rec, count=5, dt=0.1)  # only 0.5s of history

        canvas = StripChartCanvas("Wheel speed", "mm/s", WHEEL_SPEED, window=10.0)
        try:
            canvas.redraw(rec)
            assert canvas._ax.lines
            for line in canvas._ax.lines:
                assert len(line.get_xdata()) == 5
        finally:
            canvas.deleteLater()


# ---------------------------------------------------------------------------
# build_telemetry_panel(recorder=...) wiring -- reuse, don't duplicate
# ---------------------------------------------------------------------------


class TestTelemetryPanelStripChartWiring:
    def test_panel_has_strip_chart_tabs(self, qapp):
        from PySide6.QtWidgets import QTabWidget

        from robot_radio.testgui.telemetry_panel import build_telemetry_panel

        widget, _ctrl = build_telemetry_panel()
        try:
            tabs = widget.findChild(QTabWidget, "telemetry_strip_charts")
            assert tabs is not None
            names = {tabs.tabText(i) for i in range(tabs.count())}
            # Twist added 2026-07-18 (commanded vs actual body v_x/omega).
            assert names == {"Wheel speed", "Twist", "Wheel position", "Heading", "Distance"}
        finally:
            widget.deleteLater()

    def test_panel_strip_charts_read_the_caller_supplied_recorder(self, qapp):
        """No second recorder: passing a recorder to build_telemetry_panel()
        makes the strip charts read exactly that object -- the SAME one a
        caller's TurnGraphPanel already owns and feeds."""
        from robot_radio.testgui.telemetry_panel import build_telemetry_panel
        from robot_radio.testgui.turn_graphs import StripChartCanvas, TurnTraceRecorder

        shared_recorder = TurnTraceRecorder()
        _feed_moving_frames(shared_recorder, count=8, dt=0.2)

        widget, _ctrl = build_telemetry_panel(recorder=shared_recorder)
        try:
            speed_canvas = widget.findChild(StripChartCanvas, "strip_chart_wheel_speed")
            assert speed_canvas is not None
            speed_canvas.redraw(shared_recorder)
            assert speed_canvas._ax.lines
            for line in speed_canvas._ax.lines:
                assert len(line.get_xdata()) == 8
        finally:
            widget.deleteLater()

    def test_panel_without_recorder_builds_its_own_empty_one(self, qapp):
        """build_telemetry_panel() with no recorder arg must still build
        (a standalone panel test, or any future caller that doesn't share
        one) -- with an empty private recorder, not a crash."""
        from robot_radio.testgui.telemetry_panel import build_telemetry_panel
        from robot_radio.testgui.turn_graphs import StripChartCanvas

        widget, _ctrl = build_telemetry_panel()
        try:
            speed_canvas = widget.findChild(StripChartCanvas, "strip_chart_wheel_speed")
            assert speed_canvas is not None
            # No crash on redraw with nothing recorded yet.
        finally:
            widget.deleteLater()
