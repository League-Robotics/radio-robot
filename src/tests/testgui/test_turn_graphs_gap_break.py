"""src/tests/testgui/test_turn_graphs_gap_break.py -- defect 2b (2026-07-22
GUI stakeholder report): strip charts must never draw an interpolating line
across a real, un-recorded telemetry gap.

Reported symptom (paired with defect 2a, the continuous-recording fix in
``test_turn_graphs_persistence.py``): a ~3.5s idle window between two turns
had no recorded samples, so the wheel-speed strip chart drew a straight
line across the gap -- ``cmd`` appeared to ramp smoothly from 0 up to 130
over several seconds, when the robot had actually already crawled to a
stop and stayed there. The stakeholder's own rule: "if the motor stopped,
record the time at which it stopped as the time at which you break that
line" -- not the time the next sample happens to arrive.

Fix (``turn_graphs.py``): ``TurnTraceRecorder._append_series()`` inserts a
NaN sentinel, timestamped at the PRIOR point's own time, whenever the gap
since a series' last point exceeds ``GAP_BREAK_FACTOR *
_NOMINAL_TLM_PERIOD``. matplotlib does not draw a line segment across a
NaN y-value, so the plotted line breaks exactly there instead of
interpolating -- both the full-history ``_GraphCanvas`` view and the
trailing-window ``StripChartCanvas`` view (110-002) get this for free,
since both simply read ``TurnTraceRecorder.series``.

This module covers the gap-break mechanism directly (Qt-free, on
``TurnTraceRecorder``/``_append_series`` via the public ``add_tlm()``
surface) and end-to-end through a real ``StripChartCanvas`` redraw
(confirming the plotted line's y-data actually contains the NaN and would
therefore render as a broken/split segment, not an interpolated ramp).

Run with::

    QT_QPA_PLATFORM=offscreen uv run python -m pytest src/tests/testgui/test_turn_graphs_gap_break.py -v
"""
from __future__ import annotations

import math
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


def _tlm(vel, cmd_vel=None, active=True):
    from robot_radio.robot.protocol import TLMFrame

    return TLMFrame(active=active, vel=vel, cmd_vel=cmd_vel)


# ---------------------------------------------------------------------------
# Qt-free: TurnTraceRecorder._append_series() gap-break mechanism
# ---------------------------------------------------------------------------


class TestAppendSeriesGapBreak:
    def test_closely_spaced_samples_never_break(self):
        """Ordinary sample-to-sample jitter (well under the gap-break
        threshold) never inserts a NaN -- the common case must stay a
        plain, unbroken series."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder

        rec = TurnTraceRecorder()
        t = 0.0
        for i in range(20):
            t += 0.05  # 50ms apart -- well under the ~0.33s gap-break threshold
            rec.add_tlm(t, _tlm((100.0 + i, 100.0 + i)))

        vel_l = rec.series["vel_l"]
        assert len(vel_l) == 20, f"no NaN sentinels expected, got {vel_l}"
        assert not any(math.isnan(v) for _, v in vel_l)

    def test_large_gap_inserts_nan_sentinel_before_the_next_point(self):
        """A gap well past GAP_BREAK_FACTOR * the nominal TLM period gets a
        NaN sentinel woven in immediately before the next real point --
        exactly one extra entry, timestamped at (approximately) the PRIOR
        point's own time, not the resumed time."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder

        rec = TurnTraceRecorder()
        rec.add_tlm(0.0, _tlm((100.0, 100.0)))
        rec.add_tlm(0.05, _tlm((100.0, 100.0)))

        # A 3.5s idle gap -- the exact magnitude from the stakeholder's own
        # reported session.
        rec.add_tlm(3.55, _tlm((0.0, 0.0)))

        vel_l = rec.series["vel_l"]
        assert len(vel_l) == 4, f"expected 2 real + 1 NaN break + 1 real, got {vel_l}"

        t2, nan_val = vel_l[2]
        assert math.isnan(nan_val), f"expected a NaN sentinel at index 2, got {vel_l[2]}"
        # Timestamped at (a hair past) the PRIOR point's own time -- "record
        # the time at which it stopped as the time at which you break that
        # line," not the time the next sample arrives.
        assert t2 == pytest.approx(0.05, abs=1e-3)

        t3, real_val = vel_l[3]
        assert real_val == 0.0
        assert t3 == pytest.approx(3.55, abs=1e-3)

        # Strictly increasing timestamps throughout -- the NaN sentinel
        # never collides with or precedes the point before it.
        times = [t_ for t_, _ in vel_l]
        assert times == sorted(times) and len(set(times)) == len(times)

    def test_gap_break_is_per_series_not_global(self):
        """cmd_l/cmd_r only append on frames that carry a commanded
        velocity -- their own gap-break clock runs independently of
        vel_l/vel_r's, which append on every frame. A big real-world-time
        gap between two CONSECUTIVE cmd-carrying frames breaks cmd_l/cmd_r
        even though vel_l/vel_r (fed every frame in between) never see a
        gap at all."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder

        rec = TurnTraceRecorder()
        rec.add_tlm(0.0, _tlm((100.0, 100.0), cmd_vel=(100.0, 100.0)))
        # Several frames with NO commanded velocity, closely spaced --
        # vel_l/vel_r keep appending every time, no gap ever forms there.
        t = 0.0
        for i in range(60):
            t += 0.05
            rec.add_tlm(t, _tlm((5.0, 5.0)))
        # Finally another commanded-velocity frame -- the gap since the
        # LAST cmd_l point (t=0.0) is large (~3s), even though vel_l itself
        # was fed continuously throughout.
        rec.add_tlm(t + 0.05, _tlm((100.0, 100.0), cmd_vel=(130.0, 130.0)))

        vel_l = rec.series["vel_l"]
        assert not any(math.isnan(v) for _, v in vel_l), (
            "vel_l was fed every frame -- it must never see a gap-break")

        cmd_l = rec.series["cmd_l"]
        assert any(math.isnan(v) for _, v in cmd_l), (
            f"cmd_l skipped many frames -- expected a gap-break NaN, got {cmd_l}")

    def test_gap_break_factor_and_nominal_period_are_documented_constants(self):
        """Sanity check the tunable constants the docstrings above reference
        actually exist with sane values (a >0 multiplier of a >0 period)."""
        from robot_radio.testgui.turn_graphs import GAP_BREAK_FACTOR, _NOMINAL_TLM_PERIOD

        assert GAP_BREAK_FACTOR > 1
        assert 0.0 < _NOMINAL_TLM_PERIOD < 1.0


# ---------------------------------------------------------------------------
# End-to-end: a real StripChartCanvas redraw actually breaks the line
# ---------------------------------------------------------------------------


class TestStripChartRendersGapAsBrokenLine:
    def test_redrawn_canvas_ydata_contains_nan_at_the_gap(self, qapp):
        """Feed a recorder through the exact gap sequence from the
        stakeholder's report, redraw a real StripChartCanvas from it, and
        confirm the plotted wheel-speed line's y-data actually contains the
        NaN -- i.e. matplotlib will render a broken segment there, not an
        interpolated ramp between the pre-gap and post-gap values."""
        from robot_radio.testgui.turn_graphs import (
            WHEEL_SPEED, StripChartCanvas, TurnTraceRecorder,
        )

        rec = TurnTraceRecorder()
        t = 0.0
        for i in range(10):
            t += 0.05
            rec.add_tlm(t, _tlm((100.0 + i, 100.0 + i), cmd_vel=(100.0, 100.0)))

        # The ~3.5s idle gap -- no samples recorded pre-fix; now recorded
        # continuously, so this call both appends a real (near-zero) point
        # AND triggers the gap-break NaN ahead of it.
        t += 3.5
        rec.add_tlm(t, _tlm((0.0, 0.0)))

        for i in range(5):
            t += 0.05
            rec.add_tlm(t, _tlm((130.0 - i, 130.0 - i), cmd_vel=(130.0, 130.0)))

        canvas = StripChartCanvas("Wheel speed", "mm/s", WHEEL_SPEED, window=30.0)
        try:
            canvas.redraw(rec)
            assert canvas._ax.lines, "expected at least one plotted series"

            vel_l_line = None
            for line in canvas._ax.lines:
                if line.get_label() == "actual L":
                    vel_l_line = line
                    break
            assert vel_l_line is not None, "expected the 'actual L' (vel_l) line to be plotted"

            ydata = vel_l_line.get_ydata()
            nan_count = sum(1 for y in ydata if isinstance(y, float) and math.isnan(y))
            assert nan_count >= 1, (
                f"expected a NaN break in the plotted vel_l line's y-data "
                f"(matplotlib breaks the line here instead of interpolating "
                f"across the gap), got ydata={list(ydata)}"
            )
        finally:
            canvas.deleteLater()
