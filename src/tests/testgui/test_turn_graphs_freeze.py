"""src/tests/testgui/test_turn_graphs_freeze.py -- 2026-07-22 GUI
stakeholder directive, stated live and repeated: "You have to stop the
charts. Do not run the charts when the robot is stopped. Make it stop."

Covers the freeze/resume state machine in ``turn_graphs.TurnTraceRecorder``:

  * ``add_tlm()`` appends nothing at all -- no new point in any series --
    while the robot is genuinely idle (no active move AND every wheel's
    measured velocity under ``_MOVING_SPEED``), and resumes appending the
    instant real motion is observed again. No debounce: the freeze and the
    resume are both immediate, a pure function of the CURRENT frame.
  * ``self._robot_moving`` is the SINGLE shared freeze predicate both
    ``add_tlm()`` and ``add_camera()`` consult (2026-07-22 follow-up,
    stakeholder-reported: the first cut only gated ``add_tlm()``'s own
    series, so ``add_camera()`` kept appending during an idle span and
    ``latest_t()`` -- read by ``StripChartCanvas`` for its trailing-window
    anchor -- kept climbing off camera updates alone, so the
    telemetry-panel's rolling strip charts kept visibly scrolling even
    while the top, full-history graphs looked correctly frozen). This
    module asserts that leak is closed: camera samples fed during an idle
    span must not advance ``latest_t()`` either.
  * Resuming after a frozen span weaves in a gap-break NaN sentinel ahead
    of the first new point (defect 2b, ``test_turn_graphs_gap_break.py``'s
    own focus) -- this module only asserts the sentinel is present where
    expected, not the full gap-break mechanism (covered there).
  * ``TurnGraphPanel.add_tlm()`` only flags its redraw-dirty bit on frames
    that were actually recorded, so a frozen chart does not even attempt a
    redraw -- "no axis advance, nothing moves" all the way through, not
    just at the data layer.

Run with::

    QT_QPA_PLATFORM=offscreen uv run python -m pytest src/tests/testgui/test_turn_graphs_freeze.py -v
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


def _tlm(active, vel):
    from robot_radio.robot.protocol import TLMFrame

    return TLMFrame(active=active, vel=vel)


# ---------------------------------------------------------------------------
# add_tlm(): freeze / resume, motion detection
# ---------------------------------------------------------------------------


class TestAddTlmFreezeResume:
    def test_idle_frames_append_nothing_and_return_false(self):
        """A robot that never moves (active False, velocity ~0 throughout)
        must never grow any series -- every call returns False."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder

        rec = TurnTraceRecorder()
        t = 0.0
        for _ in range(10):
            t += 0.05
            recorded = rec.add_tlm(t, _tlm(False, (0.0, 0.0)))
            assert recorded is False

        assert rec.series["vel_l"] == []
        assert rec.series["vel_r"] == []
        assert rec.latest_t() is None, "nothing was ever recorded -- latest_t() must stay None"

    def test_freeze_is_immediate_no_debounce(self):
        """The VERY FIRST idle frame right after a moving frame is already
        skipped -- no grace period, no latched 'still cooling down' state."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder

        rec = TurnTraceRecorder()
        rec.add_tlm(0.0, _tlm(True, (100.0, 100.0)))
        assert len(rec.series["vel_l"]) == 1

        # The NEXT frame, only 20ms later, already reports idle -- must
        # freeze immediately, not wait out some hold time.
        recorded = rec.add_tlm(0.02, _tlm(False, (0.0, 0.0)))
        assert recorded is False
        assert len(rec.series["vel_l"]) == 1, "freeze must engage on the very next idle frame"

    def test_resume_is_immediate_no_debounce(self):
        """Symmetric to the freeze: the VERY FIRST moving frame after an
        idle span resumes appending immediately."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder

        rec = TurnTraceRecorder()
        rec.add_tlm(0.0, _tlm(True, (100.0, 100.0)))
        for i in range(5):
            rec.add_tlm(0.1 * (i + 1), _tlm(False, (0.0, 0.0)))
        assert len(rec.series["vel_l"]) == 1, "still frozen through the idle span"

        recorded = rec.add_tlm(1.0, _tlm(True, (80.0, 80.0)))
        assert recorded is True
        # +1 real point, plus a gap-break NaN ahead of it (the idle span
        # was long enough to cross the gap-break threshold).
        assert len(rec.series["vel_l"]) == 3
        assert math.isnan(rec.series["vel_l"][1][1])
        assert rec.series["vel_l"][2][1] == 80.0

    def test_active_flag_alone_counts_as_motion_even_at_zero_velocity(self):
        """A frame with active=True but a wheel velocity still near zero
        (e.g. the very start of a Move, before the wheels spin up) must
        NOT be treated as idle -- the active flag is authoritative on its
        own (the OR in the moving predicate)."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder

        rec = TurnTraceRecorder()
        recorded = rec.add_tlm(0.0, _tlm(True, (0.0, 0.0)))
        assert recorded is True
        assert len(rec.series["vel_l"]) == 1

    def test_velocity_threshold_alone_counts_as_motion_when_active_is_none(self):
        """Older/pre-fault frames that never set `active` (None, not
        False) must still be treated as moving when a wheel's measured
        velocity clears the threshold -- the velocity check is the
        fallback, not a second gate that must ALSO pass."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder
        from robot_radio.robot.protocol import TLMFrame

        rec = TurnTraceRecorder()
        frame = TLMFrame(vel=(50.0, 50.0))   # active left at its None default
        assert frame.active is None
        recorded = rec.add_tlm(0.0, frame)
        assert recorded is True
        assert len(rec.series["vel_l"]) == 1

    def test_velocity_at_or_below_threshold_with_no_active_flag_is_idle(self):
        """The boundary case: active is None/False and every wheel is at
        or under `_MOVING_SPEED` -- idle, nothing recorded."""
        from robot_radio.testgui.turn_graphs import _MOVING_SPEED, TurnTraceRecorder

        rec = TurnTraceRecorder()
        recorded = rec.add_tlm(0.0, _tlm(False, (_MOVING_SPEED, _MOVING_SPEED)))
        assert recorded is False
        assert rec.series["vel_l"] == []

    def test_never_auto_clears_across_a_freeze_resume_cycle(self):
        """110-001's own invariant, still holding under the reinstated
        freeze: a resume after an idle span APPENDS to the same series,
        never wipes it."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder

        rec = TurnTraceRecorder()
        t = 0.0
        for i in range(6):
            t += 0.05
            rec.add_tlm(t, _tlm(True, (100.0 + i, 100.0 + i)))
        first_burst = [v for _, v in rec.series["vel_l"]]
        assert first_burst == [100.0 + i for i in range(6)]

        for _ in range(10):
            t += 0.1
            rec.add_tlm(t, _tlm(False, (0.0, 0.0)))

        t += 0.05
        rec.add_tlm(t, _tlm(True, (200.0, 200.0)))

        values = [v for _, v in rec.series["vel_l"]]
        assert values[:6] == first_burst, "the first burst must survive the idle span unmutated"
        assert values[-1] == 200.0, "the resumed point must be appended, not replacing history"


# ---------------------------------------------------------------------------
# Single shared freeze predicate: add_camera() must respect it too
# ---------------------------------------------------------------------------


class TestSharedFreezePredicateCoversCamera:
    def test_camera_samples_during_idle_do_not_append_or_advance_latest_t(self):
        """The 2026-07-22 follow-up fix: camera ground-truth updates arrive
        on their OWN polling cadence, independent of wheel telemetry -- if
        add_camera() kept appending while add_tlm() was frozen,
        recorder.latest_t() (the max across EVERY series) would keep
        climbing off camera data alone, and StripChartCanvas's
        trailing-window anchor (built on latest_t()) would keep sliding
        forward even though every wheel series was correctly frozen --
        exactly the reported 'the main chart stopped, but the one in the
        telemetry section keeps going' symptom."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder

        rec = TurnTraceRecorder()
        t = 0.0
        for i in range(5):
            t += 0.05
            rec.add_tlm(t, _tlm(True, (100.0, 100.0)))
            rec.add_camera(t, 0.0, 0.0, 0.0)

        latest_before = rec.latest_t()
        dist_cam_before = len(rec.series["dist_cam"])

        # Robot goes idle -- but the camera keeps pinging at its own
        # (faster) cadence for a while, same as a real aprilcam feed would.
        for i in range(20):
            t += 0.02
            rec.add_tlm(t, _tlm(False, (0.0, 0.0)))
            rec.add_camera(t, 0.0, 0.0, 0.0)

        assert len(rec.series["dist_cam"]) == dist_cam_before, (
            "camera samples must not append while the robot is idle, even "
            "though the camera feed itself kept arriving")
        assert rec.latest_t() == latest_before, (
            f"latest_t() must stay pinned while frozen -- it must not be "
            f"pushed forward by an ungated series (camera); "
            f"was {latest_before}, now {rec.latest_t()}")

        # Resume: both wheel and camera series pick back up. The idle span
        # was long enough to also cross dist_cam's OWN gap-break threshold,
        # so its resume weaves in a NaN sentinel too (defect 2b) -- +1 NaN
        # + 1 real point.
        t += 0.05
        rec.add_tlm(t, _tlm(True, (100.0, 100.0)))
        rec.add_camera(t, 5.0, 0.0, 0.0)
        assert len(rec.series["dist_cam"]) == dist_cam_before + 2
        assert math.isnan(rec.series["dist_cam"][dist_cam_before][1])
        assert rec.latest_t() > latest_before

    def test_camera_before_any_telemetry_does_not_append(self):
        """A camera sample that arrives before the first TLM frame has
        neither a t0 anchor nor a known moving state -- must be a no-op,
        not a crash."""
        from robot_radio.testgui.turn_graphs import TurnTraceRecorder

        rec = TurnTraceRecorder()
        rec.add_camera(0.0, 1.0, 2.0, 0.0)
        assert rec.series["dist_cam"] == []


# ---------------------------------------------------------------------------
# TurnGraphPanel: the redraw-dirty flag mirrors the freeze
# ---------------------------------------------------------------------------


class TestPanelDirtyFlagMirrorsFreeze:
    def test_panel_stays_clean_while_frozen_and_dirties_on_resume(self, qapp):
        from robot_radio.testgui.turn_graphs import TurnGraphPanel

        panel = TurnGraphPanel()
        try:
            panel._dirty = False
            panel.add_tlm(0.0, _tlm(True, (100.0, 100.0)))
            assert panel._dirty is True, "a moving frame must dirty the panel"

            panel._dirty = False   # simulate the throttled timer having redrawn already
            for i in range(5):
                panel.add_tlm(0.05 * (i + 1), _tlm(False, (0.0, 0.0)))
            assert panel._dirty is False, (
                "idle frames must never flag the panel dirty -- no redraw "
                "attempt while frozen, matching 'nothing moves'")

            panel.add_tlm(1.0, _tlm(True, (90.0, 90.0)))
            assert panel._dirty is True, "the resume frame must dirty the panel again"
        finally:
            panel.deleteLater()
