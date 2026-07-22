"""src/tests/testgui/test_turn_graphs_persistence.py -- ticket 110-001:
graph-tab data persistence across view switches.

Reported symptom (``testgui-graphs-not-persistent-on-view-switch.md``):
switching the TestGUI's four live graph tabs away and back "corrupts" the
earlier tab's history -- "the existing series is deleted and then
repopulated with wrong data."

**Reproduction finding**: sprint-planning-time static reading of
``turn_graphs.py`` did NOT turn up an obvious cause (each tab owns its own
canvas/axes over one shared, persistent ``TurnTraceRecorder``, and nothing
in the tab-switch path touches the recorder). Building a REAL repro that
exercises a continuous telemetry stream (not a static dataset) found the
actual bug: ``TurnTraceRecorder.add_tlm()`` used to auto-``clear()`` (wiping
EVERY series, not just the active tab's) whenever wheel motion resumed
after an idle gap of more than ``_IDLE_STOP`` (1 s). This wipe is
independent of which tab is selected -- it happens purely from the
telemetry stream's own idle/resume pattern -- but because only the
currently-active tab's canvas ever redraws, the operator only notices the
lost history once they switch back to a tab that was showing data when the
wipe happened. That is exactly the reported "switch away, switch back,
data is gone/wrong" symptom, even though the proximate trigger was a
resume-from-idle telemetry frame, not the tab switch itself.

``canvas.py`` (also named in the issue's "Notes / where to look") was
checked and confirmed to hold the playfield/avatar canvas
(``CanvasController``), an unrelated code path -- no second graph-tab
implementation exists.

Fix: ``add_tlm()`` no longer clears the recorder on resume-from-idle; it
just skips appending while idle (the motion gate — reinstated 2026-07-22
per a later stakeholder directive, see ``turn_graphs.py``'s own header) and
resumes appending, to the SAME series, the instant real motion is observed
again. Only the explicit "Clear traces" button (``TurnGraphPanel.clear()``)
discards data now.

Gap-break note (2026-07-22, added the same session as the motion-gate
reinstatement): resuming after a skipped idle span now also weaves in a
NaN gap-break sentinel ahead of the first post-idle point (``_append_series()``,
defect 2b) — display honesty, not a data-loss regression. The counts below
account for that one extra NaN entry per resume; see the assertions'
own comments.

Run with::

    QT_QPA_PLATFORM=offscreen uv run python -m pytest src/tests/testgui/test_turn_graphs_persistence.py -v
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


def _tlm(active, vel, cmd_vel=None, enc=None):
    from robot_radio.robot.protocol import TLMFrame

    return TLMFrame(active=active, vel=vel, cmd_vel=cmd_vel, enc=enc)


def test_tab_switch_during_idle_resume_preserves_wheel_speed_history(qapp):
    """The exact repro sequence from the issue: accumulate on wheel-speed,
    switch away, let the robot go idle then move again (all while a
    DIFFERENT tab is showing), switch back -- the wheel-speed series must
    contain every point recorded while both active and inactive, in order,
    with none lost or corrupted."""
    import math

    from robot_radio.testgui.turn_graphs import TurnGraphPanel

    panel = TurnGraphPanel()
    try:
        tabs = panel._tabs
        wheel_speed_index = next(
            i for i in range(tabs.count()) if tabs.tabText(i) == "Wheel speed"
        )
        heading_index = next(
            i for i in range(tabs.count()) if tabs.tabText(i) == "Heading"
        )

        # 1. View the wheel-speed graph and let data accumulate.
        tabs.setCurrentIndex(wheel_speed_index)
        t = 0.0
        for i in range(10):
            t += 0.05
            vel = (100.0 + i, 100.0 + i)
            panel.add_tlm(t, _tlm(True, vel, cmd_vel=(100.0, 100.0), enc=(i * 5.0, i * 5.0)))

        recorded_before_switch = list(panel.recorder.series["vel_l"])
        assert len(recorded_before_switch) == 10

        # 2. Switch to another graph tab.
        tabs.setCurrentIndex(heading_index)

        # The telemetry stream keeps flowing while the operator is looking
        # elsewhere: an idle gap (robot stops -- frames below the moving
        # threshold) long enough to freeze recording, then fresh motion
        # (a distinguishable second burst) resumes it.
        t += 1.5  # > _IDLE_STOP
        panel.add_tlm(t, _tlm(False, (0.0, 0.0), enc=(50.0, 50.0)))

        for i in range(5):
            t += 0.05
            vel = (200.0 + i, 200.0 + i)
            panel.add_tlm(t, _tlm(True, vel, cmd_vel=(200.0, 200.0), enc=(60.0 + i * 5.0,) * 2))

        # 3. Switch back to wheel speed.
        tabs.setCurrentIndex(wheel_speed_index)

        vel_l = panel.recorder.series["vel_l"]
        # 10 pre-switch + 1 gap-break NaN (the idle frame itself was
        # skipped entirely by the freeze -- see this module's own header
        # note) + 5 post-idle-resume = 16.
        assert len(vel_l) == 16, (
            f"expected 10 pre-switch + 1 gap-break NaN + 5 post-idle-resume "
            f"points to survive, got {len(vel_l)}: {vel_l}"
        )
        # First burst must be intact and unmutated, in original order.
        assert [v for _, v in vel_l[:10]] == [100.0 + i for i in range(10)]
        assert math.isnan(vel_l[10][1]), f"expected a gap-break NaN sentinel at index 10: {vel_l[10]}"
        # Second burst (recorded entirely while the tab was NOT active)
        # must also be present, appended after the first, not replacing it.
        assert [v for _, v in vel_l[11:]] == [200.0 + i for i in range(5)]
        # Timestamps must be strictly increasing throughout -- no reset.
        times = [t_ for t_, _ in vel_l]
        assert times == sorted(times) and len(set(times)) == len(times)
    finally:
        panel.deleteLater()


def test_recorder_add_tlm_resumes_without_clearing_after_idle_freeze():
    """Qt-free unit test of the actual root cause: ``TurnTraceRecorder``
    must not wipe accumulated series when motion resumes after an idle
    gap -- it must only ever be cleared by an explicit ``clear()`` call."""
    import math

    from robot_radio.testgui.turn_graphs import TurnTraceRecorder
    from robot_radio.robot.protocol import TLMFrame

    rec = TurnTraceRecorder()
    t = 0.0
    for i in range(4):
        t += 0.05
        rec.add_tlm(t, TLMFrame(active=True, vel=(50.0, 50.0), enc=(i * 2.0, i * 2.0)))
    assert len(rec.series["vel_l"]) == 4

    # Idle for over the freeze threshold -- this frame is skipped entirely
    # (the freeze), not recorded as a zero.
    t += 1.5
    assert rec.add_tlm(t, TLMFrame(active=False, vel=(0.0, 0.0), enc=(8.0, 8.0))) is False
    assert len(rec.series["vel_l"]) == 4, "an idle frame must not append anything"

    # Motion resumes -- must NOT clear previously-recorded points. The
    # large gap since the last (pre-idle) point weaves in a gap-break NaN
    # sentinel first (defect 2b) -- display honesty, not data loss.
    t += 0.05
    assert rec.add_tlm(t, TLMFrame(active=True, vel=(75.0, 75.0), enc=(10.0, 10.0))) is True

    assert len(rec.series["vel_l"]) == 6, (
        "resuming motion after an idle freeze must APPEND (a gap-break NaN "
        "plus the new real point), not clear, the recorder's series; "
        f"got {rec.series['vel_l']}"
    )
    assert math.isnan(rec.series["vel_l"][4][1])
    assert rec.series["vel_l"][5][1] == 75.0

    # Explicit clear() is still the only thing that discards data.
    rec.clear()
    assert len(rec.series["vel_l"]) == 0
