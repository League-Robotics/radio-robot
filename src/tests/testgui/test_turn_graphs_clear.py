"""src/tests/testgui/test_turn_graphs_clear.py -- ``TurnGraphPanel.
set_on_clear_extra()`` (OOP sim-motor-state fix: unify the two "Clear
Traces" buttons).

Before this fix, the ops-panel "Clear Traces" button cleared ONLY the
playfield ``TraceModel`` polylines, and the turn-graphs header's own "Clear
traces" button cleared ONLY its four recorded traces (wheel speed/position,
heading, distance) -- confusingly different scope per button.
``__main__.py`` now wires each button to clear BOTH: the ops-panel button
calls ``graph_panel.clear()`` in addition to its own playfield clear (tested
indirectly -- ``_clear_traces`` is a closure with no import seam, per this
module's usual pattern for such callables), and ``TurnGraphPanel`` gained an
``on_clear_extra`` hook (settable post-construction via
``set_on_clear_extra()``) so ITS button reaches back out to the playfield
clear too. This module tests the ``TurnGraphPanel`` half directly.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_turn_graphs_clear.py -v
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


def test_clear_calls_on_clear_extra_hook_set_at_construction(qapp):
    from robot_radio.testgui.turn_graphs import TurnGraphPanel

    calls: list[str] = []
    panel = TurnGraphPanel(on_clear_extra=lambda: calls.append("extra"))
    try:
        panel.clear()
        assert calls == ["extra"]
    finally:
        panel.deleteLater()


def test_clear_calls_on_clear_extra_hook_set_after_construction(qapp):
    """``set_on_clear_extra()`` -- the post-construction setter -- exists
    because __main__.py's playfield-clear callable is only defined AFTER
    the panel itself is constructed (a forward-reference problem a
    constructor argument can't solve)."""
    from robot_radio.testgui.turn_graphs import TurnGraphPanel

    panel = TurnGraphPanel()
    try:
        calls: list[str] = []
        panel.set_on_clear_extra(lambda: calls.append("extra"))

        panel.clear()
        assert calls == ["extra"]
    finally:
        panel.deleteLater()


def test_clear_with_no_hook_does_not_raise(qapp):
    from robot_radio.testgui.turn_graphs import TurnGraphPanel

    panel = TurnGraphPanel()
    try:
        panel.clear()  # no on_clear_extra set -- must be a no-op, not raise
    finally:
        panel.deleteLater()


def test_set_on_clear_extra_none_clears_the_hook(qapp):
    from robot_radio.testgui.turn_graphs import TurnGraphPanel

    calls: list[str] = []
    panel = TurnGraphPanel(on_clear_extra=lambda: calls.append("extra"))
    try:
        panel.set_on_clear_extra(None)
        panel.clear()
        assert calls == [], "hook should no longer fire after being cleared"
    finally:
        panel.deleteLater()


def test_clear_still_clears_own_recorder(qapp):
    """The pre-existing behavior -- clearing this panel's own four recorded
    traces -- must be unaffected by the added hook."""
    from robot_radio.robot.protocol import TLMFrame
    from robot_radio.testgui.turn_graphs import TurnGraphPanel

    panel = TurnGraphPanel()
    try:
        panel.add_tlm(0.0, TLMFrame(active=True, enc=(0, 0)))
        panel.add_tlm(0.05, TLMFrame(active=True, enc=(10, 10)))
        assert len(panel.recorder.series["enc_l"]) > 0

        panel.clear()
        assert len(panel.recorder.series["enc_l"]) == 0
    finally:
        panel.deleteLater()
