"""src/tests/testgui/test_telemetry_bridge_threading.py -- emergency fix
(stakeholder report: Tour 1 died/froze right after leg 1, the straight->turn
boundary). Investigates whether ``_TourRunner._on_row``'s direct, SYNCHRONOUS
call into ``transport.on_telemetry()`` (``__main__.py`` ~line 1617-1630) --
which bypasses ``_HardwareTransport._deliver_tlm()``'s own try/except wrapper
and calls straight into ``_on_telemetry_thread_v2`` from the TOUR worker
thread, not the transport's own reader thread -- can touch Qt widgets from
the wrong thread. This is the exact class of bug the project memory
(``pyside-queuedconnection-bare-function.md``) warns about: "a queued signal
connected to a BARE FUNCTION executes on the EMITTING thread."

Per this directory's established "no test seam, re-implement the exact
production control flow inline" convention (see ``test_tour_stop.py``'s own
module docstring -- ``__main__.py``'s internals are closures with no test
seam), this file reproduces ``__main__.py``'s real ``_TelemetryBridge``
pattern (``frame_ready = Signal()``, ``on_frame_ready`` a ``@Slot()``-
decorated BOUND METHOD, connected with an explicit
``Qt.ConnectionType.QueuedConnection`` -- see ``__main__.py`` lines
~1359-1440) and ``_on_telemetry_thread_v2`` (a plain function that queues the
frame and calls ``bridge.frame_ready.emit()`` -- lines ~1750-1763) in
isolation, then drives it from a REAL background ``threading.Thread`` the
same way ``_TourRunner._on_row`` does, and empirically confirms:

1. The widget-touching slot body (``on_frame_ready``) does NOT run
   synchronously when the background thread calls the callback / emits the
   signal -- delivery is deferred until the MAIN thread's event loop is
   pumped.
2. When the slot body DOES run, it runs on the MAIN thread, never on the
   calling background thread.

Verdict (this investigation): this pattern is SAFE as implemented in
``__main__.py`` today. ``frame_ready`` is connected to ``bridge.
on_frame_ready`` -- a bound method of a ``QObject`` instance constructed on
(and never moved off) the main thread -- with an EXPLICIT
``QueuedConnection``, not the implicit/"Auto" default. Empirically (see this
file's own tests, and the ad hoc scripts run during this investigation) that
combination correctly defers to the main thread's event loop regardless of
which thread calls ``.emit()``. The memory-note gotcha this file investigates
did not reproduce for THIS connection shape in the PySide6 version installed
in this repo -- it likely applies to a lambda/partial slot with no
``QObject`` receiver at all (no thread affinity for Qt to queue against), a
shape that does not appear anywhere in the ``on_telemetry``/``_TourRunner``
call chain (audited: every cross-thread ``.connect(..., QueuedConnection)``
in ``__main__.py`` targets a bound method on a ``QObject`` -- see
``clasi/issues/`` for this dispatch's own filed note with the full audit).

Conclusion for the stakeholder's "died/froze" report: NOT a Qt-threading
defect in this path. The reported freeze coincides exactly with 107-005's
own bench finding -- a real ``kFaultWedgeLatch`` firmware fault reproducibly
trips at the straight->turn boundary when ``DEFAULT_INTER_LEG_SETTLE`` is
0.3s (fixed to 1.0s by this same dispatch, ``planner/tour.py``) -- a real
robot stopping mid-tour reads as "died" from the operator's chair, and the
separate TelemetrySecondary log-flood bug (also fixed this dispatch,
``testgui/binary_bridge.py``) would have made the console feel sluggish/
unresponsive at the same time, compounding the impression.

Run with:
    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_telemetry_bridge_threading.py -v
"""
from __future__ import annotations

import threading

import pytest

pytest.importorskip("PySide6")

from PySide6.QtCore import QObject, Qt, Signal, Slot  # type: ignore[import-untyped]
from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


class _TelemetryBridge(QObject):
    """Mirrors __main__.py's real ``_TelemetryBridge``: a ``frame_ready``
    Signal() connected to a bound ``on_frame_ready`` Slot() with an explicit
    QueuedConnection (see this file's own module docstring)."""

    frame_ready = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.call_threads: list[int] = []

    @Slot()
    def on_frame_ready(self) -> None:
        # Stand-in for the real slot's widget-touching body
        # (canvas_ctrl.refresh()/graph_panel.add_tlm()/telemetry_ctrl.
        # update_frame() -- __main__.py lines ~1394-1422). Only the THREAD
        # this runs on matters for this test, not the widget calls
        # themselves.
        self.call_threads.append(threading.get_ident())


def _make_on_telemetry(bridge: _TelemetryBridge):
    """Mirrors __main__.py's real ``_on_telemetry_thread_v2``: a plain
    function (assigned to ``transport.on_telemetry``, NOT a QObject method)
    that queues the frame and emits the bridge signal -- no direct widget
    access. This is exactly what ``_TourRunner._on_row`` calls synchronously
    from the tour worker thread."""

    def on_telemetry(frame: object) -> None:
        bridge.frame_ready.emit()

    return on_telemetry


def test_background_thread_call_defers_widget_work_to_main_thread(qapp):
    """Reproduces _TourRunner._on_row's exact call shape: a background
    thread calls the on_telemetry callback SYNCHRONOUSLY, with no thread hop
    of its own. Confirms the slot body does not run until the main thread's
    event loop is pumped -- i.e. the safety comes entirely from
    frame_ready's own QueuedConnection, not from any thread hop inside
    on_telemetry itself."""
    main_thread_id = threading.get_ident()
    bridge = _TelemetryBridge()
    bridge.frame_ready.connect(bridge.on_frame_ready, Qt.ConnectionType.QueuedConnection)
    on_telemetry = _make_on_telemetry(bridge)

    worker_thread_id: list[int] = []

    def _background_call() -> None:
        worker_thread_id.append(threading.get_ident())
        on_telemetry(object())  # exactly what _TourRunner._on_row does

    t = threading.Thread(target=_background_call)
    t.start()
    t.join(timeout=2.0)
    assert not t.is_alive(), "background thread must not block on emit()"

    # The queued slot must NOT have run yet -- delivery is deferred until
    # the main thread's event loop processes the queued event.
    assert bridge.call_threads == [], (
        "on_frame_ready ran before the main thread pumped its event loop -- "
        "frame_ready delivery is not actually deferred"
    )

    qapp.processEvents()

    assert bridge.call_threads == [main_thread_id], (
        "on_frame_ready must execute on the MAIN thread, exactly once, "
        "never on the worker thread that called on_telemetry()"
    )
    assert worker_thread_id[0] != main_thread_id, "sanity: the call really came from a different thread"


def test_multiple_background_frames_all_land_on_main_thread(qapp):
    """A tour drives many ticks in quick succession -- confirms the pattern
    holds under repeated cross-thread delivery, not just a single call."""
    main_thread_id = threading.get_ident()
    bridge = _TelemetryBridge()
    bridge.frame_ready.connect(bridge.on_frame_ready, Qt.ConnectionType.QueuedConnection)
    on_telemetry = _make_on_telemetry(bridge)

    def _background_call() -> None:
        for _ in range(20):
            on_telemetry(object())

    t = threading.Thread(target=_background_call)
    t.start()
    t.join(timeout=2.0)
    assert not t.is_alive()

    assert bridge.call_threads == []

    qapp.processEvents()

    assert len(bridge.call_threads) == 20
    assert all(tid == main_thread_id for tid in bridge.call_threads)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
