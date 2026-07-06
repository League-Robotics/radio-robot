"""tests/testgui/test_tour_stop.py — Tour/GOTO stop reactivation and Sim-mode
tour gating (ported from ``tests_old/testgui/`` per sprint 085 ticket 002).

Root cause (testgui-tour-stop-reactivation.md): ``_stop_tour()`` /
``_stop_goto()`` call ``worker.stop()``, ``thread.quit()``, ``thread.wait(3000)``
— which blocks the main thread — then drop the only Python reference to the
``_WorkerBridge``. The worker's ``finished`` signal fires *during* that
blocking wait, so the queued slot cannot run until ``wait()`` returns; by
then the bridge is eligible for GC and the pending delivery is lost, so
``_on_tour_finished`` / ``_on_goto_finished`` (the only place that
re-enabled the buttons) never runs. The fix re-enables the buttons
synchronously inside ``_stop_tour`` / ``_stop_goto`` themselves, right after
the join, instead of depending on the signal.

Direct read of ``host/robot_radio/testgui/__main__.py`` (``_stop_tour``
~line 1796, ``_on_tour_finished`` ~line 1831, ``_stop_goto``'s counterpart)
confirms this control flow is UNCHANGED since the historical fix — the
inline re-implementations below match line-for-line.

``host/robot_radio/testgui/__main__.py``'s internals are closures with no
test seam (``_build_main_window()`` returns only ``(window, app)``). Per the
established pattern in ``tests/testgui/test_set_origin.py``, these tests
re-implement the exact production control flow inline using fake
worker/thread doubles, so the logic is verified deterministically without
real ``QThread`` timing.

Real-firmware coverage: ``test_tour1_geometry.py`` (this same directory)
additionally clicks the real ``Stop Tour`` button mid-run against a live
``SimTransport``-backed tour and asserts the same synchronous re-enable —
this file pins down the exact control flow deterministically; that one
proves it holds against the real GUI/QThread/SimTransport stack.

Qt-free: these tests import only pure helpers / fake QPushButton stand-ins
and do not require a QApplication (except where noted).

Run with:
    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_tour_stop.py -v
"""
from __future__ import annotations


# ---------------------------------------------------------------------------
# Fake doubles (Qt-free)
# ---------------------------------------------------------------------------


class _FakeButton:
    """Stand-in for a QPushButton — records enable/disable calls."""

    def __init__(self, enabled: bool = False):
        self._enabled = enabled

    def setEnabled(self, value: bool) -> None:
        self._enabled = value

    def isEnabled(self) -> bool:
        return self._enabled


class _FakeWorker:
    """Stand-in for _TourRunner/_GotoRunner — records stop() calls."""

    def __init__(self):
        self.stop_called = False

    def stop(self) -> None:
        self.stop_called = True


class _FakeThread:
    """Stand-in for QThread — quit()/wait() are no-ops returning immediately.

    This mirrors the real bug: in production, wait() blocks the main thread
    while the worker's queued `finished` signal cannot yet be delivered. The
    fake makes quit()/wait() instantaneous no-ops so the test exercises only
    the synchronous re-enable logic in _stop_tour/_stop_goto themselves —
    the assertions must NOT depend on any signal ever being processed.
    """

    def __init__(self):
        self.quit_called = False
        self.wait_called_with: int | None = None

    def quit(self) -> None:
        self.quit_called = True

    def wait(self, timeout_ms: int) -> bool:
        self.wait_called_with = timeout_ms
        return True


class _FakeTransport:
    """Minimal connected-transport stand-in (identity only matters)."""


# ---------------------------------------------------------------------------
# _stop_tour synchronous reactivation
# ---------------------------------------------------------------------------


def _make_stop_tour(state: dict, tour_buttons: list, stop_tour_btn: _FakeButton):
    """Re-implement _stop_tour()'s logic inline (see module docstring)."""

    def _stop_tour() -> None:
        worker = state.get("tour_worker")
        thread = state.get("tour_thread")
        if worker is not None:
            try:
                worker.stop()
            except Exception:
                pass
        if thread is not None:
            try:
                thread.quit()
                thread.wait(3000)
            except Exception:
                pass
        state["tour_worker"] = None
        state["tour_thread"] = None
        state["tour_bridge"] = None
        if state.get("transport") is not None:
            for _tb in tour_buttons:
                _tb.setEnabled(True)
        stop_tour_btn.setEnabled(False)

    return _stop_tour


def test_stop_tour_reenables_buttons_synchronously():
    """_stop_tour() re-enables tour buttons and disables Stop Tour immediately."""
    tour_btn = _FakeButton(enabled=False)
    stop_tour_btn = _FakeButton(enabled=True)
    worker = _FakeWorker()
    thread = _FakeThread()
    state = {
        "transport": _FakeTransport(),
        "tour_worker": worker,
        "tour_thread": thread,
        "tour_bridge": object(),
    }

    stop_tour = _make_stop_tour(state, [tour_btn], stop_tour_btn)
    stop_tour()

    assert worker.stop_called, "worker.stop() must be called"
    assert thread.quit_called, "thread.quit() must be called"
    assert thread.wait_called_with == 3000, "thread.wait(3000) must be called"
    assert tour_btn.isEnabled(), (
        "Tour button must be re-enabled immediately after _stop_tour() returns "
        "— must not depend on the finished signal being processed afterward"
    )
    assert not stop_tour_btn.isEnabled(), "Stop Tour button must be disabled"
    assert state["tour_worker"] is None
    assert state["tour_thread"] is None
    assert state["tour_bridge"] is None


def test_stop_tour_no_transport_does_not_reenable():
    """_stop_tour() without a connected transport does not re-enable buttons."""
    tour_btn = _FakeButton(enabled=False)
    stop_tour_btn = _FakeButton(enabled=True)
    worker = _FakeWorker()
    thread = _FakeThread()
    state = {
        "transport": None,
        "tour_worker": worker,
        "tour_thread": thread,
        "tour_bridge": object(),
    }

    stop_tour = _make_stop_tour(state, [tour_btn], stop_tour_btn)
    stop_tour()

    assert not tour_btn.isEnabled(), (
        "Without a connected transport, tour buttons should stay disabled"
    )
    assert not stop_tour_btn.isEnabled()


def test_stop_tour_is_safe_noop_when_idle():
    """Calling _stop_tour() with no tour running is a safe no-op."""
    tour_btn = _FakeButton(enabled=True)
    stop_tour_btn = _FakeButton(enabled=False)
    state = {
        "transport": _FakeTransport(),
        "tour_worker": None,
        "tour_thread": None,
        "tour_bridge": None,
    }

    stop_tour = _make_stop_tour(state, [tour_btn], stop_tour_btn)
    stop_tour()  # should not raise

    assert tour_btn.isEnabled()
    assert not stop_tour_btn.isEnabled()
    assert state["tour_worker"] is None
    assert state["tour_thread"] is None
    assert state["tour_bridge"] is None


# ---------------------------------------------------------------------------
# _on_tour_finished natural-completion path (unaffected by the fix)
# ---------------------------------------------------------------------------


def _make_on_tour_finished(state: dict, tour_buttons: list, stop_tour_btn: _FakeButton):
    def _on_tour_finished() -> None:
        thread = state.get("tour_thread")
        if thread is not None:
            try:
                thread.quit()
                thread.wait(3000)
            except Exception:
                pass
        state["tour_worker"] = None
        state["tour_thread"] = None
        state["tour_bridge"] = None
        if state.get("transport") is not None:
            for _tb in tour_buttons:
                _tb.setEnabled(True)
        stop_tour_btn.setEnabled(False)

    return _on_tour_finished


def test_on_tour_finished_reenables_buttons():
    """Natural-completion path still re-enables tour buttons and disables Stop Tour."""
    tour_btn = _FakeButton(enabled=False)
    stop_tour_btn = _FakeButton(enabled=True)
    thread = _FakeThread()
    state = {
        "transport": _FakeTransport(),
        "tour_worker": _FakeWorker(),
        "tour_thread": thread,
        "tour_bridge": object(),
    }

    on_tour_finished = _make_on_tour_finished(state, [tour_btn], stop_tour_btn)
    on_tour_finished()

    assert tour_btn.isEnabled()
    assert not stop_tour_btn.isEnabled()
    assert state["tour_worker"] is None
    assert state["tour_thread"] is None
    assert state["tour_bridge"] is None


# ---------------------------------------------------------------------------
# _stop_goto synchronous reactivation
# ---------------------------------------------------------------------------


def _make_stop_goto(state: dict, goto_btn: _FakeButton):
    """Re-implement _stop_goto()'s logic inline (see module docstring)."""

    def _stop_goto() -> None:
        worker = state.get("goto_worker")
        thread = state.get("goto_thread")
        if worker is not None:
            try:
                worker.stop()
            except Exception:
                pass
        if thread is not None:
            try:
                thread.quit()
                thread.wait(3000)
            except Exception:
                pass
        state["goto_worker"] = None
        state["goto_thread"] = None
        state["goto_bridge"] = None
        if state.get("transport") is not None:
            goto_btn.setEnabled(True)

    return _stop_goto


def test_stop_goto_reenables_button_synchronously():
    """_stop_goto() re-enables goto_btn immediately after the join."""
    goto_btn = _FakeButton(enabled=False)
    worker = _FakeWorker()
    thread = _FakeThread()
    state = {
        "transport": _FakeTransport(),
        "goto_worker": worker,
        "goto_thread": thread,
        "goto_bridge": object(),
    }

    stop_goto = _make_stop_goto(state, goto_btn)
    stop_goto()

    assert worker.stop_called
    assert thread.quit_called
    assert thread.wait_called_with == 3000
    assert goto_btn.isEnabled(), (
        "GOTO button must be re-enabled immediately after _stop_goto() returns"
    )
    assert state["goto_worker"] is None
    assert state["goto_thread"] is None
    assert state["goto_bridge"] is None


def test_stop_goto_no_transport_does_not_reenable():
    """_stop_goto() without a connected transport does not re-enable the button."""
    goto_btn = _FakeButton(enabled=False)
    state = {
        "transport": None,
        "goto_worker": _FakeWorker(),
        "goto_thread": _FakeThread(),
        "goto_bridge": object(),
    }

    stop_goto = _make_stop_goto(state, goto_btn)
    stop_goto()

    assert not goto_btn.isEnabled()


def test_stop_goto_is_safe_noop_when_idle():
    """Calling _stop_goto() with no GOTO running is a safe no-op."""
    goto_btn = _FakeButton(enabled=True)
    state = {
        "transport": _FakeTransport(),
        "goto_worker": None,
        "goto_thread": None,
        "goto_bridge": None,
    }

    stop_goto = _make_stop_goto(state, goto_btn)
    stop_goto()  # should not raise

    assert goto_btn.isEnabled()
    assert state["goto_worker"] is None
    assert state["goto_thread"] is None
    assert state["goto_bridge"] is None


# ---------------------------------------------------------------------------
# Sim-mode tour gating (testgui-tour-sim-mode-gating.md)
# ---------------------------------------------------------------------------


def _make_tour_click_log(log_lines: list[str]):
    """Re-implement the Sim-mode-check-plus-log slice of _on_tour_clicked()."""
    from robot_radio.testgui.operations import is_sim_transport

    def _on_tour_clicked(transport, name: str) -> None:
        if is_sim_transport(transport):
            log_lines.append("[TOUR] running in SIM mode")
        log_lines.append(f"[TOUR] {name} starting — resetting to origin")

    return _on_tour_clicked


def test_tour_click_logs_sim_mode_line_for_sim_transport():
    """Starting a tour against a SimTransport logs the SIM-mode line first."""
    from robot_radio.testgui.transport import SimTransport

    log_lines: list[str] = []
    on_tour_clicked = _make_tour_click_log(log_lines)

    sim_transport = SimTransport()
    on_tour_clicked(sim_transport, "Tour 1")

    assert log_lines[0] == "[TOUR] running in SIM mode"
    assert log_lines[1] == "[TOUR] Tour 1 starting — resetting to origin"


def test_tour_click_does_not_log_sim_mode_line_for_non_sim_transport():
    """Starting a tour against a non-Sim transport does NOT log the SIM-mode line."""

    class _NonSimTransport:
        """A transport that is not a SimTransport."""

    log_lines: list[str] = []
    on_tour_clicked = _make_tour_click_log(log_lines)

    on_tour_clicked(_NonSimTransport(), "Tour 1")

    assert not any("SIM mode" in line for line in log_lines), (
        f"Non-Sim transport must not log the SIM-mode line, got: {log_lines}"
    )
    assert log_lines == ["[TOUR] Tour 1 starting — resetting to origin"]
