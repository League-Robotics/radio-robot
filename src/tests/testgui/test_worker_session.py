"""WorkerSession lifecycle -- headless, no QApplication.

The behaviour under test is the one that broke in production
(testgui-tour-stop-reactivation.md): an explicit stop must settle the UI
SYNCHRONOUSLY after joining, never via the worker's queued `finished` signal,
because that signal is lost when the bridge reference is dropped during wait().
"""
from __future__ import annotations

import pytest

from robot_radio.testgui.worker_session import WorkerSession


class FakeWorker:
    def __init__(self, raises: bool = False) -> None:
        self.stopped = 0
        self._raises = raises

    def stop(self) -> None:
        self.stopped += 1
        if self._raises:
            raise RuntimeError("worker refused to stop")


class FakeThread:
    def __init__(self, raises: bool = False) -> None:
        self.started = self.quit_calls = self.waited = 0
        self._raises = raises

    def start(self) -> None:
        self.started += 1

    def quit(self) -> None:
        self.quit_calls += 1
        if self._raises:
            raise RuntimeError("thread refused to quit")

    def wait(self, ms: int) -> None:
        self.waited += 1


def _session():
    settled: list[int] = []
    s = WorkerSession("tour", on_settled=lambda: settled.append(1))
    return s, settled


def test_idle_session_is_not_running():
    s, _ = _session()
    assert not s.running


def test_start_starts_the_thread_and_marks_running():
    s, _ = _session()
    w, t = FakeWorker(), FakeThread()
    s.start(w, t, bridge=object())
    assert t.started == 1
    assert s.running


def test_stop_cancels_joins_and_settles_synchronously():
    """The whole point: settling does NOT wait on a queued signal."""
    s, settled = _session()
    w, t = FakeWorker(), FakeThread()
    s.start(w, t, bridge=object())

    s.stop()

    assert w.stopped == 1
    assert (t.quit_calls, t.waited) == (1, 1)
    assert settled == [1], "UI must settle on the explicit-stop path"
    assert not s.running


def test_stop_when_idle_is_a_noop_and_does_not_settle():
    """Settling when nothing ran would report work that never happened."""
    s, settled = _session()
    s.stop()
    assert settled == []
    assert not s.running


def test_finished_settles_once_on_natural_completion():
    s, settled = _session()
    s.start(FakeWorker(), FakeThread(), bridge=object())
    s.finished()
    assert settled == [1]
    assert not s.running


def test_stop_after_finished_does_not_settle_twice():
    """Both paths can fire for one run; the UI must settle exactly once."""
    s, settled = _session()
    s.start(FakeWorker(), FakeThread(), bridge=object())
    s.finished()
    s.stop()
    assert settled == [1]


def test_stop_never_raises_even_when_worker_and_thread_do():
    """A halt path that raises is indistinguishable from one that worked."""
    s, settled = _session()
    s.start(FakeWorker(raises=True), FakeThread(raises=True), bridge=object())

    s.stop()  # must not raise

    assert settled == [1], "a throwing worker must still settle the UI"
    assert not s.running


def test_a_throwing_on_settled_does_not_escape():
    s = WorkerSession("goto", on_settled=lambda: (_ for _ in ()).throw(
        RuntimeError("ui blew up")))
    s.start(FakeWorker(), FakeThread(), bridge=object())
    s.stop()  # must not raise
    assert not s.running


def test_starting_twice_is_refused():
    s, _ = _session()
    s.start(FakeWorker(), FakeThread(), bridge=object())
    with pytest.raises(RuntimeError):
        s.start(FakeWorker(), FakeThread(), bridge=object())


def test_session_holds_the_bridge_for_the_run_then_releases_it():
    """The bridge reference is load-bearing -- dropping it early is the
    original defect. It must live until the run settles, and not after."""
    s, _ = _session()
    bridge = object()
    s.start(FakeWorker(), FakeThread(), bridge)
    assert s._bridge is bridge
    s.stop()
    assert s._bridge is None


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
