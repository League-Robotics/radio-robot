"""robot_radio.testgui.worker_session — WorkerSession: one background GUI task.

The TestGUI runs long jobs (a tour, a GOTO pursuit) on a ``QThread`` with a
main-thread ``_WorkerBridge`` marshalling their signals back. Tour and GOTO had
byte-identical copies of that lifecycle in ``__main__.py``, differing only in
which buttons re-enable at the end. This is that lifecycle, once, Qt-free enough
to unit-test headlessly with fakes.

WHY IT IS ITS OWN CLASS
-----------------------
The duplication was not cosmetic -- it is where a real defect lived
(``testgui-tour-stop-reactivation.md``). The subtle part is:

    An explicit stop MUST settle the UI synchronously, right after the join.
    It must NOT rely on the worker's queued ``finished`` signal.

That signal fires *during* the blocking ``thread.wait()``, but a queued slot
cannot run until ``wait()`` returns -- and by then the caller has dropped its
only reference to the bridge, so the delivery is lost and the buttons never
re-enable. Every copy of this lifecycle has to get that right; one copy is one
place to get it right, and one place to test it.

``on_settled`` is therefore invoked by BOTH paths (explicit stop and natural
completion), exactly once per run, on the calling thread.
"""
from __future__ import annotations

import logging
from typing import Any, Callable

_log = logging.getLogger(__name__)

#: How long to wait for a worker thread to finish after quit() [ms].
JOIN_TIMEOUT_MS = 3000


class WorkerSession:
    """Owns the worker/thread/bridge triple for one background GUI task.

    Parameters
    ----------
    name:
        Human label used in log lines ("tour", "goto").
    on_settled:
        Called after the thread is joined and the slots are cleared, on BOTH
        the explicit-stop and natural-completion paths. This is where UI
        re-enabling belongs -- see the module docstring for why it must not be
        driven by the worker's queued ``finished`` signal.
    log:
        Optional line logger for diagnostics.
    """

    def __init__(self, name: str,
                 on_settled: "Callable[[], None] | None" = None,
                 log: "Callable[[str], None] | None" = None) -> None:
        self.name = name
        self._on_settled = on_settled
        self._log = log
        self._worker: Any = None
        self._thread: Any = None
        self._bridge: Any = None

    @property
    def running(self) -> bool:
        """True while a worker is attached (thread started, not yet settled)."""
        return self._thread is not None

    def start(self, worker: Any, thread: Any, bridge: Any) -> None:
        """Adopt an already-wired worker/thread/bridge and start the thread.

        The caller wires the Qt signal connections (they are Qt-specific and
        must be made before the thread starts); this class owns only the
        lifetime. Holding `bridge` is load-bearing, not bookkeeping: it is the
        only strong reference keeping the main-thread marshaller alive for the
        duration of the run.
        """
        if self.running:
            raise RuntimeError(f"{self.name}: a worker is already running")
        self._worker, self._thread, self._bridge = worker, thread, bridge
        thread.start()

    def stop(self) -> None:
        """Cancel the worker, join its thread, clear state, then settle.

        Safe when idle. Never raises -- a stop path that can throw is
        indistinguishable from a stop that worked, and this one runs from the
        STOP button and from disconnect.
        """
        worker, thread = self._worker, self._thread
        if worker is not None:
            try:
                worker.stop()
            except Exception:
                _log.exception("%s: worker.stop() raised", self.name)
        if thread is not None:
            try:
                thread.quit()
                thread.wait(JOIN_TIMEOUT_MS)
            except Exception:
                _log.exception("%s: thread join raised", self.name)
        self._clear_and_settle(had_work=thread is not None)

    def finished(self) -> None:
        """Natural-completion path: the worker ended on its own.

        Joins the (already-finishing) thread and settles. Idempotent with
        ``stop()`` -- whichever runs first settles, the second is a no-op.
        """
        thread = self._thread
        if thread is not None:
            try:
                thread.quit()
                thread.wait(JOIN_TIMEOUT_MS)
            except Exception:
                _log.exception("%s: thread join raised", self.name)
        self._clear_and_settle(had_work=thread is not None)

    def _clear_and_settle(self, *, had_work: bool) -> None:
        self._worker = self._thread = self._bridge = None
        if not had_work:
            return          # nothing was running; settling again would be a lie
        if self._on_settled is not None:
            try:
                self._on_settled()
            except Exception:
                _log.exception("%s: on_settled raised", self.name)
        if self._log is not None:
            self._log(f"[INFO] {self.name} settled")
