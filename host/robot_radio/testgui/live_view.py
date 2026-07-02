"""robot_radio.testgui.live_view — Continuous aprilcam frame capture for PLAYFIELD MODE.

Threading model
---------------
:func:`build_live_view_worker` returns a ``_LiveViewWorker`` QObject instance.
The caller must move it to a ``QThread`` before starting:

    worker = build_live_view_worker()
    thread = QThread()
    worker.moveToThread(thread)
    worker.frame_ready.connect(slot, Qt.ConnectionType.QueuedConnection)
    thread.started.connect(worker.run)
    thread.start()

Call ``worker.stop()`` to request shutdown; ``run()`` exits within 2 s.
Call ``thread.quit()`` + ``thread.wait(3000)`` after ``stop()`` to join cleanly.

Signal signature
----------------
``frame_ready(bgr_ndarray: object, origin_x: float, origin_y: float,
              tag_x_cm: float, tag_y_cm: float, tag_yaw_rad: float)``

The BGR ndarray must be converted to a QPixmap on the GUI thread (in the slot).
Do NOT call QPixmap() inside ``run()``.

Daemon access pattern
---------------------
Each ``_capture_and_emit()`` call opens a fresh ``DaemonControl`` connection,
performs the capture, and closes it.  This keeps the session stateless and
avoids stuck connections on daemon restart.

Camera resolution (ticket 063-008)
------------------------------------
The camera to capture from is resolved via
``camera_prefs.select_camera(dc.list_cameras(), camera_prefs.load_camera_pref())``
— the same shared helper used by ``operations.py``'s playfield-refresh and
pose-read paths, so all three camera-consuming code paths agree on which
aprilcam camera to use instead of each picking independently (e.g. an
unconditional ``cams[0]``).

On daemon unavailability the worker logs a warning once, backs off 2 s, and
retries.  It does not raise or stop automatically unless aprilcam is not
installed at all.

Tag-100 pose (last-known-pose semantics)
-----------------------------------------
If tag 100 is not visible in a frame, the worker emits the *last known* pose
stored in ``_last_tag`` — the avatar stays where it was last seen.  It does NOT
snap to (0, 0) and is NOT hidden.  ``_last_tag`` is initialised to
``(0.0, 0.0, 0.0)`` and updated only when tag 100 is successfully read.

PySide6 import policy
---------------------
All PySide6 imports are deferred inside :func:`build_live_view_worker` so this
module is importable without PySide6 present (unit tests, static analysis).
"""

from __future__ import annotations

import logging
import time

_log = logging.getLogger(__name__)

# Target loop interval in seconds (~12 Hz).
_TARGET_INTERVAL_S = 0.08

# Back-off delay when the daemon is unavailable.
_DAEMON_BACKOFF_S = 2.0


def build_live_view_worker() -> object:
    """Return a ``_LiveViewWorker`` instance (QObject) on the calling thread.

    The caller must move it to a ``QThread`` before starting.  PySide6 is
    imported here so the module itself is importable without PySide6.

    Returns
    -------
    _LiveViewWorker
        A freshly constructed ``QObject`` worker.  Move it to a ``QThread``
        and connect ``thread.started`` to ``worker.run`` before calling
        ``thread.start()``.
    """
    from PySide6.QtCore import QObject, Signal, Slot  # type: ignore[import-untyped]

    class _LiveViewWorker(QObject):
        """QObject live-view worker for PLAYFIELD MODE.

        Move to a QThread before starting.  Connect ``frame_ready`` with
        ``Qt.ConnectionType.QueuedConnection`` so the slot runs on the GUI
        thread where QPixmap construction is safe.

        Signals
        -------
        frame_ready(bgr_ndarray, origin_x, origin_y, tag_x_cm, tag_y_cm, tag_yaw_rad)
            Emitted once per captured frame.  ``bgr_ndarray`` is a raw numpy
            BGR array — convert to QPixmap on the GUI thread, not here.
        """

        frame_ready = Signal(object, float, float, float, float, float)

        def __init__(self) -> None:
            super().__init__()
            self._stop = False
            # Last known tag-100 pose.  Holds position between frames when the
            # tag is temporarily invisible.  Initialised to world origin.
            self._last_tag: tuple[float, float, float] = (0.0, 0.0, 0.0)

        @Slot()
        def run(self) -> None:
            """Main loop (~12 Hz).  Runs on the worker QThread.

            Calls ``_capture_and_emit()`` each iteration.  On daemon
            unavailability backs off ``_DAEMON_BACKOFF_S`` seconds before
            retrying.  Exits when ``_stop`` is True.
            """
            while not self._stop:
                t0 = time.monotonic()
                try:
                    self._capture_and_emit()
                except _DaemonUnavailable as exc:
                    _log.warning("LiveViewWorker: daemon unavailable: %s", exc)
                    # Back off before retry so we don't spin on a missing daemon.
                    deadline = time.monotonic() + _DAEMON_BACKOFF_S
                    while not self._stop and time.monotonic() < deadline:
                        time.sleep(0.05)
                    continue
                except Exception as exc:
                    _log.debug("LiveViewWorker loop error: %s", exc, exc_info=True)

                elapsed = time.monotonic() - t0
                sleep_s = max(0.0, _TARGET_INTERVAL_S - elapsed)
                if sleep_s > 0.0 and not self._stop:
                    time.sleep(sleep_s)

        @Slot()
        def stop(self) -> None:
            """Request the worker to exit.  ``run()`` will finish within 2 s."""
            self._stop = True

        def _capture_and_emit(self) -> None:
            """Capture one frame from aprilcam and emit ``frame_ready``.

            Raises ``_DaemonUnavailable`` when the daemon cannot be reached so
            the caller can back off cleanly.
            """
            try:
                from aprilcam.config import Config  # type: ignore[import]
                from aprilcam.client.control import DaemonControl  # type: ignore[import]
            except ImportError as exc:
                _log.warning(
                    "LiveViewWorker: aprilcam not installed (%s); stopping.", exc
                )
                self._stop = True
                return

            from robot_radio.testgui.operations import _deskew_bgr_ndarray
            from robot_radio.testgui import camera_prefs

            try:
                dc = DaemonControl.connect_default(Config.load())
            except Exception as exc:
                raise _DaemonUnavailable(str(exc)) from exc

            try:
                cams = dc.list_cameras()
                if not cams:
                    raise _DaemonUnavailable("aprilcam daemon reports no cameras")

                cam = camera_prefs.select_camera(cams, camera_prefs.load_camera_pref())
                if cam is None:
                    raise _DaemonUnavailable("aprilcam daemon reports no cameras")
                raw_bgr = dc.capture_frame(cam)
                tag_frame = dc.get_tags(cam)
            except _DaemonUnavailable:
                raise
            except Exception as exc:
                raise _DaemonUnavailable(str(exc)) from exc
            finally:
                try:
                    dc.close()
                except Exception:
                    pass

            if raw_bgr is None:
                return

            result = _deskew_bgr_ndarray(raw_bgr, tag_frame)
            if result is None:
                # Deskew failed (e.g. uncalibrated camera or cv2 missing).
                return
            bgr, origin_x, origin_y = result

            # Extract tag-100 pose.  Use by_id() which is the correct TagFrame API
            # (tags is a list[TagRecord], not a dict).  Hold last known pose when
            # tag 100 is not visible so the avatar does not snap to (0, 0).
            tx, ty, tyaw = self._last_tag
            rec = tag_frame.by_id(100) if tag_frame is not None else None
            if rec is not None:
                wxy = getattr(rec, "world_xy", None)
                if wxy is not None:
                    tx, ty = float(wxy[0]), float(wxy[1])
                    # heading_rad is the robot's world heading; fall back to raw yaw.
                    h_rad = getattr(rec, "heading_rad", None)
                    tyaw = float(h_rad) if h_rad is not None else float(getattr(rec, "yaw", tyaw))
                    self._last_tag = (tx, ty, tyaw)
                # If world_xy is None the tag is uncalibrated; hold last known pose.

            self.frame_ready.emit(bgr, float(origin_x), float(origin_y),
                                  float(tx), float(ty), float(tyaw))

    return _LiveViewWorker()


class _DaemonUnavailable(Exception):
    """Raised internally when the aprilcam daemon cannot be reached."""
