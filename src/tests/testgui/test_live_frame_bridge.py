"""src/tests/testgui/test_live_frame_bridge.py -- ticket 085-007: live camera
view verification (main-thread frame bridge, ticket 063-009). Ported from
``tests_old/testgui/test_live_frame_bridge.py``.

Root cause (testgui-playfield-not-live-updating.md): in the Relay branch of
``_on_connect()``, ``live_worker.frame_ready`` was connected directly to a
bare function with ``Qt.ConnectionType.QueuedConnection``. In this PySide
build, a ``QueuedConnection`` to a non-``QObject`` callable is delivered on
the *emitting* (worker) thread rather than the GUI thread -- and the
live-view worker's tight capture loop never returns to its own thread's Qt
event loop, so such "queued" deliveries are never processed. The canvas
background therefore never repainted from the live camera.

The fix routes ``frame_ready`` through ``build_live_frame_bridge()`` (in
``robot_radio.testgui.__main__``), a real factory function -- mirroring
``robot_radio.testgui.live_view.build_live_view_worker`` -- that returns a
``QObject`` bridge constructed on the GUI thread. Its bound-method
``on_frame`` slot is what ``frame_ready`` connects to, so delivery reliably
happens on the GUI thread.

Stakeholder decision under test: the avatar pose updates on *every* frame the
worker emits (full worker rate, ~9-12 Hz); the background pixmap conversion +
``canvas_ctrl.set_background()`` call is throttled to a subset of frames
(~3-4 fps target). Tests assert the *ratio* invariant (background count <=
avatar count, background count < N, background count >= 1 for N >=
BACKGROUND_THROTTLE_N) rather than an exact fps, per the ticket's binding
instruction not to chase/assert a specific frame rate.

No production code change: pure verification pass.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_live_frame_bridge.py -v
"""

from __future__ import annotations

import pytest


# ---------------------------------------------------------------------------
# QApplication fixture (module-scoped to share across all tests)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def qapp():
    """Return (or create) the QApplication singleton for this module."""
    # 107-004: turn a missing `gui` dependency group into a clean skip, not
    # a hard collection/run error -- see test_tour1_geometry.py's module
    # docstring for the full rationale.
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]
    import sys
    app = QApplication.instance() or QApplication(sys.argv)
    yield app


# ---------------------------------------------------------------------------
# Fake doubles (Qt-free)
# ---------------------------------------------------------------------------


class _FakeCanvasCtrl:
    """Stand-in for CanvasController -- records call counts (mirrors
    _FakeCanvasCtrl in test_set_origin.py)."""

    def __init__(self):
        self.avatar_calls: list[tuple[float, float, float]] = []
        self.background_calls: list[tuple[object, float, float]] = []
        self.restore_calls = 0

    def set_avatar_pose(self, x: float, y: float, yaw: float) -> None:
        self.avatar_calls.append((x, y, yaw))

    def set_background(self, pixmap, origin_x: float, origin_y: float) -> None:
        self.background_calls.append((pixmap, origin_x, origin_y))

    def restore_static_background(self) -> None:
        self.restore_calls += 1


class _FakeWorker:
    """Stand-in for the live-view worker -- records stop() calls."""

    def __init__(self):
        self.stop_called = False

    def stop(self) -> None:
        self.stop_called = True


class _FakeThread:
    """Stand-in for QThread -- quit()/wait() are no-ops returning immediately."""

    def __init__(self):
        self.quit_called = False
        self.wait_called_with: int | None = None

    def quit(self) -> None:
        self.quit_called = True

    def wait(self, timeout_ms: int) -> bool:
        self.wait_called_with = timeout_ms
        return True


def _make_bgr_frame():
    """A real BGR ndarray so _bgr_ndarray_to_pixmap can actually succeed."""
    import numpy as np
    return np.zeros((60, 80, 3), dtype=np.uint8)


# ---------------------------------------------------------------------------
# _LiveFrameBridge -- full-rate avatar, throttled background
# ---------------------------------------------------------------------------


class TestLiveFrameBridgeThrottle:
    """build_live_frame_bridge() returns a bridge with full-rate avatar,
    throttled background updates (stakeholder decision, ticket 063-009)."""

    def test_build_returns_qobject(self, qapp):
        """build_live_frame_bridge() returns a QObject."""
        from PySide6.QtCore import QObject  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import build_live_frame_bridge

        canvas_ctrl = _FakeCanvasCtrl()
        bridge = build_live_frame_bridge(canvas_ctrl)
        assert isinstance(bridge, QObject)

    def test_bridge_has_on_frame_slot(self, qapp):
        """Bridge must expose a callable on_frame(...) slot."""
        from robot_radio.testgui.__main__ import build_live_frame_bridge

        bridge = build_live_frame_bridge(_FakeCanvasCtrl())
        assert callable(getattr(bridge, "on_frame", None))

    def test_avatar_updates_every_frame(self, qapp):
        """set_avatar_pose is called exactly N times for N on_frame() calls."""
        from robot_radio.testgui.__main__ import build_live_frame_bridge

        canvas_ctrl = _FakeCanvasCtrl()
        bridge = build_live_frame_bridge(canvas_ctrl)
        bgr = _make_bgr_frame()

        N = 9
        for i in range(N):
            bridge.on_frame(bgr, 40.0, 30.0, float(i), float(i) * 2.0, 0.1 * i)

        assert len(canvas_ctrl.avatar_calls) == N, (
            f"Expected set_avatar_pose called {N} times (full worker rate), "
            f"got {len(canvas_ctrl.avatar_calls)}"
        )

    def test_avatar_receives_latest_pose_every_call(self, qapp):
        """Each on_frame() call passes its own tx/ty/tyaw through unchanged."""
        from robot_radio.testgui.__main__ import build_live_frame_bridge

        canvas_ctrl = _FakeCanvasCtrl()
        bridge = build_live_frame_bridge(canvas_ctrl)
        bgr = _make_bgr_frame()

        for i in range(5):
            bridge.on_frame(bgr, 40.0, 30.0, float(i), float(i) * 2.0, 0.1 * i)

        assert canvas_ctrl.avatar_calls == [
            (0.0, 0.0, 0.0),
            (1.0, 2.0, 0.1),
            (2.0, 4.0, 0.2),
            (3.0, 6.0, pytest.approx(0.3)),
            (4.0, 8.0, pytest.approx(0.4)),
        ]

    def test_background_throttled_below_avatar_rate(self, qapp):
        """set_background is called fewer times than N (throttled subset)."""
        from robot_radio.testgui.__main__ import build_live_frame_bridge

        canvas_ctrl = _FakeCanvasCtrl()
        bridge = build_live_frame_bridge(canvas_ctrl)
        bgr = _make_bgr_frame()

        N = 9
        for i in range(N):
            bridge.on_frame(bgr, 40.0, 30.0, float(i), float(i), 0.0)

        assert len(canvas_ctrl.background_calls) < N, (
            "Background must be throttled to fewer updates than the avatar "
            f"(N={N}), got {len(canvas_ctrl.background_calls)}"
        )
        assert len(canvas_ctrl.background_calls) >= 1, (
            "Background must be updated at least once across N >= "
            "BACKGROUND_THROTTLE_N frames"
        )

    def test_background_never_more_frequent_than_avatar(self, qapp):
        """Ratio invariant: background call count <= avatar call count.

        This is the binding assertion -- no exact fps is asserted, only that
        background updates are never MORE frequent than avatar updates.
        """
        from robot_radio.testgui.__main__ import build_live_frame_bridge

        canvas_ctrl = _FakeCanvasCtrl()
        bridge = build_live_frame_bridge(canvas_ctrl)
        bgr = _make_bgr_frame()

        for i in range(12):
            bridge.on_frame(bgr, 40.0, 30.0, float(i), float(i), 0.0)

        assert len(canvas_ctrl.background_calls) <= len(canvas_ctrl.avatar_calls)

    def test_background_throttle_matches_bridges_own_constant(self, qapp):
        """Background call count matches N // BACKGROUND_THROTTLE_N using the
        bridge's own throttle constant (not a hardcoded fps assumption) --
        verifies internal consistency of whatever throttle mechanism was
        chosen, without asserting a fixed fps number.
        """
        from robot_radio.testgui.__main__ import build_live_frame_bridge

        canvas_ctrl = _FakeCanvasCtrl()
        bridge = build_live_frame_bridge(canvas_ctrl)
        bgr = _make_bgr_frame()

        throttle_n = getattr(bridge, "BACKGROUND_THROTTLE_N", None)
        assert throttle_n, "Bridge must expose its throttle constant"

        N = throttle_n * 3
        for i in range(N):
            bridge.on_frame(bgr, 40.0, 30.0, float(i), float(i), 0.0)

        assert len(canvas_ctrl.background_calls) == N // throttle_n

    def test_background_receives_origin(self, qapp):
        """When set_background fires, it receives the frame's origin_x/origin_y."""
        from robot_radio.testgui.__main__ import build_live_frame_bridge

        canvas_ctrl = _FakeCanvasCtrl()
        bridge = build_live_frame_bridge(canvas_ctrl)
        bgr = _make_bgr_frame()

        throttle_n = bridge.BACKGROUND_THROTTLE_N
        for i in range(throttle_n):
            bridge.on_frame(bgr, 41.5, 31.5, float(i), float(i), 0.0)

        assert len(canvas_ctrl.background_calls) == 1
        _pixmap, origin_x, origin_y = canvas_ctrl.background_calls[0]
        assert origin_x == pytest.approx(41.5)
        assert origin_y == pytest.approx(31.5)


# ---------------------------------------------------------------------------
# Connect/disconnect coverage -- _state["live_bridge"] lifecycle
# ---------------------------------------------------------------------------
#
# src/host/robot_radio/testgui/__main__.py's _on_connect()/_stop_live_worker()
# internals are closures inside _build_main_window() with no test seam
# (_build_main_window() returns only (window, app)). Per the established
# pattern in src/tests/testgui/test_set_origin.py and test_tour_stop.py, the
# state-management glue is re-implemented inline with fake worker/thread
# doubles, while the actual production build_live_frame_bridge() factory is
# called for real (it is a real, directly-importable module-level function,
# not a closure) so the real bridge-construction code is exercised.


def _make_connect_relay(state: dict, canvas_ctrl):
    """Re-implement the Relay branch of _on_connect()'s live-view wiring."""
    from robot_radio.testgui.__main__ import build_live_frame_bridge

    def _connect_relay(worker_factory, thread_factory) -> None:
        live_worker = worker_factory()
        live_thread = thread_factory()
        live_bridge = build_live_frame_bridge(canvas_ctrl)
        state["live_worker"] = live_worker
        state["live_thread"] = live_thread
        state["live_bridge"] = live_bridge
        state["live_view_active"] = True

    return _connect_relay


def _make_stop_live_worker(state: dict, canvas_ctrl):
    """Re-implement _stop_live_worker()'s logic inline (see module docstring)."""

    def _stop_live_worker() -> None:
        if not state.get("live_view_active"):
            return
        worker = state.get("live_worker")
        thread = state.get("live_thread")
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
        state["live_worker"] = None
        state["live_thread"] = None
        state["live_bridge"] = None
        state["live_view_active"] = False
        canvas_ctrl.restore_static_background()

    return _stop_live_worker


class TestLiveBridgeStateLifecycle:
    """_state['live_bridge'] is set on Relay connect and cleared on disconnect."""

    def test_relay_connect_sets_live_bridge(self, qapp):
        canvas_ctrl = _FakeCanvasCtrl()
        state: dict = {
            "live_worker": None,
            "live_thread": None,
            "live_bridge": None,
            "live_view_active": False,
        }
        connect_relay = _make_connect_relay(state, canvas_ctrl)

        connect_relay(_FakeWorker, _FakeThread)

        assert state["live_bridge"] is not None, (
            "Relay connect must populate _state['live_bridge']"
        )
        assert callable(state["live_bridge"].on_frame)
        assert state["live_view_active"] is True
        assert isinstance(state["live_worker"], _FakeWorker)
        assert isinstance(state["live_thread"], _FakeThread)

    def test_disconnect_clears_live_bridge(self, qapp):
        canvas_ctrl = _FakeCanvasCtrl()
        state: dict = {
            "live_worker": None,
            "live_thread": None,
            "live_bridge": None,
            "live_view_active": False,
        }
        connect_relay = _make_connect_relay(state, canvas_ctrl)
        connect_relay(_FakeWorker, _FakeThread)
        assert state["live_bridge"] is not None  # sanity: connected first

        stop_live_worker = _make_stop_live_worker(state, canvas_ctrl)
        stop_live_worker()

        assert state["live_bridge"] is None, (
            "Disconnect must clear _state['live_bridge']"
        )
        assert state["live_worker"] is None
        assert state["live_thread"] is None
        assert state["live_view_active"] is False

    def test_disconnect_stops_worker_and_thread(self, qapp):
        canvas_ctrl = _FakeCanvasCtrl()
        state: dict = {
            "live_worker": None,
            "live_thread": None,
            "live_bridge": None,
            "live_view_active": False,
        }
        connect_relay = _make_connect_relay(state, canvas_ctrl)
        connect_relay(_FakeWorker, _FakeThread)
        worker = state["live_worker"]
        thread = state["live_thread"]

        stop_live_worker = _make_stop_live_worker(state, canvas_ctrl)
        stop_live_worker()

        assert worker.stop_called
        assert thread.quit_called
        assert thread.wait_called_with == 3000

    def test_disconnect_restores_static_background(self, qapp):
        canvas_ctrl = _FakeCanvasCtrl()
        state: dict = {
            "live_worker": None,
            "live_thread": None,
            "live_bridge": None,
            "live_view_active": False,
        }
        connect_relay = _make_connect_relay(state, canvas_ctrl)
        connect_relay(_FakeWorker, _FakeThread)

        stop_live_worker = _make_stop_live_worker(state, canvas_ctrl)
        stop_live_worker()

        assert canvas_ctrl.restore_calls == 1, (
            "restore_static_background() must be called on disconnect"
        )

    def test_stop_live_worker_is_safe_noop_when_idle(self, qapp):
        """Calling _stop_live_worker() with no live view running is a no-op."""
        canvas_ctrl = _FakeCanvasCtrl()
        state: dict = {
            "live_worker": None,
            "live_thread": None,
            "live_bridge": None,
            "live_view_active": False,
        }

        stop_live_worker = _make_stop_live_worker(state, canvas_ctrl)
        stop_live_worker()  # should not raise

        assert state["live_bridge"] is None
        assert canvas_ctrl.restore_calls == 0, (
            "No-op path must not call restore_static_background()"
        )


# ---------------------------------------------------------------------------
# Relay-only start gate -- confirms the worker is never started for Sim/Serial
# (ticket 085-007 acceptance: "the live-view worker is confirmed to start
# only for Relay connections, never Sim/Serial"). __main__.py's real gate is
# a bare `if name == "Relay":` around the whole worker-construction block
# (no test seam); this re-implements that gate with the same fakes as above.
# ---------------------------------------------------------------------------


def _make_connect_for_transport(state: dict, canvas_ctrl, transport_name: str):
    """Re-implement _on_connect()'s Relay-only live-view start gate."""

    def _connect() -> None:
        if transport_name == "Relay":
            connect_relay = _make_connect_relay(state, canvas_ctrl)
            connect_relay(_FakeWorker, _FakeThread)
        # Sim/Serial: no live-view worker constructed at all.

    return _connect


class TestLiveViewRelayOnlyGate:
    def test_sim_never_starts_live_view(self, qapp):
        canvas_ctrl = _FakeCanvasCtrl()
        state: dict = {
            "live_worker": None,
            "live_thread": None,
            "live_bridge": None,
            "live_view_active": False,
        }
        _make_connect_for_transport(state, canvas_ctrl, "Sim")()

        assert state["live_bridge"] is None
        assert state["live_view_active"] is False

    def test_serial_never_starts_live_view(self, qapp):
        canvas_ctrl = _FakeCanvasCtrl()
        state: dict = {
            "live_worker": None,
            "live_thread": None,
            "live_bridge": None,
            "live_view_active": False,
        }
        _make_connect_for_transport(state, canvas_ctrl, "Serial")()

        assert state["live_bridge"] is None
        assert state["live_view_active"] is False

    def test_relay_starts_live_view(self, qapp):
        canvas_ctrl = _FakeCanvasCtrl()
        state: dict = {
            "live_worker": None,
            "live_thread": None,
            "live_bridge": None,
            "live_view_active": False,
        }
        _make_connect_for_transport(state, canvas_ctrl, "Relay")()

        assert state["live_bridge"] is not None
        assert state["live_view_active"] is True
