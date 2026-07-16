"""src/tests/testgui/test_telemetry_gating.py -- ticket 085-007: live camera
view verification (``_TelemetryBridge.on_frame_ready``/``on_truth_ready``
live-view gating, ticket 063-011). Ported from
``tests_old/testgui/test_telemetry_gating.py``.

Root cause (testgui-live-view-avatar-fight-tlm-vs-camera.md): in PLAYFIELD live
view, two drivers reposition the avatar in different coordinate frames --
``_LiveFrameBridge.on_frame`` (camera pose, A1-centred world frame, correct
owner in live view) and ``_TelemetryBridge.on_frame_ready`` (STREAM 50 TLM,
fused-telemetry pose, robot-internal frame). ``on_truth_ready`` was already
gated on ``_state["live_view_active"]`` (camera owns the avatar in live view),
but the same gate was missing on ``on_frame_ready``, so the two drivers fought
over the marker's position at ~20 Hz vs ~10 Hz.

The fix: when ``live_view_active`` is True, ``on_frame_ready`` calls
``canvas_ctrl.refresh(update_marker=False)`` -- trace paths (including the
magenta fused trace) still redraw at TLM rate, but the marker is left alone.
Otherwise (Sim/Serial, or Relay before live view starts) it calls
``canvas_ctrl.refresh(fused_yaw_rad)`` exactly as before.

``src/host/robot_radio/testgui/__main__.py``'s ``_TelemetryBridge`` is a closure
inside ``_build_main_window()`` with no test seam. Per the established pattern
in ``src/tests/testgui/test_set_origin.py`` / ``test_tour_stop.py`` /
``test_live_frame_bridge.py``, these tests re-implement ``on_frame_ready``'s
exact gating logic inline with a fake canvas ctrl (recording refresh calls and
kwargs) and a fake ``_state``.

New in this ticket (085-007): the pre-rebuild file only covered
``on_frame_ready``'s gate -- ``on_truth_ready``'s full gate (skip
``refresh()`` entirely when live view is inactive... wait, ACTIVE -- see
``TestOnTruthReadyLiveViewGating`` below) had no dedicated test. Added to
close that gap, per this ticket's own acceptance criterion.

Qt-free: no QApplication required.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_telemetry_gating.py -v
"""

from __future__ import annotations

import math
import queue

import pytest


# ---------------------------------------------------------------------------
# Fake doubles (Qt-free)
# ---------------------------------------------------------------------------


class _FakeCanvasCtrl:
    """Stand-in for CanvasController -- records every refresh() call's args/kwargs."""

    def __init__(self):
        self.refresh_calls: list[tuple[tuple, dict]] = []

    def refresh(self, *args, **kwargs) -> None:
        self.refresh_calls.append((args, kwargs))


class _FakeTraceModel:
    """Stand-in for TraceModel -- records fed frames/truth poses."""

    def __init__(self):
        self.fed_frames: list[object] = []
        self.fed_truth: list[tuple[float, float, float]] = []

    def feed(self, frame) -> None:
        self.fed_frames.append(frame)

    def feed_truth(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
        self.fed_truth.append((x_cm, y_cm, yaw_rad))


def _make_frame(pose: tuple[int, int, int] | None = (0, 0, 0)):
    from robot_radio.robot.protocol import TLMFrame
    return TLMFrame(t=0, enc=None, otos=None, pose=pose)


def _make_on_frame_ready(state: dict, trace_model, canvas_ctrl, pending_frames):
    """Re-implement _TelemetryBridge.on_frame_ready's exact gating logic inline.

    Mirrors the production code in
    ``src/host/robot_radio/testgui/__main__.py::_TelemetryBridge.on_frame_ready``.
    """

    def on_frame_ready() -> None:
        while True:
            try:
                frame = pending_frames.get_nowait()
            except Exception:
                break
            trace_model.feed(frame)
            fused_yaw_rad = None
            if frame.pose is not None:
                fused_yaw_rad = math.radians(frame.pose[2] / 100.0)
            if state.get("live_view_active"):
                canvas_ctrl.refresh(update_marker=False)
            else:
                canvas_ctrl.refresh(fused_yaw_rad)

    return on_frame_ready


def _make_on_truth_ready(state: dict, trace_model, canvas_ctrl):
    """Re-implement _TelemetryBridge.on_truth_ready's exact gating logic inline.

    Mirrors the production code in
    ``src/host/robot_radio/testgui/__main__.py::_TelemetryBridge.on_truth_ready``:
    always feeds the camera-truth trace, but only calls ``canvas_ctrl.refresh()``
    when live view is INACTIVE (in live view, the camera live-view worker's own
    ``set_avatar_pose`` already owns the marker -- calling ``refresh()`` here too
    would be a redundant redraw fighting that worker).
    """

    def on_truth_ready(x_cm: float, y_cm: float, yaw_rad: float) -> None:
        trace_model.feed_truth(x_cm, y_cm, yaw_rad)
        if not state.get("live_view_active"):
            canvas_ctrl.refresh()

    return on_truth_ready


# ---------------------------------------------------------------------------
# live_view_active=True -- marker must not be touched
# ---------------------------------------------------------------------------


class TestOnFrameReadyLiveViewActive:
    def test_refresh_called_with_update_marker_false(self):
        """In live view, refresh() is called with update_marker=False."""
        state = {"live_view_active": True}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()
        pending: "queue.Queue" = queue.Queue()
        pending.put(_make_frame(pose=(1000, 500, 9000)))

        on_frame_ready = _make_on_frame_ready(state, trace_model, canvas_ctrl, pending)
        on_frame_ready()

        assert len(canvas_ctrl.refresh_calls) == 1
        args, kwargs = canvas_ctrl.refresh_calls[0]
        assert kwargs.get("update_marker") is False, (
            f"Expected refresh(update_marker=False), got args={args} kwargs={kwargs}"
        )

    def test_no_fused_positional_arg_passed_in_live_view(self):
        """In live view, refresh() must not be called with a positional
        fused_yaw_rad argument -- only the update_marker=False kwarg."""
        state = {"live_view_active": True}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()
        pending: "queue.Queue" = queue.Queue()
        pending.put(_make_frame(pose=(0, 0, 4500)))

        on_frame_ready = _make_on_frame_ready(state, trace_model, canvas_ctrl, pending)
        on_frame_ready()

        args, kwargs = canvas_ctrl.refresh_calls[0]
        assert args == (), (
            f"refresh() must not receive a positional fused_yaw_rad arg in live "
            f"view (camera bridge owns the marker), got args={args}"
        )

    def test_trace_still_fed_in_live_view(self):
        """Even with the marker gated off, the fused trace is still fed."""
        state = {"live_view_active": True}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()
        pending: "queue.Queue" = queue.Queue()
        frame = _make_frame(pose=(100, 200, 0))
        pending.put(frame)

        on_frame_ready = _make_on_frame_ready(state, trace_model, canvas_ctrl, pending)
        on_frame_ready()

        assert trace_model.fed_frames == [frame]

    def test_multiple_frames_all_gated_in_live_view(self):
        """Every queued frame in a batch is refreshed with update_marker=False."""
        state = {"live_view_active": True}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()
        pending: "queue.Queue" = queue.Queue()
        for i in range(4):
            pending.put(_make_frame(pose=(i * 100, 0, 0)))

        on_frame_ready = _make_on_frame_ready(state, trace_model, canvas_ctrl, pending)
        on_frame_ready()

        assert len(canvas_ctrl.refresh_calls) == 4
        for args, kwargs in canvas_ctrl.refresh_calls:
            assert kwargs.get("update_marker") is False


# ---------------------------------------------------------------------------
# live_view_active=False -- unchanged prior behaviour
# ---------------------------------------------------------------------------


class TestOnFrameReadyLiveViewInactive:
    def test_refresh_called_with_fused_yaw(self):
        """Outside live view, refresh(fused_yaw_rad) is called as before."""
        state = {"live_view_active": False}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()
        pending: "queue.Queue" = queue.Queue()
        pending.put(_make_frame(pose=(0, 0, 9000)))  # 9000 centideg = 90 deg

        on_frame_ready = _make_on_frame_ready(state, trace_model, canvas_ctrl, pending)
        on_frame_ready()

        assert len(canvas_ctrl.refresh_calls) == 1
        args, kwargs = canvas_ctrl.refresh_calls[0]
        assert kwargs == {}, f"Expected no kwargs outside live view, got {kwargs}"
        assert len(args) == 1
        assert args[0] == pytest.approx(math.radians(90.0))

    def test_refresh_called_with_none_when_no_pose(self):
        """When frame.pose is None, refresh(None) is called (unchanged heading)."""
        state = {"live_view_active": False}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()
        pending: "queue.Queue" = queue.Queue()
        pending.put(_make_frame(pose=None))

        on_frame_ready = _make_on_frame_ready(state, trace_model, canvas_ctrl, pending)
        on_frame_ready()

        args, kwargs = canvas_ctrl.refresh_calls[0]
        assert args == (None,)

    def test_missing_live_view_active_key_defaults_to_inactive(self):
        """If _state lacks the 'live_view_active' key entirely, behave as if False
        (Sim/Serial transports never set it)."""
        state: dict = {}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()
        pending: "queue.Queue" = queue.Queue()
        pending.put(_make_frame(pose=(0, 0, 0)))

        on_frame_ready = _make_on_frame_ready(state, trace_model, canvas_ctrl, pending)
        on_frame_ready()

        args, kwargs = canvas_ctrl.refresh_calls[0]
        assert kwargs == {}
        assert args == (0.0,)


# ---------------------------------------------------------------------------
# on_truth_ready's gate (085-007: no dedicated test in the pre-rebuild file --
# added here to close the gap). Always feeds the truth trace; refresh() is
# skipped ENTIRELY (not called with any variant of arguments) while live view
# is active, since the live-view worker's own set_avatar_pose already owns
# the marker and a redundant refresh() would fight it.
# ---------------------------------------------------------------------------


class TestOnTruthReadyLiveViewGating:
    def test_truth_ready_skips_refresh_entirely_when_live_view_active(self):
        state = {"live_view_active": True}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()

        on_truth_ready = _make_on_truth_ready(state, trace_model, canvas_ctrl)
        on_truth_ready(12.0, 34.0, 0.5)

        assert canvas_ctrl.refresh_calls == [], (
            "on_truth_ready must not call refresh() at all while live view is "
            "active -- the live-view worker's set_avatar_pose already owns "
            "the marker"
        )

    def test_truth_ready_still_feeds_trace_when_live_view_active(self):
        """Even with refresh() gated off, the camera-truth trace is still fed."""
        state = {"live_view_active": True}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()

        on_truth_ready = _make_on_truth_ready(state, trace_model, canvas_ctrl)
        on_truth_ready(12.0, 34.0, 0.5)

        assert trace_model.fed_truth == [(12.0, 34.0, 0.5)]

    def test_truth_ready_calls_refresh_when_live_view_inactive(self):
        state = {"live_view_active": False}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()

        on_truth_ready = _make_on_truth_ready(state, trace_model, canvas_ctrl)
        on_truth_ready(1.0, 2.0, 0.3)

        assert len(canvas_ctrl.refresh_calls) == 1
        args, kwargs = canvas_ctrl.refresh_calls[0]
        assert args == () and kwargs == {}, (
            "outside live view, refresh() is called with no arguments"
        )
        assert trace_model.fed_truth == [(1.0, 2.0, 0.3)]

    def test_truth_ready_missing_live_view_active_key_defaults_to_inactive(self):
        """No 'live_view_active' key at all (Sim/Serial) behaves as inactive."""
        state: dict = {}
        trace_model = _FakeTraceModel()
        canvas_ctrl = _FakeCanvasCtrl()

        on_truth_ready = _make_on_truth_ready(state, trace_model, canvas_ctrl)
        on_truth_ready(5.0, 6.0, 0.0)

        assert len(canvas_ctrl.refresh_calls) == 1


# ---------------------------------------------------------------------------
# Real production code -- canvas_ctrl.refresh signature accepts update_marker
# ---------------------------------------------------------------------------


class TestRefreshSignatureAcceptsUpdateMarker:
    """Sanity check against the real CanvasController.refresh signature (not a
    fake) so the gating tests above stay honest about what production code
    actually accepts."""

    def test_refresh_accepts_update_marker_kwarg(self):
        import inspect
        from robot_radio.testgui.canvas import CanvasController

        sig = inspect.signature(CanvasController.refresh)
        assert "update_marker" in sig.parameters
        assert sig.parameters["update_marker"].default is True
        assert sig.parameters["update_marker"].kind == inspect.Parameter.KEYWORD_ONLY
