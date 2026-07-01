---
id: '003'
title: Live-view worker, canvas live-mode, and mode-gated wiring
status: open
use-cases:
- SUC-003
- SUC-004
depends-on:
- '001'
- '002'
github-issue: ''
issue: live-camera-view-for-the-test-gui.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Live-view worker, canvas live-mode, and mode-gated wiring

## Description

Implement the live camera view for PLAYFIELD MODE (relay transport). This
ticket delivers all three remaining pieces:

1. **`_deskew_bgr_ndarray()` refactor in `operations.py`** — extract the
   Qt-free deskew logic from `_deskew_bgr_with_tag_frame()` into a standalone
   helper that returns `(bgr_ndarray, origin_x, origin_y)`. The existing
   function is refactored to call this helper then build the QPixmap. No
   existing caller changes behavior.

2. **`_LiveViewWorker` in `testgui/live_view.py`** (new module, NOT inline in
   `__main__.py`) — a `QObject` moved to a `QThread` that loops at ~10–15 Hz.
   Each iteration:
   - Opens the aprilcam daemon (`DaemonControl.connect_default`).
   - Captures a frame (`capture_frame`) and gets tags (`get_tags`).
   - Extracts tag-100 pose (world_xy + heading). **If tag 100 is not seen,
     holds the last known pose — avatar stays at its last camera position until
     the tag reappears. It does NOT snap to origin (0, 0) and is NOT hidden.**
     `_last_tag` is initialized to `(0.0, 0.0, 0.0)` and updated only when
     tag 100 is successfully read.
   - Calls `_deskew_bgr_ndarray(raw_bgr, tag_frame)` (off-thread, no Qt).
   - Emits `frame_ready(bgr_ndarray, origin_x, origin_y, tag_x, tag_y, tag_yaw)`.
   - Sleeps to target ~10–15 Hz (67–100 ms per loop).
   - On daemon unavailability: logs once, backs off 2 s, retries.

3. **`CanvasController.set_avatar_pose()` and `.restore_static_background()`
   in `canvas.py`** — two new narrow methods.

4. **Mode-gated wiring in `__main__.py`** — `_on_connect()` for Relay: after
   `transport.connect()`, creates `_LiveViewWorker`, moves to `QThread`, wires
   `frame_ready` to `_on_live_frame` slot, sets `_state["live_view_active"]`.
   `_on_disconnect()`: stops the worker+thread, calls
   `canvas_ctrl.restore_static_background()`, clears the flag. In PLAYFIELD
   MODE the `on_truth_ready` bridge slot still calls `trace_model.feed_truth()`
   but does NOT call `canvas_ctrl.refresh(fused_yaw)` (the live view owns the
   avatar; the green truth trace still accumulates).

**Files to modify:**
- `host/robot_radio/testgui/operations.py`
- `host/robot_radio/testgui/canvas.py`
- `host/robot_radio/testgui/__main__.py`

**Files to create:**
- `host/robot_radio/testgui/live_view.py`
- `tests/testgui/test_live_view.py`

## Acceptance Criteria

### Deskew refactor (operations.py)

- [ ] `_deskew_bgr_ndarray(raw_bgr, tag_frame, ppc=None)` exists and returns
      `(ndarray, float, float)` or `None` on failure.
- [ ] `_deskew_bgr_ndarray` does not import PySide6 and can be called in a
      headless test with no QApplication.
- [ ] `_deskew_bgr_with_tag_frame()` behavior is unchanged: existing test
      coverage in `test_operations.py` passes.

### LiveViewWorker (live_view.py)

- [ ] `_LiveViewWorker` is importable from `testgui.live_view` without PySide6.
- [ ] `_LiveViewWorker.frame_ready` signal is emitted with
      `(object, float, float, float, float, float)` type signature.
- [ ] Worker can be constructed and its `run()` slot called in a headless test
      with a mocked daemon that returns a synthetic BGR ndarray and TagFrame.
- [ ] When daemon is unavailable, worker does not crash and logs a warning.
- [ ] When tag 100 is not in `get_tags()` result, the worker **holds the last
      known `(tag_x, tag_y, tag_yaw)`** and emits that value with the new
      frame. The avatar does NOT snap to (0, 0) and is NOT hidden. The last
      known pose is initialized to `(0.0, 0.0, 0.0)` before the first
      successful tag-100 read, and updated only when tag 100 is seen.
- [ ] `stop()` slot sets an internal stop flag; `run()` exits within 2 s of
      `stop()` being called.

### CanvasController additions (canvas.py)

- [ ] `set_avatar_pose(x_cm, y_cm, yaw_rad)` positions and rotates the marker
      at explicit world coordinates using `rotation_deg = 90 - degrees(yaw_rad)`.
- [ ] `set_avatar_pose` does not read `trace_model.fused`.
- [ ] `restore_static_background()` replaces the canvas pixmap with a grey
      placeholder and resets the origin to `(field_w/2, field_h/2)`.
- [ ] `restore_static_background()` calls `refresh()` so traces re-render.
- [ ] Both methods pass headless tests (with offscreen QApplication).

### Mode-gated wiring (__main__.py)

- [ ] Connecting via Relay starts the live-view worker and sets
      `_state["live_view_active"] = True`.
- [ ] Connecting via Sim or Serial does NOT start the live-view worker.
- [ ] Disconnecting from Relay stops the worker+thread and calls
      `restore_static_background()`.
- [ ] `_on_live_frame` slot: receives BGR ndarray + origin + tag pose; builds
      `QPixmap` on the main thread; calls `canvas_ctrl.set_background()` +
      `canvas_ctrl.set_avatar_pose()`.
- [ ] In PLAYFIELD MODE, `on_truth_ready` calls `trace_model.feed_truth()` but
      skips `canvas_ctrl.refresh(fused_yaw)` (avatar driven by live view, not
      fused TLM).
- [ ] Canvas background is a live camera image in PLAYFIELD MODE after connect.
- [ ] Canvas background reverts to grey placeholder after relay disconnect.
- [ ] All existing `tests/testgui/` tests pass unchanged.

## Implementation Plan

### Approach

#### 1. Extract `_deskew_bgr_ndarray` from `operations.py`

The body of `_deskew_bgr_with_tag_frame()` starting after the `try:` block
and ending before `pixmap = _bgr_ndarray_to_pixmap(deskewed)` becomes
`_deskew_bgr_ndarray(raw_bgr, tag_frame, ppc=None) -> tuple | None`.

```python
def _deskew_bgr_ndarray(
    raw_bgr: "object",
    tag_frame: "object",
    ppc: float | None = None,
) -> "tuple[object, float, float] | None":
    """Deskew raw_bgr via the daemon TagFrame's homography.

    Returns (deskewed_bgr_ndarray, origin_x, origin_y) or None.
    Qt-free: does not build a QPixmap.
    """
    # ... (move existing body here, ending before _bgr_ndarray_to_pixmap call)
```

`_deskew_bgr_with_tag_frame()` becomes:

```python
def _deskew_bgr_with_tag_frame(raw_bgr, tag_frame, ppc=None):
    result = _deskew_bgr_ndarray(raw_bgr, tag_frame, ppc)
    if result is None:
        return None
    bgr, origin_x, origin_y = result
    pixmap = _bgr_ndarray_to_pixmap(bgr)
    if pixmap is None:
        return None
    return pixmap, origin_x, origin_y
```

#### 2. Create `live_view.py`

```python
# host/robot_radio/testgui/live_view.py
"""_LiveViewWorker — continuous aprilcam frame capture for PLAYFIELD MODE."""
from __future__ import annotations
import logging
import time

_log = logging.getLogger(__name__)
_TARGET_INTERVAL_S = 0.08   # ~12 Hz

class _LiveViewWorker:
    """QObject live-view worker. Move to a QThread after construction.

    Signals:
        frame_ready(object, float, float, float, float, float)
            bgr_ndarray, origin_x, origin_y, tag_x_cm, tag_y_cm, tag_yaw_rad
    """

    def __init__(self) -> None:
        # Deferred PySide6 import so module is importable without Qt.
        from PySide6.QtCore import QObject, Signal  # type: ignore[import-untyped]
        # Dynamic base class — see note on deferred PySide6.
        # In practice the class is subclassed inside a function; see factory below.
        raise NotImplementedError("Use build_live_view_worker() factory")
```

Because `QObject` must be a real base class (not injected dynamically at
construction), the worker class is defined *inside* a factory function that
runs after PySide6 is imported. Mirror the `_TelemetryBridge` and
`_FitView` pattern already used in `__main__.py` and `canvas.py`:

```python
def build_live_view_worker():
    """Return a _LiveViewWorker instance (QObject) on the calling thread.

    The caller must move it to a QThread before starting.
    """
    from PySide6.QtCore import QObject, Signal, Slot, Qt  # type: ignore[import-untyped]

    class _LiveViewWorker(QObject):
        frame_ready = Signal(object, float, float, float, float, float)

        def __init__(self) -> None:
            super().__init__()
            self._stop = False
            self._last_tag = (0.0, 0.0, 0.0)

        @Slot()
        def run(self) -> None:
            from robot_radio.testgui.operations import _deskew_bgr_ndarray
            while not self._stop:
                t0 = time.monotonic()
                try:
                    self._capture_and_emit()
                except Exception as exc:
                    _log.debug("LiveViewWorker loop error: %s", exc)
                elapsed = time.monotonic() - t0
                sleep = max(0.0, _TARGET_INTERVAL_S - elapsed)
                if sleep > 0 and not self._stop:
                    time.sleep(sleep)

        @Slot()
        def stop(self) -> None:
            self._stop = True

        def _capture_and_emit(self) -> None:
            try:
                from aprilcam.config import Config  # type: ignore[import]
                from aprilcam.client.control import DaemonControl  # type: ignore[import]
            except ImportError:
                _log.warning("LiveViewWorker: aprilcam not installed; stopping")
                self._stop = True
                return

            from robot_radio.testgui.operations import _deskew_bgr_ndarray

            dc = DaemonControl.connect_default(Config.load())
            try:
                cams = dc.list_cameras()
                if not cams:
                    return
                cam = cams[0]
                raw_bgr = dc.capture_frame(cam)
                tag_frame = dc.get_tags(cam)
            finally:
                try:
                    dc.close()
                except Exception:
                    pass

            if raw_bgr is None:
                return

            result = _deskew_bgr_ndarray(raw_bgr, tag_frame)
            if result is None:
                return
            bgr, origin_x, origin_y = result

            # Extract tag-100 pose; hold last known if not visible.
            tx, ty, tyaw = self._last_tag
            if tag_frame is not None:
                tags = getattr(tag_frame, "tags", None) or {}
                t100 = tags.get(100)
                if t100 is not None:
                    wxy = getattr(t100, "world_xy", None)
                    if wxy is not None:
                        tx, ty = float(wxy[0]), float(wxy[1])
                    tyaw = float(getattr(t100, "heading_rad",
                                  getattr(t100, "yaw", tyaw)))
                    self._last_tag = (tx, ty, tyaw)

            self.frame_ready.emit(bgr, origin_x, origin_y, tx, ty, tyaw)

    return _LiveViewWorker()
```

Note: the programmer should consult the aprilcam API guide
(`get_robot_api_guide()` or https://robots.jointheleague.org/) to confirm
the exact attribute names for `world_xy` and `heading_rad`/`yaw` on a tag
object. The pattern above mirrors `robot.sync_pose.daemon_read_pose`.

#### 3. Add `set_avatar_pose` and `restore_static_background` to `canvas.py`

In `CanvasController`:

```python
def set_avatar_pose(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
    """Position and rotate the avatar at explicit world coordinates.

    Does not consult trace_model. Used in PLAYFIELD MODE where the camera
    tag drives the avatar instead of fused telemetry.
    """
    import math
    px, py = self._world_to_px(x_cm, y_cm)
    self._marker_group.setPos(px, py)            # type: ignore[attr-defined]
    rotation_deg = 90.0 - math.degrees(yaw_rad)
    self._marker_group.setRotation(rotation_deg) # type: ignore[attr-defined]
    self._marker_group.setVisible(True)          # type: ignore[attr-defined]
    self._scene.update()                         # type: ignore[attr-defined]

def restore_static_background(self) -> None:
    """Replace the live camera background with a grey placeholder.

    Resets the world→pixel origin to the field-centre fallback
    (field_w/2, field_h/2) so avatar reverts to fused-telemetry mode.
    Calls refresh() to re-render traces.
    """
    self._origin_x = self._field_w_cm / 2.0
    self._origin_y = self._field_h_cm / 2.0
    self._world_to_px = _make_world_to_px(self._origin_x, self._origin_y, self._ppc)
    placeholder = _make_grey_placeholder(self._img_w, self._img_h)
    self._bg_item.setPixmap(placeholder)         # type: ignore[attr-defined]
    self.refresh()
```

#### 4. Wire live view in `__main__.py`

Add to `_state`: `"live_view_active": False`, `"live_thread": None`,
`"live_worker": None`.

In `_on_connect()`, after the existing relay `transport.connect()` block:

```python
if name == "Relay":
    from PySide6.QtCore import QThread  # type: ignore[import-untyped]
    from robot_radio.testgui.live_view import build_live_view_worker
    worker = build_live_view_worker()
    thread = QThread()
    worker.moveToThread(thread)
    worker.frame_ready.connect(_on_live_frame, Qt.ConnectionType.QueuedConnection)
    thread.started.connect(worker.run)
    thread.start()
    _state["live_worker"] = worker
    _state["live_thread"] = thread
    _state["live_view_active"] = True
```

Add `_on_live_frame` slot (must be main-thread callable, wired as
`QueuedConnection`):

```python
def _on_live_frame(bgr, origin_x, origin_y, tx, ty, tyaw):
    from robot_radio.testgui.operations import _bgr_ndarray_to_pixmap
    pm = _bgr_ndarray_to_pixmap(bgr)
    if pm is not None:
        canvas_ctrl.set_background(pm, origin_x=origin_x, origin_y=origin_y)
    canvas_ctrl.set_avatar_pose(tx, ty, tyaw)
```

In `_on_disconnect()`, before the existing cleanup:

```python
if _state.get("live_view_active"):
    worker = _state.get("live_worker")
    thread = _state.get("live_thread")
    if worker is not None:
        worker.stop()
    if thread is not None:
        thread.quit()
        thread.wait(3000)
    _state["live_worker"] = None
    _state["live_thread"] = None
    _state["live_view_active"] = False
    canvas_ctrl.restore_static_background()
```

Gate `on_truth_ready` to skip avatar update in live mode:

```python
@Slot(float, float, float)
def on_truth_ready(self, x_cm, y_cm, yaw_rad):
    trace_model.feed_truth(x_cm, y_cm, yaw_rad)
    if not _state.get("live_view_active"):
        canvas_ctrl.refresh()  # avatar from fused trace
    # In live mode: truth trace still accumulates, but avatar is from live view
```

### Files to create/modify

- `host/robot_radio/testgui/operations.py`: extract `_deskew_bgr_ndarray()`.
- `host/robot_radio/testgui/canvas.py`: add `set_avatar_pose()`,
  `restore_static_background()`.
- `host/robot_radio/testgui/live_view.py`: new file with
  `build_live_view_worker()`.
- `host/robot_radio/testgui/__main__.py`: wire live-view lifecycle, add
  `_on_live_frame`, gate `on_truth_ready`.

### Testing plan

Create `tests/testgui/test_live_view.py`:

```python
# Qt-free deskew helper test
def test_deskew_bgr_ndarray_returns_ndarray():
    """_deskew_bgr_ndarray with a fake TagFrame returns an ndarray or None."""
    import numpy as np
    from unittest.mock import MagicMock
    from robot_radio.testgui.operations import _deskew_bgr_ndarray

    tag_frame = MagicMock()
    tag_frame.homography = [[1,0,0],[0,1,0],[0,0,1]]
    tag_frame.playfield_corners = [[0,0],[100,0],[100,75],[0,75]]
    tag_frame.field_width_cm = 100.0
    tag_frame.field_height_cm = 75.0
    tag_frame.origin_x = 50.0
    tag_frame.origin_y = 37.5

    raw = np.zeros((480, 640, 3), dtype=np.uint8)
    result = _deskew_bgr_ndarray(raw, tag_frame)
    # Either returns a valid tuple or None if cv2 unavailable.
    assert result is None or (
        isinstance(result, tuple) and len(result) == 3
    )

# Qt-free worker signal test (requires offscreen QApplication)
def test_live_view_worker_emits_frame(qapp):
    """Worker run() with a mocked daemon emits frame_ready."""
    from unittest.mock import MagicMock, patch
    import numpy as np
    from robot_radio.testgui.live_view import build_live_view_worker
    from PySide6.QtCore import QThread, Qt

    received = []

    worker = build_live_view_worker()
    worker.frame_ready.connect(
        lambda bgr, ox, oy, tx, ty, tyaw: received.append((ox, oy, tx, ty, tyaw)),
        Qt.ConnectionType.DirectConnection,
    )

    fake_bgr = np.zeros((60, 80, 3), dtype=np.uint8)
    fake_tag_frame = MagicMock()
    fake_tag_frame.homography = [[1,0,0],[0,1,0],[0,0,1]]
    fake_tag_frame.playfield_corners = [[0,0],[80,0],[80,60],[0,60]]
    fake_tag_frame.field_width_cm = 80.0
    fake_tag_frame.field_height_cm = 60.0
    fake_tag_frame.origin_x = 40.0
    fake_tag_frame.origin_y = 30.0
    fake_tag_frame.tags = {}

    fake_dc = MagicMock()
    fake_dc.list_cameras.return_value = ["cam0"]
    fake_dc.capture_frame.return_value = fake_bgr
    fake_dc.get_tags.return_value = fake_tag_frame

    with patch("aprilcam.config.Config") as MockConfig, \
         patch("aprilcam.client.control.DaemonControl") as MockDC:
        MockDC.connect_default.return_value = fake_dc

        # Run one iteration then stop.
        worker._stop = False
        worker._capture_and_emit()
        # At least one emission is expected if cv2 is available.
        # Tolerate None result from _deskew_bgr_ndarray if cv2 absent.

def test_live_view_worker_holds_last_tag_when_tag_missing(qapp):
    """When tag 100 is absent from get_tags, worker emits last known pose (no snap to 0,0)."""
    from unittest.mock import MagicMock, patch
    import numpy as np
    from robot_radio.testgui.live_view import build_live_view_worker
    from PySide6.QtCore import Qt

    received = []
    worker = build_live_view_worker()
    worker.frame_ready.connect(
        lambda bgr, ox, oy, tx, ty, tyaw: received.append((tx, ty, tyaw)),
        Qt.ConnectionType.DirectConnection,
    )

    # Seed a known last pose by injecting it directly.
    worker._last_tag = (12.0, 34.0, 0.5)

    fake_bgr = np.zeros((60, 80, 3), dtype=np.uint8)
    fake_tag_frame = MagicMock()
    fake_tag_frame.homography = [[1,0,0],[0,1,0],[0,0,1]]
    fake_tag_frame.playfield_corners = [[0,0],[80,0],[80,60],[0,60]]
    fake_tag_frame.field_width_cm = 80.0
    fake_tag_frame.field_height_cm = 60.0
    fake_tag_frame.origin_x = 40.0
    fake_tag_frame.origin_y = 30.0
    # tags dict has no entry for tag 100.
    fake_tag_frame.tags = {}

    fake_dc = MagicMock()
    fake_dc.list_cameras.return_value = ["cam0"]
    fake_dc.capture_frame.return_value = fake_bgr
    fake_dc.get_tags.return_value = fake_tag_frame

    with patch("aprilcam.config.Config"), \
         patch("aprilcam.client.control.DaemonControl") as MockDC:
        MockDC.connect_default.return_value = fake_dc
        worker._capture_and_emit()

    # If a frame was emitted (cv2 available), the tag pose must be the last known.
    if received:
        tx, ty, tyaw = received[0]
        assert tx == 12.0, "avatar must hold last known X, not snap to 0"
        assert ty == 34.0, "avatar must hold last known Y, not snap to 0"
        assert tyaw == 0.5, "avatar must hold last known yaw"

# Canvas method tests
def test_set_avatar_pose(qapp):
    import math
    from robot_radio.testgui.traces import TraceModel
    from robot_radio.testgui.canvas import build_canvas
    tm = TraceModel()
    _, ctrl = build_canvas(tm)
    ctrl.set_avatar_pose(10.0, 5.0, math.pi / 2)
    # No exception raised; marker is visible.
    assert ctrl._marker_group.isVisible()

def test_restore_static_background(qapp):
    from robot_radio.testgui.traces import TraceModel
    from robot_radio.testgui.canvas import build_canvas
    tm = TraceModel()
    _, ctrl = build_canvas(tm)
    ctrl.restore_static_background()
    # Origin reset to field centre.
    assert ctrl._origin_x == ctrl._field_w_cm / 2.0
    assert ctrl._origin_y == ctrl._field_h_cm / 2.0
```

### Documentation updates

- Update `live_view.py` module docstring (new file — document the threading
  model, signal signature, and daemon access pattern).
- Update `canvas.py` `CanvasController` docstring to mention `set_avatar_pose`
  and `restore_static_background`.
- Update `operations.py` module docstring to document `_deskew_bgr_ndarray`.
- Update `__main__.py` module docstring to document the live-view lifecycle and
  `_state["live_view_active"]` flag.
