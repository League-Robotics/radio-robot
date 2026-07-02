---
id: '009'
title: 'Live playfield: main-thread frame bridge with throttled background and full-rate avatar'
status: done
use-cases:
- SUC-011
depends-on:
- '008'
github-issue: ''
issue: testgui-playfield-not-live-updating.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Live playfield: main-thread frame bridge with throttled background and full-rate avatar

## Description

The playfield background stopped live-updating in PLAYFIELD MODE (Relay), even
though aprilcam itself is confirmed healthy (`testgui-playfield-not-live-updating.md`
verified via MCP: `open_camera`, `capture_frame`, and `get_tags` all return
good data for the calibrated playfield camera).

**Root cause (confirmed by code reading).** In `__main__.py::_on_connect()`,
the Relay path wires the live-view worker's signal directly to a bare
function:

```python
live_worker.frame_ready.connect(
    _on_live_frame, Qt.ConnectionType.QueuedConnection
)
```

In this PySide build, a `QueuedConnection` to a non-`QObject` callable (a bare
function, not a bound method of a `QObject` receiver) is delivered **on the
emitting (worker) thread**, not the GUI thread — the same class of bug that
previously caused the tour/GOTO segfault and motivated the existing
`_WorkerBridge` pattern used for `_TourRunner`/`_GotoRunner`. The worker's
`run()` loop (`live_view.py::_LiveViewWorker.run()`) never returns to its own
thread's Qt event loop (it's a tight `while not self._stop` loop with
`time.sleep()`), so those "queued" `frame_ready` deliveries are never actually
processed on any event loop — `_on_live_frame` never runs, and the canvas
never repaints from the camera.

## Stakeholder Decision (binding)

- Route `frame_ready` through a **main-thread `QObject` bridge** — the same
  pattern as the existing `_RXBridge` / `_TelemetryBridge` / `_WorkerBridge`
  in `__main__.py` — so delivery reliably happens on the GUI thread regardless
  of this PySide build's `QueuedConnection`-to-bare-function quirk.
- Target background image update rate: **3–4 fps** (lower is acceptable — do
  not chase a higher rate at the cost of complexity).
- The **avatar pose must stay smooth**: update `canvas_ctrl.set_avatar_pose(...)`
  on **every** frame the worker emits (full worker rate, ~9–12 Hz), even when
  the background pixmap conversion + `set_background(...)` call is throttled.
  Implementation hint from the stakeholder: the worker already emits every
  frame; the bridge's slot updates the avatar every time it fires, but only
  converts the BGR ndarray to `QPixmap` and calls `set_background` on a
  throttled subset of those calls (e.g. every 3rd frame, or gated by elapsed
  wall-clock time since the last background update).

## Affected Code

- `host/robot_radio/testgui/__main__.py` — live-view wiring in `_on_connect()`
  (around the `if name == "Relay":` block that constructs `live_worker`/
  `live_thread`), and the `_on_live_frame` function.
- `host/robot_radio/testgui/live_view.py` — `_LiveViewWorker` itself is
  **unchanged** (per the issue's fix direction: "only the delivery mechanism
  needs bridging"). Do not modify its emit rate or loop structure.

## Dependency

Depends on ticket 008 (camera-selection plumbing). The live-view worker's
`_capture_and_emit` (in `live_view.py`) will be updated by ticket 008 to
resolve its camera via the shared `camera_prefs.select_camera(...)` helper
instead of `cams[0]`. This ticket only touches the *delivery* side
(`frame_ready` signal wiring and the throttle), not camera selection — but it
executes after 008 so it is built on top of the corrected camera-resolution
code rather than the old `cams[0]` behavior, avoiding a rebase/merge of
overlapping edits to `live_view.py`'s docstring and `_capture_and_emit`.

## Acceptance Criteria

### Main-thread bridge

- [x] A new bridge class (e.g. `_LiveFrameBridge(QObject)`, following the
      exact pattern documented on `_WorkerBridge`'s docstring) is created on
      the GUI thread in `_on_connect()`'s Relay branch, and its bound-method
      slot — not a bare function — is what `frame_ready` connects to.
- [x] `live_worker.frame_ready.connect(bridge.on_frame, Qt.ConnectionType.QueuedConnection)`
      replaces the direct `_on_live_frame` connection.
- [x] The bridge instance is kept alive in `_state` (e.g.
      `_state["live_bridge"]`) for the lifetime of the connection — a dropped
      reference would silently break delivery, exactly as documented for
      `_WorkerBridge`.
- [x] `_stop_live_worker()` clears `_state["live_bridge"]` alongside the
      existing `live_worker`/`live_thread` cleanup.

### Full-rate avatar, throttled background

- [x] The bridge's frame slot calls `canvas_ctrl.set_avatar_pose(tx, ty, tyaw)`
      on **every** invocation (every frame the worker emits).
- [x] The bridge's frame slot converts `bgr` to `QPixmap`
      (`_bgr_ndarray_to_pixmap`) and calls
      `canvas_ctrl.set_background(pm, origin_x=..., origin_y=...)` only on a
      throttled subset of invocations, targeting ~3–4 fps given the worker's
      ~9–12 Hz emit rate (e.g. update background on every 3rd frame, or
      time-gate on `time.monotonic()` elapsed since the last background
      update — implementer's choice, documented in a comment).
- [x] A background update rate below the 3–4 fps target (e.g. 1–2 fps under
      load) is acceptable and must not fail any test — do not assert an exact
      fps in tests; assert the *ratio* (avatar updates every frame,
      background updates less often) and that background updates are never
      *more* frequent than avatar updates.

### No regressions

- [x] On relay connect, the canvas background visibly updates from the live
      camera (previously verified broken; must now work) — verified via the
      headless test asserting `set_background` is actually invoked at least
      once across N simulated `frame_ready` emissions.
- [x] On relay disconnect, `restore_static_background()` is still called and
      the live worker/thread/bridge are all torn down (extend the existing
      `_stop_live_worker()` coverage).
- [x] Sim and Serial transports still do not start any live-view worker or
      bridge (unchanged from ticket 003 / SUC-004).
- [x] Existing `tests/testgui/test_live_view.py` and `test_canvas.py` tests
      pass unchanged.
- [x] `_LiveViewWorker` in `live_view.py` is not modified (loop rate, signal
      signature, and last-known-tag-pose semantics all stay exactly as
      delivered in ticket 003).

## Implementation Plan

### Approach

In `__main__.py`, near the existing `_WorkerBridge` class definition, add:

```python
class _LiveFrameBridge(QObject):
    """Marshals live-view worker frames onto the Qt GUI main thread.

    Mirrors _WorkerBridge: a QueuedConnection to a bare function is delivered
    on the *emitting* thread in this PySide build (the live-view worker never
    returns to its own event loop, so such deliveries are never processed —
    see testgui-playfield-not-live-updating.md). This bridge is constructed on
    the GUI thread, so its bound-method slot runs on the GUI thread.

    Avatar pose updates on every frame (full worker rate); the background
    QPixmap conversion + canvas_ctrl.set_background() call is throttled to
    ~3-4 fps so as not to burn GUI-thread time on every ~80ms tick.
    """

    #: Convert+set the background every Nth frame (~9-12 Hz worker / 3 ≈ 3-4 fps).
    BACKGROUND_THROTTLE_N = 3

    def __init__(self) -> None:
        super().__init__()
        self._frame_count = 0

    @Slot(object, float, float, float, float, float)
    def on_frame(
        self,
        bgr: object,
        origin_x: float,
        origin_y: float,
        tx: float,
        ty: float,
        tyaw: float,
    ) -> None:
        # Avatar: every frame, full worker rate.
        canvas_ctrl.set_avatar_pose(tx, ty, tyaw)
        # Background: throttled subset.
        self._frame_count += 1
        if self._frame_count % self.BACKGROUND_THROTTLE_N != 0:
            return
        from robot_radio.testgui.operations import _bgr_ndarray_to_pixmap
        pm = _bgr_ndarray_to_pixmap(bgr)
        if pm is not None:
            canvas_ctrl.set_background(pm, origin_x=origin_x, origin_y=origin_y)
```

In `_on_connect()`'s Relay branch, replace:

```python
live_worker.frame_ready.connect(
    _on_live_frame, Qt.ConnectionType.QueuedConnection
)
```

with:

```python
live_bridge = _LiveFrameBridge()
live_worker.frame_ready.connect(
    live_bridge.on_frame, Qt.ConnectionType.QueuedConnection
)
_state["live_bridge"] = live_bridge
```

In `_stop_live_worker()`, add `_state["live_bridge"] = None` alongside the
existing `live_worker`/`live_thread` clears.

The standalone `_on_live_frame(...)` function can either be removed (if
nothing else calls it) or left in place if it is reused elsewhere — check for
other callers before deleting.

### Files to modify

- `host/robot_radio/testgui/__main__.py`

### Testing Plan

`_LiveFrameBridge` is a real `QObject` class, constructible directly in a
headless test with an offscreen `QApplication` (unlike the tour/goto closures,
this bridge can be lifted out and tested in isolation if defined at module
level, or exercised via the same "re-implement the logic inline" pattern used
in `test_set_origin.py` if it stays a nested class — prefer defining
`_LiveFrameBridge` so it is reachable for direct construction in a test if
feasible; otherwise mirror the pattern).

- **Existing tests to run**: `QT_QPA_PLATFORM=offscreen uv run python -m pytest
  tests/testgui/ -q`.
- **New tests to write** (e.g. `tests/testgui/test_live_frame_bridge.py`, or
  extend `test_live_view.py` / `test_smoke.py`):
  - Construct a `_LiveFrameBridge`-equivalent (imported directly, or
    reimplemented per the established closure-testing pattern) with fake
    `canvas_ctrl` stand-ins recording call counts (mirroring
    `_FakeCanvasCtrl` in `test_set_origin.py`).
  - Emit/call `on_frame(...)` N times (e.g. N=9) with varying `tx/ty/tyaw`;
    assert `set_avatar_pose` was called exactly N times.
  - Assert `set_background` was called fewer times than N (throttled), and at
    least once across N ≥ `BACKGROUND_THROTTLE_N` calls.
  - Assert `set_background` call count is never greater than
    `set_avatar_pose` call count (ratio invariant, not a fixed fps
    assertion).
  - A connect/disconnect smoke test (extending existing live-view coverage)
    confirming `_state["live_bridge"]` is set on Relay connect and cleared on
    disconnect.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run python -m pytest
  tests/testgui/ -q`

### Documentation updates

- Add/extend the `_LiveFrameBridge` docstring (shown above) explaining the
  bare-function `QueuedConnection` pitfall and cross-referencing
  `testgui-playfield-not-live-updating.md` and the existing `_WorkerBridge`
  docstring, matching the file's established documentation style for this
  class of bug.
- Update `__main__.py`'s module docstring / the Relay connect comment to
  describe the throttled-background / full-rate-avatar behavior.
