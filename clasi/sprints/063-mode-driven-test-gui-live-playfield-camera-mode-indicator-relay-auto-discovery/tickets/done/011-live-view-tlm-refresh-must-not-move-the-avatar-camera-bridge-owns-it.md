---
id: '011'
title: 'Live view: TLM refresh must not move the avatar (camera bridge owns it)'
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: testgui-live-view-avatar-fight-tlm-vs-camera.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Live view: TLM refresh must not move the avatar (camera bridge owns it)

## Description

In PLAYFIELD live view, the avatar "jumps all over the place" because two
drivers reposition it in different coordinate frames:

- `_LiveFrameBridge.on_frame` → `canvas_ctrl.set_avatar_pose(camera pose)` —
  A1-centred world frame, ~10 Hz. This is the correct owner of the avatar in
  live view.
- `_TelemetryBridge.on_frame_ready` (STREAM 50 TLM, ~20 Hz) →
  `canvas_ctrl.refresh(fused_yaw_rad)` → `_update_marker(...)` — repositions
  the marker at the fused-telemetry pose (robot-internal frame, origin =
  wherever last zeroed). This was NOT gated on live view.

`on_truth_ready` already had the gate (`if not
_state.get("live_view_active"): canvas_ctrl.refresh()`), with a docstring
stating the camera owns the avatar in live view. The same gate was never
added to `on_frame_ready`. The bug was invisible until ticket 009 fixed
live-frame delivery, at which point both drivers started actually running
concurrently.

The fix:

1. `CanvasController.refresh()` gains a keyword-only `update_marker: bool =
   True` parameter. When `False`, trace paths still rebuild
   (`_update_traces()`) and the scene still repaints (`_scene.update()`), but
   `_update_marker(...)` is not called — the marker is left exactly where it
   was. Default `True` preserves every existing caller's behaviour.
2. `_TelemetryBridge.on_frame_ready` now checks
   `_state.get("live_view_active")`: when `True`, it calls
   `canvas_ctrl.refresh(update_marker=False)` (traces still redraw at TLM
   rate; marker untouched — the camera bridge owns the avatar). Otherwise it
   is unchanged: `canvas_ctrl.refresh(fused_yaw_rad)`.
3. `on_truth_ready` is unchanged.
4. `_LiveFrameBridge`, `set_avatar_pose`, and `live_view.py` are untouched —
   this ticket only stops the second driver from fighting the first; it does
   not change who wins.

## Acceptance Criteria

- [x] `CanvasController.refresh()` accepts a keyword-only `update_marker: bool
      = True` parameter; when `False`, traces rebuild and the scene updates
      but `_update_marker` is not called, so the marker position/rotation is
      unchanged.
- [x] `_TelemetryBridge.on_frame_ready` calls
      `canvas_ctrl.refresh(update_marker=False)` when
      `_state.get("live_view_active")` is truthy, and
      `canvas_ctrl.refresh(fused_yaw_rad)` otherwise (unchanged behaviour).
- [x] `on_truth_ready` is unchanged (still gated exactly as before).
- [x] `_LiveFrameBridge`, `set_avatar_pose`, and `live_view.py` are not
      modified.
- [x] Module docstring in `__main__.py` updated to state that BOTH telemetry
      slots leave the marker alone in live view.
- [x] All existing testgui and simulation tests remain green; new tests added
      for both the `refresh(update_marker=False)` canvas behaviour and the
      `on_frame_ready` gating logic.

## Testing

- **Existing tests to run**:
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
  - `uv run python -m pytest tests/simulation -q`
- **New tests to write**:
  - `tests/testgui/test_canvas.py::TestRefreshUpdateMarkerParam` — feeds a
    `TraceModel` with fused points, calls `refresh(update_marker=False)` and
    asserts `_marker_group` position/rotation are unchanged, while a
    subsequent default `refresh()` does move it; also asserts trace paths
    still rebuild with `update_marker=False`.
  - `tests/testgui/test_telemetry_gating.py` — re-implements
    `on_frame_ready`'s gating logic inline (per the
    `test_set_origin.py`/`test_tour_stop.py`/`test_live_frame_bridge.py`
    pattern) with a fake canvas ctrl recording `refresh()` calls/kwargs and a
    fake `_state`; asserts `live_view_active=True` →
    `refresh(update_marker=False)` with no positional fused-yaw arg, and
    `live_view_active=False` → `refresh(fused_yaw_rad)` as before (including
    the `frame.pose is None` and missing-key cases).
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run python -m
  pytest tests/testgui/ -q && uv run python -m pytest tests/simulation -q`

## Results

- `tests/testgui/` — 474 passed (baseline 463 + 11 new: 3 in
  `test_canvas.py`, 8 in the new `test_telemetry_gating.py`).
- `tests/simulation` — 2421 passed (baseline 2421, unchanged).
