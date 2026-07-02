---
status: done
sprint: '063'
tickets:
- 063-011
---

# TestGUI: Avatar jumps wildly in PLAYFIELD live view — TLM refresh fights the camera bridge

## Symptom

In PLAYFIELD MODE with the live view running (post ticket 009), the avatar
"jumps all over the place" — teleporting rapidly between two positions.

## Root cause (confirmed by code reading, 2026-07-01)

Two drivers now reposition the avatar, in **different coordinate frames**:

1. `_LiveFrameBridge.on_frame` → `canvas_ctrl.set_avatar_pose(tx, ty, tyaw)`
   — camera tag-100 pose, **A1-centred world frame**, every worker frame
   (~9–12 Hz). Correct per design: in live view the camera drives the avatar.
2. `_TelemetryBridge.on_frame_ready` (STREAM 50 telemetry, ~20 Hz) →
   `canvas_ctrl.refresh(fused_yaw_rad)` → `_update_marker(...)` repositions
   the marker at the latest **fused-telemetry pose** — robot-internal frame
   whose origin is wherever the robot was last zeroed.

The design intent was already documented and half-implemented: the
`on_truth_ready` slot skips `canvas_ctrl.refresh()` when
`_state["live_view_active"]` ("the avatar is driven by the camera, not fused
telemetry"). But the equivalent gate was **never added to `on_frame_ready`**,
the TLM slot. Before ticket 009 this was invisible because live-view frame
delivery was broken (the QueuedConnection-to-bare-function bug) — only the TLM
driver ever ran. Fixing delivery unmasked the fight.

## Fix

- Give `CanvasController.refresh()` a way to rebuild trace paths **without
  touching the marker** (e.g. `refresh(fused_yaw_rad=None, *,
  update_marker=True)` keyword, or a `refresh_traces_only()` method).
- In `_TelemetryBridge.on_frame_ready`: when `_state["live_view_active"]`,
  still `trace_model.feed(frame)` and rebuild/redraw traces, but do NOT update
  the marker — the camera bridge owns the avatar in live view. When not in
  live view, behavior is unchanged (`refresh(fused_yaw_rad)`).
- `on_truth_ready`'s existing gate stays; with the TLM slot now redrawing
  traces marker-free at ~20 Hz, the truth polyline still appears in live view.

## Affected code

- `host/robot_radio/testgui/canvas.py` — `refresh` / `_update_marker`.
- `host/robot_radio/testgui/__main__.py` — `_TelemetryBridge.on_frame_ready`.
- Tests: `tests/testgui/` (canvas refresh marker-skip; telemetry-slot gating
  in live mode — fake canvas ctrl counting `set_avatar_pose` vs marker
  updates).

## Relation

Unmasked by [[testgui-playfield-not-live-updating]] (ticket 009). Same GUI
session also surfaced [[testgui-relay-discovery-passive-banner-fails]]
(ticket 010, fixed).
