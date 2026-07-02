---
status: done
tickets:
- NONE
---

# TestGUI: set_background() yanks the avatar off the tag — live-view jumping persists after ticket 011

## Symptom

In PLAYFIELD live view the avatar still "jumps back and forth" between two
fixed poses, ~3–7×/sec — **even with telemetry stopped** (stakeholder
verified). Ticket 011 (TLM-slot gating) did not fix it.

## Root cause (captured numerically, 2026-07-01)

A headless repro running the real live worker + bridge against the real
camera, with `set_avatar_pose`/`set_background` instrumented, shows:

```
AVATAR : world=(-27.67,-16.77) -> px=(314.3, 489.4)   # camera pose, steady
BACKGRD: size=(1074x714) ... marker_after=(535.8, 355.4)   # <-- yanked!
AVATAR : ... -> px=(314.5, 489.5)                      # bridge restores
BACKGRD: ... marker_after=(535.8, 355.2)               # yanked again
```

`CanvasController.set_background()` (canvas.py, ~line 649) internally calls
`self.refresh()` with the default `update_marker=True`, so **every background
swap repositions the marker via `_update_marker()`** — from the fused trace,
or its world-(0,0)/canvas-centre fallback when no fused data exists. The live
worker calls `set_background` every ~3rd frame (ticket 009 throttle), so the
marker ping-pongs between the camera pose and the fused/centre pose at the
background rate. Telemetry on/off only changes *where* the yank lands
(robot-internal fused pose vs. centre fallback) — not whether it happens.

Ticket 011 fixed the TLM slot's direct `refresh()` call but missed this
internal call inside `set_background`.

## Fix — lock the avatar to the tag (stakeholder requirement)

In `CanvasController`:

- `set_avatar_pose(x_cm, y_cm, yaw_rad)` stores the pose, e.g.
  `self._live_pose = (x_cm, y_cm, yaw_rad)`.
- `set_background(...)`: after rebuilding the transform/traces, do NOT run the
  fused-trace marker update when a live pose is set. Instead re-apply
  `self._live_pose` through the NEW transform (this is also more correct: the
  origin may have shifted slightly, and re-mapping keeps the avatar glued to
  the tag pixel-accurately). Concretely: replace the internal `self.refresh()`
  with `self.refresh(update_marker=(self._live_pose is None))`, then if
  `_live_pose` is set, re-apply it via `set_avatar_pose(*self._live_pose)`.
- `restore_static_background()` clears `_live_pose = None` so SIM/BENCH
  behavior (fused-driven marker) is fully restored on disconnect.

Result: in live view the ONLY thing that can move the avatar is a new camera
pose (position **and yaw** locked to the tag).

## Affected code

- `host/robot_radio/testgui/canvas.py` — `set_background`, `set_avatar_pose`,
  `restore_static_background`.
- Tests: `tests/testgui/test_canvas.py` (+ possibly `test_live_frame_bridge.py`)
  — after `set_avatar_pose(...)`, calling `set_background(...)` must leave the
  marker at the (re-mapped) live pose, NOT the fused/centre pose; after
  `restore_static_background()`, fused-driven behavior returns.

## Relation

Follow-up to [[testgui-live-view-avatar-fight-tlm-vs-camera]] (ticket 011 —
necessary but insufficient) and [[testgui-playfield-not-live-updating]]
(ticket 009, which unmasked this by making background swaps actually happen).
