---
status: done
sprint: '063'
tickets:
- 063-009
---

# TestGUI: Playfield does not live-update from the camera in Relay/Playfield mode

## Symptom

Previously (during the earlier camera work) the playfield image updated live
(~1 Hz) so the operator could watch the robot move on the field in near real
time. That live update no longer happens.

## Root cause (confirmed by code; AprilCam verified healthy)

In Relay mode `_on_connect` starts a `_LiveViewWorker` (from
`testgui/live_view.py`) on a `QThread`. The worker's `run()` is a ~12 Hz loop
that **never returns to its thread's event loop**. Its `frame_ready` signal is
connected directly to the bare function `_on_live_frame` with
`Qt.ConnectionType.QueuedConnection`:

```python
live_worker.frame_ready.connect(_on_live_frame, Qt.ConnectionType.QueuedConnection)
```

In this PySide build, a `QueuedConnection` to a non-`QObject` callable is
delivered on the **worker** thread (the same behavior that caused the tour/GOTO
segfault). Because the worker thread is stuck in `run()` and never re-enters
`exec()`, those `frame_ready` events are never processed → `_on_live_frame`
never runs → the canvas never repaints from the camera.

## AprilCam is NOT the problem (verified 2026-07-01 via MCP)

- `open_camera(index=3)` → calibrated playfield `pf_arducam-ov9782-usb-camera`
  ("main-playfield", 134.3 × 89.3 cm).
- `capture_frame` returns a clean deskewed top-down image of the field with the
  robot visible.
- `get_tags` returns tag 100 (robot) with `world_xy ≈ [-4.3, 40.1] cm`,
  heading, plus the origin tag 1.

## Fix direction

Route `frame_ready` through a **main-thread `QObject` bridge** (the same pattern
as `_RXBridge` / `_TelemetryBridge` / the new `_WorkerBridge`), so the frame is
delivered and painted on the GUI thread. QPixmap construction and canvas updates
must happen on the main thread.

## Affected code

- `host/robot_radio/testgui/__main__.py` — live-worker wiring in `_on_connect`,
  `_on_live_frame`.
- `host/robot_radio/testgui/live_view.py` — `_LiveViewWorker` (unchanged logic;
  only the delivery mechanism needs bridging).
