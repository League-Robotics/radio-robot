---
id: '007'
title: 'Operations panel: sync-pose, zero-encoders, STOP, clear-traces, refresh-playfield, STREAM toggle'
status: open
use-cases:
- SUC-008
- SUC-009
- SUC-010
- SUC-011
- SUC-012
- SUC-013
depends-on:
- '004'
- '005'
issue: plan-robot-test-gui-pyside6.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 007 — Operations panel: sync-pose, zero-encoders, STOP, clear-traces, refresh-playfield, STREAM toggle

## Description

Implement the six operations buttons in `app.py`. These are one-click actions
that send a firmware command or modify the GUI state (traces, playfield image).
The Sync Pose button reuses the `cmd_sync_pose` / `_daemon_read_pose` logic from
`host/robot_radio/io/cli.py:268` rather than reimplementing the aprilcam read
and `P` command sequence.

This ticket has an open question (OQ-1 from architecture): verify that
`cmd_sync_pose` is cleanly importable from `cli.py` without triggering Click's
CLI registration. If it is not, extract the pose-sync helper to a shared
location in `testkit/` or `robot/` as part of this ticket.

Corresponds to item 5 in the approved design's ticket breakdown.

## Acceptance Criteria

- [ ] **Sync pose from camera** button: reads the aprilcam daemon pose for tag
  100 and sends firmware `P <x_mm> <y_mm> <h_cdeg>`. Logs the pose values and
  the firmware reply. On aprilcam daemon not available: logs a warning; does
  not crash.
- [ ] **Zero encoders** button: sends the firmware zero-encoder command. Logs
  the command and reply.
- [ ] **STOP** button: sends `X` or `STOP`. Logs the command.
- [ ] **Clear traces** button: clears all four trace polylines in the trace
  model (ticket 008 will actually render them; the trace model clear method
  must be callable from here). Does NOT send any transport command.
- [ ] **Refresh playfield from cam 3** button: captures a new playfield image
  from the aprilcam daemon; replaces the canvas background `QPixmap`. Logs the
  action. On daemon not available: logs a warning.
- [ ] **STREAM on/off** toggle: when toggling on, sends `STREAM 50`; when
  toggling off, sends `STREAM 0`. Button label reflects current state.
- [ ] All buttons except Clear Traces and STREAM are disabled when no transport
  is connected.
- [ ] Programmer documents the resolution of OQ-1 (cmd_sync_pose importability)
  in the commit message.
- [ ] `uv run python -m pytest tests/simulation` passes.

## Implementation Plan

### Approach

Before writing: read `host/robot_radio/io/cli.py` lines 260-300 to evaluate
`cmd_sync_pose` importability. Check whether `@main.command` decorator wraps it
in a way that breaks standalone import. If yes, extract the core logic to a
helper (e.g., `host/robot_radio/robot/sync_pose.py`) and note this in the commit.

For Refresh playfield: use `read_camera_pose` or the aprilcam frame-grab API.
If `testkit/camera.py` does not provide a frame-grab method, use a simple
`QPixmap.fromImage` conversion from whatever the aprilcam daemon provides.

### Files to modify

- `host/robot_radio/testgui/app.py` — add operations panel widget with six
  buttons; wire handlers

### Files to create (if OQ-1 extraction is needed)

- `host/robot_radio/robot/sync_pose.py` (or similar) — extracted pose-sync helper

### Reuse

- `host/robot_radio/io/cli.py:268` — `cmd_sync_pose` / `_daemon_read_pose` (or extraction)
- `host/robot_radio/testkit/camera.py` — `read_camera_pose` for Sync Pose + Refresh Playfield

### Testing plan

Manual with Sim (for STOP, Zero, Clear, STREAM, Refresh): connect to sim, click
each button, verify log output. Manual with relay/hardware (for Sync Pose):
confirm firmware acknowledges `P` command and trace model resets. Run simulation
gate.

### Documentation updates

None yet. README is written in ticket 010.
