---
id: '063'
title: 'Mode-driven Test GUI: live playfield camera, mode indicator, relay auto-discovery'
status: planning-docs
branch: sprint/063-mode-driven-test-gui-live-playfield-camera-mode-indicator-relay-auto-discovery
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
issues:
- live-camera-view-for-the-test-gui.md
- testgui-record-pause-stop-command-log.md
- testgui-set-robot-zero-full-reset.md
- testgui-tour-stop-reactivation.md
- testgui-tour-sim-mode-gating.md
- testgui-playfield-not-live-updating.md
- testgui-playfield-oneshot-grab-and-camera-selection.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 063: Mode-driven Test GUI: live playfield camera, mode indicator, relay auto-discovery

## Goals

1. Add a prominent **mode indicator** label near the top of the Test GUI window that reflects
   the selected transport: SIM MODE / BENCH MODE / PLAYFIELD MODE.
2. Make the canvas **background follow the mode**: Sim and Serial transports keep the static
   playfield + telemetry-driven avatar; Relay transport (PLAYFIELD MODE) switches to a
   continuously-updated live deskewed camera view with the avatar placed over the real robot
   tag (tag id 100).
3. Implement **relay auto-discovery**: clicking Connect in Relay mode enumerates serial ports,
   probes each candidate for the relay banner, and opens the correct port automatically — no
   manual port entry required.

## Problem

The Test GUI currently requires the user to know and type the serial port for relay connections.
Once connected, the canvas always shows the static playfield regardless of whether the robot
is actually on the playfield in front of a camera. This makes PLAYFIELD mode indistinguishable
from BENCH mode in terms of visual feedback, and it forces the user to manually update the
background via the "Refresh Playfield" button when in live-camera scenarios.

## Solution

- **Mode indicator**: A `QLabel` inserted at the top of the right panel is updated whenever
  `transport_combo.currentIndexChanged` fires. The label text and color are derived from a
  pure function `transport_name_to_mode_label(name)` that is testable without Qt.
- **Live-view worker**: A `_LiveViewWorker` `QObject` (background thread) loops at ~10–15 Hz,
  calls the aprilcam daemon to capture + deskew a frame, and emits a signal carrying the BGR
  ndarray + origin + tag-100 pose. The Qt main thread receives the signal, builds the `QPixmap`,
  calls `canvas_ctrl.set_background(...)`, and calls a new `canvas_ctrl.set_avatar_pose(x_cm,
  y_cm, yaw_rad)` to place the avatar directly from the camera tag. The existing deskew logic
  in `operations.py` is refactored to a Qt-free `_deskew_bgr_ndarray(raw_bgr, tag_frame)`
  helper that returns `(ndarray, origin_x, origin_y)` without building a QPixmap.
- **Mode-gated wiring in `__main__.py`**: `_on_connect()` inspects the selected transport
  name. For Relay it starts `_LiveViewWorker`; on disconnect (or transport change) it stops
  the worker and restores the static background via `canvas_ctrl.restore_static_background()`.
- **Canvas live-mode**: `CanvasController` gains `set_avatar_pose(x_cm, y_cm, yaw_rad)` and
  `restore_static_background()`. Avatar position in playfield mode is driven by the camera
  tag rather than fused telemetry.
- **Relay auto-discovery**: A pure-Python `find_relay_port(port_list, probe_fn)` function in
  `transport.py` opens each candidate, reads the banner, and returns the port whose banner
  contains `RADIOBRIDGE`. `_on_connect()` in `__main__.py` calls this when Relay is selected
  and no port is pre-filled, logging success/failure clearly.

## Success Criteria

- Selecting Sim/Serial/Relay in the transport combo shows SIM MODE / BENCH MODE / PLAYFIELD
  MODE prominently near the top of the window.
- Clicking Connect in Relay mode discovers the relay port automatically; the log shows a
  clear success or "no relay found" message.
- In PLAYFIELD MODE (relay connected) the canvas shows a live-updated deskewed camera view;
  the avatar moves to track the real robot tag (tag 100) from the camera, not fused telemetry.
- In SIM/BENCH MODE the canvas behaves exactly as before (static/placeholder background,
  avatar from telemetry).
- On disconnect from relay, the canvas reverts to the static/placeholder background.
- All headless tests pass: `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui -q`.

## Scope

### In Scope

- Mode indicator label in `testgui/__main__.py` (right panel, top).
- `transport_name_to_mode_label()` pure helper in `testgui/__main__.py`.
- Relay auto-discovery: `find_relay_port()` in `testgui/transport.py`.
- `_on_connect()` relay path updated to use auto-discovery and hide `port_edit`.
- Deskew refactor: `_deskew_bgr_ndarray()` Qt-free helper extracted from
  `operations._deskew_bgr_with_tag_frame()` into `operations.py`.
- `_LiveViewWorker` in `testgui/__main__.py` (or `testgui/live_view.py`) with
  `~10–15 Hz` loop, daemon calls off-thread, BGR ndarray emitted via signal.
- `CanvasController.set_avatar_pose(x_cm, y_cm, yaw_rad)` and
  `CanvasController.restore_static_background()` in `testgui/canvas.py`.
- Mode-gated background wiring in `_on_connect()` / `_on_disconnect()`.
- Headless tests for all Qt-free logic in `tests/testgui/`.

### Out of Scope

- Serial transport auto-discovery (bench mode retains manual port entry).
- Camera selection UI (uses the same daemon + camera as existing "Refresh Playfield").
- Any firmware changes.
- Altering the `rogo` CLI or any module outside `host/robot_radio/testgui/` and
  `tests/testgui/`.

## Test Strategy

All tests are headless (`QT_QPA_PLATFORM=offscreen`). Tier: `tests/testgui/`.

- **Qt-free unit tests** (no QApplication): `transport_name_to_mode_label()`,
  `find_relay_port()` with a fake probe function, `_deskew_bgr_ndarray()` with a
  fake TagFrame, avatar pose gating logic.
- **Qt widget tests** (QApplication via offscreen platform): mode indicator label text
  and color updates on transport combo change; canvas `set_avatar_pose()` and
  `restore_static_background()` behavior; `_LiveViewWorker` signal delivery with a
  mocked daemon.
- Mirror the patterns in `tests/testgui/test_operations.py` (fake transport, MagicMock
  for daemon, deferred PySide6 imports in fixtures).

## Architecture Notes

See `architecture-update.md` for the full design. Key decisions:
- The deskew returns a BGR ndarray off-thread; QPixmap is built on the Qt main thread.
- `find_relay_port()` is Qt-free and takes an injectable `probe_fn` for testability.
- `_LiveViewWorker` is a `QThread`-less `QObject` moved to a `QThread`; the signal
  `frame_ready(ndarray, float, float, float, float)` crosses the thread boundary via
  `QueuedConnection`.
- `CanvasController` does not know it is in "live mode"; the main window is responsible
  for routing avatar position from either telemetry or camera tag.

## GitHub Issues

(No GitHub issues linked at sprint creation time.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Mode indicator and transport-combo plumbing | — |
| 002 | Relay auto-discovery in transport.py | 001 |
| 003 | Live-view worker, canvas live-mode, and mode-gated wiring | 001, 002 |
| 004 | Set Robot @ 0,0: full pose reset (heading + encoders + SI command) | — |
| 005 | Record / Pause / Stop command+response logging | — |
| 006 | Functional simulated OTOS for EKF fusion and heading-reset testing | — |
| 007 | Tour/GOTO stop reactivation and Sim-mode tour gating | — |
| 008 | Camera-selection: persisted pull-down for the playfield camera | — |
| 009 | Live playfield: main-thread frame bridge with throttled background and full-rate avatar | 008 |

Tickets 001–003 are done. Tickets 004, 005, and 006 are independent of each
other and of 001–003; they may execute in any order after 003 is done. Ticket
006 touches firmware (`source/hal/sim/`) and the `tests/_infra/sim` host
harness rather than the Test GUI itself, but is included in this sprint
because it directly enables testing ticket 004's heading-reset behaviour
(SUC-005) in Sim mode.

Tickets 007–009 were added after four newly-linked issues surfaced live bugs
in the Test GUI (`testgui-tour-stop-reactivation.md`,
`testgui-tour-sim-mode-gating.md`,
`testgui-playfield-not-live-updating.md`,
`testgui-playfield-oneshot-grab-and-camera-selection.md`). Ticket 007 is
independent and may run any time after 003. Ticket 008 (camera-selection
plumbing) must precede ticket 009 (live-view main-thread bridge) because the
live-view worker's camera resolution is touched by 008 and the live-frame
delivery fix in 009 is built on top of it — tickets execute serially in
number order: 007, then 008, then 009.
