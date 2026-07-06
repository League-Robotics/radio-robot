---
id: "007"
title: "Live camera view verification"
status: open
use-cases: [SUC-008]
depends-on: []
github-issue: ""
issue: host-testgui-full-revival.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Live camera view verification

## Description

`live_view.py`'s `build_live_view_worker` and `__main__.py`'s
`_TelemetryBridge`/`_LiveFrameBridge` already implement the full live
camera view: on a Relay (PLAYFIELD MODE) connect, `_LiveViewWorker` starts
on its own thread, repeatedly captures + deskews (via the aprilcam daemon's
live homography) and delivers frames through `_LiveFrameBridge` (a
main-thread bridge — a bare-function `QueuedConnection` callback would run
on the emitting thread and never process, per the established PySide
gotcha `testgui-playfield-not-live-updating.md`). While live view is
active, the camera — not `TLM`-driven telemetry — owns the avatar marker:
`_TelemetryBridge.on_frame_ready` calls
`canvas_ctrl.refresh(update_marker=False)` so trace paths still redraw at
TLM rate without fighting the camera-driven marker position; `on_truth_ready`
skips `refresh()` entirely when live view is inactive. Disconnect (or
leaving Relay mode) stops the worker and restores the static background.

This code predates the greenfield rebuild. This ticket ports the three
un-ported test files covering it and verifies the worker/gating logic
against the real connect flow, fixing anything a real run surfaces.

## Acceptance Criteria

- [ ] `tests_old/testgui/test_live_view.py`,
      `test_live_frame_bridge.py`, and `test_telemetry_gating.py` are
      ported to `tests/testgui/`, updated for any API drift, and pass
      under `QT_QPA_PLATFORM=offscreen`.
- [ ] The live-view worker is confirmed to start only for Relay
      connections, never Sim/Serial.
- [ ] `on_frame_ready`'s live-view gate (`refresh(update_marker=False)`
      while active) and `on_truth_ready`'s full gate (skip `refresh()`
      entirely when inactive) both hold.
- [ ] Stopping live view (disconnect, or leaving Relay mode) stops the
      worker thread and restores the static background with no lingering
      thread.
- [ ] `_deskew_bgr_ndarray`'s homography/corner-warp math (shared with
      `operations.py`'s Refresh Playfield) is confirmed unaffected.
- [ ] Any genuine bug surfaced by a real run is fixed here and documented.

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression).
- **New tests to write**: port the three files above.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
