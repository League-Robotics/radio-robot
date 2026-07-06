---
id: '007'
title: Live camera view verification
status: done
use-cases:
- SUC-008
depends-on: []
github-issue: ''
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

- [x] `tests_old/testgui/test_live_view.py`,
      `test_live_frame_bridge.py`, and `test_telemetry_gating.py` are
      ported to `tests/testgui/`, updated for any API drift, and pass
      under `QT_QPA_PLATFORM=offscreen`.
- [x] The live-view worker is confirmed to start only for Relay
      connections, never Sim/Serial.
- [x] `on_frame_ready`'s live-view gate (`refresh(update_marker=False)`
      while active) and `on_truth_ready`'s full gate (skip `refresh()`
      entirely when inactive) both hold.
- [x] Stopping live view (disconnect, or leaving Relay mode) stops the
      worker thread and restores the static background with no lingering
      thread.
- [x] `_deskew_bgr_ndarray`'s homography/corner-warp math (shared with
      `operations.py`'s Refresh Playfield) is confirmed unaffected.
- [x] Any genuine bug surfaced by a real run is fixed here and documented.

## Implementation notes (2026-07-06)

Ported all three files to `tests/testgui/` (56 tests total). **Zero
production code changes** — `live_view.py`, `build_live_frame_bridge`, and
`_TelemetryBridge` already work exactly as documented against the current
tree.

**Real bug found and fixed (test-only, but a genuine crash):**
`test_live_view.py`'s `test_deskew_bgr_with_tag_frame_behavior_unchanged`
constructs a real `QPixmap` (via `_bgr_ndarray_to_pixmap`) but, in the
pre-rebuild file, did not request the `qapp` fixture. Running the file
standalone — the exact invocation the file's own module docstring
documents (`pytest tests/testgui/test_live_view.py`) — crashed with a hard
`Fatal Python error: Aborted` inside `QImage`/`QPixmap` construction: Qt
requires a `QApplication` to exist first, and nothing earlier in the file
created one. This was latent/hidden in a full-suite run only because an
alphabetically-earlier `tests/testgui/*.py` file happens to construct a
`QApplication` first (session-persisting `QApplication.instance()`).
Fixed by adding `qapp` to that one test's fixture list, matching every
other Qt-touching test in the file. Verified: the file now passes both
standalone and as part of the full suite.

**Coverage gap closed (085-007's own acceptance criteria, not covered by
the pre-rebuild suite):**
- `on_truth_ready`'s full gate (skip `refresh()` entirely when live view is
  active) had no dedicated test in `tests_old/testgui/test_telemetry_gating.py`
  — only `on_frame_ready`'s gate was covered. Added
  `TestOnTruthReadyLiveViewGating` (4 new tests) mirroring the existing
  `on_frame_ready` reimplementation pattern.
- "the live-view worker is confirmed to start only for Relay connections,
  never Sim/Serial" had no direct test either (only the state-lifecycle
  tests, which always assumed Relay). Added `TestLiveViewRelayOnlyGate` (3
  new tests) to `test_live_frame_bridge.py`.
- Confirmed structurally (documented in the new tests' module context, not
  a new test itself): `transport_combo` is disabled while connected
  (`__main__.py` line ~2221), so "leaving Relay mode" while still connected
  is not a reachable state in the current UI — the only way to stop live
  view is Disconnect, which is the same `_stop_live_worker()` path already
  covered by `TestLiveBridgeStateLifecycle`.

Full `tests/testgui` suite: 261 passed (up from 205 pre-ticket).

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression).
- **New tests to write**: port the three files above.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
