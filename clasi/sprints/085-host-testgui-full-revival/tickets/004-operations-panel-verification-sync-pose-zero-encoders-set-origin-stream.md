---
id: "004"
title: "Operations panel verification: Sync-Pose, Zero-Encoders, Set-Origin, STREAM"
status: open
use-cases: [SUC-004, SUC-005]
depends-on: []
github-issue: ""
issue: host-testgui-full-revival.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Operations panel verification: Sync-Pose, Zero-Encoders, Set-Origin, STREAM

## Description

`OpsController` (`host/robot_radio/testgui/operations.py`) and `_set_origin`
(`host/robot_radio/testgui/__main__.py` lines ~1722–1788) already implement
every action this ticket covers:

- **Sync Pose**: reads tag-100 world pose from the aprilcam daemon, sends
  `SI x y h` (mm, mm, cdeg) via `build_setpose_command`.
- **Zero Encoders**: sends `ZERO enc`.
- **Set Robot @ 0,0**: sends `STOP` (halt + clear any in-flight goal), then
  — Sim only — teleports the plant ground truth via
  `transport.set_true_pose(0,0,0)`, then `ZERO enc`, `OZ` (re-anchor the
  OTOS heading reference), then `SI 0 0 0`, in that exact order, then
  resets the `TraceModel`/avatar display.
- **STREAM toggle**: sends `STREAM 50` (on) / `STREAM 0` (off).

This code predates the greenfield rebuild (per `architecture-update.md`
Grounding fact 1) and has not been exercised against real sprint-084
firmware/sim. This ticket ports the one un-ported test file covering this
surface and verifies each action end-to-end against the sim, fixing
anything a real run surfaces. Note per `architecture-update.md` Decision 1:
`_set_origin`'s `STOP` call is the correct top-level verb (clears the
Planner's active goal) — do not change it to `DEV DT STOP`.

## Acceptance Criteria

- [ ] `tests_old/testgui/test_set_origin.py` is ported to
      `tests/testgui/`, updated for any API drift, and passes under
      `QT_QPA_PLATFORM=offscreen`.
- [ ] Set-Origin's five-step sequence (`STOP`, sim-teleport, `ZERO enc`,
      `OZ`, `SI 0 0 0`) is confirmed to fire in that exact order against
      the sim, and the firmware's fused pose reads back at (0, 0, 0°)
      afterward.
- [ ] Sync Pose sends `SI` with the daemon-read pose converted correctly
      (mm/mm/cdeg) and is confirmed disabled (with its explanatory tooltip)
      when the active transport is `SimTransport`.
- [ ] Zero Encoders sends `ZERO enc` and the reply is logged.
- [ ] STREAM toggle sends `STREAM 50`/`STREAM 0` correctly, reverts its
      visual state on a failed send, and resets to "off" on disconnect.
- [ ] With no transport connected, Set-Origin skips the wire commands
      (logs a `[WARN]`) but the display-only reset (`TraceModel`/avatar)
      still runs.
- [ ] Any genuine bug surfaced by running this sequence against the real
      sim for the first time is fixed here and documented.

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression).
- **New tests to write**: port `test_set_origin.py`; extend with an
  end-to-end sim assertion of the pose-readback-at-origin postcondition if
  not already covered by the ported file.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
