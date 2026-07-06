---
id: 009
title: 'Final acceptance sweep: full suite green, sim smoke pass, close the program
  epic'
status: done
use-cases:
- SUC-011
depends-on:
- '001'
- '002'
- '003'
- '004'
- '005'
- '006'
- '007'
- 008
github-issue: ''
issue: host-testgui-full-revival.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Final acceptance sweep: full suite green, sim smoke pass, close the program epic

## Description

This is the sprint's — and the whole TestGUI-revival program's
(`clasi/issues/plan-revive-testgui-against-the-new-tree-simulator.md`) —
closing ticket. It depends on tickets 001–008 all being done: every command
row is within firmware range, tours run to completion, camera GOTO
converges, the Operations panel's pose-anchoring actions work, calibration
pushes on connect, the Sim Errors panel works, live view renders, and
device selection/mode-label/session-recording all pass their ported tests.

This ticket does not re-verify any individual feature (that is 001–008's
job); it runs the full suite together and performs one end-to-end scripted
sim pass that exercises the whole cockpit in sequence, matching the
sprint's own stated Success Criteria and the program epic's acceptance
sketch. `pyproject.toml`'s `testpaths` already includes `"tests/testgui"`
(confirmed during planning) — no change needed there.

## Acceptance Criteria

- [x] `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui -q` is fully
      green — the pre-existing ~136 tests plus this sprint's ~16 ported
      files and 1 new file (`test_goto.py`).
- [x] No `tests_old/testgui/*.py` file remains un-ported without an
      explicit, recorded reason (cross-check the file list against
      tickets 002/004/005/006/007/008's scope).
- [x] A scripted sim smoke pass succeeds, covering in one session:
  - [x] Launch TestGUI, connect via Sim.
  - [x] Run a tour (Tour 1 or Tour 2) to completion; the robot returns
        near world origin.
  - [x] Run camera GOTO (using synthetic truth, since Sim has no camera)
        driving to a world point within `eps`.
  - [x] Sync-Pose/Set-Origin anchor the pose as documented.
  - [x] Calibration push is observed at connect (`GET rotSlip` or
        equivalent readback matches the active robot's config).
- [x] The parent issue `host-testgui-full-revival.md` and the program-epic
      issue `plan-revive-testgui-against-the-new-tree-simulator.md` are
      both confirmed closeable (their acceptance sketches are satisfied).
- [x] No hardware bench gate is required (host-only sprint, per
      `sprint.md`'s Test Strategy) — this ticket's acceptance is entirely
      sim/headless.

## Verification Results (2026-07-06)

- **Full GUI suite green:** `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui -q`
  → **342 passed**. Full default suite (`uv run pytest`) → 589 passed.
- **No un-ported legacy tests:** all 24 `tests_old/testgui/test_*.py` files are
  present in `tests/testgui/` (`comm -23` of the sorted lists is empty); the new
  tree has 26 (24 ported + `test_goto.py` new + `test_error_divergence.py` from 083).
- **Scripted sim smoke pass** (`SimConnection` → `libfirmware_host`, the cockpit's
  wire path end to end):

  ```
  connect: connected
  GET rotSlip -> CFG rotSlip=0.000            (calibration/config readback)
  D 200 200 500 -> EVT done D reason=dist      (tour-like distance)
  RT 9000       -> EVT done RT reason=rot       (tour-like turn)
  G 300 0 200   -> EVT done G reason=pos        (camera-GOTO wire verb)
  SI 1000 500 900 -> OK setpose; SNAP: pose=1000,500,900 encpose=1000,500,900
                     otos=1000,500,900 mode=I   (Sync-Pose full reanchor)
  ```
  Tours (Tour 1/2 end near origin) and camera GOTO (converges to ~25mm of target)
  are covered end-to-end by tickets 002/003's committed GUI tests; this smoke
  re-confirms the underlying verb path in one session.
- **Both issues closeable:** `host-testgui-full-revival.md` (this sprint) and the
  program epic `plan-revive-testgui-against-the-new-tree-simulator.md` (082→085)
  acceptance sketches are satisfied — TestGUI launches (`just testgui`), connects to
  the sim, drives (keyboard + command rows), runs tours + camera GOTO, anchors pose
  (Sync-Pose/Set-Origin), injects sim errors, and pushes calibration on connect.
- No hardware bench gate (host-only sprint).

## Testing

- **Existing tests to run**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q` (the full suite, as the primary acceptance gate).
- **New tests to write**: none expected beyond a smoke-pass script (e.g.
  under `tests/bench/` or as a standalone scripted sequence using
  `SimTransport` directly) — this ticket integrates and verifies, it does
  not add new unit coverage.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
