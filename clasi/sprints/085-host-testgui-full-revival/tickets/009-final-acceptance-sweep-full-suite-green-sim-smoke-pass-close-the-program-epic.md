---
id: "009"
title: "Final acceptance sweep: full suite green, sim smoke pass, close the program epic"
status: open
use-cases: [SUC-011]
depends-on: ["001", "002", "003", "004", "005", "006", "007", "008"]
github-issue: ""
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

- [ ] `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui -q` is fully
      green — the pre-existing ~136 tests plus this sprint's ~16 ported
      files and 1 new file (`test_goto.py`).
- [ ] No `tests_old/testgui/*.py` file remains un-ported without an
      explicit, recorded reason (cross-check the file list against
      tickets 002/004/005/006/007/008's scope).
- [ ] A scripted sim smoke pass succeeds, covering in one session:
  - [ ] Launch TestGUI, connect via Sim.
  - [ ] Run a tour (Tour 1 or Tour 2) to completion; the robot returns
        near world origin.
  - [ ] Run camera GOTO (using synthetic truth, since Sim has no camera)
        driving to a world point within `eps`.
  - [ ] Sync-Pose/Set-Origin anchor the pose as documented.
  - [ ] Calibration push is observed at connect (`GET rotSlip` or
        equivalent readback matches the active robot's config).
- [ ] The parent issue `host-testgui-full-revival.md` and the program-epic
      issue `plan-revive-testgui-against-the-new-tree-simulator.md` are
      both confirmed closeable (their acceptance sketches are satisfied).
- [ ] No hardware bench gate is required (host-only sprint, per
      `sprint.md`'s Test Strategy) — this ticket's acceptance is entirely
      sim/headless.

## Testing

- **Existing tests to run**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q` (the full suite, as the primary acceptance gate).
- **New tests to write**: none expected beyond a smoke-pass script (e.g.
  under `tests/bench/` or as a standalone scripted sequence using
  `SimTransport` directly) — this ticket integrates and verifies, it does
  not add new unit coverage.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
