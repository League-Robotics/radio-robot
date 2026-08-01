---
id: '005'
title: 'testgui keyboard drive: delete (dead in Sim and hardware)'
status: done
use-cases:
- SUC-006
depends-on:
- '004'
github-issue: ''
issue: testgui-keyboard-drive-is-dead-port-to-move-twist-or-delete.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# testgui keyboard drive: delete (dead in Sim and hardware)

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Decision (sprint-planner, not stakeholder-flagged)**: Option B —
**delete**, not port. Gamepad and preset-button teleop already cover the
live driving workflow; arrow-key drive has been fully dead in both Sim
and hardware since the MOVE-protocol cutover with no reported demand for
its return. If Eric wants it kept as a live workflow instead, this is the
cheapest ticket in the sprint to flip to Option A (port to a bounded,
`replace=True` Move-based keepalive) — but plan and implement as delete
unless redirected.

## Description

`testgui/drive.py` (`KeyboardDriver`) drives the `DEV DT VW`/`DEV DT STOP`
wire family. On hardware that falls into the (now-deleted, ticket 004)
dead `binary_bridge.translate_command()` stub; in Sim,
`SimTransport._dispatch()` has no `DEV` branch and logs "not supported in
this sim." Pressing an arrow key moves nothing, in either mode, and the
100 ms keepalive + five-resend STOP deadman all run to completion around
commands that go nowhere — ~500 lines of convincing-looking dead
machinery.

## Acceptance Criteria

- [x] `testgui/drive.py` is deleted.
- [x] Its `attach()` call site and the key-event plumbing in
      `__main__.py` are deleted.
- [x] `grep -rn "DEV DT" src/host/robot_radio/` returns nothing.
- [x] The GUI builds and the existing headless button-acceptance suite
      passes with no arrow-key-drive references remaining.
- [x] Gamepad/preset-button teleop paths are confirmed unaffected (not
      touched by this ticket).

## Testing

- **Existing tests to run**: `uv run python -m pytest`;
  `test_gui_button_acceptance.py`.
- **New tests to write**: none required for a pure deletion; if any
  existing test references `testgui.drive`/`KeyboardDriver`, update or
  remove it as part of this ticket.
- **Verification command**: `uv run python -m pytest src/tests -k gui_button -q`.

## Implementation Notes

- **Approach**: delete the file and its two call sites; grep the tree
  for any stray import before declaring done.
- **Files to modify**: delete
  `src/host/robot_radio/testgui/drive.py`; edit
  `src/host/robot_radio/testgui/__main__.py` (remove `attach()` call and
  key-event plumbing).
- **Documentation updates**: `robot_radio/DESIGN.md`'s `testgui/` row
  already lists `drive.py` as live — update to remove it from the live
  list (coordinate with ticket 004's DESIGN.md edit to the same row,
  don't duplicate/conflict).
