---
id: '011'
title: Relocate turn_shape.py out of testgui; collapse to one capture path
status: open
use-cases: [SUC-008]
depends-on: ['004']
github-issue: ''
issue: relocate-turn-shape-and-collapse-to-one-capture-path.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Relocate turn_shape.py out of testgui; collapse to one capture path

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Sequencing note**: depends on ticket 004 (binary_bridge cleanup)
landing first, since `robot_radio/DESIGN.md`'s `testgui/` row is touched
by both tickets — sequence to avoid two tickets editing the same row
concurrently.

## Description

`testgui/turn_shape.py` (364 lines) is a diagnostic tool living in the
shipped GUI package, with three capture entry points at three fidelity
levels: `capture_turn()` (deterministic stepping, ideal-chip defaults),
`capture_turn_live()` ("GUI-FAITHFUL," real tick thread), and
`capture_turn_gui()` ("MOST GUI-FAITHFUL" — through `SimTransport` + real
calibration push + OTOS enabled, the only one that flips the firmware's
heading-source policy the way the GUI actually does). Three answers to
"what does a turn look like in sim" guarantees two of them mislead.

## Acceptance Criteria

- [ ] The module is moved to `src/tests/sim/turn_shape.py`.
- [ ] `capture_turn_gui()` is kept as the only source of truth.
- [ ] `capture_turn_live()` is deleted.
- [ ] `capture_turn()` is kept ONLY if a fast ideal-chip smoke variant is
      genuinely still used elsewhere — if kept, its docstring is
      rewritten to explicitly demote it ("QUICK ideal-chip smoke check
      ONLY -- not a source of truth... use `capture_turn_gui()`" per the
      issue's own example text). If not genuinely used, delete it too.
- [ ] The `-m robot_radio.testgui.turn_shape` invocation in any docs/callers
      is updated to the new `src/tests/sim/` location.
- [ ] `src/host/robot_radio/testgui/turn_shape.py` is gone;
      `grep -rn "turn_shape" src/host/` returns nothing.
- [ ] At most two capture functions remain, one explicitly demoted if
      both are kept.
- [ ] The relocated tool still runs (`uv run python -m` the new path);
      its output for a standard 90° turn matches pre-move values.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite —
  confirm no test imports `testgui.turn_shape` by its old path).
- **New tests to write**: none required for a relocation + capture-path
  collapse; if the deleted `capture_turn_live()` had any dedicated test,
  remove it.
- **Verification command**: run the relocated
  `src/tests/sim/turn_shape.py`'s `capture_turn_gui()` for a standard 90°
  turn and confirm the output matches the pre-move captured values (spot
  check, not a new automated regression test unless one already existed).

## Implementation Notes

- **Approach**: move the file first (preserving git history via `git mv`
  where practical), then delete/demote the capture functions in the new
  location.
- **Files to modify/move**:
  `src/host/robot_radio/testgui/turn_shape.py` → `src/tests/sim/turn_shape.py`.
- **Documentation updates**: `src/host/robot_radio/DESIGN.md`'s
  `testgui/` row (remove `turn_shape.py` from the dormant-list-by-name);
  `src/tests/DESIGN.md` if it enumerates `src/tests/sim/` contents.
