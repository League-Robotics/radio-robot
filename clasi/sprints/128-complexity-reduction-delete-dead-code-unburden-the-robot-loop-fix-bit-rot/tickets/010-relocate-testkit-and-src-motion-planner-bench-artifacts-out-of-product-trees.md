---
id: '010'
title: Relocate testkit/ and src/motion/planner bench artifacts out of product trees
status: open
use-cases: [SUC-008]
depends-on: ['003', '004']
github-issue: ''
issue: relocate-testkit-and-motion-bench-artifacts-out-of-product-trees.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Relocate testkit/ and src/motion/planner bench artifacts out of product trees

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Sequencing note**: depends on tickets 003/004 (testgui `Transport.halt()`
+ `binary_bridge` cleanup) landing first, since `read_camera_pose`'s new
home is `testgui/transport.py`, which those tickets already touch — do
this after, not concurrently, to avoid two tickets editing the same file
in divergent ways.

**Decision (plan of record, per the issue's own stated bar)**:
delete-by-default for anything in `testkit/` with zero live callers.
`read_camera_pose` (the one real caller) moves with its caller. **One
open confirmation before executing the delete**: the issue's own text
says "confirm with the stakeholder before keeping" — inverted here to
"confirm before discarding," since deleting bench harness code
(`SafeRun`/`BenchRun`/`Dashboard`) is more destructive than the sprint's
typical dead-product-code deletions. If no objection surfaces before this
ticket executes, proceed with delete-by-default as planned.

## Description

`src/host/robot_radio/testkit/` (`make_target`/`TestRobot`, `SafeRun`/
`BenchRun`, `Dashboard`, `PoseSource`) is bench harness code inside the
importable host package, with exactly one non-archived consumer:
`testgui/transport.py:1285` imports `read_camera_pose` from
`testkit/camera.py`. `src/motion/planner/bench/` (6 scripts, 2,229 lines)
and `src/motion/planner/py/planner_harness.py` sit inside the C++ library
tree, alongside ~9 MB of committed measurement output
(`square_tour_velocity_*.csv` ×4, four PNGs).

## Acceptance Criteria

- [ ] `read_camera_pose` is moved into `testgui/transport.py` (its one
      real caller) or a small new `testgui/camera_pose.py`.
- [ ] The rest of `testkit/` is deleted (delete-by-default, per the
      decision above) — or moved to `src/tests/tools/` if a stakeholder
      objection to deletion surfaces before this ticket executes; record
      whichever outcome actually happened, with the reason, in this
      ticket's own notes.
- [ ] `src/motion/planner/bench/` and `py/planner_harness.py` move to
      `src/tests/bench/` (joining `move_protocol_bench.py` etc.), with
      imports/paths fixed for the new location.
- [ ] The committed CSV/PNG captures are moved to a bench-results
      location outside the source tree, or deleted with the generating
      script + date noted in the new bench README (regenerable output,
      not source).
- [ ] `src/host/robot_radio/testkit/` no longer exists (or contains only
      what was explicitly kept, with a stated reason recorded in this
      ticket).
- [ ] `git ls-files src/motion/planner | grep -E "\.py$|\.csv$|\.png$"`
      returns nothing.
- [ ] `testgui` still imports its camera-pose helper from its new
      location; relocated bench scripts run from their new home
      (spot-check one hardware-shaped script's `--help`/dry-run and one
      sim script's actual run).

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite —
  confirm nothing outside `testkit`/`src/motion/planner/bench` imports
  the relocated modules by their old path).
- **New tests to write**: none required for a relocation; if any test
  imports from the old `testkit`/`src/motion/planner/bench` paths,
  update the import.
- **Verification command**: `uv run python -m pytest -q`, plus running
  one relocated bench script from its new `src/tests/bench/` home to
  confirm it still executes (sim script; hardware script may be dry-run
  only absent bench access during this ticket's own execution — full
  hardware exercise happens at the sprint's own bench-verification gate).

## Implementation Notes

- **Approach**: move `read_camera_pose` first (small, unblocks
  `testkit/`'s deletion), then delete/relocate the rest of `testkit/`,
  then relocate `src/motion/planner/bench/`+`py/`, then handle the
  committed captures last (largest diff, least risky to get wrong once
  the code around it has already moved).
- **Files to modify/move**: `src/host/robot_radio/testkit/*` (delete or
  relocate), `src/host/robot_radio/testgui/transport.py` (gains
  `read_camera_pose`), `src/motion/planner/bench/*`,
  `src/motion/planner/py/planner_harness.py` → `src/tests/bench/`.
- **Documentation updates**: `src/host/robot_radio/DESIGN.md`'s
  `testkit/` row (removed or updated), `src/motion/DESIGN.md`'s note on
  `planner/`'s bench scripts (now relocated), `src/tests/DESIGN.md` if
  it enumerates bench-tool contents.
