---
id: '010'
title: Relocate testkit/ and src/motion/planner bench artifacts out of product trees
status: done
use-cases:
- SUC-008
depends-on:
- '003'
- '004'
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

- [x] `read_camera_pose` is moved into `testgui/transport.py` (its one
      real caller) or a small new `testgui/camera_pose.py`.
- [x] The rest of `testkit/` is deleted (delete-by-default, per the
      decision above) — or moved to `src/tests/tools/` if a stakeholder
      objection to deletion surfaces before this ticket executes; record
      whichever outcome actually happened, with the reason, in this
      ticket's own notes.
- [x] `src/motion/planner/bench/` and `py/planner_harness.py` move to
      `src/tests/bench/` (joining `move_protocol_bench.py` etc.), with
      imports/paths fixed for the new location.
- [x] The committed CSV/PNG captures are moved to a bench-results
      location outside the source tree, or deleted with the generating
      script + date noted in the new bench README (regenerable output,
      not source).
- [x] `src/host/robot_radio/testkit/` no longer exists (or contains only
      what was explicitly kept, with a stated reason recorded in this
      ticket).
- [x] `git ls-files src/motion/planner | grep -E "\.py$|\.csv$|\.png$"`
      returns nothing.
- [x] `testgui` still imports its camera-pose helper from its new
      location; relocated bench scripts run from their new home
      (spot-check one hardware-shaped script's `--help`/dry-run and one
      sim script's actual run).

## Execution Notes (2026-07-31)

- No stakeholder objection surfaced before execution — proceeded with
  delete-by-default for `testkit/` per the plan of record.
  `read_camera_pose` moved (not copied) into
  `src/host/robot_radio/testgui/transport.py` as a module-level function,
  placed alongside its other free-function helpers
  (`find_relay_port`/`_relay_probe_banner`); `_truth_loop` now calls it
  directly (no more `from robot_radio.testkit.camera import
  read_camera_pose`). `camera.py`/`dash.py`/`pose.py`/`safety.py`/
  `target.py`/`__init__.py` deleted — zero remaining live callers
  (confirmed by repo-wide grep; the only other hits were archived
  `src/archive/tests_old/` code and two doc-comment mentions in
  `field/geofence.py`/`src/tests/unit/test_geofence_capture_fix_yaw_wrap.py`,
  both repointed to `testgui/transport.read_camera_pose`).
- `src/motion/planner/bench/*` (6 scripts + README) and
  `src/motion/planner/py/planner_harness.py` moved via `git mv` into
  `src/tests/bench/`, joining the existing bench-tool catalog. Import
  paths fixed: scripts that reached across a sibling `py/` directory via
  `sys.path.insert(..., parent.parent / "py")` now just insert their own
  (now shared) directory, since `planner_harness.py` is co-located;
  `planner_harness.py`'s own `loadLibrary()` now walks up to the repo
  root (`parents[3]`) and back down to the unmoved
  `src/motion/planner/build` CMake output dir, since it moved four
  levels below repo root instead of two. `encoder_refresh.py`'s
  `_REPO_ROOT` depth constant fixed the same way (`parents[4]` ->
  `parents[3]`).
- **Naming collision found and resolved**: the destination
  `src/tests/bench/` already had an unrelated `square_tour_sim.png`
  (127-007's `square_tour.py` goto-mode sim capture). The moved
  `square_tour_sim.py`'s own capture is renamed
  `motionlib_square_tour_sim.png` (both the moved file and the
  generating script's own output-path constant) to avoid clobbering it
  — documented in the relocated bench README
  (`src/tests/bench/motion_planner_bench.md`, itself renamed from the
  generic `README.md` since `src/tests/bench/` already has file-scoped
  docs, not one repo-wide README). No other filename collisions found.
  All CSV/PNG captures were MOVED (not deleted) — this directory already
  commits regenerable CSV/PNG bench captures as a matter of course (e.g.
  `speed_map.csv`, `square_tour_bench.png`), so relocating rather than
  deleting matches existing convention.
- Verification performed (no hardware): `py_compile` on every
  touched/moved file; `robot_radio.testgui.transport.read_camera_pose`
  imports and is callable; `robot_radio.testkit` confirmed
  `ModuleNotFoundError`; `square_tour_sim.py` (sim, no hardware) run to
  completion end-to-end after building the standalone
  `motionplanner` CMake target
  (`cmake -S src/motion/planner -B src/motion/planner/build && cmake
  --build src/motion/planner/build --target motionplanner`) — wrote
  `motionlib_square_tour_sim.png` without touching the pre-existing
  `square_tour_sim.png`; `square_tour_velocity.py` also run to
  completion (sim) as a second spot-check; `hil_drive.py`,
  `hil_square_tour.py`, `plant_id.py`, `encoder_refresh.py` (all
  hardware-shaped) spot-checked via `--help`. `uv run python -m pytest
  src/tests/testgui/test_smoke.py -q` (13 passed) and
  `src/tests/unit/test_geofence_capture_fix_yaw_wrap.py` (4 passed).
- Docs updated: `src/host/robot_radio/DESIGN.md` (`testkit/` row ->
  DELETED, `Consumes` AprilCam bullet), `src/host/DESIGN.md` (same
  AprilCam bullet), `docs/design/square-tour-chart-spec.md` and
  `docs/design/2026-07-28-motion-profile-exploration.md` (script path),
  `src/motion/planner/CMakeLists.txt` (comment pointing at the ctypes
  harness's new location), `src/host/robot_radio/robot/protocol.py` and
  `src/host/robot_radio/pathplan/world_pose.py` (doc-comment path
  references to `hil_drive.py`). `src/motion/DESIGN.md` and
  `src/tests/DESIGN.md` were checked and do not enumerate specific
  bench-tool filenames, so neither needed a change.

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
