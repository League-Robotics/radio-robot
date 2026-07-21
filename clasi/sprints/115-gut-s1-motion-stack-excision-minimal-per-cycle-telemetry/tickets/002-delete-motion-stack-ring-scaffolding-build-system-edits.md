---
id: "002"
title: "Delete motion stack + ring scaffolding; build-system edits"
status: open
use-cases: [SUC-045]
depends-on: []
github-issue: ""
issue: ""
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete motion stack + ring scaffolding; build-system edits

## Description

First ticket of the S1 excision. Deletes the four dead-weight firmware
subsystems and the CMake wiring that builds them, plus every test
harness whose sole reason to exist was one of those subsystems. This
ticket is expected to leave the tree **not compiling** — call sites in
`main.cpp`/`robot_loop.cpp`/`drive.cpp` still reference the deleted
types until ticket 005. This is intentional (sprint.md Architecture
Decision 1: "one coherent unit; no intermediate state compiles") — do
not attempt to keep the build green at this ticket's boundary.

Implements the gut issue's own S1 deletion + build-edit list (spec:
`clasi/issues/gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md`,
not directly linked to this sprint — see sprint.md's Tickets-section
flag; this ticket's own text is its spec of record). All line numbers
below were verified against the working tree during planning
(2026-07-21); the tag `pre-gut-motion-stack` already exists and is
identical to current HEAD, so everything deleted here is fully
recoverable.

## Acceptance Criteria

- [ ] `src/firm/motion/` (executor.{h,cpp}, jerk_trajectory.{h,cpp}),
      `src/firm/app/pilot.{h,cpp}`, `src/firm/app/heading_source.{h,cpp}`,
      `vendor/ruckig/`, `src/firm/devices/measurement_ring.h`,
      `src/firm/devices/interpolation.h` are deleted.
- [ ] Root `CMakeLists.txt`: the `vendor/ruckig` `include_directories`
      call (verified at :245) and the `RUCKIG_SOURCES` glob+append
      (verified at :303-304) are removed.
- [ ] `src/sim/CMakeLists.txt`: `RUCKIG_DIR` (verified :26),
      `heading_source.cpp`/`pilot.cpp` out of `APP_SOURCES` (verified
      :90/:92), `MOTION_SOURCES` (verified :124-127), `RUCKIG_SOURCES`
      (verified spans :131-143 — one line past the gut issue's own cited
      ":131-141", which covered only the 11 file entries, not the
      `set(...)` open/close lines) are all removed, along with
      `RUCKIG_SOURCES`'s/`MOTION_SOURCES`'s references inside
      `add_library(firmware_host SHARED ...)`.
- [ ] Test harnesses deleted (verified present today): 
      `src/tests/sim/unit/jerk_trajectory_harness.cpp` +
      `test_jerk_trajectory.py`;
      `src/tests/sim/unit/motion_executor_harness.cpp` +
      `test_motion_executor.py`;
      `src/tests/sim/unit/measurement_ring_harness.cpp` +
      `test_measurement_ring.py`;
      `src/tests/sim/system/move_queue_harness.cpp` +
      `test_move_queue.py`;
      `src/tests/sim/system/profiled_motion_harness.cpp` +
      `test_profiled_motion_sim.py`;
      `src/tests/sim/system/heading_source_harness.cpp` +
      `test_heading_source.py`;
      `src/tests/sim/system/pilot_distance_trim_harness.cpp` +
      `test_pilot_distance_trim.py`;
      `src/tests/sim/system/deadband_terminal_correction_harness.cpp` +
      `test_deadband_terminal_correction.py`;
      `src/tests/sim/system/behavior_lock_harness.cpp` +
      `test_behavior_lock.py`;
      `src/tests/sim/system/boundary_velocity_harness.cpp` +
      `test_boundary_velocity.py`;
      `src/tests/sim/parked-094/` (whole directory).
- [ ] `src/tests/bench/` scripts confirmed executor/Ruckig/segment/tour-only
      and deleted: at minimum `bench_ruckig_motion_verify.py`,
      `motion_command_verify.py`, `profiled_motion_verify.py`,
      `random_segment_demo.py`, `solve_time_characterize.py`,
      `solve_time_gdb_batch.gdb`, `solve_time_timing_harness.cpp`,
      `tour_bench_run.py`, `turn_sweep.py` — **grep each remaining
      `src/tests/bench/*.py` for `Motion::`/`Executor`/`Ruckig`/
      `segment`/`tour` imports or API calls before deleting or keeping
      it**; do not delete a script solely by filename resemblance, and
      do not keep one with an undetected dependency (`twist_drive.py`,
      `rig_soak.py`, `pid_hold_speed.py` are confirmed survivors per the
      gut issue's own Verification section — leave those alone).
- [ ] No remaining `#include` of any deleted header anywhere in `src/`
      (a broken build is expected at THIS ticket's boundary from
      call-site references in `main.cpp`/`robot_loop.cpp`/`sim_harness.h`
      — those are ticket 005/006's job — but a stray `#include` of a
      now-nonexistent path is this ticket's own defect, not theirs).

## Implementation Plan

**Approach**: Delete first, in the order listed (deepest/most-isolated
first: `vendor/ruckig/` and the motion/ files have the fewest remaining
consumers), then fix the two CMake files, then sweep test harnesses.
Leave every firmware `.cpp`/`.h` call site (`main.cpp`, `robot_loop.*`,
`drive.*`) untouched — those break loudly at compile time and are
ticket 005's job; do not pre-emptively patch them here (keeps this
ticket's diff scoped to deletion + build wiring only).

**Files to delete**: see Acceptance Criteria above (exhaustive list of
verified files; the bench-script list is a floor, not a ceiling — grep
first).

**Files to modify**: `CMakeLists.txt` (root), `src/sim/CMakeLists.txt`.

**Testing plan**: This ticket does not produce a green build (expected —
see Description). Verify instead that: (a) every deleted file is
actually gone (`git status` shows deletions, no stray leftovers), (b)
the two CMake files no longer reference any deleted path (`grep -rn
ruckig CMakeLists.txt src/sim/CMakeLists.txt` returns nothing), (c) `grep
-rln "pilot\.h\|heading_source\.h\|motion/executor\.h\|motion/jerk_trajectory\.h\|measurement_ring\.h\|interpolation\.h" src/` 
returns only files ticket 005/006 will fix (i.e., confirms the deletion
didn't miss a reference this ticket itself should have caught, like a
leftover test harness).

**Documentation updates**: none — `src/firm/motion/DESIGN.md` and
`src/firm/app/DESIGN.md` (if they describe Pilot/Executor/HeadingSource)
are addressed by ticket 009's doc-update pass alongside the rest of the
S1 doc sweep, not here.
