---
id: '001'
title: Vendor Ruckig restore + Motion::JerkTrajectory port + build/solve-time gates
status: in-progress
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: firmware-jerk-limited-motion-ruckig-return-arc-command-queue.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Vendor Ruckig restore + Motion::JerkTrajectory port + build/solve-time gates

## Description

This is the foundation ticket for sprint 109: it restores the jerk-limited
trajectory solver the pre-rebuild firmware had (deleted in sprints 102-107)
without changing any robot behavior yet. Nothing in this ticket wires the
solver into the running loop — that's ticket 003. This ticket only proves
the solver builds, runs, and fits the ARM budget.

1. Restore vendored Ruckig from history: `git show c63ec6c:libraries/ruckig`
   (or `git archive c63ec6c libraries/ruckig | tar -x`) into
   `src/vendor/ruckig/`, matching this project's current `src/vendor/`
   layout conventions (see `src/vendor/CLAUDE.md` if one exists for house
   rules on vendored code).
2. Port `source/motion/jerk_trajectory.{h,cpp}` from the same commit into
   `src/firm/motion/jerk_trajectory.{h,cpp}`, updating it to this repo's
   current naming/style conventions (CamelCase per
   `.claude/rules/naming-and-style.md` — the old file predates the
   lowerCamelCase-functions rule; bring it into conformance since it's
   being touched) and to compile under both ARM and `-DHOST_BUILD` per
   `src/firm/DESIGN.md` §3's HOST_BUILD purity invariant.
3. Add the new `solveToState(pos, vel, vmax)` entry point (nonzero target
   velocity) — verify support in `input_parameter.hpp` at c63ec6c per the
   issue's own note ("verified supported"). Keep the existing seeding
   contract, the `jerk == 0` trapezoid sentinel, and the retarget/reanchor
   entry points from the ported code intact; this ticket only adds the one
   new entry point, it does not redesign the wrapper.
4. Wire the build: root `CMakeLists.txt` re-adds the ruckig include path +
   source glob (near the old ~line 220/270 locations per the issue;
   `gnu++20` is already forced project-wide) for the ARM target;
   `src/sim/CMakeLists.txt` adds an explicit motion+ruckig source list for
   the host/sim build (explicit list, not a glob, matching the issue's
   note on sim's build style).
5. Add `src/firm/motion/DESIGN.md` (new subsystem doc, using the format of
   existing sibling docs like `src/firm/app/DESIGN.md` /
   `src/firm/devices/DESIGN.md` as the template — frontmatter `root:
   ../DESIGN.md`, sections 1-6) describing `Motion::JerkTrajectory`'s
   purpose, boundary, and the seeding/retarget/reanchor contract. Add this
   new subsystem as a row in root `src/firm/DESIGN.md`'s directory map
   table and dependency diagram (§2), matching the dependency graph in
   this sprint's `sprint.md` Architecture section (motion depends only on
   messages; nothing depends on motion yet in this ticket since Executor/
   Pilot don't exist until ticket 003).
6. Unit tests for `JerkTrajectory` alone: port/adapt the seeding-contract
   regression test from c63ec6c; add a `solveToState` test (nonzero target
   velocity solve reaches the requested state); assert the jerk==0
   trapezoid sentinel still degrades correctly.
7. On-target gates (bench, per `.claude/rules/hardware-bench-testing.md`):
   build+flash via `just build-clean` then `mbdeploy deploy --hex <path>`
   (note: `mbdeploy deploy --build` is broken per `.clasi/knowledge` — use
   the two-step form), confirm the robot still boots and drives exactly as
   before (no behavior change expected — this ticket is solver-only,
   nothing calls it from the loop yet), then run a `solve_time_
   characterize.py`-style script (new, if it doesn't exist) measuring p99
   solve time on real hardware, and `arm-none-eabi-size build/MICROBIT`
   for a flash-budget baseline.

## Acceptance Criteria

- [ ] `src/vendor/ruckig/` restored from `c63ec6c` and builds under both
      the ARM CMake target and `src/sim/CMakeLists.txt`'s host build.
- [ ] `src/firm/motion/jerk_trajectory.{h,cpp}` compiles under
      `-DHOST_BUILD` with no `MicroBit.h` anywhere in the translation
      unit (per `src/firm/DESIGN.md` §3).
- [ ] `solveToState(pos, vel, vmax)` is implemented and unit-tested
      (nonzero target-velocity solve reaches the requested end state).
- [ ] Ported seeding-contract regression test (from c63ec6c) passes.
- [ ] `jerk == 0` trapezoid sentinel behavior is preserved and tested.
- [ ] `src/firm/motion/DESIGN.md` exists (new subsystem doc, template
      matching `src/firm/app/DESIGN.md` / `devices/DESIGN.md`); root
      `src/firm/DESIGN.md` §2's directory map and dependency diagram
      updated to include `motion/`.
- [ ] Bench: firmware builds (`just build-clean`), flashes
      (`mbdeploy deploy --hex <path>`), and the robot behaves identically
      to pre-ticket (sensors alive, wheels drive, encoders increment per
      `.claude/rules/hardware-bench-testing.md`) — this ticket adds a
      dormant solver, no behavior change is expected yet.
- [ ] `solve_time_characterize.py` (new or adapted) reports p99 solve time
      on real hardware; result recorded in the ticket or a linked bench
      note for ticket 003's cycle-budget check to reference.
- [ ] `arm-none-eabi-size build/MICROBIT` flash-budget baseline recorded
      (pre- vs. post-Ruckig-restore delta) for later tickets to track
      against.

## Testing

- **Existing tests to run**: full existing `src/tests/` suite (host build)
  to confirm the vendor restore doesn't break any existing target;
  `uv run python -m pytest` for any host-side tests touched incidentally
  by the CMake change.
- **New tests to write**: `JerkTrajectory` unit tests (seeding-contract
  regression, `solveToState` nonzero-velocity solve, jerk==0 sentinel);
  a CMake smoke build for both ARM and host/sim targets in CI if such a
  smoke target exists in this repo (check `justfile` for an existing
  `build-all`/`ci` recipe before adding a new one).
- **Verification command**: `uv run python -m pytest src/tests/` (host
  build tests) plus `just build-clean` (ARM) and the sim build target for
  a full compile check on both.

## Implementation Plan

**Approach**: Restore-and-port only, no new design. Follow the ported
code's existing structure; the only net-new code is the `solveToState`
entry point. Do this ticket first and in isolation (no other ticket
depends on anything except this one existing) so a solver-only regression
is trivially bisectable.

**Files to create**:
- `src/vendor/ruckig/**` (restored from `git show c63ec6c`)
- `src/firm/motion/jerk_trajectory.h`
- `src/firm/motion/jerk_trajectory.cpp`
- `src/firm/motion/DESIGN.md`
- Unit test file(s) alongside (e.g. `src/tests/firm/motion/
  jerk_trajectory_test.cpp` — match this repo's existing test layout
  convention, check `src/tests/firm/` for siblings first)

**Files to modify**:
- Root `CMakeLists.txt` (ruckig include + source glob for ARM)
- `src/sim/CMakeLists.txt` (explicit motion+ruckig source list)
- `src/firm/DESIGN.md` (§2 directory map row + dependency diagram)

**Testing plan**: unit tests as above; bench gate per
`.claude/rules/hardware-bench-testing.md` (build, flash, confirm no
behavior change, p99 solve-time + flash-size gates).

**Documentation updates**: new `src/firm/motion/DESIGN.md`; root
`src/firm/DESIGN.md` §2 map/diagram update (this ticket only adds the
`motion` node with no incoming edges yet — ticket 003 adds the `app ->
motion` edge when `Pilot`/`Executor` start calling it).
