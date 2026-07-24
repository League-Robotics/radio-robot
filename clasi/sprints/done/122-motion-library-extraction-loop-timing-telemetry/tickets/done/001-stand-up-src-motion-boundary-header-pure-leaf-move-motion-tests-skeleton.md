---
id: '001'
title: 'Stand up src/motion: boundary header + pure-leaf move + motion_tests skeleton'
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: extract-motion-library-to-src-motion.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Stand up src/motion: boundary header + pure-leaf move + motion_tests skeleton

## Description

First half of the mechanical extraction (sprint 122's architecture §Step 5).
Create `src/motion` as a new sibling tree to `src/firm`, define the one
narrow boundary header the extraction issue calls for, move the
already-pure leaves, and get a standalone `motion_tests` CMake target
compiling and passing — before touching anything that RobotLoop itself
still owns (that is ticket 002's job). This ticket also records the
pre-extraction baseline that ticket 002's refactor gate compares against.

This ticket does NOT move `move_queue`, `state_estimator`, `odometry`, or
any part of `drive.*` yet — `src/firm` keeps using them from their current
location throughout this ticket. Both `firmware` (ARM) and
`libfirmware_host` (sim) must remain green, unchanged, at the end of this
ticket; only `motion_tests` is new.

**Locked scope reminders (do not relitigate — see sprint.md Design
Rationale Decisions 1-4):**
- The boundary is a **velocity** sink (`setWheels(vLeft, vRight)`/
  `stop()`/`tick()`), never a duty sink. Do NOT move `devices/
  velocity_pid.*` — it stays in `src/firm/devices`, unchanged, this
  sprint, even though the extraction issue's own REVISION note says
  otherwise (that revision is sprint 2's job).
- `src/motion` is a **sibling** of `src/firm` (`src/motion/`, not
  `src/firm/motion/`).
- Do NOT add `src/motion` to `.clasi/config.yaml`'s `sources:` list — it
  stays `[src/firm, src/host]`.

## Acceptance Criteria

- [x] `src/motion/` exists with its own `CMakeLists.txt` and its own
      `DESIGN.md` (may be a stub this ticket; ticket 004 finalizes it).
- [x] `src/motion/stop_condition.{h,cpp}`, `velocity_shaper.{h,cpp}`, and
      `body_kinematics.{h,cpp}` are moved from `src/firm/motion/` and
      `src/firm/kinematics/` respectively (`git mv`, preserving history).
- [x] A boundary header exists in `src/motion` declaring: an abstract
      wheel-velocity command sink (`setWheels(vLeft, vRight)`/`stop()`),
      the per-wheel state struct motion will read (position, velocity,
      sample time), and plain config passed at construction (trackwidth,
      shaper limits). No concrete implementation lives in this header.
- [x] `src/motion`'s new sources include NOTHING from `src/firm` except
      `messages/` headers (verify by grep: `grep -rn '#include "' src/motion
      | grep -v 'messages/'` shows no `src/firm`-only paths).
- [x] `src/motion/CMakeLists.txt` builds a standalone `motion_tests`
      target: plain CMake, the three moved modules, no `libfirmware_host`,
      no ctypes, no Python in the build/run path.
- [x] The existing StopCondition and VelocityShaper scenario coverage
      (currently `src/tests/sim/unit/motion_stop_condition_harness.cpp` /
      `motion_velocity_shaper_harness.cpp` + their `test_motion_*.py`
      wrappers) passes against the new location — either by repointing
      the wrapper's `-I` path, or by having it invoke the new
      `motion_tests` target (implementer's choice; module boundary from
      sprint.md is normative, not file layout of the test wrapper).
  - [x] Every callsite in `src/firm` that referenced the moved modules'
      old include path (`motion/stop_condition.h`,
      `motion/velocity_shaper.h`, `kinematics/body_kinematics.h`) is
      updated to the new `src/motion` path; `src/sim/CMakeLists.txt`'s
      `MOTION_SOURCES`/`KINEMATICS_SOURCES` repoint at the new location.
- [x] Root `CMakeLists.txt`'s ARM build is extended so `src/motion`'s
      sources/headers are picked up by the firmware link (it currently
      only globs `CODAL_APP_SOURCE_DIR = "src/firm"` via
      `RECURSIVE_FIND_FILE`/`RECURSIVE_FIND_DIR` — this is the change
      called out in sprint.md's Migration Concerns as easy to miss).
- [x] `firmware` (ARM build) and `libfirmware_host` (sim) both build green
      and their existing suites pass, unchanged, at the end of this
      ticket.
- [x] **Pre-extraction baseline recorded**: capture the full sim/closure
      suite's numeric outputs (e.g. a saved log/artifact of the current
      `uv run pytest` run against `src/tests/sim/`) before this ticket's
      changes are considered final, so ticket 002 has something concrete
      to diff against. Note where the baseline is stored (a file under
      the sprint directory or a referenced CI artifact) in this ticket's
      own notes when closing it.

## Testing

- **Existing tests to run**: full `uv run pytest` (collects `src/tests/
  sim/`); the ARM `firmware` build; `libfirmware_host` (sim) build. All
  must be green and — critically — numerically identical to their
  pre-ticket state (this ticket moves 3 pure modules; if anything
  observable changes, that is a bug in the move, not an intended result).
- **New tests to write**: the `motion_tests` CMake target itself (wraps
  the existing StopCondition/VelocityShaper scenarios); no new *scenario*
  coverage is required by this ticket (the end-to-end chained-moves
  scenario is ticket 002's job, since it needs `move_queue`).
- **Verification command**: `uv run pytest` (sim suite); a fresh
  `cmake --build` + run of the new `motion_tests` target; the existing
  ARM/`libfirmware_host` build commands unchanged.

## Implementation Plan

**Approach**: `git mv` the three pure files and their DESIGN.md content
into `src/motion`, write the boundary header fresh (it has no prior
version to move), write `src/motion/CMakeLists.txt` modeled loosely on
`src/sim/CMakeLists.txt`'s explicit-source-list style (no glob-then-filter
— see that file's own header comment for why), and extend the two
existing build graphs (root `CMakeLists.txt`, `src/sim/CMakeLists.txt`) to
find the relocated sources at their new path instead of the old one.

**Files to create**:
- `src/motion/CMakeLists.txt`
- `src/motion/DESIGN.md` (stub acceptable; ticket 004 finalizes)
- `src/motion/stop_condition.{h,cpp}` (moved from `src/firm/motion/`)
- `src/motion/velocity_shaper.{h,cpp}` (moved from `src/firm/motion/`)
- `src/motion/body_kinematics.{h,cpp}` (moved from `src/firm/kinematics/`)
- The new boundary header (e.g. `src/motion/wheel_sink.h` — naming is the
  implementer's call per sprint.md Open Question 1)

**Files to modify**:
- `src/sim/CMakeLists.txt` (`MOTION_SOURCES`, `KINEMATICS_SOURCES`
  variables, `target_include_directories`)
- Root `CMakeLists.txt` (extend the ARM source/header discovery to also
  cover `src/motion`)
- `src/tests/sim/unit/test_motion_stop_condition.py`,
  `test_motion_velocity_shaper.py` (repoint `_SOURCE_DIR`/`-I` path, or
  switch to invoking the new CMake target)
- Any `#include` in `src/firm` referencing the three moved headers'
  old paths
- `src/firm/motion/DESIGN.md`, `src/firm/kinematics/DESIGN.md` — leave a
  note that their subsystems are being relocated (ticket 004 finishes the
  reconciliation; don't leave these files silently stale mid-sprint)

**Testing plan**: run the full sim suite before touching anything and
save the output as the recorded baseline; run it again after this
ticket's moves and diff — must be identical (this ticket changes file
location and build wiring only, not logic). Build `motion_tests` fresh
and confirm it runs with no Python process anywhere in its invocation.

**Documentation updates**: `src/motion/DESIGN.md` stub (real content
deferred to ticket 004, but should exist so nothing mid-sprint references
a nonexistent file); a one-line note in `src/firm/motion/DESIGN.md` and
`src/firm/kinematics/DESIGN.md` flagging the in-flight relocation.

## Implementation Notes (closing)

**Pre-extraction baseline (recorded before any file moves, on the clean
sprint branch tip, commit `0981773b`):**

- `uv run python -m pytest -q` → **1407 passed, 2 skipped, 9 xfailed,
  2 xpassed, 0 failed** (~549s). This exact run is captured verbatim
  above and is the number this ticket's own post-move rerun is diffed
  against — no separate baseline artifact file was added under the
  sprint directory; the number is recorded here, in the ticket, per the
  acceptance criterion's own "or a referenced CI artifact" option.
- `uv run python3 build.py --clean` → both `firmware` (ARM,
  `MICROBIT.hex`) and `libfirmware_host` (sim) build green,
  `v0.20260723.4`.
- Gotcha hit while recording this: the FIRST baseline attempt raced with
  my own subsequent file edits (a background pytest/build run started
  before the `git mv`+callsite edits, but kept running WHILE those edits
  landed) and produced 6 spurious compile failures. Diagnosed via
  `git stash` (stashed everything already touched, confirmed a truly
  clean tree, reran both baseline commands with no concurrent edits) —
  the numbers above are from that clean, race-free rerun. Recorded here
  so ticket 002 doesn't repeat the mistake: never launch a long
  background baseline/build job and keep editing files concurrently.

**Post-move re-run (after all moves/rewiring in this ticket, same
tree):**

- `uv run python -m pytest -q` → **1407 passed, 2 skipped, 9 xfailed,
  2 xpassed, 0 failed** (~483s) — numerically IDENTICAL to baseline.
- `uv run python3 build.py --clean` → both `firmware` (ARM) and
  `libfirmware_host` (sim) build green, same version, `src/motion`'s
  three `.cpp` files visible in both compile logs
  (`CMakeFiles/MICROBIT.dir/src/motion/*.cpp.obj` and
  `CMakeFiles/firmware_host.dir/.../src/motion/*.cpp.o`).

**`motion_tests` — how to build and run:**

```bash
cmake -S src/motion -B src/motion/build
cmake --build src/motion/build --target motion_tests   # builds AND runs (ctest)
# or, once configured/built once:
(cd src/motion/build && ctest --output-on-failure)
```

Result: `motion_stop_condition_tests` and `motion_velocity_shaper_tests`
both pass (100% tests passed out of 2). No Python process, no
`libfirmware_host`, no ctypes anywhere in that sequence — `src/motion/
CMakeLists.txt` is a standalone CMake project (its own
`cmake_minimum_required`/`project()`), separate from both the repo-root
ARM build and `src/sim/CMakeLists.txt`.

**Exactly what changed in the CMake graphs:**

- Root `CMakeLists.txt`: added a second `RECURSIVE_FIND_DIR`/
  `RECURSIVE_FIND_FILE` pass over `src/motion` (mirroring the existing
  `src/firm` one), appending into the SAME `INCLUDE_DIRS`/`SOURCE_FILES`
  lists the ARM build already consumes — plus `${PROJECT_SOURCE_DIR}/src`
  itself as an include root (see next bullet).
- `src/sim/CMakeLists.txt`: added `MOTION_DIR` (`${REPO_ROOT}/src/motion`,
  a peer of `SOURCE_DIR`); repointed `MOTION_SOURCES`/`KINEMATICS_SOURCES`
  at it; added `${REPO_ROOT}/src` to `target_include_directories`.
- **Include-path trick (both graphs, and `src/motion/CMakeLists.txt`
  itself):** adding `src/` (the parent of BOTH `src/firm` and
  `src/motion`) as an include root means a qualified `#include
  "motion/stop_condition.h"` / `#include "motion/velocity_shaper.h"`
  keeps resolving completely unchanged, because the directory is still
  literally named `motion`, just relocated one level up. Net effect:
  `src/firm/app/move_queue.h`/`move_queue.cpp` (the two `src/firm`
  callers of `stop_condition.h`/`velocity_shaper.h`) needed ZERO text
  changes. Only `#include "kinematics/body_kinematics.h"` needed a real
  rewrite (→ `#include "motion/body_kinematics.h"`), since the
  `kinematics/` directory itself is retired (folded flat into
  `src/motion`, no nested subdirectory) — updated at all 9 call sites
  (`src/firm/app/{drive,odometry,robot_loop,fake_otos}.cpp` and 5
  `src/tests/sim/{unit,plant}` files).
- 21 `src/tests/sim/**/*.py` files (every pytest-subprocess wrapper that
  compiles `body_kinematics.cpp` and/or `stop_condition.cpp`/
  `velocity_shaper.cpp` ad hoc) had their hardcoded source paths
  repointed at `_REPO_ROOT / "src" / "motion" / ...` and gained one
  extra `-I str(_REPO_ROOT / "src")` compiler flag apiece, for the same
  "qualified `motion/...h"` include" reason above.

**Surprises / things worth flagging to the team-lead:**

- The scale of the callsite sweep was much larger than the ticket text's
  own file list suggested — the ticket's "Files to modify" section named
  2 dedicated `test_motion_*.py` wrappers, but `body_kinematics.cpp` is
  ALSO compiled ad hoc by 19 other `src/tests/sim/{unit,plant,system,
  system/faults}` pytest wrappers (every harness that pulls in
  `otos_plant.cpp`, `drive.cpp`, `odometry.cpp`, `robot_loop.cpp`, or
  `fake_otos.cpp` transitively needs it). All 21 total wrapper files were
  found by grepping for the literal `"kinematics"` path segment and
  updated identically (scripted, not hand-edited one at a time) — no
  wrapper was missed; confirmed by the unchanged post-move pytest count.
- `src/firm/motion/DESIGN.md` and `src/firm/kinematics/DESIGN.md` are
  NOT deleted this ticket — they still hold the real math/derivation
  content for the three moved modules (no cheaper way to preserve that
  history without duplicating ~460 lines into a ticket-001 "stub"). Each
  now carries an "IN-FLIGHT RELOCATION" note pointing at
  `src/motion/DESIGN.md`. Ticket 004 (design-doc reconciliation) decides
  whether to delete/redirect them for good per sprint.md Open Question 4
  — left untouched otherwise, exactly as this ticket's own scope note
  asked.
- `.clasi/config.yaml`'s `sources:` list was NOT touched (still
  `[src/firm, src/host]`), per the locked scope decision — verified via
  `git diff --stat .clasi/config.yaml` showing no change.
