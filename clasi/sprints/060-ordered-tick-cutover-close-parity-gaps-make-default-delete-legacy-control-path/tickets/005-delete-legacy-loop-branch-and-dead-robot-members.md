---
id: '005'
title: Delete legacy loop branch and dead Robot members
status: open
use-cases:
- SUC-005
depends-on:
- '004'
github-issue: ''
issue: make-ordered-tick-the-default-close-parity-gaps.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete legacy loop branch and dead Robot members

## Description

With `USE_ORDERED_TICK` now the compile-time default and all tests green (ticket 004),
the legacy control path is dead code. This ticket deletes it.

### Deletions

**`LoopTickOnce.cpp` — legacy branch and ifdef scaffolding**

Delete the entire `#ifndef USE_ORDERED_TICK` block (lines 57-159) and the surrounding
`#ifdef`/`#else`/`#endif` preprocessor scaffolding. The file becomes the ordered-tick
body only, with no conditional compilation. The comment block at the top (lines 21-55
documenting the two paths and parity gaps) can be replaced with a concise description
of the ordered-tick sequence.

**`Robot.h` / `Robot.cpp` — dead members**

- `subsystems::Drive drive` member: delete the declaration from `Robot.h` (~line 121)
  and the construction wiring from `Robot.cpp`. Delete the `#include` of
  `"../subsystems/drive/Drive.h"` from `Robot.h`. Delete `Drive.h` and `Drive.cpp`
  from `source/subsystems/drive/` (confirm no other files include them first).
- `robot._tlmBoundFn` and `robot._tlmBoundCtx` fields: audit all references. These
  were used only to pass the TLM sink into `drive.periodic()`. If no live call sites
  remain after ticket 001's removal of `drive.periodic()`, delete the fields and
  their usages in `Robot.h` and `Robot.cpp`.
- `MotionController` (old imperative class): `MotionController2` wraps it by holding
  a `MotionController&` reference. Determine whether to:
  (a) Retain the old class as a private value member of `MotionController2` (rename
      the type internally, don't expose it); or
  (b) Inline the old class logic into `MotionController2`.
  Either is acceptable. The key requirement: the old `source/superstructure/MotionController.h`
  is no longer a public header; the old class name disappears from `Robot.h` and any
  other public include.

**What to KEEP**

- `subsystems::LineSensor lineSensor` and `subsystems::ColorSensor colorSensor_` —
  RETAINED as Robot members (Sensors facade holds refs to them). Only their removed
  `.periodic()` call sites (already done in tickets 001-003).
- `subsystems::Ports ports` and `ports.periodic(ts, now)` — RETAINED.
- All `estimate.*` calls in `RobotTelemetry.cpp` that are not replaced by
  `drive2.state()` (e.g., `estimate.ekfRejectCount()` may stay if DrivetrainState
  does not expose it).

### Postcondition

After this ticket, `LoopTickOnce.cpp` contains no preprocessor guards. `Robot.h`
contains no `subsystems::Drive drive` member. The old `Drive.h/.cpp` files are
deleted. `grep -r USE_ORDERED_TICK source/ tests/` returns nothing.

## Acceptance Criteria

- [ ] `LoopTickOnce.cpp` contains no `#ifndef USE_ORDERED_TICK` / `#ifdef` / `#else` / `#endif` guards.
- [ ] `source/subsystems/drive/Drive.h` and `Drive.cpp` (the old legacy Drive) do not exist.
- [ ] `Robot.h` has no `subsystems::Drive drive` member declaration.
- [ ] `_tlmBoundFn` / `_tlmBoundCtx` are deleted from `Robot.h/.cpp` (if no live call sites found) OR retained with a comment documenting the live call site.
- [ ] `grep -r USE_ORDERED_TICK source/ tests/` returns nothing.
- [ ] Codebase compiles cleanly.
- [ ] `uv run python -m pytest` — green except the 2 known-baseline failures.

## Implementation Plan

### Approach

Work in sub-steps to keep the compiler clean at each step:
1. Delete the `#ifndef` legacy block from `LoopTickOnce.cpp` and remove the header
   includes that were only needed by the legacy path.
2. Audit `_tlmBoundFn`/`_tlmBoundCtx` references. Delete if dead.
3. Delete `subsystems::Drive drive` from `Robot.h/.cpp`. Rebuild and fix any
   compile errors (probably just include removal).
4. Delete `source/subsystems/drive/Drive.h/.cpp` files if no remaining includes.
5. Handle old `MotionController` (see decision in description above).
6. Run `grep -r USE_ORDERED_TICK source/ tests/` — must return nothing.
7. Run full test suite.

### Files to modify

- `source/robot/LoopTickOnce.cpp` — remove legacy block and scaffolding.
- `source/robot/Robot.h` — remove dead member declarations and includes.
- `source/robot/Robot.cpp` — remove dead member initialization and wiring.
- `source/subsystems/drive/Drive.h` — DELETE FILE.
- `source/subsystems/drive/Drive.cpp` — DELETE FILE.
- `source/superstructure/MotionController.h/.cpp` (old) — DELETE or move to
  internal-only depending on chosen approach.

### Testing plan

1. Rebuild sim: `cd tests/_infra/sim && python3 build.py`
2. `uv run python -m pytest` — green except 2 known-baseline failures.
3. `grep -r USE_ORDERED_TICK source/ tests/` — returns nothing.

### Documentation updates

Update the comment block at the top of `LoopTickOnce.cpp` to describe the ordered-tick
sequence without mentioning the legacy path or `USE_ORDERED_TICK`.
