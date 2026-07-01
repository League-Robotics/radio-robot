---
id: '061'
title: "Eliminate legacy MotionController class \u2014 fold goal-closure into Planner,\
  \ scrub scaffolding names"
status: planning-docs
branch: sprint/061-eliminate-legacy-motioncontroller-class-fold-goal-closure-into-planner-scrub-scaffolding-names
use-cases: []
issues:
- internalize-legacy-motioncontroller-into-planner.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 061: Eliminate legacy MotionController class — fold goal-closure into Planner, scrub scaffolding names

## Goals

Eliminate the standalone legacy `MotionController` class entirely. Move its
implementation (the S/T/D/G state machines, begin*() bodies, BVC ownership,
MotionCommand ownership, and mode/safety logic) into `Planner`, making `Planner`
the single goal-closure engine. Reroute all call sites that previously reached
`MotionController` directly (Robot, Superstructure, command handlers, telemetry)
to go through `Planner`. Delete the three legacy files
(`MotionController.h/.cpp`, `MotionControllerBegin.cpp`). Finally, scrub
residual `drive2`/`mc2` scaffolding names from the test infrastructure.

## Problem

Sprint 060 completed the ordered-tick cutover and renamed the `2`-suffix
scaffolding to permanent names (`Drive`, `Planner`, `bvc`). However, the old
imperative `MotionController` class was intentionally deferred — it remained as
a public `Robot` value member, wrapped by reference inside both `Planner`
(`_mc`) and `Superstructure` (`_mc`). This leaves three legacy source files
still present and forces all call sites to reach past `Planner` to the wrapped
`MotionController` directly, violating the intended encapsulation boundary.

Additionally, test-infrastructure C-ABI shim symbols (`drive2_api_*`,
`bus_drain_api_drive2_*`) and Python helpers (`Drive2Ctx`,
`test_drive2_subsystem.py`, `test_motioncontroller2_smoke.py`) still carry
sprint-059 scaffolding names that should be canonical.

## Solution

**Absorb-into-Planner approach:** `Planner` takes ownership of all
`MotionController` implementation. The members currently held by
`MotionController` (`_bvc`, `_activeCmd`, `_hwState`, `_safetyEnabled`, etc.)
become private members of `Planner`. The begin*() bodies (currently in
`MotionControllerBegin.cpp`) and the `driveAdvance()` body (in
`MotionController.cpp`) are moved into `Planner.cpp` or a `Planner`-owned
translation unit (`PlannerBegin.cpp`). `Superstructure` and command handlers
that previously took `MotionController&` are updated to take `Planner&`.
`Robot` drops the `motionController` value member; `Planner` owns the
implementation directly.

The execution is decomposed into 7 individually-buildable tickets that keep the
tree compiling and all tests green at every step.

## Success Criteria

- No `MotionController` class, files, or non-comment references remain in
  `source/`.
- `Planner` is the single goal-closure engine; `Superstructure` and all
  command handlers reference `Planner` directly.
- Host suite green except the 2 known-baseline config-golden failures
  (`test_tovez_validates_against_schema`, `test_default_robot_config_unchanged`),
  run **twice** for stability confirmation.
- `test_golden_tlm.py`, `test_059_ordered_tick_parity.py`, and
  `test_planner_subsystem.py` all pass.
- Firmware `build.py --clean` produces a clean `MICROBIT.hex`.
- Test-infra scaffolding names (`drive2_api_*`, `bus_drain_api_drive2_*`,
  `Drive2Ctx`, `test_drive2_subsystem.py`, `test_motioncontroller2_smoke.py`)
  renamed to canonical names.
- Sprint branch left open for stakeholder bench-test on physical tovez robot
  before any merge to master.

## Scope

### In Scope

- Move `MotionController` implementation (all members, driveAdvance, begin*()
  bodies) into `Planner`.
- Update `Planner` constructor and public interface to expose the methods call
  sites need (mode, beginX, disableSafetyOneShot, hasActiveCommand,
  emitToActiveChannel, setHardwareState, setRobotCtx, getMotionCommands context,
  etc.).
- Reroute `Superstructure` to hold `Planner&` instead of `MotionController&`.
- Reroute `MotionCommands.h` `MotionCtx::mc` from `MotionController*` to
  `Planner*`.
- Reroute `SystemCommands.cpp`, `Robot.cpp` (constructor, otosCorrect,
  distanceDrive), `RobotTelemetry.cpp` to use `planner.*` instead of
  `motionController.*`.
- Delete `source/superstructure/MotionController.h/.cpp` and
  `source/control/MotionControllerBegin.cpp`.
- Fix all includes that pull in `MotionController.h`.
- Rename test-infra `drive2`/`mc2` C-ABI symbols and Python call sites.
- Host build (`cmake --build`) must stay green after every ticket.
- Full test suite run twice at the end.
- Firmware `build.py --clean` + hex verify.
- Bench checklist for tovez updated for sprint 061.

### Out of Scope

- Any behavior change to the goal-closure logic (this is a structural move only).
- `subsystems::Ports` / `ports.periodic()` cleanup (no `Ports2` equivalent exists).
- `robot.state.actual` / `RobotStateContainer` cleanup (vestigial but not this sprint).
- Holonomic (`togov`) drivetrain changes.
- New motion features.

## Test Strategy

- After every ticket: `cmake --build build_sim` must succeed with zero errors.
- After every ticket that touches the motion path: run
  `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py
  tests/simulation/unit/test_059_ordered_tick_parity.py
  tests/simulation/unit/test_planner_subsystem.py` to confirm golden-TLM
  parity is preserved.
- Final ticket: run the full suite **twice** (`uv run python -m pytest`) and
  confirm all pass except the 2 known-baseline config-golden failures.
- Test command is `uv run python -m pytest` (NOT `uv run pytest`).
- Firmware: `build.py --clean` followed by `MICROBIT.hex` decode to confirm
  the clean firmware binary is produced.

## Architecture Notes

The chosen approach is **absorb-into-Planner**: `Planner` becomes the class
that owns `_bvc`, `_activeCmd`, and all the begin*/driveAdvance logic directly.
`MotionController` ceases to exist as a class.

The key sequencing insight is that tickets 001-003 are purely additive —
`Planner` gains new methods that delegate to its wrapped `_mc`, so all existing
call sites continue to work. Rerouting happens in ticket 004 (Superstructure),
ticket 005 (handlers and telemetry), ticket 006 (absorb implementation + drop
Robot member), and ticket 007 (delete files). This ensures the build stays
green at each step.

Declaration order in `Robot.h` changes: the `motionController` value member
is dropped, `Planner` constructs its own `_bvc`/`_activeCmd` etc. directly.
The `planner` member's constructor signature changes — it no longer takes
`MotionController&`; it takes `MotorController&`, `Odometry&`, and
`const subsystems::Drive&`.

## GitHub Issues

(None yet — to be linked after ticket creation.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Expose MotionController API on Planner via delegation (additive) | — |
| 002 | Reroute Superstructure from MotionController to Planner | 001 |
| 003 | Reroute command handlers, telemetry, and Robot orchestration to Planner | 001, 002 |
| 004 | Absorb MotionController implementation into Planner; drop Robot motionController member | 003 |
| 005 | Delete MotionController source files; fix all includes; verify grep clean | 004 |
| 006 | Scrub test-infra drive2/mc2 scaffolding names (C ABI + Python, atomic rename) | 005 |
| 007 | Final verification: double test-suite run, firmware clean build, bench checklist | 006 |

Tickets execute serially in the order listed.
