---
id: '002'
title: Unify sim and robot composition roots
status: open
use-cases: [SUC-003]
depends-on: []
github-issue: ''
issue: unify-sim-and-robot-composition-roots.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Unify sim and robot composition roots

## Description

Per `unify-sim-and-robot-composition-roots.md`: finish `boot_wiring`
adoption in `TestSim::SimHarness`, link `config/boot_config.cpp` into
the host lib, extract a shared `composeRobot(bus, clock, sleeper,
serial, radio, tuningStore*)`, derive the control period from one
shared constant in both roots (the VALUE stays 40 ms this ticket — the
planner-honesty pass, ticket 007, changes the value to 50 later), and
triage the resulting sim-behavior test fallout.

This lands BEFORE the wheel-controller work (tickets 003-006) by
deliberate sequencing (sprint Architecture Design Rationale Decision
1): the new controller's configuration (map gain/intercept, `biasMax`,
`vMin`, PID gains) must reach both sim and hardware identically, and
wiring it through `composeRobot()` once avoids repeating the exact
drift this issue already documents — `simPlannerLimits()`'s wheel-trim
gains silently booting at their fail-closed zero default while
hardware boots them live.

`main.cpp` currently boots through `App::effectiveTrackWidth()`/
`installDriveCalibration()`/`bootPlannerLimits()`/
`installShaperLimits()`/`installRotationCalibration()` (from an
existing, uncommitted-as-of-2026-07-31 `boot_wiring` start point that
did not survive into `master` — confirmed absent from the current tree
by `ls src/firm/app/boot_wiring.*`). This ticket creates it fresh and
converts the sim side to match.

## Acceptance Criteria

- [ ] `SimHarness` boots through `bootPlannerLimits()`/
      `installShaperLimits()`/`installRotationCalibration()`/
      `installDriveCalibration()` instead of its own `simPlannerLimits()`
      literals; any genuinely-different sim value is an explicit,
      commented override at the sim call site — never silent.
- [ ] `boot_config.cpp` is linked into the host lib; both roots bake the
      same robot-JSON calibration by default.
- [ ] A shared `composeRobot()` exists in `app/`; `main.cpp` and
      `SimHarness` are thin (~20-line) shells parameterized only by leaf
      implementations (real I2C bus vs. `SimPlant`).
- [ ] `boot_wiring.cpp` is added to `src/sim/CMakeLists.txt` AND every
      pytest `_APP_SOURCES` list under `src/tests/sim/` (the known
      four-source-list trap) — verified by a clean host build with no
      link errors.
- [ ] The sim's step dt and the planner's `controlPeriod`/
      `actuationDelay` derive from the SAME constant (still 40 ms this
      ticket; ticket 007 changes the value later).
- [ ] The 12 known sim-behavior test failures from adopting real
      shaping/trim/heading-hold/dutyFloor defaults are triaged: each
      gets either a documented per-test override
      (`planner().applyShaperLimits()` etc.) or an updated, justified
      expectation — none silently skipped or xfailed without comment.
- [ ] A parity check (automated test or documented manual comparison)
      confirms sim and hardware construct identical `PlannerLimits`/
      drive-calibration values by default.

## Testing

- **Existing tests to run**: full `src/tests/sim/` pytest suite;
  `motion_tests` ctest suite; a clean host build
  (`src/sim/CMakeLists.txt` + all `_APP_SOURCES` lists).
- **New tests to write**: a composition-root parity test comparing
  `composeRobot()`'s sim- and hardware-constructed `PlannerLimits`/
  drive-calibration values field-by-field.
- **Verification command**: `uv run pytest` (host/sim suites); a clean
  ARM build to confirm `main.cpp`'s shell still compiles.

## Implementation Plan

**Approach**: extract shared wiring incrementally — first make
`SimHarness` call the same `boot*()` functions `main.cpp` already uses
(or will, once `boot_wiring.cpp` exists), THEN extract the remaining
graph construction into `composeRobot()` so both roots become thin
shells. Every deliberate sim-only override is a visible, commented
exception at the sim call site, never a silent divergence.

**Files to create/modify**:
- `src/firm/app/boot_wiring.{h,cpp}` (new)
- `src/firm/main.cpp` (shell)
- `src/sim/sim_harness.h` (shell + call `boot_wiring`)
- `src/sim/CMakeLists.txt`
- ~10 pytest `_APP_SOURCES` lists under `src/tests/sim/`
- `src/firm/config/boot_config.{h,cpp}` (host-lib linkage)

**Testing plan**: full sim pytest suite; triage each of the 12 known
failures explicitly (document the resolution per test, not just make
them pass silently); new parity test above.

**Documentation updates**: `src/motion/DESIGN.md` / `src/sim/DESIGN.md`
updated to describe `composeRobot()` as the new shared composition
root, superseding the "gap is the composition root" framing in the
source issue.
