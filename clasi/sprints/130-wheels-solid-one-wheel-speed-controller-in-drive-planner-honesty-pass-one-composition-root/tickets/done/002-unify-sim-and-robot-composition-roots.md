---
id: '002'
title: Unify sim and robot composition roots
status: done
use-cases:
- SUC-003
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

- [x] `SimHarness` boots through `bootPlannerLimits()`/
      `installShaperLimits()`/`installRotationCalibration()`/
      `installDriveCalibration()` instead of its own `simPlannerLimits()`
      literals; any genuinely-different sim value is an explicit,
      commented override at the sim call site — never silent.
      (`src/sim/sim_harness.h`'s constructor calls `App::composeRobot()`,
      which calls all four internally; three explicit `BootOverrides` —
      trackWidth, controlPeriod/actuationDelay, otosConfig — are commented
      at the sim's own call site, plus a post-construction `setDutyPerSpeed()`
      override for the sim's own linear plant gain.)
- [x] `boot_config.cpp` is linked into the host lib; both roots bake the
      same robot-JSON calibration by default.
      (`src/sim/CMakeLists.txt`'s `CONFIG_SOURCES`; verified by
      `test_composition_root_parity.py`.)
- [x] A shared `composeRobot()` exists in `app/`; `main.cpp` and
      `SimHarness` are thin (~20-line) shells parameterized only by leaf
      implementations (real I2C bus vs. `SimPlant`).
      (`src/firm/app/boot_wiring.{h,cpp}`: `App::composeRobot()`/
      `App::RobotGraph`; `main.cpp`'s own body is now one `composeRobot()`
      call plus hardware-only leaf construction.)
- [x] `boot_wiring.cpp` is added to `src/sim/CMakeLists.txt` AND every
      pytest `_APP_SOURCES` list under `src/tests/sim/` (the known
      four-source-list trap) — verified by a clean host build with no
      link errors.
      (10 pytest files updated alongside `src/sim/CMakeLists.txt`; clean
      `cmake --build src/sim/build` and `just build-clean` both pass.)
- [x] The sim's step dt and the planner's `controlPeriod`/
      `actuationDelay` derive from the SAME constant (still 40 ms this
      ticket; ticket 007 changes the value later).
      (`SimHarness::kSimControlPeriod` and `kCycleDtUs` both derive from
      `App::RobotLoop::kCycle`.)
- [x] The 12 known sim-behavior test failures from adopting real
      shaping/trim/heading-hold/dutyFloor defaults are triaged: each
      gets either a documented per-test override
      (`planner().applyShaperLimits()` etc.) or an updated, justified
      expectation — none silently skipped or xfailed without comment.
      (10 failures found beyond the 2 known pre-existing (`test_clock_sync_
      activation`/`test_fake_transport`): a genuine OTOS-config parity gap
      (fixed via a third `BootOverrides` field, `otosConfig`); shaper/trim
      overrides added to `bench_test_config.cpp`'s `configureSimForBenchTest()`
      for protocol/queue-testing harnesses; a widened tolerance in
      `test_sim_wire_loopback.py` with a dated comment; two rotation-
      calibration tests `xfail`'d with a full explanation and a fresh
      follow-on issue, `clasi/issues/rotation-calibration-vs-live-heading-
      hold-gain.md`.)
- [x] A parity check (automated test or documented manual comparison)
      confirms sim and hardware construct identical `PlannerLimits`/
      drive-calibration values by default.
      (`src/tests/sim/system/composition_root_parity_harness.cpp` +
      `test_composition_root_parity.py`: field-by-field comparison,
      passing.)

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
