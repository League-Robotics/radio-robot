---
id: '007'
title: Delete the parked duty stage; one 50 ms control-period constant
status: done
use-cases:
- SUC-004
depends-on:
- '005'
- '002'
github-issue: ''
issue:
- planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md
- bench-duty-readers-see-zero-after-stageduty-park.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete the parked duty stage; one 50 ms control-period constant

## Description

Precondition step of the planner-honesty pass
(`planner-honesty-pass-50ms-period-tick-state-machine-limits-
reduction.md`, item 0). Delete `WheelPid`, `Planner::stageDuty()`,
`dutyLeft_`/`dutyRight_` and their accessors, `wheel_pid_test.cpp`,
`planner_duty_scenarios_test.cpp`, `tests/duty_plant.h`, and their
ctest registrations. This is safe now because ticket 005 already made
the map-adaptation controller (in `Drive`) the proven, shipped law —
the parked duty stage has no future, reversing sprint 128 Decision 2's
PARK. `pid.*` wire-key repointing already happened in ticket 005; this
ticket just removes the dead stage those keys used to (silently)
target.

Also lands the "one 50 ms period" change: `App::RobotLoop::kCycle`
40 -> 50 (the pacer paces again: busy ~21 ms < 50, so the delivered
period becomes exactly 50 and stable), `bootPlannerLimits()`'s
`controlPeriod`/`actuationDelay` 47 -> 50, and verification that the
sim harness (unified onto one shared constant by ticket 002) derives
the same value automatically.

Resolves `bench-duty-readers-see-zero-after-stageduty-park.md`'s
residual: `hil_drive.py --duty` and `square_tour_sim.py`'s
`plannerDuty()` read now permanently see zero (not just while parked)
once the stage is deleted — apply that issue's own suggested
resolution (drop the duty-read modes, or print a clear "duty stage
removed" message), never leave a silent zero.

## Acceptance Criteria

- [ ] `WheelPid`, `stageDuty()`, `dutyLeft_`/`dutyRight_`, their tests,
      and ctest registrations deleted (git preserves history for any
      future duty-sink revisit).
- [ ] `App::RobotLoop::kCycle` is 50; `bootPlannerLimits()`'s
      `controlPeriod`/`actuationDelay` are 50; the sim harness's derived
      constant matches automatically via ticket 002's shared-constant
      plumbing.
- [ ] `Telemetry::kPrimaryPeriod` behavior at a 50 ms cycle is
      documented (effective emission rate stated, not just left to
      infer).
- [ ] `hil_drive.py --duty` and `square_tour_sim.py`'s duty read no
      longer silently report zero — either updated to read a live value
      or emit an explicit "duty stage removed" message.
- [ ] `src/motion/DESIGN.md`'s "wheel control generations" note updated:
      generation 2 (duty stage) deleted; generation 3 (the `Drive`
      controller from tickets 004/005) is the law.

## Testing

- **Existing tests to run**: full `planner_tests` ctest suite (must
  stay green after the deletion); full sim pytest suite.
- **New tests to write**: none expected beyond removing the deleted
  stage's own tests; a `cycle_period` telemetry smoke check at the new
  50 ms value.
- **Verification command**: `uv run pytest`; ctest for `motion_tests`;
  a bench smoke run confirming `cycle_period` reads ~50 ms.

## Implementation Plan

**Approach**: deletion + one constant change, minimal new logic — the
duty stage's removal is mechanical once ticket 005 has already made it
dead code.

**Files to create/modify**:
- `src/motion/planner/wheel_pid.{h,cpp}` (deleted)
- `src/motion/planner/planner.{h,cpp}` (remove `stageDuty()`,
  `dutyLeft_`/`dutyRight_`)
- `src/motion/planner/tests/wheel_pid_test.cpp`,
  `planner_duty_scenarios_test.cpp`, `tests/duty_plant.h` (deleted)
- `CMakeLists.txt` ctest registrations
- `src/firm/app/robot_loop.h` (`kCycle`)
- `src/firm/app/boot_wiring.cpp` (from ticket 002; `bootPlannerLimits()`)
- `src/sim/sim_harness.h` (verify derived constant)
- `src/tests/bench/hil_drive.py`, `src/tests/bench/square_tour_sim.py`
- `src/motion/DESIGN.md`

**Testing plan**: full `planner_tests` ctest suite green; bench square
tour smoke run at 50 ms confirming `cycle_period` telemetry.

**Documentation updates**: `src/motion/DESIGN.md`'s generations note,
as above.
