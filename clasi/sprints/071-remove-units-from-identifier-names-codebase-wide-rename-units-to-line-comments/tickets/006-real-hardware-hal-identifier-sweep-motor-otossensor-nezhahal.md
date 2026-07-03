---
id: '006'
title: 'Real-hardware HAL identifier sweep: Motor, OtosSensor, NezhaHAL'
status: open
use-cases: [SUC-005]
depends-on: ['002']
github-issue: ''
issue: remove-units-from-identifier-names.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Real-hardware HAL identifier sweep: Motor, OtosSensor, NezhaHAL

## Description

Rename unit-suffixed identifiers in the real-hardware sensor/motor
drivers: `source/hal/real/Motor.{h,cpp}`, `source/hal/real/
OtosSensor.{h,cpp}`, `source/robot/NezhaHAL.{h,cpp}`. This ticket depends
only on ticket 002 (it reads the renamed `RobotConfig` fields, e.g.
`Motor.cpp`'s local `mmPerDeg` mirrors `RobotConfig::wheelTravelCalibL/R`)
and is otherwise independent of tickets 005 and 007 — the real HAL and the
sim plant (ticket 007) are parallel implementations of the same
interfaces and never call into each other (`architecture-update.md`
Step 5 "Why").

Scope:
- `source/hal/real/Motor.{h,cpp}`: local `mmPerDeg` → `wheelTravelCalib`
  (mirrors ticket 002's derived-unit field rename), `_lastPositionMm` →
  `_lastPosition` `// [mm]`.
- `source/hal/real/OtosSensor.{h,cpp}`, `source/robot/NezhaHAL.{h,cpp}`:
  remaining unit-suffixed locals/members renamed.

**Ambiguity-resolution watch point**: `Motor` is the other of the two
places (with `Odometry`, ticket 005) flagged in `architecture-update.md`'s
Open Question 4 as most likely to have a raw-ticks vs. mm-scaled sibling
pair. Apply the same rule if found.

`extern "C"` function names elsewhere in the HAL layer are already
unit-free per the Wire-Compatibility Exclusion Table's own audit — this
ticket only touches parameter/local/member names, never a function's
exported name (there is no C-ABI boundary inside `Motor.{h,cpp}`/
`OtosSensor.{h,cpp}`/`NezhaHAL.{h,cpp}` itself; that boundary lives in
`tests/_infra/sim/*.cpp`, ticket 007's concern).

See `architecture-update.md` Step 5 ("006 — Real-hardware HAL sweep"),
Decision 5, Open Question 4; `usecases.md` SUC-005.

## Acceptance Criteria

- [ ] `source/hal/real/Motor.{h,cpp}`: local `mmPerDeg` →
      `wheelTravelCalib` `// [mm/deg]`; `_lastPositionMm` →
      `_lastPosition` `// [mm]`.
- [ ] `source/hal/real/OtosSensor.{h,cpp}`, `source/robot/NezhaHAL.{h,cpp}`:
      no remaining unit-suffixed identifier, excluding comments.
- [ ] Every renamed identifier carries a `// [unit]` comment.
- [ ] Any raw-ticks vs. mm-scaled sibling-pair collision found in `Motor`
      is resolved per the ambiguity-resolution rule.
- [ ] OTOS pose/velocity readings and encoder-derived speed are
      numerically identical to pre-ticket for the same input sequence.
- [ ] OTOS and motor-calibration unit tests pass with unchanged numeric
      assertions.
- [ ] Full test suite green (`uv run python -m pytest`).
- [ ] `--clean` sim build performed before running tests (Motor.{h,cpp} is
      a firmware/sim-shared source file).

## Testing

- **Existing tests to run**: OTOS unit tests, motor-calibration unit
  tests, `test_sim_otos_lever_arm.py` (shares logic paths with the real
  HAL calibration math), full default suite.
- **New tests to write**: none required for the rename itself. If the
  ambiguity-resolution rule is applied, add/update a test asserting both
  renamed identifiers are read/written independently.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Rename `Motor.{h,cpp}`'s two flagged identifiers first (the
derived-unit case has a known-correct answer from ticket 002's
precedent), then sweep `OtosSensor.{h,cpp}` and `NezhaHAL.{h,cpp}` for
remaining unit-suffixed names. Grep after each file to confirm no stray
reference remains.

**Files to modify**:
- `source/hal/real/Motor.h`, `Motor.cpp`
- `source/hal/real/OtosSensor.h`, `OtosSensor.cpp`
- `source/robot/NezhaHAL.h`, `NezhaHAL.cpp`

**Testing plan**: `--clean` sim build, then OTOS/motor-calibration unit
tests in isolation, then the full suite.

**Documentation updates**: none in this ticket (ticket 008's final sweep
covers prose docs).
