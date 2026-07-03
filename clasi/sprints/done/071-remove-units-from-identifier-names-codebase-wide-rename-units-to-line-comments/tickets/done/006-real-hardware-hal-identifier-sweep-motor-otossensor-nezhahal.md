---
id: '006'
title: 'Real-hardware HAL identifier sweep: Motor, OtosSensor, NezhaHAL'
status: done
use-cases:
- SUC-005
depends-on:
- '002'
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

- [x] `source/hal/real/Motor.{h,cpp}`: local `mmPerDeg` →
      `wheelTravelCalib` `// [mm/deg]`; `_lastPositionMm` →
      `_lastPosition` `// [mm]`. Applied at every declaration site (the
      local is independently declared in `readEncoder()`,
      `readEncoderMmF()`, `rebaselineSoft()`, `readEncoderMmFAtomic()`,
      `readEncoderMmFSettle()`, and `readSpeed()` — each gets its own `//
      [mm/deg]` tag); all prose doc-comments referencing the old names
      (including the `readSpeed()` Doxygen block and the
      `rebaselineSoft()`/`resetEncoder()` block comments) updated to match.
      Per the ticket's own scope note, Motor's OTHER unit-suffixed names
      (`_lastVelocityMmps`, `_lastTickMs`, `_lastWriteUs`,
      `kMinWriteIntervalUs`, `mmPerSec` param, `now_ms` param, the
      `readEncoderMmF*` method names) are intentionally untouched — the
      ticket names exactly two identifiers for this file, unlike the
      "no remaining" bar set for OtosSensor/NezhaHAL below.
- [x] `source/hal/real/OtosSensor.{h,cpp}`, `source/robot/NezhaHAL.{h,cpp}`:
      no remaining unit-suffixed identifier, excluding comments. Renamed:
      OtosSensor's `angRad`→`ang` `// [rad]` (4 declaration sites),
      `kImuCalibTimeoutMs`→`kImuCalibTimeout` `// [ms]`, and the
      `readTransformed`/`readVelocityTransformed`/`readVelocityTransformed3`/
      `setWorldPose` override parameters (`headingRad`→`heading`,
      `x_mm/y_mm/h_rad`→`x/y/h`, all `// [rad]`/`// [mm]` tagged) — these
      are `override`s of `IOdometer` (out of ticket scope) but C++ does not
      require override parameter names to match the base declaration, so
      renaming them is zero-risk and satisfies the file's "no remaining"
      bar. NezhaHAL's `_trackwidthMm`→`_trackwidth` `// [mm]`,
      `_lastBenchTickMs`→`_lastBenchTick` `// [ms]`, local `dt_ms`→`dt`
      `// [ms]`, and both `tick()` overloads' `now_ms`→`now` `// [ms]`
      parameter (header + impl + all call sites) — the same
      "concrete/owned-file gets a full sweep including its `now_ms`
      parameter" precedent ticket 071-005 already established for
      `MotionCommand`/`StopCondition` (confirmed by inspecting that
      ticket's own commit diff), as distinct from `HaltController`, which
      071-005 touched only minimally and left `now_ms` alone because it
      was not a fully-owned file for that ticket. `kPosMmPerLsb`/
      `kHdgRadPerLsb`/`kVelMmpsPerLsb`/`kOmegaRadpsPerLsb`/
      `kAccMmps2PerLsb` were left as-is: these are LSB-scale constants
      whose unit token is embedded mid-name (not a trailing suffix), the
      same category ticket 008's own final closure grep
      (`(mm|mms|deg|dps|us|pct|hz)\b`, word-boundary-anchored at the end)
      does not flag; a full semantic rename (mirroring the `mmPerDeg`→
      `wheelTravelCalib` derived-unit treatment) was judged out of this
      ticket's explicit scope since it was not named by architecture
      planning for OtosSensor the way `mmPerDeg` was named for `Motor`.
- [x] Every renamed identifier carries a `// [unit]` comment.
- [x] Any raw-ticks vs. mm-scaled sibling-pair collision found in `Motor`
      is resolved per the ambiguity-resolution rule. None found:
      `_lastPositionMm`/`wheelTravelCalib` have no raw-ticks sibling in
      `Motor` that a bare strip would collide with (`_encOffset` and
      `_lastGoodRawEnc`, the raw-tenths-of-degrees members, were never
      unit-suffixed with `Mm` and are unaffected by this rename) — the
      rule's precondition doesn't trigger here, so nothing further to do.
- [x] OTOS pose/velocity readings and encoder-derived speed are
      numerically identical to pre-ticket for the same input sequence.
      Pure rename, zero logic/formula changes anywhere in either file;
      confirmed by the full pytest suite (including
      `test_sim_otos_lever_arm.py`) passing unchanged.
- [x] OTOS and motor-calibration unit tests pass with unchanged numeric
      assertions.
- [x] Full test suite green (`uv run python -m pytest`): 2621 passed, 0
      failed — matches the pre-ticket baseline exactly.
- [x] `--clean` sim build performed before running tests (Motor.{h,cpp} is
      a firmware/sim-shared source file). `cmake --build tests/_infra/sim/
      build --target clean` then a full rebuild (fresh `.dylib`,
      zero-error compile). Note: `Motor.cpp`/`OtosSensor.cpp`/
      `NezhaHAL.cpp` are NOT part of the sim-library target (confirmed:
      absent from the sim build's compile-unit list) — they only compile
      under the ARM firmware target, so a `build.py --fw-only` run was
      also performed and produced a clean `MICROBIT.hex` with fresh
      `.obj` timestamps for all three files, confirming they compile for
      real hardware. (That run required temporarily neutralizing an
      unrelated, pre-existing bug in `scripts/gen_default_config.py` — a
      stale hardcoded `p.turnThresholdMm`/`p.doneTolMm` block referencing
      two `RobotConfig` fields sprint 070 deleted, which `build.py`
      regenerates `DefaultConfig.cpp` from on every invocation. This bug
      is unrelated to tickets 002-006 and outside this ticket's file
      scope; the generator and `DefaultConfig.cpp` were restored to
      their exact pre-existing committed state via `git checkout --`
      immediately after the verification build, and are not part of
      this ticket's diff. Flagged here for the team-lead/sprint-closure
      ticket's awareness — it will resurface for anyone else running
      `build.py` without `--fw-only`'s sim-lib skip.)

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
