---
id: '007'
title: 'Sim-library identifier sweep: PhysicsWorld, SimOdometer, SimSetters, SimCommands'
status: done
use-cases:
- SUC-006
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim-library identifier sweep: PhysicsWorld, SimOdometer, SimSetters, SimCommands

## Description

Rename internal (non-wire) unit-suffixed identifiers in the sim plant and
its `SIMSET`/`SIMGET` glue: `source/hal/sim/PhysicsWorld.{h,cpp}`,
`source/hal/sim/SimOdometer.{h,cpp}`, `source/commands/
SimSetters.{h,cpp}`, `source/commands/SimCommands.cpp` (function/local
names only), and `tests/_infra/sim/*.cpp` (parameter names only). This
ticket depends only on ticket 002 (it reads renamed `RobotConfig` fields)
and is independent of tickets 005/006.

**HARD CONTRACT: `kSimRegistry[]` wire key strings never change.**
`SimCommands.cpp`'s `kSimRegistry[]` binds the wire key string and the
internal setter/getter function name as two independent literals,
mirroring `ConfigRegistry`'s pattern (`architecture-update.md` Decision 2,
extended to the sim surface). Only the internal function/field name
changes; every `SIMSET`/`SIMGET` key string listed in the
Wire-Compatibility Exclusion Table (`trackwidthMm`, `motorOffsetL/R`,
`encScaleErrL/R`, `encSlipL/R`, `encNoiseL/R`, `otosLinScaleErr`,
`otosAngScaleErr`, `otosLinNoise`, `otosYawNoise`, `otosLinDriftMmS`,
`otosYawDriftDegS`, `bodyRotScrub`, `bodyLinScrub`) is preserved
byte-identical.

**Namespace-collision warning** (from the Exclusion Table, flagged
explicitly for this ticket's implementer): `trackwidthMm` exists as
**three separate, independently-declared strings/names** that happen to
share a substring — the `SIMSET` wire key (`kSimRegistry[]`, excluded,
untouched), the `RobotConfig::trackwidthMm` C++ field name (already
renamed to `trackwidth` by ticket 002), and the `"tw"` `SET` wire key
(also excluded, already unit-free). Do not conflate these when grepping —
a blanket rename of anything matching `trackwidthMm` would incorrectly
touch the `SIMSET` key string. Similarly, `otosLinDriftMmS`/
`otosYawDriftDegS` name both a `SIMSET` key (untouched) and an internal
`simsetters::` function (renamed) — rename only the function.

Scope:
- `source/hal/sim/PhysicsWorld.{h,cpp}`: `_driftPerTickMm` →
  `_driftPerTick`, `sigmaMm` → `sigma`, `encoderScaleErrL/R` locals, etc.
  renamed; internal method names mirroring `SIMSET` keys (e.g.
  `simsetters::otosLinDriftMmS`) renamed to e.g.
  `simsetters::otosLinearDrift`.
- `source/hal/sim/SimOdometer.{h,cpp}`, `source/commands/
  SimSetters.{h,cpp}`: mirrors the above pattern.
- `tests/_infra/sim/sim_api.cpp`, `drive_api.cpp`: parameter names only
  (`float mm` → `float distance  // [mm]`, resolved per convention);
  function names untouched — already unit-free per the Exclusion Table's
  audit (ctypes calls from `host/robot_radio/io/sim_conn.py` are
  positional, so parameter names are not part of the ABI a Python caller
  depends on — zero `host/` changes required).

See `architecture-update.md` Step 5 ("007 — Sim-library sweep"), the
Wire-Compatibility Exclusion Table's `SIMSET`/`SIMGET` row and namespace-
collision warning; `usecases.md` SUC-006.

## Acceptance Criteria

- [x] `source/hal/sim/PhysicsWorld.{h,cpp}`: `_driftPerTickMm`→
      `_driftPerTick`, `sigmaMm`→`sigma`, `encoderScaleErrL/R` and peers
      renamed; each carries a `// [unit]` comment.
      **Correction found while implementing**: `_driftPerTickMm` actually
      lives in `SimOdometer` (not `PhysicsWorld`) — renamed there (see
      below) as `_linearDriftPerTick`/`_yawDriftPerTick` (a bare
      `_driftPerTick` for one and not the other would collide, since
      `SimOdometer` has BOTH a linear and an angular drift field — the
      ambiguity-resolution rule in `docs/coding-standards.md` applies;
      "Linear"/"Yaw" mirrors this exact knob's own wire-key vocabulary,
      `otosLinDriftMmS`/`otosYawDriftDegS`, immediately adjacent in
      `SimSetters.h`). `encoderScaleErrL/R` were already unit-free in
      `PhysicsWorld` (nothing to strip) — kept as-is, matching ticket
      006's precedent for identifiers a ticket names but that turn out
      not to need a rename. Renamed in `PhysicsWorld` itself: `sigmaMm`→
      `sigma` (`setEncoderNoise`), `update(uint32_t dt_ms)`→`update(dt)`
      `// [ms]`, the fully-private `kNominalMaxMms`/`_nominalMaxMms`/
      `nominalMaxMms()`/`setNominalMaxMms()`→`kNominalMaxSpeed`/
      `_nominalMaxSpeed`/`nominalMaxSpeed()`/`setNominalMaxSpeed()`
      `// [mm/s]`, and `setReportedEncoder`'s `mm` param→`position`
      `// [mm]`. **Deliberately NOT renamed** (verified by full-repo
      grep before touching): `trueEncLMm/RMm`, `trueVelLMms/RMms`,
      `reportedEncLMm/RMm`, `trackwidthMm()`/`setTrackwidth()`/
      `_trackwidthMm`/`kDefaultTrackwidthMm` — these public accessor
      NAMES are called directly by `source/hal/sim/WorldView.h`,
      `SimMotor.{h,cpp}`, and `SimHardware.{h,cpp}`, none of which are in
      this ticket's file scope; renaming them would cascade into three
      files this ticket does not own. `PhysicsWorld`'s own acceptance
      bar (unlike SimOdometer/SimSetters below) is a named list, not a
      "no remaining" bar — mirrors ticket 006's treatment of `Motor.cpp`.
- [x] Internal `simsetters::` function names mirroring `SIMSET` keys
      renamed (e.g. `otosLinDriftMmS`→`otosLinearDrift`); their
      corresponding `kSimRegistry[]` key-string arguments in
      `SimCommands.cpp` are byte-identical to pre-ticket (diffed).
      Renamed: `trackwidthMm`/`getTrackwidthMm`→`trackwidth`/
      `getTrackwidth` (the namespace-collision-flagged case),
      `otosLinDriftMmS`/`getOtosLinDriftMmS`→`otosLinearDrift`/
      `getOtosLinearDrift`, `otosYawDriftDegS`/`getOtosYawDriftDegS`→
      `otosYawDrift`/`getOtosYawDrift`. `git diff` of `SimCommands.cpp`
      confirms only the function-pointer arguments changed on these 3
      `kSimRegistry[]` rows — every key string is untouched.
- [x] Every `kSimRegistry[]` key string listed in the Wire-Compatibility
      Exclusion Table's `SIMSET`/`SIMGET` row is unchanged (diffed against
      pre-ticket source) — including `trackwidthMm` (the sim key, distinct
      from the already-renamed `RobotConfig::trackwidth` C++ field and the
      excluded `"tw"` `SET` key). Confirmed: all 17 rows' key strings are
      byte-identical in `git diff source/commands/SimCommands.cpp`.
- [x] `source/hal/sim/SimOdometer.{h,cpp}`, `source/commands/
      SimSetters.{h,cpp}`: no remaining internal unit-suffixed identifier
      outside the preserved key strings. Verified by grep sweep after
      editing: `SimOdometer.{h,cpp}` has zero remaining hits.
      `SimSetters.h`'s only remaining hits are (a) documented references
      to `SimHardware`/`PhysicsWorld`'s own untouched `trackwidthMm()`
      accessor (out-of-scope files, see above) and (b) `kDegToRad`/
      `kRadToDeg`, unit-conversion-factor constants with the unit token
      embedded mid-name rather than as a trailing suffix — the same
      category ticket 006 established as out of this convention's scope
      (its `kPosMmPerLsb`/`kHdgRadPerLsb` precedent). Also renamed to
      reach this bar: `encoderNoiseL/R`/`encoderNoise`'s `sigmaMm`→
      `sigma` params, and the drift-conversion helpers' local
      `periodMs`→`period`/`radPerSec`→`rate` variables.
- [x] `tests/_infra/sim/sim_api.cpp`, `drive_api.cpp`: parameter names
      renamed per convention; exported `extern "C"` function names
      byte-identical to pre-ticket (already unit-free). Renamed in
      functions that call directly into `PhysicsWorld`/`SimOdometer`/
      `simsetters::`: `sim_set_enc_l/r`, `sim_set_reported_enc_l/r`,
      `sim_set_otos_pose`, `sim_set_true_pose`,
      `sim_set_true_wheel_travel`, `sim_set_true_velocity`,
      `sim_set_encoder_noise` (`sim_api.cpp`); `drive_api_
      enable_otos_sim_model` (`drive_api.cpp`). Functions that reach
      other subsystems (line/color/servo/port/bench-OTOS,
      `drive_api_apply_setpose`) are outside this ticket's "sim plant"
      scope and untouched. Zero exported function names changed anywhere.
- [x] Zero `host/robot_radio/` file changes required or made by this
      ticket (ctypes calls are positional; confirmed no Python-side
      change needed). Confirmed: `git status` shows no `host/` diff.
      Three `tests/simulation/unit/*.py` files (`test_physics_world_
      basic.py`, `test_physics_world_body_scrub.py`,
      `test_plant_correctness.py`) needed mechanical updates — each
      embeds a standalone C++ harness that compiles directly against
      `PhysicsWorld.cpp` and called the renamed `setNominalMaxMms`.
      These are test fixtures mirroring a renamed C++ identifier moving
      in lock-step (architecture-update.md Step 1's explicitly
      anticipated case), not a `host/` or Python-identifier change.
- [x] `tests/simulation/unit/test_simset_profile_chunking.py`,
      `test_sim_commands_registry.py`, `test_069_knob_telemetry_sweep.py`
      pass unchanged (`SIMSET`/`SIMGET` wire behavior byte-identical).
      (Path correction: `test_069_knob_telemetry_sweep.py` lives in
      `tests/simulation/system/`, not `unit/`.) All pass, plus
      `test_sim_otos_lever_arm.py`, `test_drive_subsystem.py`,
      `test_ekf_dual_source.py` (the drift-setter consumers) — 63/63
      targeted tests green.
- [x] Full test suite green (`uv run python -m pytest`). **2621 passed,
      0 failed** — matches the pre-ticket baseline exactly.
      `test_golden_tlm_unchanged` passes (byte-identical TLM confirmed).
- [x] `--clean` sim build performed before running tests. `cmake --build
      tests/_infra/sim/build --target clean` (confirmed `libfirmware_
      host.dylib` removed) then a full rebuild (fresh timestamp, zero
      compile errors) before any test run.

## Testing

- **Existing tests to run**: `test_simset_profile_chunking.py`,
  `test_sim_commands_registry.py`, `test_069_knob_telemetry_sweep.py`,
  `test_sim_otos_lever_arm.py`, full default suite.
- **New tests to write**: none required — pure internal rename with an
  explicit wire-key preservation contract already covered by the existing
  `SIMSET`/`SIMGET` registry tests.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Rename `PhysicsWorld`/`SimOdometer`/`SimSetters` internal
identifiers first, verifying after each file that `SimCommands.cpp`'s
`kSimRegistry[]` key-string arguments are untouched (diff the file, not
just grep, since the function name and key string can appear on the same
line). Then update `tests/_infra/sim/*.cpp` parameter names only. Grep
specifically for `trackwidthMm` and diff `SimCommands.cpp` line-by-line at
the end to catch the flagged namespace-collision risk.

**Files to modify**:
- `source/hal/sim/PhysicsWorld.h`, `PhysicsWorld.cpp`
- `source/hal/sim/SimOdometer.h`, `SimOdometer.cpp`
- `source/commands/SimSetters.h`, `SimSetters.cpp`
- `source/commands/SimCommands.cpp` (internal names only — never
  `kSimRegistry[]`'s key-string argument)
- `tests/_infra/sim/sim_api.cpp`, `drive_api.cpp` (parameter names only)

**Testing plan**: `--clean` sim build, then the `SIMSET`/`SIMGET`
registry test tier in isolation, then the full suite.

**Documentation updates**: none in this ticket (ticket 008's final sweep
covers prose docs).
