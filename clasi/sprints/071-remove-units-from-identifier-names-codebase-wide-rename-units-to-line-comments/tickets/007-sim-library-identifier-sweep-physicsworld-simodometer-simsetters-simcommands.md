---
id: '007'
title: 'Sim-library identifier sweep: PhysicsWorld, SimOdometer, SimSetters, SimCommands'
status: open
use-cases: [SUC-006]
depends-on: ['002']
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

- [ ] `source/hal/sim/PhysicsWorld.{h,cpp}`: `_driftPerTickMm`→
      `_driftPerTick`, `sigmaMm`→`sigma`, `encoderScaleErrL/R` and peers
      renamed; each carries a `// [unit]` comment.
- [ ] Internal `simsetters::` function names mirroring `SIMSET` keys
      renamed (e.g. `otosLinDriftMmS`→`otosLinearDrift`); their
      corresponding `kSimRegistry[]` key-string arguments in
      `SimCommands.cpp` are byte-identical to pre-ticket (diffed).
- [ ] Every `kSimRegistry[]` key string listed in the Wire-Compatibility
      Exclusion Table's `SIMSET`/`SIMGET` row is unchanged (diffed against
      pre-ticket source) — including `trackwidthMm` (the sim key, distinct
      from the already-renamed `RobotConfig::trackwidth` C++ field and the
      excluded `"tw"` `SET` key).
- [ ] `source/hal/sim/SimOdometer.{h,cpp}`, `source/commands/
      SimSetters.{h,cpp}`: no remaining internal unit-suffixed identifier
      outside the preserved key strings.
- [ ] `tests/_infra/sim/sim_api.cpp`, `drive_api.cpp`: parameter names
      renamed per convention; exported `extern "C"` function names
      byte-identical to pre-ticket (already unit-free).
- [ ] Zero `host/robot_radio/` file changes required or made by this
      ticket (ctypes calls are positional; confirmed no Python-side
      change needed).
- [ ] `tests/simulation/unit/test_simset_profile_chunking.py`,
      `test_sim_commands_registry.py`, `test_069_knob_telemetry_sweep.py`
      pass unchanged (`SIMSET`/`SIMGET` wire behavior byte-identical).
- [ ] Full test suite green (`uv run python -m pytest`).
- [ ] `--clean` sim build performed before running tests.

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
