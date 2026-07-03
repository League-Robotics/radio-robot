---
id: '002'
title: 'RobotConfig field renames: strip unit suffixes from calibration/timing/geometry
  fields'
status: open
use-cases: [SUC-001]
depends-on: ['001']
github-issue: ''
issue: remove-units-from-identifier-names.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# RobotConfig field renames: strip unit suffixes from calibration/timing/geometry fields

## Description

Rename `RobotConfig`'s unit-suffixed fields (the 7 FIXME-flagged fields
plus their peers) in `source/types/Config.h`, propagating the rename
through the field's build-time codegen/consistency chain:
`source/robot/DefaultConfig.cpp`, `source/robot/ConfigRegistry.cpp` (field
argument only), and `data/robots/robot_config.schema.json`
(`firmware.field` values only). Per `architecture-update.md` Decision 3,
this is treated as **one coordinated edit per field** across all four
files — a `Config.h`-only rename would leave the codegen chain broken at
compile time.

**HARD CONTRACT: wire keys never change.** `ConfigRegistry.cpp`'s `CFG_*`
macros bind the wire key string (argument 1) and the C++ field name
(argument 2) as two independent literals — `CFG_FI("tw", trackwidthMm)`
becomes `CFG_FI("tw", trackwidth)`. Only argument 2 changes. The schema's
`"set_key"` values are untouched; only `"field"` values change. See
`architecture-update.md`'s Wire-Compatibility Exclusion Table.

**The one flagged collision risk**: `minWheelMms` is both the current C++
field name AND the wire key string (the one row in the whole registry
where the two literals are spelled identically). `CFG_F("minWheelMms",
minWheelMms)` becomes `CFG_F("minWheelMms", minWheelSpeed)` — rename the
field (2nd argument) only; the wire key string `"minWheelMms"` (1st
argument) is preserved verbatim. A careless whole-word find/replace across
the file would incorrectly also rename the wire key — this must not
happen.

Fields renamed (7 FIXME'd + peers, from `architecture-update.md` Step 5):

- `trackwidthMm` → `trackwidth`
- `minWheelMms` → `minWheelSpeed` (wire key `"minWheelMms"` stays)
- `rotationOffsetDeg` → `rotationOffset`
- `rotationOffsetDegNeg` → `rotationOffsetNeg` (untagged sibling of the
  FIXME'd field)
- `arriveTolMm` → `arriveTolerance`
- `tlmPeriodMs` → `tlmPeriod`
- `lagOtosMs` → `lagOtos`
- `halfTrackMm` → `halfTrack`
- `lagLineMs` → `lagLine`
- `lagColorMs` → `lagColor`
- `lagPortsMs` → `lagPorts`
- `minSpeedMms` → `minSpeed`
- `tickMs` → `tick`
- `sTimeoutMs` → `sTimeout`
- `controlPeriodMs` → `controlPeriod`
- `odomYawDeg` → `odomYaw`
- `halfWheelbaseMm` → `halfWheelbase`
- `mmPerDegL`/`mmPerDegR` → `wheelTravelCalibL`/`wheelTravelCalibR`
  (derived-unit rename per Decision 5 — NOT a bare suffix-strip, see
  `docs/coding-standards.md` from ticket 001)
- `mmPerDegFR`/`mmPerDegFL`/`mmPerDegBR`/`mmPerDegBL` →
  `wheelTravelCalibFR`/`wheelTravelCalibFL`/`wheelTravelCalibBR`/
  `wheelTravelCalibBL`

All `FIXME` comments in `Config.h` referencing this issue are removed.
Every renamed field gains a leading `// [unit]` comment per
`docs/coding-standards.md`.

Direct consumers of the renamed fields (mechanical updates, same file set
as each field's existing readers): `Planner`/`PlannerBegin.cpp`,
`Drive.cpp`, `MotorController`, `Motor.cpp`, `OtosSensor`,
`BodyVelocityController`, `NezhaHAL`. Test fixtures that mirror
`RobotConfig` field names by construction (`tests/_infra/
default_config_golden.json`, `tests/simulation/unit/
test_config_registry.py`, mock `RobotConfig` test-double classes such as
in `test_body_velocity_controller.py`) are updated mechanically to keep
the suite green.

Per `architecture-update.md` Open Question 2: this is flagged as the
largest ticket in the sprint by file-touch count. If it does not fit one
focused session, split by `Config.h` section (e.g. "OTOS/rotation
calibration fields" vs. "timing/lag fields") while preserving the
four-file-per-field coordination rule within each split — but attempt it
as one ticket first.

See `architecture-update.md` Step 5 ("002 — RobotConfig field renames"),
the Wire-Compatibility Exclusion Table, Decisions 2, 3, and 5;
`usecases.md` SUC-001.

## Acceptance Criteria

- [ ] `source/types/Config.h`: all fields listed above renamed; each
      carries a `// [unit]` comment per `docs/coding-standards.md`.
- [ ] All `FIXME` markers referencing this issue removed from `Config.h`
      (`grep -rn "FIXME" source/types/Config.h` returns zero results).
- [ ] `source/robot/DefaultConfig.cpp`: every `p.<oldField>` assignment and
      every `ov('<oldField>', ...)` call updated to the new field name.
- [ ] `source/robot/ConfigRegistry.cpp`: every `CFG_*` row's field-name
      (second) argument updated to the new name; the wire-key (first)
      argument is byte-identical to pre-ticket for every row, including
      `minWheelMms` (diffed against pre-ticket source).
- [ ] `data/robots/robot_config.schema.json`: every `"firmware": {"field":
      "<oldField>", ...}` value updated to `"<newField>"`; every
      `"set_key"` value is byte-identical to pre-ticket.
- [ ] Direct consumers updated: `Planner`/`PlannerBegin.cpp`, `Drive.cpp`,
      `MotorController`, `Motor.cpp`, `OtosSensor`,
      `BodyVelocityController`, `NezhaHAL` — no reference to an old field
      name remains anywhere in `source/`.
- [ ] `tests/_infra/default_config_golden.json` regenerated — field-name
      keys change, values byte-identical.
- [ ] `tests/simulation/unit/test_config_registry.py` and any mock
      `RobotConfig` test-double (e.g. in `test_body_velocity_controller.py`)
      updated to the new field names; assertions on *values* unchanged.
- [ ] `SET`/`GET` behavior is byte-identical for every affected key
      (spot-check `SET tw=`, `SET minWheelMms=`, `SET arriveTol=`
      round-trip through `GET`).
- [ ] Full test suite green (`uv run python -m pytest`), baseline 2620
      passed, 0 failed (count may shift slightly if this ticket's own test
      updates add/rename test functions — no *new* failures).
- [ ] `--clean` sim build performed before running tests (project
      knowledge: stale incremental builds on `/Volumes` — build banners
      lie).

## Testing

- **Existing tests to run**: `tests/simulation/unit/
  test_config_registry.py`, `test_body_velocity_controller.py`, any
  `SET`/`GET` command tests, full default suite.
- **New tests to write**: none required beyond mechanically updating
  existing fixtures — this is a pure rename, no new behavior to cover. If
  a spot-check `SET`/`GET` round-trip test doesn't already exist for
  `minWheelMms` specifically (the collision-risk key), add one asserting
  the wire key still round-trips correctly post-rename.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: For each field, perform the four-file coordinated edit
(Config.h → DefaultConfig.cpp → ConfigRegistry.cpp → schema.json) in one
pass, then update direct consumers. Grep for the old name across
`source/` and `tests/` after each field to confirm no stray reference
remains before moving to the next field. Handle `minWheelMms` last and by
hand (not via a blanket find/replace) since it is the one field/key
spelling collision.

**Files to modify**:
- `source/types/Config.h`
- `source/robot/DefaultConfig.cpp`
- `source/robot/ConfigRegistry.cpp`
- `data/robots/robot_config.schema.json`
- `source/superstructure/Planner.cpp`, `source/control/PlannerBegin.cpp`
- `source/subsystems/drive/Drive.cpp`
- `source/control/MotorController.cpp`
- `source/hal/real/Motor.cpp`
- `source/hal/real/OtosSensor.cpp`
- `source/control/BodyVelocityController.cpp`
- `source/robot/NezhaHAL.cpp`
- `tests/_infra/default_config_golden.json`
- `tests/simulation/unit/test_config_registry.py`
- `tests/simulation/unit/test_body_velocity_controller.py` (mock
  `RobotConfig` test-double kwargs)

**Testing plan**: `--clean` sim build, then `test_config_registry.py` and
`test_body_velocity_controller.py` in isolation, then the full suite.

**Documentation updates**: none in this ticket (prose docs quoting these
field names are ticket 008's final sweep).
