---
id: '004'
title: 'Proto-generated message field renames: drivetrain/motor/planner proto and
  codegen'
status: done
use-cases:
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Proto-generated message field renames: drivetrain/motor/planner proto and codegen

## Description

Rename unit-suffixed snake_case fields in the proto-generated internal
message structs (`msg::PlannerConfig`, `msg::DrivetrainConfig`,
`msg::MotorConfig`) and regenerate their C++ headers. These `.proto`-
defined messages are **internal, never wire-transmitted** (confirmed by
sprint 070's own architecture reading and reconfirmed by this sprint's
planning pass) — this ticket has zero wire-compatibility concerns of its
own, but it must re-confirm the exact current field list against the live
`.proto` source before editing (`architecture-update.md` Open Question 5
— the planning pass read these files at grep/summary level, not full
text, so a field this document didn't enumerate may also carry a unit
suffix and must be caught and renamed too).

This ticket depends on ticket 002 because the proto fields are a
**projection of `RobotConfig` fields by name** through
`scripts/gen_messages.py`'s literal mapping table (e.g.
`("DrivetrainConfig", "mm_per_deg_l"): "RobotConfig::mmPerDegL"`) —
ticket 002 must land first so this ticket's mapping-table update
references the *new* `RobotConfig::` names, not the old ones.

Fields renamed (confirmed list; re-verify against the live `.proto`
source at implementation time per Open Question 5):
- `protos/drivetrain.proto`: `mm_per_deg_l` → `travel_calib_l`,
  `mm_per_deg_r` → `travel_calib_r` (and any `half_track_mm`/
  `half_wheelbase_mm`/`arrive_tol_mm` fields present).
- `protos/motor.proto`: `mm_per_deg` → `travel_calib`.
- `protos/planner.proto`: re-check for any unit-suffixed field not
  enumerated by this planning pass.
- `scripts/gen_messages.py`: the field-name mapping table's literal pairs
  updated to match both the new proto field name and the new
  `RobotConfig::` field name from ticket 002.
- `source/messages/*.h`: regenerated via `scripts/gen_messages.py`.
- `source/superstructure/PlannerConfig.{h,cpp}`, `source/subsystems/drive/
  DriveConfig.cpp`: accessor call sites updated to the new proto field
  names.
- `docs/design/message-inventory.md`: regenerated in the same ticket as
  the `.proto` edits (mirrors the golden-fixture-same-ticket discipline
  sprint 068 established for TLM — `architecture-update.md` Migration
  Concerns).

See `architecture-update.md` Step 5 ("004 — Proto-generated message
renames"), Decision 5 (derived-unit naming — `travel_calib*` mirrors
`wheelTravelCalib*`), Open Question 5; `usecases.md` SUC-004.

## Acceptance Criteria

- [x] `protos/drivetrain.proto`, `protos/motor.proto`, `protos/planner.proto`
      re-read in full at implementation time to confirm the complete field
      list (not assumed from `architecture-update.md`'s grep-level
      summary); every unit-suffixed field found (not just the enumerated
      ones) is renamed. (Full re-read confirmed `trackwidth`/`half_track`/
      `half_wheelbase`/`arrive_tol` already carry no unit suffix in the
      live `.proto` source — a prior sprint's own rename, not this one's
      concern. One field beyond the planning pass's enumerated list was
      found and renamed: `drivetrain.proto`'s repeated
      `mm_per_deg_wheel` → `travel_calib_wheel` (the mecanum per-wheel
      calibration array, mirrored to `RobotConfig::{wheelTravelCalibFR,
      wheelTravelCalibFL,wheelTravelCalibBR,wheelTravelCalibBL}`).
      `planner.proto` re-confirmed to carry no unit-suffixed field at
      all — no edit needed there.)
- [x] `mm_per_deg_l`/`mm_per_deg_r` → `travel_calib_l`/`travel_calib_r`;
      `mm_per_deg` → `travel_calib`.
- [x] `scripts/gen_messages.py`'s mapping table updated: every literal
      pair references the new proto field name AND the new
      `RobotConfig::` field name (from ticket 002). (Ticket 002 never
      touched `gen_messages.py` — its mapping table is doc-only, used by
      `--emit-inventory` — so on top of the 4 renamed-proto-field rows,
      13 further rows across `DrivetrainConfig`/`MotorConfig`/
      `PlannerConfig`/`PortConfig`/`LineSensorConfig`/`ColorSensorConfig`
      still pointed at ticket 002's pre-rename `RobotConfig::` names
      (`trackwidthMm`, `halfTrackMm`, `halfWheelbaseMm`, `minWheelMms`,
      `rotationOffsetDeg[Neg]`, `odomYawDeg`, `lagOtosMs`, `arriveTolMm`,
      `minSpeedMms`, `lagPortsMs`, `lagLineMs`, `lagColorMs`) — all
      updated to ticket 002's current field names. `DesiredState::`/
      `OutputState::` doc references in the same table (ticket 003's
      naming domain) were left untouched — out of this ticket's stated
      scope (`RobotConfig::`, from ticket 002); ticket 003's own
      completion note deferred all doc-comment updates to ticket 008.)
- [x] `source/messages/*.h` regenerated (via `scripts/gen_messages.py`)
      and committed — no stale generated header referencing an old field
      name. (`drivetrain.h`, `motor.h` diffed as expected — field names,
      getters, setters renamed mechanically; `bridges.h`/`common.h`/
      `gripper.h`/`planner.h`/`ports.h`/`sensors.h` regenerated
      byte-identical, confirming no unintended change elsewhere.)
- [x] `source/superstructure/PlannerConfig.{h,cpp}`,
      `source/subsystems/drive/DriveConfig.cpp` accessor call sites
      updated; `grep -rn "mm_per_deg\|MmPerDeg" source/ protos/ scripts/`
      returns zero results. (`PlannerConfig.{h,cpp}` reference no
      `mm_per_deg`-family field — confirmed by direct read, no edit
      needed. `DriveConfig.cpp`: `cfg.setMmPerDegL/R(...)` →
      `cfg.setTravelCalibL/R(...)`. The stated grep also covers
      `scripts/`: `scripts/gen_default_config.py` held two local Python
      variables spelled `mm_per_deg_l`/`mm_per_deg_r` — unrelated to the
      proto codegen chain but matching the literal grep pattern — renamed
      to `wheel_travel_calib_l`/`wheel_travel_calib_r` (their source JSON
      keys `mm_per_wheel_deg_left`/`_right` are untouched, per the
      Wire-Compatibility Exclusion Table's per-robot-JSON exclusion).
      Regenerating `DefaultConfig.cpp` to spot-check this rename surfaced
      a **pre-existing, unrelated drift**: `gen_default_config.py`'s
      template still emits `p.turnThresholdMm`/`p.doneTolMm` (fields
      sprint 070 Decision 4 already deleted from `RobotConfig`) and a
      stale `TLM_FIELD_ALL` vs. the committed `0x1FF` literal — the
      checked-in `DefaultConfig.cpp` was hand-patched post-070/068 without
      the generator being updated to match. This is a pre-existing,
      out-of-scope generator/output mismatch unrelated to this ticket;
      `DefaultConfig.cpp` was left untouched (reverted after the
      spot-check) to avoid reintroducing deleted fields into a compiling
      header. Confirmed via the full grep: zero results across `source/`,
      `protos/`, `scripts/`.)
- [x] `docs/design/message-inventory.md` regenerated in this same ticket,
      consistent with the new field names. (Coverage unchanged at
      204/204 fields mapped, 47 new, 0 missing — pure rename, no field
      count change.)
- [x] `G`/`RT`/`TURN` command behavior byte-identical: arc geometry,
      pre-rotate threshold, and arrival tolerance produce the same numeric
      results as pre-ticket for the same input sequence. (Confirmed by
      the full suite's unchanged pass count and the golden-TLM byte-exact
      fixture test passing unmodified.)
- [x] `tests/simulation/unit/test_pursuit_arc_steering.py`,
      `test_planner_subsystem_smoke.py`, `test_rt_slip.py`, and the
      `G`/`RT`/`TURN` system-test tier pass with unchanged numeric
      assertions. (69 tests passed in the targeted run; full suite run
      below includes the system-test tier.)
- [x] Full test suite green (`uv run python -m pytest`). (2621 passed,
      0 failed — same count as the post-002/003 baseline on this branch.)
- [x] `--clean` sim build after the `.proto` edits and `gen_messages.py`
      re-run, before the test run (deployment sequencing per
      `architecture-update.md` Migration Concerns). (`cmake --build
      tests/_infra/sim/build --target clean` then a full rebuild — all
      ~50 translation units recompiled, zero compile errors on first
      attempt, fresh `.dylib`.)

## Testing

- **Existing tests to run**: `test_pursuit_arc_steering.py`,
  `test_planner_subsystem_smoke.py`, `test_rt_slip.py`, `G`/`RT`/`TURN`
  system-test tier, full default suite.
- **New tests to write**: none required — pure rename; existing
  arc/pursuit tests already cover the numeric behavior this ticket must
  preserve.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Read the full current text of `protos/drivetrain.proto`,
`protos/motor.proto`, `protos/planner.proto` first (not the architecture
doc's summary) to get the authoritative field list. Rename each
unit-suffixed field, update `scripts/gen_messages.py`'s mapping table to
reference both sides' new names, regenerate `source/messages/*.h`, update
the two accessor-call-site files, then regenerate
`docs/design/message-inventory.md`.

**Files to modify**:
- `protos/drivetrain.proto`
- `protos/motor.proto`
- `protos/planner.proto` (if a unit-suffixed field is found)
- `scripts/gen_messages.py`
- `source/messages/*.h` (regenerated, not hand-edited)
- `source/superstructure/PlannerConfig.h`, `PlannerConfig.cpp`
- `source/subsystems/drive/DriveConfig.cpp`
- `docs/design/message-inventory.md`

**Testing plan**: `--clean` sim build after regenerating headers
(required — stale incremental builds on `/Volumes` are a known project
gotcha), then the arc/pursuit test tier in isolation, then the full suite.

**Documentation updates**: `docs/design/message-inventory.md` regenerated
in this ticket (not deferred to 008, since it documents exactly what this
ticket changes).
