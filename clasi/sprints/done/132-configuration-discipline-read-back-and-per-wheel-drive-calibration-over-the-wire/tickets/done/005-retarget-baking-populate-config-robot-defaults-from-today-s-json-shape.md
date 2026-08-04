---
id: '005'
title: "Retarget baking \u2014 populate Config::Robot defaults from today's JSON shape"
status: done
use-cases:
- SUC-002
depends-on:
- '002'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Retarget baking — populate Config::Robot defaults from today's JSON shape

## Description

Build the `loadBaked()`-equivalent generation path that populates a
`Config::Robot` instance from the active robot JSON, which is still in
its **old** 13-section shape at this point in the sprint (the reshape is
ticket 017 — see sprint.md Design Rationale Decision 2). This absorbs
`gen_boot_config.py`'s existing `_require()`/fail-closed field-mapping
logic (unchanged in behavior/strictness) but retargets its OUTPUT from
the old hand-declared `boot_config.h` structs onto the new generated
`Config::Robot` group structs from ticket 002. Every `_require(cfg,
"control", "wheel_gain_left_accel")`-style call keeps reading the OLD
JSON path; only what struct/field it writes into changes.

## Acceptance Criteria

- [x] Every existing `gen_boot_config.py` `_require()` call site has a
      corresponding write into the matching `Config::Robot` group struct
      field — checklist against the full function list:
      `otos_boot_config_values`, `vel_gains_for_config`,
      `output_deadband_for_config`, `reversal_dwell_for_config`,
      `trackwidth_for_config`, `rotational_slip_for_config`,
      `rotation_calibration_for_config`, `estimator_config_for_config`,
      `shaper_config_for_config`, `wheel_correction_for_config`,
      `drive_config_for_config`, `wheel_controller_config_for_config`,
      `planner_config_for_config`.
- [x] Fail-closed behavior is unchanged: a robot JSON missing a required
      key still raises `MissingRobotConfigKeyError` (or equivalent), not
      a silent default.
- [x] Running the retargeted baking against `tovez.json` (still old
      shape) produces a populated `Config::Robot` with no missing-key
      errors.
- [x] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: `gen_boot_config.py`'s existing pytest
  coverage (fail-closed `_require()` behavior, `travel_calib_for_ports`,
  `fwd_sign_for_ports`, etc.) continues to pass against the retargeted
  output shape.
- **New tests to write**: a test asserting the populated `Config::Robot`'s
  field values match `tovez.json`'s raw values for a representative
  sample across all 7 groups.
- **Verification command**: `uv run python -m pytest <gen_boot_config
  test path> -q`.

## Implementation Plan

**Approach**: This is primarily a retargeting of existing, working
logic — keep every `_require()` call's JSON path exactly as-is, change
only the assignment target type/field name to match `Config::Robot`'s new
group struct layout.

**Files to modify**: `src/scripts/gen_boot_config.py`.

**Testing plan**: as above.

**Documentation updates**: `gen_boot_config.py`'s module docstring
updated to note it now targets `Config::Robot`'s generated groups, not
the old hand-declared `boot_config.h` structs.

## Completion Notes

**ADDITIVE, not a retarget-in-place, and this is the explicit judgment
call the dispatch instructions asked to be stated:** `boot_wiring.cpp`/
`boot_calibration.cpp`/`main.cpp` (none in this ticket's scope) still call
every pre-existing `default*Config()`/`default*BootConfig()` function
every boot — confirmed by grep before touching anything. Those old
functions' own signatures don't even correspond 1:1 to the new grouped
schema (e.g. `defaultMotorConfigs()` fills a 4-port array with two inert
placeholder ports; the new `Motors` group has only left/right), so
"retargeting in place" was not just risky but structurally impossible
without also rewriting the three callers — out of this ticket's scope and
a guaranteed `HOST_BUILD` break, which acceptance criterion 4 forbids.
Instead, `gen_boot_config.py` gained seven new, distinctly-named,
no-argument functions — `Config::defaultGeometryGroup()`/
`defaultMotorsGroup()`/`defaultDriveGroup()`/`defaultWheelControlGroup()`/
`defaultPlannerGroup()`/`defaultOtosGroup()`/`defaultEstimatorGroup()`,
one per `msg::ConfigGroupTarget` — declared in `boot_config.h` (which now
also `#include`s the generated `messages/robot_config.h`) alongside,
never instead of, the pre-existing ones. Every one of the 13 functions in
the acceptance criterion's checklist (plus the two soft `_get()` ones,
`travel_calib_for_ports()`/`fwd_sign_for_ports()`, whose drive-pair-only
slice now also feeds `Motors`) is read into exactly the same JSON path,
same `_require()` strictness, just written into the new struct. Both
families are baked from one robot JSON into two different C++ shapes for
one sprint — a deliberate, scoped duplication ticket 006
("Configurator owns `Config::Robot`") resolves by retargeting
`RobotGraph`'s composition root onto `Configurator::loadBaked()` (which
calls the new functions), after which the old family has no callers left
and a later cleanup ticket can delete it.

**The old-shape-JSON → new-grouped-struct bridge is permanent, not a
sprint-scoped shim.** Every new `default*Group()` function reads
`control.*`/`calibration.*`/`geometry.*` (today's 13-section shape) via
the exact same `_require()`/`_get()` dotted-path calls the pre-existing
functions already used — this is the generator's ordinary job (map a JSON
path to a typed field), not new machinery invented to keep the tree green
between tickets. Ticket 017's JSON reshape does not remove this bridge —
it moves the JSON's own section boundaries to match `Config::Robot`'s
grouping, which only changes what dotted path each `_require()` call
reads.

**The 3 known `test_gen_boot_config_planner.py` failures**
(`test_planner_config_for_config_reads_tovez_json`,
`test_planner_config_for_config_raises_with_no_robot_config`,
`test_generate_emits_default_planner_limits_byte_identical_to_pre_ticket_literals`)
are untouched in identity and count by this ticket — verified by diffing
the failure set with and without this ticket's changes stashed out
(`git stash`/`pytest`/`git stash pop`), same 3 tests fail identically
either way. Root cause unchanged from the sprint's own baseline note:
`tovez.json`'s `planner.heading_hold_gain` is `0.0` (130-011 zeroed the
limit-cycle-prone gain); the test's own `_EXPECTED_RAW` dict still expects
the pre-130-011 `2.0`. Not this ticket's to fix.

**Left for later tickets, deliberately not touched:**
`boot_calibration.cpp:88`'s hardcoded `Drive::kDutyPerSpeed` override
(ignoring the JSON's `duty_per_speed_left/right`, "MEASURED, NOT
CONFIGURED" per the 2026-07-31 stakeholder decision) is untouched — the
new `defaultDriveGroup()` bakes the JSON's real
`duty_per_speed_left/right` values into `Config::Robot.drive` correctly,
but nothing reads `Config::Robot.drive` yet (ticket 009,
"install(DRIVE)/install(WHEEL_CONTROL)... per-wheel Stage-A correction
live over the wire," is where that reversal — if it happens at all —
would land). Not in this ticket's acceptance criteria; noted, not acted
on.

**Verification performed**: `uv run python -m pytest
src/tests/unit/test_gen_boot_config_robot_groups.py
src/tests/unit/test_gen_boot_config_otos.py
src/tests/unit/test_gen_boot_config_planner.py
src/tests/sim/unit/test_gen_boot_config_fwd_sign.py
src/tests/sim/unit/test_gen_boot_config_required_keys.py -q` → 84 passed,
3 known-pre-existing failures (identical with/without this ticket's
diff). `HOST_BUILD` verified via `just build-sim`
(`cmake -S src/sim -B src/sim/build -DROBOT_RUN_MODE=SIM && cmake --build
src/sim/build --parallel`) → links cleanly. Per the sprint's own Test
Strategy, the full default test collection was deliberately NOT run for
this ticket (mid-sprint breakage across other, unrelated files is
expected and accepted — see sprint.md).
