---
id: '005'
title: "Retarget baking — populate Config::Robot defaults from today's JSON shape"
status: open
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

- [ ] Every existing `gen_boot_config.py` `_require()` call site has a
      corresponding write into the matching `Config::Robot` group struct
      field — checklist against the full function list:
      `otos_boot_config_values`, `vel_gains_for_config`,
      `output_deadband_for_config`, `reversal_dwell_for_config`,
      `trackwidth_for_config`, `rotational_slip_for_config`,
      `rotation_calibration_for_config`, `estimator_config_for_config`,
      `shaper_config_for_config`, `wheel_correction_for_config`,
      `drive_config_for_config`, `wheel_controller_config_for_config`,
      `planner_config_for_config`.
- [ ] Fail-closed behavior is unchanged: a robot JSON missing a required
      key still raises `MissingRobotConfigKeyError` (or equivalent), not
      a silent default.
- [ ] Running the retargeted baking against `tovez.json` (still old
      shape) produces a populated `Config::Robot` with no missing-key
      errors.
- [ ] Compiles under `HOST_BUILD`.

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
