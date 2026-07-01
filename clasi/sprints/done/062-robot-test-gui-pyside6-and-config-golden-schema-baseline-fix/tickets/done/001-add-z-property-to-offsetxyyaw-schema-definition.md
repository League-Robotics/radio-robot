---
id: '001'
title: Add z property to OffsetXYYaw schema definition
status: done
use-cases:
- SUC-016
depends-on: []
issue: tag-offset-mm-z-field-schema-mismatch.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 001 — Add z property to OffsetXYYaw schema definition

## Description

`data/robots/robot_config.schema.json` defines a shared `OffsetXYYaw` type (in
`$defs`) used by both `vision.tag_offset_mm` and `geometry.odometry_offset_mm`.
The type declares `additionalProperties: false` with only `x`, `y`, and `yaw_rad`
properties. The `tovez` robot config carries `tag_offset_mm.z = 120.0` (the tag
mount height), which is a legitimate field that the schema incorrectly rejects.

This ticket adds `z: {type: number}` to `OffsetXYYaw`, fixing the schema gap
without relaxing the schema against genuinely unknown keys.

**Root cause A** of the two baseline test failures (see issue). Root cause B
(stale golden) is addressed in ticket 002.

## Acceptance Criteria

- [x] `$defs.OffsetXYYaw.properties` in `data/robots/robot_config.schema.json`
  gains a `z` property: `{"type": "number"}`.
- [x] `additionalProperties: false` is retained on `OffsetXYYaw`.
- [x] `tests/simulation/unit/test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`
  passes.
- [x] No existing robot config files (`tovez.json`, `togov.json`) require
  modification.
- [x] `uv run python -m pytest tests/simulation` passes (no regressions).
- [x] Confirm no host code reads `tag_offset_mm.z` expecting its absence to be
  meaningful (grep check — expect zero hits for code that branches on missing z).

## Implementation Plan

### Approach

Single-file schema edit. The `OffsetXYYaw` definition is at the bottom of
`data/robots/robot_config.schema.json` in the `$defs` section. Add `z` alongside
`x`, `y`, `yaw_rad`.

### Files to modify

- `data/robots/robot_config.schema.json` — add `"z": {"type": "number"}` to
  `$defs.OffsetXYYaw.properties`.

### Files to verify (no change expected)

- `data/robots/tovez.json` — already carries `z: 120.0`; should now validate.
- `data/robots/togov.json` — does not carry `z`; continues to validate (optional
  field).
- `host/robot_radio/config/` — grep for any code that reads `tag_offset_mm` and
  treats absence of `z` specially.

### Testing plan

Run `uv run python -m pytest tests/simulation/unit/test_robot_config.py -v`.
Confirm `test_tovez_validates_against_schema` passes. Then run the full
simulation gate to confirm no regressions.

### Documentation updates

None required. The schema is self-documenting; optionally add a `description`
field to the `z` property: `"Tag or offset mount height in mm (optional)."`.
