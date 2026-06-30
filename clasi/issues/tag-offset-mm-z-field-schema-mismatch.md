---
status: pending
---

# Stale config goldens: `tag_offset_mm.z` schema + DefaultConfig golden drift (2 baseline test failures)

## Problem

Two host sim tests fail on `master` (pre-existing baseline, NOT caused by the
message-architecture sprints 054-059). They have **two distinct root causes** —
corrected diagnosis below (an earlier note lumped both under `tag_offset_mm.z`):

- `tests/simulation/unit/test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`
  — **schema gap**: a robot config carries `tag_offset_mm.z` (tag mount height) but
  the schema's `tag_offset_mm` object sets `additionalProperties:false` without a `z`
  property. (Original cause, below.)
- `tests/simulation/unit/test_default_config_pin.py::test_default_robot_config_unchanged`
  — **stale GOLDEN snapshot**, NOT a schema issue. `defaultRobotConfig()` differs from
  the pinned golden on two fields: `odomOffY` (golden 4.0 vs actual 3.5 — already stale
  at session start) and `yawRateMax` (golden 35.0 vs actual 70.0). The `yawRateMax`
  delta surfaced when sprint 055's `build.py --clean` **regenerated `DefaultConfig.cpp`
  from the schema SSOT** (commit 4745d81), refreshing a stale committed default; the
  golden file was not updated to match. This is generated-artifact staleness, not a
  behavior regression — and the live tovez robot uses its own per-robot JSON config,
  not this compile-time default. **Fix requires human review**: confirm the SSOT
  values (is `yawRateMax=70` / `odomOffY=3.5` intended?) then refresh the golden via the
  project's pin-update procedure — do NOT rubber-stamp the snapshot.

Both fail with the same `jsonschema.ValidationError`:

```
Failed validating 'additionalProperties' in schema['properties']['vision']['properties']['tag_offset_mm']:
  {'type': 'object', 'additionalProperties': False,
   'properties': {'x': {...}, 'y': {...}, 'yaw_rad': {...}}}
On instance['vision']['tag_offset_mm']:
  {'x': 47.5, 'y': 0.0, 'z': 120.0, 'yaw_rad': 0.0}
```

A robot config (`tovez`) carries `tag_offset_mm.z = 120.0` (the mounted tag's
height), but the schema's `tag_offset_mm` object declares only `x`/`y`/`yaw_rad`
with `additionalProperties: false`, so the `z` key is rejected.

## Decision needed / likely fix

Determine the source of truth: either the **schema** is missing the legitimate `z`
height property (add `z: {type: number}` to `tag_offset_mm` in
`data/robots/robot_config.schema.json` and regenerate), **or** the `z` key in the
robot config(s) is stale and should be removed. The `z=120.0` height looks
intentional (tag mounting height), so adding `z` to the schema is the more likely
correct fix — but confirm whether any firmware/host code reads `tag_offset_mm.z`
before deciding.

## Notes

Found during sprint 054 validation. These 2 failures form a known-baseline that
subsequent sprint reviews should subtract until fixed. Severity: minor (schema
validation only; does not affect motion). Keep separate from the message-based
architecture program.
</content>
