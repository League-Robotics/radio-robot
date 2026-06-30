---
status: pending
---

# `tag_offset_mm.z` rejected by robot_config schema (2 pre-existing test failures)

## Problem

Two host sim tests fail on `master` (pre-existing, unrelated to sprint 054):

- `tests/simulation/unit/test_default_config_pin.py::test_default_robot_config_unchanged`
- `tests/simulation/unit/test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`

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
