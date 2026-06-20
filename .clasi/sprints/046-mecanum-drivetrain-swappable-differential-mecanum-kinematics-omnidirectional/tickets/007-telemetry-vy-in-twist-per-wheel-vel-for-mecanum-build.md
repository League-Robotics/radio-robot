---
id: "007"
title: "Telemetry: vy in twist=, per-wheel vel= for mecanum build"
status: open
use-cases:
  - SUC-001
  - SUC-003
  - SUC-004
  - SUC-005
depends-on:
  - "046-006"
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 046-007: Telemetry: vy in twist=, per-wheel vel= for mecanum build

## Description

Extend the telemetry layer to surface the new mecanum-specific state in two
TLM fields:

1. **`twist=`**: add `vy=<value>` alongside the existing `vx=` and `omega=`
   (mecanum build only; differential format unchanged).
2. **`vel=`**: extend from 2 wheels to 4 wheels in the mecanum build
   (`vel=vFR,vFL,vBR,vBL`); differential format (`vel=vL,vR`) unchanged.

The golden-TLM oracle for the differential build must be byte-identical.

## Approach

### 1. Locate the telemetry emit code

Find the `twist=` and `vel=` emit sites. These are likely in
`source/app/TelemetryHandler.cpp` or `source/robot/Robot.cpp` where
`STREAM`/`SNAP` TLM is assembled. Read these files before implementing.

### 2. Extend twist= (mecanum build)

Under `#ifdef ROBOT_DRIVETRAIN_MECANUM`, change the `twist=` format:

```
// Differential (unchanged):
twist= vx=123.4 omega=0.342

// Mecanum:
twist= vx=123.4 vy=-12.1 omega=0.342
```

`vy` value comes from `s.inputs.fusedVy` (written by Odometry in T6).

The field order must be `vx`, `vy`, `omega` (stable order for easy parsing
by the host). Decimal precision: match the existing `vx`/`omega` format (1
decimal place or `%g` — follow whatever format the existing emit uses).

### 3. Extend vel= (mecanum build)

Under `#ifdef ROBOT_DRIVETRAIN_MECANUM`, change the `vel=` format:

```
// Differential (unchanged):
vel=123.4,118.2

// Mecanum (4 wheels: FR, FL, BR, BL):
vel=123.4,118.2,122.1,119.8
```

Values come from `s.inputs.velMms[0..3]` (the 4-wheel velocity array from T5).

### 4. Golden-TLM oracle guard

The `tests/simulation/test_tlm_oracle.py` test (or equivalent) records the
exact TLM byte output for the differential build. Since all new `#ifdef` blocks
are mecanum-only and the differential sim uses the differential struct/code
path, this oracle should not change. Confirm after implementation.

### 5. Host-side TLM parser (robot_config.py / host utilities)

If the host has a TLM parser that splits `twist=` tokens, add `vy` handling
(optional field — parser must not crash on the existing 2-field format for
differential robots). Check `host/robot_radio/` for any TLM parsing code.

## Files to Modify

- `source/app/TelemetryHandler.cpp` (or wherever `twist=`/`vel=` are emitted)
- `host/robot_radio/` TLM parser (if one exists that parses `twist=`/`vel=`)

## Acceptance Criteria

- [ ] `uv run --with pytest python -m pytest tests/simulation -q` reports `2093 passed`.
- [ ] Golden-TLM oracle test (`tests/simulation/test_tlm_oracle.py`) is unchanged.
- [ ] Differential TLM `twist=` format is byte-identical to pre-sprint (no `vy=` in differential output).
- [ ] Differential TLM `vel=` format is byte-identical to pre-sprint (`vel=vL,vR` only).
- [ ] Mecanum build: `SNAP` response includes `vy=<value>` in the `twist=` field.
- [ ] Mecanum build: `SNAP` response `vel=` field has 4 comma-separated values (`FR,FL,BR,BL` order).
- [ ] Host TLM parser (if any) handles the new `vy=` token gracefully on both robot types.

## Testing

- **Regression gate**: `uv run --with pytest python -m pytest tests/simulation -q`
- **Oracle gate**: `tests/simulation/test_tlm_oracle.py` unchanged.
- **Mecanum TLM verification**: on hardware (T8), send `SNAP` during strafe and confirm
  `vy=` is non-zero in the response.
- **New tests**: a sim-level test that verifies `twist=` format in mecanum build
  (if the sim harness supports TLM output inspection with `ROBOT_DRIVETRAIN_MECANUM`).
- **Verification command**: `uv run --with pytest python -m pytest tests/simulation -q`

## Implementation Notes

- Read the existing `twist=` and `vel=` emit code carefully — format strings,
  field delimiters, and number formatting must match the wire protocol so host
  parsers work correctly.
- If `TLM_FIELD_TWIST` (bit 5 in `tlmFields`) already controls `twist=` emission,
  no new TLM bit is needed — just extend the body of the `twist=` block under
  the mecanum guard.
- Check if the existing `SNAP` test in the sim suite asserts on the exact `twist=`
  string — if so, add a mecanum-specific variant of that test.
