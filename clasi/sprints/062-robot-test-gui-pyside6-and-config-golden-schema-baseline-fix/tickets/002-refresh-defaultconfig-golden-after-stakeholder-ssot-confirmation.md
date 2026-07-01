---
id: '002'
title: Refresh DefaultConfig golden after stakeholder SSOT confirmation
status: open
use-cases:
- SUC-017
depends-on:
- '001'
issue: tag-offset-mm-z-field-schema-mismatch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 002 — Refresh DefaultConfig golden after stakeholder SSOT confirmation

## Description

`tests/simulation/unit/test_default_config_pin.py::test_default_robot_config_unchanged`
fails because the pinned golden snapshot diverges from `defaultRobotConfig()` on
two fields:

| Field | Golden value | Actual (live SSOT) |
|-------|-------------|---------------------|
| `odomOffY` | 4.0 | 3.5 |
| `yawRateMax` | 35.0 | 70.0 |

Sprint 055's `build.py --clean` regenerated `DefaultConfig.cpp` from the schema
SSOT (commit 4745d81), refreshing a stale committed default. The golden file was
not updated to match. This is generated-artifact staleness, not a behavior
regression — the live tovez robot uses its own per-robot JSON config, not this
compile-time default.

**This ticket requires explicit stakeholder confirmation before committing.** The
programmer must pause at the acceptance-criteria gate below and not rubber-stamp
the golden update.

Root cause B of the two baseline test failures (see issue).

## Acceptance Criteria

- [ ] **STAKEHOLDER GATE (must be confirmed before committing):** Stakeholder
  has explicitly confirmed that the intended SSOT values are `yawRateMax=70`
  and `odomOffY=3.5`. Record the confirmation in the commit message (e.g.,
  "Confirmed by stakeholder 2026-07-XX: yawRateMax=70, odomOffY=3.5 are correct").
- [ ] Programmer has located the SSOT: either `data/robots/robot_config.schema.json`
  default values or `scripts/gen_default_config.py` logic, and confirmed which
  file is authoritative for `defaultRobotConfig()` values.
- [ ] Golden file updated using the project's pin-update procedure (read the
  test file to find the update command).
- [ ] `tests/simulation/unit/test_default_config_pin.py::test_default_robot_config_unchanged`
  passes.
- [ ] `uv run python -m pytest tests/simulation` passes (no regressions).
- [ ] Commit message records the stakeholder-confirmed SSOT values.

## Implementation Plan

### Approach

Before touching any file:
1. Read `tests/simulation/unit/test_default_config_pin.py` to understand the
   pin mechanism (what file is the golden, how to update it).
2. Read `scripts/gen_default_config.py` and `data/robots/robot_config.schema.json`
   to identify where `yawRateMax` and `odomOffY` defaults originate.
3. Read `source/robot/DefaultConfig.cpp` — ground truth of what `defaultRobotConfig()`
   returns at runtime.
4. **Pause and surface the SSOT values to the team-lead** for stakeholder
   confirmation. Do not proceed to file edits until confirmed.
5. After confirmation: refresh the golden via the documented update procedure.

### Files to read first (no changes yet)

- `tests/simulation/unit/test_default_config_pin.py` — golden mechanism
- `scripts/gen_default_config.py` — generator logic
- `data/robots/robot_config.schema.json` — default values section
- `source/robot/DefaultConfig.cpp` — the generated artifact

### Files to modify (only after stakeholder confirmation)

- The golden file referenced by `test_default_config_pin.py` (read the test to
  find its path).

### Testing plan

Run `uv run python -m pytest tests/simulation/unit/test_default_config_pin.py -v`
before and after. Confirm the test goes from failing to passing. Then run the
full simulation gate.

### Documentation updates

Commit message must include the stakeholder-confirmed SSOT values. No other docs
required.
