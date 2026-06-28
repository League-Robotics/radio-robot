---
id: '009'
title: Final validation and cleanup
status: open
use-cases:
- SUC-005
depends-on:
- '004'
- '005'
- '006'
- '007'
- '008'
github-issue: ''
issue: plan-declarative-argument-schema-layer-for-source-commands.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Final validation and cleanup

## Description

With all five command files migrated, perform a comprehensive end-to-end validation
and clean up any residual issues. This ticket does not write new feature code — it
verifies that the full migration is correct and complete.

**Validation scope:**

1. **Full sim test run** with explicit baseline check — confirm the expected 2 failures
   only, no additional regressions.
2. **Protocol-string spot-checks** for all migrated command families:
   - Motion: S, T, D, G, R, TURN, RT, X, VW error paths
   - OTOS: OI (nodev), OV byte-identical reply, OL/OA optional-arg paths
   - System: ECHO, SAFE, SI, ZERO (keyword validation), HALT sub-verbs
   - Config: GET, SET, GET VEL
   - Debug: DBG LOOP, DBG IRQGUARD
3. **Error-path verification** — confirm ERR code+detail strings byte-identical:
   - `S 99999` -> `ERR range l`
   - `T 0 0 0` -> `ERR range ms`
   - `D 0 0 0` -> `ERR range mm`
   - `OV 1` -> `ERR badarg`
   - `ZERO` (no args) -> `ERR badarg`
4. **Firmware clean build**: `python build.py --clean` — must succeed with no
   warnings added (treat new warnings as errors).
5. **Binary size check** (informational): helpers are inline; net size should be
   <= prior size.
6. **Code audit**: grep for any remaining inline copy loops or `setIntArg`/
   `packSensorArg`/`vwScanKV`/`vwHasKey` references that should have been removed.

**Cleanup tasks (if any issues found):**

- Fix any remaining inline copy loops that should use `argStr`.
- Remove any dead code left by the migration (e.g. unreachable `parseNoArgs` copies).
- Ensure all `completes_issue` references are consistent.

## Acceptance Criteria

- [ ] `uv run --with pytest python -m pytest tests/simulation -q` shows exactly 2
  failures: `test_default_robot_config_unchanged` and
  `TestSchemaValidation::test_tovez_validates_against_schema`. No others.
- [ ] `tests/simulation/unit/test_system_commands_coverage.py` passes fully.
- [ ] `tests/simulation/system/test_stop_condition_coverage.py` passes fully.
- [ ] `tests/simulation/system/test_ekf_odometry_commands_coverage.py` passes fully.
- [ ] `python build.py --clean` succeeds under `-fno-exceptions -fno-rtti`.
- [ ] No `setIntArg`, `packSensorArg`, `vwScanKV`, or `vwHasKey` references remain
  in the five migrated command files (grep check).
- [ ] No remaining inline char-by-char `sval` copy loops in the five migrated files.
- [ ] Binary size does not grow vs. pre-migration build (or difference is documented
  with justification).
- [ ] All 5 use cases (SUC-001 through SUC-005) are satisfied.

## Implementation Plan

### Approach

This is a validation-only ticket. Do not write feature code.

1. Run the full sim suite; record result.
2. Run the three protocol-string oracle suites individually; record pass/fail.
3. Perform spot-check commands via sim and verify replies.
4. Run `python build.py --clean`; verify success.
5. Grep the five migrated files for residual patterns (see above).
6. Apply any targeted cleanup found; re-run suite to confirm still clean.

### Files to Modify

Only if cleanup is needed:
- `source/commands/OtosCommands.cpp`
- `source/commands/SystemCommands.cpp`
- `source/commands/MotionCommands.cpp`
- `source/commands/ConfigCommands.cpp`
- `source/commands/DebugCommands.cpp`

### Testing Plan

Primary command: `uv run --with pytest python -m pytest tests/simulation -q`

Extended validation:
```
uv run --with pytest python -m pytest tests/simulation/unit/test_system_commands_coverage.py -v
uv run --with pytest python -m pytest tests/simulation/system/test_stop_condition_coverage.py -v
uv run --with pytest python -m pytest tests/simulation/system/test_ekf_odometry_commands_coverage.py -v
python build.py --clean
```

Known pre-existing failures (do NOT fix):
- `tests/simulation/unit/test_default_robot_config_unchanged`
- `tests/simulation/unit/test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`

### Documentation Updates

Update issue `plan-declarative-argument-schema-layer-for-source-commands.md` status
field is handled automatically by `completes_issue: true` in this ticket's frontmatter.
