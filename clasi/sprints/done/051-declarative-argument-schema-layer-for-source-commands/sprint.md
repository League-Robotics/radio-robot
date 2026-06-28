---
id: '051'
title: Declarative argument-schema layer for source/commands
status: done
branch: sprint/051-declarative-argument-schema-layer-for-source-commands
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- plan-declarative-argument-schema-layer-for-source-commands.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 051: Declarative argument-schema layer for source/commands

## Goals

Replace ~48 bespoke per-command `parseXxx` functions across five command files
with a small declarative `ArgSchema` type that most commands *declare* rather than
*implement*. Add a single generic `parseSchema` function and a suite of inline
helpers that both the schema path and the remaining custom parsers can share.
Wire the schema path additively into `CommandProcessor` dispatch. Migrate each
command file. Behaviour (wire replies, error codes, value handling) remains
byte-identical — the Python sim suites assert exact reply strings.

## Problem

`source/commands/` contains ~48 hand-written `parseXxx` functions duplicating
four shapes: no-arg stubs, tokens-as-STR copies, int-with-range parsers, and KV
scanners. The duplication raises per-command maintenance cost and drift risk. Local
helpers (`setIntArg`, `packSensorArg`, `vwScanKV`) prove the pattern wants
factoring; it has not been done framework-wide.

## Solution

1. New `source/types/ArgSchema.h` — `ArgKind`, `ArgDef`, `ArgSchema` types.
2. New `source/commands/ArgParse.{h,cpp}` — `parseSchema` generic parser; inline
   helpers `argInt/argFloat/argStr`, `kvFind/kvInt/kvFloat/kvHas`.
3. Additive wiring in `CommandTypes.h` (`schema` field + `makeSchemaCmd`) and
   `CommandProcessor.cpp` (dispatch branch before existing `parseFn` path; variadic
   `replyOKf/replyErrf`).
4. Per-file migration — one ticket per command file — following the migration map
   in the issue. No-arg stubs become `nullptr`. Simple commands use `ArgSchema`
   declarations. Complex commands keep custom `parseFn` but reuse the new helpers.
5. OTOS `nodev` guard factored to one inline helper (6x duplication removed).

## Success Criteria

- Clean firmware build under `-fno-exceptions -fno-rtti`; binary not larger.
- `uv run --with pytest python -m pytest tests/simulation -q` produces no new
  failures beyond the 2 known pre-existing failures.
- Protocol-string suites (`test_system_commands_coverage.py`,
  `test_stop_condition_coverage.py`, `test_ekf_odometry_commands_coverage.py`)
  pass at the same byte-identical reply strings.
- All ~48 parse stubs are deleted or converted; no duplicate shape remains.

## Scope

### In Scope

- `source/types/ArgSchema.h` — new types.
- `source/commands/ArgParse.h` + `ArgParse.cpp` — generic parser + helpers.
- `source/types/CommandTypes.h` — `schema` field, `makeSchemaCmd`.
- `source/commands/CommandProcessor.h` / `.cpp` — dispatch branch, `replyOKf/replyErrf`.
- `source/commands/OtosCommands.cpp` — migrate + `nodev` guard helper.
- `source/commands/SystemCommands.cpp` — migrate.
- `source/commands/MotionCommands.cpp` — migrate + reuse helpers.
- `source/commands/ConfigCommands.cpp` — migrate.
- `source/commands/DebugCommands.cpp` — migrate.
- `tests/_infra/sim/CMakeLists.txt` — add `ArgParse.cpp` to sim build if needed.
- New unit tests for `parseSchema` and inline helpers.

### Out of Scope

- Changes to `HandlerFn`, `ArgList`, `ParsedCommand`, or the queue path.
- Changes to non-command subsystems (Robot, MotionController, etc.).
- Any new command protocol features.
- PortController or ServoController command files.

## Test Strategy

All tickets tested with: `uv run --with pytest python -m pytest tests/simulation -q`

Known pre-existing baseline: exactly 2 failures in
`test_default_robot_config_unchanged` and
`TestSchemaValidation::test_tovez_validates_against_schema` —
unrelated; do not fix. A ticket is acceptable if no new failures appear.

Protocol-string regression oracle:
- `tests/simulation/unit/test_system_commands_coverage.py`
- `tests/simulation/system/test_stop_condition_coverage.py`
- `tests/simulation/system/test_ekf_odometry_commands_coverage.py`
- Full sim unit suite.

Each ticket must leave the tree compilable and tests no worse than baseline.

## Architecture Notes

- `ArgSchema` lives in `source/types/` so `CommandTypes.h` can reference it
  without creating a `commands/ -> commands/` dependency cycle.
- The dispatch branch for schema is additive (checked before `parseFn`); all
  existing custom parsers continue to work untouched until each file is migrated.
- Behaviour-preservation is non-negotiable: `OV`/`SI` use `ranged=false` (no
  range check, silent int16 truncation preserved); `S/T/D/G/R` keep exact
  `[lo,hi]` + detail strings. `minTokens` reproduces existing badarg guards.
  `variadic` reproduces MAX_ARGS cap and `ival=0/fval=0` init.
- No heap, no exceptions, no RTTI. All parsing is stack-based.

## GitHub Issues

(None linked.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | ArgSchema types (source/types/ArgSchema.h) | — |
| 002 | ArgParse generic parser and inline helpers | 001 |
| 003 | Framework wiring: CommandTypes + CommandProcessor dispatch | 002 |
| 004 | Migrate OtosCommands.cpp to ArgSchema | 003 |
| 005 | Migrate SystemCommands.cpp to ArgSchema | 003 |
| 006 | Migrate MotionCommands.cpp to ArgSchema | 003 |
| 007 | Migrate ConfigCommands.cpp to ArgSchema | 003 |
| 008 | Migrate DebugCommands.cpp to ArgSchema | 003 |
| 009 | Final validation and cleanup | 004, 005, 006, 007, 008 |

Tickets execute serially in the order listed.
