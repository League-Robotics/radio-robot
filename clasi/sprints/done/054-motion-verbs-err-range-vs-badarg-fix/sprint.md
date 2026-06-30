---
id: '054'
title: Motion verbs ERR range vs badarg fix
status: done
branch: sprint/054-motion-verbs-err-range-vs-badarg-fix
use-cases:
- SUC-001
issues:
- motion-verbs-err-badarg-instead-of-range.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 054: Motion verbs ERR range vs badarg fix

## Goals

Restore the correct `ERR range <field>` response for out-of-range arguments to
the motion verbs S, T, D, and R. Tighten the simulation test suite so this
class of regression cannot pass CI silently again.

## Problem

Since the sprint 051 ArgSchema migration, the motion verbs S/T/D/R are
registered as custom `parseFn`s. On a ranged-value failure each parse function
sets `res.err.code = nullptr` and `res.err.detail = "<field>"`. The dispatcher
in `CommandProcessor::dispatchTable` falls through to `desc.errFmt` (which
defaults to `"badarg"`) instead of the expected `"range"`. The result is that
`S 99999 0` replies `ERR badarg l` instead of `ERR range l`.

The field detail token is correct; only the error code is wrong.

## Solution

Two-step fix:

1. **C++ dispatcher** (`CommandProcessor.cpp`): in the `parseFn` branch, honor
   `result.err.code` when it is non-null, falling back to `desc.errFmt` only
   when `result.err.code` is null.

2. **Parse functions** (`MotionCommands.cpp`): set `res.err.code = "range"` on
   every ranged-value failure path in `parseS`, `parseT`, `parseD`, and
   `parseR`. (Arg-count failures leave `res.err.code = nullptr`, which then
   falls through to `desc.errFmt = "badarg"` — preserving that behavior.)

3. **Test hardening**: convert the loose static-string assertions in
   `test_motion_verbs_v2.py` to live firmware calls via `Sim`, and add targeted
   range-error cases to `test_protocol_v2.py`. Ensure neither file passes
   through a regression silently.

## Success Criteria

- `S 99999 0` → `ERR range l`
- `S 0 99999` → `ERR range r`
- `T 0 0 0` → `ERR range ms` (l/r=0 in range; ms=0 out-of-range)
- `D 0 0 0` → `ERR range mm`
- `S` (no args) → `ERR badarg` (arg-count failure unchanged)
- `T 0 0` (missing ms) → `ERR badarg`
- `uv run pytest` passes

## Scope

### In Scope

- `source/commands/CommandProcessor.cpp` — dispatcher `parseFn` branch
- `source/commands/MotionCommands.cpp` — `parseS`, `parseT`, `parseD`, `parseR`
- `tests/simulation/unit/test_motion_verbs_v2.py` — convert loose static tests
  to live `Sim` calls; add range-vs-badarg assertions
- `tests/simulation/unit/test_protocol_v2.py` — add live range-error assertions
  for S/T/D/R

### Out of Scope

- `parseVW`, `parseTURN` — these already set `res.err.code` correctly; no change
- `parseRT`, `parseG` — schema-based; dispatched differently; not regressed
- Bench / hardware validation (nice-to-have; not a CI gate for this sprint)
- Any motion behavior changes

## Test Strategy

- CI gate: `uv run pytest` (collects `tests/simulation/` only)
- Ticket 001: firmware fix verified by new live sim assertions in
  `test_protocol_v2.py` (already builds the sim as a `build_lib` fixture)
- Ticket 002: `test_motion_verbs_v2.py` converted from static-string to live
  `Sim` calls and range-vs-badarg assertions added

## Architecture Notes

The fix is deliberately minimal: one guard clause added in `dispatchTable`'s
`parseFn` branch; `err.code` already flows through `ParseResult.err.code`
(the struct field exists). No new types, no API surface changes.

## GitHub Issues

(none linked externally)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Fix range-vs-badarg in dispatcher and parse functions | — |
| 002 | Harden sim tests to assert exact ERR range/badarg strings | 001 |

Tickets execute serially in the order listed.
