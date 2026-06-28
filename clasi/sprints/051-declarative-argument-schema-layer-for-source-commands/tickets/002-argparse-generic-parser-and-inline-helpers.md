---
id: '002'
title: ArgParse generic parser and inline helpers
status: in-progress
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: plan-declarative-argument-schema-layer-for-source-commands.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# ArgParse generic parser and inline helpers

## Description

Implement the generic `parseSchema` function and all inline helper functions.
This is the core library that tickets 003-009 depend on. Nothing else changes
in this ticket — no `CommandTypes.h` wiring, no command file changes.

`parseSchema` must implement all three parse shapes:
1. **No-arg** (`ndefs==0`, `variadic=false`) — returns empty ArgList.
2. **Positional** (`variadic=false`) — parses `ndefs` tokens by `ArgKind`; applies
   range check only when `ranged=true`; appends `packKv` value as trailing STR if
   present in `kvs`.
3. **Variadic** (`variadic=true`) — copies all tokens as STR args (bounded to
   `sval[31]`+NUL), cap at `MAX_ARGS`, init `ival=0/fval=0.0f` for each.

**Behaviour-preservation rules encoded in the implementation:**

- `variadic` path: `ival=0`, `fval=0.0f` before `sval` copy. Cap at `MAX_ARGS`.
  Copy at most `sizeof(Argument::sval)-1` = 31 chars, then NUL-terminate.
- `positional` path: `atoi(token)` for INT (no cast — caller decides truncation);
  `atof(token)` for FLOAT; bounded `sval` copy for STR.
- Range check (`ranged=true`, INT only): if `v < lo || v > hi`, return
  `ParseResult{ok=false, err={nullptr, def.name}}`. The dispatcher substitutes
  `desc.errFmt` as the error code.
- `minTokens`: if `ntokens < minTokens`, return
  `ParseResult{ok=false, err={nullptr, nullptr}}`.
- `packKv`: scan `kvs` for `kvs[i].key == schema.packKv`; if found, copy value
  into `args[count]` as STR using the same bounded copy, increment count.
  Match `packSensorArg` byte-for-byte: key `"sensor"`, trailing STR at position
  `ndefs` (wherever count stands after positional parse).

**Inline helpers** (all in `ArgParse.h` as `inline` functions):

- `argStr(Argument&, src)` — bounded sval copy (31 chars + NUL) + type=STR
- `argInt(Argument&, v)` — type=INT, ival=v, sval[0]='\0'
- `argFloat(Argument&, v)` — type=FLOAT, fval=v, sval[0]='\0'
- `kvFind(kvs, nkv, key)` — returns pointer to matching KVPair or nullptr
- `kvInt(kvs, nkv, key, def)` — typed KV lookup with default
- `kvFloat(kvs, nkv, key, def)` — typed KV lookup with default
- `kvHas(kvs, nkv, key)` — presence check

`argStr` uses the same bounded copy as `parseSchema` variadic (31 chars + NUL).

## Acceptance Criteria

- [ ] `source/commands/ArgParse.h` declares `parseSchema` and all inline helpers.
- [ ] `source/commands/ArgParse.cpp` implements `parseSchema`.
- [ ] `ArgParse.h` includes `ArgSchema.h` from `source/types/` and `Protocol.h`
  (for `KVPair`) and `CommandTypes.h` (for `Argument`, `ArgList`, `ParseResult`,
  `MAX_ARGS`).
- [ ] Variadic path: `ival=0`, `fval=0.0f` initialised before sval copy; count
  capped at `MAX_ARGS`; sval capped at 31 chars + NUL.
- [ ] Positional INT without `ranged`: value stored as `atoi(token)` with no range
  check (preserves int16 truncation behaviour of OV/SI when handler casts).
- [ ] Positional INT with `ranged`: if out of [lo,hi], returns
  `ParseResult{ok=false, err={nullptr, def.name}}`.
- [ ] `minTokens`: if `ntokens < minTokens`, returns
  `ParseResult{ok=false, err={nullptr, nullptr}}`.
- [ ] `packKv`: appends matching KV value as trailing STR; if key not found, count
  is unchanged (matches `packSensorArg` when sensor= absent).
- [ ] `argStr` produces the same bounded sval copy as the variadic path.
- [ ] Unit tests written covering:
  - `parseSchema` no-arg, positional (INT, FLOAT, STR), variadic.
  - Range check pass and fail (correct detail string returned).
  - `minTokens` guard.
  - `packKv` present and absent.
  - All inline helper functions.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` — no new failures.
- [ ] `source/commands/ArgParse.cpp` added to `tests/_infra/sim/CMakeLists.txt`
  if not already compiled in transitively.

## Implementation Plan

### Approach

Write `ArgParse.h` with `parseSchema` declaration + all inline helpers.
Write `ArgParse.cpp` with the `parseSchema` implementation.
Write unit tests before the dispatch wiring (ticket 003) so the library is
independently verified.

### Files to Create

- `source/commands/ArgParse.h` — declaration + inline helpers.
- `source/commands/ArgParse.cpp` — `parseSchema` implementation.
- `tests/simulation/unit/test_argparse.py` or C++ equivalent — unit tests.

### Files to Modify

- `tests/_infra/sim/CMakeLists.txt` — add `ArgParse.cpp` to sim build if needed.

### Testing Plan

Run: `uv run --with pytest python -m pytest tests/simulation -q`

Known pre-existing baseline: 2 failures
(`test_default_robot_config_unchanged`, `TestSchemaValidation::test_tovez_validates_against_schema`).
No new failures allowed.

Focus unit test coverage on:
- Variadic: 0 tokens, 1 token, MAX_ARGS tokens, MAX_ARGS+1 tokens (cap).
- Positional INT ranged: value at lo, hi, lo-1, hi+1 (boundary).
- Positional INT unranged: large value accepted without ERR range.
- packKv: token not present (count unchanged), token present (appended correctly).
- minTokens: ntokens < minTokens returns ok=false with null detail.

### Documentation Updates

None beyond inline comments in ArgParse.h.
