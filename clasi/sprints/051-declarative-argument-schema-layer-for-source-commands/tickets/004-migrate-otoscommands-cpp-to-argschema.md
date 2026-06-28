---
id: '004'
title: Migrate OtosCommands.cpp to ArgSchema
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: plan-declarative-argument-schema-layer-for-source-commands.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate OtosCommands.cpp to ArgSchema

## Description

Migrate `source/commands/OtosCommands.cpp` to use the new ArgSchema framework.
This removes 7 bespoke parse functions and factors the repeated `nodev` guard.

**Migration map for OtosCommands:**

| Command | Old form | New form |
|---------|----------|----------|
| OI, OZ, OR, OP | parseOI/OZ/OR/OP (no-arg stubs) | `parseFn = nullptr` |
| OV | parseOV (3 mandatory INT, no range check) | `makeSchemaCmd` with schema `{ndefs=3, minTokens=3, ranged=false}` |
| OL, OA | parseOL/OA (0 or 1 optional INT) | `makeSchemaCmd` with schema `{ndefs=1, minTokens=0, ranged=false}` |

**CRITICAL behaviour-preservation for OV:**
`OV` currently stores args as `(int16_t)atoi(token)` — the cast is in the parser.
With `ranged=false`, `parseSchema` stores the raw `atoi` result as `int32_t` in
`ival`. The handler `handleOV` must then cast to `int16_t` at the use site:
`int16_t ox = (int16_t)args.args[0].ival;` — this cast is already present in
the current handler (line ~191) and must remain.

**nodev guard factor:**
Add one inline helper (in the file or in a small shared header):
```cpp
static bool otosReady(OtosCtx* c, const char* verb, char* rbuf, int rbsz,
                      const char* corrId, ReplyFn fn, void* ctx);
```
Returns `false` and emits `ERR nodev <verb>` when `!c->otos->is_initialized()`.
Replace the 6 identical guard blocks in handleOI/OZ/OR/OV/OL/OA with
`if (!otosReady(c, "...", rbuf, sizeof(rbuf), corrId, replyFn, replyCtx)) return;`.

## Acceptance Criteria

- [x] `parseOI`, `parseOZ`, `parseOR`, `parseOP` deleted; registrations updated to
  `makeCmd(..., nullptr, handleXxx, ...)` or `makeSchemaCmd` with empty schema.
- [x] `parseOV` deleted; replaced with static `ArgSchema ovSchema{ndefs=3,
  minTokens=3, ranged=false, ...}` and `makeSchemaCmd` registration.
- [x] `parseOL`, `parseOA` deleted; each replaced with schema `{ndefs=1,
  minTokens=0, ranged=false}`.
- [x] `handleOV` still casts `args.args[i].ival` to `int16_t` at use site
  (behaviour unchanged).
- [x] `handleOL`/`handleOA` still cast `args.args[0].ival` to `int8_t` at use site.
- [x] `otosReady` inline helper implemented; all 6 hardware handlers use it.
- [x] OtosCommands.cpp compiles cleanly; no other file changed.
- [x] `uv run --with pytest python -m pytest tests/simulation -q` — no new failures.
- [x] Spot-check via test suite: `OV 1 2 3` -> `OK setpos x=1 y=2 h=3`;
  `OV 1` -> `ERR badarg`; `OI` when not initialized -> `ERR nodev oi`.

## Implementation Plan

### Approach

1. Add `#include "ArgParse.h"` to OtosCommands.cpp.
2. Write static `ArgSchema` structs for OV, OL, OA.
3. Delete parseOI/OZ/OR/OP/OV/OL/OA.
4. Write `otosReady` inline static.
5. Update `getCommands()` to use `nullptr` for no-arg commands and `makeSchemaCmd`
   for OV/OL/OA. The `makeCmd` calls for OI/OZ/OR/OP change only `parseFn` to
   `nullptr`; alternatively use `makeSchemaCmd` with a null-schema (ndefs=0,
   variadic=false, minTokens=0, packKv=nullptr) — either is equivalent since
   the dispatcher handles both `schema==nullptr` and `ndefs==0/variadic=false`
   as empty ArgList.
6. Update handlers to use `otosReady`.

### Files to Modify

- `source/commands/OtosCommands.cpp` — remove parse functions, add schemas,
  update registrations, add nodev helper.

### Testing Plan

Run: `uv run --with pytest python -m pytest tests/simulation -q`

Primary oracle: `tests/simulation/system/test_ekf_odometry_commands_coverage.py`
(exercises OI, OZ, OR, OP, OV, OL, OA with exact reply string assertions).

Known pre-existing baseline: 2 failures. No new failures allowed.

### Documentation Updates

Update the comment block in OtosCommands.cpp to note the ArgSchema migration.
