---
id: '008'
title: Migrate DebugCommands.cpp to ArgSchema
status: open
use-cases:
- SUC-001
- SUC-003
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: plan-declarative-argument-schema-layer-for-source-commands.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate DebugCommands.cpp to ArgSchema

## Description

Migrate `source/commands/DebugCommands.cpp` to use the ArgSchema framework.
All debug commands use `ForceReply::SERIAL` and are gated on `#ifndef HOST_BUILD`
or `#if defined(BENCH_OTOS_ENABLED) || defined(HOST_BUILD)` as appropriate; these
guards must remain untouched.

**Migration map for DebugCommands:**

| Command | Old form | New form |
|---------|----------|----------|
| DBG LOOP RESET | parseDbgLoopReset (no-arg stub) | `parseFn = nullptr` |
| DBG OTOS | parseDbgOtos (no-arg stub) | `parseFn = nullptr` |
| DBG EST | parseDbgEst (no-arg stub) | `parseFn = nullptr` |
| DBG LOOP | parseDbgLoop (variadic, up to 2 STR tokens) | `makeSchemaCmd` with `variadic=true` |
| DBG I2CLOG | parseDbgI2clog (0 or 1 STR token) | `makeSchemaCmd` with `variadic=true` |
| DBG I2C | parseDbgI2c (0 or 1 STR token) | `makeSchemaCmd` with `variadic=true` |
| DBG IRQGUARD | parseDbgIrqguard (0 or 1 INT) | `makeSchemaCmd` with `{ndefs=1, minTokens=0, ranged=false}` |
| DBG WEDGE | parseDbgWedge (up to 7 optional INT) | keep custom parseFn (no variadic-INT schema support) |
| DBG OTOS BENCH | parseDbgOtosBench (mixed INT + FLOAT KV) | keep custom parseFn (mixed types) |
| I2CW | parseI2cw (STR tokens, requires ≥2) | keep custom parseFn (hex validation) |
| I2CR | parseI2cr (STR tokens, count range check) | keep custom parseFn (hex + range) |

**Handler body cleanup:** Replace inline kv loops in `handleDbgOtosBench` with
`kvFloat`/`kvInt` from ArgParse.h. The `parseDbgOtosBench` custom parseFn's
`for (i in nkv) if (strcmp(kvs[i].key, ...))` pattern is replaced by `kvFloat`.

**No-arg stubs:** DBG LOOP RESET, DBG OTOS, DBG EST all have trivial
`res.ok=true; res.args.count=0` parsers that are pure boilerplate. Delete and
use `nullptr`.

**Variadic DBG LOOP/I2CLOG/I2C:** All three use the identical tokens-as-STR
copy pattern. `variadic=true` schema replaces all three parsers.

**DBG IRQGUARD:** Currently `{INT 0 or 1}` optional. `{ndefs=1, minTokens=0,
ranged=false}` schema reproduces this exactly.

## Acceptance Criteria

- [ ] `parseDbgLoopReset`, `parseDbgOtos`, `parseDbgEst` deleted; registrations
  updated to `parseFn=nullptr`.
- [ ] `parseDbgLoop`, `parseDbgI2clog`, `parseDbgI2c` deleted; replaced by
  `variadic=true` schemas + `makeSchemaCmd` registrations.
- [ ] `parseDbgIrqguard` deleted; replaced by `{ndefs=1, minTokens=0, ranged=false}`
  schema.
- [ ] `parseDbgWedge`, `parseDbgOtosBench`, `parseI2cw`, `parseI2cr` retained.
- [ ] `handleDbgOtosBench` uses `kvFloat`/`kvInt` instead of inline loop.
- [ ] All conditional compilation guards (`#ifndef HOST_BUILD`,
  `#if defined(BENCH_OTOS_ENABLED) || defined(HOST_BUILD)`) preserved unchanged.
- [ ] `DebugCommands::getCommands()` registration order preserved (longest prefix
  first within groups, e.g. DBG OTOS BENCH before DBG OTOS).
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` — no new failures.
- [ ] Primary oracle: full sim unit suite (DBG commands are exercised in HOST_BUILD).

## Implementation Plan

### Approach

1. Add `#include "ArgParse.h"` to DebugCommands.cpp.
2. Delete parseDbgLoopReset, parseDbgOtos, parseDbgEst; update registrations.
3. Write static schemas for DBG LOOP, DBG I2CLOG, DBG I2C (variadic), DBG IRQGUARD
   (ndefs=1); delete corresponding parsers; update registrations.
4. Rewrite `handleDbgOtosBench` KV loop with `kvFloat`/`kvInt`.
5. Retain parseDbgWedge, parseDbgOtosBench, parseI2cw, parseI2cr.
6. Verify registration order unchanged.

### Files to Modify

- `source/commands/DebugCommands.cpp` — remove parse functions, add schemas,
  update registrations, update handler body.

### Testing Plan

Run: `uv run --with pytest python -m pytest tests/simulation -q`

DebugCommands are exercised in HOST_BUILD. The sim suite covers DBG LOOP,
DBG OTOS BENCH (bench=1/0), DBG EST via the unit tests.

### Documentation Updates

Update comment block in DebugCommands.cpp to note the ArgSchema migration.
