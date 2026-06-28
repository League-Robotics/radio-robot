---
id: '007'
title: Migrate ConfigCommands.cpp to ArgSchema
status: in-progress
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

# Migrate ConfigCommands.cpp to ArgSchema

## Description

Migrate `source/commands/ConfigCommands.cpp` to use the ArgSchema framework.
This file is small (3 command registrations) and the migration is straightforward.

**Migration map for ConfigCommands:**

| Command | Old form | New form |
|---------|----------|----------|
| GET VEL | parseGetVel (no-arg stub) | `parseFn = nullptr` |
| GET | parseGet (tokens-as-STR variadic) | `makeSchemaCmd` with `variadic=true` |
| SET | parseSet (custom: KV-to-"k=v" encoding) | keep custom parseFn; body rewritten with `argStr` |

**GET VEL**: The current registration passes `nullptr` implicitly — `parseGetVel`
is a no-arg stub that should have been `nullptr` all along. Delete it; register
with `parseFn=nullptr`.

**GET**: `parseGet` copies each token into `args[i].sval` (exactly the variadic
pattern). Replace with `variadic=true` schema. Handler `handleGet` reads
`args.args[i].sval` for key names — unchanged.

**SET**: `parseSet` converts KV pairs into `"key=value"` STR args using `snprintf`.
This is not the variadic shape (it reads from `kvs`, not `tokens`). Retain custom
`parseFn`; rewrite the body using `argStr` for the sval copy. The `snprintf` for
`"key=value"` reconstruction cannot be replaced by a simple `argStr` call since it
joins key+value, but the inner per-arg `sval` assignment can use `argStr` after
the format.

**CRITICAL behaviour-preservation:**
- `GET key1 key2` -> each token becomes `args[i].sval`; `handleGet` looks up each
  key in the config registry. `variadic=true` preserves this exactly.
- `SET key=val` -> `parseSet` produces `args[0].sval = "key=val"`. The custom
  parseFn is retained; only internal style changes.

## Acceptance Criteria

- [ ] `parseGetVel` deleted; GET VEL registered with `parseFn=nullptr`.
- [ ] `parseGet` deleted; GET registered with `makeSchemaCmd` using `variadic=true`
  schema; `handleGet` unchanged.
- [ ] `parseSet` retained; body uses `argStr` where applicable.
- [ ] `appendConfigCommands` updated to use new registrations.
- [ ] All existing GET/SET/GET VEL behaviour preserved byte-identically.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` — no new failures.
- [ ] Primary oracle: `tests/simulation/unit/test_system_commands_coverage.py`
  (exercises GET and SET with exact reply assertions).

## Implementation Plan

### Approach

1. Add `#include "ArgParse.h"` to ConfigCommands.cpp.
2. Delete `parseGetVel`; update GET VEL registration to `nullptr`.
3. Write static `ArgSchema getSchema{variadic=true, ...}`.
4. Delete `parseGet`; update GET registration to `makeSchemaCmd`.
5. Rewrite `parseSet` internals using `argStr` for sval assignments.
6. Update `appendConfigCommands` registrations.

### Files to Modify

- `source/commands/ConfigCommands.cpp` — remove parse functions, add schema,
  update registrations.

### Testing Plan

Run: `uv run --with pytest python -m pytest tests/simulation -q`

Spot checks:
- `GET trackwidthMm` -> `OK get trackwidthMm=<value>`
- `SET trackwidthMm=123` -> `OK set trackwidthMm=123`
- `GET VEL` -> `OK get vel=<vL>:E,<vR>:E`

### Documentation Updates

None beyond comments in ConfigCommands.cpp.
