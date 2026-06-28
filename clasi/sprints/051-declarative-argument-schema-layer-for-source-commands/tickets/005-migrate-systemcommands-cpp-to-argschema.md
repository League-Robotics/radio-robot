---
id: '005'
title: Migrate SystemCommands.cpp to ArgSchema
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

# Migrate SystemCommands.cpp to ArgSchema

## Description

Migrate `source/commands/SystemCommands.cpp` to use the ArgSchema framework,
removing no-arg parse stubs and converting variadic parsers to schema declarations.
Complex parsers with sub-verb dispatch (HALT, ZERO) retain their custom `parseFn`
but have their bodies cleaned up with `argStr` helpers.

**Migration map for SystemCommands:**

| Command | Old form | New form |
|---------|----------|----------|
| HELLO, PING, ID, VER, HELP, SNAP, + (keepalive) | no-arg stubs | `parseFn = nullptr` |
| ECHO, STREAM, SAFE | variadic parsers (tokens-as-STR) | `makeSchemaCmd` with `variadic=true` |
| SI | parseSI (3 mandatory INT, no range check) | `makeSchemaCmd` with `{ndefs=3, minTokens=3, ranged=false}` |
| RF | parseRf (0 or 1 optional INT) | `makeSchemaCmd` with `{ndefs=1, minTokens=0, ranged=false}` |
| ZERO | parseZero (custom: keyword validation + STR copy) | keep custom parseFn; body rewritten with `argStr` |
| HALT | parseHalt (complex sub-verb dispatch) | keep custom parseFn; body uses `argStr` |
| BAUD | parseBaud (optional STR) | keep custom parseFn (sval not ival for large int) |

**CRITICAL behaviour-preservation notes:**

- `ECHO` variadic: `parseEcho` currently sets `ival=0`, `fval=0.0f` explicitly
  per arg. `parseSchema` variadic path must also do this — confirmed in ticket 002.
- `STREAM` variadic: `parseStream` passes all tokens raw (including `fields=enc,pose`
  kv-style strings) as STR args. Handler already checks `args.args[i].sval` prefix.
  `variadic=true` reproduces this correctly.
- `SAFE` variadic: `parseSafe` passes tokens as STR; handler `strcmp`s `args[0].sval`.
  `variadic=true` is correct.
- `SI` stores plain `atoi` results in `ival` (no int16 cast in parser; handler uses
  `args.args[i].ival` directly as `int32_t`). `ranged=false` is correct.
- `RF` optional INT: handler checks `args.count >= 1` and uses `args.args[0].ival`.
  Schema `{ndefs=1, minTokens=0, ranged=false}` reproduces this.

## Acceptance Criteria

- [ ] `parseHello`, `parsePing`, `parseId`, `parseVer`, `parseHelp`, `parseSnap`,
  `parseKeepalive` deleted; registrations updated to `parseFn = nullptr`.
- [ ] `parseEcho`, `parseStream`, `parseSafe` deleted; replaced by `variadic=true`
  ArgSchema + `makeSchemaCmd` registrations.
- [ ] `parseSI` deleted; replaced by `{ndefs=3, minTokens=3, ranged=false}` schema.
- [ ] `parseRf` deleted; replaced by `{ndefs=1, minTokens=0, ranged=false}` schema.
- [ ] `parseZero` retained; body uses `argStr` helper for the token copy loops.
- [ ] `parseHalt` retained; body uses `argStr` helper for the token copy loops.
- [ ] `parseBaud` retained (sval path for large int; argStr acceptable).
- [ ] All handler reply strings byte-identical vs. before.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` — no new failures.
- [ ] Primary oracle: `tests/simulation/unit/test_system_commands_coverage.py`.

## Implementation Plan

### Approach

1. Add `#include "ArgParse.h"` to SystemCommands.cpp.
2. Delete no-arg parse stubs; update `buildCommandTable` registrations.
3. Write static `ArgSchema` structs for ECHO, STREAM, SAFE (variadic), SI (ndefs=3),
   RF (ndefs=1); delete the corresponding parsers; update registrations.
4. Rewrite `parseZero` and `parseHalt` body copy loops using `argStr`.
5. Optionally apply `replyOKf`/`replyErrf` to reduce body-buffer boilerplate in
   handlers (opportunistic, not required).

### Files to Modify

- `source/commands/SystemCommands.cpp` — remove parse functions, add schemas,
  update registrations, clean up bodies.

### Testing Plan

Run: `uv run --with pytest python -m pytest tests/simulation -q`

Primary oracle: `tests/simulation/unit/test_system_commands_coverage.py`.

Spot checks:
- `ECHO hello world` -> `OK echo hello world`
- `SAFE off` -> `OK safety off timeout=<n>`
- `SI 100 200 300` -> `OK setpose x=100 y=200 h=300`
- `RF` (no args) -> `OK rf chan=... group=10` (firmware only; sim may vary)
- `HALT TIME 1000` -> `OK HALT id=<n>`

### Documentation Updates

Update comment block in SystemCommands.cpp to note the ArgSchema migration.
