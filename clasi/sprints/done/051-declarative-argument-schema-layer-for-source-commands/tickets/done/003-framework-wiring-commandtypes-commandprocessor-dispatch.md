---
id: '003'
title: 'Framework wiring: CommandTypes + CommandProcessor dispatch'
status: done
use-cases:
- SUC-002
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: plan-declarative-argument-schema-layer-for-source-commands.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Framework wiring: CommandTypes + CommandProcessor dispatch

## Description

Wire the new `ArgSchema` pointer into `CommandDescriptor` and add the schema
dispatch branch in `CommandProcessor::dispatchTable`. After this ticket, the
framework can route schema-driven commands; no command file uses it yet (that
happens in tickets 004-008). Also add `replyOKf`/`replyErrf` variadic helpers
to `CommandProcessor`.

This is a purely additive change. All existing `parseFn`-based commands continue
to work identically — the schema branch is checked first but only activates when
`desc.schema != nullptr`. No existing `makeCmd` call changes.

## Acceptance Criteria

- [ ] `source/types/CommandTypes.h`:
  - Adds `#include "ArgSchema.h"`.
  - `CommandDescriptor` gains one field: `const ArgSchema* schema;` (default `nullptr`).
  - `makeCmd` zero-initialises `schema` (the new field must be set to `nullptr` in
    the existing factory so existing call sites compile unchanged).
  - New `makeSchemaCmd(prefix, schema, handlerFn, ctx, errFmt, forceReply, flags)`
    inline factory that sets `parseFn = nullptr` and `schema = schema_ptr`.
- [ ] `source/commands/CommandProcessor.cpp` `dispatchTable()`:
  - Adds `#include "ArgParse.h"`.
  - Schema branch added before `parseFn` branch:
    ```
    if (desc.schema != nullptr)
        result = parseSchema(argTokens, argNtok, kvs, nkv, *desc.schema);
    else if (desc.parseFn != nullptr)
        result = desc.parseFn(argTokens, argNtok, kvs, nkv);
    ```
  - Error reporting for the schema path uses `desc.errFmt` as the error code, same
    as the `parseFn` path. Detail comes from `result.err.detail`.
- [ ] `source/commands/CommandProcessor.h` / `.cpp`:
  - Adds `replyOKf` static method: formats `body` via `vsnprintf` into a stack
    buffer, then calls `replyOK`. Signature:
    `static void replyOKf(char* buf, int size, const char* verb, const char* id, ReplyFn fn, void* ctx, const char* fmt, ...)`.
  - Adds `replyErrf` static method analogously.
- [ ] All existing sim tests pass without new failures:
  `uv run --with pytest python -m pytest tests/simulation -q`.
- [ ] No existing `makeCmd` call site requires modification.
- [ ] Firmware builds cleanly: `python build.py --clean`.

## Implementation Plan

### Approach

Make the three changes in the order shown below. Each sub-step is compilable
on its own.

1. Add `schema` field to `CommandDescriptor` in `CommandTypes.h` + update `makeCmd`
   default + add `makeSchemaCmd`.
2. Add schema dispatch branch to `CommandProcessor.cpp`.
3. Add `replyOKf`/`replyErrf` to `CommandProcessor.h`/`.cpp`.

### Files to Modify

- `source/types/CommandTypes.h` — `#include "ArgSchema.h"`, new field, `makeSchemaCmd`.
- `source/commands/CommandProcessor.h` — `replyOKf`/`replyErrf` declarations.
- `source/commands/CommandProcessor.cpp` — `#include "ArgParse.h"`, schema branch,
  `replyOKf`/`replyErrf` implementations.

### Testing Plan

Run: `uv run --with pytest python -m pytest tests/simulation -q`

Known pre-existing baseline: 2 failures. No new failures allowed.

The schema dispatch branch activates only when `desc.schema != nullptr`. Since no
command uses `makeSchemaCmd` yet, the entire sim suite exercises the `parseFn`
path exclusively — confirming the additive change is transparent.

### Documentation Updates

Add a comment in `CommandProcessor.cpp` dispatch noting the schema-first branch
ordering and the D11 rule (no reply in schema-parse error path; dispatcher emits it).
