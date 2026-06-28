---
id: '001'
title: ArgSchema types (source/types/ArgSchema.h)
status: open
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on: []
github-issue: ''
issue: plan-declarative-argument-schema-layer-for-source-commands.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# ArgSchema types (source/types/ArgSchema.h)

## Description

Create the foundational declarative types for the argument-schema layer. This new
header in `source/types/` (alongside `CommandTypes.h`) defines the three structs
that every subsequent ticket builds on.

The types must be placed in `source/types/` — not in `source/commands/` — so that
`CommandTypes.h` can `#include` them without creating a circular dependency.

## Acceptance Criteria

- [ ] `source/types/ArgSchema.h` is created with exactly:
  - `enum class ArgKind : uint8_t { INT, FLOAT, STR };`
  - `struct ArgDef { const char* name; ArgKind kind; bool ranged; int32_t lo, hi; };`
  - `struct ArgSchema { const ArgDef* defs; int ndefs; int minTokens; bool variadic; const char* packKv; };`
- [ ] File includes only `<stdint.h>` (no other dependencies).
- [ ] File compiles cleanly under `-std=c++11 -fno-exceptions -fno-rtti`.
- [ ] `ArgDef.ranged` is documented: when `false`, no range check is applied
  (INT value accepted via `atoi` with silent truncation — preserves `OV`/`SI`
  behaviour).
- [ ] `ArgSchema.packKv` is documented: when non-null, `parseSchema` will append
  the value of the matching KV pair as a trailing STR arg (reproduces
  `packSensorArg` for T/D/TURN).
- [ ] No change to any existing file in this ticket.

## Implementation Plan

### Approach

Create a single new header. No `.cpp` needed — pure data type declarations.

### Files to Create

- `source/types/ArgSchema.h` — new file, self-contained plain-data types.

### Files to Modify

None in this ticket.

### Testing Plan

At this stage no test can directly exercise the types. Verify compilation only:

```
python build.py --clean
```

The file must compile without errors when included transitively (ticket 002 will
include it). The sim test suite need not be run for this ticket — it has no test
surface yet.

### Documentation Updates

None.
