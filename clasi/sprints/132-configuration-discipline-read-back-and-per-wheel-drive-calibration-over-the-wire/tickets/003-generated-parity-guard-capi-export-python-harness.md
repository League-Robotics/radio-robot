---
id: '003'
title: Generated parity guard (capi export + Python harness)
status: open
use-cases:
- SUC-001
depends-on:
- '002'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Generated parity guard (capi export + Python harness)

## Description

Build a generated parity guard proving the C++ `Config::Robot` group
structs (ticket 002's C++ target) and the generated pydantic model
(ticket 002's pydantic target) have not structurally drifted from each
other, mirroring `src/motion/planner/capi.cpp`'s `plannerStructSizes()`/
`plannerLimitsOffsets()` pattern (`capi.cpp:69,93`) and its Python
counterpart `planner_harness.py:207-212`. Add an `extern "C"` export in
`src/firm/config/` (new file) exporting struct sizes and per-field byte
offsets for each generated group struct. Add a Python harness walking
that export via `ctypes` and cross-checking field count/offsets/types
against the generated pydantic model's own field list.

This is the structural guarantee that replaces `check_config_sync.py`
(deleted next, ticket 004) — a strictly stronger guarantee (byte-for-byte
generated-code comparison) than the old hand-curated allowlist ever gave.

## Acceptance Criteria

- [ ] A new `extern "C"` export (naming mirrors `plannerStructSizes()`/
      `plannerLimitsOffsets()`) exists in `src/firm/config/`, exporting
      sizes and per-field byte offsets for every generated C++ group
      struct from ticket 002.
- [ ] A Python harness (mirroring `planner_harness.py`'s approach) loads
      the export via `ctypes` and walks it against the generated pydantic
      model's field list, asserting field count and relative ordering
      agree.
- [ ] The harness passes against the current generated pair.
- [ ] The harness is demonstrated to fail on an intentionally introduced
      structural mismatch (e.g. temporarily hand-editing the generated
      header to insert a field mid-struct) — done on a throwaway local
      change, reverted before this ticket lands; completion notes record
      that the induced-failure check was actually performed.

## Testing

- **Existing tests to run**: none directly (new capability).
- **New tests to write**: the harness itself is the new test — add it to
  the pytest suite so it runs on normal collection.
- **Verification command**: `uv run python -m pytest <new harness test
  path> -q`.

## Implementation Plan

**Approach**: Follow `capi.cpp`'s existing pattern directly — a `count`
parameter, a `static const uint32_t kOffsets[]` table per struct, one
function per struct family (or one function taking a struct-id enum,
whichever proves cleaner against 7 groups instead of planner's 1). Python
side mirrors `planner_harness.py`'s `ctypes` loading approach.

**Files to create**: `src/firm/config/config_parity_capi.{h,cpp}` (name to
confirm during implementation), a new Python harness file alongside
`planner_harness.py`'s sibling tests.

**Files to modify**: build wiring so the new capi export builds as a
`HOST_BUILD`-loadable shared object, matching however `capi.cpp` is
currently built/loaded.

**Testing plan**: as above.

**Documentation updates**: none required beyond the harness file's own
doc comment explaining the pattern it mirrors.
