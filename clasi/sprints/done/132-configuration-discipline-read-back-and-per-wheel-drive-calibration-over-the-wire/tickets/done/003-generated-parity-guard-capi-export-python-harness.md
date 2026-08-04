---
id: '003'
title: Generated parity guard (capi export + Python harness)
status: done
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

- [x] A new `extern "C"` export (naming mirrors `plannerStructSizes()`/
      `plannerLimitsOffsets()`) exists in `src/firm/config/`, exporting
      sizes and per-field byte offsets for every generated C++ group
      struct from ticket 002.
- [x] A Python harness (mirroring `planner_harness.py`'s approach) loads
      the export via `ctypes` and walks it against the generated pydantic
      model's field list, asserting field count and relative ordering
      agree.
- [x] The harness passes against the current generated pair.
- [x] The harness is demonstrated to fail on an intentionally introduced
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

## Completion Notes

- `src/firm/config/config_parity_capi.{h,cpp}` (new): `extern "C"` export
  mirroring `capi.cpp`'s `plannerStructSizes()`/`plannerLimitsOffsets()`
  pattern exactly. Chose "one function taking a struct-id enum"
  (`ConfigParityGroup`, plain `uint32_t` at the ABI boundary) over one
  function per struct, since 7 near-identical offset functions would have
  been pure repetition. `configParityStructSizes(out, count)` returns all
  7 group `sizeof()`s; `configParityFieldOffsets(group, out, count)`
  returns that group's field count and per-field `offsetof()` values. Every
  offset is looked up by field NAME, so a field rename/deletion in
  `robot_config.h` fails this file's own compile, and a mid-struct
  insertion shifts every later offset automatically without this file
  needing to change.
- `src/tests/unit/test_config_parity_capi.py` (new): compiles
  `config_parity_capi.cpp` as a `HOST_BUILD` shared library via
  `subprocess` (the same ad hoc per-test-run compile convention every
  `src/tests/sim/unit/` harness already uses, just producing a `-shared`
  object instead of an executable) and loads it with `ctypes`, mirroring
  `planner_harness.py`'s `loadLibrary()`. Collected under
  `src/tests/unit/` (`pyproject.toml` `testpaths`), so it runs on normal
  `uv run python -m pytest` collection — 22 tests, all passing.
- **Design choice — no third hand-maintained field list.** Rather than
  hand-writing a second Python-side field-name list (which would itself be
  a third definition, the exact disease this design exists to kill), the
  harness builds a `ctypes.Structure` mirror PER GROUP dynamically from
  the real, checked-in `robot_radio.config.robot_config_generated`
  module's own `model_fields` (pydantic `int`/`float` annotations mapped
  to `ctypes.c_int32`/`c_float`, same declared order), then compares that
  mirror's `ctypes.sizeof()`/per-field `.offset` against the REAL C++
  values the capi export reports. Two independently generated artifacts
  (ticket 002's C++ struct emission and its pydantic emission) are
  compared directly against each other; nothing about field names/order is
  re-typed by hand anywhere in this ticket's own code.
- **Induced-failure check — actually performed, per the acceptance
  criterion.** Hand-edited the checked-in `src/firm/messages/robot_config.h`
  to insert `float induced_drift_field = 0.0f;` between `Geometry`'s
  `trackwidth` and `rotational_slip` fields, reran
  `test_config_parity_capi.py`: `test_struct_size_matches_pydantic_model
  [Geometry]` failed (`28 != 24`) and `test_field_offsets_match_pydantic_model
  [Geometry]` failed (`[0, 8, 12, 16, 20, 24] != [0, 4, 8, 12, 16, 20]`,
  concrete offset mismatch at index 1), while all 6 other groups' tests
  stayed green — confirming the guard is sensitive to exactly this group,
  not merely "something broke." Reverted the edit
  (`git diff src/firm/messages/robot_config.h` clean afterward) and reran
  the full file: 22/22 passing again.
- **Nothing left deliberately broken for a later ticket.** This ticket's
  own files are additive only (`config_parity_capi.{h,cpp}`, one new test
  file) — no existing file was modified, so there is nothing here to flag
  against the sprint's "mid-sprint breakage is expected" allowance. Not
  yet wired: `Configurator`/`Config::Robot` itself (tickets 005/006) and
  `check_config_sync.py`'s deletion (ticket 004) — both explicitly out of
  this ticket's scope, per the ticket's own Description.
