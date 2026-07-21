---
id: '004'
title: Persisted-tuning schema bump (v1 to v2, 85-byte blob)
status: done
use-cases:
- SUC-049
depends-on:
- '003'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Persisted-tuning schema bump (v1 to v2, 85-byte blob)

## Description

`Config::PersistedTuning`'s `TuningSnapshot` (`src/firm/config/
persisted_tuning.h:83-88`) carries `msg::PlannerConfigPatch planner`,
which ticket 003 deletes from `config.proto`. This ticket removes that
field from the snapshot struct and — **mandatorily, in the same
commit** — bumps `kConfigSchemaVersion` from 1 to 2 (sprint.md
Architecture Decision 3). Without the bump, `deserializeSnapshot()`
would read the old 110-byte blob's now-shifted bytes into the new
85-byte layout, silently corrupting the OTOS-calibration section with
what used to be the planner term's floats — memory-safe but behaviorally
wrong. With the bump, a version mismatch at boot triggers a clean wipe
via `shouldWipe()`/`MicroBitTuningStore::wipe()` instead.

Byte math (verified during planning, both formulas are computed
constants, not magic numbers): `kBlobSize = (2 × kMotorPatchFields ×
kOptFloatBytes) + (kPlannerPatchFields × kOptFloatBytes) + (kOtosPatchFields
× kOptFloatBytes)`. Today: `(2×6×5) + (5×5) + (5×5) = 110`. Dropping the
middle term: `110 − 25 = 85`. `kNumChunks = ceil((4 + kBlobSize) / 32)`
(persisted_tuning.cpp:157-158): today `ceil(114/32) = 4`; after:
`ceil(89/32) = 3`.

**Known, expected, documented side effect** (not a bug): the version
mismatch wipes the ENTIRE `KeyValueStorage` (`kNumChunks<=4`-bounded, 5
keys total shared with the radio-channel key per
`persisted_tuning.h:165-167`'s own comment on `KEY_VALUE_STORAGE_MAX_PAIRS`),
so the FIRST boot on new firmware also loses the persisted radio
channel, producing a one-time re-pick. This belongs in the bench
checklist (ticket 010), not treated as a regression if observed.

## Acceptance Criteria

- [x] `TuningSnapshot` (persisted_tuning.h) no longer has a `planner`
      member.
- [x] `kMotorPatchFields`/`kOtosPatchFields` constants unchanged;
      `kPlannerPatchFields` constant and its use in `kBlobSize`'s
      computation removed (not just zeroed — the term itself is
      deleted, so `kBlobSize` computes to 85 from the remaining terms,
      not from a magic 85).
  - [x] `kConfigSchemaVersion` changed from `1` to `2` in the SAME
      commit as the `TuningSnapshot` field removal — never split across
      two commits (Decision 3's own hard requirement).
- [x] `serializeSnapshot()`/`deserializeSnapshot()` updated to match the
      new field set; round-trip property preserved
      (`deserializeSnapshot(serializeSnapshot(s))` reproduces `s`'s
      field values) for the 85-byte layout.
- [x] `persisted_tuning_harness.cpp` / `test_persisted_tuning.py`
      updated: assertions on the old 110-byte/4-chunk layout replaced
      with 85-byte/3-chunk assertions; a `shouldWipe(1, 2) == true` test
      case added (or confirmed already covers the general
      version-mismatch case, in which case no new case is needed —
      verify rather than assume).
- [x] `static_assert(kNumChunks <= 4, ...)` (persisted_tuning.cpp:160)
      still compiles (3 ≤ 4 — this should be a non-event, included here
      only so the ticket doesn't silently skip checking it).

## Implementation Plan

**Approach**: Edit `persisted_tuning.h` first (struct + constants), then
`persisted_tuning.cpp` (serialize/deserialize bodies), then the test
harness. This ticket's files have zero overlap with ticket 005's files
(main.cpp's `MicroBitTuningStore` construction line is unaffected in
shape) — safe to implement independently of 005 despite executing
before it in ticket order.

**Files to modify**: `src/firm/config/persisted_tuning.h`,
`src/firm/config/persisted_tuning.cpp`,
`src/tests/sim/unit/persisted_tuning_harness.cpp`,
`src/tests/sim/unit/test_persisted_tuning.py`.

**Testing plan**: `uv run python -m pytest src/tests/sim/unit/test_persisted_tuning.py`
green, covering: round-trip for the 85-byte layout; `shouldWipe`
version-mismatch behavior; chunk-count math (3, not 4). Hardware
confirmation (the wipe + radio re-pick + power-cycle persistence
sequence) is ticket 010's job, not this one's — this ticket is
host-testable only (persisted_tuning.h's own header comment: "pure, no
I/O, no MicroBitStorage dependency").

**Documentation updates**: `persisted_tuning.h`'s own header comment
(lines 1-33) already documents the version-bump discipline in general
terms — update it only if this ticket's specific 110→85/4→3 numbers are
worth citing inline (optional, not required).
