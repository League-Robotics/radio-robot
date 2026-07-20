---
id: '004'
title: Version-stamped persisted live-tuning store (Config::PersistedTuning, MicroBitStorage-backed)
status: open
use-cases: [SUC-003]
depends-on: ['001']
github-issue: ''
issue: config-as-truth-completion-no-defaults-fail-closed-version-erase.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Version-stamped persisted live-tuning store (Config::PersistedTuning, MicroBitStorage-backed)

## Description

New `Config::PersistedTuning` module: persists live-pushed `CFG` patches
(`MotorConfigPatch`, `PlannerConfigPatch`'s 5 curated fields,
`OtosConfigPatch`) across a power cycle via `MicroBitStorage` (mirroring
`com/radio_channel.h`'s existing precedent), stamped with a compiled
`kConfigSchemaVersion`. A version mismatch at boot wipes the entire store
instead of reapplying a patch whose field meanings may have changed.

## Context

See sprint.md's Design Rationale and Open Questions 1 and 3. This module
does **not** widen which fields are live-tunable — it only persists what a
`CFG` patch can already change (sprint 113's own Decision 1 rejected
widening the live wire surface; this ticket doesn't reopen that).
`com/radio_channel.h` is the only existing precedent for on-device flash
persistence in this codebase, but it is **ARM-only** — it `#include`s
`MicroBit.h` directly, has no `HOST_BUILD` variant, and is not exercised by
any host test today. Do not claim more host-testability for this module than
its own pure-function split earns (this was caught and corrected during
sprint 114's own architecture self-review).

**Flash-write frequency is a real risk, not yet resolved by sprint planning**
(sprint.md Open Question 3): a bench-tuning session can stream `CFG` patches
rapidly (e.g. a TestGUI slider). Writing flash unconditionally on every
patch risks both per-write latency inside a live control session and
page-wear over many sessions on a finite-endurance flash region shared with
`radio_channel.h`'s own key. **This ticket must choose and justify a write
policy** — e.g. debounce (write no more than once per N ms of quiescence),
or write-only-on-actual-change (compare against the last-persisted value,
skip an identical rewrite) — rather than writing unconditionally on every
patch. Document the choice inline at the call site.

## Approach

1. **Pure logic, host-testable, no `MicroBitStorage` dependency**: new
   `src/firm/config/persisted_tuning.h`/`.cpp`, `Config::` namespace:
   - A plain struct (or reuse of the existing `msg::MotorConfigPatch`/
     `PlannerConfigPatch`/`OtosConfigPatch` triple) representing "the
     currently-persistable patch state."
   - `std::array<uint8_t, N> serializePatch(...)` / a deserialize
     counterpart — pure functions, byte blob in/out, no I/O.
   - `bool shouldWipe(uint32_t storedVersion, uint32_t currentVersion)` —
     pure, trivial (`storedVersion != currentVersion`), but named and
     tested explicitly so the wipe *decision* is a single, greppable,
     unit-tested unit.
   - `constexpr uint32_t kConfigSchemaVersion = 1;` (check `src/firm/` for
     an existing build-version symbol before inventing a second one).

2. **Thin ARM-only I/O wrapper**: a small function (or two: `load
   (MicroBitStorage&) -> Opt<PatchBlob>`, `save(MicroBitStorage&, const
   PatchBlob&)`) mirroring `radiochan::load()/save()`'s exact shape —
   reads/writes the serialized blob plus its version stamp under a new,
   dedicated storage key (do not collide with `radiochan`'s own key). This
   part is **not** unit-tested by any agent (no `MicroBitStorage` available
   under `HOST_BUILD`) — covered only by the bench checklist (ticket 006).

3. **Write policy** (resolve Open Question 3 here): in
   `RobotLoop::handleConfig()`, after applying a patch in RAM as today, call
   into the persistence path — but debounce or change-detect first. A
   simple, defensible choice: track the last-persisted serialized blob in
   RAM; only call `save()` when the newly-serialized blob differs from it.
   Document why in a comment at the call site (this avoids a flash write for
   a patch that changes a field the persisted blob doesn't track, or that
   repeats an identical value — both real cases a slider-drag or a repeated
   push can produce).

4. **Boot sequence** (`main.cpp`, between the existing `Config::default*()`
   boot-bake calls and ticket 001's `markConfigured()` call): read the
   store; if `shouldWipe()` is true, wipe it and proceed on the boot-bake
   values alone; if false, deserialize and reapply the persisted patch
   through the *same* application path `handleConfig()` already uses (do
   not duplicate the merge-then-write logic — factor it out if needed so
   both the wire-triggered and boot-triggered paths share one applier).

## Files to Touch

- `src/firm/config/persisted_tuning.h`, `.cpp` (new)
- `src/firm/app/robot_loop.cpp` (`handleConfig()` — persist-on-change call)
- `src/firm/main.cpp` (boot-time read/wipe/reapply, sequenced before
  `markConfigured()`)

## Acceptance Criteria

- [ ] `serializePatch()`/deserialize and `shouldWipe()` are pure functions
      with zero `MicroBitStorage`/hardware dependency, unit-tested under
      `HOST_BUILD`.
- [ ] Version-match: a serialized-then-deserialized patch round-trips to
      identical field values (host-testable, no real flash).
- [ ] Version-mismatch: `shouldWipe()` returns true; the documented boot
      behavior is "wipe, proceed on boot-bake alone" — no partially-applied
      or misinterpreted stale patch.
- [ ] The write policy is NOT "write on every patch unconditionally" — a
      debounce or change-detection mechanism is implemented and documented
      inline with its rationale.
- [ ] The actual `MicroBitStorage` read/write call is isolated in a small,
      clearly-labeled ARM-only function, explicitly not covered by any
      agent-run test (comment says so, matching sprint.md's own honesty
      about this).
- [ ] Boot sequencing: persisted-tuning read/wipe/reapply happens after the
      Tier-1 boot bake and before `RobotLoop::markConfigured()`.

## Testing

- **Existing tests to run**: full `src/firm`/`src/tests` host-build suite
  (no regression to `handleConfig()`'s existing RAM-application behavior).
- **New tests to write**: `serializePatch`/deserialize round-trip test;
  `shouldWipe()` parametrized match/mismatch cases; a `handleConfig()`-level
  test confirming the debounce/change-detection policy actually skips a
  redundant persist call (mock or count-based assertion on the persistence
  seam, not real flash).
- **Verification command**: `uv run python -m pytest src/tests -v`
  (targeted to whatever new test files this ticket adds), then full suite.
- **Explicitly not verifiable by any agent**: the real `MicroBitStorage`
  flash round-trip, and "a reflash to a new firmware version actually wipes
  the store on real hardware" — both deferred to ticket 006's stakeholder
  bench checklist.
