---
id: '004'
title: Version-stamped persisted live-tuning store (Config::PersistedTuning, MicroBitStorage-backed)
status: done
use-cases:
- SUC-003
depends-on:
- '001'
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

- [x] `serializePatch()`/deserialize and `shouldWipe()` are pure functions
      with zero `MicroBitStorage`/hardware dependency, unit-tested under
      `HOST_BUILD`.
- [x] Version-match: a serialized-then-deserialized patch round-trips to
      identical field values (host-testable, no real flash).
- [x] Version-mismatch: `shouldWipe()` returns true; the documented boot
      behavior is "wipe, proceed on boot-bake alone" — no partially-applied
      or misinterpreted stale patch.
- [x] The write policy is NOT "write on every patch unconditionally" — a
      debounce or change-detection mechanism is implemented and documented
      inline with its rationale.
- [x] The actual `MicroBitStorage` read/write call is isolated in a small,
      clearly-labeled ARM-only function, explicitly not covered by any
      agent-run test (comment says so, matching sprint.md's own honesty
      about this).
- [x] Boot sequencing: persisted-tuning read/wipe/reapply happens after the
      Tier-1 boot bake and before `RobotLoop::markConfigured()`.

## Completion Notes

**Files changed** (beyond the ticket's own Files-to-Touch list — see
"Structural addition" below):
- `src/firm/config/persisted_tuning.h`/`.cpp` (new) — pure
  `TuningSnapshot`/`serializeSnapshot()`/`deserializeSnapshot()`/
  `shouldWipe()`/`kConfigSchemaVersion`, the `Config::TuningStore` abstract
  seam, and the ARM-only `Config::MicroBitTuningStore` adapter (guarded
  `#ifndef HOST_BUILD`, mirroring `app/comms.h`'s `Transport`/
  `SerialTransport` split).
- `src/firm/app/robot_loop.h`/`.cpp` — `handleConfig()` refactored to
  merge each live patch into a running `persistedTuning_` snapshot and
  call `persistTuningIfChanged()`; `applyMotorConfigPatch()`/
  `applyOtosPatch()` factored out (verbatim extractions, no behavior
  change) so `reapplyPersistedTuning()` (new, public) and `handleConfig()`
  share one applier per patch kind, per the ticket's own Approach step 4.
- `src/firm/main.cpp` — constructs `Config::MicroBitTuningStore`, and adds
  the boot-time read/wipe-or-reapply sequence between the Tier-1 boot bake
  and `markConfigured()`.
- `src/sim/CMakeLists.txt` — adds `persisted_tuning.cpp` to the host/sim
  shared-lib source list (its pure half must link into the sim; its
  ARM-only half compiles out under `-DHOST_BUILD`).
- `src/tests/sim/unit/persisted_tuning_harness.cpp` + `test_persisted_tuning.py`
  (new) — the pure-logic host tests.
- `src/tests/sim/unit/app_robot_loop_harness.cpp` + `test_app_robot_loop.py`
  — new `MockTuningStore` + `scenarioConfigPersistWritePolicySkipsRedundantSave()`
  proving the debounce via a call-count assertion; `persisted_tuning.cpp`
  added to the harness's own compile source list.

**Structural addition beyond the ticket's literal Files-to-Touch list**:
the ticket listed only `robot_loop.cpp` (not `.h`). Executing Approach step
3 ("in `RobotLoop::handleConfig()` ... call into the persistence path")
turned out to require an actual persistence SEAM reachable from
`handleConfig()` — and `robot_loop.h`'s own file header states its
existing contract explicitly: "Plain virtual base (not an `#ifdef
HOST_BUILD` fork)" is this codebase's established pattern for exactly this
shape of problem (`Devices::Clock`/`Sleeper`, `App::Transport`). Since
`RobotLoop`/`robot_loop.cpp` compile under both `HOST_BUILD` (sim) and ARM
from the SAME translation unit (no `#ifdef` forks inside it), the
persistence call could not reach a `MicroBitStorage&` directly without
breaking the sim build. Resolution: `RobotLoop`'s constructor gained ONE
new, trailing, `= nullptr`-defaulted parameter (`Config::TuningStore*
tuningStore`) — every existing call site (`main.cpp`, and all 26
`TestSim::SimHarness`/harness construction sites) keeps compiling
unchanged; only `main.cpp` passes a real store. This mirrors the
already-precedented Decision 6 pattern from this same sprint (ticket 001's
`Motor::reconfigure()`): a surgical, additive, non-breaking fix to a
plan-time assumption that turned out structurally incomplete, not a
sprint-blocking exception. Also matches sprint.md's own explicit design
note: "`Config::PersistedTuning`/`MicroBitStorage` has no sim
counterpart... vacuous by construction" — a null store IS that vacuous
case, not a stub.

**Write-policy decision (Open Question 3)**: change-detection debounce.
`RobotLoop::persistTuningIfChanged()` (`robot_loop.cpp`) serializes the
current cumulative `persistedTuning_` snapshot on every live CFG patch and
compares it, byte-for-byte, against `lastPersistedBlob_` (the last blob
actually written); `tuningStore_->save()` is only called when they differ.
A patch that repeats an already-persisted value, or that touches a field
outside the persisted set, costs zero flash writes. Chosen over a
time-based debounce (e.g. "no more than once per N ms of quiescence")
because change-detection needs no timer/clock dependency, degrades to
exactly the same zero-redundant-write behavior for a rapid TestGUI-slider
session, and is directly count-assertable in a host test (no wall-clock
stepping needed) — see `app_robot_loop_harness.cpp`'s
`scenarioConfigPersistWritePolicySkipsRedundantSave()`.

**Host-tested vs. bench-only**: `serializeSnapshot()`/`deserializeSnapshot()`/
`shouldWipe()` and the `Config::TuningStore` seam (via `MockTuningStore`)
are exercised by `persisted_tuning_harness.cpp` and
`app_robot_loop_harness.cpp` respectively, both under `-DHOST_BUILD`, zero
`MicroBitStorage` dependency. `Config::MicroBitTuningStore` (the real
`codal::KeyValueStorage`-backed adapter, `persisted_tuning.cpp`'s own
`#ifndef HOST_BUILD` block) is NOT exercised by any agent-run test — no
`MicroBitStorage` stand-in exists under `HOST_BUILD` anywhere in this
tree, matching sprint.md's own architecture note. It chunks the
version+blob payload across up to 4 fixed 32-byte `codal::KeyValueStorage`
keys (`kNumChunks` `static_assert`s ≤ 4, leaving 1 of the class's 5-key-total
budget for `com/radio_channel.h`'s own key) and its `wipe()` calls the
real whole-store `codal::KeyValueStorage::wipe()` (also erasing
`radiochan`'s key — an accepted, documented consequence of SUC-003's own
"the entire store is wiped" language, not a bug). Both are deferred to
ticket 006's stakeholder bench checklist, per this ticket's own Testing
section.

**pytest**: `uv run python -m pytest src/tests/sim/unit src/tests/unit -q`
— all passing; `test_app_robot_loop.py`'s single test function carries a
PRE-EXISTING `xfail(strict=False)` (111-002, the parked `drive_.tick()`
cycle-order experiment — see `.clasi/knowledge/`) that this ticket did not
introduce and does not touch; the new
`scenarioConfigPersistWritePolicySkipsRedundantSave()` scenario is subject
to the same pre-existing quarantine (it also drives `cycle()` against a
hand-scripted bus, the same mechanism the xfail already covers) but its own
functional assertions (gains applied, `saveCount()==1`) passed even where
the unrelated bus-script-order assertions the xfail already covers did not.

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
