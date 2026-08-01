---
id: '012'
title: 'Firmware: move TLM mode switch into Telemetry, STATUS projection into Comms'
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue:
- tlm-mode-switch-belongs-in-telemetry-not-the-loop.md
- status-projection-belongs-in-comms-not-the-loop.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: move TLM mode switch into Telemetry, STATUS projection into Comms

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Clean-build requirement**: this ticket touches shared per-cycle state
handling (`Telemetry`/`Comms` gain new entry points read by
`RobotLoop::tick()`). Build with `just build-clean`, not incremental — a
stale incremental build after this kind of change produces a boot
HardFault indistinguishable from power loss.

## Description

Two adjacent blocks in `RobotLoop::tick()` do another module's policy
job. The `TlmAction`→`setMode()` switch (`robot_loop.cpp`, ~541-556 at
verification time) decides what a `TLM:ON`-family token means to the
telemetry emitter — that's `Telemetry`'s business, not the loop's. The
`Comms::Status` hand-assembly (~574+) projects `RobotState` onto the
STATUS reply shape field-by-field — a `Comms` concern, the same shape of
problem `Telemetry::update()` already solves correctly for its own frame.
Both blocks are adjacent and share an ordering constraint, so this ticket
does them together per both source issues' own note that doing so is
cheaper than doing either alone.

## Acceptance Criteria

- [x] `Telemetry::applyAction(Comms::TlmAction)` (or equivalent) added,
      absorbing the mode-change switch AND the "is this a force-a-frame
      request" answer — `emit()` may lose its `force` parameter if
      `Telemetry` latches a pending one-shot request internally.
      (Implemented: `applyAction()` returns the force-frame bool rather
      than latching it internally — `emit()`'s existing, separately
      tested `force` parameter is unchanged; only where the decision is
      computed moved. Choice documented in `telemetry.h`'s own doc
      comment on `applyAction()`.)
- [x] The `TlmAction`/dependency-direction question is settled explicitly
      in code (either `Telemetry` takes `Comms::TlmAction` as a parameter,
      accepting the dependency, or the enum moves to a shared/telemetry-owned
      header, or `Telemetry` exposes three mode setters + `requestFrame()`
      for `Comms` to call) — pick one, document the choice in the function's
      own header comment, don't leave it ambiguous.
      (`Telemetry` takes `Comms::TlmAction` by value — `comms.h` was
      already an unconditional dependency of `telemetry.h`, so this adds
      no new edge. Documented in `applyAction()`'s own doc comment,
      telemetry.h.)
- [x] `Comms::updateStatus(state_, tlm_)` (or equivalent) added, absorbing
      the 8-field STATUS projection. The two telemetry-sourced fields
      (`flags`, `tlmMode`) are either passed in explicitly, or published
      into `RobotState` during `Telemetry::update()` so the projection
      reads one source (prefer the latter if it doesn't cost meaningful
      state footprint — matches the "one blackboard" pattern the rest of
      the loop already uses).
      (Implemented: `flags`/`tlmMode` passed explicitly via a `const
      Telemetry&` parameter — `comms.h` forward-declares `Telemetry` to
      avoid the include cycle `telemetry.h`'s own `#include "app/comms.h"`
      would otherwise create; publishing onto `RobotState` was rejected as
      blurring that struct's dependency-free/cross-tree-shared boundary
      for a one-caller convenience — see `updateStatus()`'s own doc
      comment, comms.h.)
- [x] `status.ready = true` (the one genuinely loop-owned fact — "we are
      past `boot()`") moves into `state_` (e.g. a lifecycle/health flag
      set once at the end of `boot()`), not hard-coded at the projection
      site.
      (`Types::RobotState::Health::ready`, set once at the tail of
      `RobotLoop::boot()`, right after `comms_.sendReady()`.)
- [x] `RobotLoop::tick()` shrinks to calling both new entry points, in
      the same relative order as today: TLM-action application, THEN
      STATUS refresh (so a same-cycle mode change is already visible in
      STATUS), THEN `comms_.sendTlmReply(tlmAction)` (which reports the
      mode just applied) — this ordering constraint is preserved exactly,
      not just approximately.
- [x] STATUS projection stays unconditional — it must run even when the
      idle gate suppressed the telemetry frame, since answering STATUS on
      a parked robot is the case STATUS exists for.
      (`comms_.updateStatus(state_, tlm_)` runs unconditionally every
      cycle, same as the code it replaced.)
- [x] `grep -n "TlmAction\|status\." src/firm/app/robot_loop.cpp` no
      longer shows a switch statement or an 8-line field-by-field STATUS
      assembly.
      (Verified: the only match is the `const Comms::TlmAction tlmAction =
      comms_.takeTlmAction();` declaration.)
- [x] `app_comms_harness.cpp` and `app_telemetry_harness.cpp` assertions
      that exercised the mode-application/STATUS-projection logic via a
      full loop tick are moved to drive the new `Telemetry`/`Comms` entry
      points directly with a synthesized `RobotState` — the parsing
      assertions (wire text → `TlmAction`) stay where they are.
      (Neither unit harness ever drove this logic via a full `RobotLoop`
      tick — both already exercised `Comms`/`Telemetry` directly; the
      only full-loop-tick TLM coverage,
      `src/tests/sim/system/robot_loop_tlm_harness.cpp`, is a genuine
      black-box wire-level acceptance test, unaffected by this internal
      refactor, and stays as-is. New direct-entry-point tests added per
      the Testing section below: `scenarioApplyActionCoversEveryTlmActionArm`
      in app_telemetry_harness.cpp,
      `scenarioUpdateStatusProjectsAllEightFieldsFromSynthesizedState` +
      `scenarioUpdateStatusReadyFalseBeforeBoot` in app_comms_harness.cpp.)

## Testing

- **Existing tests to run**: `src/tests/sim/unit/app_comms_harness.cpp`,
  `app_telemetry_harness.cpp` (via their pytest wrappers), full firmware
  pytest tiers.
- **New tests to write**: a `Telemetry::applyAction()` unit test covering
  all `TlmAction` arms (`kSetOff`/`kSetAuto`/`kSetOn`/`kNone`/`kFrame`/
  `kUnrecognized`); a `Comms::updateStatus()` unit test asserting the
  8-field projection from a synthesized `RobotState`, including the
  `ready`/`flags`/`tlmMode` handling.
- **Verification command**: `just build-clean`, then
  `uv run python -m pytest src/tests/sim -k "comms or telemetry" -q`.

## Implementation Notes

- **Approach**: add the two new entry points and their tests first
  (against a synthesized `RobotState`, no full loop needed), confirm
  behavior matches today's inline logic, THEN shrink `RobotLoop::tick()`
  to call them — this order lets the new tests catch a mismatch before
  the old inline code is deleted.
- **Files to modify**: `src/firm/app/robot_loop.cpp`,
  `src/firm/app/telemetry.{h,cpp}`, `src/firm/app/comms.{h,cpp}`,
  `src/tests/sim/unit/app_comms_harness.cpp`,
  `src/tests/sim/unit/app_telemetry_harness.cpp`.
- **Documentation updates**: `src/firm/app/DESIGN.md` — note that
  `Telemetry`/`Comms` now own their own action-application/projection
  entry points rather than the loop assembling them inline.
