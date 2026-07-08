---
id: '005'
title: Extract MainLoop::commit(bb, now)
status: in-progress
use-cases:
- SUC-005
depends-on:
- '004'
github-issue: ''
issue: mainloop-commit-phase-extract.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Extract MainLoop::commit(bb, now)

## Description

Extract the COMMIT block (`main_loop.cpp` ~262-276, shrunk by tickets
001-004 landing first) into a private `MainLoop::commit(Blackboard& bb,
uint32_t now)`, mirroring the already-landed `serviceWatchdogs()`
extraction (commit `0b2929c5`, "extract MainLoop's watchdog block").
`tick()` then reads as named phases: `serviceWatchdogs → control → plan →
commit → routeOutputs`. Do this LAST — the COMMIT block "shrinks for free"
once tickets 001/002/003 land (the per-port index loop moves into the
`bb.motors` resolution; the `odometer != nullptr` commit branch becomes
`bb.otosValid = odometer->fusableThisPass()`).

**Explicitly NOT a `Blackboard::update(...)` API** (architecture-update.md
Decision 5 — carried forward from the source issue's own rejected
alternative). `Blackboard` stays a pure DTO: no subsystem dependency, no
side-effecting method. `MainLoop` (the composition root) keeps owning the
wiring and commit ordering.

## Acceptance Criteria

- [ ] `Rt::MainLoop` gains a private `void commit(Blackboard& bb, uint32_t
      now)` method in `main_loop.{h,cpp}`, containing exactly the COMMIT
      block's logic (as shrunk by tickets 001-004 — the `bb.motors[]`
      copy loop, `bb.drivetrain`/`bb.encoderPose`/`bb.fusedPose`/
      `bb.planner` assignments, and the odometer tick/otos/otosValid
      assignment collapsed by ticket 003).
- [ ] `MainLoop::tick()`'s body calls `commit(bb, now)` at the exact point
      the inline block used to run; no behavior/ordering change relative
      to `routeOutputs()` and the periodic-telemetry emission that follow
      it.
- [ ] `Rt::Blackboard` gains NO new method — it remains a pure data struct
      with no subsystem dependency (verified by grep: no new `#include`
      of any `subsystems/*.h` beyond what it already includes for
      `kPortCount`/`Channel`, no new member function).
- [ ] `uv run python -m pytest tests/sim` is green, byte-identical
      COMMIT-step behavior (this is a pure code-motion ticket with no
      logic change of its own).

## Implementation Plan

**Approach**:
1. Confirm tickets 001-004 have landed first (this ticket's own diff is
   against their shrunk COMMIT block, not today's).
2. Cut the COMMIT block's body verbatim into a new private
   `MainLoop::commit(Blackboard& bb, uint32_t now)` method, declared in
   `main_loop.h` beside `serviceWatchdogs()`/`routeOutputs()` (same access
   level, same doc-comment style).
3. Replace the inline block in `tick()` with a single `commit(bb, now);`
   call at the same point.
4. Double-check no local variable the COMMIT block used (e.g. the
   pre-sprint `odometer`/`p` locals) needs to be threaded in as an
   additional parameter or recomputed inside `commit()` — since tickets
   001-003 already moved the port/odometer-null bookkeeping out of the
   surrounding code, this should be a clean cut, but verify at
   implementation time.

**Files to modify**: `source/runtime/main_loop.h`,
`source/runtime/main_loop.cpp`.

**Documentation updates**: update `main_loop.h`'s own file-header comment
(which currently describes `tick()`'s phase structure) to mention
`commit()` alongside `serviceWatchdogs()`/`routeOutputs()`.

## Testing

- **Existing tests to run**: full `tests/sim` — this ticket changes no
  logic, only code location, so any regression indicates a cut-and-paste
  error.
- **New tests to write**: none.
- **Verification command**: `uv run python -m pytest tests/sim`
