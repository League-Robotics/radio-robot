---
id: '005'
title: Extract MainLoop::commit(bb, now)
status: done
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

- [x] `Rt::MainLoop` gains a private `void commit(Blackboard& bb, uint32_t
      now)` method in `main_loop.{h,cpp}`, containing exactly the COMMIT
      block's logic (as shrunk by tickets 001-004 — the `bb.motors[]`
      copy loop, `bb.drivetrain`/`bb.encoderPose`/`bb.fusedPose`/
      `bb.planner` assignments, and the odometer tick/otos/otosValid
      assignment collapsed by ticket 003).
- [x] `MainLoop::tick()`'s body calls `commit(bb, now)` at the exact point
      the inline block used to run; no behavior/ordering change relative
      to `routeOutputs()` and the periodic-telemetry emission that follow
      it.
- [x] `Rt::Blackboard` gains NO new method — it remains a pure data struct
      with no subsystem dependency (verified by grep: no new `#include`
      of any `subsystems/*.h` beyond what it already includes for
      `kPortCount`/`Channel`, no new member function).
- [x] `uv run python -m pytest tests/sim` is green, byte-identical
      COMMIT-step behavior (this is a pure code-motion ticket with no
      logic change of its own).

## Completion notes

Extraction landed: `MainLoop::commit()` now holds the COMMIT block, and
`tick()` reads as the named phase sequence `serviceWatchdogs → hardware →
control/plan → commit → routeOutputs`. The rejected `Blackboard::update()`
API was NOT introduced; `Blackboard` remains a pure DTO. The odometer
`tick(now)` side-effect stays inside `commit()`, unchanged and unreordered.

**Deviation (minor, justified):** the method signature is
`commit(Blackboard& bb, uint32_t now, bool otosFusableThisPass)` rather than
the literal `commit(bb, now)` in the AC text. The COMMIT block's
`bb.otosValid = otosFusableThisPass` needs the read-once fusability value
computed earlier in `tick()` (at the single `fusableThisPass()` call site);
threading it as a parameter preserves ticket 003's single-call invariant,
whereas re-calling `fusableThisPass()` inside `commit()` would wrongly clear
the one-pass reset transient and reintroduce the stale-OTOS EKF bug. The AC's
intent (a private `commit` phase method containing the COMMIT logic) is met.

**Verification:** `uv run python -m pytest tests/sim` → 309 passed, 2 xfailed
(matches the sprint baseline). Sim builds clean; extraction diff reviewed as a
pure cut-paste (no reordering/logic change).

**Note on completion:** the programmer agent wrote and verified-built the
extraction but returned before committing/marking done (it had kicked off a
background test run and stopped awaiting its result). The team-lead re-ran the
sim gate (green), reviewed the diff, and completed the ticket bookkeeping.

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
