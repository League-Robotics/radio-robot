---
id: 090
title: 'MainLoop cohesion cleanup: Drivetrain port resolution, odometer reset/fusability
  + NullOdometer, subsystem events to replies, commit-phase extraction'
status: ticketing
branch: sprint/090-mainloop-cohesion-cleanup-drivetrain-port-resolution-odometer-reset-fusability-nullodometer-subsystem-events-to-replies-commit-phase-extraction
use-cases: []
issues:
- drivetrain-owns-motor-observation-resolution.md
- odometer-owns-reset-and-fusability.md
- null-odometer-object.md
- subsystem-events-to-replies.md
- mainloop-commit-phase-extract.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 090: MainLoop cohesion cleanup: Drivetrain port resolution, odometer reset/fusability + NullOdometer, subsystem events to replies, commit-phase extraction

## Goals

Restore cohesion to `source/runtime/main_loop.cpp` by moving five pieces of
subsystem-domain logic that leaked into the loop back to the modules that
own the underlying knowledge — with **zero observable behavior change**.
This is a pure internal-quality sprint: no new wire verb, no new EVT, no
changed motion behavior. Every ticket's acceptance bar is "the sim gate is
green and nothing a wire client can observe is different."

Five pool issues, extracted from the 2026-07-07 stakeholder design
discussion (`sprint-089-omnibus.md`), all behavior-preserving refactors of
`main_loop.cpp` and its collaborators:

1. `clasi/issues/drivetrain-owns-motor-observation-resolution.md` — move the
   `p.left - 1` port→cell resolution into `Drivetrain::tick()`; rename
   `bb.motor` → `bb.motors`.
2. `clasi/issues/odometer-owns-reset-and-fusability.md` — move the
   `SetPose→Pose2D→OdometerCommand` translation and the
   `odometerResetThisPass` fusability decision into the odometer
   (`applySetPose()` / `fusableThisPass()`). Preserves the live-debugged
   stale-OTOS EKF fix (skip fusion for exactly the one pass a reset lands).
3. `clasi/issues/null-odometer-object.md` — a `NullOdometer` so
   `hardware_.odometer()` never returns null; collapse the null branches
   (widened, in this sprint's own architecture-update.md, beyond the
   issue's own stated scope — see Decision 3).
4. `clasi/issues/subsystem-events-to-replies.md` — a typed `msg::Event` +
   one wire-layer `CommandProcessor::emitEvent()` owning all `EVT`
   formatting; subsystems expose `hasEvent()/takeEvent() → msg::Event`.
5. `clasi/issues/mainloop-commit-phase-extract.md` — extract the COMMIT
   block into a private `MainLoop::commit(bb, now)`, mirroring the
   already-landed `serviceWatchdogs()` precedent (commit `0b2929c5`).

## Problem

`MainLoop::tick()` (the ordered-tick cyclic executive established by
sprints 060/087) has accumulated logic that belongs to the subsystems it
orchestrates rather than to sequencing itself: bare port-index arithmetic
that only Drivetrain's own port binding should know; the odometer
reset-translation and stale-reading fusability gate; three `!= nullptr`
branches around `Hal::Odometer*`; hand-assembled `EVT` wire-text
`snprintf`s; and a long inline COMMIT block. None of this is a bug — it is
a cohesion debt that makes the loop harder to read as the one place you
reason about tick-pass sequencing and x[k]/commit timing.

## Solution

Five small, serially-ordered tickets (they all touch `main_loop.cpp`, so
they execute one at a time regardless of how independent their underlying
concerns are): Drivetrain absorbs its own port resolution; the odometer
absorbs reset translation and a fusability query; a `NullOdometer` retires
the nullable-pointer contract; a general `msg::Event`/`emitEvent()`
mechanism retires ad hoc `snprintf` wire-text assembly (generalizing what
Planner already does with `hasEvent()/takeEvent()`); and the COMMIT block
becomes a named private method. See `architecture-update.md` for the full
design, module boundaries, and diagrams.

## Success Criteria

- `uv run python -m pytest tests/sim` is green after every ticket.
- No wire-observable behavior changes: reply text, EVT text, TLM fields,
  ERR codes, and timing-sensitive behavior (the stale-OTOS-skip-one-pass
  EKF fix, the S-vs-R stream-watchdog gate) are bit-for-bit identical
  before and after.
- `MainLoop::tick()` reads as a sequence of named phases:
  `serviceWatchdogs → control → plan → commit → routeOutputs`.
- `bb.otosValid`/reset-fusability behavior is proven unchanged by the
  existing SI/OZ/OR/OV regression tests (not just re-read — actually run
  green before and after ticket 002/003).

## Scope

### In Scope

- `source/runtime/main_loop.{h,cpp}` (every ticket touches this file).
- `source/subsystems/drivetrain.{h,cpp}` (ticket 001).
- `source/runtime/blackboard.h` (`bb.motor` → `bb.motors` rename, ticket 001).
- `source/hal/capability/odometer.h` and every concrete `Hal::Odometer` leaf
  (`sim_odometer.{h,cpp}`, `otos_odometer.{h,cpp}`) (tickets 002/003).
- A new `Hal::NullOdometer` (ticket 003).
- `source/subsystems/hardware.h` and its two concrete owners' `odometer()`
  overrides (ticket 003 — widened scope, see architecture-update.md).
- `source/main.cpp` (`bb.otosPresent` boot snapshot) and
  `source/runtime/configurator.cpp` (odometer config null-guard) — both
  widened into ticket 003, see architecture-update.md Decision 3.
- `source/messages/` (new `msg::Event`), `source/commands/
  command_processor.{h,cpp}` (`emitEvent`), `source/subsystems/
  planner.{h,cpp}` (retyped `Event`, carried verb), `protos/planner.proto`
  (new `verb` field on `PlannerCommand`) (ticket 004).

### Out of Scope

- Any new wire verb, EVT name, or TLM field.
- Making the odometer a first-class ordered-tick subsystem with its own
  blackboard slice (noted by issue 2 as a possible future sprint).
- A full blackboard output-event queue (issue 4's "possible bigger
  version," explicitly not folded in).
- `Blackboard::update(...)` — explicitly rejected by issue 5.
- Any change to Planner's Ruckig/JerkTrajectory motion generation (sprint
  089's scope) beyond retyping its `Event`/carrying a `verb` field.

## Test Strategy

Every ticket's gate is `uv run python -m pytest tests/sim` (green,
~3–13 min). Ticket 001 additionally requires a repo-wide grep for
`bb.motor`/`.motor[` call sites (not just the issue's stated scope) per
this project's own "rename sprint: latent call-site breakage" lesson.
Ticket 002/003 additionally require the SI/OZ/OR/OV regression tests to be
run and confirmed green both before and after the change (not inferred) —
this is the load-bearing EKF-fix-preservation gate. Ticket 004 requires the
existing EVT-format tests (watchdog-fire, motion-done, safety_stop) to
produce byte-identical wire text before/after. Ticket 005 is a pure
internal extraction with no new test surface beyond the sim gate.

## Architecture Notes

See `architecture-update.md` for the full design: module responsibilities,
component/dependency diagrams, the `msg::Event` data-shape sketch, and five
design-rationale decisions (including two scope widenings beyond the raw
issue text, found during this sprint's own codebase-alignment review).

## GitHub Issues

(None — this sprint's work items are `clasi/issues/*.md` pool issues, not
GitHub issues.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed (self-review verdict: APPROVE WITH CHANGES — see
      architecture-review gate notes)
- [x] Stakeholder has approved the sprint plan (gate recorded 2026-07-08:
      "Auto-approved under stakeholder-directed autonomous mode... Proceed
      to ticketing.")

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Drivetrain owns motor-observation port resolution | — |
| 002 | Odometer owns reset translation and per-pass fusability | 001 |
| 003 | NullOdometer — collapse the nullable Hardware::odometer() contract | 002 |
| 004 | Subsystem events to replies — msg::Event + CommandProcessor::emitEvent | 003 |
| 005 | Extract MainLoop::commit(bb, now) | 004 |

Tickets execute serially in the order listed (all five touch
`source/runtime/main_loop.cpp`, so they must land one at a time regardless
of how independent their underlying concerns are).
