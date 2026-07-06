---
id: '004'
title: 'Go-to verb: G'
status: open
use-cases: [SUC-003]
depends-on: ['003']
github-issue: ''
issue: firmware-closed-loop-motion-verbs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Go-to verb: G

## Description

Register `G <x> <y> <speed>` — relative-XY go-to navigation — as a
top-level verb in `source/commands/motion_commands.*`, already documented
in `docs/protocol-v2.md` §10 (`### G`). Ported from `source_old/
superstructure/Planner.h`'s `beginGoTo()`/`GPhase` (`IDLE`/`PRE_ROTATE`/
`PURSUE`) state machine, adapted to `Subsystems::Planner::tick()`'s
`fusedPose` argument (already threaded through since ticket 001; `TURN`
already proved the fused-heading feedback path in ticket 003 — `G` is the
first verb needing the fused **position**, not just heading).

Pre-rotate engages only when the initial bearing error to the target
exceeds `PlannerConfig.turn_in_place_gate`; the pursue phase then drives
toward the target, completing when within `PlannerConfig.arrive_tol` mm.
Both fields already exist in `msg::PlannerConfig` (architecture-update.md
Grounding fact 1) — no message change needed.

**Wire keys stay stable.** `G`'s verb token, argument shapes (`x`/`y`
±10000 mm, `speed` 1-1000 mm/s), and `OK goto ...`/`EVT done G` reply text
are exactly as already documented in `docs/protocol-v2.md` §10 — this
ticket implements that existing contract without renaming or reshaping
it.

## Acceptance Criteria

- [ ] `G <x> <y> <speed>` registered, matching `docs/protocol-v2.md`
      §10's existing wire shape and range checks exactly.
- [ ] Pre-rotate phase engages only when the initial bearing error exceeds
      `PlannerConfig.turn_in_place_gate`; otherwise pursue starts
      immediately.
- [ ] Pursue phase drives toward the relative target using `fusedPose`,
      completing (emitting `EVT done G`) when within `PlannerConfig.
      arrive_tol` mm of the goal.
- [ ] `G 300 0 200` drives to the relative point and emits `EVT done G`
      (sim).
- [ ] `G` accepts no `stop=` clauses beyond what `docs/protocol-v2.md`
      already documents for it (none, per the existing §10 text) — no
      scope creep beyond the documented contract.
- [ ] No `Planner`/`Drivetrain`/`PoseEstimator` signature changes beyond
      what tickets 001-003 already established (`G` is the first real
      consumer of `fusedPose`'s position component, not a new argument).

## Implementation Plan

**Approach:** Port the `GPhase` state machine (`IDLE`/`PRE_ROTATE`/
`PURSUE`) into `Subsystems::Planner`'s existing goal-dispatch (ticket
001's `apply()`/`tick()` already has a `goal_kind` switch with a `GOTO_GOAL`
arm reserved from the schema — this ticket is the first to implement it
meaningfully). The bearing/arrival math is pure geometry against
`fusedPose`, ported from `source_old/superstructure/Planner.cpp`'s
`driveAdvance()` G-phase branch.

**Files to modify:**
- `source/subsystems/planner.cpp` (implement the `GOTO_GOAL` `apply()`/
  `tick()` arms — pre-rotate/pursue phasing)
- `source/commands/motion_commands.h`, `source/commands/
  motion_commands.cpp` (register `G`)

**Testing plan:**
- Sim-level tests: `G 300 0 200` reaches the target and emits
  `EVT done G`; pre-rotate engages/doesn't engage at bearing errors
  above/below `turn_in_place_gate`; arrival tolerance honored at
  `arrive_tol`'s boundary.
- Existing suites stay green.

**Documentation updates:** `docs/protocol-v2.md` §10's existing `### G`
section needs no wire-shape change (already accurate) — confirm during
implementation and note any discrepancy found between the documented text
and the ported behavior explicitly, rather than silently reconciling one
against the other.
