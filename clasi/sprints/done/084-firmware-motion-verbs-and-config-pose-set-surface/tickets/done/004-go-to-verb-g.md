---
id: '004'
title: 'Go-to verb: G'
status: done
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

- [x] `G <x> <y> <speed>` registered, matching `docs/protocol-v2.md`
      §10's existing wire shape and range checks exactly.
- [x] Pre-rotate phase engages only when the initial bearing error exceeds
      `PlannerConfig.turn_in_place_gate`; otherwise pursue starts
      immediately.
- [x] Pursue phase drives toward the relative target using `fusedPose`,
      completing (emitting `EVT done G`) when within `PlannerConfig.
      arrive_tol` mm of the goal.
- [x] `G 300 0 200` drives to the relative point and emits `EVT done G`
      (sim).
- [x] `G` accepts no `stop=` clauses beyond what `docs/protocol-v2.md`
      already documents for it (none, per the existing §10 text) — no
      scope creep beyond the documented contract.
- [x] No `Planner`/`Drivetrain`/`PoseEstimator` signature changes beyond
      what tickets 001-003 already established (`G` is the first real
      consumer of `fusedPose`'s position component, not a new argument).

## Implementation Notes (closing)

- Implemented in `source/subsystems/planner.{h,cpp}` (`GPhase` state
  machine + `enterPursue()`/`pursueSteer()`), `source/commands/
  motion_commands.{h,cpp}` (`G` verb registration), and
  `source/dev_loop.cpp` (`motionVerbForMode()` extended to map
  `DriveMode::GO_TO` -> `"G"`, exactly the extension that function's own
  084-002 comment already anticipated for this ticket).
- **Doc discrepancy found, not silently reconciled**: `docs/protocol-v2.md`
  §10's own `### G` example section shows a bare `EVT done G` with no
  `reason=` token, but the same section's general "`reason=` field"
  convention (and its own reason-token table) documents `pos` as G's
  reason token, and every other verb's own example section (`T`/`D`/`R`/
  `TURN`/`RT`) *does* show `reason=<token>`. The implementation follows
  the general/uniform convention (`EVT done G reason=pos`) since
  `dev_loop.cpp`'s EVT-emission path has no per-verb exception to omit
  `reason=` for any verb. The `### G` example text itself was left
  unedited (out of this ticket's scope to rewrite protocol docs) but
  should be corrected in a follow-up.
- A real bug was found and fixed during implementation: the PURSUE
  per-tick re-steer hook (`pursueSteer()`) must be gated on `!stopping_`
  — without that guard it kept re-targeting the ramp away from zero every
  tick after the POSITION stop fired, so the robot never actually stopped
  after "`EVT done G`". Confirmed fixed via the sim library.
- Measured (sim, 2026-07-06): `G 300 0 200` fires `EVT done G reason=pos`
  at x≈302 mm (well within the default 25 mm `arrive_tol`), then a
  pre-existing sprint-081 `velocity_pid.cpp` zero-crossing settle (same
  mechanism already documented for `TURN`/`RT` in
  `test_motion_commands_arc_turn.py`) creeps the plant back to a final
  rest of x≈262 mm by ~6 s — reproduced identically with a plain `D`, so
  this is not a `G`-specific issue and is out of this ticket's scope to
  fix.

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
