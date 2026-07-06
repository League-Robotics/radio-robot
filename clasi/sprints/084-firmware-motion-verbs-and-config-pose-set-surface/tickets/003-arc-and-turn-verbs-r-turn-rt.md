---
id: '003'
title: 'Arc and turn verbs: R / TURN / RT'
status: done
use-cases:
- SUC-002
depends-on:
- '002'
github-issue: ''
issue: firmware-closed-loop-motion-verbs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Arc and turn verbs: R / TURN / RT

## Description

Register `R <speed> <radius>` (constant-curvature arc, open-loop),
`TURN <heading> [eps=<cdeg>]` (absolute-heading turn-in-place, closed-loop
against fused heading), and `RT <relAngle>` (relative turn-in-place,
closed-loop against per-wheel encoder arc) as top-level verbs in
`source/commands/motion_commands.*` (ticket 002's family), staging
`msg::PlannerCommand`s into `Subsystems::Planner` exactly like `S`/`T`/`D`.

**These three verbs are undocumented in `docs/protocol-v2.md` today**
(architecture-update.md Grounding fact 5 — confirmed by grepping every
`### ` heading in the doc). This ticket derives their wire shape from
`source_old/commands/MotionCommands.cpp` (`parseR`/`handleR`/`parseTURN`/
`handleTURN`/`handleRT`) since the spec itself is incomplete, and **writes
the missing `docs/protocol-v2.md` sections** as part of this ticket — not
deferred to ticket 009.

`R` is open-loop (matches `S`'s family: runs until stopped or a `stop=`
clause fires — computed as `omega = speed / radius`, `v = speed`, handed to
`Planner` as a `VelocityGoal`). `TURN`/`RT` are always self-terminating
(implicit `HEADING`/`ROTATION` stop conditions respectively, from ticket
001's `Motion::evaluateStopCondition`) and need `Planner::tick()`'s
`fusedPose` argument for heading feedback — the first verbs in this sprint
to actually consume it.

**Wire keys stay stable** for every verb this sprint already registered
(002) — this ticket only *adds* `R`/`TURN`/`RT`/`stop=` support, matching
the argument shapes and range checks in `source_old/commands/
MotionCommands.cpp` (`speed`/`radius` ±1000/±10000; `heading` ±18000 cdeg;
`eps` 10-1800 cdeg, default 300) since there is no existing new-tree wire
contract for these three to preserve — this ticket **creates** that
contract, once, here.

## Acceptance Criteria

- [x] `R <speed> <radius> [stop=...]` registered: computes
      `omega = speed/radius` (0 when `radius == 0`), stages a
      `VelocityGoal`; open-loop, runs until stopped/`stop=` fires; ranges
      `speed` ±1000 mm/s, `radius` ±10000 mm.
- [x] `TURN <heading> [eps=<cdeg>] [stop=...]` registered: absolute-heading
      turn-in-place, closes against `Planner::tick()`'s `fusedPose`
      heading argument; completes within `eps` (default 300 cdeg);
      `heading` range ±18000 cdeg, `eps` range 10-1800 cdeg.
- [x] `RT <relAngle> [stop=...]` registered: relative turn-in-place,
      closes against per-wheel encoder arc (a `ROTATION` stop condition,
      `Motion::evaluateStopCondition`'s existing kind from ticket 001).
- [x] `RT 9000` rotates ~90° (within plant tolerance, sim) and emits
      `EVT done RT reason=<token>`; `TURN <heading>` reaches the commanded
      absolute heading within `eps`; `R <speed> <radius>`'s realized arc
      curvature matches `speed`/`radius` within plant tolerance.
- [x] All three accept `stop=` clauses from ticket 002's implemented set
      (`{t, d, heading, pos, rot}`), OR-combined with each verb's own
      built-in stop (none for `R`; heading/rotation for `TURN`/`RT`).
- [x] New `docs/protocol-v2.md` §10 subsections `### R`, `### TURN`,
      `### RT` are written, matching this ticket's implemented wire shape
      exactly (verb grammar, ranges, `OK`/`EVT` examples) — closing the
      doc gap identified in architecture-update.md Grounding fact 5.
- [x] No change to ticket 001/002's files beyond the new verb
      registrations and doc additions (no `Planner`/`Drivetrain`/
      `PoseEstimator` signature changes).

## Implementation Plan

**Approach:** Extend `motion_commands.cpp`'s table with three more
handlers. `TURN`/`RT` are the first consumers of `Planner::tick()`'s
`fusedPose` argument (already threaded through since ticket 001) — no
`Planner` signature change needed, just the first real use.

**Files to modify:**
- `source/commands/motion_commands.h`, `source/commands/
  motion_commands.cpp` (add `R`/`TURN`/`RT` parse/handle functions and
  table entries)
- `docs/protocol-v2.md` (new `### R`/`### TURN`/`### RT` sections under
  §10, following the existing `### G`/`### VW` section format/style
  exactly)

**Testing plan:**
- Sim-level tests: `R`'s realized curvature vs. commanded `speed`/`radius`;
  `RT 9000` ~90° rotation and `EVT done RT`; `TURN <heading>` convergence
  within `eps` from several starting headings; `stop=` clauses on all
  three; a bare (no `stop=`) `R` running open-ended until `STOP`.
- Existing `tests/sim/`/`tests/bench/`/`tests/unit/` suites stay green.

**Documentation updates:** `docs/protocol-v2.md` §10 gains the three new
verb sections (see Acceptance Criteria) — this is the primary deliverable
alongside the code, not an afterthought.
