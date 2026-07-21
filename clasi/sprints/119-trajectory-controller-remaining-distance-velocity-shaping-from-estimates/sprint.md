---
id: '119'
title: 'Trajectory controller: remaining-distance velocity shaping from estimates'
status: roadmap
branch: sprint/119-trajectory-controller-remaining-distance-velocity-shaping-from-estimates
worktree: false
use-cases: []
issues:
- bench-turns-spin-forever-non-termination.md
- nocal-straight-terminal-wedge-needs-velocity-integrator.md
- turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md
- predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 119: Trajectory controller: remaining-distance velocity shaping from estimates

## Goals

Sprint 5 (final) of the predict-to-now odometry arc (see
`clasi/issues/predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md`
— this sprint's close also closes that arc issue). Replace
`Motion::Executor`'s profile-elapsed/dwell completion machinery with a
trajectory controller that shapes velocity from the estimator's
remaining-distance-to-goal, computed every control tick from
`App::StateEstimator` (not from a fixed profile). Per stakeholder
decision, this sprint is **detailed only after the sprint-118 gate
(stakeholder threshold ratification) has actually landed** — it is
sketched here at roadmap level and is NOT ready for detail planning
today.

## Problem

Both standing bench blockers —
`bench-turns-spin-forever-non-termination.md` (a 90° pivot never ramps
down, spins until the ack timeout) and
`nocal-straight-terminal-wedge-needs-velocity-integrator.md` (a pure-P
velocity loop droops below deadband at the terminal ramp and stalls) —
are terminal-behavior failures of the SAME machinery:
`Motion::Executor`'s profile-elapsed/dwell completion gate
(`src/firm/motion/executor.cpp:885-908`). Weeks of tuning that machinery
directly have not produced a completing tour. The arc's whole premise is
that termination decided from a live, validated position/velocity
estimate (sprints 115-118) — not a pre-committed profile clock — fixes
both failure modes at once.

## Solution

- Per wheel per segment: `remaining = goal − wheelNow(wheel).distance`;
  `v_cmd = sign(remaining) · sqrt(v_terminal² + 2·a_dec·|remaining|)`
  capped at `v_max` — arrives *with* the commanded terminal velocity,
  addressing both the terminal-droop wedge (no more falling below
  deadband and stalling short) and the never-ramps-down pivot (velocity
  actively shrinks toward the goal instead of holding constant omega).
- Completion decided from estimates at control rate; a stale basis
  (`valid`/`basisStamp`) fails safe to the existing timeout backstop —
  this sprint does not remove the timeout safety net, it makes hitting it
  in normal operation the exception again instead of the rule.
- `App::Odometry`/`App::HeadingSource` consolidation decision: now that
  the estimator is the live consumer, decide (and execute) what happens
  to the two legacy pose paths they still duplicate.
- Disposition of
  `turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md`
  — that test module characterizes lead-compensation gain-tuning
  machinery this controller replaces outright; likely disposition is
  deletion/replacement with acceptance coverage for whatever this sprint
  actually ships, not a numeric re-tune.

## Success Criteria

Tour completes on the bench. No timeout-fault turns. No terminal wedge.
Both blocker issues close.

## Scope

### In Scope

- Trajectory controller replacing `Motion::Executor`'s completion
  machinery.
- `bench-turns-spin-forever-non-termination.md`,
  `nocal-straight-terminal-wedge-needs-velocity-integrator.md`.
- `turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md`
  disposition.
- `Odometry`/`HeadingSource` consolidation decision.
- Closing the arc issue itself.

### Out of Scope

- Anything already covered by sprints 115-118 (rings, estimator, fake
  OTOS, external fusion) — this sprint only CONSUMES those, per the
  arc's own "controller sprint is detailed only after the estimator is
  bench-proven" stakeholder decision.

## Test Strategy

Sim first, then the full bench tour gate — this sprint's success
criteria ARE the test strategy (tour completion, zero timeout faults,
zero wedges). `uv run python -m pytest` + sim suite; `just build-clean`;
`mbdeploy deploy`; hardware bench gate per
`.claude/rules/hardware-bench-testing.md`.

## Architecture

(Architecture for this sprint's change, sized to the change — a
one-paragraph note for a trivial sprint, a fuller write-up with
component/data-model detail for a substantial one. May read "N/A —
trivial" when the change has no architectural impact.)

### Architecture Overview

(High-level structure and component relationships, if applicable.)

### Design Rationale

(Significant decisions with alternatives considered and reasoning, if
applicable.)

### Migration Concerns

(Data migration, backward compatibility, deployment sequencing — or
"None" if not applicable.)

## Use Cases

(Use cases sized to the change — may read "N/A — trivial" for small
sprints that don't warrant new or updated use cases.)

### SUC-001: (Title)
Parent: UC-XXX

- **Actor**: (Who)
- **Preconditions**: (What must be true before)
- **Main Flow**:
  1. (Step)
- **Postconditions**: (What is true after)
- **Acceptance Criteria**:
  - [ ] (Criterion)

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
