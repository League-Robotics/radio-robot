---
id: '003'
title: Bounded linear position-feedback trim in Pilot
status: open
use-cases:
- SUC-005
- SUC-007
depends-on:
- '002'
github-issue: ''
issue: motion-control-terminal-blips-reconciled-fix-plan.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bounded linear position-feedback trim in Pilot

## Description

Issue step 5. Adds `App::Pilot`'s own bounded linear position-feedback
trim, mirroring the existing heading PD's gain/arithmetic split exactly:
`v_cmd = twist.v + distance_kp * (twist.sRef - twist.sMeas)`, clamped to a
bounded ceiling. `Motion::Executor::Twist` gains `sRef`/`sMeas` (the linear
dominant-channel's own since-activation reference position and the
measured-path accumulator, 0/0 for kPivot/kTimed — a pure straight/arc-leg
mechanism, matching the issue's own "keep the existing heading PD as the
angular tracker" instruction: this trim does NOT touch the rotational
channel). Two new `PlannerConfig` fields: `distance_kp` [1/s] (field 39)
and `distance_tol` [mm] (field 40, repurposing the role
`kDistanceSettleEpsilonMm` currently plays as a hardcoded constant — this
ticket adds the field but does NOT yet wire it into the completion decision;
that rewire is ticket 004's scope). The gain/arithmetic split (Executor
exposes references, Pilot owns the gain and the correction arithmetic)
mirrors `motion/DESIGN.md` §2c's own documented, load-bearing boundary for
the heading PD — see sprint Architecture Design Rationale Decision 3. This
ticket claims no new harness `xfail` flip; it is a regression-guard plus the
087-009 non-regression verification.

## Acceptance Criteria

- [ ] `Twist` gains `float sRef` [mm] and `float sMeas` [mm]:
      `sRef = plannedPositionSinceActivation` (kArc only, else 0),
      `sMeas = measuredPathSinceActivation_` (kArc only, else 0).
- [ ] `App::Pilot::tick()` computes the trim and adds it to the velocity
      handed to `Drive::setTwist()`:
      `float trim = distanceKp_ * (twist.sRef - twist.sMeas); trim =
      clamp(trim, -trimCeiling_, trimCeiling_); float v = twist.v + trim;`
      — the trim is 0 whenever `sRef`/`sMeas` are both 0 (kPivot/kTimed), a
      harmless no-op requiring no mode branching in `Pilot`.
- [ ] The trim's clamp ceiling is a named constant or config value, chosen
      and documented in this ticket's own completion notes, along with the
      empirically-verified `distance_kp * distance_tol >= v_deadband`
      inequality (sprint Architecture Design Rationale Decision 6 gives the
      known `v_deadband` range, ~15-19mm/s, from `MIN_SPEED_DEFAULT`'s own
      comment in `gen_boot_config.py`, as a starting point — re-verify
      against the actual current source, don't just cite the architecture
      doc's numbers unchecked).
- [ ] `msg::PlannerConfig`/`PlannerConfigPatch` gain `distance_kp` (field
      39) and `distance_tol` (field 40) in `planner.proto`; regenerated via
      `scripts/gen_messages.py`; `gen_boot_config.py` bakes their defaults
      with the inequality derivation in a comment.
- [ ] `App::Pilot::applyPlannerPatch()` — document whether `distance_kp` is
      added to `PlannerConfigPatch`'s curated live-tunable set (mirroring
      `heading_kp`) or left boot-config-only for this ticket; either choice
      is acceptable, but it must be a stated decision in the ticket's
      completion notes, not an oversight.
- [ ] **Guardrail (SUC-007, 087-009 non-regression)**: the trim's clamp
      ceiling is verified (by a targeted test, or by inspection cited in
      the ticket) to sit below the authority that could reproduce a
      solve-side reversal — the trim only ever perturbs the SAMPLED
      velocity reference `Drive` receives; it never triggers, seeds, or
      retargets a `JerkTrajectory` solve. Confirm no `solveToRest`/
      `solveToState`/`solveToVelocity`/`retarget`/`reanchor` call is added
      or modified by this ticket.
- [ ] **Guardrail (SUC-007)**: `git diff --stat` shows no changes to
      `src/firm/app/robot_loop.cpp` or `src/firm/motion/
      jerk_trajectory.{h,cpp}`.
- [ ] No new harness `xfail` flip claimed; every currently-passing check
      (including ticket 001's two flips) stays passing.
- [ ] `app/DESIGN.md`'s `Pilot` PD-cascade write-up gains a parallel entry
      describing the new linear trim alongside the existing heading PD
      description.
- [ ] `uv run python -m pytest` is green end to end.

## Implementation Plan

- **Approach**: additive, mirrors the heading PD's existing shape in
  `pilot.cpp` exactly (gain read from config, arithmetic computed in
  `tick()`, added to the twist before `Drive::setTwist()`).
- **Files to modify**: `src/firm/motion/executor.h`/`.cpp` (`Twist`
  fields), `src/firm/app/pilot.h`/`.cpp` (gain member + arithmetic +
  clamp), `src/protos/planner.proto`, generated `src/firm/messages/`,
  `src/scripts/gen_boot_config.py`, `src/firm/app/DESIGN.md`.
- **Documentation updates**: as listed above; this ticket's own completion
  notes must record the chosen `distance_kp`/`distance_tol`/clamp-ceiling
  values and confirm the deadband inequality holds against them.

## Testing

- **Existing tests to run**: full `test_behavior_lock.py` harness, then
  the full `uv run python -m pytest`.
- **New tests to write**: a targeted clamp-authority check (unit or
  harness-based) verifying the trim's bounded magnitude near a command's
  own tolerance — the 087-009 guardrail verification.
- **Verification command**: `uv run python -m pytest`.
