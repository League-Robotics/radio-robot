---
id: '122'
title: Same-axis carry-through & chain-margin cleanup
status: roadmap
branch: sprint/122-same-axis-carry-through-chain-margin-cleanup
worktree: false
use-cases: []
issues:
- chain-advance-reset-defeats-same-axis-compatible-leg-continuity.md
- chain-advance-completion-margin-narrow-pocket.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 122: Same-axis carry-through & chain-margin cleanup

> Roadmap-level plan (Phase 1). Architecture, use cases, and tickets are
> filled in at detail-planning time, after sprint 121 lands.

## Goals

Finish the chain-advance boundary story that sprint 121 starts. 121 makes
ORTHOGONAL boundaries (turn->straight, straight->turn) land at zero with the
final-move predicate. 122 owns the OTHER half: SAME-AXIS COMPATIBLE
boundaries (e.g. two `Distance` legs at the same `v_max`, same sign), where
velocity SHOULD carry through the boundary seamlessly and does not today.

## Problem

Two coupled defects in `App::MoveQueue`'s completion/reset logic
(`src/firm/app/move_queue.cpp`):

1. **Reset defeats same-axis continuity** (SUC-003 regression).
   `MoveQueue::tick()` hard-resets the completing axis's shaper
   (`shaperVX_`/`shaperOmega_`) to `(0, 0)` at EVERY completion boundary,
   unconditionally. For two genuinely compatible same-axis, same-kind chained
   legs, the next `Move`'s `activate()` then reads that just-zeroed
   `commandedSpeed()` as its carried starting point, so the robot decelerates
   to ~16% of `v_max` and re-accelerates at the boundary instead of carrying
   straight through. Reproduction:
   `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
   (`src/tests/testgui/test_tour_closure_gate.py`) measures a dip to
   24.0 mm/s against a 90%-of-`v_max` no-dip floor.
2. **Chain-margin narrow pocket.** `kStoppingMarginFactorChain` /
   `kDiscretizationCyclesChain` sit in a swept-but-fragile pocket. After 121,
   these constants NO LONGER govern orthogonal boundaries (land-at-zero
   replaces that use), so their only remaining role is same-axis boundaries —
   exactly 122's concern. This is where the narrow-pocket story concludes.

## Solution (candidate — confirm at detail time)

Make the completing-axis reset in `MoveQueue::tick()` CONDITIONAL on whether
the incoming chained `Move` (`pending_[0]`, when `pendingCount_ > 0`) shares
the ending `Move`'s own stop-kind axis and sign:

- Same axis, same kind, compatible sign (`Distance`->`Distance`, both `v_x`):
  SKIP the reset — carry `commandedSpeed()`/`commandedAccel()` through,
  restoring SUC-051/SUC-003 seamless hand-off.
- Different axis/kind (the orthogonal case, now owned by 121's land-at-zero):
  keep the reset — cuts the stale-residual leak the reset was added to guard.

RE-SWEEP `kStoppingMarginFactorChain`/`kDiscretizationCyclesChain` jointly
with the conditional reset against the tour-closure gate (the unconditional
reset was part of what the prior sweep tuned against, so a conditional variant
needs its own pass). If the re-sweep shows the conditional variant regresses
chain accuracy again (as a `pendingCount()`-gated variant already did once in
118-003), escalate for an explicit stakeholder decision to accept a bounded
same-axis dip and replace the no-dip assertion with a stated
bounded-recovery-time check.

## Success Criteria

- `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
  passes with its 90%-of-`v_max` no-dip floor intact, OR the stakeholder has
  explicitly accepted the dip and the assertion is replaced with a stated,
  bounded-recovery-time check.
- The narrow-pocket finding is re-verified (not silently changed) by whatever
  the fix turns out to be; chain turn accuracy under the tour-closure gate
  does not regress.
- 121's orthogonal-boundary land-at-zero behavior is unaffected.

## Scope

### In Scope

- `App::MoveQueue::tick()` completing-axis reset conditionalization
  (`src/firm/app/move_queue.cpp`).
- Joint re-sweep of `kStoppingMarginFactorChain`/`kDiscretizationCyclesChain`
  against the tour-closure gate.
- Firmware; Sim-testable via the closure gate; hardware bench verify if the
  reset/margin change is behaviorally observable on the stand.

### Out of Scope

- Orthogonal-boundary land-at-zero (sprint 121 owns it).
- Heading-hold on Distance moves (sprint 123).
- Any host/tour-runner change.

## Dependencies / Sequencing

- **Depends on 121.** Land-at-zero must land first: it decouples the two
  chain constants from orthogonal boundaries, so 122's re-sweep governs ONLY
  the same-axis case. Doing 122 before 121 would re-entangle the two.
- Independent of 123/124/125/126/127.

## Architecture

Deferred to detail planning. Expected tier: compact-to-substantial (a single
firmware module, `App::MoveQueue`, but a control-behavior change that needs a
re-sweep and a same-axis-carry regression guard).

## Use Cases

Deferred to detail planning. Expected to refine SUC-003/SUC-051 (seamless
same-axis hand-off).

## Tickets

Deferred to detail planning.
