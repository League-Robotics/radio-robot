---
id: '123'
title: Heading-hold during Distance moves
status: roadmap
branch: sprint/123-heading-hold-during-distance-moves
worktree: false
use-cases: []
issues:
- heading-hold-during-distance-moves.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 123: Heading-hold during Distance moves

> Roadmap-level plan (Phase 1). Architecture, use cases, and tickets are
> filled in at detail-planning time, after sprints 121 (and ideally 122) land.

## Goals

Close the loop on the uncommanded omega axis during a straight (Distance-kind
TWIST) `Move`. Today omega is commanded to a constant 0 and nothing corrects
heading during the leg, so any entry heading error (chain hand-off residue in
sim; asymmetric friction/deadband/load on hardware) persists the whole leg and
translates into lateral path error — the robot drives straight in the wrong
direction and never squares up.

## Problem

A Distance-kind TWIST `Move` is heading-open-loop. Measured post-119-005 (sim,
ideal chip): a straight from rest holds heading exactly (leg 1: +0.00 deg), but
every straight entered with residual omega arcs and keeps the acquired error
forever. 121's land-at-zero removes the sim's entry-error source; heading-hold
is the complement that makes straights self-squaring against whatever remains —
and on hardware, against disturbances the sim does not model.

## Solution (candidate — confirm at detail time)

In `MoveQueue::shapeAndStage()` (`src/firm/app/move_queue.cpp`), for a
Distance-kind TWIST `Move` whose commanded omega is 0, drive the omega axis
with a small proportional correction toward the leg's ACTIVATION heading
instead of a constant 0:

    omegaCorrection = -headingGain * (theta - active_.activationTheta)  // [rad/s]

- `theta` is the same same-cycle odometry reading `tick()` already passes in.
- Route the correction through the existing `shaperOmega_` (cruise target =
  omegaCorrection) so it stays slew/jerk-limited — no new actuation path.
- Clamp the correction magnitude (config ceiling, e.g. 0.3 rad/s) so a large
  entry error re-squares gently, not with a swerve.
- Reference is the `Move`'s own activation heading — each leg squares to its
  own entry line; no global heading state, no cross-`Move` memory.
- Gain source: resurrect the orphaned `control.heading_kp` robot-JSON key as
  this loop's gain (currently has no consumer). `0` disables the hold
  (byte-identical to today), the same `0 == off` contract `ShaperLimits` uses.

Scope guards: Distance-kind TWIST moves with `cruiseOmega == 0` ONLY. Arcs
(Distance + nonzero omega), WHEELS moves, and Angle-kind moves are untouched.
Naming/units per `.claude/rules/coding-standards.md` — the gain is
dimensionless (1/s); no units in identifiers, tag in a `// [1/s]` comment.

## Success Criteria

- Sim: inject a deliberate 3 deg entry heading error at the start of a 700 mm
  Distance move; the leg ends with |heading - activation heading| <= 0.3 deg
  and lateral drift materially reduced (today: error persists 100%).
- Leg-1 exactness preserved: a clean-entry straight stays +0.00 deg — assert
  no omega command exceeds the clamp during a clean leg (no injected wobble).
- TOUR_1/TOUR_2 closure gates unchanged or improved; no oscillation at any
  `heading_kp` in the shipped range (state the stable gain range measured).
- Hardware bench verify on the stand (firmware control change).

## Scope

### In Scope

- `MoveQueue::shapeAndStage()` heading-hold correction on the omega axis for
  Distance-kind TWIST moves (`src/firm/app/move_queue.cpp`).
- `control.heading_kp` config plumbing (attic -> live consumer).
- Sim acceptance + hardware bench verify.

### Out of Scope

- Coordinated two-axis (v_x + omega) shaping for arcs (a separate concern; may
  surface in sprint 124's TOUR_4 arc-exit measurement).
- Orthogonal-boundary land-at-zero (121) and same-axis carry (122).

## Dependencies / Sequencing

- **Depends on 121** (land-at-zero) so this loop's acceptance numbers measure
  the HOLD itself, not the boundary bug. Ideally 122 lands too, but not
  strictly required.
- Independent of 124/125/126/127.

## Architecture

Deferred to detail planning. Expected tier: compact — a ~10-line addition plus
config plumbing to one firmware module, no new module, no new wire message, no
host involvement.

## Use Cases

Deferred to detail planning.

## Tickets

Deferred to detail planning.
