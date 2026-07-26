---
id: '125'
title: Duty boundary, wheel observer, and NezhaMotor shrink
status: roadmap
branch: sprint/125-duty-boundary-wheel-observer-and-nezhamotor-shrink
worktree: false
use-cases: []
issues:
- firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 125: Duty boundary, wheel observer, and NezhaMotor shrink

## Goals

Rewrite the firmware base's command primitive from bounded wheel-speed
moves to per-wheel **duty** (`[-1,1]`), add the per-wheel command
observer (predict-correct wheel state from commanded duty + encoder
samples), and shrink `NezhaMotor` to its base-contract residue
(protocol + dwell/deadband + clamp, ~200 lines) — with `MotorVelocityPid`
and the bounded `MoveWheels` primitive relocating up into the motion
library, which owns velocity tracking now that duty is the boundary
primitive. Second of the three firmware-base-hardening sprints
(124 wire/state → **125 duty boundary** → 126 characterization/gate/
freeze).

Also absorbs whatever sprint 124's architecture defers from its own
scope valve (candidate: the Drive/Sensors device-ownership reshuffle —
named bus-phase methods `requestLeft()`/`collectLeft()`/`requestRight()`/
`collectRight()`, `RobotLoop` owning no devices) — see 124's
architecture for the actual valve decision and rationale; this sprint's
own detail-planning pass will confirm what actually lands here once 124
is closed.

## Problem

Today the base's command primitive is a bounded wheel-SPEED move
(`MoveWheels` + stop conditions + timeout), with the velocity PID
resident in `NezhaMotor`. Motor residency keeps `NezhaMotor` doing two
jobs (hardware protocol AND velocity control) that change for different
reasons. [NOTE 2026-07-26: the original rate argument here — "the PID
cannot update faster than the loop, encoder freshness ~80 ms bounds it"
— was measured FALSE: the register is live at ≤ 16 ms
(`docs/design/encoder-refresh-characterization.md`); the relocation
stands on the separation-of-concerns and one-estimate-one-controller
grounds.] Separately, the base has no per-wheel command observer: encoder
samples can repeat relative to the loop, and between fresh samples the base
has no principled estimate of what the wheel is actually doing — three
prior open-loop predictors (stop_lead, margins, analytic coast) tried to
patch this from outside and failed; a predict-correct observer inside
the base is the fix a bare open-loop predictor cannot be.

## Solution

- Command primitive becomes per-wheel **duty** (`[-1,1]`), one visible
  write per wheel per cycle. Safety: zero-on-silence (a cycle with no
  duty write commands zero) plus a plausibility clamp (`|duty| ≤ 1`,
  NaN → 0).
- A per-wheel command observer (dead time, rise shape, deadband floor,
  characterized against hardware) predicts wheel state between encoder
  samples and corrects on each real sample — model error survives at
  most one encoder interval. Reports both the observer estimate and the
  raw encoder value; consumers (motion library) choose which to trust.
- `NezhaMotor` shrinks to protocol + dwell/deadband + clamp.
  `MotorVelocityPid` and the bounded `MoveWheels` primitive relocate to
  the motion library's lowest tier, which is statically linked into the
  same firmware image.
- `appliedDuty` feedback: the base reports the actually-written duty
  (post dwell/deadband shaping) so the motion-side PID's anti-windup
  sees actuator truth, not the commanded value.
- Primary design reference: `docs/design/base-explicit-loop-sketch.md`
  (full `NezhaMotor` inventory with KEEP/MOVE/DELETE verdicts, boundary
  structs, resolved/open questions) — carried forward from before the
  split, now built atop 124's wire/`RobotState` schema instead of the
  old frame shape.

## Success Criteria

- A `move_wheels`-shaped duty command runs to completion on the stand;
  the wheel starts and stops on command.
- The observer's estimate and the raw encoder value are both visible in
  telemetry, per wheel, every frame.
- Zero-on-silence is demonstrated: a cycle with no duty write leaves the
  wheel at zero, not holding its last command.
- `NezhaMotor` measured at roughly its ~200-line target; `MotorVelocityPid`
  and `MoveWheels` no longer live in the base.
- Bench gate (per `.claude/rules/hardware-bench-testing.md`) exercised on
  the stand, not just in sim/unit tests.

## Scope

### In Scope

- Per-wheel duty command primitive + zero-on-silence + plausibility clamp.
- Per-wheel command observer (predict-correct), consuming commanded duty
  + encoder samples, built on 124's `RobotState`.
- `NezhaMotor` shrink; relocation of `MotorVelocityPid` and `MoveWheels`
  to the motion library.
- `appliedDuty` reporting.
- Whatever 124's scope valve explicitly defers here (see 124's
  architecture Design Rationale for the actual decision).

### Out of Scope

- Twist semantics, kinematics, odometry/pose, estimator/OTOS fusion,
  shaping, chain hand-off, settle completion, heading hold, tours — all
  motion-library territory, unaffected by this sprint's base-contract
  change.
- Characterization battery, numeric gate, and freeze declaration — split
  to sprint 126, which depends on this sprint landing first.
- The wire/state schema itself — that is 124's completed scope; this
  sprint builds on it, not concurrently with it.

## Test Strategy

To be detailed at detail-planning time, after 124 closes and this
sprint's architecture is written against 124's actual landed schema.
Expected shape: sim tests for the observer against `WheelPlant` (exact
tracking, since the sim plant is first-order by construction), bench
step-response characterization for real dead-time/deadband/tau, and a
stand bench-gate run per `.claude/rules/hardware-bench-testing.md`.

## Architecture

N/A — roadmap-level stub. Full architecture (7-step methodology,
diagrams, design rationale) is written when this sprint is detail-planned,
after sprint 124 lands and its actual `RobotState`/wire shape is known.

### Architecture Overview

(High-level structure and component relationships, if applicable.)

### Design Rationale

N/A — roadmap-level stub. Deferred to detail planning.

### Migration Concerns

N/A — roadmap-level stub. Deferred to detail planning.

## Use Cases

N/A — roadmap-level stub. Use cases are written when this sprint is
detail-planned.

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
