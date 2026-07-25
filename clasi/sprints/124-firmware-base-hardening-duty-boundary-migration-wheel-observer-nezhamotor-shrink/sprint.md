---
id: '124'
title: 'Firmware base hardening: duty-boundary migration, wheel observer, NezhaMotor
  shrink'
status: roadmap
branch: sprint/124-firmware-base-hardening-duty-boundary-migration-wheel-observer-nezhamotor-shrink
worktree: false
use-cases: []
issues:
- firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 124: Firmware base hardening: duty-boundary migration, wheel observer, NezhaMotor shrink

## Goals

Second of the three firmware-base-hardening sprints. Rewrites the main
loop around the explicit-dataflow design
(`docs/design/base-explicit-loop-sketch.md`) and migrates the base's
command primitive from wheel-VELOCITY to wheel-DUTY: the velocity PID and
the bounded `MoveWheels` move relocate up into `src/motion` (the sibling
motion library sprint 122 stood up); the base gains a per-wheel command
observer (predict from commanded duty, correct on fresh encoder samples,
folding in the freshness gate / glitch rejection / velocity-estimator A-B
mechanisms `NezhaMotor` currently carries as three separate ad hoc
pieces); `NezhaMotor` shrinks to protocol + dwell/deadband + clamp per
the sketch's KEEP/MOVE/DELETE inventory. ROADMAP-LEVEL ONLY — this
sprint has not yet been detail-planned; full architecture, use cases,
and tickets are written when this sprint is promoted to Detail Mode
(expected: after sprint 123 lands, since the wider per-wheel telemetry
frame this sprint needs — commanded duty + observed + raw encoder state
per wheel — depends on the headroom COBS+CRC frees).

## Problem

`Devices::NezhaMotor` (885 lines/channel) still owns the velocity PID,
three overlapping and ad hoc measurement-conditioning mechanisms
(freshness gate, glitch rejection, a live-switchable EMA/least-squares
velocity-estimator pair), a duty boxcar filter, and an undecided slew
cap — all velocity DECISIONS living in what should be a pure bus adapter.
`Motion::WheelSink` (sprint 122) is deliberately still a velocity sink,
not a duty sink, with the duty-sink rewrite explicitly deferred here. The
main loop's own construction still wires objects by reference
(`Drive(motorL, motorR)`-shaped composition) rather than threading plain
per-cycle values, which is what makes "the base never lies" hard to audit
by inspection today.

## Solution

Per `clasi/issues/firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md`
(post-split, base-contract scope) and `docs/design/base-explicit-loop-sketch.md`:
rewrite `RobotLoop::cycle()`'s internal dataflow to the sketch's
sense→observe→decide→act→report shape (values as cycle-local structs,
not held references), move `velocity_pid.*`/the kff mapping/`MoveWheels`
into `src/motion`'s lowest tier, add the per-wheel command observer to
the base (consuming commanded duty + encoder samples, reporting
position/velocity/estimate age/rejects/wedged), and shrink `NezhaMotor`
to protocol + dwell/deadband + clamp per the sketch's inventory (delete
the duty boxcar; decide the slew cap by bench test — recommend delete;
fold wedge detection into the observer's innovation logic). Reconcile
both `src/firm`'s and `src/motion`'s `DESIGN.md` sets so `close_sprint`'s
validator passes against the new module boundaries.

## Success Criteria

- `RobotLoop::cycle()` reads top-to-bottom as sense→observe→decide→act→
  report with every cross-object value a printable cycle-local (no
  `Drive(motorL, motorR)`-shaped reference webs).
- The base's command primitive is per-wheel duty (`WheelSample`/
  `DutyCommand` per the sketch's boundary structs); `Motion::WheelSink`'s
  velocity-sink shape is retired in favor of a duty sink.
- The per-wheel command observer replaces the freshness gate/glitch
  rejection/velocity-estimator-A-B trio with one principled,
  innovation-bounded predict-correct estimator; `appliedDuty` feeds back
  through `WheelSample` for the relocated PID's anti-windup.
- `NezhaMotor` shrinks to protocol + dwell/deadband + clamp (~200 lines);
  the duty boxcar is deleted; the slew cap is either bench-deleted or
  explicitly folded into the observer's characterized model — not left
  silently in place.
- `MoveWheels` + stop conditions + timeout live in `src/motion`, wired
  through the new duty boundary.
- Design-doc sets for both `src/firm` and `src/motion` reconciled;
  `close_sprint`'s validator passes.
- Sim suite green with zero behavior change outside the intentional
  boundary/primitive change; bench-verified per
  `.claude/rules/hardware-bench-testing.md` (sensors alive, wheels drive
  and encoders run, real-link round-trip) since this changes the motor
  stack.

## Scope

### In Scope

- Explicit-dataflow loop rewrite (`RobotLoop::cycle()`).
- Duty-sink boundary rewrite (`Motion::WheelSink` → duty; new
  `WheelSample`/`DutyCommand` structs per the sketch).
- `velocity_pid.*` + kff mapping + `MoveWheels` relocation to
  `src/motion`.
- Per-wheel command observer (base-side), folding in freshness gate /
  glitch rejection / velocity-estimator A-B.
- `NezhaMotor` shrink; `MotorArmor` simplification (wedge → observer
  innovation logic); duty boxcar deletion; slew-cap decision.
- Design-doc reconciliation across `src/firm`/`src/motion`.
- Bench verification of the new motor/duty stack (stand-mounted).

### Out of Scope

- COBS+CRC wire framing (sprint 123 — must land first; this sprint's
  wider per-wheel telemetry needs its freed headroom).
- Characterization battery, numeric gate scoring, and the freeze
  declaration (sprint 125 — depends on this sprint).
- Any motion-library accuracy work (settle completion, heading hold,
  chain hand-off, tours) — `src/motion`'s own concern, unblocked but not
  addressed by this sprint.

## Test Strategy

(Describe the overall testing approach for this sprint: what types of tests,
what areas need coverage, any integration or system-level testing needed.)

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
