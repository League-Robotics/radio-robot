---
id: 098
title: 'Heading-loop cascade control: turns terminate on target'
status: done
branch: sprint/098-heading-loop-cascade-control-turns-terminate-on-target
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- heading-loop-cascade-control-turns-terminate-on-target.md
- real-robot-motion-calibration-undershoot.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 098: Heading-loop cascade control: turns terminate on target

## Goals

Command an in-place turn of any angle at any speed and have the robot
**terminate exactly on the requested heading** (goal ≈ ±1°), with the
speed-dependent overshoot and run-to-run scatter a 58-turn playfield
dataset proved gone. Add the missing **outer heading feedback loop**,
cascaded onto sprint 097's already-fixed inner wheel-velocity PID loops —
the stakeholder-provided cascade architecture
(`heading-loop-cascade-control-turns-terminate-on-target.md`). Stage the
heading source so Stage 1 (encoder-derived heading) is a complete,
shippable, independently-sufficient deliverable before any new sensor
plumbing (Stage 2, OTOS) is risked in the same overnight run.

## Problem

A 58-turn playfield dataset proved **nothing in the firmware regulates
heading**: the wheel-velocity PID loops track their setpoint well
(sprint 097), but `Motion::SegmentExecutor` plays the Ruckig-solved
rotational velocity sample straight through to the wheel setpoint every
tick — an OPEN loop on heading, patched only by a bang-bang divergence
replan and a ride-the-tail terminal. The landed error is the small
difference of two ~25° open-loop transients (a ~−22° accel deficit vs. a
~+24..+30° decel surplus), which is also why the SAME turn scatters σ≈2°
run-to-run, and why the divergence replan's own re-anchoring produces a
90° ridge (+4..+6° worse than the best empirical fit) on some fast short
turns. A fixed empirical aim-short table cannot fix a converged-servo
problem — only a feedback loop on the quantity that is actually wrong can.

## Solution

Cascade control: each executor tick, for PRE_PIVOT/TERMINAL_PIVOT, sample
the Ruckig plan's desired `(θ, ω)`, compute
`ω_cmd = ω_desired + Kp·(θ_desired − θ_measured) + Kd·(ω_desired − ω_measured)`,
and feed the result — instead of the raw plan sample — to the UNCHANGED
inner wheel-velocity PID loops. Completion becomes `|heading_error| < tol
AND |rate| < rate_tol`, held for a short dwell, replacing
`STOP_ROTATION`'s arc-threshold + ride-the-tail terminal for these two
phases. The pre-existing divergence replan (`maybeReplanPivot()`) is
retired to stall-protection only — its gross-divergence reanchor branch
stays as a stalled-wheel safety net; its sub-gross EXTEND branch (chasing
nominal tracking lag) is now redundant with the PD loop's own continuous
correction and is retired for these phases.

Heading source is staged: **Stage 1 (mandatory)** closes the loop on
encoder-derived heading (`(encR−encL)/trackwidth`) — zero new sensor
plumbing, the dataset's own ground truth, sufficient by itself to satisfy
the acceptance criterion. **Stage 2 (optional)** revives OTOS ticking in
the live loop and lets the executor prefer OTOS heading when valid, with
encoder fallback — a slip-immunity upgrade, not required for acceptance.
A minimal **Configurator revival (optional)** makes live gain tuning
possible without a reflash, de-risking the bench-tuning loop but not
required either.

New `heading_kp`/`heading_kd` PD gains are per-robot tunables in
`data/robots/tovez.json`, plumbed through `scripts/gen_boot_config.py`
exactly like the existing `vel_*` gains — this also retires `main.cpp`'s
hand-written `PlannerConfig` defaults, which today bypass that generator
entirely.

## Success Criteria

- Sim regression suite green throughout (no regression from the pre-sprint
  615+ in `tests/sim/`); the sim plant's own near-zero tracking asymmetry
  means the cascade must be a no-op-to-improvement there, not a
  discriminator of gain quality.
- `tests/bench/turn_sweep.py --relay --both` on the playfield: every cell
  lands within ≈±1° of target, the 90° ridge is gone, run-to-run scatter
  has collapsed from the pre-sprint ~2° baseline.
- Zero commanded terminal reversal at the wheel level beyond what the
  `Hal::Motor` reversal-dwell/deadband armor already absorbs — verified on
  the stand before the playfield leg, not assumed.
- The mandatory path (encoder-only Stage 1) is independently shippable —
  Stage 2 (OTOS) and the Configurator wiring are explicitly deferrable
  without blocking sprint closure.

## Scope

### In Scope

- `msg::PlannerConfig` heading-gain fields (`heading_kp`/`heading_kd`,
  proto fields 13/14) + `scripts/gen_boot_config.py` plumbing (also folding
  in `main.cpp`'s hardcoded motion-limit defaults, retiring that
  duplicate) + `data/robots/tovez.json` starting values.
- `Motion::SegmentExecutor`'s rotational tick path (PRE_PIVOT/
  TERMINAL_PIVOT only): the PD cascade, the tolerance/dwell completion
  gate, and `maybeReplanPivot()`'s retirement to stall-protection.
- Stage 1 hardware acceptance (stand + playfield) — the sprint's own
  acceptance gate.
- [Optional] Stage 2: OTOS heading ticking in `main.cpp`, threaded through
  the executor's existing (currently always-empty) `PoseEstimate` seam,
  with encoder fallback.
- [Optional] A minimal `Rt::Configurator` revival for live heading/velocity
  gain tuning (additive only — boot-config-applied-once-at-construction is
  unchanged).
- Sprint closure validation (build-clean, sim green, final hardware pass).

### Out of Scope

- TRANSLATE/linear-channel control — this sprint is the rotational channel
  only.
- BLEND (streaming `MOVE`/`MOVER` teleop) — the cascade is scoped to
  discrete PRE_PIVOT/TERMINAL_PIVOT phases; extending it to streaming
  turns is a possible future follow-up, not required for this sprint's
  acceptance criterion (see `architecture-update.md` Open Question 1).
- Full pose fusion / EKF — sprint 099's scope
  (`restore-pose-estimation-otos-encoders-delayed-camera-fixes.md`); Stage
  2 here needs OTOS *heading* only, not position or fused pose.
- `data/robots/togov.json` (mecanum) gain characterization — this sprint's
  dataset, acceptance instrument, and hardware access are all `tovez`
  only; `togov` inherits the `Kp=Kd=0` firmware-default fallback.
- A return to 093/094-era full runtime config authority — the optional
  Configurator wiring is additive only.

## Test Strategy

Every source-touching ticket carries sim coverage in
`tests/sim/unit/segment_executor_harness.cpp`, run via
`uv run python -m pytest` (the project-standard invocation — never bare
`pytest`), which must stay green throughout (no regression from the
pre-sprint baseline). The sim plant's own near-zero tracking asymmetry
means sim tests prove STABILITY and MECHANISM correctness (loop closes,
no premature completion, stall protection still fires, no reverse-creep),
not final gain quality — that discrimination happens on real hardware.
The sprint's acceptance instrument is `tests/bench/turn_sweep.py --relay
--both`, already built (the same tool that produced the 58-turn dataset
this sprint's design is based on), run first on the stand (no-wedge safety
check) and then on the playfield (radio relay, the accuracy/scatter
measurement) — a two-location procedure ticket 003/006 document explicitly.

## Architecture Notes

See `architecture-update.md` for the full design, diagrams, and design
rationale. Key constraints: the cascade lands INSIDE the existing
`Motion::SegmentExecutor` class (no new class — Decision 1), tolerance/
dwell stay file-local constants rather than wire-configurable fields
(Decision 3, matching the existing `kDivergenceThreshold`-family
precedent), and Stage 2's OTOS reading reuses the executor's existing
(currently always-empty) `PoseEstimate` parameter rather than introducing
a new type (Decision 4) — so reverting Stage 2 to encoder-only is a
one-line change at the call site, not a signature rollback. No new
inter-module dependency edges are introduced anywhere in this sprint.

## GitHub Issues

(None linked yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Heading-loop `PlannerConfig` plumbing (proto, `gen_boot_config.py`, robot JSON) | — |
| 002 | Heading PD cascade, tolerance/dwell completion, and replan retirement | 001 |
| 003 | Stage 1 hardware acceptance: encoder heading loop on the stand and playfield | 002 |
| 004 | [OPTIONAL/DEFERRABLE] OTOS heading source with encoder fallback (Stage 2) | 003 |
| 005 | [OPTIONAL/DEFERRABLE] Configurator live heading/velocity gain tuning | 003 |
| 006 | Sprint 098 closure validation | 003 (+004/005 if run) |

Tickets execute serially in the order listed. The mandatory path is
001 → 002 → 003 → 006; tickets 004 and 005 are each independently
deferrable without blocking 006.
