---
id: '005'
title: '[OPTIONAL/DEFERRABLE] Configurator live heading/velocity gain tuning'
status: open
use-cases: [SUC-003]
depends-on: ['003']
github-issue: ''
issue:
- heading-loop-cascade-control-turns-terminate-on-target.md
- real-robot-motion-calibration-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# [OPTIONAL/DEFERRABLE] Configurator live heading/velocity gain tuning

## ⚠️ OPTIONAL/DEFERRABLE — skip if the overnight run's risk budget is spent

The mandatory path (001→002→003→006) already satisfies the sprint's
acceptance criterion WITHOUT this ticket. Independent of ticket 004 — skip
either, both, or neither without affecting the other. If skipped, ticket
006 notes the deferral and closes the sprint with reflash-based tuning
(ticket 003's own method) as the only tuning path, exactly as sprints
093-097 already operate today.

## Description

Wire a minimal `Rt::Configurator` into `main.cpp`'s live loop so a binary
`SET` config delta actually reaches the running `Drivetrain`/`Hardware`,
cutting heading/velocity gain-tuning iteration from a reflash (~5 minutes)
to a live `SET` (seconds). Additive only — boot config still applies once,
directly, at construction, exactly as today; this does NOT reintroduce
093/094-era full runtime config authority.

Reference: `architecture-update.md` M7, SUC-003. `real-robot-motion-
calibration-undershoot.md`'s "Also discovered" section is the origin of
this gap: binary `SET` already acks into `bb.configIn` (ticket 096), but
nothing has drained it since 093/094 removed runtime config authority.

Depends on 003 — tune against the bench-verified Stage 1 baseline, not a
moving target.

## Acceptance Criteria

- [ ] `main.cpp` constructs one `Rt::Configurator`, seeded from the SAME
      boot `msg::DrivetrainConfig`/`msg::PlannerConfig` values already
      passed directly to `drivetrain.configure()`/
      `drivetrain.configureMotion()` at construction — boot behavior is
      PROVABLY unchanged (a freshly booted robot with no `SET` ever sent
      behaves identically to today).
- [ ] `main.cpp`'s loop calls `configurator.applyOne(bb)` once per pass
      (mirroring the pre-093/094 pattern) — placed so it drains at most one
      `bb.configIn` delta per pass, matching `Configurator::applyOne()`'s
      own documented one-delta-per-call contract.
- [ ] `Rt::Configurator::applyOne()`'s existing `kPlanner` case gains ONE
      new line: `drivetrain_.configureMotion(plannerConfig_);` immediately
      after the `foldPlanner(...)` call, alongside the existing
      `bb.plannerConfig = plannerConfig_;` publish — today that case only
      folds+publishes (a residue of ticket 094-002 relocating
      `Subsystems::Planner` out of `source/`); `Subsystems::Drivetrain` is
      the correct live target now (the Configurator already holds a
      `Drivetrain&`).
- [ ] `kMotor`/`kDrivetrain`/`kOdometer`'s existing, already-correct
      fold-and-apply paths are UNCHANGED — this ticket touches the
      `kPlanner` case only.
- [ ] SIM ACCEPTANCE: a new scenario drives a `SET`-equivalent config
      delta for `heading_kp` mid-session (via `bb.configIn`/whatever the
      sim harness's existing config-delta injection surface is) and
      confirms the VERY NEXT segment's commanded twist reflects the new
      gain — no restart, no reflash-equivalent.
- [ ] Full `uv run python -m pytest` stays green, no regression.
- [ ] HARDWARE ACCEPTANCE: a bench session sends a live `SET` for
      `heading_kp` (or `heading_kd`) over serial/relay and confirms (via
      `TLM`/a subsequent `turn_sweep.py` cell) the change took effect
      WITHOUT a reflash.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`.
- **New tests to write**: the live-`SET`-changes-live-behavior sim
  scenario itemized above.
- **Verification command**: `uv run python -m pytest`; a bench
  `SET heading_kp=<value>` followed by an immediate re-run of one
  `turn_sweep.py` cell as the hardware confirmation.

## Implementation Plan

**Approach**: Construct-and-tick the existing `Rt::Configurator` class
(already fully implemented, just never instantiated in `main.cpp` since
093/094) plus the one-line `kPlanner` fix.

**Files to modify**: `source/main.cpp`, `source/runtime/configurator.cpp`.

**Files to create**: none.

**Testing plan**: as above.

**Documentation updates**: none required structurally.
