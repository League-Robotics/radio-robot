---
id: '119'
title: 'Land at zero and clean house: motion semantics + deletions'
status: roadmap
branch: sprint/119-land-at-zero-and-clean-house-motion-semantics-deletions
worktree: false
use-cases: []
issues:
- land-at-zero-completion-delete-stop-lead.md
- turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md
- kill-the-silent-off-shaping-config-boundary.md
- specify-and-assert-the-leg-handoff-contract.md
- delete-the-config-attic-and-dead-tour-kwargs.md
- relocate-narrative-comments-and-refresh-stale-docs.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 119: Land at zero and clean house: motion semantics + deletions

## Goals

Follow-on to sprint 118 ("loop schedule truth"). With the loop's timing and
odometry-freshness defects fixed, this sprint closes the *remaining*
open items from the 2026-07-22 turn-execution review: make turn/distance
completion an emergent property of the velocity shaper's own taper
("land at zero") instead of a tuned time-lead guess, delete `stop_lead_ms`
and the anticipation machinery it required, close the silent-off
shaping/anticipation config boundary that cost weeks of confusion, specify
the chain-advance leg hand-off contract that today is only tuned around,
and sweep the accumulated dead config surface (`control.*` attic keys,
dead `run_tour()` kwargs) and stale doc references the review's bloat
inventory (§6) flagged. Sequenced strictly after 118: every issue here
either directly depends on 118's odometry-freshness fix (land-at-zero's
`remaining` must be computed from this-cycle data) or on land-at-zero
itself landing first (the config-boundary/attic/doc cleanup tickets all
reference `stop_lead_ms`'s deletion).

## Problem

Per `docs/code_review/2026-07-22-turn-execution-review.md` R2/D3/D5/D1
and its bloat inventory (§6): turn completion today fires on a predicted
heading from a hand-tuned `stop_lead_ms` scalar that has been retuned four
times in three weeks and is provably wrong at any ω/cycle-time/shaper
setting other than the one it was last tuned against — a compensator for
latency instead of a removal of it. The correctness feature that actually
fixes this (velocity-shaper taper + forward prediction) has a silent-off
config boundary that left it disabled in every live GUI session until
today. The chain-advance leg hand-off (carried shaper state across tour
legs, asymmetric reversal dwell) is tuned around, never specified. And
"config as source of truth" has drifted into "config as attic": 20 dead
`control.*` keys and 4 dead `run_tour()` kwargs survive with zero living
consumers, each one an invitation for a future agent to "wire it back up."

## Solution

Six issues, sequenced so land-at-zero lands first (everything else either
depends on its deletion or is independent cleanup that should not race
it):

1. **`land-at-zero-completion-delete-stop-lead.md`** — declare MOVE
   completion when `remaining ≤ ε AND |ω_cmd| ≤ ε_ω` (the shaper's own
   decel taper already targets this), keep the StopCondition
   threshold/timeout as the always-armed backstop, delete `stop_lead_ms`
   + the anticipation block, quarantine (not delete) `StateEstimator`'s
   `bodyAt()` now that it has no firmware production consumer.
2. **`turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md`**
   — folds into (1): `test_turn_error_characterization.py`'s
   postcompensation tests characterize a lead-compensation approach that
   no longer exists once (1) lands; rewrite or delete per that module's
   own disposition note.
3. **`kill-the-silent-off-shaping-config-boundary.md`** — default-on
   shaping/anticipation at `SimLoop.configure_from_robot()` (the seam
   every caller already goes through) instead of only the TestGUI's
   connect-time push; add a loud telemetry-flag + GUI-banner off-state
   indicator so a 20×-accuracy-delta feature never again has an invisible
   off state.
4. **`specify-and-assert-the-leg-handoff-contract.md`** — one contract
   paragraph in `motion/DESIGN.md` (carried-axis ramp, decay-axis
   behavior, reversal-dwell asymmetry budget), asserted in the tour
   boundary test instead of tuned around.
5. **`delete-the-config-attic-and-dead-tour-kwargs.md`** — remove the 20
   dead `control.*` keys (schema + all three robot JSONs + allowlist) and
   `run_tour()`'s 4 dead kwargs + `DEFAULT_INTER_LEG_SETTLE`.
6. **`relocate-narrative-comments-and-refresh-stale-docs.md`** — last,
   so it trims only what survives the deletions above: shrink
   `move_queue.h`/`sim_harness.h`/`tour.py` header essays to contract-only,
   fix the stale protocol-v2/`source/commands/` references, the "Managed —
   Ruckig" GUI label, and the dangling deleted-issue xfail citations.

## Success Criteria

Full `uv run` pytest suite green, sim tour-closure gate + button-acceptance
suite green with `stop_lead_ms` fully deleted (grep gate: no `stop_lead`
string survives in `src/` or `data/`), isolated 90° twist lands within
±2° sim-deterministic, no dead `control.*` key survives, every stale doc
reference in the review's bloat inventory fixed.

## Scope

### In Scope

- Land-at-zero MOVE completion predicate (`MoveQueue::tick()`), deletion
  of `stop_lead_ms` and its config/JSON/estimator-patch surface.
- Rewrite/deletion of `test_turn_error_characterization.py`'s
  postcompensation tests.
- Default-on shaping/anticipation push at `configure_from_robot()`; loud
  telemetry/GUI off-state indicator (append-only `docs/protocol-v4.md`
  change).
- Chain-advance leg hand-off contract (`motion/DESIGN.md`) + boundary-test
  assertion.
- Deletion of the 20 dead `control.*` config keys and 4 dead `run_tour()`
  kwargs + `DEFAULT_INTER_LEG_SETTLE`.
- Narrative-comment relocation and stale-doc-reference sweep (sequenced
  last).

### Out of Scope

- Hardware bench verification — deferred to the phase-B bench session
  that follows both 118 and 119 (see 118's own deferral note; the same
  applies here — this sprint's acceptance bar is the sim suite + closure
  gate + button acceptance, not the stand).
- Any new StateEstimator consumer (fake-OTOS/fusion bench work) —
  `bodyAt()` is quarantined, not wired to a new consumer, this sprint.
- Anything from the review's §6 bloat inventory not named in the six
  issues above (e.g. `Header archaeology` items outside the three named
  files) — out of scope unless discovered to be entangled during
  execution.

## Test Strategy

Sim-only this sprint (bench deferred, per the overnight mandate). Each
issue's own acceptance criteria (isolated-turn accuracy bands, closure
gate bands, grep gates for dead strings/keys) is the ticket-level test
plan; full detail lands when this sprint is detail-promoted. The full
`uv run python -m pytest` suite and the sim tour-closure/button-acceptance
gates must stay green after every ticket, not just at sprint end.

## Architecture

(Deferred — this sprint is in Roadmap Mode. Full Architecture and Use
Cases sections are written when this sprint is detail-promoted, after
sprint 118 closes and its odometry-freshness fix is available to build
on. Expected tier at detail time: substantial — land-at-zero changes
`MoveQueue`'s completion predicate and touches `StateEstimator`'s
consumer count to zero; the config-boundary fix changes a cross-module
default (`SimLoop.configure_from_robot()` → estimator/shaper push); a
data/config-schema deletion (20 keys) qualifies as a data-model change on
its own. A diagram may or may not be warranted — that call is deferred to
detail-time inspection of whether any new composition (vs. pure deletion/
relocation) is actually introduced.)

### Architecture Overview

(Deferred to detail-mode planning.)

### Design Rationale

(Deferred to detail-mode planning.)

### Migration Concerns

(Deferred to detail-mode planning. Anticipated: `stop_lead_ms` deletion
from three robot JSONs + the pydantic schema + `EstimatorConfigPatch`
wire arm is a config/schema migration — every JSON must drop the field in
the same commit as the schema, per `delete-the-config-attic...`'s own
"schema deletion... in one commit" scope guard, which the land-at-zero
issue's own delete list should follow by the same discipline.)

## Design Overlay

Subsystems this sprint is expected to touch (for the eventual
`seed_sprint_design_overlay` call at detail-promotion time): `src/firm/app`
(`MoveQueue` completion predicate, `StateEstimator` consumer-count
change), `src/firm/motion` (leg hand-off contract paragraph;
`motion/DESIGN.md`), `src/host/robot_radio` (`configure_from_robot()`
default-on push; dead-kwarg deletion in `tour.py`), `src/tests`
(boundary-test un-xfail; postcompensation test rewrite) — plus
`docs/design/design.md`'s cadence/command-surface line if the land-at-zero
predicate changes how completion is described at the system level. Per
the flat-overlay-slot precedent set in sprints 116/117/118 (only one
subsystem `DESIGN.md` can occupy the overlay's single `DESIGN.md` slot at
a time), the detail-mode planner will need to pick ONE of
`app`/`motion`/`robot_radio`/`tests` for the overlay slot and edit the
rest directly on their canonical path, same as 118 did for `src/sim`.
Not seeded now — Roadmap Mode does not seed overlays; this call happens
during this sprint's own Phase 2 (Architecture) once detail-promoted.

## Use Cases

(Deferred — Roadmap Mode. Full Use Cases (SUC-NNN, continuing from
sprint 118's allocation) are written at detail-promotion time, one per
issue at minimum: land-at-zero completion, silent-off config-boundary
default, leg hand-off contract. The config-attic/dead-kwarg deletion and
doc-relocation issues are not expected to need their own use case — they
are internal cleanup with no behavior change.)

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets (rough cut — finalized at detail-promotion)

Not yet created (Roadmap Mode). Anticipated shape, one ticket per issue
except where an issue explicitly folds into another, dependency-ordered:

| # (rough) | Title | Depends On | Issue(s) |
|---|-------|------------|----------|
| 1 | Land-at-zero completion predicate; delete `stop_lead_ms` + anticipation block; quarantine `StateEstimator` production consumer | 118 (odometry-freshness) | land-at-zero-completion-delete-stop-lead.md |
| 2 | Rewrite/delete `test_turn_error_characterization.py` postcompensation tests | 1 | turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md |
| 3 | Default-on shaping/anticipation push at `configure_from_robot()`; telemetry flag + GUI banner off-state indicator; `docs/protocol-v4.md` append | 1 (field-list agreement per that issue's own note) | kill-the-silent-off-shaping-config-boundary.md |
| 4 | Leg hand-off contract paragraph (`motion/DESIGN.md`) + boundary-test assertion | 1 | specify-and-assert-the-leg-handoff-contract.md |
| 5 | Delete 20 dead `control.*` config keys + 4 dead `run_tour()` kwargs + `DEFAULT_INTER_LEG_SETTLE` | 1 (coordinates `stop_lead_ms` field, per scope guard) | delete-the-config-attic-and-dead-tour-kwargs.md |
| 6 | Relocate narrative comments to DESIGN.md/git; refresh stale doc references | 1, 2, 3, 4, 5 (trims only what survives) | relocate-narrative-comments-and-refresh-stale-docs.md |

Tickets execute serially in the order listed (`worktree: false`, same as
118 — land-at-zero must precede the deletion/docs tickets per the
stakeholder mandate). Ticket 1 is the sprint's critical path; tickets
3-5 are independent of each other once 1 lands and could in principle
parallelize, but this sprint stays serial.
