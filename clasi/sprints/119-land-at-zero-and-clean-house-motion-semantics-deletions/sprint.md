---
id: '119'
title: 'Land at zero and clean house: motion semantics + deletions'
status: roadmap
branch: sprint/119-land-at-zero-and-clean-house-motion-semantics-deletions
worktree: false
use-cases: []
issues:
- kill-the-silent-off-shaping-config-boundary.md
- specify-and-assert-the-leg-handoff-contract.md
- delete-the-config-attic-and-dead-tour-kwargs.md
- relocate-narrative-comments-and-refresh-stale-docs.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 119: Land at zero and clean house: motion semantics + deletions

## Goals

**Amended (2026-07-23, mid-execution) — scope reduced from six issues to
four.** Land-at-zero completion (`land-at-zero-completion-delete-stop-lead.md`)
and its companion test-disposition issue
(`turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md`)
were PULLED FORWARD into sprint 118 as its ticket 004: ticket 118-002's
own closure-gate run went red at the unchanged `stop_lead_ms=45` once
odometry freshness landed (fresh data overcompensation, not a bug in
002), a 0-120ms sweep found no safe re-baseline value, and per the
turn-execution review's own R6 rule and this project's
sprint-end-must-be-testable convention, the fix was deleted forward into
118 rather than left for this not-yet-detailed sprint to inherit as a
known-red gate. Full rationale: sprint 118's own sprint.md, Decision
Record and Design Rationale Decision 4.

Follow-on to sprint 118 ("loop schedule truth" — now shipping land-at-zero
completion too). With the loop's timing, odometry-freshness, AND
completion-predicate defects fixed in 118, this sprint closes the
*remaining* open items from the 2026-07-22 turn-execution review: close
the silent-off shaping/anticipation config boundary that cost weeks of
confusion, specify the chain-advance leg hand-off contract that today is
only tuned around, and sweep the accumulated dead config surface
(`control.*` attic keys, dead `run_tour()` kwargs) and stale doc
references the review's bloat inventory (§6) flagged. Sequenced strictly
after 118: every remaining issue here depends on 118's completed fixes
(the config-boundary/attic/doc cleanup tickets reference
`stop_lead_ms`'s deletion, already done in 118).

## Problem

Per `docs/code_review/2026-07-22-turn-execution-review.md` D1/D5 and its
bloat inventory (§6) — the `stop_lead_ms`/completion-predicate problem
(R2/D3) is now RESOLVED by sprint 118's ticket 004, not this sprint's
problem to solve: the correctness feature that fixes turn accuracy
(velocity-shaper taper + land-at-zero completion) has a silent-off config
boundary that left it disabled in every live GUI session until recently.
The chain-advance leg hand-off (carried shaper state across tour legs,
asymmetric reversal dwell) is tuned around, never specified. And "config
as source of truth" has drifted into "config as attic": (118's ticket 004
already removed `stop_lead_ms` from this list) 20 dead `control.*` keys
and 4 dead `run_tour()` kwargs survive with zero living consumers, each
one an invitation for a future agent to "wire it back up."

## Solution

**Amended: four issues** (was six; land-at-zero + its test-disposition
companion delivered by sprint 118's ticket 004 — see Goals). Sequenced so
the config-boundary fix lands first among the remainder (independent
cleanup that should not race it), handoff-contract and config-attic
deletion can run in either order relative to each other, doc relocation
runs last:

1. **`kill-the-silent-off-shaping-config-boundary.md`** — default-on
   shaping/anticipation at `SimLoop.configure_from_robot()` (the seam
   every caller already goes through) instead of only the TestGUI's
   connect-time push; add a loud telemetry-flag + GUI-banner off-state
   indicator so a 20×-accuracy-delta feature never again has an invisible
   off state. Note: the "anticipation" half of this issue's original
   framing is narrower now that 118 deleted `stop_lead_ms` — the shaper
   limits (a_max/a_decel/alpha_max/alpha_decel/j_max/yaw_jerk_max) and
   estimator weights are still the live fields to default-push; confirm
   the field list against 118's actual delete list when this ticket is
   detailed.
2. **`specify-and-assert-the-leg-handoff-contract.md`** — one contract
   paragraph in `motion/DESIGN.md` (carried-axis ramp, decay-axis
   behavior, reversal-dwell asymmetry budget), asserted in the tour
   boundary test instead of tuned around.
3. **`delete-the-config-attic-and-dead-tour-kwargs.md`** — remove the 20
   dead `control.*` keys (schema + all three robot JSONs + allowlist) and
   `run_tour()`'s 4 dead kwargs + `DEFAULT_INTER_LEG_SETTLE`. Note:
   `stop_lead_ms` is no longer in this issue's scope-guard concern — 118's
   ticket 004 already deleted it; verify at detail time that this issue's
   own text (which originally said "coordinate if sequenced in the same
   sprint as the land-at-zero deletion") is updated to reflect that the
   coordination already happened, not still pending.
4. **`relocate-narrative-comments-and-refresh-stale-docs.md`** — last,
   so it trims only what survives the deletions above: shrink
   `move_queue.h`/`sim_harness.h`/`tour.py` header essays to contract-only,
   fix the stale protocol-v2/`source/commands/` references, the "Managed —
   Ruckig" GUI label, and the dangling deleted-issue xfail citations.

## Success Criteria

Full `uv run` pytest suite green, sim tour-closure gate + button-acceptance
suite green, no dead `control.*` key survives, every stale doc reference
in the review's bloat inventory fixed. (`stop_lead_ms` deletion + isolated
90° accuracy are now sprint 118's success criteria, already delivered by
its ticket 004 — verify at this sprint's own detail-promotion that they
still hold, but they are not this sprint's own deliverable.)

## Scope

### In Scope

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

- **(Amended)** Land-at-zero MOVE completion predicate, `stop_lead_ms`
  deletion, and `test_turn_error_characterization.py`'s postcompensation
  test disposition — delivered by sprint 118's ticket 004, not this
  sprint's scope.
- Hardware bench verification — deferred to the phase-B bench session
  that follows both 118 and 119 (see 118's own deferral note; the same
  applies here — this sprint's acceptance bar is the sim suite + closure
  gate + button acceptance, not the stand).
- Any new StateEstimator consumer (fake-OTOS/fusion bench work) —
  `bodyAt()` is quarantined by 118, not wired to a new consumer, this
  sprint either.
- Anything from the review's §6 bloat inventory not named in the four
  remaining issues above (e.g. `Header archaeology` items outside the
  three named files) — out of scope unless discovered to be entangled
  during execution.

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

(Deferred to detail-mode planning. `stop_lead_ms`'s own config/schema
migration is now 118's concern, already delivered by its ticket 004 —
this sprint's remaining config-schema work is the 20-key `control.*`
attic deletion, unrelated to `stop_lead_ms`.)

## Design Overlay

**Amended (2026-07-23):** with land-at-zero delivered by 118, this
sprint's `App::StateEstimator` consumer-count concern is moot (118
already quarantined it) — `src/firm/app` drops off this list unless the
config-boundary ticket (issue 1) turns out to touch `app/DESIGN.md` for
some other reason at detail time. Subsystems this sprint is expected to
touch (for the eventual `seed_sprint_design_overlay` call at
detail-promotion time): `src/firm/motion` (leg hand-off contract
paragraph; `motion/DESIGN.md`), `src/host/robot_radio`
(`configure_from_robot()` default-on push; dead-kwarg deletion in
`tour.py`), `src/tests` (boundary-test un-xfail) — plus
`docs/design/design.md` if the config-boundary default-on push changes
how the system doc describes the sim/production config seam. Per the
flat-overlay-slot precedent set in sprints 116/117/118 (only one
subsystem `DESIGN.md` can occupy the overlay's single `DESIGN.md` slot at
a time), if more than one of `motion`/`robot_radio`/`tests` needs a
`DESIGN.md` edit, the detail-mode planner picks ONE for the overlay slot
and edits the rest directly on their canonical path, same as 118 did for
`src/sim`. Not seeded now — Roadmap Mode does not seed overlays; this
call happens during this sprint's own Phase 2 (Architecture) once
detail-promoted.

## Use Cases

(Deferred — Roadmap Mode. Full Use Cases (SUC-NNN, continuing from
sprint 118's allocation, now through SUC-066) are written at
detail-promotion time, one per issue at minimum: silent-off
config-boundary default, leg hand-off contract. The config-attic/dead-kwarg
deletion and doc-relocation issues are not expected to need their own use
case — they are internal cleanup with no behavior change.)

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

**Amended (2026-07-23): 4 tickets, was 6** — land-at-zero completion and
its test-disposition companion (former tickets 1-2) were delivered by
sprint 118's ticket 004 instead; see Goals/Decision cross-reference to
118's own sprint.md. Not yet created (Roadmap Mode). Anticipated shape,
one ticket per remaining issue, dependency-ordered:

| # (rough) | Title | Depends On | Issue(s) |
|---|-------|------------|----------|
| 1 | Default-on shaping/anticipation push at `configure_from_robot()`; telemetry flag + GUI banner off-state indicator; `docs/protocol-v4.md` append | 118 (delivered — field list must match 118's actual final config surface) | kill-the-silent-off-shaping-config-boundary.md |
| 2 | Leg hand-off contract paragraph (`motion/DESIGN.md`) + boundary-test assertion | 118 (delivered) | specify-and-assert-the-leg-handoff-contract.md |
| 3 | Delete 20 dead `control.*` config keys + 4 dead `run_tour()` kwargs + `DEFAULT_INTER_LEG_SETTLE` | 118 (delivered — `stop_lead_ms` already gone, no coordination needed) | delete-the-config-attic-and-dead-tour-kwargs.md |
| 4 | Relocate narrative comments to DESIGN.md/git; refresh stale doc references | 1, 2, 3 (trims only what survives) | relocate-narrative-comments-and-refresh-stale-docs.md |

Tickets execute serially in the order listed (`worktree: false`, same as
118). Tickets 1-3 are independent of each other (all depend only on 118,
already delivered) and could in principle parallelize; ticket 4 must run
last since it trims only what survives 1-3's deletions. This sprint
stays serial regardless.
