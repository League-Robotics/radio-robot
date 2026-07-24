---
id: '126'
title: OTOS estimator v2 — fusion, sim-first (S2)
status: roadmap
branch: sprint/126-otos-estimator-v2-sim-first
worktree: false
use-cases: []
issues:
- estimator-v2-otos-fusion-sim-first.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 126: OTOS estimator v2 — fusion, sim-first (S2)

> Re-planned per `clasi/issues/replan-sprints-122-plus-to-close-goal-exact-tours.md`
> (was: dead-legacy cleanup, now folded into 127). This sprint puts the OTOS
> into the loop — the stakeholder's standing directive (goal doc,
> non-negotiable 3) — with sim as the proving ground, and ends at the S2 gate.

## Goals

Three stages, per the spine issue (`estimator-v2-otos-fusion-sim-first.md`):

1. Heading/omega fusion ON in sim (the v1 blend already exists; weights are
   0.0). Wrapped-OTOS vs unwrapped-odometry heading discipline designed and
   unit-tested up front.
2. Position (x/y) fusion — NEW capability: schema arm on
   `EstimatorConfigPatch`, StateEstimator blend, and the consumption decision
   (recommended and to be stakeholder-signed: MoveQueue stop conditions and
   the analytic completion consume ESTIMATOR pose; `App::Odometry` remains
   the raw encoder integrator, never corrupted).
3. Promote the realistic-error-profile closure gate from xfail to a HARD S2
   gate: per-motion <=0.5deg / <=5mm; tour net <=1deg, closure <=25mm, with
   fusion active and the calibration push in the loop.

## Problem

The OTOS is wired but unused (weights 0.0, no x/y arm, FAKE_OTOS synthesizes
from encoders); encoder-only estimation cannot see wheel slip — the exact
error class the playfield will add. Sim OTOS == ground truth, so fusion is
validated against a known answer before hardware depends on it.

## Success Criteria

- Ideal-chip S1 gates remain green with fusion enabled (perfect sensor must
  not perturb a perfect estimate).
- Realistic-profile TOUR_1/TOUR_2 pass the S2 bar as hard gates; injected
  encoder-error runs show fused pose tracking truth measurably better than
  encoder-only (numbers stated).
- weights==0 remains byte-identical to today (0-==-off contract, asserted).
- Wrap-discipline unit tests cover +-180deg crossings during blends.

## Scope

### In Scope
- `src/firm/app/state_estimator.*`, `src/firm/app/move_queue.cpp`
  (consumption point), `src/protos`/generated messages (schema arm), host
  `estimator_config()` plumbing, sim gate configs and tests.

### Out of Scope
- Real-OTOS hardware work (128 owns it; mount is a stakeholder precondition
  there). Bench transport (125). Legacy/hygiene (127).

## Dependencies / Sequencing

- After 122 (completion semantics stable) and nominally 124; before 128.
- Stakeholder decision at sprint START: estimator-pose consumption point.

## Architecture / Use Cases / Tickets

Deferred to detail planning. Expected tickets: (1) heading fusion on + wrap
tests; (2) position-fusion schema + blend + consumption; (3) S2 gate
promotion.
