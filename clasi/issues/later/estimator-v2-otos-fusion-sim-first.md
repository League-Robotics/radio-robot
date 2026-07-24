---
status: pending
filed: 2026-07-23
filed_by: team-lead (replan Step 0; review §5, gap G1)
related:
- replan-sprints-122-plus-to-close-goal-exact-tours.md
- heading-hold-during-distance-moves.md
tickets: []
sprint: '126'
---

# Estimator v2: OTOS fusion, sim-first (heading + position) → S2 gate

## Description

The OTOS exists to provide body-frame heading AND position independent of
wheel slip, and it is currently unused: `weight_heading_otos =
weight_omega_otos = 0.0` everywhere; the wire schema has NO position-fusion
arm at all (`EstimatorConfigPatch` carries no x/y weight;
`BodyEstimate.x/y` documented encoder-only); `MoveQueue` stop conditions and
`Odometry` consume encoders exclusively; 120-002's FAKE_OTOS synthesizes
`frame.otos` FROM the encoders (plumbing exerciser — adds zero information by
construction). Stakeholder directive (goal doc, non-negotiable 3): the OTOS
is used, sim-first.

Sim is the correct on-ramp because sim OTOS ≡ ground truth (zero-error
knobs), so fusion is validated against a known answer before any hardware
depends on it.

## What to do (three stages, one sprint)

1. **Heading/omega weights ON in sim (code exists).** Enable the existing v1
   complementary blend (`StateEstimator::update()`) with nonzero
   `weight_heading_otos`/`weight_omega_otos` in the sim gate configs; verify
   ideal-chip gates unchanged (OTOS ≡ truth) and realistic-profile accuracy
   improves. Wrapped-OTOS-heading vs unwrapped-odometry-theta discipline is
   designed and unit-tested UP FRONT (chip reports wrapped; `Odometry::theta()`
   is unwrapped by contract; the blend must unwrap against the current
   estimate) — this is the one known trap, do not discover it in a gate.
2. **Position fusion (new capability).** Schema: add x/y weight (+ reuse
   staleness) to `EstimatorConfigPatch`; `StateEstimator` blends OTOS x/y
   onto the body peer. **Consumption decision (stakeholder sign-off):**
   recommended — `MoveQueue`'s stop conditions and `landAtZero`/analytic
   completion consume ESTIMATOR pose; `App::Odometry` remains the raw
   encoder integrator, never corrupted (clean fallback, clean A/B).
3. **S2 gate.** Promote the realistic-error-profile closure gate from xfail
   to a hard gate at the S2 bar (per-motion ≤0.5°/≤5 mm; tour net ≤1°,
   closure ≤25 mm) with fusion active and the calibration push in the loop.

## Acceptance

- Ideal-chip S1 gates remain green with fusion enabled (fusion of a perfect
  sensor must not perturb a perfect estimate).
- Realistic-profile TOUR_1/TOUR_2 pass the S2 bar as HARD gates.
- Injected encoder-error runs (scale err, slip) show fused pose tracking
  truth measurably better than encoder-only — numbers stated.
- Fusion OFF (weights 0) remains byte-identical to today's behavior — the
  0-==-off contract, asserted.
- Wrap-discipline unit tests cover ±180° crossings during blends.

## Sequencing

After 122 (completion semantics stable) and nominally after 124; before the
bench accuracy campaign (S3 depends on real-OTOS fusion existing). No
hardware required.
