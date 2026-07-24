# Goal: Exact Tours — sim, then bench, then playfield

**Owner:** Eric Busboom · **Stated:** 2026-07-23 · **Status:** governing target
for all motion work until met. Every motion-accuracy sprint cites this document
and states which stage and which bar it advances.

## The goal, in the stakeholder's words

Send the robot a command to drive a distance, turn an angle, or drive a curve,
and it ends up where it was told — accurately, repeatably, without hand-tuned
fudge factors. The simulation tour is 100% deterministic: every error it shows
is manufactured by our own code and can therefore be removed entirely. Sim
must be driven to exact FIRST, because any error tolerated in sim is error we
will never be able to attribute on hardware. Then the same tours run on the
bench as close to exact as the hardware allows, using the OTOS — which was
bought and mounted for exactly this — for heading and distance. Then the
playfield, measured against camera truth.

## Stage bars

Numbers are gates, not aspirations: each stage's bar becomes a hard (non-xfail)
CI/bench gate when its sprint closes, and never loosens afterward. "Per motion"
= one Move (straight, turn, or arc), measured against sim ground truth or
bench/camera reference from the motion's own start pose.

| Stage | Scope | Per-motion bar | Tour bar (TOUR_1/2/3/4) |
|---|---|---|---|
| S1 | Sim, ideal chip (deterministic) | heading ≤ 0.1°, position ≤ 1 mm | net heading ≤ 0.5°, closure ≤ 5 mm, per-leg heading gain on straights ≤ 0.1° |
| S2 | Sim, realistic error profile + OTOS fused | heading ≤ 0.5°, position ≤ 5 mm | net heading ≤ 1°, closure ≤ 25 mm |
| S3 | Bench (stand), calibrated robot, real OTOS fused | heading ≤ 1°, position ≤ 1% of commanded | net heading ≤ 3°, closure ≤ 50 mm |
| S4 | Playfield, camera-verified | heading ≤ 2°, position ≤ 2% | closure ≤ 100 mm and visually clean patterns (TOUR_3 circle, TOUR_4 crossings) |

Rationale for S1's numbers: the sim plant is exact and float32 pose
accumulation over a 13-leg tour is below 0.01°/0.1 mm — everything above that
is termination/hand-off semantics we control. S1 is achievable outright; if a
sprint claims it cannot reach the bar, the claim must name the physical
mechanism and its floor, with a measurement.

## Non-negotiables

1. **No tuned compensation constants.** Any constant in the motion path must be
   derived from named physical quantities (cycle time, decel ceiling, plant τ,
   measured trackwidth). A constant that needs re-sweeping when an unrelated
   stage changes is a defect (see `stop_lead_ms`, deleted 118-004, four
   retunes; see the chain-margin sweep pockets, 118-003/119-005). Where an
   epsilon is unavoidable (termination predicates), it is stated in physical
   units with its derivation, and tightening it must not require re-tuning
   anything else.
2. **Errors are removed at their source, not absorbed downstream.** A stage
   that adds a compensator for an upstream defect is rejected in review.
3. **The OTOS is used.** Optical-flow odometry exists on this robot to provide
   heading and position independent of wheel slip. Target state: estimator
   fuses OTOS heading AND position; encoder odometry remains the fallback,
   never the only source. Sim first (where OTOS ≡ truth, so fusion is
   validated against a known answer), bench second (after the mount is made
   rigid and `otos_untrusted` is cleared for the tour robot), playfield third.
4. **Determinism is the debugging asset.** The deterministic-stepped sim gate
   is the reference for every accuracy claim; real-time/threaded and bench
   runs are validations, not tuning environments.
5. **Completion means at rest on the target** (or, for chained same-axis
   compatible legs, at the carried velocity on the target). A motion that
   reports done while still carrying uncommanded speed into the next leg is
   not done.

## Current position vs the bars (2026-07-23, v0.20260723.3)

- S1: straights from rest are exact (+0.00°); final turns ≈ 0.3°; chained
  turns ±2.2°; straights after turns +1.3–4.2° each; TOUR_1 net +17.9°.
  **Not met — every miss traced to the chain-boundary completion margin**
  (`land-at-zero-at-orthogonal-chain-boundaries.md`, sprint 121) plus absent
  heading hold (sprint 123).
- S2: blocked on estimator v2 (OTOS fusion) — **not yet planned in any sprint**.
- S3: tours complete 13/13 on the bench (fake OTOS), closure 750–1370 mm /
  120–155° on an uncalibrated robot — **accuracy campaign not yet planned**;
  transport reliability is sprint 125; mount fix is a physical prerequisite.
- S4: not started (camera-truth tooling exists in the TestGUI).

## Definition of done

All four stage gates green and permanent: the four tours run in sim
(deterministic and realistic-profile), on the bench, and on the playfield,
inside the bars above, with OTOS fusion active from S2 onward, zero tuned
compensation constants in the motion path, and every gate a hard assert.
