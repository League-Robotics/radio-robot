---
id: '122'
title: Analytic completion & same-axis carry — margin machinery deleted
status: roadmap
branch: sprint/122-same-axis-carry-through-chain-margin-cleanup
worktree: false
use-cases: []
issues:
- chain-advance-reset-defeats-same-axis-compatible-leg-continuity.md
- chain-advance-completion-margin-narrow-pocket.md
- land-at-zero-at-orthogonal-chain-boundaries.md
- s1-gate-ratchet-harden-ideal-chip-gates-at-goal-bars.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 122: Analytic completion & same-axis carry — margin machinery deleted

> Re-planned per `clasi/issues/replan-sprints-122-plus-to-close-goal-exact-tours.md`
> and sprint 121-003's close-out finding (2026-07-23). This sprint is the S1
> keystone: it deletes every completion margin constant, replaces them with
> one derived rule, restores same-axis carry, and hardens the S1 gates.

## Goals

Three jobs, one theme — completion semantics become physics, not tuning:

1. **Analytic completion replaces ALL margin machinery.** 121-003 proved
   (and the team-lead boundary trace confirmed, numbers below) that no
   fraction of a COMMANDED-speed envelope can represent the PLANT's coast:
   under the taper the plant lags the command by `alpha_decel * tau_plant`
   (7 x 0.13 ~= 0.9 rad/s — measured 0.96–1.34 rad/s at the ack instant), so
   a tight margin crosses the threshold "at cmd ~= 0" yet still coasts ~7deg
   (the 0.92 -> 8deg inversion), and a loose one fires early and leaks
   2–4deg into the next leg. Replace `kStoppingMarginFactorChain` (0.48),
   `kStoppingMarginFactorOrthogonal` (0.67 — 121-003's own labeled interim
   defect marker), `kStoppingMarginFactorFinal` (0.92), and
   `kDiscretizationCyclesChain` with ONE derived firing rule on MEASURED
   speed:

       Angle:    remaining <= |omega_measured| * (kCycle/2 + tau_plant)
       Distance: remaining <= |v_measured|     * (kCycle/2 + tau_plant)

   applied uniformly to final, orthogonal-chain, and (as the terminal
   condition under carry) same-axis boundaries. `tau_plant` enters the robot
   JSON as ONE new named, bench-derived constant (plant_harness
   characterization 0.12–0.14 s) — a measured physical quantity per the
   replan's standing rule 3, not a swept margin. **No sweeping anywhere in
   this sprint: if the analytic form misses its numbers, the model is wrong —
   re-derive (e.g. a second-order coast term), never tune.**
2. **Same-axis carry restored.** The unconditional completing-axis shaper
   reset defeats SUC-003/SUC-051 (dip to 24 mm/s at a compatible
   Distance->Distance boundary vs the 90%-of-v_max floor). Make the reset
   conditional on the incoming Move sharing the ending Move's stop-kind axis
   and sign (the `sameAxisCompatible()` split 121-003 already landed is the
   scaffolding).
3. **S1 gate ratchet.** With 1–2 landed, convert the ideal-chip gates to
   permanent hard asserts at the goal-doc S1 bar (per-motion <=0.1deg/<=1mm;
   tour net <=0.5deg, closure <=5mm, per-leg straight gain <=0.1deg) — see
   the ratchet issue for the named-floor escape (stakeholder adjudicates;
   tolerances never loosen).

## Problem

Current measured state (deterministic sim, ideal chip, HEAD=121-003 commit
81fa7858): TOUR_1 net +21.0deg over 540; straights after turns +1.2–2.8deg
each; turns +0.7–2.3deg; single-boundary trace: predicate fires at +90.95
with plant omega 1.34 rad/s, coasts to +93.7. Root cause per Goals-1:
commanded-envelope margins cannot express plant coast. Separately, same-axis
boundaries dip to 16% of v_max (reset defeats carry). Both are
completion/hand-off semantics — the last error sources standing between this
codebase and S1.

## Solution (plan of record)

- `MoveQueue::landAtZero()` -> analytic completion: fire when remaining is
  inside the measured-speed coast envelope (formulas above). Measured speed
  comes from the same-cycle odometry twist the tick already has (post-118
  ordering). Delete the four margin constants and their comment archaeology;
  `move_queue.cpp`'s anonymous-namespace sweep history moves to DESIGN.md as
  a closed chapter.
- Conditional reset per Goals-2; orthogonal boundaries keep the reset (the
  residual is near-zero once analytic completion fires correctly).
- Gate work per Goals-3, including per-motion gates (90/360 turn, 700 mm
  straight) alongside the tour gates.

## Success Criteria

- The margin/discretization constants NO LONGER EXIST in firmware; grep-clean.
- Deterministic sim, ideal chip: straights following turns gain <=0.3deg
  each; turn legs |error| <=0.5deg; TOUR_1 net 540deg +-1deg — and then the
  ratchet: S1 bar met (<=0.1deg/motion, tour <=0.5deg) or the physical floor
  is named with a measurement and stakeholder sign-off.
- `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
  passes with the 90% no-dip floor intact (or an explicit stakeholder-accepted
  bounded-recovery alternative).
- 121's orthogonal land-at-zero behavior strictly improves (per-boundary
  leak <=0.3deg, from measured 2–4deg).
- Full suite green; S1 gates run hard (no xfail) in the default suite.

## Scope

### In Scope

- `App::MoveQueue` completion predicate + reset conditionalization
  (`src/firm/app/move_queue.cpp`); `tau_plant` config key + boot plumbing;
  gate hardening in `src/tests/testgui/test_tour_closure_gate.py` plus a new
  per-motion gate file.

### Out of Scope

- Heading-hold (123); new tours (124); any host/tour-runner change beyond
  gate tests; OTOS fusion (126-replanned).

## Dependencies / Sequencing

- After 121 closes (stakeholder decision 2026-07-23: Accept + defer, with
  the amendment recorded in 121's close-out).
- Blocks 123/124's acceptance numbers; independent of 125.

## Architecture

Compact: one firmware module (`App::MoveQueue`), one config key, gate tests.
The analytic rule is the derived version of what the deleted `stop_lead_ms`
and the margin family approximated by sweep — see
`docs/code_review/2026-07-23-exactness-review.md` §2.1 and 121's close-out.

## Use Cases

Refines SUC-003/SUC-051 (seamless same-axis hand-off) and SUC-074 (land at
zero) — SUC-074's accuracy numbers transfer here and tighten to the S1 bar.

## Tickets

Detail-planning to cut approximately:

1. Analytic completion (measured-speed coast rule, margin deletion,
   `tau_plant` config key).
2. Conditional completing-axis reset (same-axis carry) + the no-dip gate.
3. S1 gate ratchet (hard gates, per-motion + tour, ratchet rule recorded).
