---
title: Turn lead compensation needs gain co-tuning to deliver exact turns (109-010
  follow-up)
filed: 2026-07-17
filed_by: team-lead (sprint 109 close-out)
status: pending
---

# Turn lead compensation needs gain co-tuning to deliver exact turns

Sprint 109 ticket 010 built the full turn-error characterization and
lead-compensation machinery but shipped **neutralizing defaults** — turns are
NOT yet exact with an ideal OTOS. This issue carries the remaining work.

## What 010 delivered (all in place, tested, committed)

- Rate-sweep characterization harness (deterministic stepped sim,
  `src/tests/testgui/test_turn_error_characterization.py`); fitted baseline
  error model `error_deg ≈ +0.05·yaw_rate_max[rad/s] − 0.14`.
- Root cause of the dominant staleness: **cycle ordering** — `applyOtosSample()`
  runs in the kPace block AFTER `Pilot::tick()` reads heading, so the PD always
  sees the previous cycle's OTOS pose (~one kCycle = 40 ms constant staleness),
  on top of `Otos::kReadPeriod` (20 ms) sampling.
- Three independently tunable PlannerConfig lead loci: `heading_lead_bias`
  (measurement-age projection on the PD input), `plan_lead` (wheel-velocity
  reference lead), `terminal_lead` (predicted-state completion test).
  Divergence checking stays un-led (ticket 006 lesson).
- Sim-fidelity fix: `OtosPlant` VELOCITY_XL registers were hardcoded zero
  (silently defeating any omega-based age projection in every sim test) — now
  real finite-difference omega.

## What's missing (this issue)

A raw lead at the sprint's `heading_kp = 6.0` **regresses** tours (faults);
010's grid sweep found no (heading_lead_bias, plan_lead, terminal_lead)
combination that both avoids regression and beats ticket 009's baseline
(ideal-chip residuals 0.4–2.2°; realistic 12/13 turns ≤1°, one 4.9° outlier on
TOUR_2 leg 14 tied to the same latency mechanism).

Likely path (from the 010 analysis): the lead and the PD gain must be
**co-tuned** — kp=6 was bench-tuned against terminal stiction on real hardware
with no lead; with a properly led measurement the loop's effective phase margin
changes, so sweep (kp, kd, leads) jointly rather than leads alone. Consider
also whether an architectural fix to the cycle ordering (sample OTOS before
Pilot::tick in the schedule — a robot_loop.cpp change with bus-timing
implications) beats compensating for it in software.

## Acceptance sketch

- Ideal-OTOS turns exact to plant epsilon in sim (closes 009's deferred
  criterion); error-vs-rate slope ≈ 0 post-compensation.
- TOUR_2 leg-14 outlier collapses under the realistic profile.
- Tours remain 100% reliable; full suite green.
- Values that work land as persisted PlannerConfig defaults (per-robot JSON
  where hardware differs from sim); bench re-tune on the real robot before
  trusting hardware turns.
