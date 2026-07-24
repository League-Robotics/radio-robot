---
status: pending
filed: 2026-07-23
filed_by: team-lead (replan Step 0; review §4, gap G2)
related:
- replan-sprints-122-plus-to-close-goal-exact-tours.md
- estimator-v2-otos-fusion-sim-first.md
- bench-move-commands-intermittently-never-reach-firmware.md
tickets: []
sprint: '128'
---

# Bench accuracy campaign (S3): calibrate, fuse real OTOS, gate with numbers

## Description

Sprint 120 made bench tours COMPLETE (13/13, twice). Accuracy is untouched:
closure 750–1370 mm / 120–155° per 120-002's own record, on an uncalibrated
robot. No sprint currently plans bench accuracy — this is the campaign that
takes the S1/S2-exact firmware to the S3 bar on the stand
(per-motion heading ≤1°, position ≤1% of commanded; tour net ≤3°, closure
≤50 mm — `docs/design/goal-exact-tours.md`).

## Physical precondition (STAKEHOLDER TASK — surface at sprint start)

Remount the OTOS rigidly to the chassis on the tour robot and clear
`geometry.otos_untrusted` in its JSON. No hardware fusion work starts until
this is done; schedule the sprint around it.

## What to do

1. **Calibration battery** (workflows largely exist under `calibration/`):
   - Velocity loop: tune per-robot gains off the neutral profile; includes
     the pure-P droop / terminal-wedge work
     (`later/nocal-straight-terminal-wedge-needs-velocity-integrator.md` —
     pull it out of later/): vel_ki with anti-windup, verified against the
     15 mm/s deadband at land-at-zero's low-speed tails.
   - Wheel geometry: per-wheel travel calib + trackwidth (existing
     linear/angular calibration flows), persisted via the tuning store.
   - OTOS scales: OL/OA calibration against measured motion (existing flow).
   - τ_plant: bench re-measurement of the actuation time constant (the
     analytic completion's named constant — plant_harness characterized
     0.12–0.14 s; confirm per-robot, record in the robot JSON).
2. **Real-OTOS fusion on hardware:** enable estimator v2 weights on the
   bench robot; A/B encoder-only vs fused on the stand (heading first, then
   position); FAKE_OTOS build retires to a diagnostic tool.
3. **S3 bench gate:** a scripted stand gate mirroring the sim gate's
   per-leg TRUE-heading/position assertions (reference: encoder+OTOS fused
   vs commanded; camera cross-check where the rig allows), run on demand and
   before every hardware-touching sprint closes. Numbers, not adjectives, in
   the gate output.

## Acceptance

- Calibrated robot passes the S3 bar on TOUR_1 and TOUR_2 on the stand,
  repeatably (state N runs, all numbers).
- Single-motion S3 checks pass both directions (90/360 turns, 700 mm
  straight, one arc).
- Every calibration value lands in the robot JSON with its derivation note;
  zero swept controller constants introduced.
- `otos_untrusted` cleared; fusion weights nonzero on the bench robot;
  encoder-only fallback still selectable and gated as a diagnostic mode.

## Sequencing

Prereqs: 125 (transport trustworthy), 126-replanned (estimator v2 exists),
127-replanned bit-6 fix (fault flags trustworthy on the stand), stakeholder
remount. This sprint is the S3 stage gate; playfield (S4) follows it.
