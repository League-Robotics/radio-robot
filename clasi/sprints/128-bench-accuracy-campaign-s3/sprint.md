---
id: '128'
title: Bench accuracy campaign (S3)
status: roadmap
branch: sprint/128-bench-accuracy-campaign-s3
worktree: false
use-cases: []
issues:
- bench-accuracy-campaign-s3.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 128: Bench accuracy campaign (S3)

> New sprint per the replan directive (gap G2). Takes the S1/S2-exact
> firmware to the S3 bar on the stand. Spine issue:
> `bench-accuracy-campaign-s3.md`.

## Goals

Calibrate the tour robot (velocity loop incl. the vel_ki droop work pulled
from later/, per-wheel travel + trackwidth, OTOS OL/OA scales, per-robot
tau_plant re-measurement), enable real-OTOS fusion on hardware (PHYSICAL
PRECONDITION, stakeholder task, surfaced at sprint start: rigid OTOS remount
+ clear `otos_untrusted`), and land a numeric S3 bench gate mirroring the sim
gate's per-leg assertions.

## Success Criteria

- Calibrated robot passes the S3 bar (per-motion heading <=1deg, position
  <=1% of commanded; tour net <=3deg, closure <=50mm) on TOUR_1 and TOUR_2
  on the stand, repeatably (N runs stated, all numbers published).
- Single-motion S3 checks pass both directions (90/360 turns, 700mm
  straight, one arc).
- Every calibration value in the robot JSON with derivation; zero swept
  controller constants; `otos_untrusted` cleared; fusion weights nonzero on
  the bench robot; encoder-only fallback selectable as a diagnostic mode.

## Dependencies / Sequencing

Prereqs: 125 (transport), 126 (estimator v2), 127 bit-6 (truthful flags),
stakeholder remount. Baseline: 120-002's completion-only record
(closure 750–1370mm / 120–155deg, uncalibrated) is the "before" number.

## Architecture / Use Cases / Tickets

Deferred to detail planning. Expected tickets: (1) calibration battery +
persistence; (2) real-OTOS fusion on hardware + A/B; (3) S3 bench gate
script + record.
