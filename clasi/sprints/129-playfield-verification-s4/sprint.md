---
id: '129'
title: Playfield verification (S4)
status: roadmap
branch: sprint/129-playfield-verification-s4
worktree: false
use-cases: []
issues:
- playfield-verification-s4.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 129: Playfield verification (S4)

> New sprint per the replan directive (gap G4). The goal-closing stage:
> camera-truth gates for all four tours. Spine issue:
> `playfield-verification-s4.md`.

## Goals

Scriptable camera-truth measurement (per-leg commanded-vs-achieved from the
aprilcam feed, calibration residual stated); TOUR_1/2/3/4 on the playfield
with the fused, calibrated robot, N>=3 runs each; divergence-vs-bench
attributed with measurements; goal doc flipped to met (or the named floor
recorded per stage rule).

## Success Criteria

- All four tours inside the S4 bar (per-motion <=2deg/<=2%; closure <=100mm)
  on camera, N>=3, numbers published; TOUR_3 spiral/scallop and TOUR_4
  crossing-offset checks pass on camera-truth data.
- The S4 gate is repeatable by the stakeholder from the TestGUI or one bench
  command.
- `docs/design/goal-exact-tours.md` status updated by this sprint's close.

## Dependencies / Sequencing

After 128. Requires playfield + camera rig session time (stakeholder hands).

## Architecture / Use Cases / Tickets

Deferred to detail planning. Expected tickets: (1) camera-truth gate
tooling; (2) four-tour S4 runs + attribution; (3) goal-doc closeout.
