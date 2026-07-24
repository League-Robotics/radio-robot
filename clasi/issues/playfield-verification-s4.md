---
status: pending
filed: 2026-07-23
filed_by: team-lead (replan Step 0; review §8, gap G4)
related:
- replan-sprints-122-plus-to-close-goal-exact-tours.md
- bench-accuracy-campaign-s3.md
- tour-3-icosagon-and-tour-4-infinity-test-patterns.md
tickets: []
sprint: '129'
---

# Playfield verification (S4): camera-truth gates for all four tours

## Description

The final stage of `docs/design/goal-exact-tours.md`: the four tours run on
the physical playfield, measured against the camera/AprilTag truth the
TestGUI already produces, inside the S4 bar (per-motion heading ≤2°,
position ≤2% of commanded; tour closure ≤100 mm; TOUR_3 visually a clean
circle; TOUR_4 crossings within band on camera). Closing this flips the goal
document to "met."

## What to do

1. Camera-truth measurement path made scriptable: capture per-leg camera
   poses during a tour run (the aprilcam/tracker feed the playfield overlay
   already consumes) and emit the same per-leg commanded-vs-achieved table
   the sim and bench gates use. Camera calibration checked against the
   board (`camera_distance_scale`, homography residual stated).
2. Run TOUR_1/2/3/4 on the playfield, fused estimator active, calibrated
   robot; N≥3 runs each; publish the numbers table.
3. Divergence triage: any playfield-vs-bench delta is attributed (floor
   surface/slip → the OTOS's whole purpose; camera measurement band; battery
   sag) with a measurement, not a guess.
4. Goal-doc closeout: update `docs/design/goal-exact-tours.md` status to met
   (or record the named, measured floor per stage rule).

## Acceptance

- All four tours inside the S4 bar on camera, N≥3, numbers published in the
  sprint record.
- TOUR_3 spiral/scallop check passes on camera-truth vertices; TOUR_4
  crossing-offset measured on camera within band.
- The S4 gate script is repeatable by the stakeholder from the TestGUI or a
  one-line bench command.

## Sequencing

After the S3 bench campaign. Requires: playfield + camera rig session time
(stakeholder hands), calibrated fused robot from S3.
