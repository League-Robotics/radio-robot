---
status: pending
review: docs/code_review/2026-07-01-full-codebase-review.md
findings: CR-06
severity: high
sprint: '065'
---

# OTOS fused despite persistent WARNING bits — "spin on placement" regression re-opened

## Problem

`Robot::otosCorrect` documents a two-tier gate (READABLE vs HEALTHY, D9 /
027-005): fuse into the EKF only when `otosStatus == 0`. A 2026-06-17 change
set `healthy = poseOk`
([Robot.cpp:200-267](../../source/robot/Robot.cpp)), so a robot with
`warnOpticalTracking` persistently set (lifted, on the stand, freshly placed)
has its **frozen** OTOS pose and near-zero velocity fused. The Mahalanobis
gates reject the frozen observation only temporarily: the EKF gate-recovery
paths force-snap position and heading to the OTOS value after 10 consecutive
rejections ([EKFTiny.cpp:217-250](../../source/state/EKFTiny.cpp),
[EKFTiny.cpp:420-437](../../source/state/EKFTiny.cpp)). Net: hold the robot
in the air while wheels spin, or carry it to a new spot, and within ~10 OTOS
samples the fused pose/heading snaps to stale garbage — the exact
precondition the D9 gate was written to prevent ("spin on placement").

The 06-17 rationale (transient warn bits shouldn't drop fusion entirely) is
legitimate; the implementation lost the transient-vs-persistent distinction.

## Fix direction

Gate fusion on warn-bit **persistence**: fuse through ≤ K consecutive warn
samples (transient), block fusion after (persistent), re-admit after N clean
reads. Keep raw telemetry visibility unchanged.

## Acceptance / tests

- Sim needs a "warn-bit-set-but-readable" OTOS state (currently only
  `setLift` = read failure exists) so this gate is testable.
- Test: with warn persistently set and wheels spinning, fused pose follows
  encoders (no snap to frozen OTOS); with a 1–2 sample warn blip, fusion
  continues normally.
