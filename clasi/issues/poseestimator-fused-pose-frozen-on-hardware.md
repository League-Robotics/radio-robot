---
status: pending
---

# PoseEstimator fused pose frozen on real hardware — blocks TURN completion and G arrival

## Description

During sprint 089 ticket 007 bench verification (2026-07-07, robot Tovez on
the stand, fw 0.20260707.18), `PoseEstimator`'s fused pose (`TLM pose=`) did
not accumulate from real wheel motion: a `G` run covered **1.3+ m of real
encoder travel** while `pose=` stayed frozen at `(0, 0, -7)` the entire run.

Consequences on real hardware:

- `TURN` never completes — its `STOP_HEADING` stop condition reads
  `fusedPose.pose.h`, which never changes.
- `G` never arrives — target-region detection depends on the fused pose.
- `RT` is unaffected — its stop condition is the raw encoder-arc
  differential, and it completed correctly on the bench.

`PoseEstimator` was explicitly unchanged by sprint 089 (architecture-update
Decision 9), so this is a **pre-existing defect**, not a Ruckig-migration
regression — but it blocked ticket 007's TURN accuracy criterion and the G
settle smoke check, and it must be fixed before those can be re-verified.

## Evidence

- Sprint 089 `bench-verification-log.md` (commit `2f809195`) — full TLM
  traces showing `enc=` climbing ~1.3 m while `pose=` stays constant.
- Cross-checked against `DEV M n STATE` per-motor registers to rule out a
  telemetry artifact.

## Needed

1. Root-cause `PoseEstimator` fusion on the real robot (encoder-odometry
   ingestion path on hardware — sim is unaffected; ticket 006's full sim
   suite is green).
2. After the fix, re-run the bench pass for `TURN` (accuracy vs. the
   086/087 tolerance bars) and the `G` settle smoke check from sprint 089
   ticket 007.
