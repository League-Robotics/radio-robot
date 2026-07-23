---
status: in-progress
filed: 2026-07-22
filed_by: team-lead (turn-execution review R1/F3/D2, claims verified against code)
related:
- restore-the-interleaved-request-settle-tick-loop-schedule.md
sprint: '118'
tickets:
- 118-002
---

# Stop decision must consume this-cycle odometry (MoveQueue tick after integrate)

## Description

`MoveQueue::tick()` — the stop decision — runs at `robot_loop.cpp:586` while
`odom_.integrate()` runs later at `:619`, so every stop decision uses odometry
integrated at the END of the previous cycle: one full cycle of heading
staleness (5.7° at 2 rad/s on the 50 ms sim cycle; ~2.3° at 20 ms). The
StateEstimator basis is likewise last-cycle (`stateEstimator_.update()` at
`:629`). The 45 ms anticipation lead exists in large part to cancel this
ordering choice one screen away.

Verified (2026-07-22): both `MoveQueue::tick()` and `Odometry::integrate()`
are pure compute with no bus traffic (`move_queue.cpp` reads cached odometry;
`odometry.cpp:17-18` reads cached motor positions; `state_estimator.cpp` is
arithmetic). The encoder samples `integrate()` consumes are collected earlier
in the same cycle by the motor ticks, so relocating `integrate()` or the
MoveQueue tick within the cycle does not change which encoder data exists —
only whether the stop decision observes it.

**Constraint — `applyOtosSample()` is NOT pure**: it performs the rate-limited
OTOS I2C burst read (`odometry.cpp:52` → `otos.cpp:90,113`) and is deliberately
fenced in the pace block outside any motor request/collect window
(`robot_loop.cpp:606-611`). Do not drag it into a settle window.

## Proposed fix

Within the restored interleaved schedule (see
`restore-the-interleaved-request-settle-tick-loop-schedule.md`, whose
stakeholder-confirmed skeleton places `moveQueue_.tick()` in the R-settle
block BEFORE the pace block's `odom_.integrate()` — preserving exactly the
staleness this issue removes): move `moveQueue_.tick()` (+ its completion
ack/fault staging) INTO the pace block, AFTER `applyOtosSample()` →
`odom_.integrate()` → `stateEstimator_.update()`. All three relocated pieces
are pure compute, so the borrowed-window bus discipline is untouched.
`drive_.tick()` stays where that issue puts it (R-settle block); the staged
stop/twist reaches motor duty at the next cycle's motor ticks — that one
cycle of decision-to-duty latency remains and is acceptable once the
land-at-zero taper (companion issue
`land-at-zero-completion-delete-stop-lead.md`) makes terminal ω small.

Note the interleave issue's R-settle block then holds `processMessage(cmd)`
and `drive_.tick()` only. A MOVE that arrives and activates in cycle N gets
its first shaped stage at N's pace block; fine.

Reconciliation with the interleave issue is REQUIRED, not optional — whoever
implements second must not silently revert the other's ordering. Implement
both in one ticket or adjacent tickets in the same sprint.

## Acceptance

- Stop decision reads odometry/estimator state updated in the SAME cycle
  (assert via `app_robot_loop_harness` ordering test).
- Interleave schedule invariants preserved (per-port select→settle→collect,
  no bus traffic in settle windows, fault bit 0 clear).
- Sim tour closure gate passes at current or better bands.
