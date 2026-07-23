---
source_file: DESIGN.md
source_hash: 0d3bb89f8e6fcc04a6f69f8f0505e162c3b1f6592f337ea69ea85a69aee4c347
---
# Diff: DESIGN.md

Comparison of the sprint overlay copy of `DESIGN.md` against its pristine (seed-commit) canonical version.

```diff
--- DESIGN.md (pristine)
+++ DESIGN.md (current)
@@ -89,24 +89,53 @@
 argument, mirroring `Motion::StopCondition`'s "hand-fed readings, no
 owned collaborator" shape — see that module's own file-header precedent).
 
+**118 (loop schedule truth) — landed.** Restores `cycle()`'s schedule to
+what this file already claimed it was: `kSettle`/`kClear` back to their
+genuine 4ms vendor-settle/clearance budget (regressed to 0 by commit
+`5f5a2ba7`, which had been satisfying the vendor's mandatory settle as a
+*blocking* sleep hidden inside `motorL_.tick()`/`motorR_.tick()` instead —
+tripping the I2C clearance safety-net fault bit every cycle), `kCycle`
+40ms/~25Hz (was a fictional 20ms/~50Hz under the regression), and
+`Telemetry::kPrimaryPeriod` coupled back to `kCycle` (40). Two call-order
+changes ride along: `drive_.tick()` moves back inside the R-settle block
+(retiring the 112-005 "hoist `drive_.tick()` above both motor ticks"
+experiment, which had been tracked only in project memory, not in an
+issue — the interleaved order restored here is the one this file's §2/§4
+already described); and `moveQueue_.tick()` — the stop decision — moves
+from the R-settle block into the trailing pace block, evaluated AFTER
+`applyOtosSample()`/`odom_.integrate()`/`stateEstimator_.update()` rather
+than before them, so a MOVE's completion decision reads odometry
+integrated in the SAME cycle, not the previous one (closes a full cycle
+of avoidable heading/distance staleness the `stop_lead_ms` anticipation
+constant had been partly compensating for — see the turn-execution review
+`docs/code_review/2026-07-22-turn-execution-review.md` D2/F3; deleting
+`stop_lead_ms` itself is a later sprint's job, sequenced after this one).
+
 ## 2. Orientation
 
 `RobotLoop` has two phases. `boot()` steps `Preamble` until every device
 leaf reaches a terminal state (present-and-ready or confirmed-absent),
 emitting a boot telemetry frame each pass; commands are not consumed during
-boot. `cycle()` is the steady-state loop body: request/settle/collect/PID
-for the left motor, decode at most one inbound command (`Comms::pump`),
-apply it (`processMessage`), request/settle/collect/PID for the right motor,
-the unconditional `moveQueue_.tick(now, odom_)` call, then a trailing block that samples OTOS, integrates
-odometry (`Odometry::integrate`), refreshes `App::StateEstimator`'s
-predict-to-now estimates from that same cycle's staged `Frame` (117 —
-see below), polls line/color at a rate-limited, alternating cadence
-(`updateLineColor()` — see below), and paces the whole cycle. `Telemetry::emit()` is called once per cycle and decides for itself
-whether to send the primary frame, the secondary diagnostic frame, or (on a
-tie) alternate between them. `Drive`, `Odometry`, and `MoveQueue` are pure,
-bounded, non-bus-touching helpers that `RobotLoop` calls at specific points
-in its own schedule; `MoveQueue::tick()` is called unconditionally once per
-cycle and drains to `Drive::stop()` once its queue empties.
+boot. `cycle()` is the steady-state loop body, interleaved per port (118 —
+select L → collect L → select R → collect R, restoring the schedule this
+section always claimed): request/settle(borrow: `Comms::pump`)/collect/PID
+for the left motor, a post-duty clearance window (borrow: telemetry
+assembly + emit), request/settle(borrow: `processMessage` +
+`Drive::tick()`)/collect/PID for the right motor, then a trailing pace
+block that samples OTOS, integrates odometry (`Odometry::integrate`),
+refreshes `App::StateEstimator`'s predict-to-now estimates from that same
+cycle's staged `Frame` (117), evaluates the `MoveQueue`'s unconditional
+per-cycle stop decision (`moveQueue_.tick(now, odom_)` — 118: relocated
+here, AFTER odometry/estimator refresh, so the decision reads THIS
+cycle's data, not last cycle's), polls line/color at a rate-limited,
+alternating cadence (`updateLineColor()` — see below), and paces the
+whole cycle. `Telemetry::emit()` is called once per cycle and decides for
+itself whether to send the primary frame, the secondary diagnostic
+frame, or (on a tie) alternate between them. `Drive`, `Odometry`, and
+`MoveQueue` are pure, bounded, non-bus-touching helpers that `RobotLoop`
+calls at specific points in its own schedule; `MoveQueue::tick()` is
+called unconditionally once per cycle and drains to `Drive::stop()` once
+its queue empties.
 See `robot_loop.cpp` for the exact call order — it is the schedule's single
 source of truth.
 
@@ -236,7 +265,9 @@
 such blocks: left-motor settle, post-duty clearance, right-motor settle,
 and a final perception+odometry+pace block. The four gaps
 (`kSettle`, `kClear`, `kSettle`, `kPace`) are sized so their sum equals the
-whole-cycle target `kCycle` (20ms / ~50Hz) — `kPace` is *derived* as
+whole-cycle target `kCycle` (40ms / ~25Hz — 118: restored from a fictional
+20ms/~50Hz that `kSettle=kClear=0` had been faking, see §1's "118 (loop
+schedule truth)" note) — `kPace` is *derived* as
 `kCycle` minus the other three, not a second independent `kCycle`-sized
 sleep, specifically so the schedule's total holds even under a
 zero-real-time-cost virtual clock (anchoring the final block to the cycle
@@ -279,7 +310,7 @@
 encode path. `emit()` sends at most one frame type per call and normally
 lets whichever frame is due win; when both are genuinely due in the same
 call it *alternates* rather than always favoring primary — at the real
-loop period (~20ms), primary is due on essentially every call, so an
+loop period (~40ms, 118), primary is due on essentially every call, so an
 unconditional "primary wins ties" rule starves secondary to 0Hz. The
 alternation costs at most one primary frame delayed by one cycle roughly
 once per secondary period; a non-tied call is unaffected.
```
