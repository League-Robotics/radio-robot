---
source_file: DESIGN.md
source_hash: bd90451bb66d8754c1772eae18669539cb9d8f45625e4e2857b3dd0ce3acfd85
---
# Diff: DESIGN.md

Comparison of the sprint overlay copy of `DESIGN.md` against its pristine (seed-commit) canonical version.

```diff
--- DESIGN.md (pristine)
+++ DESIGN.md (current)
@@ -231,6 +231,29 @@
 planned consumer for future fake-OTOS/fusion bench work, per the same
 "greenfield, not yet wired to motion" posture the 117 note above already
 established for the estimator as a whole.
+
+**120 (bench tour bring-up: ack ring + build-selectable fake OTOS + I2C
+safety-net diagnosis, DRAFT — verify/refine against shipped code at
+execution time).** Three independent, phase-B bench-observability fixes.
+See "Telemetry's ack ring" (§4) for the ack-slot→ack-ring change and the
+`kFlagFaultI2CSafetyNet` paragraph (§4) for the bit-6 diagnosis. The
+third change: `Devices::Otos` gains a new synthetic-sample method (see
+[`devices/DESIGN.md`](../devices/DESIGN.md), edited directly by ticket 2,
+not overlaid here) that reports a pose+twist `RobotLoop` feeds it from
+that SAME cycle's `Odometry` output, instead of a real I2C burst read —
+selected by a compile-time build option (`FAKE_OTOS`), never a runtime
+toggle. This is the first FIRMWARE PRODUCTION CONSUMER of the "OTOS is
+present and reads a meaningful pose on a stand" property the previous
+paragraph's quarantine note anticipated — NOT yet a consumer of
+`StateEstimator::bodyAt()` itself (that stays quarantined; the fake feeds
+`Devices::Otos`/`frame.otos` directly, one layer below the estimator, and
+`StateEstimator`'s own OTOS-fusion weights stay 0.0, unchanged). The one
+new call site lives in `RobotLoop::cycle()` (§2), not in `Devices::Otos`'s
+own construction (`main.cpp`) — see this sprint's own Architecture Design
+Rationale (Decision 3) for why the branch sits at the per-cycle call
+site rather than at composition time; `main.cpp`'s `Devices::Otos
+otos(bus, otosConfig)` construction line is unchanged between the real
+and bench builds.
 
 ## 2. Orientation
 
@@ -452,16 +475,30 @@
 alternation costs at most one primary frame delayed by one cycle roughly
 once per secondary period; a non-tied call is unaffected.
 
-**Telemetry's ack slot (115-005 — replaces the old depth-3 AckEntry
-ring).** `Telemetry::ack(corrId, errCode)` overwrites a single
-`ackCorr_`/`ackErr_` pair (`errCode == 0` means OK); a command acked within
-the same primary period as another overwrites it (stakeholder-accepted
-tradeoff — rare at bench rates, `wait_for_ack` timeout+retry covers it).
-`flags` bit 5 (`kFlagAckFresh`) is a ONE-SHOT pulse Telemetry tracks
-internally, not a caller-set bit: true on the very next `emitPrimary()`
-call after an `ack()` call, then cleared — `ack_corr`/`ack_err`'s VALUES
-persist across frames (so a reader who missed the fresh pulse can still see
-what the last ack was), only the freshness bit clears.
+**Telemetry's ack ring (120, DRAFT — verify/refine against shipped
+`telemetry.{h,cpp}` at execution time, same convention 118/119 used for
+this overlay) — replaces the 115-005 single-slot design, which itself
+had replaced the original depth-3 `AckEntry` ring.** Bench measurement
+at the real 40ms cycle / ~15Hz host read rate
+(`bench-single-ack-slot-observability-collapses-at-40ms.md`) showed the
+115-005 single-slot design's own "rare at bench rates" assumption no
+longer holds: `move_protocol_bench.py` lost 12/43 checks, every miss a
+transient enqueue/STOP/CONFIG ack overwritten before the host's next
+read. `Telemetry::ack(corrId, errCode)` now pushes onto a small, bounded
+ring (depth 4) instead of overwriting a single pair; `emit()` serializes
+the ring's current contents into a new, additive wire field. `ack_corr`/
+`ack_err` (the pre-120 scalar pair) and `flags` bit 5 (`kFlagAckFresh`)
+keep their EXACT prior meaning — "the freshest ack" — for any reader
+that never looked past them; the ring is purely additive, so no existing
+host consumer needs to change to keep working. A command acked within
+the same primary period as 4 OTHER commands still overwrites the ring's
+oldest entry (a saturated-ring tradeoff, not the old single-slot
+tradeoff) — the rapid-fire acceptance test this ticket adds is designed
+to confirm 4 is enough for the queue's own 5-deep `ERR_FULL` ceiling in
+practice; revisit the depth constant if it isn't. `flags` bit 5 remains a
+ONE-SHOT pulse Telemetry tracks internally: true on the very next
+`emitPrimary()` call after ANY `ack()` push since the last emit, then
+cleared.
 
 **The `flags` bit-string (115-005 — replaces the old separate
 `fault_bits`/`event_bits`/nine-bool frame).** ONE `uint32` carries every
@@ -475,7 +512,23 @@
 hardware this has been observed as a one-shot latch coincident with
 `Preamble::done()`'s transition, not a live/continuous indicator; a steady
 1 after boot with no in-flight anomaly is not itself evidence of a defect,
-only a bit that flips *during* driving is actionable), bit 7
+only a bit that flips *during* driving is actionable. **120, DRAFT —
+diagnosis in progress, verify/refine against ticket 3's actual on-chip
+trace at execution time:** bench evidence (120's own source issue,
+`bench-i2c-safety-net-fault-asserts-every-cycle.md`) shows this bit set
+100% of frames, idle AND driving, contradicting 118-001's own prediction
+that the loop-schedule restore would clear it while driving. The leading
+candidate is that `>0` against a monotonically non-decreasing counter
+latches permanently after a single early (boot/`Preamble`) trip —
+`MicroBitI2CBus::resetStats()` exists and zeroes the counter, but is
+never called anywhere in production firmware. Ticket 3 traces the raw
+counter (not just this derived bit) idle vs. driving to confirm whether
+that theory holds, or whether a real ongoing bus-timing defect remains;
+this paragraph is updated to state the confirmed conclusion once ticket
+3 lands — if a fix ships, this note is replaced by a plain description
+of when the bit sets; if the count is confirmed a boot-time latch, this
+note is replaced by a statement to that effect and 118-001's own
+acceptance claim is corrected in its record.), bit 7
 `kFlagFaultWedgeLatch` (`motorL_.wedged() || motorR_.wedged()`), bit 8
 `kFlagFaultI2CNak` (declared, not yet wired — no per-transaction NAK
 aggregate exists yet), bit 9 `kFlagFaultCommsMalformed`
```
