---
source_file: DESIGN.md
source_hash: 8516b40230615a80985efaf1c6c1efc818d7e1ae770a780e05141267eba333df
---
# Diff: DESIGN.md

Comparison of the sprint overlay copy of `DESIGN.md` against its pristine (seed-commit) canonical version.

```diff
--- DESIGN.md (pristine)
+++ DESIGN.md (current)
@@ -15,9 +15,11 @@
 
 * `Comms` (wire framing),
 * `Telemetry` (outbound frames),
-* `Drive` (twist → wheel targets),
-* `Odometry` (dead reckoning),
-* `Deadman` (the one staleness gate), and
+* `Drive` (velocity → wheel targets, twist or wheels variant),
+* `Odometry` (dead reckoning, plus cumulative path length),
+* `MoveQueue` (the 1-active + 4-pending bounded-motion queue — every
+  `Move` is self-bounding by construction, so there is no separate
+  staleness gate), and
 * `Preamble` (boot-time device detection).
 
 This is the seam that owns the robot's *timing* — every I2C
@@ -46,6 +48,17 @@
 CONFIG{motor,otos}+deadman only — S2 (sprint 116) replaces TWIST+deadman
 with the bounded MOVE protocol.
 
+**116-005/116-006 (S2, MOVE protocol cutover) — landed.** `App::Deadman`
+(`app/deadman.{h,cpp}`, both test harnesses) is deleted in turn — the
+same wholesale-deletion treatment 115-005 gave `Pilot`/`HeadingSource`
+above. `RobotLoop::handleTwist()` is replaced by `handleMove()`; a new,
+small `App::MoveQueue` (`app/move_queue.{h,cpp}`) owns the 1-active +
+4-pending queue and drives one `Motion::StopCondition`
+(`motion/stop_condition.{h,cpp}` — a fresh, much smaller `motion/`
+directory than the one S1 deleted, and NOT a revival of `Pilot`/
+`Motion::Executor`) per active `Move`. The command surface is now
+MOVE+STOP+CONFIG{motor,otos} — no deadman.
+
 ## 2. Orientation
 
 `RobotLoop` has two phases. `boot()` steps `Preamble` until every device
@@ -54,14 +67,15 @@
 boot. `cycle()` is the steady-state loop body: request/settle/collect/PID
 for the left motor, decode at most one inbound command (`Comms::pump`),
 apply it (`processMessage`), request/settle/collect/PID for the right motor,
-the deadman check, then a trailing block that samples OTOS, integrates
+the unconditional `moveQueue_.tick(now, odom_)` call, then a trailing block that samples OTOS, integrates
 odometry (`Odometry::integrate`), polls line/color at a rate-limited,
 alternating cadence (`updateLineColor()` — see below), and paces the whole
 cycle. `Telemetry::emit()` is called once per cycle and decides for itself
 whether to send the primary frame, the secondary diagnostic frame, or (on a
-tie) alternate between them. `Drive` and `Odometry` are pure, bounded,
-non-bus-touching helpers that `RobotLoop` calls at specific points in its
-own schedule; `Deadman` is polled once per cycle and gates `Drive::stop()`.
+tie) alternate between them. `Drive`, `Odometry`, and `MoveQueue` are pure,
+bounded, non-bus-touching helpers that `RobotLoop` calls at specific points
+in its own schedule; `MoveQueue::tick()` is called unconditionally once per
+cycle and drains to `Drive::stop()` once its queue empties.
 See `robot_loop.cpp` for the exact call order — it is the schedule's single
 source of truth.
 
@@ -106,10 +120,13 @@
   decodes at most one frame per call by construction (at most one
   `readLine()` per transport, first transport to have something wins), so
   `processMessage()` needs no separate "already handled" flag.
-- **Deadman is the only staleness gate:** one `App::Deadman`, armed by every
-  actuation command (currently only `Twist`, via `Twist.duration`), checked
-  once per cycle, expiry → `Drive::stop()`. Do not add a second ad hoc
-  watchdog anywhere in `app/`.
+- **No deadman — every `Move` is structurally self-bounding:**
+  `App::MoveQueue::tick()` runs unconditionally once per cycle and drains
+  to `Drive::stop()` once the active `Move`'s `Motion::StopCondition` or
+  `timeout` fires and nothing is pending — an emergent property of every
+  queued `Move` carrying its own bound, not a second, independently-timed
+  staleness timer. `App::Deadman` does not exist in this tree. Do not add
+  an ad hoc watchdog anywhere in `app/`.
 - **Telemetry always carries the last staged snapshot, not a diff:** a
   cycle that doesn't update a `Frame` field still sends whatever was last
   staged. Nothing here is "only send on change" — a dropped or unread frame
@@ -153,7 +170,7 @@
 (subsystems/fibers each owning a slice of the schedule) hides the bus
 schedule and the sleeps inside layers, which makes both hard-realtime
 problems — bus discipline and fiber-scheduler yielding — undebuggable.
-Modules (`Drive`, `Odometry`, `Telemetry`, `Comms`, `Deadman`, `Preamble`)
+Modules (`Drive`, `Odometry`, `Telemetry`, `Comms`, `MoveQueue`, `Preamble`)
 were factored *out* of that one function only as passive, bounded helpers;
 none of them run their own timing loop.
 
@@ -178,18 +195,25 @@
 
 **Command dispatch.** `processMessage` reads the `Cmd` populated (or not)
 by this cycle's single `Comms::pump()` call and switches on `cmd_kind`:
-`TWIST` stages a target on `Drive` and arms `Deadman`; `STOP` stops `Drive`
-(immediate, safety-critical) and disarms `Deadman`; `CONFIG` merges present
-wire fields into each motor's *own* current gains (never blanket-copies one
-motor's gains onto the other — their calibration can legitimately differ)
-and applies `travel_calib` to whichever motor `side` names, or (OTOS arm)
-applies scale/offset/init directly. Every path that applies a command acks
-via `Telemetry::ack(corrId, errCode)` (115-005: a single ack slot, not a
-ring — see "Telemetry's ack slot" below); `Comms`'s dearmor path itself
-never replies synchronously — a malformed frame is silently counted
-(`Comms::malformedCount()`) and surfaced as a telemetry flags bit instead
-of answered inline. This keeps replies flowing through one channel (the
-ack slot) rather than two.
+`MOVE` validates the envelope's shape (velocity variant present, stop
+variant present, `timeout > 0`) and the config-completeness gate, then
+delegates to `moveQueue_.enqueue()` (`replace=true` flushes pending and
+preempts the active `Move`; `replace=false` enqueues, or acks `ERR_FULL`
+past 4 pending); `STOP` stops `Drive` (immediate, safety-critical) and
+flushes `moveQueue_`; `CONFIG` merges present wire fields into each
+motor's *own* current gains (never blanket-copies one motor's gains onto
+the other — their calibration can legitimately differ) and applies
+`travel_calib` to whichever motor `side` names, or (OTOS arm) applies
+scale/offset/init directly. Every path that applies a command acks via
+`Telemetry::ack(corrId, errCode)` (115-005: a single ack slot, not a
+ring — see "Telemetry's ack slot" below); `moveQueue_` additionally emits
+a completion ack against `Move.id` (the same `Telemetry::ack()` call) when
+the active `Move` ends, whether by its stop condition or by `timeout` —
+the latter also sets `kFlagFaultMoveTimeout` (bit 15, see below).
+`Comms`'s dearmor path itself never replies synchronously — a malformed
+frame is silently counted (`Comms::malformedCount()`) and surfaced as a
+telemetry flags bit instead of answered inline. This keeps replies
+flowing through one channel (the ack slot) rather than two.
 
 **Telemetry's two send paths.** The primary frame (`msg::Telemetry`, ack
 slot + `flags` + pose/enc/vel/otos/line/color) rides a `ReplyEnvelope`
@@ -233,13 +257,15 @@
 `kFlagFaultWedgeLatch` (`motorL_.wedged() || motorR_.wedged()`), bit 8
 `kFlagFaultI2CNak` (declared, not yet wired — no per-transaction NAK
 aggregate exists yet), bit 9 `kFlagFaultCommsMalformed`
-(`Comms::malformedCount() > 0`), bit 10 `kFlagEventDeadmanExpired`
-(`Deadman::expired()`, the transition cycle only), bit 11
+(`Comms::malformedCount() > 0`), bit 10 `kFlagEventDeadmanExpired` (116:
+ORPHANED — its producer, `Deadman::expired()`, was deleted along with
+`App::Deadman`; nothing sets this bit any more, see §6), bit 11
 `kFlagEventBootReady` (`Preamble::done()`'s first-true transition), bit 12
 `kFlagEventConfigApplied` (declared, not yet wired), bits 13/14
 `kFlagLinePresent`/`kFlagColorPresent` (see §2's line/color polling note),
-bit 15 `kFlagFaultMoveTimeout` (declared now, wired by sprint 116's
-protocol-set-point issue — S1 has no MOVE command to time out). Declaring a
+bit 15 `kFlagFaultMoveTimeout` (116: wired — set on the cycle an active
+`Move` ends via `timeout` rather than its kind-specific stop condition).
+Declaring a
 bit before it is wired is deliberate — it reserves the bit number for a
 future caller without renumbering. `RobotLoop` assembles every bit EXCEPT
 `kFlagAckFresh` via `Telemetry::setFlag(bit, active)` at the point in the
@@ -282,21 +308,30 @@
   the one call that actually sends, at most one frame type, bounded work,
   never sleeps, never touches the I2C bus. See §4's "Telemetry's ack slot"
   and "The `flags` bit-string" notes above for the 115-005 shape.
-- **`Drive::setTwist(v_x, v_y, omega)`/`stop`/`tick()`:** setTwist only
-  stages a target — `v_y` is accepted and IGNORED (115-005: wire-forward
-  for sprint 116's MoveTwist, sprint.md Decision 5; every call site through
-  S1 passes 0). `tick()` computes wheel velocities via
-  `BodyKinematics::inverse()` and stages them onto the two motor leaves via
-  their own `setVelocity()` — it never calls a motor's own `tick()`, and
-  (115-005) has NO feedforward term any more: `configure()`/
-  `actuationLag_`/the `a_x`/`alpha` acceleration-feedforward staging
-  (112-002) were deleted along with `msg::PlannerConfig`, the type the gain
-  came from. `Drive` depends on nothing but `Devices::Motor` and
-  `BodyKinematics` now.
-- **`Odometry::integrate()`:** call once per cycle, after both motors' own
-  `tick()` has run that cycle; reads each leaf's current `position()` and
-  accumulates world pose via midpoint-arc integration over
-  `BodyKinematics::forward()`'s per-cycle body-frame delta.
+- **`Drive::setTwist(v_x, v_y, omega)`/`setWheels(v_left, v_right)`/`stop`/
+  `tick()`:** `setTwist` only stages a target — `v_y` is accepted and
+  IGNORED (wire-forwarded since 115 for a future holonomic base, now
+  carried by 116's `MoveTwist`; every call site through this sprint still
+  passes 0). `setWheels` (116) is a second, independent staging path for
+  `MoveWheels` — last-wins against whichever of `setTwist`/`setWheels` was
+  called most recently; `tick()` computes from whichever is live; `stop()`
+  clears both to zero regardless of which was staged (Decision 3:
+  `MoveWheels` is staged directly, never translated into an equivalent
+  twist via `BodyKinematics::forward()`). `tick()` computes wheel
+  velocities for the `setTwist` path via `BodyKinematics::inverse()` and
+  stages them onto the two motor leaves via their own `setVelocity()` — it
+  never calls a motor's own `tick()`, and (115-005) has NO feedforward term
+  any more: `configure()`/`actuationLag_`/the `a_x`/`alpha`
+  acceleration-feedforward staging (112-002) were deleted along with
+  `msg::PlannerConfig`, the type the gain came from. `Drive` depends on
+  nothing but `Devices::Motor` and `BodyKinematics` now.
+- **`Odometry::integrate()`/`pathLength()`:** `integrate()` — call once per
+  cycle, after both motors' own `tick()` has run that cycle; reads each
+  leaf's current `position()` and accumulates world pose via midpoint-arc
+  integration over `BodyKinematics::forward()`'s per-cycle body-frame
+  delta. `pathLength()` (116) is a read-only accessor over a running total
+  of `|distance|` that `integrate()` already computes internally each
+  cycle — the DISTANCE stop-condition's source of truth.
 - **`applyOtosSample(otos, now, frame)`:** safe to call every cycle — a
   too-soon call given OTOS's own internal rate limit is already a
   documented no-bus-traffic no-op. Carries the FULL `OtosReading` (x, y,
@@ -306,9 +341,16 @@
   loop's job, not this function's).
 - **`RobotLoop::updateLineColor(nowUs)`:** private, called once per cycle
   from the `kPace` block — see §2's own doc comment for the full contract.
-- **`Deadman::arm(duration)`/`disarm()`/`expired()`:** `arm()` always sets a
-  fresh deadline from now (re-arms, never stacks); negative/NaN duration
-  clamps to 0 (immediate expiry).
+- **`MoveQueue::enqueue(move)`/`tick(now, odom)`/`flush()`/`active()`:**
+  (116) `enqueue()` applies `replace`/enqueue semantics (`ERR_FULL` past 4
+  pending) and, for the newly-active slot, stages its velocity onto
+  `Drive` and captures its `Motion::StopCondition` baseline; `tick()`
+  advances the active `Move`'s `StopCondition`, hands off to the next
+  pending `Move` on stop/timeout (same cycle, no motion gap), and calls
+  `Drive::stop()` when the queue drains empty; `flush()` clears every
+  pending slot without disturbing the active one (used by `STOP`).
+  `active()` reports whether a `Move` is currently in progress (feeds
+  `frame_.mode`/`driving_`).
 - **`Preamble::step()`/`done()`/per-device status accessors:** `step()`
   never blocks; `done()` is true once every device has reached a terminal
   state (present-and-ready or confirmed-absent).
@@ -327,10 +369,13 @@
 - **`SerialPort`, `Radio` (ARM builds only):** the two real transports
   `SerialTransport`/`RadioTransport` adapt into `app::Transport` — see
   [com/DESIGN.md](../com/DESIGN.md).
-
-`Motion::Executor`/`Motion::Cmd`/`Motion::fromMove()` and
-[motion/DESIGN.md](../motion/DESIGN.md) are GONE (115-005) — `app/` depends
-on nothing under `motion/` any more.
+- **`Motion::StopCondition`** (116, `src/firm/motion/stop_condition.h`):
+  the bounded-motion stop/timeout comparison `App::MoveQueue` owns and
+  drives per active `Move` — see [motion/DESIGN.md](../motion/DESIGN.md).
+  This is NOT a revival of the deleted `Motion::Executor`/`Motion::Cmd`/
+  `Motion::fromMove()` (115-005, still gone) — the recreated `motion/`
+  directory contains only this one small, pure-comparison module,
+  mirroring `kinematics/`'s existing small-pure-computation pattern.
 
 ## 6. Open Questions / Known Limitations
 
@@ -346,9 +391,19 @@
   no binary command arms it from the wire today.
 - **`kFlagFaultI2CNak` (bit 8) and `kFlagEventConfigApplied` (bit 12) are
   declared but unwired** — reserved bit numbers with no live producer yet.
-  `kFlagFaultMoveTimeout` (bit 15) is declared for sprint 116's own MOVE
-  protocol-set-point work — S1 has no MOVE command, so this bit can never
-  be set by anything in this sprint.
+  `kFlagFaultMoveTimeout` (bit 15) is now wired (116) — set on the cycle
+  an active `Move` ends via `timeout` rather than its stop condition.
+- **`kFlagEventDeadmanExpired` (bit 10) is orphaned by 116, not
+  reassigned.** Its sole producer, `Deadman::expired()`, was deleted along
+  with `App::Deadman`; the bit constant still exists in `telemetry.h` (no
+  wire-shape change) but nothing in the tree calls
+  `Telemetry::setFlag(kFlagEventDeadmanExpired, ...)` any more, so it now
+  reads permanently 0. Left as declared-dead rather than deleted or
+  repurposed — this sprint's scope did not include a `flags` wire-shape
+  change, and reassigning a bit number to a new meaning without a version
+  signal would be a silent protocol break for any reader still checking
+  it. Whether to formally delete or repurpose this bit is open for a
+  future sprint.
 - **The pre-115 heading-PD/distance-trim/measurement-age-projection design
   history (formerly documented at length in this file's own §2, plus
   `motion/DESIGN.md`) is not carried forward here.** `Pilot`/
```
