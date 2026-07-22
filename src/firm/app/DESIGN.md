---
root: ../../../docs/design/design.md
---

# App — Loop and Passive App Modules

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** in-flux

---

## 1. Purpose

`app/` is the single cooperatively-timed control loop (`App::RobotLoop`) and
the passive modules it drives:

* `Comms` (wire framing),
* `Telemetry` (outbound frames),
* `Drive` (velocity → wheel targets, twist or wheels variant),
* `Odometry` (dead reckoning, plus cumulative path length),
* `MoveQueue` (the 1-active + 4-pending bounded-motion queue — every
  `Move` is self-bounding by construction, so there is no separate
  staleness gate),
* `StateEstimator` (117 — predict-to-now wheel/body peer estimates,
  zero-order-hold extrapolation, v1 complementary blend against OTOS),
  and
* `Preamble` (boot-time device detection).

This is the seam that owns the robot's *timing* — every I2C
transaction, every wait, every cadence decision lives here or is called from
here in visible order. It exists as its own subsystem because timing
discipline on a shared single-master I2C bus, and cooperative yielding to the
CODAL fiber scheduler, are the two hard realtime problems this firmware has;
drawing the boundary around "the thing that owns the schedule" keeps that
problem in one place instead of smeared across device leaves.

**115-005 (gut S1) deletion note.** `Pilot` (109-003/109-005 — bridged
`Motion::Executor` into the loop's cycle and computed the heading PD
cascade on top of it) and `HeadingSource` (109-005 — decided which sensor
was truth for heading) are DELETED wholesale, along with `Motion::Executor`/
`Motion::JerkTrajectory`/`vendor/ruckig` (`motion/DESIGN.md` and its own
Pilot/Executor/HeadingSource sections go with them — see that directory's
own history if it still exists, or the git tag below if it does not). This
sprint's own tag `pre-gut-motion-stack` preserves the full pre-deletion tree
(files, DESIGN.md prose, tests) for recovery — deleting the DOCUMENTATION of
a deleted subsystem here is not a loss of information, only a redirect to
where the real historical record lives. `RobotLoop` lost the `Pilot&`
constructor parameter, the MOVE dispatch case, and every `pilot_.*` call
site; `Drive` lost its `configure(msg::PlannerConfig)`/acceleration-
feedforward staging (112-002) entirely, since `msg::PlannerConfig` itself no
longer exists. The command surface through this sprint (S1) is TWIST+STOP+
CONFIG{motor,otos}+deadman only — S2 (sprint 116) replaces TWIST+deadman
with the bounded MOVE protocol.

**116-005/116-006 (S2, MOVE protocol cutover) — landed.** `App::Deadman`
(`app/deadman.{h,cpp}`, both test harnesses) is deleted in turn — the
same wholesale-deletion treatment 115-005 gave `Pilot`/`HeadingSource`
above. `RobotLoop::handleTwist()` is replaced by `handleMove()`; a new,
small `App::MoveQueue` (`app/move_queue.{h,cpp}`) owns the 1-active +
4-pending queue and drives one `Motion::StopCondition`
(`motion/stop_condition.{h,cpp}` — a fresh, much smaller `motion/`
directory than the one S1 deleted, and NOT a revival of `Pilot`/
`Motion::Executor`) per active `Move`. The command surface is now
MOVE+STOP+CONFIG{motor,otos} — no deadman.

**117 (predict-to-now estimator v1) — landed.** A new passive module,
`App::StateEstimator` (`app/state_estimator.{h,cpp}`), is added
alongside `Odometry` — NOT a replacement for it: `Odometry`'s dead-
reckoned `x_`/`y_`/`theta_` still feed `frame_.pose` exactly as before,
and `StateEstimator` reads that same per-cycle `Frame` data as an
independent, additive consumer, greenfield in the same sense
`StateEstimator`'s own source issue used for the deleted `Pilot`/
`HeadingSource` era: it does not yet drive motion (no consumer wires its
output into `Drive`/`MoveQueue` this sprint — that is the later
trajectory-controller sprint, gated on this one being bench-proven).
`StateEstimator` holds per-wheel and body state as PEER estimates (each
independently valid/stale), computes zero-order-hold "predict to now"
extrapolation from the newest basis reading — generalizing the deleted
`HeadingSource::headingLead()` equation (`heading = basis.heading +
basis.omega × age`) to the full body pose (x, y, heading, v_x, v_y,
omega) — and blends a v1 complementary weight against OTOS heading/omega
whose weights are fail-closed baked config (`Config::
defaultEstimatorConfig()`), defaulting to 0.0 (encoder-only output this
sprint, per stakeholder decision) and live-tunable via a new
`ConfigDelta.estimator` (`EstimatorConfigPatch`) arm dispatched by
`RobotLoop::handleConfig()`, mirroring `OtosConfigPatch`'s existing
merge-then-apply pattern — see §3/§4 below for the full detail. Pure
computation: never touches the I2C bus, never sleeps, no `Devices::
Clock&` collaborator of its own (every query takes an explicit `now`/`t`
argument, mirroring `Motion::StopCondition`'s "hand-fed readings, no
owned collaborator" shape — see that module's own file-header precedent).

## 2. Orientation

`RobotLoop` has two phases. `boot()` steps `Preamble` until every device
leaf reaches a terminal state (present-and-ready or confirmed-absent),
emitting a boot telemetry frame each pass; commands are not consumed during
boot. `cycle()` is the steady-state loop body: request/settle/collect/PID
for the left motor, decode at most one inbound command (`Comms::pump`),
apply it (`processMessage`), request/settle/collect/PID for the right motor,
the unconditional `moveQueue_.tick(now, odom_)` call, then a trailing block that samples OTOS, integrates
odometry (`Odometry::integrate`), refreshes `App::StateEstimator`'s
predict-to-now estimates from that same cycle's staged `Frame` (117 —
see below), polls line/color at a rate-limited, alternating cadence
(`updateLineColor()` — see below), and paces the whole cycle. `Telemetry::emit()` is called once per cycle and decides for itself
whether to send the primary frame, the secondary diagnostic frame, or (on a
tie) alternate between them. `Drive`, `Odometry`, and `MoveQueue` are pure,
bounded, non-bus-touching helpers that `RobotLoop` calls at specific points
in its own schedule; `MoveQueue::tick()` is called unconditionally once per
cycle and drains to `Drive::stop()` once its queue empties.
See `robot_loop.cpp` for the exact call order — it is the schedule's single
source of truth.

**Line/color polling (`RobotLoop::updateLineColor()`, 115-005).** Runs once
per cycle from the trailing `kPace` block. Ticks EXACTLY ONE of
`Devices::LineSensorLeaf`/`Devices::ColorSensorLeaf` per call — never both —
alternating which one on the NEXT call, so at most one of the two is even
OFFERED a chance to check its own `readDue()` in any given cycle (the
098-004 per-pass-read regression precedent: a per-cycle sensor read must
never disrupt the motor request/collect cadence). Each leaf's own
`tick()`/`readDue()` rate-limits the actual bus transaction further (the
same `Otos::readDue()` pattern `Devices::Otos` already uses). A fresh
reading packs into `frame_.line`/`frame_.color` (4 raw grayscale bytes,
ch1 low byte; RGBC scaled 16→8 bits, R low byte) and sets the corresponding
`flags` bit (13/14) for THIS cycle only — the OTHER leaf's own bit is
explicitly cleared the same cycle (it was not even touched), matching the
wire spec's "line/color word fresh" (fresh THIS frame, not merely "known at
some point") semantics.

**Predict-to-now estimation (`RobotLoop`'s `StateEstimator::update()`
call, 117).** Runs once per cycle from the trailing `kPace` block,
immediately after `frame_.pose` is staged (i.e. after
`applyOtosSample()` and `odom_.integrate()` — the same position this
sprint's source issue specified as "after applyOtosSample()/
odom_.integrate(), before pilot_.plan()"; `Pilot` no longer exists, so
this is simply the end of that block). `update(frame, now)` reads
`frame.encLeft`/`frame.encRight` (position, velocity, their own collect
`time`) to refresh each wheel's peer `WheelEstimate` basis, and
`frame.pose`/`frame.twist` (already fused by `Odometry`/
`BodyKinematics::forward()` earlier the same cycle) plus `frame.otos`/
`frame.otosPresent` (when fresh) to refresh the body peer's
`BodyEstimate` basis via the v1 complementary blend. Pure computation
over already-staged data — no I2C access, no sleep, bounded work, same
posture `Odometry::integrate()` and `applyOtosSample()` already keep in
this same block.

## 3. Constraints and Invariants

- **Single-loop bus ownership:** every I2C transaction happens from
  `RobotLoop::cycle()`'s own call sequence. No app module ever initiates bus
  traffic from its own `tick()`/staging methods on its own timing — see the
  system doc's "single-loop bus ownership" invariant (`docs/design/design.md`
  §5). `Odometry::integrate()`,
  `applyOtosSample()`, and `updateLineColor()` are called only from the
  loop's trailing block, never from inside a motor request→collect window.
- **The timing schedule is exactly `robot_loop.cpp`'s `runAndWait` calls:**
  `grep 'runAndWait\|sleepUntil' app/robot_loop.cpp` must remain the
  firmware's complete list of waits. A sleep hidden inside any other
  function (a module's `tick()`, a handler, a helper) silently breaks the
  cycle's timing budget and starves the CODAL fiber scheduler — the radio
  looks dead when the loop fails to yield, even though nothing is wrong
  with the RF.
- **`runAndWait` bodies other than the final block never touch the bus and
  never sleep:** they exist only to spend an already-mandatory clearance
  window on other bounded work (comms pump, telemetry assembly, command
  dispatch). Moving bus traffic into one of these bodies reintroduces the
  shared-bus timing collisions a single-master I2C bus cannot tolerate.
- **Command dispatch is bounded to at most one per cycle:** `Comms::pump()`
  decodes at most one frame per call by construction (at most one
  `readLine()` per transport, first transport to have something wins), so
  `processMessage()` needs no separate "already handled" flag.
- **No deadman — every `Move` is structurally self-bounding:**
  `App::MoveQueue::tick()` runs unconditionally once per cycle and drains
  to `Drive::stop()` once the active `Move`'s `Motion::StopCondition` or
  `timeout` fires and nothing is pending — an emergent property of every
  queued `Move` carrying its own bound, not a second, independently-timed
  staleness timer. `App::Deadman` does not exist in this tree. Do not add
  an ad hoc watchdog anywhere in `app/`.
- **Telemetry always carries the last staged snapshot, not a diff:** a
  cycle that doesn't update a `Frame` field still sends whatever was last
  staged. Nothing here is "only send on change" — a dropped or unread frame
  never loses data because the next one repeats it.
- **`Frame` fields written late in a cycle are read by the NEXT cycle's
  emit, not lost:** pose/OTOS/line/color are staged at the end of the cycle
  they are computed in, and picked up by the following cycle's
  `updateTlm()` + `emit()`. This is deliberate — treat a one-cycle
  staleness on those fields as normal, not a bug.
- **Devices isolation still applies inside `app/`:** wire-plane `msg::*`
  types are converted to/from `Devices::*` types only in `main.cpp`
  (outside this directory); no app module should reach around that.
- **Line/color are now sampled in steady state (115-005), OTOS is not the
  only one any more.** The previous version of this note said the
  opposite — `Preamble` detects presence at boot; `updateLineColor()` (§2
  above) now also samples both in steady state, rate-limited and
  alternating. There is still no full 3-way round-robin abstraction
  (otos|line|color) — each sensor is its own bounded step, not a unified
  scheduler class.
- **Config patches cover `MotorConfigPatch`, `OtosConfigPatch`
  (109-004), and `EstimatorConfigPatch` (117) only.** `RobotLoop::
  handleConfig` replies `ERR_UNIMPLEMENTED` for `DRIVETRAIN`/`WATCHDOG`/
  `NONE` (`DrivetrainConfigPatch` has no on-robot fusion consumer).
  `PlannerConfigPatch` is GONE, not merely out of scope — 115-005 (gut
  S1) deleted the type and `ConfigDelta`'s own `PLANNER` oneof arm
  entirely, along with `Pilot`/`Motion::Executor`, the only things that
  ever consumed it. `OtosConfigPatch` (issue
  `otos-calibration-config-message.md`) restores a RUNTIME path to
  `Devices::Otos::setLinearScalar()`/`setAngularScalar()`/`setOffset()`/
  `init()` — previously only ever called once at boot from baked
  `boot_config` — applied immediately and synchronously inside
  `handleConfig()` (still "the loop's own cycle" per the single-loop bus
  ownership invariant above: a rare, command-triggered I2C/config
  transaction sandwiched into the existing schedule, not a new per-cycle
  bus consumer). `EstimatorConfigPatch` (117) merges present
  `weight_heading_otos`/`weight_omega_otos`/`staleness_ms` fields onto
  `StateEstimator`'s own live weight state — a pure in-memory update, NOT
  an I2C transaction (unlike the OTOS branch above), and NOT persisted
  into `persistedTuning_`/flash (Design Rationale Decision 4, overlay
  `design.md`'s sibling — a reboot reverts to the baked JSON default).

## 4. Design

**Why one loop.** `RobotLoop::cycle()` is deliberately one function with
every bus transaction and every wait visible in call order, rather than a
dispatch graph of modules each with their own timing. The alternative
(subsystems/fibers each owning a slice of the schedule) hides the bus
schedule and the sleeps inside layers, which makes both hard-realtime
problems — bus discipline and fiber-scheduler yielding — undebuggable.
Modules (`Drive`, `Odometry`, `Telemetry`, `Comms`, `MoveQueue`, `Preamble`)
were factored *out* of that one function only as passive, bounded helpers;
none of them run their own timing loop.

**The timing primitive.** `runAndWait(gap, body)` marks time, runs `body`,
then sleeps until at least `gap` has elapsed since its own mark. Each block
anchors to its *own* mark rather than a shared cycle-start mark, so a slow
body degrades gracefully — its sleep shrinks toward (never below) 1ms
instead of stacking on top of an unrelated deadline. The schedule has four
such blocks: left-motor settle, post-duty clearance, right-motor settle,
and a final perception+odometry+pace block. The four gaps
(`kSettle`, `kClear`, `kSettle`, `kPace`) are sized so their sum equals the
whole-cycle target `kCycle` (20ms / ~50Hz) — `kPace` is *derived* as
`kCycle` minus the other three, not a second independent `kCycle`-sized
sleep, specifically so the schedule's total holds even under a
zero-real-time-cost virtual clock (anchoring the final block to the cycle
start instead of its own mark was a diagnosed defect: it double-counted the
first three blocks' time against the target). `kCycle` matches
`Telemetry::kPrimaryPeriod` by construction (115-005: primary period now
EQUALS the cycle period — every loop iteration emits a frame, closing
`kcycle-kprimaryperiod-mismatch.md`) so the primary-frame throttle and the
loop's own pace agree.

**Command dispatch.** `processMessage` reads the `Cmd` populated (or not)
by this cycle's single `Comms::pump()` call and switches on `cmd_kind`:
`MOVE` validates the envelope's shape (velocity variant present, stop
variant present, `timeout > 0`) and the config-completeness gate, then
delegates to `moveQueue_.enqueue()` (`replace=true` flushes pending and
preempts the active `Move`; `replace=false` enqueues, or acks `ERR_FULL`
past 4 pending); `STOP` stops `Drive` (immediate, safety-critical) and
flushes `moveQueue_`; `CONFIG` merges present wire fields into each
motor's *own* current gains (never blanket-copies one motor's gains onto
the other — their calibration can legitimately differ) and applies
`travel_calib` to whichever motor `side` names, or (OTOS arm) applies
scale/offset/init directly. Every path that applies a command acks via
`Telemetry::ack(corrId, errCode)` (115-005: a single ack slot, not a
ring — see "Telemetry's ack slot" below); `moveQueue_` additionally emits
a completion ack against `Move.id` (the same `Telemetry::ack()` call) when
the active `Move` ends, whether by its stop condition or by `timeout` —
the latter also sets `kFlagFaultMoveTimeout` (bit 15, see below).
`Comms`'s dearmor path itself never replies synchronously — a malformed
frame is silently counted (`Comms::malformedCount()`) and surfaced as a
telemetry flags bit instead of answered inline. This keeps replies
flowing through one channel (the ack slot) rather than two.

**Telemetry's two send paths.** The primary frame (`msg::Telemetry`, ack
slot + `flags` + pose/enc/vel/otos/line/color) rides a `ReplyEnvelope`
through `Comms::sendReply()`. The secondary diagnostic frame
(`msg::TelemetrySecondary`) is not a `ReplyEnvelope` oneof arm, so
`Telemetry` holds its own `Transport&` pair and performs its own
armor+broadcast for that one frame type, reusing `Comms`'s armor buffer
size and `WireRuntime::base64Encode()` rather than duplicating a private
encode path. `emit()` sends at most one frame type per call and normally
lets whichever frame is due win; when both are genuinely due in the same
call it *alternates* rather than always favoring primary — at the real
loop period (~20ms), primary is due on essentially every call, so an
unconditional "primary wins ties" rule starves secondary to 0Hz. The
alternation costs at most one primary frame delayed by one cycle roughly
once per secondary period; a non-tied call is unaffected.

**Telemetry's ack slot (115-005 — replaces the old depth-3 AckEntry
ring).** `Telemetry::ack(corrId, errCode)` overwrites a single
`ackCorr_`/`ackErr_` pair (`errCode == 0` means OK); a command acked within
the same primary period as another overwrites it (stakeholder-accepted
tradeoff — rare at bench rates, `wait_for_ack` timeout+retry covers it).
`flags` bit 5 (`kFlagAckFresh`) is a ONE-SHOT pulse Telemetry tracks
internally, not a caller-set bit: true on the very next `emitPrimary()`
call after an `ack()` call, then cleared — `ack_corr`/`ack_err`'s VALUES
persist across frames (so a reader who missed the fresh pulse can still see
what the last ack was), only the freshness bit clears.

**The `flags` bit-string (115-005 — replaces the old separate
`fault_bits`/`event_bits`/nine-bool frame).** ONE `uint32` carries every
status/fault/event/presence bit: bit 0 `kFlagOtosPresent` (OtosReading
fresh THIS frame — chip detected AND this cycle's burst actually
refreshed the cached pose, NOT the old pre-115 "chip ever detected"
semantic), bit 1 `kFlagOtosConnected` (live bus health), bit 2 `kFlagActive`
(motion in progress), bits 3/4 `kFlagConnLeft`/`kFlagConnRight` (motor bus
connectivity), bit 5 `kFlagAckFresh` (Telemetry-internal, see above), bit 6
`kFlagFaultI2CSafetyNet` (`I2CBus::clearanceSafetyNetCount() > 0` — on real
hardware this has been observed as a one-shot latch coincident with
`Preamble::done()`'s transition, not a live/continuous indicator; a steady
1 after boot with no in-flight anomaly is not itself evidence of a defect,
only a bit that flips *during* driving is actionable), bit 7
`kFlagFaultWedgeLatch` (`motorL_.wedged() || motorR_.wedged()`), bit 8
`kFlagFaultI2CNak` (declared, not yet wired — no per-transaction NAK
aggregate exists yet), bit 9 `kFlagFaultCommsMalformed`
(`Comms::malformedCount() > 0`), bit 10 `kFlagEventDeadmanExpired` (116:
ORPHANED — its producer, `Deadman::expired()`, was deleted along with
`App::Deadman`; nothing sets this bit any more, see §6), bit 11
`kFlagEventBootReady` (`Preamble::done()`'s first-true transition), bit 12
`kFlagEventConfigApplied` (declared, not yet wired), bits 13/14
`kFlagLinePresent`/`kFlagColorPresent` (see §2's line/color polling note),
bit 15 `kFlagFaultMoveTimeout` (116: wired — set on the cycle an active
`Move` ends via `timeout` rather than its kind-specific stop condition).
Declaring a
bit before it is wired is deliberate — it reserves the bit number for a
future caller without renumbering. `RobotLoop` assembles every bit EXCEPT
`kFlagAckFresh` via `Telemetry::setFlag(bit, active)` at the point in the
cycle each condition becomes known (mirrors the old `setFault()`/
`setEvent()` call-site pattern, now unified onto one bit space); Telemetry
itself ORs in `kFlagAckFresh` at `emitPrimary()` time.

**Boot contract.** `Preamble::step()` advances at most one not-yet-resolved
device's own detection entry point per call, never sleeps, and never
touches the bus more than that one call's leaf. `RobotLoop::boot()` owns
the pacing sleep between `step()` calls and emits a boot telemetry frame
each pass, so a host watching the wire can distinguish "still booting" from
"dead" well before the command loop starts. A power-settle wait
(`kPowerSettle`, ported unchanged from the retired `DeviceBus`) blocks the
very first probe from racing the rails on power-up — it exists because the
very first device probed (a motor's `begin()`) has no retry pacing of its
own to lean on. A wall-clock defensive bound (`kMaxPreamble`) forces every
remaining slot terminal if a leaf's own detection never resolves; this is a
safety net against a future leaf regression, not the primary termination
path (every slot already self-bounds its own retry count given step() is
called with real elapsed time between calls).

## 5. Interfaces

### Exposes

- **`RobotLoop::run()` / `boot()` / `cycle()`:** `run()` never returns —
  `boot()` once, then `cycle()` forever. `boot()`/`cycle()` are exposed
  separately so a host harness can step a bounded number of cycles and
  inspect state between them; `cycle()` assumes every device already
  resolved from a prior `boot()` (no readiness checks inside it).
- **`Comms::pump(Cmd&)`:** non-blocking, decodes at most one frame per call
  across both transports; resets `out.status` to `kNone` at entry so a
  caller never sees stale decode state.
- **`Comms::sendReply(const msg::ReplyEnvelope&)`:** encodes, armors, and
  broadcasts on both transports via the async/drop-on-full send path —
  never blocks the loop on backpressure.
- **`Telemetry::setFrame`/`setFlag`/`ack`/`emit(now)`:** staging calls are
  cheap and can be called any number of times per cycle; `emit(now)` is
  the one call that actually sends, at most one frame type, bounded work,
  never sleeps, never touches the I2C bus. See §4's "Telemetry's ack slot"
  and "The `flags` bit-string" notes above for the 115-005 shape.
- **`Drive::setTwist(v_x, v_y, omega)`/`setWheels(v_left, v_right)`/`stop`/
  `tick()`:** `setTwist` only stages a target — `v_y` is accepted and
  IGNORED (wire-forwarded since 115 for a future holonomic base, now
  carried by 116's `MoveTwist`; every call site through this sprint still
  passes 0). `setWheels` (116) is a second, independent staging path for
  `MoveWheels` — last-wins against whichever of `setTwist`/`setWheels` was
  called most recently; `tick()` computes from whichever is live; `stop()`
  clears both to zero regardless of which was staged (Decision 3:
  `MoveWheels` is staged directly, never translated into an equivalent
  twist via `BodyKinematics::forward()`). `tick()` computes wheel
  velocities for the `setTwist` path via `BodyKinematics::inverse()` and
  stages them onto the two motor leaves via their own `setVelocity()` — it
  never calls a motor's own `tick()`, and (115-005) has NO feedforward term
  any more: `configure()`/`actuationLag_`/the `a_x`/`alpha`
  acceleration-feedforward staging (112-002) were deleted along with
  `msg::PlannerConfig`, the type the gain came from. `Drive` depends on
  nothing but `Devices::Motor` and `BodyKinematics` now.
- **`Odometry::integrate()`/`pathLength()`:** `integrate()` — call once per
  cycle, after both motors' own `tick()` has run that cycle; reads each
  leaf's current `position()` and accumulates world pose via midpoint-arc
  integration over `BodyKinematics::forward()`'s per-cycle body-frame
  delta. `pathLength()` (116) is a read-only accessor over a running total
  of `|distance|` that `integrate()` already computes internally each
  cycle — the DISTANCE stop-condition's source of truth.
- **`applyOtosSample(otos, now, frame)`:** safe to call every cycle — a
  too-soon call given OTOS's own internal rate limit is already a
  documented no-bus-traffic no-op. Carries the FULL `OtosReading` (x, y,
  heading, v_x, v_y, omega, burst-read time) into `frame.otos` (115-005 —
  previously a bare `Pose2D`, velocities silently dropped). Must not be
  called from inside a motor request→collect window (bus-discipline is the
  loop's job, not this function's).
- **`RobotLoop::updateLineColor(nowUs)`:** private, called once per cycle
  from the `kPace` block — see §2's own doc comment for the full contract.
- **`MoveQueue::enqueue(move)`/`tick(now, odom)`/`flush()`/`active()`:**
  (116) `enqueue()` applies `replace`/enqueue semantics (`ERR_FULL` past 4
  pending) and, for the newly-active slot, stages its velocity onto
  `Drive` and captures its `Motion::StopCondition` baseline; `tick()`
  advances the active `Move`'s `StopCondition`, hands off to the next
  pending `Move` on stop/timeout (same cycle, no motion gap), and calls
  `Drive::stop()` when the queue drains empty; `flush()` clears every
  pending slot without disturbing the active one (used by `STOP`).
  `active()` reports whether a `Move` is currently in progress (feeds
  `frame_.mode`/`driving_`).
- **`Preamble::step()`/`done()`/per-device status accessors:** `step()`
  never blocks; `done()` is true once every device has reached a terminal
  state (present-and-ready or confirmed-absent).
- **`StateEstimator::update(frame, now)`/`wheelAt(wheel, t)`/`bodyAt(t)`/
  `whereAmI(now)`/`wheelNow(wheel)`/`reset(x, y, heading)`/
  `innovations()`/`setWeights(weights)`** (117): `update()` — call once
  per cycle from the trailing `kPace` block, after `frame_.pose` is
  staged; refreshes both wheel peers' and the body peer's basis. `wheelAt`/
  `bodyAt` — pure ZOH extrapolation from the current basis to an
  explicit query time `t`; no owned clock, hand-fed `t` always, mirroring
  `Motion::StopCondition`'s own testability shape. `whereAmI(now)` is
  exactly `bodyAt(now)`; `wheelNow(wheel)` returns the wheel's raw basis
  with no extrapolation. `reset(x, y, heading)` re-anchors the body
  peer's world pose only (wheel peers are untouched — they track
  per-wheel distance, not world pose, the same reasoning `Odometry::
  pathLength()` is untouched by `Odometry::reset()`). `innovations()`
  returns the most recent OTOS-vs-predicted heading/omega residual —
  computed for diagnostic/validation purposes even while its fusion
  weight is 0, never fed back into the estimate itself at that weight.
  `setWeights()` is `RobotLoop::handleConfig()`'s own entry point for a
  live `EstimatorConfigPatch` (§3 above) — a plain in-memory update, not
  a bus transaction. All of the above are pure computation: no I2C
  access, no sleep, bounded per call.

### Consumes

- **`Devices::NezhaMotor`, `Devices::Otos`, `Devices::ColorSensorLeaf`,
  `Devices::LineSensorLeaf`, `Devices::I2CBus`, `Devices::Clock`,
  `Devices::Sleeper`:** the device leaves and time/bus seams `app/` drives
  — see [devices/DESIGN.md](../devices/DESIGN.md).
- **`BodyKinematics::inverse()`/`forward()`:** stateless twist↔wheel math —
  see [kinematics/DESIGN.md](../kinematics/DESIGN.md).
- **`msg::CommandEnvelope`/`ReplyEnvelope`/`Telemetry`/`TelemetrySecondary`,
  `msg::wire::encode`/`decode`, `WireRuntime::base64Encode`/`Decode`:** the
  wire schema and codec — see [messages/DESIGN.md](../messages/DESIGN.md).
- **`SerialPort`, `Radio` (ARM builds only):** the two real transports
  `SerialTransport`/`RadioTransport` adapt into `app::Transport` — see
  [com/DESIGN.md](../com/DESIGN.md).
- **`Motion::StopCondition`** (116, `src/firm/motion/stop_condition.h`):
  the bounded-motion stop/timeout comparison `App::MoveQueue` owns and
  drives per active `Move` — see [motion/DESIGN.md](../motion/DESIGN.md).
  This is NOT a revival of the deleted `Motion::Executor`/`Motion::Cmd`/
  `Motion::fromMove()` (115-005, still gone) — the recreated `motion/`
  directory contains only this one small, pure-comparison module,
  mirroring `kinematics/`'s existing small-pure-computation pattern.
- **`Telemetry::Frame`** (117): `StateEstimator::update()` reads the SAME
  per-cycle `Frame` struct `Telemetry::setFrame()` stages — it does not
  hold its own leaf/bus references and does not read `Devices::Motor`/
  `Devices::Otos` directly. Wire-plane `msg::EstimatorConfigPatch` stops
  at `RobotLoop::handleConfig()` exactly like `msg::MotorConfigPatch`/
  `msg::OtosConfigPatch` already do (devices/app isolation invariant
  above, extended by analogy) — `StateEstimator`'s own `setWeights()`
  takes a plain, Devices-local-style weights struct, never a `msg::*`
  type.
- **`Config::defaultEstimatorConfig()`** (117, `config/boot_config.h`):
  fail-closed baked fusion-weight defaults (`weight_heading_otos =
  weight_omega_otos = 0.0` this sprint, `staleness_ms`), constructed once
  at boot in `main.cpp` and passed to `StateEstimator`'s constructor —
  see [config/DESIGN.md](../config/DESIGN.md).

## 6. Open Questions / Known Limitations

- **`MotorConfigPatch` and `OtosConfigPatch` (109-004) are live-appliable.**
  Only `DrivetrainConfigPatch`/`WatchdogConfigPatch` still reply
  `ERR_UNIMPLEMENTED` (no on-robot fusion consumer for the former; the
  latter routes to `bb.streamWatchdogWindowIn` directly, not
  `handleConfig`, per config.proto's own `CONFIG_WATCHDOG` comment); see
  §3. `PlannerConfigPatch` is not a third "still unimplemented" case — it
  no longer exists as a type at all (115-005).
- **In-session pose reset has no wire verb yet.** `Odometry::reset()`
  exists and is exercised by the host simulator's teleport-to-origin, but
  no binary command arms it from the wire today.
- **`kFlagFaultI2CNak` (bit 8) and `kFlagEventConfigApplied` (bit 12) are
  declared but unwired** — reserved bit numbers with no live producer yet.
  `kFlagFaultMoveTimeout` (bit 15) is now wired (116) — set on the cycle
  an active `Move` ends via `timeout` rather than its stop condition.
- **`kFlagEventDeadmanExpired` (bit 10) is orphaned by 116, not
  reassigned.** Its sole producer, `Deadman::expired()`, was deleted along
  with `App::Deadman`; the bit constant still exists in `telemetry.h` (no
  wire-shape change) but nothing in the tree calls
  `Telemetry::setFlag(kFlagEventDeadmanExpired, ...)` any more, so it now
  reads permanently 0. Left as declared-dead rather than deleted or
  repurposed — this sprint's scope did not include a `flags` wire-shape
  change, and reassigning a bit number to a new meaning without a version
  signal would be a silent protocol break for any reader still checking
  it. Whether to formally delete or repurpose this bit is open for a
  future sprint.
- **The pre-115 heading-PD/distance-trim/measurement-age-projection design
  history (formerly documented at length in this file's own §2, plus
  `motion/DESIGN.md`) is not carried forward here.** `Pilot`/
  `Motion::Executor`/`HeadingSource` and everything they computed
  (`heading_kp`/`heading_kd` cascade, `distance_kp` trim, `kDeadTime`
  divergence-replan projection, `HeadingSource::headingLead()`) are deleted
  wholesale by 115-005 — the git tag `pre-gut-motion-stack` is the
  authoritative historical record if that design work is ever revisited,
  not a summary re-derived from memory here.
- **`StateEstimator`'s predictions are not exposed on the wire (117).**
  Neither `msg::Telemetry` nor `msg::TelemetrySecondary` gained a field
  for `whereAmI()`/`wheelNow()` output this sprint — validation runs
  host-side against the raw `EncoderReading`/`OtosReading` fields already
  telemetered (sprint 115), replaying the identical ZOH math in Python
  over a captured TLM-log CSV. A future on-robot consumer (the
  remaining-distance trajectory controller) will need `whereAmI()`
  results live, in-process — that consumer calls the estimator directly
  (same process, same cycle), not over the wire, so this gap may never
  need closing; flagged as open only because it was an explicit sizing
  choice, not an oversight.
- **`EstimatorConfigPatch`-set fusion weights are volatile, not
  persisted.** Unlike `MotorConfigPatch`/`OtosConfigPatch` (114-004),
  a live-tuned weight does not survive a reboot — it reverts to the
  baked JSON default. Revisit once fake-OTOS/external-pose fusion
  (future sprints) give these weights real, nonzero, bench-validated
  values worth persisting.
