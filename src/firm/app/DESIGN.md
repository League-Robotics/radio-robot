---
root: ../DESIGN.md
---

# App — Loop and Passive App Modules

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-17 · **Status:** in-flux

---

## 1. Purpose

`app/` is the single cooperatively-timed control loop (`App::RobotLoop`) and
the passive modules it drives: 

* `Comms` (wire framing), 
* `Telemetry` (outbound frames), 
* `Drive` (twist → wheel targets), 
* `Odometry` (dead reckoning),
* `Deadman` (the one staleness gate), 
* `Preamble` (boot-time device detection), and
* `Pilot` (109-003/109-005 — bridges `Motion::Executor` into the loop's
  cycle and computes the heading PD cascade on top of it; see §2's own
  subsection and `motion/DESIGN.md`), and
* `HeadingSource` (109-005 — decides which sensor is truth for heading
  right now: OTOS-first, encoder-differential fallback; see §2's own
  subsection).

This is the seam that owns the robot's *timing* — every I2C
transaction, every wait, every cadence decision lives here or is called from
here in visible order. It exists as its own subsystem because timing
discipline on a shared single-master I2C bus, and cooperative yielding to the
CODAL fiber scheduler, are the two hard realtime problems this firmware has;
drawing the boundary around "the thing that owns the schedule" keeps that
problem in one place instead of smeared across device leaves.

## 2. Orientation

`RobotLoop` has two phases. `boot()` steps `Preamble` until every device
leaf reaches a terminal state (present-and-ready or confirmed-absent),
emitting a boot telemetry frame each pass; commands are not consumed during
boot. `cycle()` is the steady-state loop body: request/settle/collect/PID
for the left motor, decode at most one inbound command (`Comms::pump`),
apply it (`processMessage`), request/settle/collect/PID for the right motor,
then a trailing block that samples OTOS, integrates odometry
(`Odometry::integrate`), and paces the whole cycle. `Telemetry::emit()` is
called once per cycle and decides for itself whether to send the primary
frame, the secondary diagnostic frame, or (on a tie) alternate between them.
`Drive` and `Odometry` are pure, bounded, non-bus-touching helpers that
`RobotLoop` calls at specific points in its own schedule; `Deadman` is
polled once per cycle and gates `Drive::stop()`. See `robot_loop.cpp` for
the exact call order — it is the schedule's single source of truth.

**`Pilot` (109-003/109-005).** `Pilot::tick(now)` runs in the motorR settle
block (after `processMessage()`/the deadman check, before `drive_.tick()`)
and samples `HeadingSource` (see below) and `Motion::Executor`, staging the
result onto `Drive` via `setTwist()` whenever the executor is not `kIdle` —
while `kIdle` it does nothing at all, so a same-cycle raw `TWIST` (which
always calls `Pilot::flush()` first) is never immediately overwritten.
`Pilot::plan()` runs in the trailing `kPace` block (after
`odom_.integrate()`) and performs at most one `JerkTrajectory` solve per
cycle (`Motion::Executor::plan()`'s own budget). `RobotLoop::handleMove()`
decodes a `Move` command (`Comms::pump()`/`processMessage()`, same dispatch
switch as `TWIST`/`CONFIG`/`STOP`) into a `Motion::Cmd` and calls
`Pilot::enqueue()`; `RobotLoop::drainPilotEvents()` drains
`Motion::Executor`'s completion-event FIFO into `Telemetry`'s ack ring
every cycle. See `motion/DESIGN.md` §2b for the executor's own queue/state-
machine contract.

**The heading PD cascade lives in `Pilot::tick()`, not `Executor`
(109-005).** `Motion::Executor::tick()` returns a `Twist` carrying the
feedforward rate (`omega`/`omegaDes`, meaningful as feedforward-ONLY when
`headingActive` is true), the arc/pivot's own progressive heading reference
(`thetaRef`), and the measured heading rebaselined to the command's own
activation instant (`thetaMeas`, computed from whatever `Pilot` passed into
`tick()` — see below). `Pilot::tick()` adds `heading_kp*(thetaRef -
thetaMeas) + heading_kd*(omegaDes - omegaMeasEst)` on top of `omega` when
`headingActive` is true — `omegaMeasEst` is `Pilot`'s OWN finite-difference
estimate of `thetaMeas`'s rate across consecutive `tick()` calls, kept
separately from `Executor`'s own internal dwell-rate estimate (same method,
two independent state variables, serving two different decisions — "what
should the PD command right now" vs. "is this command done"). This split
(gains/arithmetic in `Pilot`, plan/reference/measurement-relative-to-
activation in `Executor`) matches sprint.md's own SUC-002 flow ("Each
cycle, Pilot::tick() computes omega_cmd = omega_ff + heading_kp*(...)") and
keeps every sensor type and every gain out of `motion/` entirely — see
`motion/DESIGN.md` §2c for the executor-side half.

**`HeadingSource` (109-005).** A passive reader, no bus traffic of its
own — `sample()` reads `Devices::Otos::pose()`/`poseFresh()`/`connected()`/
`present()` and `Devices::NezhaMotor::position()` (both leaves), all
already refreshed elsewhere in THIS SAME cycle by `applyOtosSample()`/the
motors' own `tick()` calls, never issuing a read itself. Policy: OTOS
whenever `present() && connected() && poseFresh()`; after
`kFallbackStaleCycles` (5, v1/not-bench-tuned) CONSECUTIVE cycles without
that, demote to the encoder-differential formula `(right.position() -
left.position()) / trackWidth`; re-promote to OTOS on the very next cycle
it is usable again (no analogous hysteresis on the recovery side).
`msg::PlannerConfig.heading_source` (`HeadingSourceMode`) overrides this
per-robot (`FORCE_OTOS`/`FORCE_ENCODER` skip the state machine entirely —
for a robot with a known-bad OTOS mount, or a bench rig with none wired at
all) — baked from the robot JSON's `control.heading_source` via
`gen_boot_config.py`. `Pilot::tick()` calls `sample()` every cycle
(`kIdle` included, so a fallback that happens between commands is still
visible) and forwards `heading()` into `Executor::tick()`'s own
`measuredHeadingAbs` parameter. Visibility: `Telemetry`'s primary frame
gains `headingSource` (mirrors `telemetry.proto`'s `HeadingSourceStatus`);
`event_bits` bit 3 (`kEventHeadingFallback`) fires the one cycle the active
source flips either direction — see `Pilot::headingSourceIsOtos()`/
`headingSourceFellBack()`/`headingSourceRecovered()` and
`RobotLoop::updateTlm()`.

## 3. Constraints and Invariants

- **Single-loop bus ownership:** every I2C transaction happens from
  `RobotLoop::cycle()`'s own call sequence. No app module ever initiates bus
  traffic from its own `tick()`/staging methods on its own timing — see the
  root doc's "single-loop bus ownership" invariant. `Odometry::integrate()`
  and `applyOtosSample()` are called only from the loop's trailing block,
  never from inside a motor request→collect window.
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
  dispatch, drive staging). Moving bus traffic into one of these bodies
  reintroduces the shared-bus timing collisions a single-master I2C bus
  cannot tolerate.
- **Command dispatch is bounded to at most one per cycle:** `Comms::pump()`
  decodes at most one frame per call by construction (at most one
  `readLine()` per transport, first transport to have something wins), so
  `processMessage()` needs no separate "already handled" flag.
- **Deadman is the only staleness gate:** one `App::Deadman`, armed by
  every actuation command, checked once per cycle, expiry → `Drive::stop()`
  (109-003: also `Pilot::flush()`, since a stale Executor plan is just as
  wrong as a stale raw twist). Do not add a second ad hoc watchdog anywhere
  in `app/`. `Pilot`/`Executor` re-arm `Deadman` every non-`kIdle` cycle
  with a fixed ~300ms lease (`kPilotDeadmanLease`, `robot_loop.cpp`) —
  deliberately NOT derived from a `Move`'s own `time` field: a TIMED
  command's own deadline is its own ramp-down/completion bound
  (`Motion::Executor`'s `RAMP_TO_REST` logic), independent of this generic
  "host went silent" lease every actuation source shares.
- **Telemetry always carries the last staged snapshot, not a diff:** a
  cycle that doesn't update a `Frame` field still sends whatever was last
  staged. Nothing here is "only send on change" — a dropped or unread frame
  never loses data because the next one repeats it.
- **`Frame` fields written late in a cycle are read by the NEXT cycle's
  emit, not lost:** pose/OTOS are staged at the end of the cycle they are
  computed in, and picked up by the following cycle's `updateTlm()` +
  `emit()`. This is deliberate — treat a one-cycle staleness on those two
  fields as normal, not a bug.
- **Devices isolation still applies inside `app/`:** wire-plane `msg::*`
  types are converted to/from `Devices::*` types only in `main.cpp`
  (outside this directory); no app module should reach around that.
- **Only OTOS is sampled in steady state.** `Preamble` detects line/color
  sensor *presence* at boot, but nothing in the steady-state cycle samples
  them — deliberately deferred (see §6). Do not "helpfully" wire a
  line/color read into the trailing block without also extending
  `Telemetry`'s wire schema; there is nowhere for the data to go yet.
- **Config patches cover `MotorConfigPatch` and `OtosConfigPatch` (109-004).**
  `RobotLoop::handleConfig` replies `ERR_UNIMPLEMENTED` for every other
  `ConfigDelta` patch kind (`DRIVETRAIN`/`PLANNER`/`WATCHDOG`/`NONE`). This
  is a scope boundary, not an oversight — `DrivetrainConfigPatch` has no
  on-robot fusion consumer, and `PlannerConfigPatch`'s heading gains target
  a segment executor that no longer exists in this tree. `OtosConfigPatch`
  is the one addition since this note was first written (issue
  `otos-calibration-config-message.md`): it restores a RUNTIME path to
  `Devices::Otos::setLinearScalar()`/`setAngularScalar()`/`setOffset()`/
  `init()` — previously only ever called once at boot from baked
  `boot_config` — applied the same way `MotorConfigPatch` already is,
  immediately and synchronously inside `handleConfig()` (still "the loop's
  own cycle" per the single-loop bus ownership invariant above: this is a
  rare, command-triggered I2C transaction sandwiched into the existing
  schedule, not a new per-cycle bus consumer, and `otos.h`'s own doc
  comment already documents these four primitives as issuing their write
  immediately rather than staging it, "matching the OI/OR/OL/OA
  wire-command shape").

## 4. Design

**Why one loop.** `RobotLoop::cycle()` is deliberately one function with
every bus transaction and every wait visible in call order, rather than a
dispatch graph of modules each with their own timing. The alternative
(subsystems/fibers each owning a slice of the schedule) hides the bus
schedule and the sleeps inside layers, which makes both hard-realtime
problems — bus discipline and fiber-scheduler yielding — undebuggable.
Modules (`Drive`, `Odometry`, `Telemetry`, `Comms`, `Deadman`, `Preamble`)
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
whole-cycle target `kCycle` (~40ms / ~25Hz) — `kPace` is *derived* as
`kCycle` minus the other three, not a second independent `kCycle`-sized
sleep, specifically so the schedule's total holds even under a
zero-real-time-cost virtual clock (anchoring the final block to the cycle
start instead of its own mark was a diagnosed defect: it double-counted the
first three blocks' time against the target). `kCycle` matches
`Telemetry::kPrimaryPeriod` by construction so the primary-frame cadence
and the loop's own pace agree.

**Command dispatch.** `processMessage` reads the `Cmd` populated (or not)
by this cycle's single `Comms::pump()` call and switches on `cmd_kind`:
`TWIST` calls `Pilot::flush()` (preempts/flushes the `Motion::Executor`
queue) THEN stages a target on `Drive` and arms `Deadman` — flush first so
this cycle's own later `Pilot::tick()` call sees `state()==kIdle` and does
not restage a twist over the raw one; `STOP` likewise calls
`Pilot::flush()`, then stops `Drive` (unchanged, immediate, safety-critical
— see `motion/DESIGN.md` §2b's own note on why the wire `STOP` stays an
instant stop rather than routing through `Executor`'s own graceful
`solveToVelocity(0)` decel) and disarms `Deadman`; `CONFIG` merges present
wire fields into each motor's *own* current gains (never blanket-copies one
motor's gains onto the other — their calibration can legitimately differ)
and applies `travel_calib` to whichever motor `side` names; `MOVE`
(109-003) decodes into a `Motion::Cmd` and calls `Pilot::enqueue()`, acking
the ENQUEUE outcome (accepted/replaced/full/trivial/unimplemented) against
the envelope's own `corr_id` — a later completion event for the SAME
command rides a separate ack keyed by the `Move`'s own `id` field instead
(`RobotLoop::drainPilotEvents()`, called every cycle). Every path that
applies a command acks through `Telemetry`'s ack ring; `Comms`'s dearmor
path itself never replies synchronously — a malformed frame is silently
counted (`Comms::malformedCount()`) and surfaced as a telemetry fault bit
instead of answered inline. This keeps replies flowing through one channel
(the ack ring) rather than two.

**Telemetry's two send paths.** The primary frame (`msg::Telemetry`, ack
ring + fault/event bits + pose/enc/vel) rides a `ReplyEnvelope` through
`Comms::sendReply()`. The secondary diagnostic frame
(`msg::TelemetrySecondary`) is not a `ReplyEnvelope` oneof arm, so
`Telemetry` holds its own `Transport&` pair and performs its own
armor+broadcast for that one frame type, reusing `Comms`'s armor buffer
size and `WireRuntime::base64Encode()` rather than duplicating a private
encode path. `emit()` sends at most one frame type per call and normally
lets whichever frame is due win; when both are genuinely due in the same
call it *alternates* rather than always favoring primary — at the real
loop period (~40-50ms), primary is due on essentially every call, so an
unconditional "primary wins ties" rule starves secondary to 0Hz. The
alternation costs at most one primary frame delayed by one cycle roughly
once per secondary period; a non-tied call is unaffected.

**Fault/event bit layout.** `fault_bits` bit 0 (`kFaultI2CSafetyNet`) mirrors
`I2CBus::clearanceSafetyNetCount() > 0` — on real hardware this has been
observed as a one-shot latch coincident with `Preamble::done()`'s
transition, not a live/continuous indicator; a steady 1 after boot with no
in-flight anomaly is not itself evidence of a defect, only a bit that flips
*during* driving is actionable. Bit 1 (`kFaultWedgeLatch`) mirrors
`motorL_.wedged() || motorR_.wedged()`. Bit 3 (`kFaultCommsMalformed`)
mirrors `Comms::malformedCount() > 0`. Bit 2 (`kFaultI2CNak`) is declared
but not wired (no per-transaction NAK aggregate exists yet). `event_bits`
bit 0 (`kEventDeadmanExpired`) mirrors `Deadman::expired()`; bit 1
(`kEventBootReady`) fires on `Preamble::done()`'s first-true transition;
bit 2 (`kEventConfigApplied`) is declared but not wired. Declaring a bit
before it is wired is deliberate — it reserves the bit number for a future
caller without renumbering.

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
- **`Telemetry::setFrame`/`setFault`/`setEvent`/`ack`/`emit(now)`:** staging
  calls are cheap and can be called any number of times per cycle;
  `emit(now)` is the one call that actually sends, at most one frame type,
  bounded work, never sleeps, never touches the I2C bus.
- **`Drive::setTwist`/`stop`/`tick()`:** `setTwist`/`stop` only stage a
  target; `tick()` computes wheel velocities via `BodyKinematics::inverse()`
  and stages them onto the two motor leaves via their own `setVelocity()` —
  it never calls a motor's own `tick()`.
- **`Odometry::integrate()`:** call once per cycle, after both motors' own
  `tick()` has run that cycle; reads each leaf's current `position()` and
  accumulates world pose via midpoint-arc integration over
  `BodyKinematics::forward()`'s per-cycle body-frame delta.
- **`applyOtosSample(otos, now, frame)`:** safe to call every cycle — a
  too-soon call given OTOS's own internal rate limit is already a
  documented no-bus-traffic no-op. Must not be called from inside a motor
  request→collect window (bus-discipline is the loop's responsibility, not
  this function's).
- **`Deadman::arm(duration)`/`disarm()`/`expired()`:** `arm()` always sets a
  fresh deadline from now (re-arms, never stacks); negative/NaN duration
  clamps to 0 (immediate expiry).
- **`Preamble::step()`/`done()`/per-device status accessors:** `step()`
  never blocks; `done()` is true once every device has reached a terminal
  state (present-and-ready or confirmed-absent).
- **`Pilot::enqueue(cmd)`/`flush()`/`plan()`/`tick(now)`/`popEvent(out)`/
  `queueDepth()`/`activeId()`/`state()`/`configureHeading(config)`**
  (109-003/109-005): see this file's own §2 "`Pilot`" subsection for the
  cycle-placement contract. `Telemetry`'s primary frame gains
  `queueDepth`/`activeId`/`execState`/`headingSource` (mirroring
  `telemetry.proto`'s `queue_depth`/`active_id`/`exec_state`/
  `heading_source`), populated by `RobotLoop::updateTlm()` from `Pilot`'s
  own accessors (the last also from `Pilot::headingSourceIsOtos()`).
- **`HeadingSource::configure(config)`/`sample()`/`heading()`/
  `usingOtos()`/`fellBackThisSample()`/`recoveredThisSample()`**
  (109-005): see this file's own §2 "`HeadingSource`" subsection.

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
- **`Motion::Executor`, `Motion::Cmd`, `Motion::fromMove()`** (109-003) —
  the queue/state-machine `Pilot` bridges into the loop's cycle; see
  [motion/DESIGN.md](../motion/DESIGN.md) §2b/§2c.

## 6. Open Questions / Known Limitations

- **Line/color steady-state sampling is absent.** `Preamble` detects
  presence at boot; no cycle slot samples either sensor in steady state,
  and `Telemetry`'s wire schema carries no line/color fields yet. A full
  perception round-robin (otos|line|color) is deliberately deferred.
- **Only `MotorConfigPatch` and `OtosConfigPatch` are live-appliable
  (109-004).** Drivetrain and planner config patches reply
  `ERR_UNIMPLEMENTED`; see §3.
- **In-session pose reset has no wire verb yet.** `Odometry::reset()`
  exists and is exercised by the host simulator's teleport-to-origin, but
  no binary command arms it from the wire today.
- **`kFaultI2CNak` and `kEventConfigApplied` are declared but unwired** —
  reserved bit numbers with no live producer yet.
- **`HeadingSource`'s `kFallbackStaleCycles` (5) is a v1, NOT-bench-tuned
  constant** — a conservative guess ("a few tenths of a second"), flagged
  for revision once a real bench arc/pivot sweep exists (ticket 009's own
  gate). Same posture as `Motion::Executor`'s own `kTerminalDecelWindowS`/
  `kStopTimeBackstopFactor` (`motion/DESIGN.md` §2c) and `kDeadTime`
  (§6 below).
- **`kDeadTime` (divergence-replan dead-time projection, ticket 006's own
  consumer) is re-derived, not carried over from the old 120ms/20ms-tick
  value, but NOT freshly bench-characterized this ticket.** Ticket 005's
  own acceptance criterion calls for a fresh stand characterization; the
  USB deploy path was confirmed broken this session (one `mbdeploy probe`
  attempt, per `.claude/rules/hardware-bench-testing.md`'s own escalation
  path — see this ticket's completion notes for the exact failure/output).
  In its place, `kDeadTime` is set to 130ms — the midpoint of the ALREADY
  bench-measured `motor_lag` figure sprint 100's own
  `architecture-update.md` records ("120-140ms" — a real-time physical
  actuation-transport delay, independently re-derived from THAT bench
  session, not a tick-count artifact of the old 20ms cycle) — rather than
  hand-picked by scaling the OLD constant's own tick count onto the new
  40ms cycle (explicitly disallowed by ticket 005's own semantics item 6:
  "do not hand-pick a new constant from the old one"). This value has NO
  live call site yet (ticket 006 is the first consumer, via
  `retarget()`/`reanchor()`'s own divergence triggers) — it is declared
  (`Motion::kDeadTime`, `motion/executor.h`) and derived here so ticket 006
  does not have to re-derive it from scratch, and is flagged for a real
  fresh bench characterization (not a reuse of a DIFFERENT sprint's
  measurement, however well-reasoned) once USB deploy is fixed, per this
  ticket's own acceptance criterion.
