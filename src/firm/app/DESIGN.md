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
**111-003:** on a natural running→`kIdle` transition happening INSIDE a
single `tick()` call (a command completing on its own, not a same-cycle
flush — distinguished by sampling `executor_.state()` both before and
after the internal `executor_.tick()` call), `Pilot::tick()` stages
`Drive::setTwist(0, 0)` exactly once, rather than leaving `Drive` holding
the previous cycle's stale twist until the ~300ms deadman lease force-
stops it. A same-cycle flush is unaffected — `stateBefore` is already
`kIdle` by the time it's sampled, so this branch is naturally never taken
and the raw command's own twist survives untouched.
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

**The bounded linear position-feedback trim lives in `Pilot::tick()`, not
`Executor` either (112-003) — the SAME gain/arithmetic split, one channel
over.** `Motion::Executor::tick()` additionally exposes the LINEAR
channel's own since-activation reference/measured pair on `Twist`
(`sRef`/`sMeas` — `plannedPositionSinceActivation`/
`measuredPathSinceActivation_`, kArc only, 0/0 for kPivot/kTimed — a pure
straight/arc-leg mechanism that never touches the rotational channel).
`Pilot::tick()` adds `distance_kp*(sRef - sMeas)`, clamped to
`kDistanceTrimCeiling` (a fixed, Pilot-local C++ constant — only the GAIN
is per-robot wire-tunable, not the ceiling), onto `twist.v` before staging
the result on `Drive` — the identical gain-in-Pilot/reference-in-Executor
boundary the heading PD paragraph above documents, chosen for the same
reason: `motion/DESIGN.md` §2c's own "no gain, no sensor type" boundary
for `Executor` stays true without a carve-out (sprint 112 Architecture
Design Rationale Decision 3). Downstream of the PLANNED reference
`refLeft()`/`refRight()` expose (112-002) — the trim perturbs only the
SAMPLED velocity `Drive::setTwist()` receives, never the `JerkTrajectory`
solve itself (no `solveToRest`/`solveToState`/`solveToVelocity`/
`retarget`/`reanchor` call reads `sRef`/`sMeas`), so the ramp/lobe/bounds
harness checks graded against the planned reference (112-002's own
re-grade) are unaffected. The clamp ceiling is sized well below anything
that could look like the solve-side reversal
`.clasi/knowledge/d-drive-terminal-instability.md`/087-009 documents — see
pilot.h's own `kDistanceTrimCeiling` doc comment for the full sizing
derivation (the deadband inequality `distance_kp * distance_tol >=
v_deadband`, re-verified against the actual current `Devices::NezhaMotor`
write-shaping deadband rather than an unchecked architecture-doc figure).
`distance_tol` (the new field alongside `distance_kp`) is NOT yet read by
this trim, or by anything else — it repurposes the role
`Motion::kDistanceSettleEpsilonMm` currently plays as a hardcoded
constant, wired into the completion decision by a later ticket.

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

**`HeadingSource::headingLead()` — measurement-age projection (109-010,
locus 1 of `motion/DESIGN.md` §2c's own three lead-compensation loci).**
`sample(nowUs)` (`nowUs` — `App::RobotLoop`'s own `clock_.nowMicros()`,
threaded through `Pilot::tick(now, nowUs)` as a plain parameter, NOT a new
`Devices::Clock` dependency for either class) tracks `ageS_ = nowUs -
Devices::Otos::lastReadUs()` every cycle — the REAL elapsed time since
OTOS's own cached pose was actually sampled, which is a roughly constant
one-`kCycle` (40ms) gap by construction (`applyOtosSample()` runs in the
cycle's LAST block; `Pilot::tick()`, which calls `sample()`, runs EARLIER
in the SAME cycle — see `robot_loop.cpp`'s own cycle-placement comments).
`headingLead()` returns `heading() + otos_.pose().omega * (ageS_ +
heading_lead_bias)` when `usingOtos_` (collapses to `heading()` unchanged
on the encoder fallback, which has no analogous cross-cycle read-then-
consume gap). This is a SEPARATE quantity from `heading()` — `Executor::
tick()` takes BOTH (`measuredHeadingAbs`/`measuredHeadingLeadAbs`) and
exposes a SEPARATE `Twist::thetaMeasLead` field alongside the existing
`thetaMeas`; `Pilot`'s own heading-PD error term uses `thetaMeasLead`,
while `Executor`'s own dwell/divergence bookkeeping keeps using the raw,
unleaded `thetaMeas` throughout. See `motion/DESIGN.md` §2c for the full
characterization writeup (the fitted equation, the sim-fidelity gap found
and fixed alongside this work, and the honest post-compensation finding —
the shipped `heading_lead_bias` default NEUTRALIZES this projection rather
than improving turn accuracy, a disclosed outcome, not a silent one).

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
- **Config patches cover `MotorConfigPatch`, `OtosConfigPatch` (109-004), and
  `PlannerConfigPatch` (109-008).** `RobotLoop::handleConfig` still replies
  `ERR_UNIMPLEMENTED` for `DRIVETRAIN`/`WATCHDOG`/`NONE`. `DrivetrainConfigPatch`
  remains out of scope — it has no on-robot fusion consumer (unchanged from
  when this note was first written). `PlannerConfigPatch` is NO LONGER a
  scope boundary: the note's original reasoning ("targets a segment executor
  that no longer exists in this tree") described the gap between the
  pre-rebuild segment executor's deletion and Motion::Executor/App::Pilot's
  restoration (109-003/005) — now that `Motion::Executor` and `App::Pilot`
  own the heading PD cascade and per-command tracking/replan gains,
  `handleConfig`'s `PLANNER` arm forwards the decoded patch to
  `Pilot::applyPlannerPatch()` (merge-then-write onto Pilot's own live
  `msg::PlannerConfig` baseline, then re-applied to `Executor::configure()`/
  `HeadingSource::configure()`/`Pilot::configureHeading()` so it takes
  effect immediately) — see `pilot.h`'s own doc comment for the merge
  contract and which `msg::PlannerConfig` fields `PlannerConfigPatch` does
  NOT cover (the schema curates 20 of the struct's fields; a_max/v_body_max/
  yaw_rate_max/etc. are boot-config-only, unreachable from this arm).
  `OtosConfigPatch` (issue `otos-calibration-config-message.md`) restores a
  RUNTIME path to `Devices::Otos::setLinearScalar()`/`setAngularScalar()`/
  `setOffset()`/`init()` — previously only ever called once at boot from
  baked `boot_config` — applied the same way `MotorConfigPatch` already is,
  immediately and synchronously inside `handleConfig()` (still "the loop's
  own cycle" per the single-loop bus ownership invariant above: this is a
  rare, command-triggered I2C/config transaction sandwiched into the
  existing schedule, not a new per-cycle bus consumer, and `otos.h`'s own
  doc comment already documents these four primitives as issuing their
  write immediately rather than staging it, "matching the OI/OR/OL/OA
  wire-command shape"; `Executor::configure()`/`HeadingSource::configure()`
  touch no bus at all — pure in-memory limit setters).

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

- **`RobotLoop::updateTlm()` now populates `frame_.hasTwist`/`frame_.twist`
  (109-009 fix).** Both fields were added to `Telemetry::Frame` by an
  earlier ticket but never actually set anywhere — `hasTwist` defaulted
  `false` permanently, so the wire's `twist=` field was silently absent on
  every build. The sim tour-closure gate (109-009) needed a real velocity
  trace to assert "no dip at a same-`v_max` boundary" against and was the
  first consumer to notice. Fixed with `BodyKinematics::forward
  (motorL_.velocity(), motorR_.velocity(), drive_.trackWidth(),
  frame_.twist.v_x, frame_.twist.omega)` — the same linear/homogeneous
  equations `Odometry::integrate()` already uses for position deltas,
  fed velocities instead (mathematically valid without a separate `dt`,
  per that method's own comment). `Drive` gained a `trackWidth()` read-only
  accessor for this call.
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
- **`Drive::configure(config)`/`setTwist`/`stop`/`tick()`:** `configure()`
  (112-002) reads `actuation_lag`, mirroring `Executor::configure()`/
  `HeadingSource::configure()`'s own "call once, before first use"
  convention — `main.cpp`'s boot wiring calls it once. `setTwist`/`stop`
  only stage a target — `setTwist(v_x, omega, a_x=0, alpha=0)`, the last two
  DEFAULTED (112-002: `Motion::Executor::Twist::aRef`/`alphaRef`, forwarded
  through `Pilot::tick()`) so every pre-existing 2-arg call site (e.g.
  `RobotLoop::handleTwist()`'s raw `TWIST` path) compiles and behaves
  unchanged. `tick()` computes wheel velocities via `BodyKinematics::
  inverse()` and stages them onto the two motor leaves via their own
  `setVelocity()` — it never calls a motor's own `tick()` — PLUS (112-002) a
  model feedforward term: the SAME `inverse()` map reused for the staged
  acceleration (`aL = a_x - alpha*b/2`, `aR = a_x + alpha*b/2` — kinematics
  is linear, so this is exact), added onto `vL`/`vR` as `actuation_lag * a`
  before staging. A no-op whenever `a_x`/`alpha` are 0 (every call site that
  never supplies them).
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
- **`MotorConfigPatch`, `OtosConfigPatch` (109-004), and `PlannerConfigPatch`
  (109-008) are live-appliable.** Only `DrivetrainConfigPatch`/`WatchdogConfigPatch`
  still reply `ERR_UNIMPLEMENTED` (no on-robot fusion consumer for the
  former; the latter routes to `bb.streamWatchdogWindowIn` directly, not
  `handleConfig`, per config.proto's own `CONFIG_WATCHDOG` comment); see §3.
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
  "do not hand-pick a new constant from the old one"). Ticket 006 (the
  intended first consumer, via `checkDivergence()`'s own divergence-
  comparison) tried wiring it in as a `peek(elapsed + kDeadTime)` lead and
  reverted it: 130ms is a large fraction of a typical sub-second pivot/arc's
  own total duration, so "where the plan will be 130ms from now" is not a
  fair stand-in for "where the plan already is" without a matching
  measured-transport-lag model on the OTHER side of the comparison (the
  sim's own measured signal has none to project past) — the projection
  produced false-positive divergence triggers against
  `motion_executor_harness.cpp`'s own pivot dwell scenarios.
  `checkDivergence()` compares against the CURRENT elapsed sample instead;
  `kDeadTime` STILL has no live call site. It stays declared
  (`Motion::kDeadTime`, `motion/executor.h`) with its derivation preserved
  here, flagged for a real fresh bench characterization (not a reuse of a
  DIFFERENT sprint's measurement, however well-reasoned) once USB deploy is
  fixed — a genuine measured-transport-lag model on the comparison's other
  side is likely a precondition for this projection ever being safe to
  wire in, not just a better constant value.
