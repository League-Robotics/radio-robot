---
root: ../DESIGN.md
---

# Motion (`src/firm/motion`, namespace `Motion`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-17 · **Status:** in-flux

---

## 1. Purpose

`motion/` solves jerk-limited (or trapezoid, if unconfigured) motion
profiles for a single 1-DoF channel, and (as of 109-003) sequences those
solves into continuous, queued motion. `Motion::JerkTrajectory` wraps one
vendored `ruckig::Ruckig<1>`/`ruckig::Trajectory<1>` pair (`src/vendor/
ruckig/`) and answers exactly one question per instance: "given where this
channel is now (or was last told it was), what is the jerk-limited path to
a requested target (position, and optionally velocity), and what is that
path's state at time T?" It knows nothing about goal kinds (arc, pivot,
straight leg), wire verbs, CODAL, or queueing — `Motion::JerkTrajectory`
never appears in a `msg::` type or an `App::` header. `Motion::Cmd`/
`Motion::Executor` (109-003) is the first (and, as of this ticket, only)
consumer: it holds two `JerkTrajectory` instances (linear and rotational
channel), sequences a fixed ring of normalized arc commands through them,
and is itself driven from the loop's cycle via `App::Pilot`
(`app/pilot.{h,cpp}`) — see `app/DESIGN.md` for the loop-glue half of this
story. 109-001 restored the solver only; 109-003 was the first real call
site (TIMED mode); 109-005 added DISTANCE mode (coupled arcs and pure
pivots, §2c) — the sprint's own turn-accuracy motivation.

## 2. Orientation

### 2a. `Motion::JerkTrajectory` — the single-channel solver

One class, `JerkTrajectory`, with three solve entry points and one sample
entry point:

- **`solveToRest(targetPosition, maxVelocity)`** — position-control,
  decelerate to rest exactly at `targetPosition`. Equivalent to
  `solveToState(targetPosition, 0, maxVelocity)`.
- **`solveToState(targetPosition, targetVelocity, maxVelocity)`**
  (109-001's new entry point) — position-control, arrive at
  `targetPosition` carrying `targetVelocity` (nonzero) rather than stopping
  — the boundary-velocity-carry primitive a future queue/executor needs so
  consecutive same-direction commands don't decelerate to zero at each
  hand-off.
- **`solveToVelocity(targetVelocity, maxVelocity)`** — velocity-control,
  open-ended (no target position); used for cruise ramp-up and,
  target-velocity 0, any stop-triggered terminal decel.
- **`retarget(newRemaining)`** / **`reanchor(position, velocity)`** —
  divergence-triggered re-solves for a future replan policy (normal case:
  re-baseline and re-solve, seeded from this channel's own remembered
  state; gross case: re-solve seeded from a caller-supplied state,
  accepting a velocity discontinuity).
- **`sample(elapsed)`** / **`peek(elapsed)`** — evaluate the held
  trajectory; `sample()` updates the remembered seed state used by the next
  solve, `peek()` is a pure read that does not.

Every method's public signature uses only `float`/`JerkTrajectory::State` —
no `ruckig::` type crosses this class's boundary. See `jerk_trajectory.h`'s
class comment for the full design rationale (solve modes, the jerk == 0
sentinel, the direction-mirrored acceleration bounds) — it is the primary,
load-bearing design reference for this subsystem; this document stays at
map/boundary altitude and does not repeat it.

### 2b. `Motion::Cmd` / `Motion::Executor` — the ring queue and state machine (109-003)

`Motion::Cmd` (`cmd.h`) is a normalized, plain-value copy of a decoded
`msg::Move` (envelope.proto) — `fromMove()` is the one conversion point.
`Motion::Executor` (`executor.{h,cpp}`) owns:

- A fixed ring of `kQueueDepth` (8) pending `Cmd`s plus one active `Cmd`.
- A state machine — `State::{kIdle, kRunning, kRampToRest, kStopping}` —
  mirrored on the wire by `telemetry.proto`'s `ExecutorState` enum
  (`App::Pilot::state()`/`RobotLoop::updateTlm()` translate one to the
  other; see `app/DESIGN.md`).
- Two owned `JerkTrajectory` instances (linear, rotational).
- A small completion-event FIFO (`popEvent()`), drained by
  `RobotLoop::drainPilotEvents()` into `Telemetry`'s existing ack ring —
  see `messages/DESIGN.md`'s and `telemetry.proto`'s own doc comments for
  why completion events ride the ack ring rather than the orphaned
  `messages/event.h` (sprint.md's Open Question 3, resolved by this
  ticket).

**109-003 scope was TIMED mode + `replace` only; 109-005 adds DISTANCE
mode (kArc/kPivot) — see §2c below.** `Cmd::isTimed()` (`Move.time > 0`)
is the teleop primitive, implemented end to end: TIMED drives BOTH
channels independently and directly from `Cmd::vMax`/`Cmd::omega` — there
is no heading reference to slave against (unlike DISTANCE mode's
dominant/slaved-channel coupling, §2c). `enqueue()` classifies every
incoming `Cmd` in this order:

1. **Degenerate** (`Cmd::isDegenerate()`: zero distance, zero heading
   delta, `time<=0`) → `EnqueueOutcome::kTrivial`, never queued.
2. **TIMED, `replace==false`** → activates immediately if `kIdle` and the
   ring is empty, else appends to the ring tail (`kFull` if already at
   `kQueueDepth`, plan untouched).
3. **TIMED, `replace==true`** → replaces the ring's own tail if
   non-empty (`kSuperseded` completion event for the evicted entry),
   else retargets the ACTIVE command in place if one is running (a fresh
   `solveToVelocity()` toward the new target, seeded from the channel's
   own last sample per `JerkTrajectory`'s seeding contract — smooth, never
   an instantaneous step; `kSuperseded` for the old active id), else
   behaves like a fresh enqueue.
4. **DISTANCE mode** (`time<=0`, non-degenerate) → `Cmd::isPivot()`
   (`distance==0`) selects `Mode::kPivot`; otherwise `Mode::kArc`
   (`distance!=0`, `deltaHeading` possibly 0 for a plain straight leg).
   Both activate/queue exactly like TIMED (same ring, same `kFull`/
   `replace` rules) — the ONLY difference is which channel(s) `activate()`
   requests a solve for and how `tick()` computes the twist and decides
   completion. See §2c.

**Deadline-driven `RAMP_TO_REST`.** A TIMED command's `time` is a total
duration from activation, ramps included (sprint.md's own
stakeholder decision). Rather than pre-planning a fixed three-segment
profile, `tick()` compares the command's remaining time each cycle
against `estimateStopDuration()` — an analytic v1 approximation
(`|v|/aDecel`, plus one `aDecel/jerk` S-curve-ramp term when jerk-limited,
and explicitly **zero when the channel is already at rest** — a channel
that never moves, e.g. the rotational channel of a pure-linear TIMED
command, must never report a nonzero "time needed to stop" just because
its OWN aDecel/jerk pair is nonzero; this exact bug — a stationary
rotational channel's own `aDecel/jerk` term alone exceeding a short TIMED
deadline and firing `RAMP_TO_REST` before the moving linear channel ever
ramped up — was caught by this ticket's own sim system test
(`test_move_queue.py`) before being fixed) — of how long a
`solveToVelocity(0, ...)` decel from the currently sampled velocity would
take. Once remaining time is at or below that estimate, both channels get
a fresh solve request toward 0 and the state flips to `kRampToRest`;
completion (`kDone`) fires once both channels sample near-zero velocity
AND no solve is still pending.

**Solve budget: at most one `JerkTrajectory` solve per `plan()` call**
(the `kPace`-block budget, `src/firm/DESIGN.md` §3). A fresh TIMED command
needing both channels solved takes two `plan()` calls (~2 loop cycles,
~80ms) — matching sprint.md's own stated tolerance. `tick()` never solves
— sample-only (`JerkTrajectory::sample()`), matching `App::Pilot::tick()`'s
motor-settle-block placement.

**Per-channel elapsed time is tracked separately from the command's own
elapsed time.** `JerkTrajectory::sample(elapsed)`'s own contract is
"elapsed time since THIS TRAJECTORY WAS SOLVED" (jerk_trajectory.h), not
since the command activated — the two channels are solved on different
`plan()` calls (at most one per cycle) and a mid-flight `replace`
re-solves one channel from a fresh t=0 without touching the other's own
clock. `Executor` keeps `linearElapsedS_`/`rotationalElapsedS_`,
independent of `activeElapsedMs_` (the command-level clock the
`RAMP_TO_REST` deadline comparison uses), reset to 0 on that channel's OWN
successful solve. Conflating the two (using one "elapsed since
activation" value for both channels' `sample()` calls) was this ticket's
OTHER caught bug — sampling a trajectory at the wrong point on its own
timeline reads a stale/wrong state, most visibly right after a replace.

**`flush()`** (TWIST/STOP preemption, `App::Pilot::flush()`) empties the
ring and clears the active command, pushing a `kFlushed` event for each,
returning to `kIdle`. It does not itself touch `Drive` — see
`app/DESIGN.md`'s `Pilot`/`RobotLoop::handleTwist()`/`handleStop()` notes
for how a raw `TWIST`/panic-stop `STOP` and `flush()` interact within one
cycle.

### 2c. DISTANCE mode — `kArc`/`kPivot`, the heading feedforward, dwell completion, overshoot carry (109-005)

**Dominant-channel planning.** `Cmd::isPivot()` (`distance==0`) plans
ONLY the rotational channel (`solveToRest(deltaHeading, ...)`) — the
linear channel is never solved (`JerkTrajectory::sample()` on an
un-`calculate()`'d instance returns a safe zero `State{}`, so `v` stays 0
throughout with no special-casing). Otherwise (`distance!=0`, a `kArc`
command — a straight leg when `deltaHeading==0`, a curve otherwise) ONLY
the linear channel is solved (`solveToRest(effectiveDistance_, ...)`,
ceilinged by `Cmd::vMax`); the rotational channel is never solved — it is
SLAVED every `tick()` to the linear channel's own sampled (position,
velocity) via the arc ratio `headingRatioPerMm_ = deltaHeading/distance`
(computed ONCE at `activate()` from the Cmd's own UN-adjusted `distance` —
the arc's curvature is a property of the requested geometry, independent
of how the overshoot carry below nudges the effective target):
`thetaRef(t) = headingRatioPerMm_ * linear.position(t)`, `omegaFf(t) =
headingRatioPerMm_ * linear.velocity(t)`. This reuses the SAME single-
channel `JerkTrajectory` wrapper unchanged for both cases — no multi-DOF
solve, per sprint.md's own Decision 2 ("dominant-channel planning... vs.
a true 2-DOF simultaneous solve").

**The heading PD cascade lives in `App::Pilot`, not here.** `Executor`
computes and exposes the feedforward half only (`Twist::omega`/
`omegaDes`/`thetaRef`, plus the command-relative measured heading
`thetaMeas`) — `Pilot::tick()` adds `heading_kp*(thetaRef-thetaMeas) +
heading_kd*(omegaDes-omegaMeas)` on top when `Twist::headingActive` is
true (sprint.md's own SUC-002 flow explicitly assigns this arithmetic to
`Pilot`). This keeps every sensor type and every gain entirely out of
`motion/` — see `app/DESIGN.md`'s own `Pilot`/`HeadingSource` subsections
for the other half of this split.

**The terminal-decel PD gate is ERROR-based, not time-based.**
`headingActive` goes false once the command has ALREADY satisfied the
dwell gate's own tolerance/rate test (below), not during a fixed final
window of the dominant channel's own PLANNED duration. A time-based
window was this ticket's own FIRST implementation and was caught by this
ticket's own sim system test (`test_heading_source.py`): a real (laggy)
plant that was still meaningfully off target when the fixed window opened
had its correction authority pulled exactly when it was needed most,
latching a several-degree overshoot the PD was never given the chance to
close (~96° vs. a commanded 90° pivot, observed in that test before the
fix). The error-based gate ("stop correcting once you've already landed
within tolerance", not "stop correcting once the plan says you should be
nearly done") closes the intended failure mode (a commanded reversal
right at an ALREADY-GOOD landing —
`.clasi/knowledge/d-drive-terminal-instability.md`) without also
disabling correction while genuinely still far off — see `executor.h`'s
own "Terminal-decel PD gate" comment for the full before/after.

**Distance completion + same-sign overshoot carry.** A `kArc` command's
own distance criterion is `|measuredPathSinceActivation_| >=
|effectiveDistance_|`, where `measuredPathSinceActivation_` accumulates
`App::Odometry::lastDistance()` (encoder-relative, NOT OTOS) every
`tick()` since activation — `Executor` holds this accumulator (not
`Pilot`), so the completion DECISION stays here even though the raw
sample comes from outside. The signed remainder
(`measuredPathSinceActivation_ - effectiveDistance_`) becomes
`pendingOvershoot_`, consumed by the VERY NEXT activation IFF that
command is itself a same-sign `kArc` command (`effectiveDistance_ =
cmd.distance - pendingOvershoot_`, clamped to a same-sign residual rather
than ever flipping direction) — any other next command silently drops
the carry. This is single-command bookkeeping only, NOT the full
boundary-velocity carry (ticket 006's own scope, which is about NOT
decelerating to rest at the shared boundary at all).

**Dwell completion (heading-bearing commands).** A REST-TERMINATED
heading-bearing command (`queueCount_==0` at the moment its own
distance/pivot criterion is met) additionally holds
`|deltaHeading-thetaMeas| < heading_dwell_tol` AND `|thetaRate| <
heading_dwell_rate` (`msg::PlannerConfig`, both new fields) for
`arrive_dwell` seconds (REUSED from the pre-existing terminal-completion
dwell field — its 150ms default is also exactly ticket 005's own dwell-
hold spec) before completing `kDone`; a `STOP_TIME` backstop
(`stopTimeBackstopMs()` — a generous multiple of the dominant channel's
own solved duration, v1/not-bench-tuned) forces completion regardless, so
a persistent oscillation or a stuck measurement can never wedge the
executor open forever. A CHAINED (non-terminal) heading-bearing command
skips the hold-timer/rate gate entirely and completes on the tolerance
test alone ("accurate handoff... without a dwell", sprint.md's own
semantics item 4).

**`kDeadTime` (`Motion::kDeadTime`, `executor.h`) is declared but has no
live call site yet** — ticket 006's own divergence-replan
`retarget()`/`reanchor()` triggers are the first consumer. Re-derived at
the 40ms cycle from sprint 100's own already-bench-measured `motor_lag`
figure (120-140ms, `architecture-update.md`) rather than hand-picked by
scaling the old 120ms/20ms-tick constant onto the new cycle (explicitly
disallowed by this ticket's own semantics) — but NOT itself a fresh bench
characterization (USB deploy was confirmed broken this session; one
`mbdeploy probe` attempt per `hardware-bench-testing.md`'s own escalation
path). See `app/DESIGN.md`'s own "kDeadTime" Open-Questions entry for the
full derivation and the flag for a real re-characterization later.

## 3. Constraints and Invariants

- **HOST_BUILD-pure, no `MicroBit.h`.** `jerk_trajectory.{h,cpp}` compile
  under both the ARM target and `-DHOST_BUILD` unchanged — this is a leaf
  library exactly like `kinematics/`, not an ARM-only module.
- **No heap.** `ruckig::Ruckig<1>`/`ruckig::Trajectory<1>` are
  compile-time-DoF, `std::array`-backed value types held as ordinary stack/
  member state — no dynamic allocation anywhere in this subsystem.
- **Seeding contract: never seed a solve from a measured observation.**
  Every `solveToRest()`/`solveToState()`/`solveToVelocity()` call reads its
  current (position, velocity, acceleration) back from this channel's OWN
  remembered last sample — never from a live sensor reading (`leftObs`/
  `rightObs` in the pre-102 codebase's terms). This is the single most
  important invariant in this subsystem: an earlier version of this
  codebase fed a measured wheel speed into a similar formula and produced a
  traced limit-cycle oscillation (bug 087-009, `.clasi/knowledge/`).
  `retarget()`/`reanchor()` are the ONLY two narrow, deliberate exceptions
  (both still solver-internal re-solves, never a bus read) — see the header
  comment. A `test_never_reads_measured_observations`-style static text
  check (`src/tests/sim/unit/test_jerk_trajectory.py`) pins this by scanning
  the class's own code for `leftObs`/`rightObs`.
- **`calculated_` UB guard.** A default-constructed `ruckig::Trajectory<1>`
  has real-zero `duration` but an uninitialized `profiles` array; `sample()`/
  `peek()`/`duration()` must never touch `traj_` before the first successful
  `calculate()` — guarded by the `calculated_` flag, returning a safe zero
  `State{}`/`0.0f` instead.
- **This subsystem does not own the never-solves-backward guard, divergence
  thresholds, or replan rate limiting.** Those are a future caller's
  (`Motion::Executor`, ticket 003) responsibility — `retarget()`/
  `reanchor()` solve whatever they are told to solve, including a
  backward-pointing target, by design (see the header comment's
  `scenarioBackwardTargetIsDefinedButUnguarded`-style test).
- **`jerk == 0` sentinel, not a literal zero jerk.** `configure()`'s
  `j_max`/`yaw_jerk_max == 0.0f` maps to Ruckig's own `max_jerk = +infinity`
  (a trapezoid profile), matching the existing `msg::PlannerConfig` wire
  convention — do not special-case a literal `0.0` max_jerk anywhere else in
  this subsystem.
- **`Motion::Executor` calls `JerkTrajectory` — it never solves the math
  itself** (§2b's own "calls into JerkTrajectory... never does the solve
  math itself" boundary). Divergence thresholds and boundary-velocity carry
  across a DISTANCE command BOUNDARY (not-decelerating-to-rest at a shared
  boundary) are still explicitly OUT of scope (ticket 006) — do not add
  them to `Executor` incrementally. The heading PD cascade's own GAINS and
  ARITHMETIC live in `App::Pilot`, not here (§2c) — `Executor` owns nothing
  beyond the ring/state machine/solve requests/completion events/the
  feedforward+measured-heading bookkeeping §2c describes — no bus access,
  no wire codec, no `App::Drive` reference, no `heading_kp`/`heading_kd`
  (that is `App::Pilot`'s own boundary).

## 4. Design

`JerkTrajectory` has one private worker, `solvePositionControl()`, shared by
`solveToRest()`, `solveToState()`, `retarget()`, and `reanchor()` — all four
are the same Ruckig `Position`-control-interface solve, differing only in
what current/target state (including, since 109-001, a target velocity)
each passes in. `solveToVelocity()` is the one genuinely different mode
(Ruckig's `Velocity` control interface, open-ended, no target position).
Every solve writes into a temporary `ruckig::Trajectory<1>` first and only
commits it to the held `traj_` on success (`Result::Working`) — a failed
solve must never corrupt the trajectory a caller is still sampling.
Direction (for the direction-mirrored acceleration bounds and the
no-reversal velocity band) is computed purely from `targetPosition -
currentPosition`'s sign — a math fact derived from the solve's own inputs,
never a caller-supplied flag.

## 5. Interfaces

### Exposes

- **`Motion::JerkTrajectory`** (`jerk_trajectory.h`) — `configure()`,
  `reset()`/`seedCurrent()`, `solveToRest()`/`solveToState()`/
  `solveToVelocity()`, `retarget()`/`reanchor()`, `sample()`/`peek()`,
  `duration()`. See §2 above and the header's own class comment for the
  full contract.
- **`Motion::Cmd`** (`cmd.h`) — the normalized command value type,
  `fromMove()`.
- **`Motion::Executor`** (`executor.h`) — `configure()`, `enqueue()`,
  `flush()`, `plan()`, `tick(dtMs, measuredDistanceDelta,
  measuredHeadingAbs)`, `popEvent()`, `queueDepth()`/`activeId()`/
  `state()`. See §2b/§2c above for the full contract; `Executor::Twist`'s
  `headingActive`/`thetaRef`/`thetaMeas`/`omegaDes` fields (109-005) are
  what `App::Pilot`'s heading PD cascade consumes.
- **`Motion::kDeadTime`** (`executor.h`, 109-005) — the divergence-replan
  dead-time constant; no live call site yet (ticket 006's own consumer).

### Consumes

- **`msg::PlannerConfig`** (`messages/planner.h`) — `configure()`'s only
  input: `a_max`/`a_decel`/`v_body_max`/`j_max` (linear channel) or
  `yaw_acc_max`/`yaw_rate_max`/`yaw_jerk_max` (rotational channel).
- **`msg::Move`** (`messages/envelope.h`) — `Motion::fromMove()`'s only
  input. `Motion::Cmd`/`Executor` are the only `motion/` types that
  reference a `msg::*` type beyond `PlannerConfig`.
- **`ruckig::Ruckig<1>`/`ruckig::Trajectory<1>`/`ruckig::InputParameter<1>`**
  (`vendor/ruckig/`) — the vendored solver this class wraps; a private
  implementation detail never exposed past this header/`.cpp` pair.

### Consumed by

`App::Pilot` (`app/pilot.{h,cpp}`, 109-003/109-005) is `Motion::Executor`'s
one consumer, driven from `App::RobotLoop`'s own cycle — the root
`src/firm/DESIGN.md` §2 dependency diagram's `app -> motion` edge 109-003
added. `App::HeadingSource` (109-005) is NOT a `motion/` consumer — it is
a separate `app/`-only seam `Pilot` samples and feeds into `Executor::
tick()`'s own `measuredHeadingAbs` parameter as a plain `float`; no new
dependency edge from `motion/` to `app/` or `devices/` is introduced. See
`app/DESIGN.md` for the loop-glue half.

## 6. Open Questions / Known Limitations

- **`solveToState()`'s direction-band interaction is untested for a
  target velocity that opposes the solve's own direction of travel** (e.g.
  requesting a positive `targetVelocity` on a negative-direction solve).
  Ruckig's own `min_velocity`/`max_velocity` band (§3's no-reversal
  invariant) would reject such an input as infeasible, which is the
  correct outcome, but it is the CALLER's job (`Motion::Executor`) to never
  construct such a request — same caller-responsibility boundary as
  `retarget()`/`reanchor()`'s unguarded backward target. `Executor` itself
  still never calls `solveToState()` (109-005's DISTANCE mode uses
  `solveToRest()` — a target-velocity-0 boundary handoff is ticket 006's
  own boundary-velocity-carry scope) — this remains untested until then.
- **Flash budget grew for real at this ticket** (109-001's own "flash-
  neutral until something calls it" note no longer holds — this IS the
  first real call site). `arm-none-eabi-size` after this ticket: FLASH
  used rose from 133072B (35.70%) to 292272B (78.41%) of the 364KB
  region — still comfortably within budget, but a real, expected jump now
  that Ruckig's actual solve code is linked in (no longer dead-code-
  eliminated). Track this number, don't be alarmed by it in isolation.
- **Boundary-velocity carry and divergence replan do not exist yet** —
  109-003 landed TIMED mode + `replace` (§2b); 109-005 added DISTANCE mode
  (kArc/kPivot), the heading PD cascade split with `App::Pilot`, dwell
  completion, and single-command distance-overshoot carry (§2c). Ticket
  006 adds cross-boundary carry (no decel-to-rest at a shared same-`v_max`
  boundary) and the `retarget()`/`reanchor()` divergence-replan triggers
  (`Motion::kDeadTime`'s first consumer); the sprint 109 `sprint.md`
  Architecture section is the forward-looking reference.
- **Dominant-channel-with-slaved-PD accuracy under curved arcs is an
  empirical bet, not derived analytically** (sprint.md's own Open Question
  1) — this ticket's own sim system test
  (`tests/sim/system/test_heading_source.py`) shows near-exact pivot/arc
  landing under the sim's IDEAL (no drift/noise) OTOS after fixing the
  terminal-decel gate to be error-based (§2c), but the SPRINT's own
  decisive 1°-with-drift-enabled/exact-without-it gate is ticket 009's job,
  not this one's — a real bench arc/pivot sweep (this ticket's own
  acceptance criterion) has not been run (USB deploy confirmed broken this
  session).
- **`estimateStopDuration()`'s v1 approximation is scheduling-only, not
  exact-landing-time.** No acceptance criterion in this ticket's testing
  plan asserts a TIMED command lands at rest AT EXACTLY its own deadline —
  only that it IS jerk-limited and DOES complete. A future ticket wanting
  tighter deadline precision would need either a closed-form three-segment
  plan computed at activation time, or a tighter iterative estimate here.
