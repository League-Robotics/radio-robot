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
pivots, §2c) — the sprint's own turn-accuracy motivation; 109-006 added
cross-boundary carry (the "no decel between same-vmax commands" headline
requirement) and the divergence-replan triggers (§2d).

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

**`kDeadTime` (`Motion::kDeadTime`, `executor.h`) is declared, with its
derivation preserved, but STILL has no live call site** — ticket 006's own
`checkDivergence()` (below) is documented as the intended first consumer,
but wiring it in as a `peek(elapsed + kDeadTime)` projection was tried
during this ticket's own implementation and reverted: 130ms is a large
fraction of a typical sub-second pivot/arc's own total duration, so
"where the plan will be 130ms from now" is not a fair stand-in for "where
the plan already is" without a matching measured-transport-lag model on
the OTHER side of the comparison (the sim's own measured signal has no
real transport lag to project past). `checkDivergence()` compares against
the CURRENT elapsed sample instead. Re-derived at the 40ms cycle from
sprint 100's own already-bench-measured `motor_lag` figure (120-140ms,
`architecture-update.md`) rather than hand-picked by scaling the old
120ms/20ms-tick constant onto the new cycle (explicitly disallowed by this
ticket's own semantics) — but NOT itself a fresh bench characterization
(USB deploy was confirmed broken this session; one `mbdeploy probe`
attempt per `hardware-bench-testing.md`'s own escalation path). See
`app/DESIGN.md`'s own "kDeadTime" Open-Questions entry for the full
derivation and the flag for a real re-characterization later.

### 2d. Boundary-velocity carry + replan triggers (109-006)

**`exitSpeed(active, next)` — the "no decel between same-vmax commands"
primitive.** `Executor::computeExitVelocity()` implements the sprint
issue's own formula verbatim, evaluated with ONE-command lookahead only
(`ring_[0]`, never a second successor beyond it), recomputed at `activate()`
and whenever the active's own immediate successor changes
(`maybeRetargetActiveForSuccessorChange()`, triggers (a)/(b)-tail below):

| Condition | `exitSpeed` |
|---|---|
| No queued successor (`ring_[0]` empty) | 0 (decelerate to rest) |
| Successor is TIMED | 0 (TIMED chaining is 109-003's own in-place `solveToVelocity()`/replace path, not this carry) |
| Pivot on either side (an arc chaining into a pivot, or vice versa) | 0 (no shared dominant channel to carry a velocity through) |
| Sign reversal (opposite-signed `distance`/`deltaHeading`) | 0 (decelerates THROUGH zero, never carries a signed velocity across a reversal) |
| Arc → arc, same sign | `min(vmaxEff(active), vmaxEff(next), reachableEntrySpeed(\|next.distance\|))`, linear domain |
| Pivot → pivot, same sign | the SAME rule, evaluated in the rotational domain (`deltaHeading`/`yaw_rate_max`/`yaw_acc_max`/`yaw_jerk_max` in place of `distance`/`v_body_max`/`a_decel`/`j_max`) |

`reachableEntrySpeed(d) = -k + sqrt(k^2 + 2*aDecel*d)`, `k = aDecel^2/(2*jerk)`
(`jerk<=0`, the existing "off" sentinel, collapses to `sqrt(2*aDecel*d)`) —
the fastest speed a channel could enter a `d`-length decel-to-rest segment
at and still reach rest by its own end. The result feeds
`JerkTrajectory::solveToState()`'s own `targetVelocity` argument (`plan()`'s
own kArc/kPivot branch, replacing 109-005's `solveToRest()` call) — Ruckig
itself is what actually avoids decelerating to rest at the boundary; this
class only computes what velocity to ask it to arrive at. Only rest-
terminated pivots still get the dwell landing (109-005's own restriction,
`executor.h`'s own "Dwell completion" comment) — a chained pivot now
genuinely sweeps through its own boundary at nonzero rate instead of
landing and re-accelerating, which is what actually exercises that
restriction with a real non-dwelling handoff.

**Velocity-continuous handoff (trigger (d)).** `activateNextOrIdle()`
no longer unconditionally `reset()`s both channels to zero on a queue
pop — it resets both, then re-seeds ONLY the just-completed command's own
dominant channel (`linear_` for `kArc`, `rotational_` for `kPivot`) from
THIS tick's own last sample (`completionLinearVelocity_`/
`completionLinearAcceleration_` or the rotational twin, refreshed every
`kArc`/`kPivot` tick()) via `JerkTrajectory::seedCurrent(0, velocity,
acceleration)` before `activate(next, retarget=true)`. Harmless when the
completing command's own exit velocity was 0 (its own last sample is
already at, or within epsilon of, rest) — the SAME code path handles both
a full-stop handoff and a velocity-continuous one; only the sampled state
differs.

**Replan triggers (a)-(e):**

| Trigger | What re-solves | Seed |
|---|---|---|
| (a) enqueue adjacent to active, exit speed changes beyond threshold (>1mm/s linear / >0.02rad/s rotational) | in-place `solveToState()` toward the SAME target at the NEW exit velocity | the channel's own remembered last sample (never measured) |
| (b) replace — tail | as (a) (`maybeRetargetActiveForSuccessorChange()` after a tail replace) | as (a) |
| (b) replace — active | in-place `solveToVelocity()` (TIMED) or a full re-`activate()` from the moving state (DISTANCE) — both already `retarget=true` (109-003/109-005) | the channel's own remembered last sample |
| (c) divergence — 5mm retarget (linear only) | `JerkTrajectory::retarget(newRemaining)`, `newRemaining` computed against the MEASURED position | velocity/acceleration from the channel's own remembered state (retarget()'s own contract); the FRAME rebase itself (`linearFrameOffset_`) is set to the measured position, not the channel's own pre-rebase position — see this section's own "Two bugs this ticket caught" below |
| (c) divergence — 40mm/0.3rad reanchor | `JerkTrajectory::reanchor(position, velocity)` | the ONE sanctioned measured-state seed (position AND velocity), acceleration forced to 0 by `reanchor()` itself |
| (d) handoff | `activateNextOrIdle()`'s own velocity-continuous seed (above) | this tick's own last sample |
| (e) STOP | `RobotLoop::handleStop()`'s existing immediate `Drive::stop()` + `Pilot::flush()` (109-003's own established deviation, NOT changed by this ticket — see below) | n/a |

Divergence thresholds (c) are evaluated once per `kArc`/`kPivot` `tick()`
(`checkDivergence()`), comparing the MEASURED channel state against the
dominant channel's own CURRENT-elapsed sampled position (not a
`kDeadTime`-projected one, per this section's own kDeadTime note above) —
`tick()` only ever SETS a pending flag (`pendingLinearReanchor_`/
`pendingLinearRetarget_`/`pendingRotationalReanchor_`); the actual
`retarget()`/`reanchor()` call happens inside the NEXT `plan()` (tick()
"never solves", this file's own boundary). A reanchor is additionally
rate-limited to at most one per `kDivergenceReanchorMinIntervalMs` (60ms).
The 5mm linear retarget tier ALSO requires the divergence to persist for
`kDivergenceRetargetStreakTicks` (3) CONSECUTIVE ticks before acting —
this is NOT part of the sprint issue's own verbatim threshold table; see
"Two bugs this ticket caught" below for why it was added. The rotational
domain has no equivalent small-threshold retarget tier at all — the
heading PD cascade in `App::Pilot` already continuously corrects small
heading drift; only a GROSS (0.3rad) divergence during a pivot warrants
replanning the trajectory itself.

**STOP (trigger (e)) is unchanged, by design, not oversight.** The sprint
issue's own trigger table describes "flush queue + `solveToVelocity(0)`
both channels" for STOP; ticket 003 already established (and this ticket's
own acceptance criteria are silent on changing it) that the wire `STOP`
command's panic-stop path stays an IMMEDIATE `Drive::stop()` for safety,
with `Pilot::flush()` clearing the executor's own queue/active command
alongside it (`RobotLoop::handleStop()`'s own comment) — a graceful
`solveToVelocity(0)` decel is what `Executor`'s OWN internally-triggered
stops (`RAMP_TO_REST`, and 109-006's own `emergencyStopping_` solve-failure
safety net below) use, not the wire STOP path itself.

**Solve failure → `emergencyStopping_` (an edge case, not a formal
acceptance criterion).** A failed solve (any of `solveToState()`/
`solveToVelocity()`/`retarget()`/`reanchor()` returning false) inside
`plan()`'s NORMAL (fresh/continuing) dispatch branch is fatal for the
active command (`completeActive(kSolveFail)`, unchanged from 109-003) —
but this ticket additionally flushes the REST of the ring (each own
`kFlushed`) and drives BOTH channels to `solveToVelocity(0)` before
`activateNextOrIdle()` (`emergencyStopping_`, a dedicated `tick()`
sample-only branch bypassing the normal completion tests entirely) rather
than dropping straight to `kIdle` at whatever velocity was live —
`App::Pilot::tick()` only calls `Drive::setTwist()` while
`state()!=kIdle`, so an immediate `kIdle` transition on solve failure
would otherwise leave `Drive` holding a stale nonzero twist forever. A
failed divergence-triggered `retarget()`/`reanchor()` (as opposed to a
fresh/continuing solve) is deliberately NOT treated as fatal — see "Two
bugs this ticket caught" immediately below.

**`RAMP_TO_REST` accepts a mid-decel enqueue.** `enqueue()`'s own
immediate-activation condition now also covers `state_==kRampToRest` (not
only `kIdle`) when the queue is empty, with `retarget=true` (moving-state
replan, keeps whatever velocity/acceleration seed the decelerating command
still has) instead of `retarget=false` — a TIMED command coasting to rest
with nothing queued behind it no longer has to reach FULL rest before a
newly enqueued command can take over.

**Two bugs this ticket caught (both against `test_heading_source.py`'s own
ideal-plant ACCURACY gate, not a contrived edge case):**

1. **A `+kDeadTime` divergence-comparison lead produced false-positive
   triggers against short pivot/arc trajectories** (this section's own
   kDeadTime note above) — reverted to comparing against the current
   elapsed sample.
2. **A single-tick divergence-retarget reaction to ordinary velocity-PID
   ramp-lag silently regressed heading accuracy.** The real (non-ideal)
   wheel plant's own encoder-measured position routinely lags the
   Ruckig-planned position by a few mm during a command's own ramp-up
   (expected, self-resolving PID tracking behavior, not a fault) —
   reacting to a SINGLE momentary sample past the 5mm threshold rebased
   the linear channel's own frame down to the momentarily-lagging measured
   value with nothing to claw it back, and doing this two or three times
   over a command's own lifetime compounded into a multi-degree heading
   undershoot by completion (caught by the coupled-arc scenario:
   ~2.9deg short of a commanded 45deg arc). Fixed two ways: (a) the frame
   rebase itself must be set to the MEASURED position the `retarget()`
   call's own `newRemaining` was computed against, not the channel's own
   pre-rebase position (getting this backwards reintroduces the exact
   same divergence into `thetaRef` every retarget, since the "corrected"
   frame origin no longer matches what `newRemaining` assumed); (b) the
   5mm tier additionally requires `kDivergenceRetargetStreakTicks` (3)
   CONSECUTIVE ticks past threshold before acting, filtering a transient
   ramp-lag blip from a genuinely sustained divergence. A failed
   divergence `retarget()`/`reanchor()` is also NOT treated as fatal
   (`plan()`'s own dispatch, above) — `JerkTrajectory::solvePositionControl()`
   only ever commits a solve into the held trajectory on success, so a
   failed correction attempt leaves the previous, still-valid trajectory
   completely untouched; silently declining and letting `checkDivergence()`
   re-evaluate next tick is strictly safer than tearing the whole active
   command down over one bad correction attempt.

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
  math itself" boundary). This still holds after 109-006's own boundary-
  velocity/divergence-replan additions (§2d): `tick()` never solves (it
  only DETECTS divergence and sets a pending flag) — every actual
  `solveToState()`/`solveToVelocity()`/`retarget()`/`reanchor()` call
  happens inside `plan()`, still at most one per call. The heading PD
  cascade's own GAINS and ARITHMETIC live in `App::Pilot`, not here (§2c) —
  `Executor` owns nothing beyond the ring/state machine/solve requests/
  completion events/the feedforward+measured-heading bookkeeping §2c/§2d
  describe — no bus access, no wire codec, no `App::Drive` reference, no
  `heading_kp`/`heading_kd` (that is `App::Pilot`'s own boundary).

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
  `state()`. See §2b/§2c/§2d above for the full contract; `Executor::Twist`'s
  `headingActive`/`thetaRef`/`thetaMeas`/`omegaDes` fields (109-005) are
  what `App::Pilot`'s heading PD cascade consumes.
- **`Motion::kDeadTime`** (`executor.h`, 109-005) — the divergence-replan
  dead-time constant; declared, derivation preserved, but STILL no live
  call site (§2d's own kDeadTime note — a naive projection regressed a
  sim accuracy test and was reverted).

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
  `retarget()`/`reanchor()`'s unguarded backward target. `computeExitVelocity()`
  (§2d, 109-006) is the guard that keeps this true in practice — it only
  ever returns a same-sign, reachable exit velocity, or 0 — but this
  remains an invariant `Executor` must uphold, not one `JerkTrajectory`
  itself enforces.
- **Flash budget grew for real at this ticket** (109-001's own "flash-
  neutral until something calls it" note no longer holds — this IS the
  first real call site). `arm-none-eabi-size` after this ticket: FLASH
  used rose from 133072B (35.70%) to 292272B (78.41%) of the 364KB
  region — still comfortably within budget, but a real, expected jump now
  that Ruckig's actual solve code is linked in (no longer dead-code-
  eliminated). Track this number, don't be alarmed by it in isolation.
- **Boundary-velocity carry and divergence replan now exist (109-006, §2d)**
  — 109-003 landed TIMED mode + `replace` (§2b); 109-005 added DISTANCE mode
  (kArc/kPivot), the heading PD cascade split with `App::Pilot`, dwell
  completion, and single-command distance-overshoot carry (§2c); 109-006
  adds cross-boundary carry (no decel-to-rest at a shared same-`v_max`
  boundary) and the `retarget()`/`reanchor()` divergence-replan triggers.
  `Motion::kDeadTime` remains declared but unconsumed (§2d's own note) —
  the dead-time projection this constant was reserved for is deferred to a
  real bench characterization (USB deploy confirmed broken this session).
  A solve-failure fault-bit wire-up (`App::Telemetry`'s own `fault_bits`)
  was considered for the "solve failure" edge case but deferred — it
  requires touching `app/telemetry.h`/`robot_loop.cpp`, outside this
  ticket's own stated file scope (`executor.{h,cpp}`, `motion/DESIGN.md`);
  `Executor`'s own `emergencyStopping_` safety net (§2d) handles the
  behavior-level requirement (flush + decel to rest) without it.
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
