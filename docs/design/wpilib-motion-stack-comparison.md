# WPILib vs. Motion Stack v2 — a source-level comparison

**Date:** 2026-07-12
**Compared against:** sprint 099 (`clasi/sprints/099-restore-pose-estimation-otos-encoders-and-delayed-camera-fixes/architecture-update.md`) and sprint 100 (`clasi/sprints/100-motion-stack-v2-self-contained-stateless-drive-subsystem/`, including the driving issue `issues/motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md`).
**WPILib sources:** `wpilibsuite/allwpilib`, tag `v2026.2.2` (last released form of the classes deleted on main) and branch `main` (2027 dev cycle), read directly — every claim below about WPILib behavior is from the actual source, not from docs or memory. Provenance notes at the end.

The question this document answers: *how does WPILib go from "I am at pose A with velocity v₀; be at pose B with velocity v₁" to motor output, what is visible in that code, and what should we learn from it — better and worse — for motion stack v2?*

**Structure:** §1 how WPILib does it (condensed, with the load-bearing code); §2 concept map; §3 deep agreements; §4 where our design is stronger; §5 lessons L1–L8; §6 ranked recommended actions; Appendices A–C carry the three source-reading memos **in full**, so every claim is traceable to quoted WPILib source without re-fetching anything.

---

## 1. How WPILib commands a pose+velocity → pose+velocity transition

WPILib never solves the free two-point boundary-value problem in SE(2). It
decomposes the problem exactly the way our v2 design does: **the caller supplies
the path geometry; the library solves 1-DOF timing along it; a tracking
controller closes the loop; a separate wheel-velocity servo does the actuation.**
There are two planning entry points and one execution pipeline.

### 1.1 The 1-DOF answer: `TrapezoidProfile` / `ExponentialProfile`

For a single axis, `TrapezoidProfile.calculate(t, current, goal)` is a **pure
function** over `State{position, velocity}` — nonzero initial *and* goal
velocities are first-class. The nonzero-boundary trick is worth seeing: it
extends the profile virtually to zero-velocity endpoints, solves the full
trapezoid, then truncates:

```java
// TrapezoidProfile.java — "Deal with a possibly truncated motion profile
// (with nonzero initial or final velocity) by calculating the parameters as
// if the profile began and ended at zero velocity"
double cutoffBegin = m_current.velocity / m_constraints.maxAcceleration;
double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;
// ... same for the goal end; full trapezoid solved; then:
m_endAccel = accelerationTime - cutoffBegin;
m_endDecel = m_endFullVelocity + accelerationTime - cutoffEnd;
```

The deceleration branch is evaluated **backward from the goal state**, so the
profile lands exactly on `(goal.position, goal.velocity)` — the same guarantee
our Ruckig `solveToExit` gets from `InputParameter::target_velocity`. The
intended usage (per the class javadoc) is *recompute every loop* with a
possibly-moving goal — `ProfiledPIDController` does exactly that in two lines:

```java
m_setpoint = m_profile.calculate(getPeriod(), m_setpoint, m_goal);
return m_controller.calculate(measurement, m_setpoint.position);
```

Note what it advances the profile **from**: the previous *profiled setpoint*,
never the measurement. Feeding the measurement back into the reference would
inject sensor lag and noise into the plan as phantom motion — the same argument
our flying-handoff rule makes ("next plan seeds from the REFERENCE; measured
state gates, never seeds").

`ExponentialProfile` exists because a trapezoid lies about acceleration: a real
voltage-capped motor obeys `v̇ = A·v + B·u` with `A = −kV/kA`, so **achievable
acceleration shrinks as speed rises**. The exponential profile is the
time-optimal profile under that first-order model — bang-bang in voltage with a
closed-form inflection point, decel phase again computed backward from the goal
state. This class is WPILib's institutional memory of the same disease our
sprint calls out as `v_body_max=1000 vs ~620 real plateau`: *plans must be
feasible against the measured actuator envelope, not the nameplate.* See Lesson
L1.

### 1.2 The 2-D answer: user supplies geometry, library supplies timing

`TrajectoryGenerator` (2027: `DrivetrainSplineTrajectoryGenerator`) turns
waypoint **poses** into quintic-hermite (or clamped-cubic) splines, discretizes
by recursive bisection until adjacent points differ by less than (5 in, 0.05 in,
~5°) in twist, then time-parameterizes with the Team 254 / Sprunk (2008)
two-pass algorithm:

- **Forward pass** seeds with `config.startVelocity` and propagates
  `v_f = √(v_i² + 2·a·ds)` point to point, clamping each point's velocity by
  every registered constraint, iterating to a fixed point because acceleration
  limits may themselves depend on velocity.
- **Backward pass** seeds with `config.endVelocity` and clamps each point to
  what can still decelerate into its successor.
- A third loop integrates distances/velocities into timestamps.

Boundary velocities are literally the two pass seeds
(`setStartVelocity`/`setEndVelocity`) — this is how FRC autos chain segments
without stopping, the analogue of our `entrySpeed`/`exitSpeed`.

Constraints are a composable interface — each is a function of
`(pose, curvature, velocity)` returning a max velocity and a min/max
acceleration interval; the parameterizer takes the min over velocities and
intersects the acceleration intervals. Two are load-bearing:

- `DifferentialDriveKinematicsConstraint` — the **joint linear/angular cap**:
  `v → toWheelSpeeds({v, 0, v·κ}) → desaturate(maxWheel) → toChassisSpeeds().vx`.
  Angular velocity is never constrained independently; curvature is the
  coupling variable and the wheel-speed budget binds both. This is exactly our
  `v_eff = min(vBodyMax, omegaMax/|κ|, (vWheelMax − headroom)/(1+|κ|W/2))` fold.
- `DifferentialDriveVoltageConstraint` — bounds **acceleration** by what the
  outer wheel's motor can actually produce at its current speed:
  `a = (V − kS·sgn(v) − kV·v)/kA`, mapped through turn geometry. This is the
  constraint our fold has no analogue of (see L1).

The output is an **immutable, time-indexed** list of samples
`(t, pose, v, a, κ)`; `ω = v·κ` is derived at the consumer, never stored
independently — the same master-DOF rule as ours ("the second channel is
derived ω(t)=κ·v(t) — never independently solved").

A differential trajectory cannot pivot (the spline tangent degenerates); FRC
practice for turn-in-place is a `ProfiledPIDController` on heading — i.e.
WPILib also treats the pivot as a separate 1-DOF profiled-heading problem,
which is precisely our pivot mode and the 098 heading loop it generalizes.

### 1.3 Execution: sample → pose controller → IK → FF + PID → volts

The canonical wiring (`RamseteCommand.execute()`, 20 ms cadence) is a strict
layer cake with a type at every boundary:

```
Trajectory.sample(elapsed)                 (t, pose, v, a, κ)
  → pose controller (Ramsete / LTV)        → ChassisSpeeds (v, ω)
  → DifferentialDriveKinematics            → per-wheel velocity setpoints
  → SimpleMotorFeedforward + wheel PID     → volts
```

The **Ramsete** control law (deprecated 2025, deleted on main — see §1.5), with
error computed in the robot frame:

```java
double k = 2.0 * m_zeta * Math.sqrt(Math.pow(omegaRef, 2) + m_b * Math.pow(vRef, 2));
// v_cmd = v_ref·cos(e_θ) + k·e_x
// ω_cmd = ω_ref + k·e_θ + b·v_ref·sinc(e_θ)·e_y
```

Set `sinc(e_θ) ≈ 1`, freeze `k`, and rename `(e_x, e_y) → (e_along, e_cross)`:
this **is our Kanayama trim law**. Ramsete is the nonlinear, gain-scheduled
member of the same family (Kanayama 1990 is its ancestor); our
`ω_cmd = ω_ref + k_θ·e_θ + k_c·v_ref·e_cross` already carries the one part of
the schedule that matters for stability (cross-track authority scaling with
`v_ref`). The deltas — the `√(ω_ref² + b·v_ref²)` gain schedule and the
`cos(e_θ)` velocity projection — are examined as Lessons L2 and L4.

Its replacement, **`LTVUnicycleController`**, drops the hand-derived law for a
table of LQR gains: at construction it linearizes the unicycle about each
velocity in `[−vmax, vmax]` (0.01 m/s steps), solves a discrete Riccati
equation per point, and at runtime does an interpolated table lookup keyed on
`v_ref`, times the robot-frame error vector. Two details worth noting: the DARE
is ill-conditioned near `v = 0` so the model velocity is clamped to 1e-4 (their
version of our pivot-mode special case), and **dt is baked into the table at
construction** — the controller is welded to the 20 ms loop.

Feedforward is where released WPILib is most sophisticated. The old
`kS·sgn(v) + kV·v + kA·a` form is deprecated; the current form is **exact
discrete plant inversion**:

```java
// SimpleMotorFeedforward.calculateWithVelocities(currentVelocity, nextVelocity)
double A = -kv / ka;  double B = 1.0 / ka;
double A_d = Math.exp(A * m_dt);
double B_d = 1.0 / A * (A_d - 1.0) * B;
return ks * Math.signum(currentVelocity) + 1.0 / B_d * (nextVelocity - A_d * currentVelocity);
```

— "what constant voltage over the next dt moves the discretized plant from
exactly v_k to exactly v_{k+1}." `RamseteCommand` feeds it the **setpoint
sequence** (previous setpoint, current setpoint), never measurement. See L3.

Also present: `ChassisSpeeds.discretize()` — a Lie-group correction (`Pose2d.log`
of the desired one-step pose) for the fact that holding `(v_x, v_y, ω)` constant
over a step sweeps an arc, not the intended chord. For a differential drive
(`v_y ≡ 0`) it is a no-op; it matters for holonomic drives only. Our closed-form
arc reference already *is* the arc — nothing to correct.

Two structural facts about the execution layer deserve emphasis because our
design diverges from them deliberately:

1. **WPILib followers never project onto the path.** Error is
   `poseRef(t) ⊖ measured` — purely time-indexed. A robot that falls behind
   schedule accrues along-track "error" that the controller pushes against, and
   there is **no replan mechanism anywhere**: if the plant can't keep up, the
   reference walks away and PID authority is the only recourse.
2. **Completion is purely temporal.** `RamseteCommand.isFinished()` is
   `m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds())` — the segment ends
   when the clock runs out, *wherever the robot is*, with no pose or velocity
   gate. (Pose tolerance exists on the controllers as a queryable `atReference()`
   but the shipped command ignores it.)

Both are tenable for FRC robots (strong motors, ~10 V headroom budgeted at plan
time, 50 Hz loop with 1 kHz inner loops) and untenable for a laggy brick-servo
plant — this is where our design is genuinely ahead (§4).

### 1.4 Pose: an odometry integrator plus a vision overlay

`Odometry.update(gyroAngle, wheelPositions)` takes **cumulative** encoder
distances and an **absolute** gyro angle, differences internally, converts the
wheel delta to a `Twist2d`, then — the notable move — **discards the
encoder-derived rotation and substitutes the gyro delta**
(`twist.dtheta = angle.minus(m_previousAngle)`), integrates translation via the
SE(2) exponential map, and snaps heading absolutely to the gyro. Heading never
accumulates integration error beyond the gyro's own drift, and trackwidth
cancels out of odometry entirely.

`PoseEstimator.addVisionMeasurement(visionPose, timestamp)` handles **delayed
vision fixes** — sprint 099's exact problem — with a mechanism that is
structurally identical to 099's design:

- A bounded (1.5 s) time-indexed buffer of **odometry-only** poses (the one
  series corrections never touch — the same insight as 099's "recorded from
  `encX_/encY_/encTheta_`, never from `fusedPose`").
- A fix at time T is compared against the estimate *at T* (sampled from the
  buffer), the innovation is scaled by a per-axis gain, and the correction is
  stored as an overlay pair `(visionPose, odometryPose)`:

```java
public Pose2d compensate(Pose2d pose) {
  var delta = pose.minus(this.odometryPose);
  return this.visionPose.plus(delta);   // estimate(t) = fix ⊕ (odom(t) ⊖ odom(T))
}
```

That composition is line-for-line 099's step 3
(`implied.x = fix.x + (encNow.x − enc(T).x)`). The raw integrator is never
rewound or mutated; corrections are re-anchored rigid transforms — same
architecture, independently arrived at.

The gain is not a running covariance but a **closed-form steady-state Kalman
gain** recomputed only when stddevs change: `k = q/(q + √(q·r))` per axis
(derived in `wpimath/algorithms.md` from the Riccati equation with A=0, C=I).
Practice guidance in their docs: trust the gyro over vision heading, scale
vision x/y stddev with distance from the tag.

Two robustness details worth cross-checking against 099 (L5): the innovation is
measured against the **already-compensated** historical estimate, so repeated
fixes converge instead of double-correcting; and inserting a fix at time T
discards any previously-recorded fixes *newer* than T.

### 1.5 Where the ecosystem actually went

The trajectory-following layer described in §1.3 was **deprecated in 2025 and
deleted from the 2027 main branch** — `RamseteController`, `RamseteCommand`,
`SwerveControllerCommand`, `TrapezoidProfileCommand` are gone; the LTV
controllers and the math primitives survive. What competitive teams actually
run in 2024–2026:

- **Generation is external and offline/host-side**: PathPlanner (GUI +
  kinematics/torque-aware generation) and Choreo (full time-optimal NLP,
  solved in a desktop app, exported as sampled files). WPILib's own spline
  generator is effectively legacy.
- **Followers are deliberately dumb**: per-axis PID on pose error plus the
  trajectory's velocity feedforward.
- **Velocity loops run at the motor**: TalonFX/SPARK close velocity PID at
  ~1 kHz on their own encoders; the roboRIO sends setpoints + feedforward at
  50 Hz. `RamseteCommand` even shipped a constructor variant that stops at raw
  wheel-speed setpoints specifically for this topology.
- The 2027 branch's trajectory rework (`Trajectory<SampleType>`,
  JSON/protobuf-serializable samples, `ChassisAccelerations` + acceleration
  FK/IK) is shaped for *consuming externally-generated* trajectories, not for
  owning generation.

The convergent shape — hard planning off the hot loop, simple pure trackers on
it, the velocity servo at the actuator with its own faster clock — is our
architecture: host proxy decomposition (`primitives_for_move()`), closed-form
on-robot planning (Ruckig 1-DOF, not NLP), P-trim tracking, and
`Hal::MotorVelocityPid` at the motor bricks on the I2C flip-flop cadence.

One more independent convergence: the 2027 branch **stripped unit suffixes from
public identifiers** (`vxMetersPerSecond` → `vx`, `trackWidthMeters` →
`trackwidth`), moving units into doc comments — WPILib arrived at this
project's naming rule on its own.

---

## 2. Concept map

| Ours (sprints 099/100) | WPILib equivalent | Notes |
|---|---|---|
| `Drive::Goal{arcLength, deltaHeading, exitSpeed}` | waypoint poses + `TrajectoryConfig{startVelocity, endVelocity}` | both: geometry supplied, not solved |
| `master_profile` / Ruckig `solveToExit` | `TrapezoidProfile`/`ExponentialProfile` calculate-to-goal-state | ours jerk-limited; theirs accel-limited (trapezoid) or voltage-exact (exponential) |
| `MotionPlan` (immutable) + `referenceAt(t)` | `Trajectory` (immutable) + `sample(t)` | ours closed-form continuous; theirs interpolates discrete samples |
| `v_eff` ceiling fold | `DifferentialDriveKinematicsConstraint` + `desaturateWheelSpeeds` | same joint wheel-budget math, both plan-time and run-time |
| — (no analogue) | `DifferentialDriveVoltageConstraint` / `maxAchievableAcceleration(V, v)` | **the gap: acceleration feasibility vs. speed** (L1) |
| Kanayama trim law (`k_s`, `k_θ`, `k_c·v_ref·e_cross`) | Ramsete (`k = 2ζ√(ω_ref²+b·v_ref²)`, sinc) → LTV (velocity-scheduled LQR table) | same family; theirs gain-scheduled (L2) |
| exact arc-frame projection (`e_along`, `e_cross`, `e_θ`) | robot-frame error vs. *time-indexed* reference pose | ours separates schedule slip from geometric error |
| `replan()` (re-time same geometry) + envelopes/sustain/N-max | **nothing** — PID authority or bust | ours is new capability, not a reinvention |
| terminal settle machine, `DONE_STOP` gate (pos+vel, dwelled) | `isFinished() = timer elapsed` | ours state-based; theirs schedule-based |
| flying handoff seeds from reference | `ProfiledPIDController` advances from previous *setpoint*; `RamseteCommand` FF on setpoint pair | same principle, both places |
| `StepState` (caller-owned, 5 scalars) | controller `reset()` methods + command `initialize()` | same philosophy, ours stricter |
| `Hal::MotorVelocityPid` at motor bricks (Level 2) | velocity PID on smart motor controllers @ ~1 kHz | the dominant FRC topology, validated |
| `admit()` typed `Verdict` NACK at the wire | generation-time exceptions; runtime accepts anything | ours is stronger; theirs degrade to do-nothing trajectory + report |
| 099 encoder ring + fix composition + EKF update | `PoseEstimator` odometry buffer + overlay + closed-form gain | near-identical architecture (L5) |
| `EkfTiny` innovation gate on OTOS | per-axis stddev trust (no gate); gyro *overrides* encoder heading | different trust models (L6) |
| `PlanRecord`/`MotionTrace` wire arms, tier-0 replay | 2027 serializable trajectory samples; AdvantageKit log-replay culture | same interpretability instinct |
| `StepInput.t` (caller's clock, argument) | dt baked into controllers/FF/LTV at construction | ours replays; theirs is welded to 20 ms (§4) |

---

## 3. Deep agreements — where WPILib validates the v2 design

These are places the two designs agree on load-bearing structure, which is
worth recording because the agreements were arrived at independently:

1. **Nobody solves the free pose-to-pose boundary problem.** Both systems
   reduce it to: geometry given → one master DOF timed under feasibility
   constraints → `ω = κ·v` derived → tracker closes the loop. Our
   constant-curvature primitive is the low end of the same spectrum whose high
   end is Choreo's NLP; WPILib's own spline generator sits in between and is
   the part of their stack that aged worst.
2. **Boundary velocities are first-class** at both ends of the plan
   (`entrySpeed`/`exitSpeed` ↔ forward/backward pass seeds), and segment
   chaining hands off on the *reference*, not the measurement.
3. **Two levels of control with a velocity-setpoint boundary** is the dominant,
   proven FRC topology — outer pose loop at the slow clock, velocity servo at
   the actuator's own faster clock. Their docs explicitly bless the
   smart-motor-controller variant, which is topologically our motor bricks.
4. **The plan is an immutable value object sampled by elapsed wall-clock time**,
   with the timer owned by the layer above the math (their command,
   our wafer adapter). A dropped tick advances the sample point rather than
   stretching the trajectory in both designs.
5. **Ratio-preserving saturation**: `desaturate()` scales both wheels to
   preserve L/R ratio (hence curvature) at the cost of speed — our
   curvature-preserving saturation is the same commitment: never widen the arc
   under saturation.
6. **Statelessness as the default, small explicit state at the edges.** Their
   profiles are pure `f(t, current, goal)`; controllers hold only diagnostics
   or a scalar or two with explicit `reset()`; run state lives in the command
   and is rebuilt in `initialize()`. Our caller-owned `StepState` is the same
   idea with a stronger guarantee (bit-replayable).
7. **Turn-in-place is a separate 1-DOF heading problem**, not a degenerate
   path — their `ProfiledPIDController`-on-heading idiom is our pivot mode and
   the 098 loop it grew from.
8. **Pose corrections ride an untouched dead-reckoning series** (their
   odometry buffer overlay; our encoder-pose ring composition). See L5.
9. **Feasibility failures degrade loudly and safely** — their generator returns
   a do-nothing trajectory and reports to the driver station rather than
   throwing mid-match; our `admit()` NACKs with a typed verdict and aborts
   surface as `EventNotify`. Ours is more explicit; the instinct is shared.

---

## 4. Where our design is stronger — do not move toward WPILib here

- **State-based termination.** Their shipped follower ends on the clock,
  wherever the robot is; teams patch this in the ecosystem layers. Our
  settle machine (banded one-sided walk-in, pos+vel tolerance held 150 ms,
  literal-zero snap, timeout-with-warning) is a direct answer to a real WPILib
  weakness — and it exists precisely because our plant is laggy and weak where
  theirs is strong. Keep it.
- **Replan exists at all.** WPILib has no mechanism for "the plant fell behind
  the reference"; the reference walks away and PID strains after it. Our
  bounded, rate-limited re-timing of the same geometry is a genuine addition,
  not a reinvention — and the envelope/sustain/N-max policy around it has no
  analogue to copy, so it must earn its keep on the bench.
- **Path-frame error via exact projection.** Time-indexed error (theirs)
  conflates schedule slip with geometric divergence. Exact closed-form
  projection is *only possible because* we restricted geometry to
  constant-curvature arcs — this is the payoff of that restriction, and a
  reason not to envy their splines (you cannot exactly project onto a quintic
  hermite; they never try).
- **Purity and replayability.** `TrackRecord` carrying the full `StepInput` for
  bit-exact offline replay is stronger than anything in WPILib proper (the
  ecosystem bolted this on as AdvantageKit's log-replay). Their controllers
  hide small state (`m_firstRun` latches, mutable `ChassisSpeeds` fields); ours
  structurally cannot.
- **Time as an argument, not configuration.** Their LTV gain table and discrete
  feedforward bake `dt = 20 ms` in at construction — retuning the loop rate
  invalidates objects silently. Our `StepInput.t` + adapter-computed elapsed is
  cleaner and is what makes tier-0 replay possible.
- **Queue-time admission.** Nothing in WPILib checks feasibility of a request
  against the chain tail at enqueue time with a typed reject. `admit()`'s
  joint-step/sign-reversal/radius checks encode hardware protections (the
  encoder-wedge reversal family) WPILib never needed. Keep.
- **Jerk-limited references.** WPILib has no jerk limiting anywhere; FRC
  drivetrains tolerate acceleration steps. Our brick servo + I2C write cadence
  + wedge sensitivity motivated Ruckig, and nothing in their code argues
  against it.
- **Continuous closed-form sampling.** Their `sample(t)` interpolates discrete
  samples with constant-acceleration kinematics; our `referenceAt(t)` evaluates
  the actual solution. No sampling artifacts, no sample-density knob.

---

## 5. Lessons — things WPILib does that we should consider

### L1. Acceleration feasibility against the velocity-servo envelope — the strongest lesson

**What they do:** two mechanisms ensure the reference never demands
acceleration the plant cannot produce: `DifferentialDriveVoltageConstraint`
caps planned acceleration by `a = (V − kS·sgn(v) − kV·v)/kA` *at the faster
wheel, as a function of its current speed*; `ExponentialProfile` bakes the same
first-order model into the profile shape itself. Their parameterizer grew an
inner fixed-point loop specifically because acceleration limits depend on
velocity.

**Our exposure:** the v2 plan fixes the *velocity* ceiling honestly (measured
plateau → `v_wheel_max`, the `v_eff` fold), but `Limits`' acceleration ceiling
is a constant. As seen from Level 1, the plant is a first-order velocity servo
with τ ≈ 130 ms and a plateau: its achievable acceleration at wheel speed `v`
is roughly `(v_plateau − |v|)/τ` — large at rest, **near zero at the plateau**.
A constant `a_max` chosen from low-speed step response will overpromise near
top speed; the reference outruns the plant exactly at cruise, `e_along` grows,
and the replan policy gets exercised for a plan-feasibility sin — the same
class of disease (infeasible plan masked by feedback) this sprint exists to
kill, one derivative up.

**Recommendation (concrete, cheap, fits existing tickets):**
1. Add a tier-0 test — *reference followability*: for every planned reference,
   assert at each sample that the required wheel acceleration is within the
   plant model's achievable envelope at that wheel's speed,
   `|v̇_wheel| ≤ (vPlateau − |v_wheel|)/tau` (+ margin). This is the voltage
   constraint re-expressed as a test instead of a planner constraint — no
   planner change, catches a bad `a_max` choice before the bench does.
   Fits ticket 006 (tier-0 suite) scope.
2. During the ticket-010 plateau re-measure, capture the **acceleration
   envelope** too (the 098-005 drive-arm step method already produces exactly
   this data) and pin `a_max` conservatively against the measured
   high-speed tail, not the from-rest response.
3. If the flat `a_max` proves too conservative at low speed (slow segment
   starts), the principled upgrade is an ExponentialProfile-style
   speed-dependent ceiling in the master profile — file as a future issue, do
   not build now.

### L2. Velocity-scheduled tracker gains

**What they do:** Ramsete's gain `k = 2ζ√(ω_ref² + b·v_ref²)` grows with the
reference rates; LTV replaces even that with per-velocity LQR gains in an
interpolated table (2×3 matrix keyed on `v_ref`, built offline at
construction). The justification for LTV over Ramsete is that fixed-form gains
are "least-squares suboptimal" across the speed range.

**Our position:** the trim law already carries the stability-critical part of
the schedule — `k_c·v_ref·e_cross` scales cross-track authority with speed, and
the design table (`ω_n = v√k_c`, ζ ≥ 1.3 everywhere) shows the damping was
checked across the range. `k_θ = 6` is fixed, but it is hardware-proven at the
worst case (pivot, where `v_ref = 0` kills the scheduled terms entirely).

**Recommendation:** do nothing pre-bench. If the ticket-010 arc grids show
heading tracking degrading specifically at high `v_ref` (loose at speed / tight
at crawl), the cheap adoption is LTV-lite: `k_θ(v)` as a 3–4 point interpolated
table in `tovez.json`, not a DARE solver in firmware. The bench grid should
*record* per-speed tracking quality either way — make sure `arc_sweep.py`'s
output can answer this question per speed bucket (it already sweeps speeds; the
analysis notebook should split gains-fit by speed).

### L3. Discrete plant-inversion feedforward (a Level-2 future, not a v2 change)

**What they do:** `calculateWithVelocities(vNow, vNext)` inverts the
*discretized* motor model (`A_d = e^{AT}`) to compute the exact one-step
voltage, fed with the setpoint sequence. This replaced the continuous
`kS + kV·v + kA·a` form because ZOH control of a continuous plant has
systematic lag the exact discretization removes. It is their main tool for
aggressive tracking — acceleration enters the actuation path through FF, not
through outer-loop gain.

**Our position:** Level 2 (`Hal::MotorVelocityPid`, kFF on velocity) is
bench-proven and explicitly out of scope — the two-levels decision is right and
this does not reopen it. But note what our stack leaves on the table:
`RefState.a` exists in the reference and nothing downstream consumes it; the
wheel setpoint stream is velocity-only, so Level 2 always discovers
acceleration by error. That is fine at current speeds (the settle trade already
accepts slowness over wrongness).

**Recommendation:** file a small issue ("acceleration-aware Level-2
feedforward — WPILib-style discrete plant inversion, kS/kV/kA + e^{AT}") as the
*named upgrade path* if bench acceptance ever shows FF-starved tracking at
speed (symptom: `e_along` proportional to `a_ref`, growing envelope hits during
accel phases specifically, clean at cruise). Do not implement in sprint 100.
The tier-0 plant model, however, **should** use the exponential-discretization
form for its lag model rather than a naive Euler lag — it is two lines and
makes the plant model honest at the 20 ms step size.

### L4. The `cos(e_θ)` velocity projection — examined and (nearly) dismissed

Ramsete/Kanayama command `v_ref·cos(e_θ) + k·e_x`: reference velocity is
projected onto the robot's actual heading, gracefully slowing when pointed
wrong. Our law uses bare `v_ref + clamp(k_s·e_along)`. Inside our replan
envelope (`|e_θ| ≤ 0.15 rad + …`) the factor is ≥ 0.989 — under 1.1% of
`v_ref`, well inside trim authority. It only matters at heading errors the
envelope already classifies as replan-worthy.

**Recommendation:** document the omission as deliberate (this analysis is the
record); optionally add the factor if the tracker is ever revisited — it is one
`cosf` and cannot destabilize (it only ever reduces `v_cmd`) — but it is not
worth touching the proven law for now.

### L5. Pose-estimator cross-checks for sprint 099

099's design is architecturally the same as WPILib's estimator (clean
dead-reckoning series + delta composition onto the fix + tempered application)
— independently reinvented, which is strong validation. Three details from
their source worth checking 099's tickets against:

1. **Repeated-fix convergence.** WPILib measures the innovation against the
   *already-compensated* estimate at the fix time, so a stream of fixes
   converges rather than re-applying overlapping corrections. 099's composition
   builds `implied` from the raw fix plus the clean encoder delta and then
   EKF-tempers it — also convergent (the EKF state carries prior corrections),
   but worth a tier-0 test: **apply the same fix twice; assert the second
   application moves the estimate by strictly less** (and near-zero if the
   fix's noise is honest).
2. **Ordering policy.** WPILib accepts out-of-order fixes within the buffer
   window but *discards recorded fixes newer than the inserted one*. 099
   should state its policy for a fix older than the last-applied fix (camera
   frames can reorder through the relay): apply-if-in-ring is fine, but the
   test matrix should include the out-of-order case explicitly.
3. **Interpolation between ring entries** — theirs is SE(2)-geodesic
   (constant-curvature arc), ours linear-x/y + wrapped-θ between 50 ms
   snapshots. At 50 ms and our speeds the chord error is sub-mm; fine — but
   note it in the code comment so nobody "fixes" it into something slower
   without a reason.

Also worth stealing outright: their guidance to **scale fix noise with
observation distance** (`ekf_r_fix_xy` as a function of camera-to-tag range) —
the aprilcam host script knows the range; the wire already carries per-fix
nothing, but the host can pre-scale before sending. Zero firmware cost.

### L6. Heading source: override vs. blend

WPILib odometry *discards* encoder-derived Δθ and substitutes the gyro's,
unconditionally — when you have a dedicated heading sensor, heading is not
fused, it is taken. We blend OTOS heading through the gated EKF instead. The
difference is defensible (their gyros are drift-characterized MEMS with no
failure mode resembling the OTOS's tracking loss; our gate exists because the
OTOS *does* momentarily disagree), but it frames a question the 099/100 bench
work should answer empirically: once the OTOS rejection gate exists, is fused
heading actually better than OTOS-primary heading with encoder fallback? The
098 pivot re-run (ticket 012 gates the encoder→EKF heading-source switch)
is exactly the right experiment — this comparison just says: treat
"OTOS-primary, encoder-fallback" as a live candidate, not a fallback, when
reading those results.

**Update (2026-07-12):** the stakeholder proposed a concrete design for this —
a gated switch (OTOS heading verbatim when its per-interval delta passes a
rate-limit check and a cross-sensor agreement check; otherwise encoder deltas
rebased onto the last trusted OTOS heading), filed as
[`clasi/issues/heading-source-arbiter-otos-primary-with-encoder-delta-fallback.md`](../../clasi/issues/heading-source-arbiter-otos-primary-with-encoder-delta-fallback.md)
for decision before sprint 099 executes.

### L7. Constraint composability — a shape to remember, not to build

Their `TrajectoryConstraint` interface (max velocity + accel interval as a
function of `(pose, κ, v)`, composed by min/intersection, including
*region-scoped* constraints like `RectangularRegionConstraint`) is the proven
shape for "slow down near the field wall / in the scoring zone" — an
on-the-path cousin of our geofence rules. Our `Limits` is a flat struct and
should stay one for v2. If a future sprint wants region-scoped speed caps on
the playfield, this is the reference design: constraints as pure functions
folded at plan time, not runtime checks.

### L8. SE(2) exponential-map integration — minor exactness note

Their odometry integrates via `Twist2d.exp` (exact under constant twist, with
Taylor guards near θ=0); our dead-reckoning uses midpoint-heading integration
(`cosf/sinf(encThetaMid)`), which is the second-order approximation of the same
arc — error O(dθ³) per step, negligible at our tick rates. No change
warranted; noted so the equivalence is on record. Our `arc_math`'s
`poseAlongArc` is already the exact closed form on the reference side, where it
matters more.

---

## 6. Recommended actions, ranked

| # | Action | Where it lands | Cost |
|---|---|---|---|
| 1 | Tier-0 **reference-followability test**: planned wheel accel ≤ plant-model envelope `(vPlateau − |v|)/tau` at every sample (L1) | ticket 006 (tier-0 suite) acceptance | small |
| 2 | Measure the **acceleration envelope** during the plateau re-measure; pin `a_max` against the high-speed tail (L1) | ticket 010 (bench) procedure | small |
| 3 | Tier-0 plant model uses **exponential discretization** for the lag stage, not Euler (L3) | ticket 006 | trivial |
| 4 | `arc_sweep.py` analysis splits tracking quality **per speed bucket** so an L2-style `k_θ(v)` schedule decision is data-driven (L2) | tickets 010/011 notebooks | small |
| 5 | 099 tier-0 tests: **repeated-fix convergence** + **out-of-order fix** cases (L5) | sprint 099 test tickets | small |
| 6 | Host-side: **scale `PoseFix` noise with camera-to-tag distance** before sending (L5) | 099/100 field scripts | small |
| 7 | Record the deliberate omission of `cos(e_θ)` (L4) and the midpoint-vs-exp integration equivalence (L8) in code comments at the trim law / dead-reckoning sites | tickets 004 / 099 | trivial |
| 8 | File future issues (not sprint 100): acceleration-aware Level-2 feedforward (L3); speed-dependent `a_max` ceiling (L1); region-scoped limits shape (L7) | `clasi/issues/` | paperwork |
| 9 | Read the 098-pivot re-run results with "OTOS-primary heading" as a live candidate, not a fallback (L6) | ticket 012 review | judgment |

And, explicitly, the **do-not-copy list**: time-based completion, absence of
replan, dt baked into constructed objects, on-robot spline generation, hidden
controller state. Each is a place WPILib's own trajectory (deprecation of the
whole follower layer, ecosystem migration to external generators and logged
replay) suggests they would agree.

---

## 7. Provenance

Researched 2026-07-12 by three parallel source-reading agents against
`wpilibsuite/allwpilib`:

- **Profiles/trajectories** (branch `main`): `TrapezoidProfile.java`,
  `ExponentialProfile.java`, `DrivetrainSplineTrajectoryGenerator/Parameterizer`
  (née `TrajectoryGenerator`/`TrajectoryParameterizer`, Team 254 lineage,
  Sprunk 2008), `SplineHelper`, `trajectory/constraint/*`,
  `ProfiledPIDController.java`.
- **Controllers/pipeline** (tag `v2026.2.2`, since `main` deleted them):
  `RamseteController.java`, `LTVUnicycleController.java`,
  `HolonomicDriveController.java`, `ChassisSpeeds.java`,
  `DifferentialDriveKinematics.java`, `SimpleMotorFeedforward.java`,
  `DifferentialDriveFeedforward.java`, `RamseteCommand.java`,
  `TimedRobot.java`, `CommandScheduler.java`, plus docs.wpilib.org for the
  smart-motor-controller topology.
- **Odometry/estimation/architecture** (branch `main` + `v2025.3.2`):
  `Odometry.java`, `PoseEstimator.java`, `TimeInterpolatableBuffer.java`,
  `Pose2d/Twist2d/Transform2d` exp/log, `DifferentialDrivetrainSim.java`,
  `wpimath/algorithms.md`, plus pathplanner.dev / Choreo / AdvantageKit docs
  for ecosystem practice.

Naming caveat: the `main` branch is the 2027 dev cycle (`edu.wpi.first.*` →
`org.wpilib.*`, `ChassisSpeeds` → `ChassisVelocities`, etc.); released WPILib
that teams run today matches the `v2026.2.2` names. Algorithms are unchanged
across the rename.

The three agents' full source-reading memos are preserved verbatim below as
Appendices A–C, so every claim in §1–§6 can be traced to quoted source without
re-fetching WPILib.

---

## Appendix A — WPILib motion-profile & trajectory generation (source memo)

**Provenance:** all code fetched 2026-07-12 from `wpilibsuite/allwpilib` branch
`main` via the GitHub API (session scratchpad copies; not checked in).

**Naming note (important for cross-referencing docs):** the `main` branch is
the 2027 dev cycle and has renamed things relative to released WPILib
(2024–2026) — package `edu.wpi.first.math` → `org.wpilib.math`;
`TrajectoryGenerator` → `DrivetrainSplineTrajectoryGenerator`;
`TrajectoryParameterizer` → `DrivetrainSplineTrajectoryParameterizer`;
`Trajectory.State` → `DrivetrainSplineSample` (now extends `HolonomicSample`);
`ChassisSpeeds` → `ChassisVelocities`; `DifferentialDriveWheelSpeeds` →
`DifferentialDriveWheelVelocities`; static
`DifferentialDrive.desaturateWheelSpeeds` pattern → instance method
`DifferentialDriveWheelVelocities.desaturate()`. The algorithms are unchanged
(the parameterizer still carries the Team 254 MIT header and cites Sprunk
2008). Everything below quotes the main-branch source.

**Big picture — two disjoint planning layers:**

1. **1-DOF profiles** (`TrapezoidProfile`, `ExponentialProfile`):
   stateless-ish `calculate(t, current, goal)` functions over
   `State{position, velocity}`. Designed to be **recomputed every loop from
   the previous setpoint toward a freely-moving goal** (receding horizon).
   This is WPILib's literal answer to "go from current position+velocity to
   goal position+velocity" for one axis.
2. **2-D spline trajectories**: waypoint poses → hermite splines → arc-length
   point cloud (`PoseWithCurvature`) → forward/backward-pass time
   parameterization under pluggable constraints → a time-indexed list of
   samples `(t, pose, velocity, acceleration, curvature)`. Precomputed once,
   then tracked by a follower (Ramsete/LTV + wheel-level PID). Nonzero
   start/end velocities are first-class via
   `TrajectoryConfig.setStartVelocity/setEndVelocity`.

### A.1 `TrapezoidProfile` (trajectory/TrapezoidProfile.java, 303 lines)

**(a) Role.** 1-DOF pose+velocity → pose+velocity planner. Given
`Constraints{maxVelocity, maxAcceleration}` and two `State{position, velocity}`
endpoints, computes where the constrained reference should be after time `t`.
Class javadoc: *"While this class can be used for a profiled movement from
start to finish, the intended usage is to filter a reference's dynamics based
on trapezoidal velocity constraints"* — i.e. call it every loop with
`previousProfiledReference` and the raw goal; *"when the unprofiled reference
is within the constraints, calculate() returns the unprofiled reference
unchanged."*

**(b) Key API.**
```java
public TrapezoidProfile(Constraints constraints)
public State calculate(double t, State current, State goal)  // current is at t=0
public double timeLeftUntil(double target)
public double duration()            // returns m_endDecel
public boolean isFinished(double t) // t >= duration()
```
`State` is just `{public double position; public double velocity;}` (plus
struct serialization and value equality). `Constraints` throws
`IllegalArgumentException` if either bound is negative. Units are
caller-defined but must be consistent (docs use meters/seconds); time in
seconds.

**(c) Algorithm — the full core of `calculate()`:**

Direction-flip trick: the math is written for goal ≥ start only; the other
direction is handled by mirroring both states through the origin:
```java
m_direction = shouldFlipAcceleration(current, goal) ? -1 : 1;   // initial.position > goal.position
m_current = direct(current);   // multiplies position AND velocity by m_direction
goal = direct(goal);

if (Math.abs(m_current.velocity) > m_constraints.maxVelocity) {
  m_current.velocity = Math.copySign(m_constraints.maxVelocity, m_current.velocity);
}
```

Nonzero endpoint velocities — the truncated-profile trick. It extends the
profile virtually backward/forward to zero-velocity endpoints, computes a
*full* trapezoid, then cuts off the ends:
```java
// Deal with a possibly truncated motion profile (with nonzero initial or
// final velocity) by calculating the parameters as if the profile began and
// ended at zero velocity
double cutoffBegin = m_current.velocity / m_constraints.maxAcceleration;
double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

double cutoffEnd = goal.velocity / m_constraints.maxAcceleration;
double cutoffDistEnd = cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

double fullTrapezoidDist =
    cutoffDistBegin + (goal.position - m_current.position) + cutoffDistEnd;
double accelerationTime = m_constraints.maxVelocity / m_constraints.maxAcceleration;

double fullVelocityDist =
    fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

// Handle the case where the profile never reaches full velocity  (triangle profile)
if (fullVelocityDist < 0) {
  accelerationTime = Math.sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
  fullVelocityDist = 0;
}

m_endAccel = accelerationTime - cutoffBegin;                                  // phase-boundary times
m_endFullVelocity = m_endAccel + fullVelocityDist / m_constraints.maxVelocity;
m_endDecel = m_endFullVelocity + accelerationTime - cutoffEnd;
```

Then a three-branch piecewise evaluation at time `t` (accelerate / cruise /
decelerate / done), with the decel branch computed **backward from the goal
state** so the profile lands exactly on `goal.position, goal.velocity`:
```java
State result = new State(m_current.position, m_current.velocity);
if (t < m_endAccel) {
  result.velocity += t * m_constraints.maxAcceleration;
  result.position += (m_current.velocity + t * m_constraints.maxAcceleration / 2.0) * t;
} else if (t < m_endFullVelocity) {
  result.velocity = m_constraints.maxVelocity;
  result.position +=
      (m_current.velocity + m_endAccel * m_constraints.maxAcceleration / 2.0) * m_endAccel
          + m_constraints.maxVelocity * (t - m_endAccel);
} else if (t <= m_endDecel) {
  result.velocity = goal.velocity + (m_endDecel - t) * m_constraints.maxAcceleration;
  double timeLeft = m_endDecel - t;
  result.position =
      goal.position - (goal.velocity + timeLeft * m_constraints.maxAcceleration / 2.0) * timeLeft;
} else {
  result = goal;
}
return direct(result);   // un-mirror
```

`timeLeftUntil(target)` re-derives phase distances and solves the kinematic
quadratics (`t = (-v + √(v² + 2ad))/a`) per phase; it clamps phase distances
when the target lands mid-phase.

**(d) Design decisions.**
- **Functionally stateless planning:** `calculate()` takes current and goal
  explicitly; the small cached fields (`m_direction`, `m_endAccel`…) only
  exist so `timeLeftUntil`/`duration`/`isFinished` can be queried after the
  fact. There is no accumulated integration state — which is exactly what
  makes per-loop recompute safe.
- **No "timeout"/duration parameter in:** duration falls out of the math
  (`m_endDecel`); `isFinished(t)` is just `t >= duration()`.
- **Limitations (implicit in the math):** `shouldFlipAcceleration` compares
  positions only, so an overshoot-required case (e.g. moving toward the goal
  faster than it can stop, or goal velocity pointing away) is not solved
  optimally — the profile family is accel/cruise/decel only. Initial velocity
  is silently clamped to `maxVelocity`. No jerk limiting.
- Only nonnegative-constraints validation; everything else is unchecked
  doubles.

### A.2 `ExponentialProfile` (trajectory/ExponentialProfile.java, 472 lines)

**(a) Role & problem it solves vs trapezoid.** A trapezoid assumes a constant
achievable acceleration, but a real DC-motor-driven mechanism under a voltage
cap obeys `dx/dt = v; dv/dt = A·v + B·u` with `A = -kV/kA`, `B = 1/kA` —
achievable acceleration *shrinks as speed rises*. `ExponentialProfile`
computes the time-optimal profile for that first-order model under an input
(voltage) limit, so the reference is followable at max effort at every speed.
Constraints are built directly from characterization constants:

```java
public static Constraints fromCharacteristics(double maxInput, double kV, double kA) {
  return new Constraints(maxInput, -kV / kA, 1.0 / kA);
}
public double maxVelocity() { return -maxInput * B / A; }   // steady-state speed = maxInput/kV
```

**(b) API mirrors TrapezoidProfile:** `State{position, velocity}`,
`calculate(double t, State current, State goal)`,
`timeLeftUntil(current, goal)`, `calculateProfileTiming(...)` returning
`ProfileTiming{inflectionTime, totalTime}`.

**(c) Algorithm.** Bang-bang in voltage: apply `u = ±maxInput` until an
**inflection point**, then `∓maxInput` until arriving exactly at the goal
state. Everything is closed-form off the exponential solution of the linear
ODE:
```java
// position under constant input u from `initial`:
return initial.position
    + (-B * u * t + (initial.velocity + B * u / A) * (Math.exp(A * t) - 1)) / A;
// velocity:  (v0 + Bu/A) e^{At} − Bu/A
// time to reach velocity v:  log((A v + B u)/(A v0 + B u)) / A
```
The inflection velocity comes from `solveForInflectionVelocity()`, which
intersects the forward (+u) and backward (−u) phase-plane curves:
```java
var scalar = (A * current.velocity + B * u) * (A * goal.velocity - B * u);
var power = -A / B / u * (A * position_delta - velocity_delta);
var a = -A * A;
var c = (B * B) * (u * u) + scalar * Math.exp(power);
if (-1e-9 < c && c < 0) { return 0; }   // numerical-stability guard, c ~ -1e-13
return U_dir * Math.sqrt(-c / a);
```
`calculate(t)` then evaluates: before inflection → forward dynamics from
`current`; after → **backward in time from `goal`**
(`computeDistanceFromTime(t - timing.totalTime, -u, goal)`), guaranteeing
exact endpoint velocity; past `totalTime` → goal. Direction choice
(`shouldFlipInput`) is a phase-plane region test comparing goal position
against the forward/reverse switching curves (`x_forward`, `x_reverse`), with
special cases when `|v0| ≥ maxVelocity`. `calculateProfileTiming`
special-cases inflection velocity within 1e-9 of ±maxVelocity (the
exponential never quite reaches steady state — time-to-velocity would be log
of ~0), nudging by epsilon and covering the cruise segment at `maxVelocity`.

**(d) Design decisions.** Same recompute-every-loop contract as trapezoid;
used by `ProfiledPIDSubsystem`-style code where gravity/back-EMF matters
(elevators, flywheel position). No cruise phase in general — the velocity
asymptotically approaches `maxInput/kV`; can pair with a lower velocity cap by
wrapping. Negative `t` returns `current` unchanged.

### A.3 Spline trajectory pipeline: `DrivetrainSplineTrajectoryGenerator` + `SplineHelper`/`SplineParameterizer` + `DrivetrainSplineTrajectoryParameterizer`

(Released-WPILib names: `TrajectoryGenerator` + `TrajectoryParameterizer`.)

#### A.3.1 Geometry generation (`...Generator.generate(...)` overloads)

**(a) Role.** Turn waypoints into a dense, geometry-only list of
`PoseWithCurvature{Pose2d pose; double curvature;}`, then hand off to the time
parameterizer with the config's velocity bookends:
```java
return DrivetrainSplineTrajectoryParameterizer.parameterize(
    points, config.getConstraints(),
    config.getStartVelocity(), config.getEndVelocity(),
    config.getMaxVelocity(), config.getMaxAcceleration(), config.isReversed());
```

**(b) Two spline modes:**
- **Clamped cubic** — `generate(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, config)`:
  only endpoint headings are specified; interior headings are solved
  automatically for curvature continuity.
  `SplineHelper.getCubicSplinesFromControlVectors` sets up a **tridiagonal
  system for the interior first derivatives** (diagonal all 4.0, off-diagonals
  1.0, RHS `3*(x_{i+2} − x_i)` clamped by the endpoint tangents) and solves it
  with the **Thomas algorithm** — the classic clamped-cubic-spline
  construction (source cites a UiO numerical-analysis chapter).
- **Quintic hermite** — `generate(List<Pose2d> waypoints, config)`: every
  waypoint is a full pose. Each adjacent pair becomes a
  `QuinticHermiteSpline` from control vectors `(x, x', x'')` per axis, with
  tangent magnitude chosen heuristically: `scalar = 1.2 * distance(p0, p1)`
  (comment: *"This just makes the splines look better."*) and second
  derivative initialized to 0. Then `SplineHelper.optimizeCurvature`
  (Sprunk 2008 §4.1.2) replaces the zero second derivatives at each interior
  knot with a **distance-weighted average of the second derivatives of
  equivalent cubic splines** (`alpha = dBC/(dAB+dBC)`,
  `ddx = alpha*ddxA + beta*ddxB`), reducing ∫|x''|.

Spline evaluation (`Spline.getPoint(t)`): polynomial + derivatives via a
coefficients matrix; heading is `new Rotation2d(dx, dy)`, and
```java
final double curvature = (dx * ddy - ddx * dy) / ((dx * dx + dy * dy) * Math.hypot(dx, dy));
```
Returns `Optional.empty()` if `hypot(dx,dy) < 1e-6` (degenerate tangent).

**(c) Arc-length discretization** (`SplineParameterizer.parameterize`):
recursive bisection with an explicit stack. A segment `[t0,t1]` is accepted
only when the relative pose twist is small enough:
```java
private static final double kMaxDx = 0.127;      // 5 inches
private static final double kMaxDy = 0.00127;    // 0.05 inch
private static final double kMaxDtheta = 0.0872; // ~5 degrees
...
final var twist = (end.get().pose.minus(start.get().pose)).log();
if (|twist.dy| > kMaxDy || |twist.dx| > kMaxDx || |twist.dtheta| > kMaxDtheta) { split in half }
```
Hard iteration cap `kMaxIterations = 5000` → `MalformedSplineException` (*"you
probably had two or more adjacent waypoints that were very close together with
headings in opposing directions"*). The generator catches this, reports via an
injectable error handler (`setErrorHandler`, default
`MathSharedStore.reportError` = DriverStation), and returns a **"do nothing
trajectory"** (a single zeroed sample) rather than throwing — a
robot-safety-flavored error policy.

**Reversed driving:** handled by flipping every waypoint by
`Transform2d(0, π)` (and negating tangent components of control vectors)
before spline generation, then flipping poses back and negating curvature
afterward; the parameterizer later negates velocity/acceleration signs.

#### A.3.2 Time parameterization (`DrivetrainSplineTrajectoryParameterizer.parameterize`, Team 254-derived, algorithm from Sprunk 2008)

**(a) Role.** Assign to each geometric point a max feasible velocity and
consistent accelerations, then integrate to timestamps. Each point carries a
working `ConstrainedState {PoseWithCurvature pose; double distance;
maxVelocity; minAcceleration; maxAcceleration;}`.

**(b) Forward pass** — seeds with `startVelocity` (nonzero endpoint velocity
is literally the seed of the pass):
```java
var predecessor = new ConstrainedState(points.get(0), 0, startVelocity, -maxAcceleration, maxAcceleration);
for (int i = 0; i < points.size(); i++) {
  ...
  while (true) {
    // Enforce global max velocity and max reachable velocity by global
    // acceleration limit. v_f = √(v_i² + 2ad).
    constrainedState.maxVelocity = Math.min(maxVelocity,
        Math.sqrt(predecessor.maxVelocity² + predecessor.maxAcceleration * ds * 2.0));
    constrainedState.minAcceleration = -maxAcceleration;
    constrainedState.maxAcceleration = maxAcceleration;

    for (final var constraint : constraints) {
      constrainedState.maxVelocity = Math.min(constrainedState.maxVelocity,
          constraint.getMaxVelocity(pose, curvature, constrainedState.maxVelocity));
    }
    enforceAccelerationLimits(reversed, constraints, constrainedState);
    if (ds < 1E-6) break;

    double actualAcceleration = (v_i² − v_{i−1}²) / (2 ds);
    if (constrainedState.maxAcceleration < actualAcceleration - 1E-6) {
      predecessor.maxAcceleration = constrainedState.maxAcceleration;  // tighten predecessor, retry
    } else { ...; break; }
  }
  predecessor = constrainedState;
}
```
Note the inner `while(true)`: because **acceleration limits may be functions
of velocity** (e.g. voltage constraint), lowering a velocity changes the
allowed acceleration, so it iterates to a fixed point, propagating tightened
acceleration back to the predecessor.

**(c) Backward pass** — seeds a virtual successor with `endVelocity` and
sweeps in reverse, clamping each point's velocity to what can still decelerate
into its successor (`ds` negative, using `successor.minAcceleration`):
`newMaxVelocity = √(successor.maxVelocity² + successor.minAcceleration * ds * 2.0)`;
if the current state's min-acceleration is violated it tightens the
successor's `minAcceleration` and retries — the comment in the forward pass
says under-achievable decels *"will be repaired in the backward pass."*

**(d) Time integration.** Third loop converts distances/velocities to times
with `a = (v_f² − v_i²)/(2 ds)` per segment and `dt = (v_f − v_i)/a` (or
`ds/v` when a ≈ 0); if both `a` and `v` are ~0 mid-trajectory it throws
`TrajectoryGenerationException("Something went wrong at iteration i of time
parameterization.")`. A recent main-branch refinement: a sample's stored
acceleration is the acceleration of the segment *leaving* it
(`segAccel[i+1]`), final sample reuses its incoming one.

**(e) The output sample** (released: `Trajectory.State{timeSeconds,
velocityMetersPerSecond, accelerationMetersPerSecondSq, poseMeters,
curvatureRadPerMeter}`; main: `DrivetrainSplineSample`) contains **time, pose,
velocity, acceleration, curvature**. On main, velocity/acceleration are stored
as field-relative `ChassisVelocities`/`ChassisAccelerations`, constructed from
the path-tangential scalars as
`new ChassisVelocities(velocity, 0.0, velocity * curvature).toFieldRelative(pose.getRotation())`
— i.e. angular velocity `ω = v·κ` is derived, and `forwardVelocity()` projects
back to the scalar. Trajectory sampling between stored samples
(`DrivetrainSplineTrajectory.interpolate`) uses constant-acceleration
kinematics (`v_f = v_0 + at`, `Δs = v_0 t + ½at²`) and interpolates the pose
along the chord by `Δs / chordLength`.

**(f) Endpoint velocities are first-class:** `TrajectoryConfig` stores
`m_startVelocity`/`m_endVelocity` (default 0) with
`setStartVelocity(double | LinearVelocity)` / `setEndVelocity(...)` builder
setters, flowing directly into the two pass seeds. This is how multi-segment
autos chain trajectories without stopping. `TrajectoryConfig` also has
typed-units overloads throughout (`LinearVelocity.in(MetersPerSecond)`),
fixing the trajectory layer's units to SI meters/seconds (unlike the
unit-agnostic 1-DOF profiles).

### A.4 `trajectory/constraint/` — the constraint interface and implementations

**(a) Interface** — per-point, called with pose, curvature, and the tentative
velocity:
```java
public interface TrajectoryConstraint {
  double getMaxVelocity(Pose2d pose, double curvature, double velocity);      // absolute max, m/s
  MinMax getMinMaxAcceleration(Pose2d pose, double curvature, double velocity);
  class MinMax { double minAcceleration = -Double.MAX_VALUE; double maxAcceleration = Double.MAX_VALUE; }
}
```
Constraints compose by `min()` over velocities and interval-intersection over
accelerations (`enforceAccelerationLimits` takes `max` of mins, `min` of
maxes; when reversed it evaluates at `velocity * factor` with `factor = -1`
and swaps/negates the bounds). If a constraint returns `min > max`, the
parameterizer throws
`TrajectoryGenerationException("Infeasible trajectory constraint: <class>")`.

**(b) `CentripetalAccelerationConstraint`** — velocity-only:
```java
// ac = v²k  =>  v = √(ac/k)
return Math.sqrt(m_maxCentripetalAcceleration / Math.abs(curvature));
```
`getMinMaxAcceleration` returns an unbounded `MinMax` (tangential accel
doesn't affect centripetal accel). Effect: slows the robot through tight
turns.

**(c) `DifferentialDriveKinematicsConstraint`** — velocity-only, and **yes,
this is exactly the joint linear/angular coupling**. It converts the chassis
state at each point (`v`, `ω = v·curvature`) into wheel speeds via inverse
kinematics, desaturates, and converts back:
```java
var chassisVelocities = new ChassisVelocities(velocity, 0, velocity * curvature);
var wheelVelocities = m_kinematics.toWheelVelocities(chassisVelocities).desaturate(m_maxVelocity);
return m_kinematics.toChassisVelocities(wheelVelocities).vx;
```
With `toWheelVelocities` being `left/right = vx ∓ (trackwidth/2)·ω` and
`desaturate` scaling both wheels by `attainableMax / max(|left|,|right|)`
(ratio-preserving), the returned chassis `vx` is reduced precisely so that the
**outer wheel** on a curve stays under the wheel-speed cap. So angular
velocity is not constrained independently — it is constrained jointly with
linear velocity through the wheel-speed budget, and the same
`toWheelSpeeds → desaturate → toChassisSpeeds` pattern is reused at runtime
for teleop/follower outputs. `TrajectoryConfig.setKinematics(DifferentialDriveKinematics)`
is nothing but sugar for
`addConstraint(new DifferentialDriveKinematicsConstraint(kinematics, m_maxVelocity))`
(mecanum and swerve overloads add the analogous 4-wheel constraints).

**(d) `DifferentialDriveVoltageConstraint`** — acceleration-only
(`getMaxVelocity` returns `Double.POSITIVE_INFINITY`). Uses
`SimpleMotorFeedforward.maxAchievableAcceleration(maxVoltage, wheelVelocity)`
(i.e. `a = (V − kS·sgn(v) − kV·v)/kA`) at the **faster wheel**, and
min-achievable at the slower wheel, then maps wheel accel to chassis accel
through turn geometry:
```java
// Achassis = Aouter / (1 + |curvature|·T/2)   (outer wheel radius = 1/|k| + T/2)
maxChassisAcceleration = maxWheelAcceleration
    / (1 + m_kinematics.trackwidth * Math.abs(curvature) * Math.signum(velocity) / 2);
```
with documented special cases for `velocity == 0` (signum breaks; both bounds
reduced in magnitude) and turning about a point **inside** the wheelbase
(`T/2 > 1/|k|` — inner wheel reverses direction, so the corresponding bound is
negated). This is the constraint that makes trajectories physically followable
at a stated voltage headroom (docs recommend ~10 V of a 12 V battery). Note
the velocity-dependence of these bounds is what forces the parameterizer's
inner fixed-point loop.

Also in the package: `MaxVelocityConstraint` (flat cap),
`Elliptical/RectangularRegionConstraint` (apply another constraint only inside
a field region), `MecanumDriveKinematicsConstraint`,
`SwerveDriveKinematicsConstraint`.

### A.5 `ProfiledPIDController` (controller/ProfiledPIDController.java, 469 lines)

**(a) Role.** Marries layer 1 to feedback: *"a PID control loop whose setpoint
is constrained by a trapezoid profile."* The user sets a `goal` (position, or
full `TrapezoidProfile.State` with nonzero goal velocity); every loop the
controller advances an internal profiled **setpoint** one period along a
freshly computed profile and runs plain PID against that moving setpoint.

**(b) Key API.** `ProfiledPIDController(Kp, Ki, Kd, TrapezoidProfile.Constraints, period=0.02)`;
`setGoal(State|double)`, `calculate(measurement[, goal[, constraints]])`,
`getSetpoint()`, `atGoal()` (= `atSetpoint() && m_goal.equals(m_setpoint)`),
`reset(State|position,velocity)`, `enableContinuousInput(min,max)`.

**(c) The receding-horizon core** — the entire profile "regeneration" is these
two lines at the bottom of `calculate(double measurement)`:
```java
m_setpoint = m_profile.calculate(getPeriod(), m_setpoint, m_goal);
return m_controller.calculate(measurement, m_setpoint.position);
```
The profile is recomputed **every iteration** from the previous *profiled
setpoint* (not the measurement) toward the goal, advanced by exactly one
controller period, and stored back. Because `TrapezoidProfile.calculate` is a
pure function of `(t, current, goal)`, the goal can change freely between
calls and the setpoint trajectory stays kinematically consistent (velocity
continuity is preserved through `m_setpoint.velocity`). PID then servos the
plant onto `m_setpoint.position`; typical usage adds feedforward from
`getSetpoint().velocity` externally.

Continuous-input (heading wrap) handling rewrites both goal and setpoint into
the measurement's neighborhood before profiling:
```java
double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
double goalMinDistance = MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
double setpointMinDistance = MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);
// "the setpoint only needs to be offset from the measurement by the input range modulus"
m_goal.position = goalMinDistance + measurement;
m_setpoint.position = setpointMinDistance + measurement;
```

**(d) Design decisions.** `reset(measurement)` is mandatory at enable ("Users
should call reset() when they first start running the controller") — it snaps
`m_setpoint` to the measured state so the profile starts from reality (this is
where current *velocity* enters). Because tracking runs off the internal
setpoint rather than the measurement, large disturbances produce PID
correction, not profile re-planning — replanning from measurement is possible
by calling `reset()` each loop but is not the default. `setConstraints`
rebuilds the `TrapezoidProfile` on the fly (also exposed live via Sendable
dashboard properties for `maxVelocity`/`maxAcceleration`). C++/newer Java also
ship `ProfiledPIDController`-equivalents using `ExponentialProfile` via the
same two-line pattern.

### A.6 Direct answer to the differential-kinematics question

Angular and linear velocity are constrained **jointly**, in three places, all
through the same wheel-space transform (`DifferentialDriveKinematics`,
main-branch names):

```java
// inverse kinematics
toWheelVelocities(c) = { left: c.vx − (trackwidth/2)·c.omega,  right: c.vx + (trackwidth/2)·c.omega }
// forward kinematics
toChassisVelocities(w) = { vx: (left+right)/2, vy: 0, omega: (right−left)/trackwidth }
// ratio-preserving desaturation (DifferentialDriveWheelVelocities.desaturate)
realMax = max(|left|, |right|); if (realMax > attainableMax) scale both by attainableMax/realMax;
```
1. **Planning-time velocity**: `DifferentialDriveKinematicsConstraint` (§A.4c)
   caps the per-point path velocity so `|v| + (T/2)|v·κ| ≤ maxWheelVelocity`.
2. **Planning-time acceleration**: `DifferentialDriveVoltageConstraint`
   (§A.4d) bounds chassis acceleration by what the outer/inner wheel motors
   can do at voltage.
3. **Runtime**: follower output
   `ChassisVelocities → toWheelVelocities → desaturate` keeps commanded wheel
   speeds feasible while preserving the turn ratio (so the robot slows rather
   than widening its arc).

For differential trajectories the trajectory itself never stores `ω`
independently — curvature is the coupling variable (`ω = v·κ`, see
`DrivetrainSplineSample` constructor), so limiting `v` as a function of `κ`
*is* the joint constraint.

### A.7 Cross-cutting observations

- **Two authorship lineages:** the 1-DOF profiles are WPILib-native; the time
  parameterizer and spline parameterizer carry Team 254 (2018) MIT headers,
  and both the parameterizer and curvature optimization cite Sprunk 2008
  (uni-freiburg thesis) as the derivation.
- **Statelessness gradient:** `TrapezoidProfile`/`ExponentialProfile.calculate`
  are pure planning functions (safe to re-invoke with a moving goal);
  `ProfiledPIDController` adds exactly one piece of state (the previous
  setpoint); spline trajectories are fully precomputed immutable artifacts
  with algebra (`transformBy`, `relativeTo`, `concatenate`, time-`sample()`
  with constant-accel interpolation).
- **Units:** profiles are unit-agnostic doubles; the trajectory/kinematics
  layer is fixed SI (meters, seconds, rad/m curvature) with optional
  typed-units (`LinearVelocity`, `Distance`) overloads on main.
- **Error-handling philosophy:** generation failures during a match degrade to
  a do-nothing trajectory + DriverStation report instead of throwing;
  genuinely infeasible *user constraints* and impossible integration states
  throw `TrajectoryGenerationException`.
- **Pose+velocity → pose+velocity, summarized:** for one axis,
  `TrapezoidProfile`/`ExponentialProfile` solve it exactly (truncated-profile
  / inflection-point math, decel phase computed backward from the goal state).
  For 2-D poses, WPILib does *not* solve a free boundary-value problem — the
  user supplies the path shape (waypoint poses → hermite splines), and the
  library optimally time-parameterizes it between the configured start/end
  velocities under composable physical constraints. (The main branch also now
  contains separate `DifferentialTrajectory`/`HolonomicTrajectory` sample
  types intended for externally planned trajectories, e.g. Choreo-style, but
  the in-library planner remains the spline pipeline above.)

---

## Appendix B — WPILib trajectory execution pipeline (source memo)

**Provenance:** all code quoted verbatim from `wpilibsuite/allwpilib` tag
**`v2026.2.2`** (latest stable release), fetched via the GitHub API. Reason:
on branch `main` (2027 alpha cycle) the repo was restructured — package
`edu.wpi.first.math` → `org.wpilib.math`, and **`RamseteController`,
`RamseteCommand`, `HolonomicDriveController`, `TrapezoidProfileCommand`, and
`SwerveControllerCommand` have been deleted outright**; `ChassisSpeeds` was
renamed `ChassisVelocities` and `DifferentialDriveWheelSpeeds` →
`DifferentialDriveWheelVelocities` (verified by directory listing of
`wpimath/src/main/java/org/wpilib/math/{controller,kinematics}` at `main`).
The LTV controllers (`LTVUnicycleController`, `LTVDifferentialDriveController`)
survive the purge. So the "classic" API below is the last-released form, and
its deprecation notices were carried out.

**Big picture:** WPILib splits execution into a strict layer cake, all driven
by one 20 ms soft-real-time loop:

```
Trajectory.sample(t)                      [time-indexed reference: Pose2d + v + curvature]
  -> pose controller (Ramsete/LTV/Holonomic): (measured Pose2d, reference) -> ChassisSpeeds
  -> kinematics (DifferentialDriveKinematics.toWheelSpeeds): ChassisSpeeds -> per-wheel m/s
  -> feedforward (SimpleMotorFeedforward.calculateWithVelocities(prev, next)) -> volts
   + per-wheel velocity PID (PIDController on measured wheel m/s)             -> volts
  -> BiConsumer<Double,Double> outputVolts -> motor.setVoltage()
```

Every stage is a pure-ish object the user composes; the `RamseteCommand`
(§B.7) is the canonical glue.

### B.1 `controller/RamseteController.java` — nonlinear unicycle pose controller (DEPRECATED)

**Role:** time-varying nonlinear feedback law for a unicycle model; converts
global pose error into adjusted `(v, ω)`. Javadoc rationale: per-side PID on
wheel arc length can't recover global pose ("multiple endpoints existing for
the robot which have the same encoder path arc lengths"), so a global-pose
controller "adjust[s] the references of the PID controllers."

**Deprecation is explicit and total:**
```java
@Deprecated(since = "2025", forRemoval = true)
public RamseteController(double b, double zeta) {
```
Javadoc: `@deprecated Use LTVUnicycleController instead.` Both constructors
carry it, and the class is gone from `main`. `RamseteCommand` carries the
identical annotation.

**Gains:** `b` ("Tuning parameter (b > 0 rad²/m²) for which larger values make
convergence more aggressive like a proportional term") and `zeta`
("0 rad⁻¹ < zeta < 1 rad⁻¹ ... larger values provide more damping"). Defaults
`this(2.0, 0.7)` — "well-tested to produce desirable results."

**The control law**, quoted from `calculate()`:
```java
m_poseError = poseRef.relativeTo(currentPose);

// Aliases for equation readability
final double eX = m_poseError.getX();
final double eY = m_poseError.getY();
final double eTheta = m_poseError.getRotation().getRadians();
final double vRef = linearVelocityRefMeters;
final double omegaRef = angularVelocityRefRadiansPerSecond;

// k = 2ζ√(ω_ref² + b v_ref²)
double k = 2.0 * m_zeta * Math.sqrt(Math.pow(omegaRef, 2) + m_b * Math.pow(vRef, 2));

// v_cmd = v_ref cos(e_θ) + k e_x
// ω_cmd = ω_ref + k e_θ + b v_ref sinc(e_θ) e_y
return new ChassisSpeeds(
    vRef * m_poseError.getRotation().getCos() + k * eX,
    0.0,
    omegaRef + k * eTheta + m_b * vRef * sinc(eTheta) * eY);
```
Note error is computed in the **robot frame** (`poseRef.relativeTo(currentPose)`),
and `sinc` has a Taylor guard:
`if (Math.abs(x) < 1e-9) return 1.0 - 1.0/6.0*x*x;`. The `vy` slot of the
returned `ChassisSpeeds` is hard-coded `0.0` — a differential drive can't
translate sideways.

**Trajectory adapter overload** — how ω_ref is derived from a trajectory
sample (curvature × velocity):
```java
public ChassisSpeeds calculate(Pose2d currentPose, Trajectory.State desiredState) {
  return calculate(currentPose, desiredState.poseMeters,
      desiredState.velocityMetersPerSecond,
      desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter);
}
```

**State:** only diagnostics (`m_poseError`, `m_poseTolerance`, `m_enabled`) —
no integrators, no reset() needed. `setEnabled(false)` makes calculate() pass
through pure feedforward `(vRef, 0, omegaRef)` — used for tuning isolation
(docs troubleshooting page recommends exactly this).

### B.2 `controller/LTVUnicycleController.java` — LQR gains, gain-scheduled on velocity

Javadoc: "a roughly drop-in replacement for RamseteController with more
optimal feedback gains in the 'least-squares error' sense" (ref: Controls
Engineering in FRC §8.9, theorem 8.9.1).

**What it improves:** Ramsete's `b`/`zeta` are unphysical knobs; LTV lets you
specify **max tolerable state error and max control effort** (Bryson's-rule
Q/R), and the gain is re-derived per operating point instead of the single
hand-derived nonlinear law. Field:
`// LUT from drivetrain linear velocity to LQR gain` —
`InterpolatingMatrixTreeMap<Double, N2, N3> m_table`.

**Construction = offline gain table build.** The model, quoted from the
in-code comment block:
```
// ẋ = v cosθ,  ẏ = v sinθ,  θ̇ = ω
// We're going to make a cross-track error controller, so we'll apply a
// clockwise rotation matrix to the global tracking error to transform it
// into the robot's coordinate frame. Since the cross-track error is always
// measured from the robot's coordinate frame, the model used to compute the
// LQR should be linearized around θ = 0 at all times.
//     [0  0  0]              [1  0]
// A = [0  0  v]          B = [0  0]
//     [0  0  0]              [0  1]
```
and the loop that fills the table (0.01 m/s resolution from −maxVelocity to
+maxVelocity; default maxVelocity 9 m/s, must be < 15):
```java
for (double velocity = -maxVelocity; velocity < maxVelocity; velocity += 0.01) {
  // The DARE is ill-conditioned if the velocity is close to zero, so don't
  // let the system stop.
  if (Math.abs(velocity) < 1e-4) {
    A.set(State.kY.value, State.kHeading.value, 1e-4);
  } else {
    A.set(State.kY.value, State.kHeading.value, velocity);
  }
  var discABPair = Discretization.discretizeAB(A, B, dt);
  ...
  var S = DARE.dareNoPrecond(discA, discB, Q, R);
  // K = (BᵀSB + R)⁻¹BᵀSA
  m_table.put(velocity, discB.transpose().times(S).times(discB).plus(R)
      .solve(discB.transpose().times(S).times(discA)));
}
```
Default Q/R: `qelems = (0.0625 m, 0.125 m, 2.0 rad)`,
`relems = (1 m/s, 2 rad/s)` via `StateSpaceUtil.makeCostMatrix` (1/tol²).
Note **dt is baked in at construction** (`Discretization.discretizeAB(A, B, dt)`)
— the controller is tied to the loop period.

**Runtime is trivial** — table lookup (with interpolation) keyed on the
*reference* linear velocity, times robot-frame pose error:
```java
m_poseError = poseRef.relativeTo(currentPose);
var K = m_table.get(linearVelocityRef);
var e = MatBuilder.fill(Nat.N3(), Nat.N1(),
    m_poseError.getX(), m_poseError.getY(), m_poseError.getRotation().getRadians());
var u = K.times(e);
return new ChassisSpeeds(linearVelocityRef + u.get(0, 0), 0.0, angularVelocityRef + u.get(1, 0));
```
Same `Trajectory.State` adapter overload as Ramsete (ω_ref = v·curvature).
Same stateless/no-reset design. K is a 2×3 gain: rows (v, ω), columns
(e_x, e_y, e_θ). Sibling class `LTVDifferentialDriveController` does the same
with a 5-state model (x, y, heading, v_left, v_right) and outputs
`DifferentialDriveWheelVoltages` directly.

### B.3 `controller/HolonomicDriveController.java` — decoupled x/y PID + profiled heading

Javadoc: "Holonomic trajectory following is a much simpler problem to solve
compared to skid-steer style drivetrains because it is possible to
individually control field-relative x, y, and angular velocity." Because
"heading dynamics are decoupled from translations, users can specify a custom
heading" — heading is an independent reference, profiled by the
`ProfiledPIDController`.

Constructor:
`HolonomicDriveController(PIDController xController, PIDController yController, ProfiledPIDController thetaController)`;
it force-enables angle wrap:
`m_thetaController.enableContinuousInput(0, Units.degreesToRadians(360.0));`.

**calculate() — structure is FF + decoupled per-axis feedback, composed in the
field frame then rotated:**
```java
if (m_firstRun) {
  m_thetaController.reset(currentPose.getRotation().getRadians());
  m_firstRun = false;
}
// Calculate feedforward velocities (field-relative).
double xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();
double yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();
double thetaFF = m_thetaController.calculate(
    currentPose.getRotation().getRadians(), desiredHeading.getRadians());
...
double xFeedback = m_xController.calculate(currentPose.getX(), trajectoryPose.getX());
double yFeedback = m_yController.calculate(currentPose.getY(), trajectoryPose.getY());
return ChassisSpeeds.fromFieldRelativeSpeeds(
    xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
```
Key contrasts with Ramsete/LTV: (a) feedback runs on **field-frame** axis
errors, not robot-frame cross-track error, then the sum is rotated into the
chassis frame via `fromFieldRelativeSpeeds`; (b) the trajectory's
`Trajectory.State` supplies only translation reference — the heading is a
**separate** `Rotation2d desiredHeading` argument (differential trajectories
couple heading to path tangent; holonomic ones don't); (c) it holds a
`m_firstRun` latch that resets the theta profile to the current heading on
first use — the only implicit reset in any of these controllers. The `thetaFF`
name is misleading — it's the profiled-PID *output* (feedback on the profiled
heading reference), not a pure feedforward.

### B.4 `kinematics/ChassisSpeeds.java` — the chassis twist-rate carrier + `discretize()`

Fields (public, mutable, SI-suffixed names):
```java
/** Velocity along the x-axis. (Fwd is +) */   public double vxMetersPerSecond;
/** Velocity along the y-axis. (Left is +) */  public double vyMetersPerSecond;
/** Represents the angular velocity of the robot frame. (CCW is +) */ public double omegaRadiansPerSecond;
```
Javadoc distinguishes it from `Twist2d`: "Whereas a Twist2d represents a
change in pose w.r.t to the robot frame of reference, a ChassisSpeeds object
represents a robot's velocity." Also: "A strictly non-holonomic drivetrain,
such as a differential drive, should never have a dy component." Overloads
exist taking typed units (`LinearVelocity`, `AngularVelocity` from
`edu.wpi.first.units.measure`) that convert via `.in(MetersPerSecond)` — the
units system is a compile-time veneer over raw SI doubles.

**`discretize()` — quoted in full:**
```java
public static ChassisSpeeds discretize(
    double vxMetersPerSecond, double vyMetersPerSecond,
    double omegaRadiansPerSecond, double dtSeconds) {
  // Construct the desired pose after a timestep, relative to the current pose. The desired pose
  // has decoupled translation and rotation.
  var desiredDeltaPose = new Pose2d(
      vxMetersPerSecond * dtSeconds,
      vyMetersPerSecond * dtSeconds,
      new Rotation2d(omegaRadiansPerSecond * dtSeconds));
  // Find the chassis translation/rotation deltas in the robot frame that move the robot from its
  // current pose to the desired pose
  var twist = Pose2d.kZero.log(desiredDeltaPose);
  // Turn the chassis translation/rotation deltas into average velocities
  return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
}
```
**Why it exists:** a discrete controller holds `(vx, vy, ω)` constant for a
whole 20 ms step, but a chassis simultaneously rotating while translating
sweeps an **arc**, not the straight chord the caller intended — the naive
command lands the robot rotated-and-displaced off the target ("translational
skew"). `discretize()` inverts this: it builds the pose you actually want
after `dt` (decoupled translation + rotation), takes the SE(2) matrix **log**
(`Pose2d.log`) to get the constant twist whose exponential lands exactly on
that pose, and divides by dt. It's the exact correction (via the Lie-group log
map) for ZOH control of a rigid body. Javadoc: "This is useful for
compensating for translational skew when translating and rotating a holonomic
(swerve or mecanum) drivetrain," with the caveat that post-discretization
desaturation scaling "rotates the direction of net motion in the opposite
direction of rotational velocity, introducing a different translational skew
which is not accounted for by discretization." For a differential drive it's a
no-op in y and thus irrelevant; it matters for swerve/mecanum.

**`fromFieldRelativeSpeeds`** — a clockwise frame rotation, nothing more:
```java
// CW rotation into chassis frame
var rotated = new Translation2d(vxMetersPerSecond, vyMetersPerSecond).rotateBy(robotAngle.unaryMinus());
return new ChassisSpeeds(rotated.getX(), rotated.getY(), omegaRadiansPerSecond);
```
(`fromRobotRelativeSpeeds` is the CCW inverse.) Full algebra suite
(`plus/minus/times/div/unaryMinus`) supports composing FF and FB speed
contributions.

### B.5 Differential-drive kinematics

`kinematics/DifferentialDriveKinematics.java` — one parameter,
`public final double trackWidthMeters` (javadoc: "the empirical value may be
larger than the physical measured value due to scrubbing effects").

**Inverse (chassis → wheels), used on the command path:**
```java
public DifferentialDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
  return new DifferentialDriveWheelSpeeds(
      chassisSpeeds.vxMetersPerSecond - trackWidthMeters / 2 * chassisSpeeds.omegaRadiansPerSecond,
      chassisSpeeds.vxMetersPerSecond + trackWidthMeters / 2 * chassisSpeeds.omegaRadiansPerSecond);
}
```
i.e. `left = vx − ω·W/2`, `right = vx + ω·W/2`. `vy` is silently ignored — the
type system doesn't forbid handing a holonomic ChassisSpeeds to a diff-drive
kinematics.

**Forward (wheels → chassis), used on the measurement/odometry path:**
```java
public ChassisSpeeds toChassisSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
  return new ChassisSpeeds(
      (wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond) / 2, 0,
      (wheelSpeeds.rightMetersPerSecond - wheelSpeeds.leftMetersPerSecond) / trackWidthMeters);
}
```
plus a `toTwist2d(leftDistance, rightDistance)` distance-delta form for
odometry.

**`DifferentialDriveWheelSpeeds.desaturate()`** — ratio-preserving rescale
when inverse kinematics demands more than the drivetrain has (mutates in
place):
```java
public void desaturate(double attainableMaxSpeedMetersPerSecond) {
  double realMaxSpeed = Math.max(Math.abs(leftMetersPerSecond), Math.abs(rightMetersPerSecond));
  if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
    leftMetersPerSecond  = leftMetersPerSecond  / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
    rightMetersPerSecond = rightMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
  }
}
```
This preserves the L/R ratio (hence curvature) at the cost of speed. Notably
`RamseteCommand` does **not** call it — trajectory generation is expected to
respect velocity constraints up front (`DifferentialDriveVoltageConstraint`
etc.).

### B.6 Feedforward — `SimpleMotorFeedforward` and `DifferentialDriveFeedforward`

`controller/SimpleMotorFeedforward.java`. Model:
`u = kS·sgn(v) + kV·v + kA·a`, gains in "volts", "V/(units/s)",
"V/(units/s²)". The constructor takes an optional `dtSeconds`, **default
0.020 s** — dt is a constructor-time property, mirroring the loop period.

The old continuous form is deprecated:
```java
@Deprecated(forRemoval = true, since = "2025")
public double calculate(double velocity, double acceleration) {
  return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
}
```
**The current discrete form is exact plant inversion of the discretized
first-order motor model** (javadoc: "Note this method is inaccurate when the
velocity crosses 0."):
```java
public double calculateWithVelocities(double currentVelocity, double nextVelocity) {
  // See wpimath/algorithms.md#Simple_motor_feedforward for derivation
  if (ka < 1e-9) {
    return ks * Math.signum(nextVelocity) + kv * nextVelocity;
  } else {
    double A = -kv / ka;
    double B = 1.0 / ka;
    double A_d = Math.exp(A * m_dt);
    double B_d = A > -1e-9 ? B * m_dt : 1.0 / A * (A_d - 1.0) * B;
    return ks * Math.signum(currentVelocity) + 1.0 / B_d * (nextVelocity - A_d * currentVelocity);
  }
}
```
This is the scalar specialization of `LinearPlantInversionFeedforward`
(`u_ff = B⁺(r_{k+1} − A r_k)` with `A_d = e^{AT}`, `B_d = A⁻¹(e^{AT} − I)B`);
the derivation is checked in at `wpimath/algorithms.md` ("Simple motor
feedforward" section: dx/dt = −kᵥ/kₐ·x + 1/kₐ·u − kₛ/kₐ·sgn(x), discretize the
affine model, solve for uₖ). Instead of asking "what voltage sustains this v
and a," it asks "what constant voltage over the next dt drives the plant from
exactly v_k to exactly v_{k+1}" — this kills the systematic lag of continuous
FF under ZOH.

**`DifferentialDriveFeedforward`** exists
(`controller/DifferentialDriveFeedforward.java`) and generalizes this to the
coupled 2×2 drivetrain plant (linear + angular kV/kA, via
`LinearSystemId.identifyDrivetrainSystem`), accounting for the cross-coupling
between sides that two independent `SimpleMotorFeedforward`s ignore:
```java
public DifferentialDriveWheelVoltages calculate(
    double currentLeftVelocity, double nextLeftVelocity,
    double currentRightVelocity, double nextRightVelocity, double dtSeconds) {
  var feedforward = new LinearPlantInversionFeedforward<>(m_plant, dtSeconds);
  var r = VecBuilder.fill(currentLeftVelocity, currentRightVelocity);
  var nextR = VecBuilder.fill(nextLeftVelocity, nextRightVelocity);
  var u = feedforward.calculate(r, nextR);
  return new DifferentialDriveWheelVoltages(u.get(0, 0), u.get(1, 0));
}
```
(Design wart: it allocates and rediscretizes a
`LinearPlantInversionFeedforward` on every call.) It's the FF companion of
`LTVDifferentialDriveController`; the classic `RamseteCommand` path uses
`SimpleMotorFeedforward` per side instead.

### B.7 `RamseteCommand.java` — the canonical wiring (wpilibNewCommands)

Javadoc: "The command handles trajectory-following, PID calculations, and
feedforwards internally... a more-or-less 'complete solution'." Both
constructors `@Deprecated(since = "2025", forRemoval = true)`.

Full-service constructor dependency list (all injected as suppliers/consumers
— the command owns **no hardware**): `Trajectory`, `Supplier<Pose2d> pose`,
`RamseteController`, `SimpleMotorFeedforward`, `DifferentialDriveKinematics`,
`Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds` (measured),
`PIDController leftController`, `PIDController rightController`,
`BiConsumer<Double,Double> outputVolts`. The second constructor omits FF/PID
and outputs raw m/s — javadoc: "Advanced teams seeking more flexibility (for
example, those who wish to use the onboard PID functionality of a 'smart'
motor controller) may use the secondary constructor."

**`initialize()` — reset semantics:**
```java
public void initialize() {
  m_prevTime = -1;
  var initialState = m_trajectory.sample(0);
  m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(
      initialState.velocityMetersPerSecond, 0,
      initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
  m_prevLeftSpeedSetpoint = m_prevSpeeds.leftMetersPerSecond;
  m_prevRightSpeedSetpoint = m_prevSpeeds.rightMetersPerSecond;
  m_timer.restart();
  if (m_usePID) { m_leftController.reset(); m_rightController.reset(); }
}
```

**`execute()` — quoted in full (the whole pipeline in one 20 ms tick):**
```java
public void execute() {
  double curTime = m_timer.get();

  if (m_prevTime < 0) {
    m_output.accept(0.0, 0.0);
    m_prevTime = curTime;
    return;
  }

  var targetWheelSpeeds =
      m_kinematics.toWheelSpeeds(
          m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

  double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
  double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

  double leftOutput;
  double rightOutput;

  if (m_usePID) {
    double leftFeedforward =
        m_feedforward.calculateWithVelocities(m_prevLeftSpeedSetpoint, leftSpeedSetpoint);
    double rightFeedforward =
        m_feedforward.calculateWithVelocities(m_prevRightSpeedSetpoint, rightSpeedSetpoint);

    leftOutput = leftFeedforward
        + m_leftController.calculate(m_speeds.get().leftMetersPerSecond, leftSpeedSetpoint);
    rightOutput = rightFeedforward
        + m_rightController.calculate(m_speeds.get().rightMetersPerSecond, rightSpeedSetpoint);
  } else {
    leftOutput = leftSpeedSetpoint;
    rightOutput = rightSpeedSetpoint;
  }

  m_output.accept(leftOutput, rightOutput);
  m_prevSpeeds = targetWheelSpeeds;
  m_prevTime = curTime;
}
```
Reading it against the pipeline: (1) elapsed-time sample
`m_trajectory.sample(curTime)` — `Trajectory.sample()` binary-searches the
timestamped state list and kinematically interpolates between samples
(`v_f = v_0 + at`, `Δs = v_0 t + ½at²`, then pose lerp by arc-length fraction
— from `Trajectory.State.interpolate`); (2) Ramsete against measured pose from
the injected odometry supplier; (3) inverse kinematics to per-wheel m/s;
(4) discrete FF using **(previous setpoint, current setpoint)** as the
(current, next) pair — note it feeds forward on the setpoint sequence, not
measured velocity, and the older releases' explicit `/dt` acceleration
computation is gone, replaced by `calculateWithVelocities` with dt baked into
the feedforward object; (5) per-wheel `PIDController.calculate(measured,
setpoint)` — measured wheel speeds re-read from the supplier per side;
(6) volts out via the `BiConsumer` (user typically
`leftMotors.setVoltage(l); rightMotors.setVoltage(r);`). `isFinished()` is
purely temporal: `m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds())` — no
pose-convergence check. `end(interrupted)` zeroes output **only if
interrupted** ("not appropriate for paths with nonstationary endstates" to
always stop).

The per-wheel `PIDController.calculate` (v2026.2.2) is textbook discrete PID
with fixed-period backward-difference derivative and clamped integral:
`m_errorDerivative = (m_error - m_prevError) / m_period; ... return
m_kp*m_error + m_ki*m_totalError + m_kd*m_errorDerivative;`, `reset()` clears
`error/prevError/totalError/errorDerivative/haveMeasurement`. Doc: "The
PIDController assumes that the calculate() method is being called regularly at
an interval consistent with the configured period" (default 20 ms).

**Siblings:** `SwerveControllerCommand.execute()` is the holonomic analog and
stops one layer higher (no FF/PID — module states out):
`var desiredState = m_trajectory.sample(curTime); var targetChassisSpeeds =
m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
m_outputModuleStates.accept(targetModuleStates);` — javadoc: "will not return
output voltages but rather raw module states from the position controllers
which need to be put into a velocity PID." `TrapezoidProfileCommand` (1-DOF
mechanisms, also `@Deprecated since 2025` — "Use a TrapezoidProfile instead")
is just `m_output.accept(m_profile.calculate(0.02, m_currentState.get(),
m_goal.get()));` per tick — the 20 ms literal hard-coded.

### B.8 Scheduling model — TimedRobot + CommandScheduler

`TimedRobot.java`: `public static final double kDefaultPeriod = 0.02;`. The
constructor does `addPeriodic(this::loopFunc, period)`. `startCompetition()`
is a priority-queue-of-callbacks loop blocked on an FPGA **Notifier** alarm
(`NotifierJNI.updateNotifierAlarm` / `waitForNotifierAlarm`) — hardware-timed
wakeups, not `sleep()`. Overrun handling skips missed periods rather than
bursting:
```java
// Increment the expiration time by the number of full periods it's behind
// plus one to avoid rapid repeat fires from a large loop overrun.
callback.expirationTime += callback.period
    + (currentTime - callback.expirationTime) / callback.period * callback.period;
```
`addPeriodic(callback, period, offset)` lets users co-schedule faster loops
(e.g. a 10 ms drivetrain control callback) on the same Notifier thread,
phase-offset from the main loop — the sanctioned way to run control faster
than 50 Hz on the RIO without threads.

The command-based idiom: user's `robotPeriodic()` calls
`CommandScheduler.getInstance().run()` every 20 ms. `run()` order:
(1) `subsystem.periodic()` for every registered subsystem — **this is where
odometry updates happen**, so the pose measurement consumed by
`RamseteCommand.execute()` is at most one phase old within the same tick;
(2) poll button event loop → schedule commands; (3) `command.execute()` for
each scheduled command, then `isFinished()` → `end(false)` and removal;
(4) schedule default commands on unrequired subsystems. A `m_watchdog` prints
"CommandScheduler loop overrun" with per-epoch timing when the tick exceeds
the period.

**Where state lives:** the trajectory is immutable data; controllers keep only
tuning + last-error diagnostics (Ramsete/LTV have no reset at all;
HolonomicDriveController has the one-shot theta-profile reset;
PIDController/ProfiledPIDController have explicit `reset()`); all inter-tick
tracking state (`m_timer`, `m_prevSpeeds`, `m_prev*Setpoint`, `m_prevTime`)
lives in the **Command** object and is re-initialized in `initialize()`, so a
command instance is safely re-runnable. Time is wall-clock elapsed (`Timer`),
not tick-counted — a dropped tick advances the sample point rather than
stretching the trajectory.

### B.9 Where the velocity PID actually runs in FRC practice

The WPILib design explicitly supports pushing the innermost loop off the RIO:

- docs.wpilib.org "Ramsete Controller" page, on tracking the wheel-speed
  setpoints, verbatim: "two PID Controllers, one for each side may be used to
  track these velocities," using either "the WPILib PIDController" or "**the
  Velocity PID feature on smart motor controllers such as the TalonSRX and the
  SPARK MAX**."
- `RamseteCommand`'s own javadoc (§B.7): the raw-wheel-speeds constructor
  exists precisely for "those who wish to use the onboard PID functionality of
  a 'smart' motor controller."
- docs.wpilib.org PIDController page: the software `PIDController` "is
  intended primarily for synchronous use from the main robot loop, and so this
  value is defaulted to 20ms."

So the two topologies are: (a) **all-RIO** — full `RamseteCommand`
constructor, 50 Hz velocity PID + FF summed to volts, `motor.setVoltage()`;
(b) **hybrid, the common competitive practice** — RIO runs pose controller +
kinematics at 50 Hz and sends each side's **velocity setpoint plus an
arbitrary/added feedforward voltage** to the CAN motor controller (CTRE Talon
FX / REV SPARK MAX), whose firmware closes the velocity loop at ~1 kHz on its
own encoder. (The 1 kHz figure is from vendor documentation — CTRE Phoenix and
REV document their internal closed-loop rate as 1 kHz — not from
docs.wpilib.org, which only names the capability.) The hybrid wins on: 20×
loop rate for disturbance rejection, immunity to RIO loop jitter/overruns, and
no CAN measurement latency inside the fast loop. WPILib's discrete FF
(`calculateWithVelocities`) still runs RIO-side in both topologies; only the
feedback term moves.

### B.10 Cross-cutting design observations

- **Layer boundaries are types:** `Trajectory.State` (time, pose, v, a,
  curvature) → `ChassisSpeeds` → `DifferentialDriveWheelSpeeds` → volts. Each
  controller stage is swappable (Ramsete↔LTV are literally
  signature-identical).
- **dt is configuration, not argument**, in the modern API: LTV bakes dt into
  its gain table, SimpleMotorFeedforward into `m_dt` (default 0.020),
  PIDController into `m_period`. Everything assumes the fixed 20 ms cadence;
  TrapezoidProfileCommand hard-codes `0.02`.
- **Units:** raw SI doubles with unit-suffixed identifiers
  (`vxMetersPerSecond`) plus optional typed-units overloads
  (`LinearVelocity.in(MetersPerSecond)`) that erase to the same doubles.
- **Controllers are near-stateless; commands hold the run state** and reset it
  in `initialize()` — the framework's answer to re-entrancy.
- **The 2025→2027 arc:** Ramsete (and its command, and the WPILib-bundled
  trajectory-following commands generally) deprecated in 2025 "Use
  LTVUnicycleController instead," fully deleted on main; the ecosystem has
  effectively moved trajectory *generation+following* wiring to third-party
  libraries (PathPlanner/Choreo) with wpimath supplying the
  controller/kinematics/FF primitives.

---

## Appendix C — WPILib odometry / pose estimation / architecture (source memo)

**Provenance:** all code quoted verbatim from `wpilibsuite/allwpilib` branch
`main` (fetched 2026-07-12), plus the `v2025.3.2` tag for one comparison,
docs.wpilib.org, and pathplanner.dev. **Caveat:** `main` is the **2027
development branch**: the Java package `edu.wpi.first.*` is gone (0 paths in
the git tree) — everything is now `org.wpilib.*`, and several classes were
renamed (§C.7). Released 2025/2026 WPILib that teams actually run uses
`edu.wpi.first.math.*` with slightly different API shapes; differences flagged
where they matter.

### C.1 Odometry — `kinematics/Odometry.java` + `DifferentialDriveOdometry.java`

**Role:** the only stateful dead-reckoning integrator. Generic over
wheel-position type `T` (`Odometry<T>`);
`DifferentialDriveOdometry extends Odometry<DifferentialDriveWheelPositions>`.

**State (all of it):**
```java
private final Kinematics<T, ?, ?> m_kinematics;
private Pose2d m_pose;
private Rotation2d m_gyroOffset;
// Always equal to m_poseMeters.getRotation()
private Rotation2d m_previousAngle;
private final T m_previousWheelPositions;
```

**The whole update, verbatim:**
```java
public Pose2d update(Rotation2d gyroAngle, T wheelPositions) {
    var angle = gyroAngle.rotateBy(m_gyroOffset);

    var twist = m_kinematics.toTwist2d(m_previousWheelPositions, wheelPositions);
    twist.dtheta = angle.minus(m_previousAngle).getRadians();

    var newPose = m_pose.plus(twist.exp());

    m_kinematics.copyInto(wheelPositions, m_previousWheelPositions);
    m_previousAngle = angle;
    m_pose = new Pose2d(newPose.getTranslation(), angle);

    return m_pose;
}
```
Design decisions worth stealing:
- **Absolute inputs, internal differencing.** `update()` takes *cumulative*
  encoder distances and *absolute* gyro angle; the class differences them
  against stored previous values. No dt anywhere — pure geometry per step.
- **Gyro overrides kinematic dtheta.**
  `twist.dtheta = angle.minus(m_previousAngle)` — the encoder-derived rotation
  from `toTwist2d` is discarded and replaced with the gyro delta. Consequence:
  `DifferentialDriveOdometry`'s constructor passes a dummy trackwidth:
  `super(new DifferentialDriveKinematics(1), ...)` — trackwidth only affects
  dtheta, which is overwritten, and dx=(left+right)/2 is
  trackwidth-independent.
- **Heading is set absolutely, not integrated:** final pose is
  `new Pose2d(newPose.getTranslation(), angle)` — translation is integrated
  via twist exp, but rotation is snapped to the (offset-corrected) gyro each
  step, so heading never accumulates integration drift beyond the gyro's own
  drift.
- **`resetPosition` semantics:** never touches gyro hardware; it recomputes a
  software offset:
```java
public void resetPosition(Rotation2d gyroAngle, T wheelPositions, Pose2d pose) {
    m_pose = pose;
    m_gyroOffset = gyroAngle.unaryMinus().rotateBy(m_pose.getRotation());
    m_previousAngle = m_pose.getRotation();
    m_kinematics.copyInto(wheelPositions, m_previousWheelPositions);
}
```
  It also re-latches `m_previousWheelPositions` to the *current* encoder
  reading, so the next update integrates from zero delta — encoders need not
  be zeroed either. Separate `resetPose/resetTranslation/resetRotation` exist
  for partial resets; `resetPose` keeps the gyro continuous by folding the
  rotation change into `m_gyroOffset`.

`DifferentialDriveKinematics.toTwist2d` (the FK used above):
```java
public Twist2d toTwist2d(double leftDistance, double rightDistance) {
    return new Twist2d(
        (leftDistance + rightDistance) / 2, 0, (rightDistance - leftDistance) / trackwidth);
}
```

### C.2 Pose estimation with delayed vision — `estimator/PoseEstimator.java`

**Role:** wraps an `Odometry<T>` ("drop-in replacement for Odometry; if you
never call addVisionMeasurement... this will behave exactly the same as
Odometry"). `DifferentialDrivePoseEstimator extends
PoseEstimator<DifferentialDriveWheelPositions>` and just constructs the
odometry; **defaults: state stddevs `(0.02 m, 0.02 m, 0.01 rad)`, vision
stddevs `(0.1, 0.1, 0.1)`**.

**State:**
```java
private final Odometry<T> m_odometry;
// Diagonal of process noise covariance matrix Q
private final double[] m_q = new double[] {0.0, 0.0, 0.0};
// Diagonal of Kalman gain matrix K
private final double[] m_vision_k = new double[] {0.0, 0.0, 0.0};
private static final double kBufferDuration = 1.5;
// Maps timestamps to odometry-only pose estimates
private final TimeInterpolatableBuffer<Pose2d> m_odometryPoseBuffer =
    TimeInterpolatableBuffer.createBuffer(kBufferDuration);
// Maps timestamps to vision updates
private final NavigableMap<Double, VisionUpdate> m_visionUpdates = new TreeMap<>();
private Pose2d m_poseEstimate;
```

**Key mechanism — NOT a replay-the-filter design.** It maintains two parallel
histories: (a) a 1.5 s buffer of *odometry-only* poses keyed by timestamp, and
(b) a sorted map of `VisionUpdate` records. A `VisionUpdate` is a pair
`(visionPose, odometryPose)` at one timestamp; "replaying odometry forward" is
done algebraically by rebasing later odometry deltas:

```java
private static final class VisionUpdate {
    private final Pose2d visionPose;    // the vision-compensated pose estimate
    private final Pose2d odometryPose;  // the pose estimated based solely on odometry
    ...
    public Pose2d compensate(Pose2d pose) {
      var delta = pose.minus(this.odometryPose);
      return this.visionPose.plus(delta);
    }
}
```
i.e. estimate(t) = visionPose ⊕ (odometryPose(t) ⊖ odometryPose(t_vision)).
The raw odometry integrator is **never rewound or corrected** — corrections
live in an overlay, and the relative odometry motion since the vision fix is
rigidly re-anchored onto the corrected pose.

**Per-loop update** just feeds the buffer and applies the latest overlay:
```java
public Pose2d updateWithTime(double currentTime, Rotation2d gyroAngle, T wheelPositions) {
    var odometryEstimate = m_odometry.update(gyroAngle, wheelPositions);
    m_odometryPoseBuffer.addSample(currentTime, odometryEstimate);
    if (m_visionUpdates.isEmpty()) {
      m_poseEstimate = odometryEstimate;
    } else {
      var visionUpdate = m_visionUpdates.get(m_visionUpdates.lastKey());
      m_poseEstimate = visionUpdate.compensate(odometryEstimate);
    }
    return getEstimatedPosition();
}
```

**`addVisionMeasurement(visionRobotPose, timestamp)` verbatim core** (numbered
comments are the actual source):
```java
// Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()
    || m_odometryPoseBuffer.getInternalBuffer().lastKey() - kBufferDuration > timestamp) {
  return;
}
// Step 1: Clean up any old entries
cleanUpVisionUpdates();
// Step 2: Get the pose measured by odometry at the moment the vision measurement was made.
var odometrySample = m_odometryPoseBuffer.getSample(timestamp);
if (odometrySample.isEmpty()) { return; }
// Step 3: Get the vision-compensated pose estimate at the moment the vision measurement was made.
var visionSample = sampleAt(timestamp);
if (visionSample.isEmpty()) { return; }
// Step 4: Measure the transform between the old pose estimate and the vision pose.
var transform = visionRobotPose.minus(visionSample.get());
// Step 5: We should not trust the transform entirely, so instead we scale this transform by a
// Kalman gain matrix representing how much we trust vision measurements compared to our current
// pose. Then, we convert the result back to a Transform2d.
var scaledTransform =
    new Transform2d(
        m_vision_k[0] * transform.getX(),
        m_vision_k[1] * transform.getY(),
        Rotation2d.fromRadians(m_vision_k[2] * transform.getRotation().getRadians()));
// Step 6: Calculate and record the vision update.
var visionUpdate = new VisionUpdate(visionSample.get().plus(scaledTransform), odometrySample.get());
m_visionUpdates.put(timestamp, visionUpdate);
// Step 7: Remove later vision measurements. (Matches previous behavior)
m_visionUpdates.tailMap(timestamp, false).entrySet().clear();
// Step 8: Update latest pose estimate. Since we cleared all updates after this vision update,
// it's guaranteed to be the latest vision update.
m_poseEstimate = visionUpdate.compensate(m_odometry.getPose());
```
Notes:
- The innovation (Step 4) is measured against the **already-vision-compensated**
  historical pose (`sampleAt`, which composes the newest vision update
  at-or-before t with the buffered odometry pose at t, clamping t into the
  buffer's time range), so repeated measurements converge rather than
  double-correct.
- Out-of-order vision measurements ARE handled (any timestamp within the 1.5 s
  window), but Step 7 discards vision updates *newer* than the one being added
  — a deliberate simplification.
- Measurements older than the buffer window are silently dropped (Step 0) —
  bounded memory, bounded rewind.

**Kalman-gain-like blending.** Per-axis scalar gains, closed form, computed
once per stddev change — no covariance propagation at runtime:
```java
// Solve for closed form Kalman gain for continuous Kalman filter with A = 0
// and C = I. See wpimath/algorithms.md.
for (int row = 0; row < 3; ++row) {
  if (m_q[row] == 0.0) {
    m_vision_k[row] = 0.0;
  } else {
    m_vision_k[row] = m_q[row] / (m_q[row] + Math.sqrt(m_q[row] * r[row]));
  }
}
```
where `q = stateStdDev²`, `r = visionStdDev²`. `wpimath/algorithms.md` derives
it: steady-state Riccati with A=0, C=I gives `p = √(qr)`, then
`K = P(P+R)⁻¹ → k = q/(q + √(qr))`. Corner cases: q=0 → k=0 (never trust
vision on that axis); r=0 → k=1 (snap to vision). Per-measurement stddevs
supported via the `addVisionMeasurement(pose, t, stdDevs)` overload. Docs
guidance (docs.wpilib.org state-space-pose-estimators): "make the vision
heading standard deviation very large, make the gyro heading standard
deviation small, and scale the vision x and y standard deviation by distance
from the tag."

**`TimeInterpolatableBuffer<T>`**: a `TreeMap<Double,T>` + interpolator
function. `addSample` evicts entries older than `historySize`; `getSample(t)`
returns exact hit, else clamps to nearest end, else interpolates floor/ceiling
entries with `(t - t0)/(t1 - t0)`. For `Pose2d` the interpolator is
`Pose2d.interpolate`, which is **geodesic on SE(2)**:
`var twist = endValue.minus(this).log(); ... this.plus(scaledTwist.exp())` —
so between-sample poses follow constant-curvature arcs, consistent with the
odometry model.

**All resets (`resetPosition/resetPose/resetTranslation/resetRotation`) clear
both buffers.** resetTranslation/resetRotation carefully re-seed one synthetic
VisionUpdate that preserves the *other* component of the latest vision
correction — the level of care needed when a reset and an outstanding vision
overlay coexist.

### C.3 SE(2) exp/log — `geometry/Pose2d.java`, `Twist2d.java`, `Transform2d.java`

`Twist2d` is a plain mutable struct: `public double dx, dy, dtheta;` — "A
change in distance along a 2D arc since the last pose update."

**API move on main:** in released WPILib (verified against `v2025.3.2`), the
map lives on Pose2d: `public Pose2d exp(Twist2d twist)` and
`public Twist2d log(Pose2d end)`. On main it's been refactored to
`Twist2d.exp() → Transform2d` and `Transform2d.log() → Twist2d`; odometry
composes `m_pose.plus(twist.exp())`. The math is identical.

`Twist2d.exp()` verbatim (main):
```java
public Transform2d exp() {
    double sinTheta = Math.sin(dtheta);
    double cosTheta = Math.cos(dtheta);
    double s;
    double c;
    if (Math.abs(dtheta) < 1E-9) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;   // Taylor of sinθ/θ
      c = 0.5 * dtheta;                         // Taylor of (1−cosθ)/θ
    } else {
      s = sinTheta / dtheta;
      c = (1 - cosTheta) / dtheta;
    }
    return new Transform2d(
        new Translation2d(dx * s - dy * c, dx * c + dy * s), new Rotation2d(cosTheta, sinTheta));
}
```
`Transform2d.log()` verbatim (inverse; `halfThetaByTanOfHalfDtheta` =
(θ/2)/tan(θ/2), Taylor `1 − θ²/12` near zero):
```java
public Twist2d log() {
    final double dtheta = m_rotation.getRadians();
    final double halfDtheta = dtheta / 2.0;
    final double cosMinusOne = m_rotation.getCos() - 1;
    double halfThetaByTanOfHalfDtheta;
    if (Math.abs(cosMinusOne) < 1E-9) {
      halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halfThetaByTanOfHalfDtheta = -(halfDtheta * m_rotation.getSin()) / cosMinusOne;
    }
    Translation2d translationPart =
        m_translation
            .rotateBy(new Rotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta))
            .times(Math.hypot(halfThetaByTanOfHalfDtheta, halfDtheta));
    return new Twist2d(translationPart.getX(), translationPart.getY(), dtheta);
}
```
Javadoc cites the derivation: "Controls Engineering in the FIRST Robotics
Competition" §10.2 "Pose exponential". **Why it beats naive Euler**
(`x += v cosθ dt`): Euler assumes heading is constant across the step, biasing
translation whenever the robot turns while moving; the pose exponential is the
*exact* solution of the pose ODE under the constant-twist (constant-curvature
arc) assumption, so per-step error comes only from the twist not being
constant, not from the integrator. WPILib leans on this everywhere: odometry
integration, pose interpolation in the vision buffer, and trajectory math all
share the same exp/log pair, so the whole stack is self-consistent on SE(2).
`Pose2d` itself is a `final` immutable value class (`Translation2d` +
`Rotation2d`, where `Rotation2d` stores cos/sin), with group ops
`plus(Transform2d)`, `minus(Pose2d)→Transform2d`, `relativeTo`, `toMatrix()`.

### C.4 Units handling

- **Core math is SI doubles**: meters, meters/sec, radians, seconds, volts.
  Every math-class javadoc says "in meters"/"in radians"; no runtime unit
  objects inside hot paths.
- **Typed units are a boundary convenience.** `wpiunits`
  (`org.wpilib.units.Measure<U extends Unit>`, concrete `Distance`,
  `LinearVelocity`, `Time`, etc.) — every math class offers typed overloads
  that convert immediately and delegate to the double version, e.g.
  `DifferentialDriveOdometry`:
  `this(gyroAngle, leftDistance.in(Meters), rightDistance.in(Meters), initialPose);`.
  Java Measure is object-based (with `MutMeasure` variants to avoid
  allocation); C++ uses the `units::` strong-typedef library at compile time
  with zero runtime cost.
- **Main-branch identifier cleanup:** the 2027 branch **stripped unit suffixes
  from public fields** — `DifferentialDriveWheelPositions.leftMeters` → `left`,
  `ChassisSpeeds.vxMetersPerSecond` → `ChassisVelocities.vx`,
  `trackWidthMeters` → `trackwidth` — units now live only in javadoc. WPILib
  independently arrived at "name the quantity, units in docs."

### C.5 Architectural shape — where state lives

Layering, bottom-of-math to top:

| Layer | Statefulness | Evidence |
|---|---|---|
| Geometry (`Pose2d`, `Transform2d`, `Rotation2d`, `Translation2d`) | Immutable value objects; all ops return new instances | `public final class Pose2d`, final fields |
| Kinematics (`Kinematics<P,S,A>` interface, `DifferentialDrive/Mecanum/SwerveDriveKinematics`) | **Stateless pure math**; only immutable config (`public final double trackwidth`) | FK/IK: `toChassisVelocities`, `toWheelVelocities`, (main adds `toChassisAccelerations`/`toWheelAccelerations`), `toTwist2d(start, end)` |
| Odometry (`Odometry<T>`) | Small stateful integrator: pose + gyro offset + previous samples (5 fields, §C.1) | absolute-in/differencing-inside; explicit `reset*` family |
| Estimator (`PoseEstimator<T>`) | Odometry + two bounded time-indexed buffers (1.5 s) + 3 scalar gains | correction is an overlay, never mutates the odometry integrator |
| Controllers (`PIDController`, `LTVUnicycleController`, ...) | Tiny explicit state + `reset()`: `m_prevError`, `m_totalError`, setpoint/measurement caches; `calculate(measurement)` advances it | period is fixed config (default 20 ms), not measured |
| Trajectory (`Trajectory<SampleType>`) | **Immutable value object**: sorted `List<SampleType>` + `InterpolatingTreeMap`; `sampleAt(time)` is a pure lookup/interpolation; `transformBy/relativeTo/concatenate` return new trajectories | it has no clock — **the command layer owns the timer** |
| Command layer (wpilibj) | Owns scheduling, timers, subsystem ownership; subsystems own their estimator instance and call `update()` in `periodic()` | |

Time injection is uniform: every stateful class takes timestamps as parameters
(`updateWithTime(currentTime, ...)`, `addVisionMeasurement(pose, timestamp)`);
the no-arg variants default to `MathSharedStore.getTimestamp()`. Nothing reads
sensors; the user shuttles numbers from devices into math classes each loop.
That's the whole dependency story — no DI framework, no globals beyond the
clock default.

**Simulation** (`DifferentialDrivetrainSim.java`): a stateful physics plant,
completely decoupled from odometry/estimation. State vector
`x = [x, y, heading, leftVelocity, rightVelocity, leftPosition, rightPosition]ᵀ`
(N7); `setInputs(leftVoltage, rightVoltage)`; `update(dt)` integrates
nonlinear dynamics with Dormand-Prince:
`m_x = NumericalIntegration.rkdp(this::getDynamics, m_x, m_u, dt);`.
`getDynamics` composes the nonlinear pose kinematics (`ẋ = v cosθ`,
`ẏ = v sinθ`, `θ̇ = (vr − vl)/2rb`) with a **linear 2-state velocity plant**
whose A/B come from either physical constants or SysId kV/kA. The plant
factory (renamed `LinearSystemId` → `Models` on main),
`Models.differentialDriveFromSysId(kVLinear, kALinear, kVAngular, kAAngular)`:
```java
double A1 = -0.5 * (kVLinear / kALinear + kVAngular / kAAngular);
double A2 = -0.5 * (kVLinear / kALinear - kVAngular / kAAngular);
double B1 = 0.5 / kALinear + 0.5 / kAAngular;
double B2 = 0.5 / kALinear - 0.5 / kAAngular;
// states/outputs [left velocity, right velocity], inputs [left voltage, right voltage]
```
(with a `trackwidth` overload converting angular gains via
`kVAngular * 2.0 / trackwidth`). So a sim plant is characterized by the *same
four feedforward constants* teams measure on the real robot with SysId. In
user code, sim classes sit behind the HAL: the sim update loop reads the same
commanded voltages, steps the plant, and writes encoder/gyro sim values, so
odometry/estimator/controller code paths are byte-identical between sim and
real.

### C.6 Ecosystem reality check (2024–2026)

What competitive teams actually run:
- **Trajectory generation is external to WPILib.** PathPlanner (GUI editor +
  PPLib vendor library; "Trajectory V2" generates against swerve
  kinematics/dynamics, torque-limits accel/decel from motor capability) and
  **Choreo** (SleipnirGroup; time-optimal NLP via TrajoptLib/Sleipnir solver,
  trajectories solved offline in the GUI, exported as sampled `.traj` files).
  WPILib's own spline `TrajectoryGenerator`/PathWeaver path is effectively
  legacy for serious teams.
- **Following is deliberately simple: PID + feedforward on pregenerated
  samples.** From pathplanner.dev: holonomic drives use
  `PPHolonomicDriveController` — per-axis translation PID + rotation PID
  (`new PIDConstants(5.0, 0.0, 0.0)` typical) added on top of the trajectory's
  velocity feedforwards; differential drives use `PPLTVController`.
  `AutoBuilder.configure(...)` takes a pose supplier (usually the WPILib pose
  estimator), a robot-relative speeds supplier, and a consumer of "ROBOT
  RELATIVE ChassisSpeeds, AND feedforwards". Choreo's ChoreoLib is the same
  shape: sample callback → PID on pose error + sample velocity FF.
- **Velocity loops run on the motor controllers.** TalonFX (Phoenix 6) /
  SPARK MAX run onboard velocity PID (~1 kHz) with kS/kV/kA feedforward; the
  roboRIO sends velocity setpoints at 50–250 Hz. AdvantageKit's TalonFX/Spark
  swerve templates (the de-facto standard team scaffolding) wire exactly this:
  odometry at high rate from device timestamps, `SwerveDrivePoseEstimator`,
  vision via PhotonVision/Limelight `addVisionMeasurement` with
  distance-scaled stddevs.
- Net: the WPILib estimator (§C.2) is the one piece of WPILib motion everyone
  uses; generation and low-level velocity control both migrated off the RIO.

### C.7 Recent motion-API developments on main (2027 alphas)

Directly observed in the tree/source (`v2027.0.0-alpha-*` territory; the docs
"New for 2027" changelog corroborates):
- **Full package rename** `edu.wpi.first.*` → `org.wpilib.*`; C++ headers to
  `wpi/math/...`.
- **Trajectory rework:** old concrete `Trajectory` + `Trajectory.State`
  replaced by abstract `Trajectory<SampleType extends TrajectorySample>`
  (sorted sample list + `InterpolatingTreeMap`, abstract kinematic
  `interpolate`, `transformBy/relativeTo/concatenate`), with
  `DrivetrainSplineTrajectory`/`DrivetrainSplineSample` produced by
  `DrivetrainSplineTrajectoryGenerator`. Samples are JSON-serializable
  (`io.avaje.jsonb` annotations) and struct/protobuf-serializable — clearly
  shaped to interop with externally generated (Choreo/PathPlanner-style)
  sampled trajectories rather than owning generation.
- **`ChassisSpeeds` → `ChassisVelocities`**, new **`ChassisAccelerations`**,
  and the `Kinematics<P, S, A>` interface grew acceleration FK/IK
  (`toChassisAccelerations`/`toWheelAccelerations`) — second-order kinematics
  for feedforward-heavy followers.
- **RamseteController deleted** (was deprecated through 2024–25);
  `LTVUnicycleController`/`LTVDifferentialDriveController` are the retained
  differential-drive trackers.
- **`LinearSystemId` → `Models`** with clearer names
  (`differentialDriveFromSysId`, `differentialDriveFromPhysicalConstants`, ...).
- **`Pose2d.exp/log` → `Twist2d.exp()` / `Transform2d.log()`**;
  `Pose2d.kZero` preallocated constants; unit-suffix removal on public fields
  (§C.4).
- **Commands v3** (`org.wpilib.command3`, coroutine-flavored replacement for
  command-based v2), still WIP.
- PoseEstimator itself is architecturally unchanged from the 2024 rewrite
  (buffer + overlay, §C.2) — that mechanism is stable across 2024→2027.

**Most transferable points for sprint 099:** (a) corrections as an *overlay*
(visionPose, odometryPose) pair over an untouched dead-reckoning integrator —
no filter rewind, no odometry mutation; (b) bounded `TimeInterpolatableBuffer`
of odometry-only poses with SE(2)-geodesic interpolation for sampling at the
camera timestamp; (c) innovation measured against the already-compensated
historical estimate so repeated fixes converge; (d) per-axis closed-form gain
`k = q/(q + √(qr))` from stddevs instead of a running covariance; (e) resets
must clear/re-seed the buffers explicitly.

Sources: [allwpilib main tree](https://github.com/wpilibsuite/allwpilib),
[wpimath/algorithms.md](https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/algorithms.md),
[docs.wpilib.org pose estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html),
[2027 changelog](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html),
[pathplanner.dev follow-a-path](https://pathplanner.dev/pplib-follow-a-single-path.html),
[SleipnirGroup/Choreo](https://github.com/SleipnirGroup/Choreo),
[AdvantageKit TalonFX swerve template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/),
[Phoenix 6 PID docs](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html).
