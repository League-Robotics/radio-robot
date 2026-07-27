---
status: pending
---

# Rebuild the motion planner core: per-wheel distance profiling + velocity-domain PID

> **Implementation status (2026-07-27, out of process).** Steps T1-T5 and T7
> are DONE and verified; T0 and T6 are NOT done — see "What was built" at the
> end of this file for what changed relative to the proposal below, the three
> defects the work uncovered, and why the two cleanup steps were deferred.

## Description

`Motion::Planner` has not caught up with two changes that landed around it: the
rebuilt `App::RobotLoop::cycle()`
([robot_loop.cpp:407-469](../../src/firm/app/robot_loop.cpp#L407-L469)) and the
measured per-wheel, per-direction speed→duty calibration now inverted inside
`App::Drive`. Three defects follow.

**1. The velocity loop is open on the robot.** `Motion::Planner` runs
`Motion::WheelPid` every tick and discards the result: `Planner::update()`
publishes only velocities into `state_.wheel*.cmdVelocity`
([planner.cpp:865](../../src/motion/planner/planner.cpp#L865)), and
[robot_loop.cpp:415](../../src/firm/app/robot_loop.cpp#L415) hands those to
`Drive`, which converts open-loop. `commandedDutyLeft/Right()` has **zero**
consumers in `src/firm` — only `capi.cpp`, two host bench scripts, and the duty
tests read it. `Planner::stageDuty()`
([planner.cpp:116-205](../../src/motion/planner/planner.cpp#L116-L205), ~90
lines) is dead compute inside the 47 ms budget, and
[main.cpp:264-278](../../src/firm/main.cpp#L264-L278) configures gains for it.

**2. The profiler plans on body axes, not on wheels.** `planActive()` switches
four ways over linear `v_x` / angular `omega`
([planner.cpp:748-829](../../src/motion/planner/planner.cpp#L748-L829)), then
inverse-kinematics to wheels. But the thing that must hit a distance target is a
*wheel*, and the per-axis formulation cannot express a per-wheel distance
ledger.

**3. A dead motion stack is still compiled.** `move_queue`, `velocity_shaper`
and `stop_condition` drive nothing on the robot (superseded by
`src/motion/planner/`) yet are built into `libmotion`, `libfirmware_host`, and
three test binaries. `src/sim/sim_harness.h` still constructs a
`Motion::MoveQueue`.

## Cause

The `src/motion/planner/` tree was introduced (commit `f6dbf598`, 2026-07-26) as
the loop's motion decider, superseding `Motion::MoveQueue` without retiring it.
Its duty stage was designed when `App::Drive` still held no calibration, so duty
was the natural output; the subsequent Drive calibration (commit `5065775a`)
made the velocity boundary the better one, but the duty stage was never
re-pointed or removed. `src/motion/DESIGN.md` (last reviewed 2026-07-24)
predates both changes and does not mention `planner/` at all.

## Proposed fix

Keep the `Planner` shell — the `tick(const RobotState&) → TickResult` /
`update(RobotState&) const` contract, the 5-deep queue, lookahead, estimation
(`WheelChannel`/`PoseTracker`), and the completion/ack machinery. Replace the
decision core. **The main loop is not modified**; the single actuation output
stays `state_.wheel*.cmdVelocity` in [mm/s].

Four stakeholder decisions constrain the design (2026-07-27):

| | Decision |
|---|---|
| PID output | **Velocity-domain trim.** `cmdVelocity = profiledTarget + trim`, trim in [mm/s]. Drive's calibrated map remains the feedforward — zero main-loop change. |
| PID gating | **P and accel-feedforward live in all phases; the INTEGRATOR engages only during HOLD.** |
| Wheel coupling | **Ratio-locked per-wheel.** Each wheel has its own distance target and profile; each tick both are scaled by the same feasible fraction (the more-constrained wheel binds), preserving the commanded left:right ratio and therefore the turn radius. |
| Scope | Keep the shell, rebuild the core. |

### The shape reduction

New pure module `src/motion/planner/shape.{h,cpp}`. Every move this planner runs
is **constant-ratio** (the commanded left:right ratio never changes within a
move), so one derivation covers all four Kinds: compute the duration `T` the
move takes at cruise, then `d_w = v_w · T`.

```cpp
struct MoveShape {
  float unitLeft, unitRight;          // [1] normalized so max(|uL|,|uR|) == 1
  float distanceLeft, distanceRight;  // [mm] signed per-wheel targets
  float cruise;                       // [mm/s] dominant wheel's speed
  float duration;                     // [s] Kind::Time only
  bool hasDistanceTarget, hasTimeBudget, valid;
};
MoveShape  shapeOf(const Move&, float trackWidth);
AxisLimits shapeLimits(const MoveShape&, const PlannerLimits&);
bool       shapesCompatible(const MoveShape& a, const MoveShape& b);
```

`Twist` → `vL = v_x - omega*W/2`, `vR = v_x + omega*W/2`; `Wheels` → as given.
Then `p = max(|vL|,|vR|)`, `u = v/p`, `cruise = p`. Duration: `Distance` →
`threshold / |(vL+vR)/2|`; `Angle` → `threshold*W / |vR-vL|`; `Time` →
`threshold/1000`. Degenerate combinations (a pivot with a distance stop, a
straight with an angle stop) set `valid = false`. `Angle(Θ, ω, W)` falls out as
`d = ∓Θ·W/2` — the wheel-arc case emerges from the general formula rather than
being hand-coded.

`shapeLimits()` maps the body ceilings into one shape-space `AxisLimits` via
`mean = |(uL+uR)/2|` and `diff = |uR-uL|`, taking the min of the linear-derived
and angular-derived caps. It reduces bit-identically to today's hand-written
cases: straight → `{vMax, aMax, aDecel, jerkMax}`
([planner.cpp:771](../../src/motion/planner/planner.cpp#L771)); pivot →
`{omegaMax·W/2, alphaMax·W/2, …}`
([planner.cpp:803-805](../../src/motion/planner/planner.cpp#L803-L805)). This
one function replaces the four-way switch.

### The phase machine and the ratio lock

`enum class MovePhase : uint8_t { Idle, Accel, Hold, Decel, Settle };` in
`planner_types.h`. `ActiveMove` gains `shape`, `shapeLimits`,
`baselineLeft/Right`, `phase`, `decelLatched`. Per tick, replacing
`planActive()`:

```
1. r_w = dirSign_w * (targetDistance_w - traveled_w)   // traveled_w = basisPosition_w - baseline_w
                                                       // then the existing ZOH/actuation-delay anticipation, per wheel
2. b   = boundaryFraction(dt)
3. per wheel: step_w = profileStep(r_w, lambda*|u_w|, cruise*|u_w|, b*|u_w|,
                                   scale(shapeLimits, |u_w|), dt, shapeAccel_*|u_w|)
4. lambda = min over wheels of (step_w.velocity / |u_w|)   // <-- THE RATIO LOCK
5. if (decelLatched) lambda = min(lambda, lambdaPrev)      // <-- never accelerate at the end
6. cmdLeft_ = lambda*unitLeft;  cmdRight_ = lambda*unitRight
7. phase = maxSeverity(step_L.phase, step_R.phase); latch on Decel
```

**Why the ratio lock preserves exact landing.** `profileStep()` is homogeneous
of degree 1 in `(remaining, previous, cruise, boundary, aMax, aDecel, jMax)` —
every term in `profile.cpp` is a sum or comparison of velocity- or
distance-dimensioned quantities, with no absolute constants but `kTiny`. On a
perfect plant `r_w = |u_w|·R` for a common `R`, so wheel `w`'s arguments are
exactly `|u_w|` times the shape-space arguments, so `step_w.velocity =
|u_w|·Λ` for the same `Λ` — both wheels agree identically and `min()` is a tie.
The closing step `remaining/dt` survives intact. On a real plant where the two
remainings genuinely diverge, the behind wheel binds and the result is
conservative and correct.

Two numerical guards, both required: **normalize by the max** so the dominant
wheel has `|u| == 1.0f` exactly and its arithmetic is untouched (only the
sub-dominant wheel eats rounding); and **break ties toward the dominant wheel**
(`kRatioTie = 1e-6f`), without which a 1-ulp rounding difference lets the
sub-dominant wheel steal the closing step and the landing loses exactness.

The latch is deliberately monotone **non-increasing**, not decreasing: if a
successor is queued mid-decel and raises the boundary, hold rather than
re-accelerate.

`Kind::Time` has no distance target — `profileStep` is called with `remaining =
+inf` and the decel trigger comes from the clock (`ticksLeft < stepsNeeded`).
**The strict `<` must be preserved verbatim**;
[planner.cpp:56-59](../../src/motion/planner/planner.cpp#L56-L59) records it as
a measured fix for a zero-command frame that chain observers read as a hand-off
dip. `timedRamp()` folds into this and is deleted.

### `profile.h` — two additive changes only

```cpp
enum class StepPhase : uint8_t { Accel, Hold, Decel, Closing };
struct ProfileResult { float velocity; bool closing; StepPhase phase; };  // phase is NEW
struct AxisLimits {
  float vMax, aMax, aDecel, jMax;
  float aDecelPlan = 0.0f;  // NEW: 0 == "same as aDecel". The brake-START decision
                            // (brakeDistance/feasible) uses this; the terminal step and
                            // the per-step floor still use full aDecel, so landing stays
                            // discrete-exact and an infeasible state still brakes hard.
};
```

Both are purely additive — every existing brace-init at
[planner.cpp:771](../../src/motion/planner/planner.cpp#L771),
[:793](../../src/motion/planner/planner.cpp#L793) and in `profile_test.cpp`
compiles and behaves identically.

### Lookahead — compatibility becomes a shape identity test

`boundaryVelocity()` → `boundaryFraction(dt)`. Compatibility collapses to
`|uL_a - uL_b| <= 1e-3 && |uR_a - uR_b| <= 1e-3`, which subsumes every case
[planner.cpp:638-669](../../src/motion/planner/planner.cpp#L638-L669)
hand-enumerates and adds two it could not express:

| current → next | units | boundary |
|---|---|---|
| straight → same-direction straight | (1,1)→(1,1) | `> 0` |
| straight → reversal | (1,1)→(-1,-1) | **0** |
| straight → turn (axis change) | (1,1)→(-1,1) | **0** |
| turn L → turn L | (-1,1)→(-1,1) | `> 0` |
| arc R=200 → same arc | (0.6,1)→(0.6,1) | `> 0` *(new)* |
| `Wheels(300,100)` → `Wheels(150,50)` | (1,.33)→(1,.33) | `> 0` *(new — old code returned 0 for all Wheels)* |
| anything → `Kind::Stop`, or empty queue | — | 0 |

Magnitude: `min(active.cruise, next.cruise)`, further capped for a
distance-targeted successor by `maxEntryVelocity(nextDominantDistance, 0,
shapeLimits(next), dt)`. Fail closed to 0 when `limits_.aDecel <= 0` or either
shape is invalid.

### The velocity trim

New type alongside `WheelPid` (which is parked, not deleted):

```cpp
struct VelocityTrimGains {
  float kp   = 0.0f;  // [1]     DIMENSIONLESS: mm/s of trim per mm/s of error
  float ki   = 0.0f;  // [1/s]
  float iMax = 0.0f;  // [mm/s]  integrator clamp; 0 disables integration
  float kaff = 0.0f;  // [s]     accel feedforward (~= plant tau)
  float trimMax = 0.0f;  // [mm/s] total trim clamp; 0 = unclamped
};
class WheelTrim {
  float compute(float target, float targetAccel, float measured, float dt, MovePhase phase);
};
```

**There is no `kff`, and the field must be deleted rather than defaulted to 0.**
In the duty formulation `kff` *was* the entire plant inverse (`1/1370`). In a
velocity trim the feedforward has already happened twice — the profiled target
*is* the `1·target` term, and `Drive::correctedCommand()`
([drive.cpp:88-99](../../src/firm/app/drive.cpp#L88-L99)) then applies a
freshly-measured per-wheel per-direction affine inverse, which is strictly
better than one scalar. A zero-defaulted `kff` invites pasting the old number
back in.

**`kaff` survives and gets simpler.** In the duty domain it was `tau·kff` — a
product that drifts with battery voltage and every recalibration. In the
velocity domain it is just `tau [s] ≈ 0.23`, directly measurable and
calibration-independent. Caution: at `aMax = 300 mm/s²`, `kaff·a = 69 mm/s` of
lead against a 150-400 mm/s cruise — commission at half authority.

**Integrator gating is by explicit phase, not by `iAccelGate`.** Today's
`iAccelGate` only *derates* to 0.2×
([wheel_pid.cpp:25-27](../../src/motion/planner/wheel_pid.cpp#L25-L27)); the
phase is what it actually meant. HOLD only, and **frozen — not reset — through
decel**, so a chain does not throw away trim it will need again.

**The trim is applied only in `update()`, never folded into
`cmdLeft_`/`cmdRight_`:**

```cpp
state.wheelLeft.cmdVelocity  = cmdLeft_  + trimLeft_;
state.wheelRight.cmdVelocity = cmdRight_ + trimRight_;
```

Two independent reasons, both already encoded in the existing code: `measure()`'s
ZOH anticipation
([planner.cpp:552-563](../../src/motion/planner/planner.cpp#L552-L563)) computes
*planned* travel from `cmdLeft_`/`cmdLeftPrevious_`, so a trim inside them
corrupts the ledger; and the next tick's `previous` argument to `profileStep()`
would carry the trim and break the discrete-exact accounting. The profiler plans
against a command that is not exactly what is actuated — which is correct,
because the ledger is anchored to *measured* positions and closing that gap is
the trim's entire job. Keep the EMA filter-lag prediction the duty stage learned
([planner.cpp:172-177](../../src/motion/planner/planner.cpp#L172-L177)) inside
`stageTrim()`; it was a measured fix.

New `PlannerLimits` fields `trimKp / trimKi / trimIMax / trimKaff / trimMax`,
fail-closed at all-zero. `applyVelGains()` → `applyTrimGains()`.

### Constraints that override the naive reading of the requirement

**The distance ledger is re-anchor plus bounded extrapolation, never
accumulation.** Each tick, take the freshly measured encoder position as truth
and extrapolate forward only across the known gap since that read; throw the
extrapolation away and recompute it from scratch next tick. Its error is bounded
by one gap and is corrected by the next anchor, so it never accumulates. This is
already what `WheelChannel::positionAt()`
([estimation.cpp:22-30](../../src/motion/planner/estimation.cpp#L22-L30)) and
`Planner::measure()` do; the per-wheel rework must preserve it.

What must NOT happen is the other operation: `distance += v·dt` accumulated
across ticks. `wheel.velocity` is a naive per-tick difference quotient with no
filter in the firmware leaf
([nezha_motor.cpp:212](../../src/firm/devices/nezha_motor.cpp#L212), documented
at [nezha_motor.h:140](../../src/firm/devices/nezha_motor.h#L140)), so
accumulating it rectifies noise one-way. This codebase already paid for that
lesson — [planner.cpp:571-577](../../src/motion/planner/planner.cpp#L571-L577)
records `pathLength()` inflating ~0.07 mm per jitter cycle and completing Moves
measurably short. **Per-wheel travel is anchored on
`channel.basisPosition() - baseline_w`** — a telescoping difference of real
cumulative encoder positions — with the extrapolation added on top.

Three details of the existing scheme the rework must carry forward:

- **Two spans are extrapolated, not one.** `[anchorTime → now]` (already
  elapsed, unobserved because the sample is older than the loop) *and*
  `[now → now + actuationDelay]` (this tick's command does not reach the wheels
  until the next cycle). At 47 ms and 400 mm/s the second span is ~19 mm. Each
  is attributed to the command that actually drove it — `cmdPrevious_` for the
  elapsed span, `cmd_` for the future one
  ([planner.cpp:552-563](../../src/motion/planner/planner.cpp#L552-L563)).
- **The extrapolation uses the COMMANDED velocity, not the measured one**
  ([planner.cpp:544-548](../../src/motion/planner/planner.cpp#L544-L548)): the
  command is exact where the encoder derivative is very noisy, and this term is
  *differenced* for the angular axis, where per-wheel noise does not cancel the
  way it does in the mean. Keep commanded — the velocity trim's job is to make
  the plant track, so commanded only gets more trustworthy as this work lands.
  **Open call, settle from the sim CSV:** measured is the honest alternative if
  tracking proves poor; it is a one-line switch and the per-tick capture carries
  both.
- **Chained moves do not re-baseline on the fresh encoder read.**
  `activateNext()` baselines a successor on the predecessor's cumulative
  boundary (`baseline + threshold`), so a leg's sub-tick overshoot is debited to
  the next leg and a four-leg square leaks zero total error. Re-baselining on
  the live encoder at each move start would silently forgive each leg's
  overshoot and let it accumulate around the tour.

**Do NOT store a decel-start point.** `profileStep()` stores no such point and
asks per-tick "is the max reachable velocity still feasible?" — the same
decision with no cached value that can go stale on a boundary change, an encoder
correction, or a `replace`.

**"Leeway to coast into the end" means decel-authority headroom, not
command-authority headroom.** On this plant coasting is bad: `tau ≈ 230 ms`, and
[planner.cpp:24-30](../../src/motion/planner/planner.cpp#L24-L30) records +0.9°
of post-settle coast per turn. Implement as `aDecelPlan < aDecel` — brake
earlier, keep full authority in reserve. Never "stop commanding early and let
friction finish."

**Three differential correctors would act on one physical axis** once this
lands: the ratio lock (holds commanded ratio), the per-wheel trim (deliberately
breaks it to fix actual ratio), and `applyHeadingHold()`.
[main.cpp:214-220](../../src/firm/main.cpp#L214-L220) records a 2-3 Hz limit
cycle from over-gained control on this exact hardware. A per-wheel trim that
drives both wheels onto their own distance targets *is* heading hold done on
distance, so `headingHoldGain` is the one to retire once the trim proves out.
Ship it at 0 and revisit on the bench.

### Sequence

Each step leaves the tree building and the existing suites green.

**T0 — Dead-code subtraction (no planner change).** Delete `move_queue`,
`velocity_shaper`, `stop_condition` + their three harnesses in
`src/tests/sim/unit/` + entries in `src/motion/CMakeLists.txt` and
[src/sim/CMakeLists.txt:135-141](../../src/sim/CMakeLists.txt#L135-L141).
`src/sim/sim_harness.h` includes and constructs a `Motion::MoveQueue` — that
goes too, as does `src/tests/sim/unit/app_move_queue_harness.cpp`. **Verify
before deleting** whether `bench_test_config.h:45` / `app_drive_harness.cpp:62`
use `WheelVelocityPid`'s class or only its `Motion::Gains` POD; if only the POD,
relocate it and delete the class. `odometry`, `state_estimator`,
`body_kinematics`, `wheel_sink.h` are **LIVE in firmware** — keep them.
Gate: `just build-sim`, `uv run python -m pytest`, `just build`.

**T1 — `profile.h`: report the phase, split plan-decel from authority-decel.**
Purely additive. Gate: `planner_tests` unmodified + 2 new profile cases.

**T2 — `shape.{h,cpp}`, pure and tested, not yet wired.** `planner.cpp`
untouched. Gate: everything + new `shape_test`.

**T3 — Rewrite `planActive()` onto the per-wheel phase machine + ratio lock.**
Replaces [planner.cpp:748-829](../../src/motion/planner/planner.cpp#L748-L829).
`profileVelocity_/profileAccel_` → `shapeVelocity_/shapeAccel_`; `ActiveMove`
gains the baselines. **Leave the settle creep
([planner.cpp:686-725](../../src/motion/planner/planner.cpp#L686-L725))
alone** — doubling the blast radius here is how this step fails. **The only
high-risk step.** Gate: `planner_scenarios_test` and `planner_noise_test` pass
**unmodified** — that is the regression contract. Explicit review focus: the
`activeBoundary_ > 0` early-completion shortcut
([planner.cpp:377-386](../../src/motion/planner/planner.cpp#L377-L386)), whose
units change with the rework and which directly controls chain exactness
(`testSameAxisChainExactAndCarried` asserts 800.000 ± 1e-3).

**T4 — Lookahead in shape space.** `boundaryFraction()` + `lookahead_test`.

**T5 — The velocity trim.** `WheelTrim`, `MovePhase`, `trimLeft_/trimRight_`,
`stageTrim()` in the slot `stageDuty()` occupies today. Gate: every existing
test unchanged *because gains default to zero* — assert that directly.

**T6 — Park the duty stage.** Move `stageDuty()`/`WheelPid`/`PidGains` to
`duty_stage.{h,cpp}` behind `MOTION_PLANNER_DUTY_STAGE`, defined **only** in
`src/motion/planner/CMakeLists.txt` — not in the ARM or sim builds, so ~90 lines
and two PID objects leave the image and the 47 ms budget. Keep it alive for the
host: `bench/hil_drive.py --duty-plane` works, and `planner_duty_scenarios_test`
+ `duty_plant.h` are the only tier that exercises a non-ideal *actuator*. Update
the ctypes mirror `py/planner_harness.py:139-167` in the same commit —
`plannerStructSizes()` guards it, so a miss fails loudly.

**T7 — Sim runs and charts.** See Verification.

**Deferred, out of scope here:** wiring real trim gains into
[main.cpp:264-278](../../src/firm/main.cpp#L264-L278) and stopping
[configurator.cpp:120-134](../../src/firm/app/configurator.cpp#L120-L134) from
retargeting duty-domain wire `pid.*` values (~1370× misscaled) at the planner.
That is the hardware hook-up, after sim proves the core.

## Verification

### Unit tests

`cmake -S src/motion/planner -B src/motion/planner/build && cmake --build src/motion/planner/build --target planner_tests`

The existing 27 cases across `profile_test`, `estimation_test`,
`planner_scenarios_test`, `planner_noise_test` must pass **unmodified** — that
is the whole safety net. New binaries:

- **`shape_test`** — table-driven over every (Kind × VelocityKind), including
  the arc case (`v_x=200, ω=1, W=100, D=500` → `u=(0.6,1)`, `d=(375,625)`, mean
  500, Δheading 2.5 rad), the degenerate guards, and the `shapeLimits()`
  reduction identities against today's hand-written straight and pivot cases.
- **`phase_test`** — the sequence is `Accel* Hold* Decel*` with no Accel or Hold
  after any Decel; a short move skips Hold and still lands exactly; `Kind::Time`
  fires on the strict `<`; `Kind::Stop` is Decel every tick.
- **`ratio_lock_test`** — for straight / pivot / 3:1 arc / `Wheels(300,100)` /
  `Wheels(-200,600)`: commanded-ratio invariance `cmdLeft_·uR == cmdRight_·uL`
  to 1e-6 relative **every tick**, including the closing step and the drain;
  under a new asymmetric-plant knob (left 8% slow) the commanded ratio stays
  exact with trim off and the *actual* ratio converges with trim on; a binding
  sub-dominant wheel scales the dominant wheel *down* rather than clipping it;
  both wheels land exactly on the same tick.
- **`wheel_trim_test`** — fail-closed (zero gains → `cmdVelocity` bit-for-bit
  equal to the profiled target); integrator exactly 0 through accel, grows in
  hold, **frozen not reset** through decel; a wound integrator produces no
  post-move motion; `kaff == tau` keeps ramp error inside one noise band;
  standing robot does not drift with trim on.
- **`lookahead_test`** — every row of the compatibility table, plus a 3-move
  same-shape chain that never dips below `cruise - aDecel·dt`, and a mid-chain
  reversal that dips to exactly 0.

### Sim square tour + charts

**Primary: `src/motion/planner/bench/square_tour_sim.py`** — pure Python plant
driving the real `Motion::Planner` through the ctypes ABI, schedule-faithful, no
firmware build. Install **real** ceilings (the plant tau ≈ 0.23 s is already
modeled; add a per-wheel gain-asymmetry knob so the trim has something to
correct) and extend the chart to show:

1. Per-wheel commanded vs actual velocity, with ACCEL/HOLD/DECEL shaded as
   background bands.
2. Remaining distance per wheel against the computed brake distance — the two
   curves touching *is* the decel trigger, making the brake-start logic visible.
3. Trim contribution plotted separately from the profiled target, with the
   integrator trace, so hold-only gating shows as a flat line during ramps.
4. XY path from the pose tracker vs ground truth, closure error annotated.

Emit a per-tick CSV (`t, phase, remL, remR, cruise, lambda, cmdL, cmdR, trimL,
trimR, intL, intR, measL, measR, posL, posR`) alongside the PNG, following the
`turn_prediction_capture.py` pattern. Run three tours: (a) trim off, symmetric
plant; (b) trim off, 8% asymmetric plant; (c) trim on, 8% asymmetric plant. (c)
should close the square where (b) does not.

**Confirmation: full-stack sim.** A MOVE-driven square tour through `SimLoop`
(`src/sim` → real `RobotLoop::cycle()` → real `App::Drive` with its calibrated
map → real `WheelPlant` physics), following `src/tests/bench/square_tour.py`'s
`SimBackend` pattern (`connect(start_tick_thread=False)`, manual stepping —
never the tick thread for a measurement run). `TestSim::simPlannerLimits()` at
[sim_harness.h:55-80](../../src/sim/sim_harness.h#L55-L80) is deliberately
*unshaped* (`aMax = 1e6`), so the bench must install real ceilings or the
profile will be invisible. This is the only run that exercises the trim together
with Drive's feedforward inversion — the real hardware path.

Acceptance: square closure error and final heading error, trim on vs off, on the
asymmetric plant.

## What was built

Delivered on `pid-removal`, out of process, 2026-07-27. Every planner test
suite green (9 binaries), `just build-sim` and `just build` clean.

**Done as proposed:** T1 (`profile.h` `StepPhase` + `aDecelPlan`), T2
(`shape.{h,cpp}` + `shape_test`), T3 (per-wheel phase machine + ratio lock in
`Planner::planWheels()`), T4 (`boundaryLambda()` shape-space lookahead), T5
(`WheelTrim` in `wheel_trim.{h,cpp}`, hold-only integrator), T7 (the sim tour,
`bench/square_tour_velocity.py`).

**Changed during implementation:**

- **The axis-unit bridge.** Rather than converting the whole planner to wheel
  units, `ActiveMove::axisPerLambda` bridges only inside `planWheels()`.
  Everything outside it -- the completion tests, `activeBoundary_`,
  `profileVelocity_`, the settle creep -- keeps its existing units, which is
  why `planner_scenarios_test` and `planner_noise_test` passed unmodified.
- **`Wheels` Moves keep ramp-and-hold.** Per-wheel distance profiling applies
  to Twist `Distance`/`Angle` Moves. The `Wheels` early-return path is
  deliberately untouched (the scenario tests lock that semantic in).
- **`WheelTrim` is a NEW type**, not a modified `WheelPid`; the duty stage is
  untouched and still passes its own tests.
- **`decelPlanFraction` added to `PlannerLimits`** as the user-facing leeway
  knob feeding `aDecelPlan`, with a per-Move feasibility floor in
  `shapeLimits()`.
- **Heading hold stays ON** (`headingHoldGain = 2.0` in the tour). The
  proposal suggested retiring it once the trim proved out; measurement says
  otherwise. The trim's integrator is gated to the hold phase, and ~31% of a
  tour is spent in ramps where proportional action alone leaves a few percent
  of gain error. That residual integrates into ANGLE, which no wheel-RATE
  loop can retire. Inner rate loop + outer angle loop is the right pair; the
  ratio lock is not a third corrector (it never reads a measurement).

**Three defects the new tests found, all fixed:**

1. **The drain broke the commanded ratio.** `drainToZero()` and the planned
   stop subtracted the same ABSOLUTE step from each wheel, so a robot
   stopping out of an arc straightened as it slowed. Now proportional
   (`rampCommandsToZero()`); identical arithmetic for 1:1 and -1:1 shapes.
2. **The drain used the wrong decel ceiling.** It ramped at the body-frame
   `aDecel` while the profile landed under shape-space limits, so the
   terminal step did not fit in one drain step and leaked ~18 um per landing
   on a tight arc. Now `drainDecel()` uses the shape's own ceiling.
3. **`PlannerLimits` ctypes mirror scrambled.** The trim fields were added
   mid-struct in C++ and appended in `py/planner_harness.py`; both sizes
   matched, so the size-only `plannerStructSizes` guard passed while every
   field after the insertion read a different member. It presented as a
   wildly mistuned controller. Fixed by appending in both, and
   `plannerLimitsOffsets()` now checks per-field OFFSETS. **New rule: append
   `PlannerLimits` fields at the END, never mid-struct.**

**NOT done, deliberately:**

- **T6 (park the duty stage behind a build flag).** Deferred: `stageDuty()`
  is called unconditionally and its `velK*` fields are set by
  `sim_harness.h`, so making it conditional is invasive, and the payoff (~90
  lines out of the image) is modest against the risk.
- **T0 (delete the dead legacy stack).** Deferred: the proposal assumed pure
  subtraction, but a reference sweep found ~40 non-archive files touching
  `move_queue`/`velocity_shaper`/`stop_condition`, including live host Python
  (`sim_loop.py`, `transport.py`, `planner/tour.py`, `protocol.py`) and many
  sim system tests. Most are the PROTOCOL sense of "stop condition"
  (`stop_distance`/`stop_angle`), not the C++ class, so the cut needs a
  careful triage pass rather than a bulk delete. `src/sim/sim_harness.h` also
  still constructs a `Motion::MoveQueue`. Worth doing; needs its own session.

**A fourth build source list exists.** Beyond `src/motion/planner/CMakeLists.txt`
and `src/sim/CMakeLists.txt`, eight `src/tests/sim/**/test_*.py` files each
carry their OWN hand-maintained `_APP_SOURCES` list and compile their harness
with a bare `c++` invocation. A new planner `.cpp` must be added to all three
places or those tests fail at LINK time, not compile time. Patched here for
`shape.cpp` and `wheel_trim.cpp`.

**Test baseline.** After the source-list fix the suite is 9 failed / 598
passed under `src/tests/testgui`. Those 9 (`test_tour_closure_gate`,
`test_gui_button_acceptance` turns/tours, `test_sim_transport_tour1`,
`test_error_divergence`) are **PRE-EXISTING**: a worktree built at `5065775a`
fails the identical 9 with byte-identical numbers (turn error
-10.469468549668306). They are Tour-1/Tour-2 90-degree turns landing ~7-10
deg short in the TestGUI sim -- unrelated to this work and still open.

**Bench result** (`bench/square_tour_velocity.py`, asymmetric plant, wheels
+-4.8% apart in true gain after a shared-constant calibration):

| | closure | worst leg | worst turn |
|---|---|---|---|
| trim OFF | 177.1 mm (8.85%) | 129.8 mm | 21.6 deg |
| trim ON | 16.0 mm (0.80%) | 1.9 mm | 1.6 deg |

Two tuning facts worth keeping: `trimKaff` at the FULL plant tau is unstable
(closure 696 mm); half tau is the working value. And the sim's Drive mirror
must model `NezhaMotor::writeShapedDuty()`'s output deadband -- without it a
gently-landing profile stalls dead below breakaway, which looked like a
planner collapse but was a modeling omission.

## Related

- `docs/design/motion-planner-sketch.md` — the planner's design of record
  (stakeholder-reviewed 2026-07-25).
- `docs/design/wheel-speed-command-mapping.md` — the speed→duty study whose
  ~6-9 mm/s residual is what the velocity trim exists to close. Its Result
  2/Result 5 tables are on `tovez_nocal`'s placeholder travel calibration and
  differ from `tovez.json`'s shipped constants by a factor of 1.336; the doc has
  not been updated.
- `clasi/issues/wheel-speed-feedforward-calibration.md` — the calibration that
  landed in `App::Drive` (commit `5065775a`).
- `src/motion/DESIGN.md` — materially stale: still describes `WheelSink` as a
  velocity sink, still names the deleted `WheelState`, and does not mention
  `planner/` at all. Should be rewritten as part of this work.
