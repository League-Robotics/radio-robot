---
status: pending
filed: 2026-07-25
filed_by: team-lead (stakeholder-directed, parallel motion effort)
related:
- motion-library-development-kickoff-parallel-effort.md
---

# Motion planner: Motion::Planner — discrete-exact profiling, estimation, host-only development

This issue is **self-contained**: everything needed to build the planner is
spelled out below. Background docs exist (`docs/design/motion-planner-sketch.md`,
`docs/design/base-explicit-loop-sketch.md`) but nothing below requires
reading them.

## 1. What this is

A single subsystem, `Motion::Planner`, that owns everything between "a
Move arrived" and "here are the wheel velocity targets for the next 50 ms
control interval": the move queue and lookahead, acceleration-limited
velocity profiling, encoder/OTOS state estimation, and odometry.

Hard rules:

- **Host-only.** Lives in `src/motion/planner/`, builds standalone with
  its own CMakeLists (`cmake -S src/motion/planner -B src/motion/planner/build
  && cmake --build src/motion/planner/build --target planner_tests`). No
  firmware headers, no `msg::*`, no devices, no clock objects — every
  time-taking function takes an explicit `now` and all state flows through
  the `RobotState` struct below.
- **Exactness bar:** in a zero-error simulation (perfect velocity-tracking
  plant), motion is EXACT — a 500 mm distance Move travels 500.000 mm
  (≤ 1e-3 mm error), a 90° rotation turns exactly 90° (≤ 1e-5 rad), a
  chain of Moves leaks zero total error across boundaries. Achieved by
  planning in discrete time (below), with **no tuned margin constants**.
- **Never edit** `src/firm/**` or `src/motion/state_estimator.*` /
  `src/motion/move_queue.*` etc. from this effort — sprints 124-126 own
  those in the main environment. New files under `src/motion/planner/`
  only. Work on the `motion-planner` branch.
- C++17, Google style with project overrides: UpperCamelCase types,
  lowerCamelCase functions/variables (never PascalCase functions),
  trailing-underscore members, snake_case filenames. **No units in
  identifiers** — units go in a leading bracketed comment tag:
  `float velocity = 0.0f;  // [mm/s] signed`.

## 2. Current status — v1 is DONE and passing

Commit `4aea58c1` on the `motion-planner` branch contains a complete,
tested v1 of everything in §3-§6 below. 3 ctest suites pass (profile
sweep, estimation units, 11 end-to-end zero-error scenarios); the Python
ctypes harness (`python3 src/motion/planner/py/planner_harness.py`)
measures 0.011 µm landing error on 500 mm and 0.014 arcsec on a 90° turn.
Sections 3-6 document what EXISTS (so agents can verify/extend it);
section 7 is the remaining work to sprint.

## 3. The data contract: RobotState and the planner types

`src/motion/planner/types/robot_state.h` — a plain, dependency-free,
trivially-copyable struct (no pointers, no heap; directly mirrorable as a
Python `ctypes.Structure`). It is a MIRROR of the sprint-124 blackboard
header; at the joint checkpoint it is deleted and the build points at
`src/firm/types/robot_state.h`. Sections the planner touches:

```cpp
struct RobotState {
  struct Time { uint32_t cycleStart, cycleBusy, cyclePeriod; } time;  // [ms]
  struct Wheel {
    float position;     // [mm] sensed cumulative travel (input)
    float velocity;     // [mm/s] sensed, raw -- NOISY (input)
    uint32_t sampleTime;  // [ms] when the sensed pair was collected (input)
    bool connected;
    float cmdVelocity;  // [mm/s] planner OUTPUT (written by update())
  } wheelLeft, wheelRight;
  struct Otos { bool present, connected;
    float x, y, heading, vx, vy, omega; uint32_t time; } otos;  // input
  struct Pose { float x, y, heading, vx, vy, omega; } pose;     // output
  struct Estimate {  // output: ZOH bases (value+velocity+basisTime+valid)
    BodyBasis body; WheelBasis wheelLeft, wheelRight;
    float innovationHeading, innovationOmega; bool innovationsValid;
  } estimate;
  struct Command { bool moveActive; uint32_t activeMoveId; } command;  // output
  // (perception/health sections exist but the planner ignores them)
};
```

Planner-owned value types (`planner_types.h`, all C-layout):

```cpp
struct Move {
  enum class Kind : uint8_t { Time, Distance, Angle };
  enum class VelocityKind : uint8_t { Twist, Wheels };
  uint32_t id;
  Kind kind;
  float threshold;  // [ms] Time / [mm] Distance / [rad] Angle; POSITIVE
  float timeout;    // [ms] safety backstop; 0 = none
  VelocityKind velocityKind;
  float v_x, v_y, omega;  // [mm/s] [mm/s] [rad/s] signed cruise, Twist
  float vLeft, vRight;    // [mm/s] signed cruise, Wheels
};
// Direction comes from the velocity SIGN; thresholds are magnitudes.
// v_y carried but ignored on differential (§11 Decision 1, 2026-07-26).

struct PlannerLimits {
  float vMax, aMax, aDecel;            // [mm/s] [mm/s^2] [mm/s^2]
  float omegaMax, alphaMax, alphaDecel;  // [rad/s] [rad/s^2] [rad/s^2]
  float trackWidth;       // [mm]
  float controlPeriod;    // [ms] (50 on the real robot)
  float actuationDelay;   // [ms] command-staged-to-effective latency
  float velocityFilterWeight;  // EMA weight on fresh samples; 1 = unfiltered
  uint32_t otosStaleness;      // [ms]
  float headingOtosWeight;     // [0..1] fail-closed default 0
  bool requireSettle;          // §7.2, added 2026-07-25
  float settleWindow;          // [ms]
  float headingHoldGain;       // [1/s] §7.4, fail-closed default 0
};

struct TickResult {
  bool completed; uint32_t moveId; bool timedOut;
  bool settled;   // §7.2 — always evaluated, see §10
};
```

## 4. The Planner interface — exact semantics

```cpp
class Planner {
 public:
  static constexpr int kQueueDepth = 5;  // 1 active + 4 pending
  explicit Planner(const PlannerLimits& limits);
  bool move(const Move& next, bool replace);
  void stop();
  TickResult tick(const RobotState& state);
  void update(RobotState& state) const;
  // observability: active(), pendingCount(), activeMoveId(),
  // commandedLeft(), commandedRight()
};
```

- **`tick(const RobotState&)` — DO THE WORK.** Reads sensor + time
  sections, runs the whole pipeline (filter → predict-to-now → odometry →
  lookahead → one profile step), holds every result internally. The
  `const` parameter means the compiler proves it cannot write the
  blackboard. Returns the completion event (at most one Move can end per
  tick). `state.time.cycleStart` is the clock.
- **`update(RobotState&) const` — SAVE.** Writes tick()'s results into
  the state: `wheelLeft/Right.cmdVelocity`, `command.*`, `pose`,
  `estimate`. The method-level `const` means it provably computes nothing
  new. The ONLY method that mutates RobotState. Caller order per cycle:
  `tick(state)` then `update(state)`.
- **`move(next, replace)`**: queues a Move as it arrives (no handshake).
  Activation happens on the NEXT tick() (that's where time/pose baselines
  live). `replace=true` drops pending AND active; the replacement
  activates next tick. Returns false (queue provably unchanged) when
  active+pending would exceed 5 — the caller acks ERR_FULL. Also returns
  false for invalid shapes: Distance with `v_x == 0` (including a
  pure-sideways `v_y`-only Move — §11 Decision 1), Angle with
  `omega == 0`, negative threshold, or (v1) a Wheels Move whose kind is
  not Time.
- **`stop()`**: flush everything, command zero immediately (safety
  semantics — no ramp).
- **Completion events** are RETURNED, never written into RobotState (acks
  are protocol bookkeeping, not robot state). `timedOut` is set when the
  timeout backstop fired rather than the stop condition.
- With no active Move, tick() ramps the staged commands to zero at
  `aDecel` per interval ("drain").

## 5. The profiler — discrete-exact trapezoid (the core algorithm)

**Model.** The planner emits ONE velocity per control interval, held
constant (zero-order hold) for the whole interval. With a perfect
velocity plant, distance advanced in interval k is exactly `v_k · dt`.
So a Move of distance D is exact iff `Σ v_k · dt == D` — an accounting
problem. All profile math runs in the POSITIVE frame (normalize by the
cruise-velocity sign, re-apply the sign to the output).

**`brakeDistance(v, boundary, decel, dt)`** — the minimum distance
consumed getting the command from `v` down to `boundary` (the velocity to
land at), as the exact discrete staircase, NOT v²/2a:

- `delta = decel·dt` (per-interval step).
- If `v <= boundary + delta`: return 0 (one allowed step reaches boundary).
- Else `m = ceil((v − boundary)/delta) − 1` steps strictly above boundary,
  velocities `v−delta, v−2·delta, …`; sum
  `= (m·v − delta·m(m+1)/2)·dt`.

**`profileStep(remaining, previous, cruise, boundary, limits, dt)`** —
one policy step, returns `{velocity, closing}`:

1. `r = max(0, remaining)`;
   `ceiling = min(cruise, vMax, previous + aMax·dt)`;
   `floor = max(0, previous − aDecel·dt)`; if ceiling < floor, ceiling =
   floor (cruise dropped below reachable — brake toward it).
2. **Exact terminal step:** `landing = r/dt`. If landing is within
   `[floor, ceiling]` AND `landing <= boundary + aDecel·dt` (the command
   AFTER landing is `boundary`, so the hand-off itself must respect the
   decel limit — without this clause the post-landing stop violates the
   limit and drain drift adds unaccounted distance), emit
   `{landing, closing=true}`. This one interval closes the sum to
   `remaining` EXACTLY.
3. Else feasibility: `feasible(v) ⇔ r − v·dt >= brakeDistance(v,
   boundary, aDecel, dt)`. If `feasible(ceiling)`, emit ceiling (the
   policy is max-feasible, i.e. time-optimal within the constraint set).
4. Else if `!feasible(floor)` emit floor (infeasible entry, e.g. a
   replace-at-speed into a short Move: brake at the full decel ceiling
   every tick; overshoot is physics, bounded by the entry velocity's own
   braking distance).
5. Else bisect (≈48 iterations) between floor (feasible) and ceiling
   (infeasible) and emit the feasible side. Bisection precision does not
   affect exactness — the terminal step closes the sum; it only affects
   time-optimality by < 1 tick.

Invariant (why overshoot never happens on a feasible profile): choosing a
feasible v this tick leaves `r' >= brakeDistance(v)`, and
`brakeDistance(v) = (v−delta)·dt + brakeDistance(v−delta)`, so the floor
choice is feasible next tick by construction.

**`maxEntryVelocity(distance, boundary, limits, dt)`** — the largest v
with `v·dt + brakeDistance(v, boundary, aDecel, dt) <= distance` (bisect
from vMax). Used for lookahead.

**Lookahead / boundary velocity.** For the ACTIVE Move, the landing
velocity `boundary` is:

- No pending next Move → 0.
- Next Move continues the same axis, same direction (linear: active
  Distance/Time + next Distance/Time with matching `sign(v_x)`; angular:
  active Angle + next Angle with matching `sign(omega)`) →
  `min(|next cruise|, maxEntryVelocity(next.threshold, 0, limits, dt))`
  (Time successor: no distance bound → cap is vMax).
- Otherwise (orthogonal or opposed) → 0.

**Chain-exact accounting (cumulative baselines).** On a normal (not
timed-out) completion, the NEXT Move of the same measure inherits its
baseline as `previousBaseline + previousThreshold` — "where the boundary
IS", not "where we happened to be when completion fired". Any sub-tick
crossing residual is thereby debited to the successor, so the chain's
TOTAL is exact; only the final (boundary = 0) landing needs the terminal
step. A timeout aborts the motion: no carry, baselines re-anchor to the
current pose. Distance carries `baselinePath + threshold`; Angle carries
`baselineHeading + sign(omega)·threshold`.

**Completion detection** (checked at the top of tick, BEFORE planning,
against last tick's plan):

- Time: `elapsed >= threshold`.
- Distance: `remaining <= 1e-3 mm` (float-noise epsilon, NOT a motion
  margin — the terminal step already landed exactly), OR, when the
  planned boundary velocity is > 0 (carry case),
  `remaining <= profileVelocity·dt + 1e-3` — hand off during the tick
  the crossing falls inside.
- Angle: same with `1e-5 rad`.
- Timeout: `timeout > 0 && elapsed >= timeout` → completed with
  `timedOut = true`.
- On completion, the next pending Move activates THE SAME TICK (seamless
  hand-off) and its first command is computed that tick. The profile's
  carry velocity persists across a same-axis activation and resets to 0
  on an axis change (we landed at ~0 there anyway).

**Per-kind command mapping** (differential drive, track width b):

- Distance (Twist): profile in path space; both wheels get
  `dir · velocity`.
- Angle (Twist): profile in heading space with the angular limits;
  wheels get `∓ omega·b/2` (left negative for positive omega).
- Time (Twist): both axes ramp toward cruise at `aMax`; when the
  remaining ticks `(threshold − elapsed)/period` are just enough to
  reach the boundary at `aDecel` per tick, step toward it (time-domain
  taper). Linear axis may carry into the next Move; angular lands at 0.
- Wheels (Time-bounded only at v1): per-wheel time-domain ramp, linear
  limits, boundary 0.

**Measurement side of `remaining`:** Distance uses
`pathLength + |bodyVelocity|·actuationDelay − baselinePath`; Angle uses
`(heading + omega·actuationDelay − baselineHeading)·dir`. The
actuation-delay term plans against the state at the moment the command
takes effect (one-cycle staging latency on the real robot; delay 0 in
the zero-error harness).

## 6. Estimation (sense side, runs first inside tick)

1. **Per-wheel EMA velocity filter** (`WheelChannel`): raw encoder
   velocities are very noisy. The filter advances ONLY when `sampleTime`
   changes (measured 2026-07-26: the register itself is live at <=16 ms —
   see §7.3 — but through the loop a sample can still repeat, so
   most cycles re-see the SAME sample; re-feeding it would silently
   re-weight old data). First fresh sample seeds. Filtered velocity
   feeds extrapolation and the profiler's current-speed input; traveled
   distance is ALWAYS anchored to measured positions (the encoder's
   integral is far cleaner than its derivative), so filter lag can never
   accumulate distance error.
2. **ZOH predict-to-now**: `position(t) = anchorPosition +
   filteredVelocity · (t − anchorTime)`.
3. **Arc-exact odometry** (`PoseTracker`), fed the PREDICTED positions:
   per step `ds = (dLeft+dRight)/2`, `dTheta = (dRight−dLeft)/b`;
   straight (`dTheta == 0`): `x += ds·cos(θ)`, `y += ds·sin(θ)`; else
   radius `R = ds/dTheta`, `x += R·(sin(θ+dTheta) − sin θ)`,
   `y += −R·(cos(θ+dTheta) − cos θ)`; `θ += dTheta`;
   `pathLength += |ds|`. Pure rotation (dLeft = −dRight) adds zero path
   length.
4. **OTOS heading blend** (v1): when `otos.present`, age ≤
   `otosStaleness`, and `headingOtosWeight > 0`:
   `heading += weight · shortestWay(otosHeading − heading)` (wrap via
   `remainder(·, 2π)`).

## 7. Remaining work (sprint scope — each item self-contained)

**Status as of 2026-07-25.** Items 1, 2 and 4 are DONE (see §10 below for
what landed, including two estimation defects the noise tier exposed).
Item 3's tooling is written but the measurement is UNRUN — no robot is
attached to this checkout. Items 5 and 6 remain blocked on the joint
checkpoint, item 7 remains deferred, and item 8 is still open.

1. **Noise/lag scenario tier.** Extend the test plant to inject:
   (a) zig-zag/white velocity noise on published samples, (b) stale
   sample cadence — publish a FRESH sample only every N-th cycle
   (N = 2 — a degraded-mode repeated-sample stretch; measured
   2026-07-26, the register is live at ≤ 16 ms and fresh-every-cycle is
   the expected regime, see §7.3), repeating the
   previous sample (same `sampleTime`) between, (c) `actuationDelay =
   50 ms` — the plant applies the PREVIOUS tick's command. Assert:
   distance/angle scenarios complete with bounded error (gate: ≤ one
   cruise interval's travel, i.e. `cruise·0.05` mm, tighter after
   tuning), limits still respected, no oscillation at the goal (command
   monotonically → 0 after landing).
2. **Settle-confirm completion option (M1).** Add `bool requireSettle`
   + `float settleWindow  // [ms]` to PlannerLimits. When on, a
   Distance/Angle Move that has reached profile-complete additionally
   waits until `|remaining| <= epsilon` AND `|measured body
   velocity| <= epsilonV` (suggest 1.0 mm / 5 mm/s linear; 0.005 rad /
   0.05 rad/s angular) before reporting completion, up to settleWindow;
   past the window, complete anyway with `timedOut = false` (report
   truthfully — add a `settled` bool to TickResult). In the zero-error
   sim, settle-complete and profile-complete coincide on the same tick
   (test that).
3. **Bench measurement — encoder refresh interval. DONE 2026-07-26 —
   and the answer overturns the premise.** A dedicated bench firmware
   (`src/tests/firmware/encoder_rate/` — own CODAL CMake project, raw
   0x46 protocol, no driver conditioning) tight-polled both encoders on
   the stand. Findings (full write-up:
   `docs/design/encoder-refresh-characterization.md`; firmware + raw captures in `src/tests/firmware/encoder_rate/`):
   - **The ~80 ms folklore is FALSE.** The register returned a fresh
     value on EVERY poll at 16/24/32 ms periods, single-port,
     alternating ports, with duty writes before the select, and at 8%
     duty. No latch quantization in the count deltas. The register is
     live at ≤ 16 ms.
   - **The historical staleness was a loop-schedule artifact:** a 0x10
     transaction interposed between the 0x46 select-write and its read
     returns raw 0, 416/416 times (phase F) — the driver's glitch
     rejection then reads that cycle as "stale." The pre-118 loop
     schedule interposed exactly like that; the current interleaved
     schedule keeps the window clean.
   - Select latch: the brick holds ONE pending select, last wins
     (phase G) — ordering matters, data stays valid.
   Consequences: plan for FRESH samples at the 50 ms loop cadence
   (fresh-sample EMA gating stays correct — it keys on `sampleTime`);
   the noise tier's stale-cadence emulation becomes a degraded-mode
   test, not the expected regime; the angular `settled` concern below
   (encoder quantum, not refresh) still stands. The telemetry-based
   capture half of `src/motion/planner/bench/encoder_refresh.py` is
   superseded for this question but remains useful to CONFIRM per-cycle
   freshness through the real loop's own telemetry. Base-side follow-up
   (main repo): correct the "~80 ms" claims in
   `src/firm/devices/DESIGN.md`, `nezha_motor.{h,cpp}`, and
   `docs/design/base-explicit-loop-sketch.md` items 4/7.
4. **Heading hold on Distance Moves (M3).** A P controller on the
   uncommanded axis: during a Distance Move, `omegaCorrection = kHeading
   · (baselineHeading − heading)` (gain in PlannerLimits, default 0 =
   off), added as a differential `∓ omegaCorrection·b/2` on top of the
   profiled wheel commands, clamped so the faster wheel never exceeds
   vMax. Test: inject a one-off heading disturbance in the plant;
   assert the heading returns to baseline and the DISTANCE stays exact
   (the correction is differential, ds unchanged).
5. **RobotState joint checkpoint** (after sprint 124 lands in the main
   repo): delete `types/robot_state.h`, point the build at
   `src/firm/types/robot_state.h`, reconcile field names/offsets, rerun
   everything including the ctypes layout guard.
6. **Duty-plane back end (M4). LIBRARY SIDE DONE 2026-07-26 — validated
   co-located on a MEASURED plant.** The velocity PID now lives in the
   planner's output stage (`wheel_pid.{h,cpp}`; `Planner::stageDuty()`
   on every tick exit; `commandedDutyLeft/Right()` + `plannerDuty` capi;
   gains `PlannerLimits.velKff/velKp/velKi/velIMax`, fail-closed zeros;
   rest clamp: exact-zero target + wheel near rest → duty 0 + integral
   reset, which killed a measured ~1° reverse-creep from post-settle
   integral unwind). Real plant identified on the stand
   (`bench/plant_id.py`, firmware PID reduced to kff, open-loop duty
   steps): **gain ~1370 mm/s/duty (L 1420 / R 1320), tau ~230 ms**,
   brick duty slew visible. `tests/duty_plant.h` encodes those numbers
   (+0.07 mm encoder quantization + velocity noise) and
   `planner_duty_scenarios_test.cpp` proves solid one-loop motion on it:
   distance 500 → ±0.17 mm; 90° turn → +0.89°; 4-move chain →
   748.97/750 mm with full-speed carry; stop-means-stopped; zero creep.

   **Topology finding (HIL, hard-won):** closing this loop HOST-SIDE
   over the serial transport is unstable (~6 cycles dead time +
   integrator windup across it — runaway legs, sign reversals;
   `hil_drive.py --duty` preserves the experiment). The loop MUST run
   co-located with the samples — which the validated test topology is.

   **What remains is FIRMWARE INTEGRATION (the 125 replan's scope):**
   (a) link the planner into the ARM image (the root CMake currently
   excludes `src/motion/planner/**`; the exclusion must narrow to
   tests/capi/bench only), (b) RobotLoop's cycle calls
   `planner.tick(state)`/`update(state)` after both collects and writes
   the planner's DUTY to the motors — sense → plan → act in ONE cycle,
   duty writes after the plan, (c) NezhaMotor sheds its PID (pure bus
   adapter + raw duty primitive), (d) delete MoveQueue/VelocityShaper +
   land-at-zero constants once the planner carries the load, (e) wire
   Moves to the planner's queue. Remaining tuning frontier: the settle
   rest gate admits up to alphaDecel·dt of residual omega, whose coast
   (~omega·tau) is the turn's +0.9°.
7. **Wheels-Move stop conditions beyond Time** — only if protocol demand
   materializes (v1 deliberately rejects them).
8. **Open design decisions to settle:** final `Motion::Move` field shape
   (1:1 with wire `msg::Move` minus baggage vs simplified), and whether
   `BodyTwist3` moves into `types/` or the kinematics array overloads
   are dropped.

## 8. Testing requirements (all host, no hardware except item 7.3)

- **Unit tier**: profile (brakeDistance staircase values; sweep distances
  × cruises × limit sets from rest, assert per-step limit respect and
  exact landing ≤ 1e-4 mm in the pure-accounting loop; max-braking
  behavior on infeasible entry; cruise actually held on long runs),
  estimation (fresh-sample gating: a repeated `sampleTime` must not
  advance the EMA; convergence under injected noise; ZOH predict; arc
  odometry straight/rotation; OTOS wrap-seam blend).
- **Scenario tier** (Planner + perfect plant, closed loop:
  stamp time → tick → update → integrate commands over one interval →
  publish fresh samples): distance forward/backward exact ≤ 1e-3 mm;
  90° turn exact ≤ 1e-5 rad; same-axis chain total exact AND crossed at
  speed (after reaching cruise, speed never dips more than one decel
  step below cruise before the first completion); orthogonal chain both
  exact; timeout reported with timedOut; stop() flushes and zeroes;
  replace preempts (replacement exact from its own baseline); queue full
  rejects the 6th and frees a slot only on COMPLETION (activation alone
  does not free — 1 active + 4 pending is still 5); invalid shapes
  rejected; Time Move runs its duration and tapers to zero.
- **ctypes tier**: `libmotionplanner` shared lib with a flat C API
  (`plannerCreate/Destroy/Move/Stop/Tick/Update` + `plannerStructSizes`
  layout guard); Python mirror asserts C++ `sizeof` == `ctypes.sizeof`
  for every struct before use; reruns the distance + turn exactness
  scenarios from Python.
- `planner_tests` (ctest custom target) builds and runs everything,
  nonzero on failure.

## 9. Ground rules recap (see §10 for what landed)

Parallel effort: this checkout, `motion-planner` branch, CLASI
out-of-process (`clasi oop on`). Sprints 124-126 run in the main
environment and own `src/firm/**` and the legacy `src/motion/*.{h,cpp}`
modules — never edit those here. Boundary-header changes need sign-off
from both efforts once 124 lands.

## 10. What landed (2026-07-25)

### Done

**§7.1 — noise/lag scenario tier.** New suite
`src/motion/planner/tests/planner_noise_test.cpp` (13 scenarios) over a new
`TestPlanner::NoisyPlant` in `tests/test_support.h`, with each real-world
defect independently switchable so a failure names its own cause:
velocity-sample noise (alternating + deterministic pseudo-random), stale
sample cadence (`sampleDivisor`, fresh sample every N-th cycle), one-cycle
actuation latency, first-order velocity **tracking lag**, encoder
**position quantisation**, and an unmodelled **creep** drive. Every
scenario asserts the per-tick invariants throughout (never past `vMax`,
never a step past the accel/decel ceiling — angular-aware — and a
monotonically shrinking command envelope after the goal, i.e. no hunting).

Measured on the standard dirty plant (40 mm/s zig-zag + 15 mm/s white
noise, 100 ms sample cadence, 50 ms actuation latency, 0.5 mm encoder
quantum, wheel closing 80% of a velocity step per interval), against the
§7.1 gate of one cruise interval:

| scenario | error | gate |
|---|---|---|
| distance 500 mm forward | 2.000 mm | 7.5 mm |
| distance 300 mm backward | 1.500 mm | 7.5 mm |
| 90° turn | 0.0265 rad | 0.100 rad |
| chain 500 + 300 mm, total | 2.000 mm | 7.5 mm |

**Where the residual error actually is** — `testTrackingLagSensitivity()`
sweeps it and asserts it monotone: of every defect above, only the wheel
failing to reach its commanded velocity is outside the planner's model,
and it shows up as a pure *overshoot* (the coast after the last command,
which a velocity sink cannot take back).

| tracking lag | 1.0 | 0.9 | 0.8 | 0.7 | 0.6 | 0.5 |
|---|---|---|---|---|---|---|
| overshoot [mm] | +0.400 | +1.100 | +2.000 | +3.500 | +5.000 | +7.575 |

At lag 1.0 the dirty plant still lands sub-millimetre — everything else is
reconstructed or anchored away. Closing the rest needs the plant model
that arrives with §7.6 (velocity PID inside the planner's output stage);
until then it is characterised, not tuned away.

**§7.2 — settle-confirm completion (M1).** `PlannerLimits::requireSettle`
+ `settleWindow`, and `TickResult::settled`. `settled` is **always**
evaluated and reported truthfully; `requireSettle` only controls whether
completion is *deferred* to obtain it. Deferral applies only to a
Distance/Angle Twist Move landing at rest — never to a timeout (which
aborts the motion) and never when the Move hands off at speed into a
same-axis successor (there is nothing to settle at a carried boundary, and
deferring would stall the chain; tested).

The gates are the issue's suggested tolerances (1.0 mm / 0.005 rad
arrival) with one change: the rest gate is `|velocity| <= max(floor, one
decel step)`, floor 5 mm/s / 0.05 rad/s. A body within one decel step of
zero is *provably* at rest by the end of the next interval, and the
profiler's own terminal step is capped at exactly one decel step above the
boundary — which is what makes settle-complete and profile-complete land
on the **same tick** in a zero-error plant, as §7.2 requires (verified in
both C++ and the ctypes harness: both complete on tick 76).

The arrival residual is measured against the **anchored** (un-extrapolated)
odometry, not the planned one — "have we actually arrived?" must not be
asked of a prediction.

On a dirty plant `settled` behaves as a real signal rather than a
formality: a well-tracked wheel (0.5 mm overshoot) confirms; the standard
plant (2.0 mm overshoot) refuses to claim arrival and completes at window
expiry with `settled = false`, `timedOut = false` — a distinct fact from a
timeout, and worth its own bit. Window expiry defers by exactly the window
(6 ticks of a 300 ms window) and no longer.

Note for hardware: the angular arrival epsilon (0.005 rad) is **finer than
one encoder tick reads** on a 100 mm track with a 0.5 mm quantum
(= 0.01 rad of heading). A coarse encoder alone can keep an otherwise
perfect turn from confirming — which is a concrete reason §7.3's
measurement matters before this gate is trusted on the robot.

**§7.4 — heading hold on Distance Moves (M3).**
`PlannerLimits::headingHoldGain` (`[1/s]`, fail-closed default 0). P on the
uncommanded axis back to the Move's activation heading, applied as a
differential on top of the profiled pair and clamped so the faster wheel
never passes `vMax` — clamped symmetrically about the profiled velocity, so
the *mean* of the pair (which is what the odometry integrates as `ds`) is
untouched and **the distance stays exact**. A 0.20 rad kick mid-Move is
driven out to < 1e-5 rad while the 500 mm lands to within 0.06 µm; with the
gain at its default the same kick is carried to the end untouched (the
control that proves the recovery is the P term). An absurd gain against a
1.0 rad error stays inside `vMax`. Exercised from Python too.

### Two estimation defects the noise tier exposed (both fixed)

1. **The odometer ratcheted on noise.** The pose was integrated from
   ZOH-*extrapolated* wheel positions, and `pathLength` accumulates `|ds|`
   — which rectifies zero-mean jitter into one-way drift. A *standing*
   robot's odometer climbed at roughly the velocity noise times the period,
   every tick, forever. This also contradicted §6.1's own stated principle
   ("traveled distance is ALWAYS anchored to measured positions"). The pose
   is now integrated from the measured anchors, and the extrapolation is
   applied as a non-accumulating additive term in `measure()`. Regression
   test: `testStandingRobotDoesNotDrift()` — 400 idle ticks on the dirty
   plant, 0.000000 mm of drift. This alone took the dirty-plant distance
   error from 0.677 mm to 0.060 mm and the chain from 0.862 mm to 0.125 mm.

2. **The anticipation terms extrapolated the future from a dirty
   derivative.** The ZOH-predict and actuation-delay terms used the filtered
   *measured* velocity. Under the velocity-tracking plant the profiler is
   built on, the *commanded* velocity says the same thing and is exact —
   and it matters most on the angular axis, where per-wheel noise does not
   cancel the way it does in the mean. Both spans now use the command, each
   attributed to the command actually driving it (the elapsed span to the
   tick-before-last when there is staging latency, the future span to last
   tick's), via a one-deep command history. This took the dirty-plant turn
   error from 0.040 rad to 0.026 rad and, with settle-confirm, from
   0.020 rad to 0.0003 rad. Measured-velocity anticipation was tried and
   measured *worse* as well as noisier; the failure mode it might have
   caught (a plant that does not track) is bounded by one sample interval
   and fully corrected by the next anchor, because the pose is anchored.

`update()` now stamps `estimate.body.basisTime` at the older of the two
wheel anchors rather than at the tick, since that is when the pose is
actually valid, and marks it invalid until both wheels have a sample.

### Not done, and why

**§7.3 — bench measurement: RUN AND ANSWERED 2026-07-26** (the robot was
attached to this checkout at the time), via a dedicated bench firmware
rather than the telemetry path — see §7.3's own updated text and
`docs/design/encoder-refresh-characterization.md`. Headline: the register is
live at ≤ 16 ms; the ~80 ms folklore was interposed-traffic sample
invalidation under the pre-118 loop schedule. `src/motion/planner/bench/
encoder_refresh.py`'s telemetry capture remains useful as a THROUGH-THE-
LOOP freshness confirmation (expect: fresh every 50 ms cycle on the
current schedule); set `velocityFilterWeight` and the noise tier's
`sampleDivisor` accordingly (staleness emulation = degraded mode now,
not the expected regime).

**§7.5 (RobotState joint checkpoint)** and **§7.6 (duty-plane back end)**
remain blocked: both wait on sprint 124 landing in the main environment,
and §7.6 is explicitly "do NOT start" before both efforts sign off the
boundary.

**§7.7 (Wheels-Move stop conditions beyond Time)** remains deferred — no
protocol demand has materialised.

**§7.8 (open design decisions)** — SETTLED by the stakeholder 2026-07-26.
See §11.

### Verification

`cmake --build src/motion/planner/build --target planner_tests` — 4/4 ctest
suites pass (`profile_test`, `estimation_test`, `planner_scenarios_test`,
`planner_noise_test`). `python3 src/motion/planner/py/planner_harness.py` —
ctypes layout guard passes against the widened `PlannerLimits`/`TickResult`,
and all four scenarios pass (500 mm to 0.011 µm; 90° to 0.014 arcsec; the
settle and heading-hold scenarios added this round). Every pre-existing
zero-error exactness gate is unchanged and still passing — no regression.

## 11. Design decisions (stakeholder, 2026-07-26)

§7.8's two open questions are settled, plus a third this effort raised.
Only the first is implementable before sprint 124 merges in from the other
repo; the other two are staged and MUST NOT be started here until it does.

### Decision 1 — `Motion::Move` stays flattened, but carries `v_y`. DONE.

The wire's two `oneof`s (`envelope.proto`'s `Move`) stay flattened into
enum-tagged fields, and `replace` stays a `move()` parameter rather than a
field: the planner's types stay drivetrain- and wire-agnostic, and a
protocol-v5 schema change touches one conversion function in the base's
dispatch instead of the planner's internals.

`v_y` is now carried (`planner_types.h`), for two reasons: a holonomic
drivetrain then needs no type change, and `.claude/rules/
naming-and-style.md` rule 2 says a twist HAS three components. It is
accepted and ignored on this differential build, exactly as the wire's own
`MoveTwist.v_y` is.

One guard came with it: a Move whose ONLY commanded linear velocity is
sideways (`v_x == 0`, `v_y != 0`) asks for a motion this drivetrain cannot
make, and `move()` REJECTS it rather than accepting it and silently
driving nothing. `v_y` riding along on an otherwise valid Move is accepted
and ignored. Both tested in `testInvalidMovesRejected()`.

### Decision 2 — drop the `msg::BodyTwist3` kinematics array overloads. BLOCKED.

`src/motion/body_kinematics.h`'s array-form `inverse(msg::BodyTwist3, ...)`
/ `forward(..., msg::BodyTwist3&)` overloads exist only to carry a wire
message type, which is precisely the coupling the planner is designed to
avoid. They go; the scalar forms (plain floats, no dependencies) stay as
the shared API. No new `BodyTwist3` is added to `src/motion/planner/types/`.

**Not actionable from this effort.** `src/motion/body_kinematics.*` is
legacy-motion, which §9 forbids editing here, and sprint 124 is being
finished against it in the other repo — editing it now guarantees a merge
conflict. Do it AFTER the 124 merge lands, and at the same time consider
re-pointing the planner's four lines of inlined diff-drive math
(`planner.cpp`'s Distance/Angle/Time command mapping) at the surviving
scalar forms, so `saturate()`'s curvature-preserving logic and the
planner's heading-hold clamp stop being two takes on the same idea.

### Decision 3 — reserve a protocol-v5 flags bit for `settled`. BLOCKED, TIME-SENSITIVE.

`TickResult::settled` (the Move met its stop condition AND physically
arrived at rest, vs. stopped near it — see §10) becomes host-visible: a new
completion-ack flags bit alongside the existing timeout bit 15. `ack_err`
stays 0 in both cases; these are outcome facts, not errors.

**Not actionable from this effort** — it changes `src/protos/` and the
completion-ack contract, which the main environment owns and which §9 says
needs sign-off from both efforts. **Raise it with the 124/125 effort now**:
while protocol v5 is still open this is a free bit reservation; once
124/125 land it is a protocol change.

Caveat to carry into that conversation: on the real robot `settled` may be
unreliable on the ANGULAR axis until §7.3's bench measurement runs. The
arrival epsilon is 0.005 rad, but one encoder tick reads as
`quantum / trackWidth` of heading — 0.010 rad for a 0.5 mm quantum on a
100 mm track — so a coarse encoder alone can stop an otherwise perfect turn
from ever confirming. Reserving the bit is still right (it costs nothing
and the window is now); trusting the bit on hardware waits on §7.3.

### Sequencing

1. **Now:** Decision 1 — done.
2. **Waiting:** sprint 124 finishes its hardware tests in the other repo
   and merges into `motion-planner`.
3. **After the merge:** Decision 2, then §7.5 (the RobotState joint
   checkpoint — delete `types/robot_state.h`, point the build at
   `src/firm/types/robot_state.h`, rerun the ctypes layout guard).
4. **In parallel, needs the robot on the stand:** ~~§7.3's encoder-refresh
   measurement~~ (DONE 2026-07-26 — see §7.3), and a velocity step
   response to replace the noise tier's guessed `trackingLag = 0.8` with
   a measured one (§10 shows tracking lag is now the ENTIRE residual
   error).
5. **Cross-effort, start the conversation now:** Decision 3.


## HANDOFF STATE (2026-07-26, end of parallel-checkout session)

Branch `motion-planner` in the parallel checkout
(`/Volumes/Proj/proj/RobotProjects/radio-robot-motion`), tip `28ceb6ab`,
sitting ON TOP of sprint 124's closed state (merged from the main repo).
Working tree clean. 6/6 test suites + ctypes harness green.

**What the library now is:** the complete one-loop motion stack --
discrete-exact profiler (jerk-limited S-curve accel side), estimation
(fresh-gated EMA + filter-lag-compensated feedback, wrap-safe time math,
telescoping signed Distance measure), M4 duty stage (WheelPid with accel
feedforward, ramp-gated integral, rest clamp), M1 terminal settle
(bidirectional rate-limited creep, per-robot rest floors), heading hold,
cumulative chain baselines. Plant constants MEASURED on the robot
(gain ~1370 mm/s/duty, tau ~230 ms; `bench/plant_id.py`).

**Reference result** (`bench/square_tour_sim.py`, schedule-faithful sim,
10% L/R gain mismatch): 8/8 moves settled, tour closes +0.96 mm /
+0.09 deg, peak +8.9% rounded (from +25% sharp).

**For the 125 replan (firmware integration, §7.6):** link the planner
into the ARM image (narrow the root-CMake planner exclusion to
tests/capi/bench), RobotLoop calls tick/update after both collects and
writes planner DUTY (sense->plan->act one cycle), NezhaMotor sheds its
PID, delete MoveQueue/VelocityShaper + margin constants, route wire
Moves to the planner queue. Named frontiers: deadbeat discrete
feedforward (the residual +8.9% hump), settle-window tightening via
better profile hand-off, `kMinWriteIntervalUs` must scale with kCycle,
serial frame budget (protocol-v5 lean frame) before raising loop rate.
