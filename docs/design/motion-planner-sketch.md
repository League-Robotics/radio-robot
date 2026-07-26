# Sketch: the standalone Motion Planner

**Status:** REVIEWED by stakeholder 2026-07-25 — interface shape,
discrete-exact profiler, estimation/filtering approach, and decisions
1-2 below are settled; §8's remaining questions stay open. Not yet an
issue/sprint.
**Companions:**
`clasi/issues/motion-library-development-kickoff-parallel-effort.md`,
`clasi/sprints/124-.../issues/robot-state-blackboard-one-struct-for-all-shared-state-and-telemetry.md`,
`docs/design/base-explicit-loop-sketch.md`,
`src/motion/DESIGN.md`.

## 1. What we are building, in one paragraph

A single subsystem, `Motion::Planner`, that owns everything between "a
`Move` arrived" and "here are the wheel velocity targets for the next
50 ms": the move queue and lookahead, acceleration-limited velocity
profiling, encoder/OTOS state estimation, and odometry. It is sequestered:
host-buildable with zero hardware dependency, zero `src/firm` dependency
except the `types/` directory (`RobotState`), tested entirely on the host
— fast C++ unit/scenario tests plus a ctypes shared library driven from
Python. Its correctness bar: **in a zero-error simulation the motion is
exact** — a 500 mm distance move travels exactly 500 mm; a 90° spin turns
exactly 90°; a chain of moves leaks zero error across boundaries.

## 2. The interface — the whole interface

```cpp
namespace Motion {

class Planner {
 public:
  explicit Planner(const PlannerLimits& limits);

  // Moves are handed in as they arrive (no request/signal handshake).
  // replace=true preempts the active Move and flushes pending, as today.
  // Returns false when the queue is full (caller acks ERR_FULL).
  bool move(const Move& next, bool replace);

  // Flush everything, command zero. (STOP.)
  void stop();

  // DO THE WORK. Reads the sensor and time sections of `state` (wheel
  // samples with their timestamps, OTOS sample, `state.time`) and runs
  // the whole pipeline — velocity filter, predict-to-now estimation,
  // odometry, lookahead, one profile step of the active Move — holding
  // every result internally. Const: tick() cannot touch the blackboard.
  // Returns the cycle's completion event, if any — at most one Move can
  // end per tick, and the caller needs its id + timed-out flag to emit
  // the wire completion ack (protocol v4 §7.2). Returned, not saved
  // into RobotState: acks are protocol bookkeeping, not robot state
  // (the 124 blackboard issue's own rule).
  // TickResult { bool completed; uint32_t moveId; bool timedOut; } —
  // same shape MoveQueue::tick() returns today.
  TickResult tick(const RobotState& state);

  // SAVE. Writes the results tick() computed into `state`: the pose
  // and estimate sections, and the command section (wheel velocity
  // targets for the next interval, active flag). The ONLY Planner
  // method that mutates RobotState. Called after tick(), at whatever
  // point the loop's publish schedule wants the planner's section to
  // become visible (the blackboard's publish-when-coherent rule).
  void update(RobotState& state) const;
};

}  // namespace Motion
```

Construction takes limits (`PlannerLimits`: per-axis velocity, accel,
decel ceilings, control period, queue depth is a compile-time constant);
everything else flows through `RobotState`. No held clock, no held
devices, no callbacks. `state.time` is the clock.

Two deliberate properties:

- **Compute and publish are separate methods with opposite constness**
  (stakeholder, 2026-07-25): `tick(const RobotState&)` does the work and
  provably cannot write the blackboard; `update(RobotState&) const`
  saves the computed results and provably computes nothing new. The
  compiler enforces the blackboard discipline — exactly one mutation
  point, placed by the loop wherever its publish schedule wants it. A
  test can tick against a frozen state and inspect what WOULD be
  published, or diff state before/after update() for golden
  comparisons.
- **`Move` is a plain motion-owned struct** (`Motion::Move`: axis
  {Distance, Angle, Time, Wheels}, target, cruise velocity, timeout).
  Today's `move_queue.h` includes `messages/envelope.h` for `msg::Move` —
  that dependency goes away; the base's dispatch converts wire
  `msg::Move` → `Motion::Move` at the boundary. This matters for the
  sequestration goal: the standalone build then needs *only* `types/`.

### RobotState is the data interface

Per the sprint-124 blackboard issue: `RobotState` lives in
`src/firm/types/robot_state.h`, dependency-free, trivially copyable.
The planner reads `wheelLeft/wheelRight` (position, velocity,
sampleTime, connected), `otos`, and `time`; it writes `pose`,
`estimate`, and the command fields (`wheelLeft.cmdVelocity`,
`wheelRight.cmdVelocity`, `command.moveActive`, …).

Until 124's ticket lands that header, the motion tree carries its own
copy conforming to the issue's sketch (in the standalone build's include
path only), swapped for the real one at the joint checkpoint — same
strategy the kickoff issue prescribes for the observer input. Because
the struct is plain data, the swap is a delete-one-file operation, and
the same layout is directly mirrorable as a `ctypes.Structure`.

## 3. The profiler — discrete-exact trapezoid, no margin constants

This is the core new design, and it replaces the reactive
`VelocityShaper` + land-at-zero margin-factor machinery for deciding
motion (`kStoppingMarginFactorChain` 0.48 / `Orthogonal` 0.67 /
`Final` 0.92 — three swept fudge constants and their residuals). Those
margins exist because a *continuous* braking formula (v²/2a) was bolted
onto a *discrete* 20 Hz control loop. The fix is to plan in the loop's
own discrete time, where exactness is natural.

**Model.** The planner emits one velocity target per control interval,
held constant for the whole interval (ZOH — zero-order hold, "hold the
last value until the next sample"; the same term describes the
estimator's extrapolation in §4). With a perfect velocity plant,
distance advanced in interval k is exactly `v_k · dt`. So a Move of
distance D is exact iff the emitted sequence satisfies
`Σ v_k · dt == D`. That is an accounting problem, not a control problem
— and accounting can be exact.

**Policy (greedy max-feasible velocity).** Each tick, with distance-to-go
`r` (from the estimate; in perfect sim, exact) and previous command `v`:

1. Compute the candidate ceiling: `vReach = min(vCruise, v + aMax·dt)`.
2. Choose the largest `vNext ≤ vReach` such that landing at the boundary
   velocity `vEnd` remains feasible under the decel limit **in discrete
   steps**: `r − vNext·dt ≥ brakeDistance(vNext, vEnd)`, where
   `brakeDistance` is the exact staircase sum
   `Σ_{i=1..n} max(vEnd, vNext − i·aDecel·dt) · dt` — not v²/2a.
   (Closed form with one small correction term; O(1), no search loop
   needed in the common case.)
3. **Terminal step:** when `r` is reachable in one interval within the
   decel limit, command exactly `vNext = (r − remaining-after-this-tick
   plan) / dt` — i.e. the last interval's velocity is chosen so the sum
   closes to D exactly. The feasibility invariant maintained in step 2
   guarantees this final value is always within `aDecel·dt` of the
   previous command, so the terminal step never violates the limits.
4. After the terminal step, command `vEnd` (0, or the next Move's
   carried velocity) and declare the Move complete.

Properties:

- **Exact in zero-error sim, by construction.** No prediction constants
  to sweep, nothing to re-tune when dt or the limits change.
- **Self-correcting on hardware.** `r` is re-read from the estimate
  every tick, so disturbance error drains through the same policy —
  the profile is replanned from measured reality each interval, not
  played open-loop.
- **The "start ramping down a few cycles early" behavior the
  stakeholder asked for falls out of step 2**: the planner holds cruise
  until the tick where holding it once more would make landing
  infeasible, then decelerates — the earliest-necessary, latest-possible
  ramp, computed rather than scheduled.
- Accel-limited now; a jerk limit is a later additive stage on
  `vReach`/`aDecel` (explicitly not Ruckig — stakeholder 2026-07-25).

**Lookahead / boundary velocity.** The planner owns the queue, so at
activation (and re-check each tick, since the queue can grow mid-Move)
it sets `vEnd`:

- No pending next Move → `vEnd = 0`.
- Next Move continues the same axis, same direction → `vEnd =
  min(|next.cruise|, what the next Move can itself brake from)` with
  carried sign — full-speed chaining, no dip.
- Next Move is orthogonal or opposed → `vEnd = 0` (land at rest, then
  the next Move ramps itself up).

This subsumes M2 (same-axis carry / no-dip floor) as a natural output of
the profiler rather than a special-case reset rule.

**Axes.** One profiler instance runs the active Move's commanded axis:

- Distance → profile in path space, wheel targets `v, v` (heading hold
  on the uncommanded axis is M3, later — the slot is the uncommanded
  axis's target, currently pinned to its commanded value or 0).
- Angle → profile in heading space, wheel targets `±ω·b/2` via
  `BodyKinematics::inverse()` — exact θ iff wheel distances are exact.
- Time → constant cruise until elapsed (still ramped in/out by the same
  policy with `vEnd` from lookahead; completion by clock).
- Wheels → per-wheel profiles, same policy applied twice.

Timeout backstop, replace/flush semantics, ERR_FULL, and the queue depth
carry over from `MoveQueue` unchanged in behavior.

## 4. Estimation — stale samples in, predict-to-now out

The 20 Hz loop plus slow encoder refresh means `tick()` can never trust
raw samples as "now." The refresh figure on record is **~80 ms**
(the Nezha brick's 0x46 register vs. a ~16 ms collect cadence —
`src/firm/devices/DESIGN.md`, `nezha_motor.cpp`), but note its
provenance: bench-observed during the freshness-gate bug fix (~4 of 5
collects returned the same raw count), never formally characterized —
and the related flip-flop-cadence estimate was off 2× until measured
(architecture-update-086). Nothing below depends on the exact value
(freshness is detected per-sample, not assumed periodic), but it should
be measured before tuning the filter — see §7. The estimation stage of
`tick()` produces a coherent current-instant estimate:

1. **Per-wheel velocity filtering.** Raw encoder velocities are very
   noisy (stakeholder, 2026-07-25); the base-loop sketch deletes the
   motor-side EMA/least-squares A/B machinery (inventory item 9) and
   assigns velocity conditioning to the estimation layer — i.e. here,
   where it is host-testable. v1 is an EMA per wheel with two rules:
   - **Filter only on fresh samples.** The encoder refreshes far
     slower than the loop (~80 ms on record — see the provenance note
     above), so many cycles re-see the SAME sample. Feeding a repeated
     stale sample back into the EMA silently re-weights old data; the
     filter advances only when `sampleTime` changes.
   - **Filtered velocity never enters the distance accounting.** The
     smoothed velocity is used for ZOH extrapolation (predict-to-now)
     and as the profiler's current-speed input; traveled distance is
     always anchored to measured *positions* (cumulative counts — the
     encoder's integral is far cleaner than its derivative), so filter
     lag can delay a velocity estimate but can never accumulate
     distance error. In zero-error sim the filter converges to the true
     constant velocity and exactness (§3) is unaffected.

   The EMA weight is a `PlannerLimits` field. If EMA lag proves too
   costly at these sample rates, the upgrade path is a small
   command-model observer (predict from commanded velocity, correct on
   fresh samples — the same shape the post-124 base-side observer uses),
   not a stack of additional filters.
2. **Per-wheel predict-to-now.** From each wheel's last fresh sample
   position and the filtered velocity, extrapolate distance to
   `state.time` (ZOH). Post-124, the base-side wheel observer
   (contract: `base-explicit-loop-sketch.md`) hands up estimates already
   conditioned by the command model; this stage then normalizes rather
   than filtering/extrapolating itself. The design tolerates both — the
   estimate section's ZOH bases (value, velocity, basisTime) are the
   interface either way.
3. **Odometry.** Integrate predicted wheel deltas through
   `BodyKinematics::forward()` into pose — the existing `Odometry`
   module, fed predicted rather than raw positions.
4. **OTOS fusion.** v1 complementary blend on heading/omega with
   fail-closed weights (existing `StateEstimator` design); position arm
   is M5. Staleness-gated by the OTOS sample's own timestamp.
5. **Save — `update()`'s job, not tick()'s.** The computed pose and
   estimate stay planner-internal until `update(state)` writes
   `state.pose` and `state.estimate` (ZOH bases, so "predict to t" is a
   free function over the state — a consumer holding a copied
   `RobotState` gets extrapolation for free, per the 124 issue),
   alongside the command section.

The profiling stage of `tick()` then computes distance-to-go against the estimate predicted to
the epoch the command takes effect. The one-cycle actuation floor (a
command staged this cycle drives the wheels next cycle) is a constant,
known latency — the planner predicts forward by `actuationDelay` (a
`PlannerLimits` field, default one period) so the profile is planned
against the state at the moment it will actually apply. In zero-error
sim with delay 0 this is the identity; on hardware it is the difference
between landing on the number and landing one cycle's travel past it.

## 5. What the planner emits — velocity now, duty at the M4 checkpoint

Output is **wheel velocity targets** written into `RobotState`, tracked
by the base's existing velocity PID. The kickoff issue pins the post-124
contract as per-wheel **duty** out (PID moves into motion). That change
is confined to a thin back-end stage of `tick()` — profile → wheel
velocities → (PID) → duty — everything in §3/§4 is unchanged by it. We
build velocity-out now, and the M4 joint checkpoint swaps the back end
when the base's duty primitive and observer exist. The planner's
interface doesn't change shape: it writes whichever command fields the
contract of record says.

## 6. Relationship to the existing `src/motion` modules

| Existing | Fate |
|---|---|
| `MoveQueue` | Queue/lifecycle bookkeeping absorbed into `Planner` (internal); land-at-zero margin predicates **deleted**, replaced by §3. `msg::Move`/`messages/envelope.h` dependency dropped. |
| `VelocityShaper` | Superseded on the profiled axis by §3's planner (which is feed-forward + limits, not reactive clamping). May survive as the rate limiter for non-primary axes / manual-twist mode. |
| `StopCondition` | Time/timeout backstop survives; Distance/Angle comparisons become the profiler's own accounting (completion is emitted by the profile closing, §3.4). Hardware-truth settle confirmation (M1's measured-state gate) layers on as a config: `requireSettle` → completion additionally waits for \|remaining\| ≤ ε and \|v\| ≤ ε within a bounded window. In sim, profile-complete and settle-complete coincide. |
| `Odometry` | Kept as-is, fed predicted wheel positions (§4.2). |
| `StateEstimator` | Math kept; `Input` struct dies — reads/writes `RobotState` (per the 124 issue, its query API dissolves into free functions over the state's ZOH bases). |
| `BodyKinematics` | Kept as-is (drops the `msg::BodyTwist3` overload OR that type moves to `types/` — the last non-`types` firm include in the tree). |

## 7. Testing — host-only, three tiers (plus one bench measurement)

**Bench prerequisite (one-time, before filter tuning):** characterize
the encoder refresh interval properly — robot on the stand, constant
commanded velocity, log timestamped raw 0x46 collects, histogram the
intervals between raw-count *changes* (per wheel, at 2-3 speeds). The
~80 ms figure is bench-observed folklore, not a characterization (§4);
the measured distribution sets the EMA weight, the expected staleness
window, and the noise model the scenario tests inject. This is the only
hardware touch in the whole plan and it produces a dataset, not a
dependency — everything below stays host-only.

1. **C++ unit tests** (standalone `motion_tests` CMake, no Python in the
   loop): profiler policy (feasibility invariant, terminal-step
   exactness, lookahead vEnd selection), velocity filter (fresh-sample
   gating, convergence under injected noise, lag bound), estimator
   predict-to-now, odometry integration — each against hand-fed numbers.
2. **C++ scenario tests**: `Planner` + the model plant
   (`TestSim::WheelPlant`) with **zero error injected** — the exactness
   gates: single distance Move lands `error == 0` (to float
   accumulation, asserted at ≤ 1e-6 mm); 90°/±180° spins exact; chained
   same-axis pair leaks 0; orthogonal chain leaks 0; replace/stop/
   timeout semantics. Then the same scenarios with injected noise/lag
   asserting bounded (not exact) error — the self-correction property.
3. **Python harness via ctypes**: build `libmotion` as a shared library
   with a flat C API (`plannerCreate(limits)`, `plannerMove`,
   `plannerTick(const RobotState*, TickResult*)`, and
   `plannerUpdate(RobotState*)` — same compute/save split as the C++
   surface), `RobotState` mirrored as a
   `ctypes.Structure`. This gives: interactive scenario scripts,
   profile plots (velocity staircase vs. distance-to-go), parameter
   sweeps, and — once 124's telemetry-as-RobotState-stream lands —
   **offline replay of recorded bench telemetry through the real
   planner**, the exact enabler the blackboard issue names.

The goal-doc bars (`docs/design/goal-exact-tours.md`) govern: M1-tier
numbers proven on the model plant in milliseconds per run; the full-sim
gate (M6) re-measures the same numbers when the library re-links into
`libfirmware_host` — divergence between the two is a base/boundary bug
by definition.

## 8. Decisions and open questions

**Settled (stakeholder, 2026-07-25):**

1. **Modules stay separate inside the motion tree.** `Planner::tick()`
   is the composition the standalone build and tests use; the base loop
   may call the same modules individually if it wants the dataflow
   loop-visible. Both entry paths stay valid; pick the loop's shape
   before M6.
2. **The name is `Motion::Planner`.** The base-loop sketch's
   `Motion::Controller` spelling is superseded — update that doc's
   `motion_` line when it is next touched.

**Still open:**

3. **Completion semantics on hardware**: profile-complete (§3.4) with
   optional measured-settle confirmation (§6, M1) — is settle-confirm
   the default ON for hardware, OFF for sim? Proposal: a
   `PlannerLimits` field, default on.
4. **`Motion::Move` shape**: carry over `msg::Move`'s fields 1:1 (kind,
   threshold, cruise, timeout, id) or take the chance to simplify?
   Proposal: 1:1 minus wire baggage, so dispatch conversion is trivial.
5. **Does `BodyTwist3` move into `types/`** or does `BodyKinematics`
   drop the array overloads? (Last remaining non-`types` firm include.)
