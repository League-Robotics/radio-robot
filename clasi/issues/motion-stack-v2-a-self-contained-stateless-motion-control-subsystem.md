---
status: pending
---

# Motion Stack v2 — a self-contained, stateless motion-control subsystem

## Context

The current motion pipeline (Motion::SegmentExecutor: 3-phase machine + divergence
replans + dead-time-projected stops) fails audit against the canonical model:
feedback is implemented as replanning tuned by ~12 gain-dependent constants;
translation terminates on prediction, not state; the plan is knowingly infeasible
(v_body_max=1000 vs ~400 sim / ~620-740 real plateau); segments carry no pose and no
boundary velocities; the PoseEstimator is constructed but never ticked. The one true
outer loop ever added (sprint 098 heading PD, kp=6) is the only motion that lands
reliably (±1° turns on hardware) — this design generalizes it.

**Stakeholder decisions (2026-07-12):**
- **Self-contained subsystem**: the entire motion-control system lives in its own
  directory, refers to nothing outside it (copying code where needed), and is
  driven through an exceedingly thin adapter. It must be traceable in one place and
  thoroughly testable from Python.
- **Stateless**: the subsystem holds no hidden state. The planner is a pure
  function; the plan is immutable data; every step call is fed the current time,
  pose, and velocity by the caller. Pose ownership is OUTSIDE the subsystem.
  (One precise, explicit exception — see "Statelessness accounting".)
- Packaging: **two sprints** — sprint 099 (pose restore, already designed) first,
  verified on the old motion stack; motion v2 follows.
- Teleop: **MOVER (deadman-velocity) cuts over inside the v2 sprint**; BLEND
  streaming merge (MOVE s=1) deferred — v2 replies ERR to stream=true until a
  follow-up sprint.
- Terminal behavior: **accept the settle trade** — state-based termination costs
  ~280 ms typical per stop segment vs the old early-fire projection; in exchange,
  zero lag-calibration constants, and terminal accuracy can only be slow, never
  wrong.
- **Two levels of control: the wheel PID stays at the motors.** The subsystem
  outputs wheel VELOCITY setpoints; the existing leaf MotorVelocityPid turns them
  into duty, unchanged. See "Two levels of control" below for the rationale.

## The prime directive: one directory, pure functions, Python-testable

**Directory: `source/drive/` — namespace `Drive`.** Rules:

1. **No references outside the directory.** The subsystem defines its own plain
   value types (Pose, Twist, WheelState, WheelVelocities, Limits — no msg::, no
   Hal::, no CODAL, no blackboard). Allowed external dependencies: libc/libm and
   the vendored Ruckig headers (a library, like <math.h>). Code that exists
   elsewhere is **copied in**, per stakeholder direction: the differential
   IK/saturation math (from kinematics/body_kinematics) and the Ruckig wrapper
   pattern (from motion/jerk_trajectory).
2. **Everything is value-in / value-out.** No member mutation anywhere in the
   subsystem; the caller owns all state as a transparent POD (`StepState`) passed
   in and returned. Any tick observed anywhere (sim, bench, field) can be replayed
   bit-for-bit in Python from its recorded inputs.
3. **The subsystem's output is wheel velocity setpoints** [mm/s], staged to the
   motors' existing velocity PIDs. The visible cascade in this directory is
   reference → trims → IK → saturation → wheel clamp → setpoints.
4. **One thin adapter** connects it to the robot stack (queues, blackboard, HAL
   staging, wire acks). The adapter contains zero control math.

### Two levels of control (decision 2026-07-12)

Motion control is explicitly TWO control levels with an observable boundary:

- **Level 1 — the motion planner/tracker (`source/drive/`)**: plans the segment,
  tracks the reference against the fused pose, and emits wheel velocity setpoints.
  Stateless (policy timers aside), pure, Python-steppable.
- **Level 2 — the wheel velocity PID (HAL leaf, unchanged)**: `Hal::
  MotorVelocityPid` in NezhaMotor/SimMotor turns setpoints into duty, exactly as
  today.

Why the PID stays at the motors rather than inside the subsystem:
- It is bench-tuned and hardware-proven (the 098 stack was exactly this shape:
  outer loop → velocity setpoints → leaf PID), and it runs at the I2C flip-flop
  cadence (~40-80 ms per motor) with its own dt, synchronized to its write
  opportunities. Pulling it into the 20 ms step() makes a *different
  discrete-time system* (multiple integrator updates per fresh encoder sample)
  requiring retune for zero functional gain.
- SimMotor deliberately runs the *identical* law — copying it into the subsystem
  forks the one control law that must never drift between sim and hardware.
- From the subsystem's perspective the plant is honestly "a velocity servo with
  ~130 ms lag" — a cleaner contract, and a simpler tier-0 Python plant model.
- **Isolation improves**: `cmd_vel` (subsystem output) and `vel` (measured) are
  both existing TLM fields. cmd_vel wrong → level-1 bug; vel failing to track
  cmd_vel → level-2/actuator bug. The boundary is thin, observable, and already
  instrumented.

## Architecture

```
                         (outside: PoseEstimator owns pose — sprint 099)
                         (outside: queues, wire, HAL staging, motor armor)
                         (outside: LEVEL 2 — leaf wheel velocity PID, unchanged)
                                        │ thin adapter
  ┌─ source/drive/ (LEVEL 1) ───────────▼──────────────────────────────┐
  │ Drivetrain (façade; immutable config only)                         │
  │   plan(PlanRequest) ──────────────► MotionPlan (immutable data)    │
  │        pure function                  │ referenceAt(t) → RefState  │
  │                                       │   (the plottable table)    │
  │   step(plan, StepInput, StepState) ─► StepOutput                   │
  │        pure function                    ├ WheelVelocities [mm/s]   │
  │          inside: reference sample →     ├ Status (done/replan/…)   │
  │          tracker trims → IK →           └ TrackRecord (full trace) │
  │          saturate → wheel clamp                                    │
  │   replan(plan, measured, t) ──────► new MotionPlan (re-timed)      │
  │   admit(goal, chainTail, limits) ─► Verdict (pure feasibility)     │
  └────────────────────────────────────────────────────────────────────┘
```

Rules that hold everywhere:
- **One trajectory per segment.** Master DOF = path length (arcs) or heading
  (pivots); the second channel is derived ω(t)=κ·v(t) — never independently solved.
- **P-only outer loops.** No integral action in any trim; no kd (encoder ω̂ is
  0-80 ms-stale staggered noise). The wheel PI owns steady-state authority.
- **No dead-time compensation in the control law.** Feedforward carries the signal;
  ~130 ms lag is bounded phase lag at these gains (kp=6 proven). kOutputHops/
  kDeadTime do not exist in v2 (enforced by a grep test).
- **No reversal write-trains, structurally.** Directional velocity band in every
  solve; one-sided wheel clamp on forward arcs; reverse = a new segment after a
  stop + dwell.
- **Errors are explicit.** Aborts surface as status + EventNotify; admission
  failures NACK at the wire. Never silent success.

## The two core headers

### `source/drive/drivetrain.h` — the planner interface (stateless)

```cpp
// drivetrain.h -- Drive::Drivetrain: the motion-control subsystem's single
// entry point. Holds ONLY immutable configuration (limits, gains, geometry)
// fixed at construction; every method is a pure function of its arguments.
// The caller owns the pose estimate, the clock, the plan object, and the
// StepState value -- this class remembers nothing between calls.
#pragma once
#include "drive/types.h"        // Pose, Twist, WheelState, WheelVelocities, Limits
#include "drive/motion_plan.h"

namespace Drive {

// Goal -- the desired outcome, relative to the start pose: one constant-
// curvature arc primitive. kappa = deltaHeading / arcLength.
//   straight: deltaHeading = 0     pivot: arcLength = 0 (exitSpeed must be 0)
struct Goal {
  float arcLength = 0.0f;     // [mm] signed path length; 0 = pivot in place
  float deltaHeading = 0.0f;  // [rad] total heading change, CCW+
  float exitSpeed = 0.0f;     // [mm/s] boundary velocity at segment end; 0 = stop
};

struct PlanRequest {
  Goal goal;
  Pose start;                 // [mm][mm][rad] world anchor (caller's estimate at start)
  float entrySpeed = 0.0f;    // [mm/s] chain-inherited (reference-continuous)
  float entryAccel = 0.0f;    // [mm/s^2]
};

enum class Verdict : uint8_t {
  OK, EXIT_UNREACHABLE, JOINT_STEP_TOO_LARGE, JOINT_SIGN_REVERSAL,
  PIVOT_NONZERO_EXIT, RADIUS_TOO_TIGHT, CEILING_INFEASIBLE, SOLVE_FAILED,
};

struct PlanResult {
  Verdict verdict = Verdict::SOLVE_FAILED;
  MotionPlan plan;            // valid iff verdict == OK
};

// ChainTail -- predicted chain state for queue-time admission; a pure value
// the CALLER carries (the adapter keeps it on the blackboard).
struct ChainTail {
  Pose pose;                  // predicted world pose at chain tail
  float exitSpeed = 0.0f;     // [mm/s]
  float kappa = 0.0f;         // [1/mm]
};

class Drivetrain {
 public:
  Drivetrain(const Limits& limits, float trackwidth);   // [mm] config, immutable

  // admit -- pure feasibility check for queueing `goal` after `tail`:
  // exit reachable within length; joint wheel-speed step v*|dKappa|*W/2
  // within cap; NO per-wheel sign reversal at nonzero joint speed; inner-
  // wheel floor for arcs entered at speed (R >= ~100mm); pivot => exit 0.
  Verdict admit(const Goal& goal, const ChainTail& tail) const;
  ChainTail advance(const Goal& goal, const ChainTail& tail) const;  // compose predicted tail

  // plan -- pure: solve ONE master jerk-limited profile (path length for
  // arcs, heading for pivots), target velocity = exitSpeed, under the
  // trim-headroom-folded ceiling
  //   v_eff = min(vBodyMax, omegaMax/|k|, (vWheelMax - headroom)/(1+|k|W/2)),
  // headroom = trimVMax + trimOmegaMax*W/2 -- wheels cannot saturate, and
  // trims keep authority at ceiling. The world goal pose is composed and
  // frozen into the plan here (replans re-aim at it; drift cannot compound).
  PlanResult plan(const PlanRequest& request) const;

  // replan -- pure: re-TIME the same anchored path from the measured state
  // (project pose onto the arc -- closed form -- re-solve master from
  // (s_meas, v_meas) to the SAME goal and exitSpeed). Never new geometry;
  // cross-track convergence stays the tracker's job. Solve failure returns
  // verdict != OK and the CALLER keeps the old plan (expected outcome for
  // asks reachable only by reversing).
  PlanResult replan(const MotionPlan& plan, const BodyState& measured,
                    float elapsed) const;  // [s]

  // planVelocity -- MOVER teleop: velocity-mode plan toward (v, omega) with
  // a deadman duration; same MotionPlan/step interface, no pose goal.
  PlanResult planVelocity(const Twist& target, float deadman,   // [ms]
                          const BodyState& current) const;

 private:
  Limits limits_;      // immutable after construction
  float trackwidth_;   // [mm]
};

}  // namespace Drive
```

### `source/drive/motion_plan.h` — the plan object and the step function

```cpp
// motion_plan.h -- Drive::MotionPlan: one solved segment. IMMUTABLE after
// planning -- pure data plus const queries. step() is a const method: all
// mutable state lives in the caller-owned StepState value. Same plan +
// same input + same state => same output, always: any tick recorded on
// sim, bench, or field replays bit-for-bit offline.
#pragma once
#include "drive/types.h"

namespace Drive {

// RefState -- the reference trajectory at one instant: THE plottable
// artifact. referenceAt(t) sampled over [0, duration] is the "show me the
// plan before anything moves" table.
struct RefState {
  float s = 0.0f;       // [mm] or [rad] master-DOF position
  float v = 0.0f;       // [mm/s] body speed along path (0 during pivot)
  float a = 0.0f;       // [mm/s^2]
  float theta = 0.0f;   // [rad] reference heading (world)
  float omega = 0.0f;   // [rad/s] = kappa * v (arc) or master rate (pivot)
  float alpha = 0.0f;   // [rad/s^2]
  float x = 0.0f;       // [mm] reference world position (closed-form arc)
  float y = 0.0f;       // [mm]
};

// StepState -- ALL mutable state in the subsystem, owned by the CALLER as
// a transparent value. Everything else is pure. Five scalars: the policy
// timers are the subsystem's ONLY state (the wheel-PID integrators live at
// LEVEL 2, the HAL leaf, per the two-levels decision). See the plan's
// "Statelessness accounting".
struct StepState {
  float dwellStart = -1.0f;      // [s] terminal tolerance first held (<0 = not held)
  float sustainStart = -1.0f;    // [s] replan-envelope first exceeded (<0 = inside)
  float lastReplan = -1.0f;      // [s] rate-limit anchor
  uint8_t replanCount = 0;       // toward the N-max abort
  bool settling = false;         // terminal state machine entered
};

struct StepInput {
  float t = 0.0f;          // [s] elapsed since plan start (caller's clock)
  BodyState measured;      // caller-maintained pose estimate + body twist
  WheelState left, right;  // measured wheel position/velocity (+validity)
  float poseStep = 0.0f;   // [mm] magnitude of an external pose-fix step
  float poseStepTheta = 0.0f;  // [rad] applied since last step (0 = none)
};

enum class Status : uint8_t {
  RUNNING,          // tracking the reference
  SETTLING,         // stop segment past T_plan: banded one-sided walk-in
  REPLAN_DUE,       // caller should invoke Drivetrain::replan and swap plans
  DONE_STOP,        // completion gate held (pose+vel tolerance, dwelled)
  DONE_HANDOFF,     // vExit != 0: exhausted AND within handoff envelope
  ABORT_TIMEOUT,    // explicit failure -- never silent
  ABORT_REPLAN_LIMIT,
};

// TrackRecord -- one step's FULL introspection row, including everything
// needed to REPLAY the step offline (the inputs) and everything needed to
// diagnose it (the intermediates). This is the wire trace payload.
struct TrackRecord {
  StepInput in;                                  // replay: the exact inputs
  RefState ref;                                  // the sampled reference
  float eAlong = 0.0f, eCross = 0.0f, eTheta = 0.0f;  // [mm][mm][rad] exact arc projection
  float vTrim = 0.0f, omegaTrim = 0.0f;          // [mm/s][rad/s] post-clamp
  float vCmd = 0.0f, omegaCmd = 0.0f;            // [mm/s][rad/s] body command
  float wheelLeft = 0.0f, wheelRight = 0.0f;     // [mm/s] post-IK/saturate/clamp setpoints
  bool trimSaturated = false;
  Status status = Status::RUNNING;
};

struct StepOutput {
  WheelVelocities command;   // [mm/s] setpoints for the LEVEL-2 motor velocity PIDs
  Status status = Status::RUNNING;
  TrackRecord record;
};

class MotionPlan {
 public:
  // --- pure queries on the immutable solve ---
  float duration() const;        // [s]
  float kappa() const;           // [1/mm]
  Pose anchor() const;           // world pose at segment start
  Pose goal() const;             // world goal, frozen at plan time
  float exitSpeed() const;       // [mm/s]
  float effectiveCeiling() const; // [mm/s]|[rad/s] the folded v_eff -- dumpable
  bool isPivot() const;
  bool isVelocityMode() const;   // MOVER teleop plan
  RefState referenceAt(float elapsed) const;   // [s] closed-form, pure

  // --- the step: const on the plan; ALL mutation in *state ---
  // reference sample -> path-frame errors (exact circle projection) ->
  // P-trims (clamped; pivot mode: v == literal 0, heading-only) -> IK ->
  // curvature-preserving saturate -> one-sided wheel clamp (forward arcs)
  // -> wheel velocity setpoints. Terminal state machine per the settle
  // spec (LEVEL 2, the leaf PID, turns setpoints into duty outside).
  // Emits REPLAN_DUE (never replans itself); large pose-fix steps bypass
  // the sustain filter; small ones reset it.
  StepOutput step(const StepInput& in, StepState* state) const;

 private:
  // Immutable solve results: master trajectory polynomial (Ruckig),
  // geometry (kappa, anchor, goal), limits snapshot. No mutable members.
};

}  // namespace Drive
```

Supporting files in the directory (each host-clean, value-typed):
`types.h` (Pose/Twist/WheelState/BodyState/WheelVelocities/Limits), `arc_math.{h,cpp}`
(composeArc, poseAlongArc, exact circle projection, wrapAngle), `master_profile.{h,cpp}`
(the Ruckig wrapper, copied pattern + solveToExit with the directional band),
`tracker.{h,cpp}` (pure trim law), `policy.{h,cpp}` (pure evaluate:
envelopes/terminal machine — returns status + next state), `drivetrain.{h,cpp}`,
`motion_plan.{h,cpp}`. No PID here — level 2 stays at the HAL leaf.

## Statelessness accounting (the stakeholder hypothesis, examined)

The hypothesis — "this system has no state at all; someone else holds the pose" —
is **correct** for the level-1 subsystem, with one small, explicit residue. The
planner, plan, reference, tracker, admission, and kinematics are all pure. The
single irreducibly historical thing is the **policy timers** (dwell start, sustain
start, last-replan, replan count, settling flag — five scalars): "tolerance held
for 150 ms" and "error sustained 200 ms" are irreducibly about history, and a
timestamp is the minimal encoding. They are a caller-owned `StepState` value, not
hidden members.

The wheel-PI integrator — the other genuinely stateful thing in motion control —
lives at **level 2** (the HAL leaf, per the two-levels decision), where state is
expected: it IS the accumulated correction that pushes through stiction, and it
cannot be derived from the current observation. It stays exactly where it is
today, unchanged and bench-tuned.

Everything else the old stack kept as hidden state is gone or externalized: no
remembered last-sample seeding (the plan is immutable; sampling is pure), no
baseline snapshots (pose is world-frame, caller-supplied), no phase machine (one
primitive per plan), no divergence bookkeeping beyond PolicyState. The pose
estimate, the clock, the plan value, and the StepState blob all live with the
caller — snapshot them and any moment of the system's life is reproducible.

## The wafer adapter (outside the directory)

`Subsystems::Drivetrain` (existing shell, thinned) becomes the ONLY bridge. Its
complete responsibility list — no control math anywhere:

- Drain queues with existing precedence (driveIn escape hatch first, replaceIn,
  segmentIn); DIRECT mode (S/wheels/neutral + governRatio) unchanged and untouched
  by the subsystem.
- Hold the current `MotionPlan` value, `StepState`, plan-start timestamp, and the
  `ChainTail` (committed to the blackboard for queue-time admission NACKs).
- Convert types at the boundary: msg::MotorState → Drive::WheelState; bb fused
  pose + BodyKinematics::forward twist → Drive::BodyState; Drive::WheelVelocities
  → msg::MotorCommand velocity staging via hardware_.motor(i).apply(), exactly
  today's path — the leaf velocity PID (level 2) and motor armor beneath are
  untouched.
- React to Status: REPLAN_DUE → call replan(), swap plan values; DONE_* → pop next
  ring segment (plan from reference-continuous entry per the handoff spec) or
  neutral the motors; ABORT_* → flush ring, re-anchor ChainTail, emit EventNotify.
- Forward the PoseEstimator's pose-step event into StepInput.poseStep.
- Commit StepOutput.record → bb.motionTrace each pass.

Estimated size: ~200 lines. The adapter is also the ONLY place the subsystem's
clock (elapsed = now − planStart) is computed from firmware time.

## Control laws and numbers (unchanged from review; now all inside step())

Trim law (Kanayama form; errors reference−measured; exact arc projection):

```
v_cmd = v_ref + clamp(k_s·e_along, ±trimVMax)
ω_cmd = ω_ref + clamp(k_θ·e_θ + k_c·v_ref·e_cross, ±trimOmegaMax)
pivot mode (|v_ref| < minSpeed): v_cmd ≡ 0.0f, ω_cmd = ω_ref + k_θ·e_θ   (= 098)
```

| Param | Initial | Basis |
|---|---|---|
| k_θ | 6.0 [1/s] | carry 098; PM ≈ 31-45° at 130-170 ms delay, hardware-proven |
| k_c | 1.5e-5 [rad/mm²] | ω_n = v√k_c ≤ 2.3 rad/s at plateau; ζ ≥ 1.3 everywhere |
| k_s | 2.0 [1/s] | PM ≈ 70°; accel transient e_along ≈ 65 mm informs envelope |
| k_d | 0 — not shipped | encoder ω̂ = stale staggered noise; 098 hit ±1° without |
| trimVMax | 120 mm/s | |
| trimOmegaMax | 1.0 arc / 2.0 pivot [rad/s] | pivot spin-up trim ≈ α·τ ≈ 1.6 rad/s must not pin |

Replan envelopes (rate-scheduled lag *allowance*, not a compensator — affects only
when we replan, never what we command):

| Parameter | Initial |
|---|---|
| e_along envelope | 40 mm + 0.25 s·\|v_ref\| |
| e_cross envelope | 35 mm flat |
| e_θ envelope | 0.15 rad + 0.20 s·\|ω_ref\| |
| trim-saturated trigger | saturated AND outside envelope |
| sustain | 200 ms · rate limit ≥ 300 ms · N-max 3 → ABORT_REPLAN_LIMIT |

Pose-fix steps: ≤30 mm/3° → absorb with trims, reset sustain timers; larger →
REPLAN_DUE immediately (bypass sustain; rate limit + N-max still apply); never
during terminal dwell (complete on pre-step basis, report honestly).

Terminal (stop segments): t ≥ T_plan → SETTLING: ω trims off; along walk-in banded
one-sided (inside tol → literal 0.0f + 150 ms dwell; outside → clamp(k_s·e_along,
50 mm/s stiction floor, 100 mm/s), never negative; overshot → 0.0f and complete).
Completion: |e_along| ≤ 10-15 mm ∧ |v̂| ≤ 15 mm/s, held 150 ms → the emitted
setpoint snaps to a literal 0.0f (the level-2 PI's integrator-freeze deadband
engages only on an exact zero). Timeout T_plan + 1.5 s → complete-with-warning
within 2× tol else ABORT_TIMEOUT.

Flying handoff (vExit≠0): exhausted AND e_cross ≤ 30 mm, |e_θ| ≤ 5°, e_along ≤
0.14·vExit + 40 mm (budgets legitimate lag). **Next plan seeds from the REFERENCE
(entrySpeed = vExit, a = 0)** — C¹ by construction; measured state gates, never
seeds (seeding measurement injects 130-220 ms of lag as phantom deceleration).
Envelope violated → replan-the-joint (same pure replan); fallback brake-to-stop +
flush + EventNotify.

## Wire schema & config (adapter-side; the subsystem knows nothing of protos)

- **motion.proto MotionSegment**: add `arc_length=14 [mm]`, `delta_heading=15
  [rad]`, `exit_speed=16 [mm/s]`, `primitive=17 [bool]`; firmware rejects
  primitive=false after cutover; host proxy decomposes legacy MOVE into ≤3
  primitives (`primitives_for_move()` in legacy_translate) + new `SEG` proxy verb
  for real arcs.
- **Plan dump**: PlanDumpRequest (envelope arm 18) → PlanRecord replies (arm 10):
  goal/anchor/v_eff/duration/exit_speed/entry_speed/replan_count (~85 B each,
  ring dump = N replies sharing corr_id). Full reference tables are dumped
  host-side by linking the same code (tier 0), not over the wire.
- **MotionTrace** (ReplyEnvelope arm 11, ~90-120 B): serialized TrackRecord
  including the StepInput replay fields; armed via StreamControl.trace at the TLM
  period; emitted from bb.motionTrace. Telemetry proto NOT extended (166/186 B).
- **EventNotify** gets a real body: seg_seq, status, e_final_pos, e_final_theta —
  unsolicited on aborts/flush.
- **PlannerConfig fields 15-31** (→ Drive::Limits via the adapter): v_wheel_max,
  steer_headroom, wheel_step_max, track_k_s/k_theta/k_cross, trim_v_max/omega_max,
  replan_err_pos/theta, replan_hold, replan_min_period, replan_max,
  handoff_tol_pos/v, arrive_vel_tol, arrive_dwell. tovez.json sources them
  (v_wheel_max ≈ 620 measured; v_body_max 1000 → ~550; generator default 350);
  gen_boot_config.py + check_config_sync.py updated. Budget check on the grown
  PlannerConfigPatch (≤ 186 B or split the patch).
- **PoseFix** (sprint 099 scope): retype stubbed envelope arm 7 → bb.poseFixIn →
  EKF history ring; PoseStepped event feeds StepInput.poseStep via the adapter.

## Testing: the four-tier ladder (isolate where problems are)

The same compiled control code runs at every tier; a failure at tier N replays at
tier N−1 because MotionTrace carries the full StepInput. The ladder localizes any
defect to the layer that first shows it.

**Tier 0 — pure Python (no firmware, no sim harness).** `source/drive/` compiles
alone into `libdrive_host` with a thin ctypes ABI (`tests/_infra/drive/drive_api.cpp`
— mirrors the proven sim_api.cpp pattern): create Drivetrain(limits), plan(),
referenceAt() table dump, step(input, state) with StepState as a ctypes struct,
replan(). Python owns the loop:
  - Plan-table tests + notebook plots BEFORE anything moves (the interpretability
    deliverable at its purest).
  - Purity/property tests: same (plan, input, state) → identical output; state
    round-trips; no NaN under fuzzed inputs.
  - **Closed-loop against a Python plant model of the level-2 velocity servo**
    (leaf PID + motor as the subsystem sees them: first-order lag 120-140 ms,
    stiction, encoder staleness 80 ms, quantization, slip — all knobs in Python):
    convergence, envelope calibration, gain sweeps, terminal walk-in, pose-fix
    steps, flying handoffs — milliseconds per run, fully plottable.
  - Replay harness: feed recorded TrackRecord.in sequences from ANY higher tier.
**Localizes:** control-law and policy defects.

**Tier 1 — firmware sim.** SimHandle (tests/_infra/sim) through the adapter: wire
envelopes in, MotionTrace + true_pose out. Validates the adapter, queue precedence,
clocking, staging, and the loop order — the things tier 0 cannot see. Runs with
the sim's motor_lag knob at 120-140 ms for all tracker/replan validation (the
default zero-lag path stays only for golden-TLM bit-exactness — validating v2 at
zero lag would repeat the 2026-07-11 false-green failure). Fault-knob matrix:
enc_slip/scale → tracker convergence vs true_pose; stiction → terminal walk-in,
no premature DONE, no reversal; trackwidth error → cross-gain corrects radius;
infeasible asks → typed ERR, queue untouched. **Localizes:** adapter/integration
defects (tier-1 failure that replays clean at tier 0 = adapter bug).

**Tier 2 — bench (robot on stand).** Same wire tooling (dual-transport capture à
la turn_sweep.py → `tests/bench/arc_sweep.py`), same MotionTrace CSVs into
tests/notebooks/out/. Arc/pivot/chain grids; MOVER deadman; OTOS coexistence
soak; plateau re-measure pinned into tovez.json as v_wheel_max; re-run the 098
pivot acceptance grid (gates the encoder→EKF heading-source switch). **Localizes:**
plant-model error (bench failure that replays clean at tier 0 with recorded inputs
= the Python/sim plant lied → fix the plant model, then the gains).

**Tier 3 — field (playfield).** Camera-verified chains via aprilcam (existing
playfield_camera_run.py pattern + geofence rules): multi-segment world-frame runs
with live PoseFix corrections — the only tier where the full
camera → EKF → tracker loop closes. Plan-vs-actual overlay: RefState polyline vs
fused vs camera ground truth. **Localizes:** pose-estimation and world-frame
anchoring defects.

Unit harnesses (tests/sim/unit pattern, C++ side, kept minimal since tier 0 covers
behavior): arc math round-trips, solveToExit boundary tuples + wrong-sign-fails-
cleanly, v_eff fold invariant (max wheel speed of sampled ref ≤ v_wheel_max −
headroom ∀t), admission verdict table, grep test that kOutputHops/kDeadTime/msg::/
MicroBit appear nowhere in source/drive/.

## Cutover (hard cut, parked window — no runtime dual stack; ~43 KB flash)

1. `source/drive/` + tier-0 Python suite land WITHOUT entering the firmware build
   — robot stays drivable on the old stack; zero flash cost until cutover.
2. Sprint 099 lands independently (old motion stack; OTOS no-hang bench gate).
3. One atomic cutover ticket: adapter rewrite, blackboard payload type, wire
   admission, host proxy decomposition, build-list swap (drive/ in;
   segment_executor/stop_condition OUT, parked on disk). Golden-TLM regeneration
   is an explicit reviewed step.
4. Parked files (segment_executor.*, segment.h, motion_baseline.h,
   stop_condition.*) deleted only after bench sign-off.

## Sprint packaging & sequencing

**Sprint A = 099 (pose restore)** — as designed; adds PoseStepped event + BodyState
publish for v2.

**Sprint B = motion v2:**

| # | Ticket | Depends | Tier |
|---|---|---|---|
| 1 | Schema + config: motion.proto 14-17, planner.proto 15-31, MotionTrace/PlanRecord/PlanDump/EventNotify, tovez.json + generators + budget check | — | host |
| 2 | source/drive/ core: types, arc_math, master_profile (solveToExit) + C++ unit harnesses | 1 | host |
| 3 | plan()/MotionPlan/referenceAt + admission (Drivetrain façade) | 2 | host |
| 4 | step(): tracker + policy + terminal machine + IK/saturate/clamp/PI cascade | 3 | host |
| 5 | drive_api ctypes ABI + tier-0 Python suite (plant model, closed loop, replay harness, plan-table notebooks) | 4 | tier 0 |
| 6 | THE CUTOVER: wafer adapter, wire admission, host proxy decomposition, golden-TLM regen | 5, Sprint A | tier 1 + HITL smoke |
| 7 | MOVER velocity mode (planVelocity + adapter replaceIn path) | 6 | tier 1 + 2 |
| 8 | Trace/plan-dump wire arms + notebook overlays | 6 | tier 1 |
| 9 | Tier-1 fault-knob matrix + lag-on validation | 6,7 | tier 1 |
| 10 | Bench: arc_sweep grids, plateau → v_wheel_max, envelope/gain tuning, 098 pivot grid re-run | 7,8,9 | tier 2 |
| 11 | Field: camera-verified chain + live PoseFix runs | 10 | tier 3 |
| 12 | Cleanup: delete parked files, reserve retired proto fields, retire heading_kp/kd + governRatio segment path | 11 | host |

## Kept / copied / gutted

**Kept outside (untouched):** the level-2 wheel velocity PID (Hal::MotorVelocityPid
in the NezhaMotor/SimMotor leaves — the two-levels decision), Hal motor armor +
NezhaHardware flip-flop timing, PoseEstimator/EkfTiny (extended per 099, stays
outside — pose is not the subsystem's job), queues/blackboard/wire, DIRECT
escape-hatch mode + governRatio, sim plant + golden-TLM constraint.

**Copied INTO source/drive/ (per stakeholder direction, so the directory is
self-contained):** differential IK/saturate math (body_kinematics), the
Ruckig-wrapper pattern (jerk_trajectory, generalized to solveToExit). Vendored
Ruckig headers are the directory's only library dependency.

**Gutted at cutover:** SegmentExecutor (3-phase machine, divergence
retarget/reanchor, dead-time projection, BLEND), MotionBaseline, stop_condition's
motion role, kOutputHops/kDeadTime and the divergence-constant family, the
infeasible-by-design v_body_max=1000 ceiling, in-firmware MOVE decomposition.
