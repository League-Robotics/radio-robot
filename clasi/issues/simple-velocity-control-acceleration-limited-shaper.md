---
status: pending
---

# Simple velocity control: a swappable acceleration-limited velocity shaper

## Description

Implement the acceleration-limited velocity control described in
[docs/design/simple-velocity-control-guide.md](../../docs/design/simple-velocity-control-guide.md):
an acceleration/braking slew-rate limiter on the commanded velocity, with
goal-aware braking (slow down as the stop point approaches), plus a modest
wheel-speed loop underneath. Add the guide's **derived acceleration limits** to
the robot configuration and encapsulate the shaping in **one class with an
`update`-style method interface**: feed it the current status (current
commanded velocity, target velocity, measured elapsed time, and — for
goal-aware braking — current position and the stop position), and it returns
the next velocity to command.

The design is deliberately **swappable**. Two interchangeable behaviors behind
the same seam:

- **Off (passthrough):** target velocity becomes the command immediately — the
  robot lurches to full speed. This is exactly today's behavior in
  [App::Drive](../../src/firm/app/drive.cpp) after the sprint-115 gut (`setTwist()`
  stages the raw target, `tick()` converts it with no shaping).
- **On (shaped):** the target is routed through the `VelocityShaper` and the
  command ramps under the configured acceleration/braking limits, braking early
  enough to stop on the goal.

You should be able to cut the shaper out entirely or add it back with a config
flag (or equivalent seam), with no other code change.

Naming rules apply to all new code: types UpperCamelCase, functions/variables
lowerCamelCase, **no units in any identifier** — units go in `// [unit]` tags
(see `.claude/rules/coding-standards.md`).

## Motivation / context

Sprint 115 gutted the motion stack (executor / pilot / heading-source / Ruckig)
down to a minimal controlled-speed base. `App::Drive` is now a **pure velocity
follower** with **no acceleration limiting at all**
([drive.cpp:10-18](../../src/firm/app/drive.cpp#L10)): a body twist goes straight
through `BodyKinematics::inverse()` into `setVelocity()` every tick. Good, honest
minimal base — but every velocity change is a step, which stresses traction,
encoder tracking, and the terminal stop.

The guide is the right first re-introduction of shaping on that minimal base:
one small, well-understood slew limiter + goal-aware braking, not a restoration
of the deleted planner/executor machinery. Per the guide's own analysis, at the
recommended limits the required traction is 14–27% of the estimated drive
traction (~4.46 m/s² ceiling from the measured breakaway data), so these are
conservative *operating* limits, not traction limits.

## Proposed design

### 1. The `VelocityShaper` class (the core)

A small, self-contained object implementing the guide's velocity slew-rate
limiter. **One update method.** Every MOVE is bounded by a stop condition
(time | distance | angle) — the set point has no hold-velocity-forever command
(bare TWIST leaves the wire), so the shaper **always** operates inside a bounded
move: there is always a defined end to shape toward, never an absent goal. The
command velocity is caller-held state; the shaper is a pure function of
`(command, target, remaining, dt)`.

The shaper and the move's `StopCondition` are two views of the same bounded
move: the `StopCondition` computes how much is left and decides *when* to stop;
the shaper brakes against that **same remaining** so the velocity reaches zero
*on* the stop point instead of overshooting it and getting hard-zeroed there.

```cpp
namespace App {

// VelocityShaper -- acceleration-limited velocity command with goal-aware
// braking (docs/design/simple-velocity-control-guide.md). Move the commanded
// velocity toward the target by no more than the accel/brake limit permits over
// the MEASURED elapsed time; never overshoot; for a spatial stop condition,
// brake early enough to reach zero on the goal.
struct VelocityShaper {
  float accel = 0.0f;     // [mm/s^2] max increase in |command| toward target
  float brake = 0.0f;     // [mm/s^2] max decrease in |command| toward target
  float maxSpeed = 0.0f;  // [mm/s] ceiling on |command|; 0 = unbounded

  // Returns the next command velocity.
  //   v_allowed   = sqrt(2*brake*|remaining|)   // room left to stop on the goal
  //   v_effective = clamp(target, -v_allowed, +v_allowed), then clamp to maxSpeed
  //   dv          = clamp(v_effective - command, -brake*dt, +accel*dt)
  //   return        command + dv
  // `remaining` -- signed distance to the active move's stop point, from its
  // StopCondition (distance move: commanded - |path|; angle move, on the omega
  // axis: commanded - |dHeading|).
  float update(float command, float target, float remaining, float dt) const;
  //           [mm/s]         [mm/s]         [mm] signed      [s] -> [mm/s]
};

}  // namespace App
```

- **Spatial stop condition (distance / angle):** the guide's stopping-distance
  logic (§"Position moves: brake before the goal") caps the target to
  `sqrt(2*brake*|remaining|)`, then slews (§"The core velocity shaper"):
  `dv = clamp(v_effective - command, -brake*dt, +accel*dt)`. Ramps up under
  `accel`, ramps down under `brake`, reaches zero on the goal; handles
  forward/reverse and asymmetric accel vs brake; cannot overshoot.
- **Temporal stop condition (time):** there is no spatial distance to brake
  against, so the shaper accel-limits the ramp-up only and holds the commanded
  velocity; the `StopCondition` ends the move at the deadline. (Whether a time
  move should *also* brake to zero at the deadline — `v_allowed = brake *
  t_remaining` — is a planner decision; the simplest correct behavior is
  accel-limit + hard-stop-at-deadline, matching a "run at this velocity for
  this long" move.)
- **Use measured `dt`**, not an assumed 20 ms — the guide is explicit, and the
  loop's cycle time is not exactly constant. `RobotLoop` already tracks cycle
  timing to source it.
- **Seamless chaining (spec) is a known boundary:** ramping to zero on every
  spatial goal is single-move point-to-point; when a next MOVE is queued the
  spec wants a seamless handoff, not a stop. Blending the shaper across chained
  moves is the guide's deferred "full trajectory generator" territory — out of
  scope here. A chained move simply re-targets and the shaper ramps from the
  current velocity: safe and correct, just not time-optimal. Flag for the
  planner.
- One scalar shaper is the primitive; the body twist composes it per axis
  (one instance/state for `v_x`, one for `omega`, and a future one for `v_y`
  once the base is holonomic — differential ignores `v_y`, per the 115
  wire-forward decision). Keep the primitive scalar so it is trivially unit-
  testable against the guide's worked examples.

### 2. The swap seam in `App::Drive`

`Drive` gains the retained command state and a shaping mode:

- Hold `vxCommand_`, `omegaCommand_` (and future `vyCommand_`) — the shaper's
  carried state.
- `tick()` takes the measured elapsed time: `tick(dt)`.
- **Off:** command = target (identity) — byte-for-byte today's follower: the
  velocity lurches to the target and the move's stop condition hard-zeros it at
  expiry (overshooting a spatial goal).
- **On:** command = `shaper.update(command, target, remaining, dt)` per axis,
  then `BodyKinematics::inverse(vxCommand_, omegaCommand_, ...)` as now — the
  velocity ramps and, for a spatial stop condition, reaches zero on the goal.
- Selected by a config flag (recommended) so it flips at runtime with no
  recompile — off restores the exact current lurch-to-speed behavior for A/B.

`remaining` comes from the **active move's `StopCondition`** — the same
`App::Odometry::pathLength()` baseline the distance/angle conditions measure
against (per the set-point issue's execution model). Because stop conditions
only exist once the **MOVE protocol** lands (sprint-115 S2 /
`protocol-set-point-...`), this shaper naturally sequences **with or after** that
work — it has nothing to shape toward before then. It is not a separate motion
stack; it is one small class `Drive` consults between the active move's velocity
and the kinematics.

### 3. Derived acceleration limits in configuration

The guide's recommended initial limits, in the codebase's existing units
(mm/s², rad/s) and mapped to the config quantities that **already exist** in
`data/robots/*.json` `control` (they were consumed by the now-deleted
executor/planner, so they are currently dormant):

| Guide limit | Value | Config field | Current JSON value | Reconcile to |
|---|---|---|---|---|
| wheel acceleration | 1.0 m/s² | `a_max` | 800 mm/s² | **1000 mm/s²** |
| wheel braking | 1.2 m/s² | `a_decel` | 800 mm/s² | **1200 mm/s²** |
| max lateral accel | 1.0 m/s² | *(new — `v_y`, holonomic future)* | — | 1000 mm/s² (defer/ignore for differential) |
| max yaw acceleration | 4.0 rad/s² | `max_rot_accel_dps2` | 600 deg/s² (≈10.5 rad/s²) | **≈229 deg/s²** (4.0 rad/s²) |
| max yaw rate | 1.5 rad/s | `yaw_rate_max` | 70 deg/s (≈1.22 rad/s) | **≈86 deg/s** (1.5 rad/s) |
| body speed ceiling | (guide sets none) | `v_body_max` | 1000 mm/s | keep |

Recommendation: **reuse the existing dormant fields** rather than add parallel
duplicates (`a_decel` already *is* "brake"), reconciling their values toward the
guide. Two constraints the planner must resolve:

- **The plumbing for these fields was deleted with the planner.** `a_max` &c.
  mapped into `msg::PlannerConfig` (boot bake via `gen_boot_config.py`, live
  `SET` keys) — all removed by the 115 gut. So the derived limits need a **fresh,
  minimal config carrier** (a small `ShaperConfig`, or fields folded into the
  existing `DrivetrainConfig`), **not** the deleted `PlannerConfig`. Whatever
  carrier is chosen must follow sprint-114 config-as-truth: values come from
  `data/robots/*.json`, **no behavioral defaults baked in source**, fail-closed
  when unconfigured. See `sprint-114-config-as-truth-and-deadband`.
- **Provenance.** Preserve the guide's derivation data (robot mass 0.550 kg,
  driven-wheel load 0.450 kg, longitudinal/lateral breakaway 3 N / 4 N →
  coefficients 0.56 / 0.74) alongside the values so a future retune has the
  measured basis, not just magic numbers.

### 4. Wheel-speed loop (already present)

The guide's control stack is `target → accel limiter → velocity PI → PWM`. The
velocity PI already exists as [Devices::velocity_pid](../../src/firm/devices/velocity_pid.h)
under the motors; this issue adds the **accel-limiter stage in front of it** at
the body-twist level. No change to the PI loop is required for the shaper itself
(the guide's PI-tuning guidance is separate, existing work).

Explicitly **out of scope** (guide's "optional" / "when justified" sections):
jerk limiting and a full trajectory generator (Ruckig) — the guide recommends
adding jerk limiting only if testing exposes a real need, and the S-curve/Ruckig
path is what 115 just removed.

## Verification

Per `.claude/rules/hardware-bench-testing.md` (robot on the stand, wheels free):

1. **Unit tests** against the guide's worked examples: 0.5 m/s at 1.0 m/s² takes
   ~12.5 updates at 40 ms; asymmetric accel/brake; forward↔reverse; a spatial
   goal caps the target to `sqrt(2*brake*remaining)` and reaches zero within
   `remaining` with margin (no overshoot); a distant goal leaves the ramp
   unclamped until braking range.
2. **Sim:** a step twist command ramps under the limits instead of stepping;
   toggling the shaper off reproduces the immediate-lurch follower exactly.
3. **Bench (shaper off):** confirm current behavior is byte-for-byte unchanged
   (pure A/B baseline).
4. **Bench (shaper on):** command a twist step and watch the streamed telemetry
   (per-cycle frame) show the commanded velocity ramp, encoders following; the
   ramp spans ~10+ cycles for a 0.4 s ramp; no wheel slip / no brownout at the
   recommended limits. Reverse and pivot likewise.
5. **Goal-aware (once distance-MOVE exists):** a distance-bounded move
   decelerates before the goal and stops on it without the terminal
   step-then-brake, in both directions, short and long moves.
6. **Config-as-truth:** the derived limits come from the active robot JSON; a
   live config patch changes the ramp rate without reflash; unconfigured →
   fail-closed (no silent source default).

## Related

- [docs/design/simple-velocity-control-guide.md](../../docs/design/simple-velocity-control-guide.md)
  — the algorithm and the derived-limit derivation this issue implements.
- `gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry`
  — this shaper is a build-out **on** that minimal base; the goal-aware path
  couples to that issue's S2 MOVE distance stop condition. Schedule after the
  gut lands.
- `protocol-set-point-the-minimal-firmware-s-complete-command-surface` — the
  MOVE contract whose `distance` stop condition feeds `stepToGoal()`.
- `sprint-114-config-as-truth-and-deadband` (memory) — the config-as-truth
  constraints the new limit carrier must satisfy (no source defaults, version
  erase, fail-closed).
- `predict-to-now-odometry-estimator-...` — the fuller motion build-out on the
  minimal base; this shaper is the small first step of it, not a substitute.
