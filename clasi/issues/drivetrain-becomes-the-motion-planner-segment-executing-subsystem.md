---
status: pending
sprint: 094
---

# Drivetrain becomes the motion planner (segment-executing subsystem)

## Context

Today motion planning lives in `Subsystems::Planner`, which owns the Ruckig
trajectory machinery and emits a `msg::DrivetrainCommand{TWIST}` that is routed
through the blackboard (`bb.driveIn`) to `Subsystems::Drivetrain`. The Drivetrain
is a thin faceplate that deliberately holds **no** motor references and receives
every setpoint as a message. The `Planner` was meant to be an *executive* but got
wrapped into *motion planning*; the message plumbing between the two is pure
overhead.

This change makes the **Drivetrain the motion planner**: it owns its wheel motors
(via references into the hardware container), owns the trajectory generator
(Ruckig, relocated from the Planner), and executes a queue of motion **segments**.
Higher-level components stay responsible for *where* to go (path selection); the
Drivetrain owns *how* — trajectory generation, kinematics, and execution.

**Prerequisite (separate ticket):** the main loop is being gutted — the
`bb.motionIn`/`bb.driveIn`/`routeOutputs` message routing, the Planner tick, and
the wire-output/telemetry emission (`clasi/issues/get-wire-output-events-telemetry-out-of-the-main-loop.md`)
move out of `Rt::MainLoop`. This plan is **Drivetrain-internal** and assumes that
gutting lands first/alongside. The seam is called out under "Integration seam."

### Locked decisions (from stakeholder)
1. **Scope** = Drivetrain-internal restructure + new command surface + state
   publishing + jerk knob. Loop routing collapse is the sibling "gut the loop" ticket.
2. **One unified segment shape** (the holonomic form): `distance` + `direction` +
   `finalHeading` + motion limits — legit for both drivetrains. A **differential**
   drive satisfies an independent `finalHeading` by **doing a rotate at the end**
   (translate, then pivot), not a coupled arc. In-place turn = `distance=0`.
3. **Consolidate on Ruckig** (`Motion::JerkTrajectory`); **retire `Motion::VelocityRamp`**.
   The `Planner` class is parked/emptied to an executive shell.

## Design

### 1. Segment — the drivetrain's only motion command (one shape)
A single POD segment struct (new, e.g. `source/messages/segment.h` in `msg::`):
`distance // [mm]`, `direction // [rad]`, `finalHeading // [rad]`, plus motion
limits `vMax // [mm/s]`, `aMax // [mm/s^2]`, `jMax // [mm/s^3]` and rotational
limits `omegaMax // [rad/s]`, `yawAccMax // [rad/s^2]`, `yawJerkMax // [rad/s^3]`.
**No duration** — it is an *output* of the Ruckig solver.

The shape is legit for both drivetrains; the executor differs by drivetrain:
- **Holonomic** drive: profile translation (`distance` along `direction`) and
  rotation (to `finalHeading`) **independently and synchronized** to finish
  together (stretch the faster channel).
- **Differential** drive (Tovez, the case this pass): decompose into phases —
  [optional pivot to `direction` if not already facing it] → translate `distance`
  (straight; heading = travel direction) → **pivot to `finalHeading` at the end**.
  Each phase is one Ruckig profile (linear for translate, rotational for pivots).
  Degenerate cases fall out: `distance==0` → pure in-place turn (pivot only);
  `finalHeading==` travel direction → no terminal pivot (plain straight).

Note: arcs (today's `R` verb) are **not** a coupled-curvature primitive anymore —
they compose as straight + pivot. If smooth arcs are needed later, that is a
follow-up, not this ticket.

### 2. Segment queue + graceful stop
A small FIFO owned by the Drivetrain (bounded ring, e.g. 8 slots, matching
`Rt::WorkQueue` sizing). Execute the head; on completion pop and start the next.
**When the queue empties with nonzero commanded velocity, synthesize a virtual
decel-to-zero segment** (Ruckig `solveToRest`/`solveToVelocity(0)` using the last
segment's limits) — the graceful stop. This replaces today's terminal-decel arming
in `Planner`.

### 3. Motion executor (relocated from Planner)
Introduce a Drivetrain-owned executor component (e.g. `Motion::SegmentExecutor`)
that wraps the two `Motion::JerkTrajectory` channels (linear + rotational),
`Motion::evaluateStopCondition`/`remainingToStop` (`source/motion/stop_condition.*`),
`Motion::MotionBaseline`, and the divergence-replan logic (`maybeReplan*`,
dead-time projection `kDeadTime`/`kOutputHops` — **preserve 093's compile-split
sim-40ms / hw-80ms dead-time**, see actuation-latency memory). This is largely a
*lift* of `planner.cpp` internals, minus the `VelocityRamp` and `GOTO` pursuit
paths (those are executive concerns; GOTO/pursuit is parked with the Planner).
The executor produces a body twist per tick; the Drivetrain converts it to wheel
targets via `BodyKinematics::inverse` and runs the existing **ratio governor**
(`Drivetrain::governRatio` — kept as-is).

### 4. Ownership: Drivetrain holds motor references
- The hardware container (`Subsystems::NezhaHardware`/`SimHardware`, base
  `Subsystems::Hardware`) keeps owning the 4 `Hal::Motor`, the shared `I2CBus`, and
  the odometer. It becomes a **container, not a ticked subsystem**.
- The Drivetrain is constructed with a reference to the container + its bound port
  pair, and resolves its two wheels via `hardware.motor(port)` (keeps runtime
  `DEV DT PORTS` rebinding working while giving direct authority). It **reads
  `MotorState` directly** from its motor refs and **writes duty/velocity targets
  directly** — the held `Hal::DrivetrainToHardwareCommand` output and the
  `bb.driveIn` mailbox go away. PID stays in the motors; ratio governor stays in
  the Drivetrain.

### 5. I2C bus seam (HARD constraint — do not restructure timing)
The Nezha flip-flop split-phase bus scheduler currently lives in
`NezhaHardware::tick()`. Per the actuation-latency knowledge, **decoupling it
naively hangs the bus** (a `0x60` duty write cannot land between a split-phase
`0x46` REQUEST/COLLECT). Recommendation: keep the flip-flop **inside the container**,
exposed as an explicit `serviceBus(now)` (rename of the current `tick`) that the
(gutted) loop calls once per pass — Drivetrain writes go through the motor refs and
are sequenced by that pump. **This ticket must not change flip-flop timing or the
80ms dead-time**; that is a separate, already-scoped follow-up
(`clasi/issues/motor-actuation-latency-flipflop-coupling.md`).

### 6. State publishing
`Drivetrain::state()` publishes **measured** encoder positions + velocities read
from its owned motors' `msg::MotorState` (`.position`/`.velocity`) — preserving the
current TLM semantic that `enc=`/`vel=` are *measured*, not the commanded
`vel_[]` targets (see `tlm_frame.h`). Telemetry (`enc=`/`vel=`) is re-sourced from
`DrivetrainState` (or a blackboard cell the Drivetrain populates), matching the
"publishes its state" goal. Populate via the loop-output drain seam from the
gut-the-loop issue, not synchronous emit.

### 7. Jerk config knob
Fields already exist end-to-end (`PlannerConfig::j_max` / `yaw_jerk_max`, the
`PlannerConfigField` enum, and the `Configurator` fold) — they migrate to whatever
config struct the executor reads (fold jerk into `DrivetrainConfig`, or keep a
motion-config sub-struct). What's missing is a **live wire key**: add `jmax` /
`yawjmax` SET/GET keys in `source/commands/config_commands.cpp` (mirror in all
three places: `applyConfigKey` SET, GET formatter, dump-list — the config sync-lint
enforces agreement). **Recommended starting defaults** (replacing today's `0.0` =
trapezoid): `jMax ≈ 5000` `// [mm/s^3]` (≈6× `aMax` 800; ~0.16s jerk-limited edges)
and `yawJerkMax ≈ 100` `// [rad/s^3]` (≈5× `yawAccMax` 20; ~0.2s) — tuned on the bench.

### Integration seam (with the gut-the-loop ticket)
- Wire motion verbs are re-parsed into unified segments and enqueued into the
  Drivetrain instead of building `msg::PlannerCommand` (handlers in
  `source/commands/motion_commands.cpp`):
  `D`→`{distance, direction=0, finalHeading=current}` (straight, no terminal pivot);
  `TURN`→`{distance=0, finalHeading=<absolute>}`; `RT`→`{distance=0,
  finalHeading=current+<relative>}`. `R` (arc) either composes as straight+pivot or
  is deferred (arcs are no longer a primitive — see §1).
- **Open item:** `S` (velocity stream) and `T` (timed) are velocity/time-bounded,
  not distance-bounded, so they don't map to a segment. Either keep a low-level
  direct-twist path (`setTwist`, dev/bench) or defer — resolve with the gut-the-loop
  ticket. Not blocking the Drivetrain restructure.

## Key files
- **New:** `source/messages/segment.h` (segment structs + kind enum);
  `source/motion/segment_executor.{h,cpp}` (or fold into drivetrain) + queue.
- **Rewrite:** `source/subsystems/drivetrain.{h,cpp}` — hold container + motor
  refs, own the executor + queue, `apply(segment)` / `enqueue` / `stop` surface,
  `state()` from measured motor state, drop held-output/`driveIn`.
- **Rename/trim:** `source/subsystems/hardware.h`, `nezha_hardware.{h,cpp}`,
  `sim_hardware.{h,cpp}` — `tick` → `serviceBus`, container role; keep flip-flop.
- **Lift then park:** `source/subsystems/planner.{h,cpp}` (motion logic → executor;
  class emptied to executive shell / parked); **delete** `source/motion/velocity_ramp.{h,cpp}`.
- **Config:** `source/commands/config_commands.cpp` (jmax/yawjmax keys),
  `source/main.cpp` `defaultPlannerConfig()` jerk defaults, and the field-mask
  plumbing in `source/runtime/commands.h` / `configurator.cpp` if jerk moves structs.
- **Telemetry:** `source/telemetry/tlm_frame.{h,cpp}` re-source `enc=`/`vel=` from Drivetrain state.
- **Reuse (do not rewrite):** `Motion::JerkTrajectory`, `Motion::stop_condition`,
  `Motion::MotionBaseline`, `BodyKinematics`, `Hal::MotorVelocityPid`, `libraries/ruckig`.

## Risks
- **Flash headroom:** Ruckig-in-use costs ~151 KB; only ~43 KB (11.7%) free. Moving
  (not duplicating) the solver should be flash-neutral, but **measure with
  `arm-none-eabi-size`** — dropping `VelocityRamp` recovers some.
- **I2C bus timing:** see §5 — preserve the flip-flop and 093 dead-time exactly.
- **Dynamic port rebind** must keep working through the container motor-resolver.
- **Coupling with the gut-the-loop ticket** — sequence so the loop routing and this
  restructure land coherently; grep for `bb.driveIn`/`bb.motionIn`/`routeOutputs` users.

## Verification
1. **Sim unit tests:** `just build-sim` then `uv run python -m pytest` (collects
   `tests/sim/`). Extend/port `test_planner.py`/`test_jerk_trajectory.py` into
   segment-executor tests: straight (no terminal pivot), translate-then-pivot to a
   final heading, pure in-place turn (`distance=0`), multi-segment queue, auto
   decel-to-zero on queue drain, stop mid-segment.
2. **Flash budget:** `arm-none-eabi-size build/MICROBIT` before/after.
3. **Standing hardware bench gate** (HAL/motor/command-surface touch — required by
   `.claude/rules/hardware-bench-testing.md`): `just build-clean` then
   `mbdeploy deploy <full-UID> --hex MICROBIT.hex`; on the stand confirm over the
   real link — encoders alive and incrementing with commanded direction; a `D`
   (straight), a translate-then-pivot-to-heading, and `TURN`/`RT` (in-place) each complete and the queue
   drains to a **graceful** stop (no terminal reverse-creep — regression check vs
   093); `SET jmax`/`yawjmax` take effect (visibly smoother edges); `TLM` `enc=`/`vel=`
   still report measured wheel state. Feed the RX watchdog (`send_fast PING` ~200ms).
