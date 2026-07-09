---
status: in-progress
sprint: 093
tickets:
- 093-001
- 093-002
- 093-003
- 093-004
---

# Simplify the main loop — strip it to bare wheel driving

## Context

The cyclic-executive main loop has grown into a "total wreck" that never
realized the intended vision. `Rt::MainLoop::tick()` (280+ lines) now
orchestrates the safety watchdog + estop, the streaming-drive watchdog, the
odometer, the pose estimator (EKF fusion), the Planner (motion-goal closure),
telemetry emission, and a fan-out of two-plane routing — all in one pass. The
command surface is ~30 verbs across seven family files. The `S` "drive"
command doesn't even drive the wheels directly: it converts wheel speeds to a
body twist, hands off to the Planner, which ramps and converts back to wheels.

**Goal:** dump nearly everything out of the loop and rebuild it, from the
ground up, as a minimal *command → wheels* path. Keep the object **classes**
in the tree (Planner, PoseEstimator, EkfTiny, both watchdogs, Configurator,
telemetry) — they are simply no longer ticked or wired. Reduce the live command
surface to four verbs. The result is a loop a person can read in one sitting.

**Decisions locked with the stakeholder:**
- **Removed code is left un-wired, not deleted.** Command-family files and the
  Blackboard mailboxes they post to stay in the tree; the loop just stops
  registering/ticking them. Fully reversible, least churn.
- **Four live verbs:** `S`, `STOP`, `PING`, `HELLO`. The boot `DEVICE:` banner
  (emitted at startup, not a command) stays so the robot still announces itself
  and host auto-detection (robot_radio / TestGUI) keeps working.
- **Carried out as a CLASI sprint** (issue → sprint-planner → tickets →
  programmer execution).

> **Safety note (stakeholder-directed):** removing the serial-silence safety
> watchdog + estop is an explicit instruction and contradicts the long-standing
> "non-negotiable safety watchdog" note. It is acceptable because the robot runs
> **on the stand, wheels off the ground** (`.claude/rules/hardware-bench-testing.md`).
> This should be called out in the sprint architecture as a deliberate,
> stakeholder-owned removal.

## What the loop keeps vs. loses

**Kept (constructed + ticked):** `Communicator`, `NezhaHardware` (+ its four
`NezhaMotor`s), `Drivetrain`, `Rt::Blackboard`, `Rt::CommandRouter`,
`Rt::MainLoop`. The `bb.driveIn` / `bb.motorIn[]` mailboxes.

**Removed from the loop (classes remain in the tree, orphaned):** `Planner`,
`PoseEstimator` + `EkfTiny`, the odometer tick/drains, `SerialSilenceWatchdog`
+ `StreamingDriveWatchdog` + `estop()`, telemetry emission, and the runtime
`Rt::Configurator` (boot config is applied once, directly, at construction).

## The new loop (target shape)

`Rt::MainLoop::tick(Blackboard& bb, uint32_t now)` collapses to:

```
hardware_.tick(now, bb.motorIn, bb.motorResetIn);        // apply staged per-port cmds
drivetrain_.tick(now, bb.motors, kPortCount, bb.driveIn);// drain driveIn, govern, hold output
// commit (clock edge):
for (port 1..kPortCount) bb.motors[port-1] = hardware_.state(port);
bb.drivetrain = drivetrain_.state();
// routeOutputs (drivetrain half only):
if (drivetrain_.hasCommand()) {
  auto cmd = drivetrain_.takeCommand();
  if (drivetrain_.active()) {
    bb.motorIn[cmd.wheel[0].port-1].post(cmd.wheel[0].command);
    bb.motorIn[cmd.wheel[1].port-1].post(cmd.wheel[1].command);
  }
}
```

Everything else in the current `tick()` — `serviceWatchdogs()`, the
`hardwareBroadcastIn` drain, the odometer apply/setPose/tick, `poseEstimator_.tick()`,
the `bb.motionIn`→`planner_.apply()` motion executor, the stream-watchdog stop,
`planner_.tick()` + event emission, and the periodic telemetry block — is
deleted from the loop. `estop()`, `serviceWatchdogs()`, `commit()`'s pose/planner/otos
copies, and `routeOutputs()`'s planner half go away.

`MainLoop` shrinks to two references (`hardware_`, `drivetrain_`); the
`poseEstimator_`/`planner_` refs, both watchdog members, `activeVelocityVerb_`,
and the four reply-sink fields are removed (no loop-originated EVT/telemetry
remains).

## Command surface: four verbs

Reduce `Rt::CommandRouter`'s table
([source/runtime/command_router.cpp:24-41](source/runtime/command_router.cpp#L24-L41))
to register exactly `PING`, `HELLO`, `S`, `STOP`. Simplest un-wired approach:
`buildTable()` stops calling `devCommands`/`telemetryCommands`/`configCommands`/
`poseCommands`/`otosCommands`, and assembles a minimal table containing only the
four wanted descriptors (the other verbs' factories and files stay on disk,
just uncalled). `PING`/`HELLO` handlers are reused verbatim from
[source/commands/system_commands.cpp](source/commands/system_commands.cpp).

**Rewrite `handleS`** ([source/commands/motion_commands.cpp:351-390](source/commands/motion_commands.cpp#L351-L390)):
`S <left> <right>` (signed wheel velocities, `// [mm/s]`, ±1000 clamp). Drop the
`BodyKinematics::forward()` twist conversion, the stop-condition clauses, and
the `bb.motionIn` post. Instead build a `msg::DrivetrainCommand` with the
`WHEELS` arm and post it to `bb.driveIn`:

```cpp
msg::WheelTargets wt;                          // messages/drivetrain.h / common.h
wt.w_[0].speed = {true, (float)left};          // Opt<float> speed
wt.w_[1].speed = {true, (float)right};
wt.w_count = 2;                                 // apply() reads w()[0]=left, w()[1]=right
msg::DrivetrainCommand cmd; cmd.setWheels(wt);
bb.driveIn.post(cmd);
```

`Drivetrain::apply()` already maps `WHEELS` → `setWheelTargets(left, right)`
([source/subsystems/drivetrain.cpp:43-58](source/subsystems/drivetrain.cpp#L43-L58)),
which bypasses kinematics and (re)activates authority — exactly the direct
wheel drive we want. Reply stays `OK drive l=… r=…`.

**Rewrite `handleStop`** ([source/commands/motion_commands.cpp:719-737](source/commands/motion_commands.cpp#L719-L737)):
post `buildDrivetrainStop(msg::Neutral::BRAKE)` (the canonical `{NEUTRAL,
standby=true}` command, [source/commands/dev_commands.h](source/commands/dev_commands.h))
to `bb.driveIn`. Reply `OK stop`.

Behavior change to note in the sprint doc: with the Planner gone there is **no
acceleration ramp** — `S` sets wheel-velocity targets that the NezhaMotor PID
tracks immediately. That is the intended simplification (all ramping/dead-time/
decel logic lived in the Planner).

## Files to modify

- [source/runtime/main_loop.h](source/runtime/main_loop.h) /
  [source/runtime/main_loop.cpp](source/runtime/main_loop.cpp) — gut `tick()`;
  drop `serviceWatchdogs`/`estop`/`commit` pose+planner copies/`routeOutputs`
  planner half; shrink the constructor + members.
- [source/main.cpp](source/main.cpp) — stop constructing `PoseEstimator`,
  `Planner`, `Rt::Configurator`; drop `defaultPlannerConfig()`, their
  `configure()` calls, the `bb.motorCaps[]`/`otosPresent` seeding, and
  `loop.feedWatchdog()`. Keep `drivetrain.configure(dtConfig)` and
  `drivetrain.setMotorCapabilities(...)` (governance still needs them). Simplify
  the slack loop to `comm.tick → route → yield-once-per-slack` (drop the
  `configurator.pending/applyOne` branch). **Keep the `uBit.sleep(1)`
  yield-once-per-slack** — still required for radio RX delivery (Decision 9),
  unrelated to the loop's control content.
- [source/runtime/command_router.cpp](source/runtime/command_router.cpp) —
  `buildTable()` reduced to the four verbs.
- [source/commands/motion_commands.cpp](source/commands/motion_commands.cpp) —
  rewrite `handleS`/`parseS` (drop stop-clauses) and `handleStop`; leave
  `T/D/R/TURN/RT/G` handlers in place, unregistered.
- **[tests/_infra/sim/sim_api.cpp](tests/_infra/sim/sim_api.cpp) — MUST be updated
  in lockstep.** It shares the one `MainLoop::tick()` and constructs the same
  subsystems (the "1:1 mirror" invariant, `main_loop.h`). Its `MainLoop`
  construction and wiring must match the new signature or the sim build breaks.

## Test fallout (a sprint concern, flag prominently)

The `tests/sim/` suite is the CLASI close-gate. Large parts of it exercise the
now-removed surface: Planner goal closure, pose/EKF fusion, telemetry `STREAM`/
`SNAP`, `SET`/`GET`, `T/D/R/TURN/RT/G`, and `S`'s old twist-through-Planner
semantics. These will fail. Per the greenfield-rebuild preference (park old
trees rather than drag them along), the sprint should **move the obsoleted sim
tests to a parked location** (e.g. `tests_old/` or a quarantined subdir) and
keep a small, focused sim suite for the new minimal loop: `S` drives both
wheels the commanded direction/magnitude, `STOP` neutralizes, `PING`/`HELLO`
answer. Decide the exact parking scheme during ticket planning; it is real
scope, not an afterthought.

## Verification

1. **Build sim:** `just build-sim` (or the worktree recipe) — sim compiles with
   the updated `sim_api.cpp`.
2. **Focused sim tests:** run the trimmed `tests/sim/` set — `S`/`STOP`/`PING`/
   `HELLO` behave; no reference to removed verbs.
3. **Bench on the stand** (`.claude/rules/hardware-bench-testing.md`): build +
   flash (`just build-clean`; `mbdeploy deploy <UID> --hex MICROBIT.hex`), open
   the link, and confirm:
   - `PING` → `OK`; `HELLO` → `DEVICE:…` banner.
   - `S 200 200` → both wheels spin forward, encoders climb together; `S 200 -200`
     → opposite spin; magnitude tracks command.
   - `STOP` → wheels neutralize, encoders hold.
   - Round-trip works over the real transport (serial at bench; relay if testing
     the radio path — the yield-once-per-slack must keep radio alive).

## CLASI routing

File this as a `clasi/issues/` item, then dispatch `sprint-planner` to write the
architecture update (calling out the deliberate safety-watchdog removal and the
test-parking plan) and cut tickets. Suggested ticket shape: (1) minimal command
table + `S`/`STOP` rewrite, (2) `MainLoop::tick()` gut + `main.cpp` slim +
`sim_api.cpp` lockstep, (3) sim-test parking + focused suite, (4) bench-gate
verification.
