---
id: '002'
title: MainLoop::tick() gut + main.cpp slim + sim_api.cpp lockstep
status: open
use-cases: [SUC-001, SUC-002, SUC-003]
depends-on: ['001']
github-issue: ''
issue:
- simplify-the-main-loop-strip-it-to-bare-wheel-driving.md
- get-wire-output-events-telemetry-out-of-the-main-loop.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# MainLoop::tick() gut + main.cpp slim + sim_api.cpp lockstep

## Description

The structural core of the sprint. `Rt::MainLoop` shrinks from four
subsystem references + four reply-sink parameters down to two references
(`Hardware&`, `Drivetrain&`) and no reply sinks. `tick()` collapses to the
target shape in `clasi/issues/simplify-the-main-loop-strip-it-to-bare-wheel-
driving.md` ("The new loop (target shape)"): tick `Hardware`, tick
`Drivetrain`, commit `bb.motors[]`/`bb.drivetrain`, route `Drivetrain`'s own
output back to `bb.motorIn[]`. Everything else currently in `tick()` —
`serviceWatchdogs()`, `estop()`, the odometer/pose/planner portions of
`commit()`, the `Planner` half of `routeOutputs()`, the motion-executor
`bb.motionIn` drain, and the periodic-telemetry block — is deleted, not
stubbed.

Both composition roots (`source/main.cpp`, `tests/_infra/sim/sim_api.cpp`)
must be updated in the SAME change — they share the one `MainLoop::tick()`
(the "1:1 mirror" invariant documented in `main_loop.h`'s own file header) —
or the sim build breaks immediately. This depends on ticket 001 landing
first: `S`/`STOP` must already be posting to `bb.driveIn` directly before
`bb.motionIn`'s sole drain path (the Planner motion executor) disappears,
or those two verbs would go silently dead the moment this ticket lands.

This ticket also resolves the companion issue
`get-wire-output-events-telemetry-out-of-the-main-loop.md` — by removing
every loop-originated event/telemetry PRODUCER (the watchdogs' `EVT`s,
`Planner`'s `EVT done`, periodic `TLM`) rather than building that issue's
proposed `eventsOut`/`telemetryOut` queue + `drainLoopOutputs()` seam
(architecture-update.md Decision 1). No new queue is added.

## Acceptance Criteria

- [ ] `source/runtime/main_loop.h`: constructor is
      `MainLoop(Subsystems::Hardware& hardware, Subsystems::Drivetrain& drivetrain)`.
      `poseEstimator_`, `planner_`, `watchdog_`, `streamWatchdog_`,
      `activeVelocityVerb_`, and all four `ReplyFn`/`ctx` fields are removed.
      `serviceWatchdogs()`, `estop()`, and `feedWatchdog()` are removed
      entirely (no watchdog left to check/feed). `commit()`'s signature
      drops the `otosFusableThisPass` parameter (nothing derives it any
      more). `routeOutputs()` keeps only the `Drivetrain` half — its
      `plannerEngagedThisPass` parameter is removed.
- [ ] `source/runtime/main_loop.cpp`: `tick()` body matches the issue's
      target shape (hardware tick → drivetrain tick → commit → route). The
      `#include`s for `commands/command_processor.h`,
      `commands/telemetry_commands.h`, and `<cstring>` are dropped (no
      longer used in this file); `commands/dev_commands.h` is dropped too
      UNLESS `estop()`'s removal still leaves some other symbol from it in
      use (verify — expect a clean drop). `hal/capability/hal_command.h`
      stays (still used by `Hal::DrivetrainToHardwareCommand` in
      `routeOutputs()`).
- [ ] `source/main.cpp`: no longer constructs `Subsystems::PoseEstimator`,
      `Subsystems::Planner`, or `Rt::Configurator`. `defaultPlannerConfig()`
      is deleted. The `bb.motorCaps[]`/`bb.otosPresent` boot-seeding loop is
      deleted (no longer read by any live verb — `DEV M`'s capability gate
      and `OI/OZ/...`'s "ERR nodev" guard are both unregistered). Boot
      config application becomes direct:
      `drivetrain.configure(dtConfig)` and
      `drivetrain.setMotorCapabilities(hardware.motor(bootPorts.left)
      .capabilities(), hardware.motor(bootPorts.right).capabilities())`
      are KEPT (governance still needs them — `bootPorts` still comes from
      `drivetrain.ports()`). `loop.feedWatchdog(...)` calls (both the boot
      call and the slack-phase ingest call) are deleted.
- [ ] `source/main.cpp`'s slack loop becomes:
      `comm.tick() → if (comm.hasCommand()) { take + router.route(command, bb) } else if (!yieldedThisSlack) { uBit.sleep(1); yieldedThisSlack = true; }`
      — the `configurator.pending(bb)/applyOne(bb)` branch is deleted. The
      once-per-slack `uBit.sleep(1)` yield (093's prior fix, unrelated to
      this gut) is preserved verbatim — do not touch its cadence or
      placement.
- [ ] `tests/_infra/sim/sim_api.cpp`: `SimHandle` drops its
      `poseEstimator`/`planner`/`configurator` members. `loop` is
      constructed as `Rt::MainLoop loop(hardware, drivetrain);` (no reply
      sinks). `asyncStore` is removed (no longer has a writer);
      `syncStoreSerial`/`syncStoreRadio` are KEPT (still used by
      `sim_command_on()`'s own per-command reply). `sim_tick()` and
      `sim_command_on()` drop their `while (s->configurator.pending(...))`
      drain loops.
- [ ] `sim_get_async_evts()` is KEPT as a no-op stub (per architecture-
      update.md Decision 4): still exported with its existing signature,
      always writes 0 bytes / returns 0, so `host/robot_radio/io/sim_conn.py`
      needs no matching change this sprint. Add a one-line comment at the
      stub noting it is a deliberate no-op, not a leftover.
- [ ] `just build-sim` compiles cleanly.
- [ ] `S 200 200` / `S 200 -200` / `STOP` / `PING` / `HELLO` still behave
      correctly via `sim.command(...)` after this ticket (manual/ad hoc
      check sufficient here — ticket 003 commits the real suite).
- [ ] No reference to `poseEstimator_`, `planner_`, `Rt::Configurator`,
      `SerialSilenceWatchdog`, or `StreamingDriveWatchdog` remains in
      `main_loop.{h,cpp}`, `main.cpp`, or `sim_api.cpp` (grep-clean).

## Implementation Plan

**Approach**: Structural composition-root + loop rewrite, in lockstep across
both entry points. Do the header (`main_loop.h`) and implementation
(`main_loop.cpp`) together first, then update `main.cpp`, then
`sim_api.cpp` — building `just build-sim` after each of the latter two
catches a mismatched constructor call immediately.

**Files to modify**:
- `source/runtime/main_loop.h`
- `source/runtime/main_loop.cpp`
- `source/main.cpp`
- `tests/_infra/sim/sim_api.cpp`

**Testing plan**:
- Existing: `just build-sim` must succeed (this is the load-bearing check —
  a signature mismatch between `main_loop.h` and either composition root
  fails the build immediately). Full `tests/sim/` suite is NOT expected
  green after this ticket (ticket 003 restores a green, focused suite).
- New: none committed by this ticket.

**Documentation updates**: update `main_loop.h`'s file-header doc comment
(currently describes the four-subsystem/watchdog/reply-sink design in
detail) to describe the new two-reference, no-watchdog, no-reply-sink
shape — the header comment is normative documentation for this class and
must not be left describing removed members.
