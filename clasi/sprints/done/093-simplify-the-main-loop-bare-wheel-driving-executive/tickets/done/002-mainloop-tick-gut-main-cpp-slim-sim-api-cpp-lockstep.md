---
id: '002'
title: MainLoop::tick() gut + main.cpp slim + sim_api.cpp lockstep
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- '001'
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

- [x] `source/runtime/main_loop.h`: constructor is
      `MainLoop(Subsystems::Hardware& hardware, Subsystems::Drivetrain& drivetrain)`.
      `poseEstimator_`, `planner_`, `watchdog_`, `streamWatchdog_`,
      `activeVelocityVerb_`, and all four `ReplyFn`/`ctx` fields are removed.
      `serviceWatchdogs()`, `estop()`, and `feedWatchdog()` are removed
      entirely (no watchdog left to check/feed). `commit()`'s signature
      drops the `otosFusableThisPass` parameter (nothing derives it any
      more). `routeOutputs()` keeps only the `Drivetrain` half — its
      `plannerEngagedThisPass` parameter is removed.
- [x] `source/runtime/main_loop.cpp`: `tick()` body matches the issue's
      target shape (hardware tick → drivetrain tick → commit → route). The
      `#include`s for `commands/command_processor.h`,
      `commands/telemetry_commands.h`, and `<cstring>` are dropped (no
      longer used in this file); `commands/dev_commands.h` is dropped too
      UNLESS `estop()`'s removal still leaves some other symbol from it in
      use (verify — expect a clean drop). `hal/capability/hal_command.h`
      stays (still used by `Hal::DrivetrainToHardwareCommand` in
      `routeOutputs()`).
- [x] `source/main.cpp`: no longer constructs `Subsystems::PoseEstimator`,
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
- [x] `source/main.cpp`'s slack loop becomes:
      `comm.tick() → if (comm.hasCommand()) { take + router.route(command, bb) } else if (!yieldedThisSlack) { uBit.sleep(1); yieldedThisSlack = true; }`
      — the `configurator.pending(bb)/applyOne(bb)` branch is deleted. The
      once-per-slack `uBit.sleep(1)` yield (093's prior fix, unrelated to
      this gut) is preserved verbatim — do not touch its cadence or
      placement.
- [x] `tests/_infra/sim/sim_api.cpp`: `SimHandle` drops its
      `poseEstimator`/`planner`/`configurator` members. `loop` is
      constructed as `Rt::MainLoop loop(hardware, drivetrain);` (no reply
      sinks). `asyncStore` is removed (no longer has a writer);
      `syncStoreSerial`/`syncStoreRadio` are KEPT (still used by
      `sim_command_on()`'s own per-command reply). `sim_tick()` and
      `sim_command_on()` drop their `while (s->configurator.pending(...))`
      drain loops.
- [x] `sim_get_async_evts()` is KEPT as a no-op stub (per architecture-
      update.md Decision 4): still exported with its existing signature,
      always writes 0 bytes / returns 0, so `host/robot_radio/io/sim_conn.py`
      needs no matching change this sprint. Add a one-line comment at the
      stub noting it is a deliberate no-op, not a leftover.
- [x] `just build-sim` compiles cleanly.
- [x] `S 200 200` / `S 200 -200` / `STOP` / `PING` / `HELLO` still behave
      correctly via `sim.command(...)` after this ticket (manual/ad hoc
      check sufficient here — ticket 003 commits the real suite). See
      Closing Notes below: `STOP`'s wire reply (`OK stop`) is correct and
      unchanged from ticket 001; a pre-existing gap (inherited from ticket
      001, NOT introduced here) means the wheels do not physically
      neutralize. Flagged, not silently patched — out of this ticket's
      file scope.
- [x] No reference to `poseEstimator_`, `planner_`, `Rt::Configurator`,
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

## Closing Notes

All acceptance criteria met. `main_loop.h`/`main_loop.cpp` gutted to the
two-reference, four-step `tick()`; `main.cpp` and `sim_api.cpp` updated in
lockstep with the new `MainLoop(hardware, drivetrain)` constructor. Both
`just build-sim` and the full ARM firmware build (`uv run python3 build.py
--clean`) compile cleanly (toolchain was available this session). Grep for
`poseEstimator_`/`planner_`/`Rt::Configurator`/`SerialSilenceWatchdog`/
`StreamingDriveWatchdog` across `main_loop.{h,cpp}`, `main.cpp`,
`sim_api.cpp` returns zero hits (doc comments describing their absence were
deliberately reworded to avoid the literal banned tokens too).

**Safety-relevant finding — flag for team-lead, recommend reopening ticket
001 (not fixed here, out of this ticket's file scope):** ad hoc
`sim.command(...)` verification found that `PING`/`HELLO`/`S 200 200`/
`S 200 -200` all behave correctly, including physically (wheel velocity/PWM
tracks the commanded sign and magnitude). `STOP`'s wire reply (`OK stop`) is
also correct. However, **`STOP` does not physically neutralize the
wheels**: after `S 300 300` + settle, issuing `STOP` and ticking for several
more seconds shows PWM decaying only ~86→79 (not dropping to ~0), i.e. the
motor keeps tracking its last commanded velocity target indefinitely.

Root cause (confirmed by code inspection, not just symptom): ticket 001's
`handleStop` (`source/commands/motion_commands.cpp`) posts
`buildDrivetrainStop(msg::Neutral::BRAKE)` — a `{NEUTRAL, standby=true}`
command — to `bb.driveIn` ONLY. `Drivetrain::apply()` processes this by
calling `setNeutral()` (which sets `active_ = true`) and THEN, because
`standby.val` is true, `standby()` (which sets `active_ = false`) — so by
the time `Drivetrain::tick()` finishes this same pass, `active()` is
`false`. `MainLoop::routeOutputs()`'s Drivetrain half — verbatim,
pre-existing code, unchanged by this ticket, matching the issue's own
literal target-shape pseudocode — only posts the held command to
`bb.motorIn[]` `if (drivetrain_.active())`. So the computed NEUTRAL/BRAKE
command is silently dropped every time, and nothing else in the reduced
4-verb surface ever posts a fresh command to `bb.motorIn[]` to override the
motor's stale target.

This is NOT a regression from ticket 002 — the exact same `routeOutputs()`
gating logic (Decision 2, ticket 006's original bug-fix, protecting a
DEV-M-stolen port from a stale Drivetrain reassertion) existed unchanged
before this ticket; ticket 002 only removed the Planner half and the
`plannerEngagedThisPass` parameter. It also predates this ticket's
existence as an *observable* problem: ticket 001 only verified `STOP`'s
WIRE REPLY (`OK stop`), not physical wheel behavior (Planner/hardwareBroadcastIn
were still present then, and neither was fed by the rewritten `S`/`STOP`
verbs, so the gap was already latent but untested).

Confirms by direct comparison: `dev_commands.cpp`'s `handleDevStop`/`DtMode::STOP`
handlers (`DEV STOP` / `DEV DT STOP`, both still on disk though unregistered)
BOTH post the SAME `buildDrivetrainStop()` to `bb.driveIn` for Drivetrain's
own bookkeeping AND ALSO post a direct neutral straight to `bb.motorIn[]`
(addressed) or `bb.hardwareBroadcastIn` (broadcast) — i.e., the codebase's
own established pattern already knows `bb.driveIn.post(buildDrivetrainStop(...))`
alone never reaches hardware once `standby=true` takes the Drivetrain
inactive. Ticket 001's `handleStop` rewrite copied the bookkeeping half but
not the direct-to-hardware half.

Recommended fix (for whoever picks this up, likely a reopened ticket 001 or
a new ticket ahead of ticket 004's bench verification, which requires
`STOP` to physically neutralize both wheels): in
`source/commands/motion_commands.cpp`'s `handleStop`, additionally post a
direct `msg::MotorCommand` neutral (BRAKE) to `bb.motorIn[left-1]`/
`bb.motorIn[right-1]` (ports from `b.drivetrainConfig.left_port`/
`right_port`, mirroring `dev_commands.cpp`'s `DtMode::STOP` case) alongside
the existing `bb.driveIn.post(buildDrivetrainStop(...))` call. Not
implemented here: `motion_commands.cpp` is outside ticket 002's file list
(ticket 001's territory, already closed `done`), and ticket 004's own
Implementation Plan explicitly directs "if it finds a defect, fix it in the
ticket where the defect was introduced (reopen, don't silently patch here)
unless the team-lead directs otherwise."
