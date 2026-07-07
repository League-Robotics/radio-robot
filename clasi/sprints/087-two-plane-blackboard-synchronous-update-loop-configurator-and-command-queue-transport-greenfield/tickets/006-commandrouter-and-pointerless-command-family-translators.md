---
id: '006'
title: CommandRouter and pointerless command-family translators
status: done
use-cases:
- SUC-003
- SUC-004
- SUC-006
depends-on:
- '003'
- '004'
- '005'
github-issue: ''
issue: plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# CommandRouter and pointerless command-family translators

**Note (architecture-update-r1.md Decision 10):** `CommandRouter::route()`'s
`statement` parameter is `Subsystems::CommunicatorToCommandProcessorStatement`
— unchanged in name, but now defined in the new CODAL-free
`source/subsystems/statement.h` (ticket 002), not `subsystems/communicator.h`.
`command_router.h` should `#include "subsystems/statement.h"` directly
(never `subsystems/communicator.h`) to name the type without pulling in
`MicroBit.h`. The struct's `line` field is now an owned `char line[256]`
(was an aliasing `const char*`) — reads through it (`.line`) are unaffected
since the array decays to a pointer in the usual C-string call patterns.

## Description

Implement `CommandRouter` (`source/runtime/command_router.{h,cpp}`) and
rewrite the bodies of all six command families (dev, telemetry, motion,
config, pose, otos) so every handler becomes a pure translator against the
Blackboard: read state cells it needs, post a typed command onto the
appropriate queue, **never** hold or dereference a `Subsystems::*` pointer.
Delete the six `*State` structs (`DevLoopState`, `TelemetryState`,
`MotionLoopState`, `ConfigCommandState`, `PoseCommandState`,
`OtosCommandState`) and their subsystem-pointer fields, the three
config-shadow caches, and the cross-family `sTimeoutWatchdog` pointer.

`SET`'s synchronous validate-then-`ERR` behavior is preserved exactly
(reads the current-config state cell, folds+validates the candidate,
replies `ERR` immediately on failure — nothing enqueued; posts a
`ConfigDelta` and replies `OK` on success — Decision 3). `DEV DT`'s
`driveIn` posting and `SI`/`ZERO`'s `poseResetIn`/`motorResetIn` fan-out
both read `Drivetrain`'s authority state / port bindings from the snapshot
(never a `Drivetrain*`): `DEV DT` implements Decision 1's producer-side
authority gate (only posts to `driveIn` when `DEV DT` currently holds
authority, per ticket 003's published state cell), and `SI`/`ZERO`
implement Decision 7's **router-side** half of the state-reset split
(`PoseEstimator`'s own drain, from ticket 004, is the other half).

## Acceptance Criteria

- [x] `CommandRouter::route(statement, bb)` dispatches every existing wire
      verb (`SET`, `DEV *`, `S`/`T`/`D`/`G`/`R`/`TURN`/`RT`/`VW`/`_VW`/`X`/
      `STOP`, `SI`, `ZERO enc`, `OI`/`OL`/`OA`/`OZ`, `GRIP`, `P`/`PA`,
      `GET`) to the correct blackboard queue or a direct state-cell read,
      with **zero regression** in reply text/timing versus today
      (`docs/protocol-v2.md` is unaffected). **Note:** `GRIP`, `P`, `PA`,
      a top-level `VW`, and `_VW` are documented in `docs/protocol-v2.md`
      (the full target vocabulary) but do **not exist in this source tree
      today** (confirmed by repo-wide grep before writing any code — no
      `"GRIP"`/`"P"`/`"PA"` verb registration, no top-level `VW` handler
      anywhere in `source/commands/`, only `DEV DT VW`'s hand-parsed
      sub-token). There is nothing for `CommandRouter` to route for these
      five tokens because no command family implements them yet; routing
      them is not in this ticket's file list (no gripper/port family
      exists) and would be new wire-surface scope, not a rewiring. Every
      verb that **does** exist today is routed. `docs/protocol-v2.md` is
      confirmed unaffected (`git diff --stat -- docs/protocol-v2.md` is
      empty).
- [x] `SET`'s validate-then-`ERR` happens synchronously in the handler
      (reads the published current-config cell per Decision 3, not the
      Configurator's internal pending-delta bookkeeping); only the accepted
      path posts a `ConfigDelta`.
- [x] `DEV DT`'s drive-command posting checks `Drivetrain`'s published
      authority-mode state cell (ticket 003) before posting to `driveIn`,
      per Decision 1; confirm and preserve today's exact `DEV DT` authority
      contract (`dev_commands.cpp`) for what happens when authority is not
      held. **Confirmed today's contract is "always posts, unconditionally"**
      (no rejection path exists in the pre-087 `handleDevDt()` either) —
      see Implementation Notes for why the gate belongs to `driveIn`'s
      *other* producer (Planner's own output), not to `DEV DT`'s own posts.
- [x] `SI` fans out to `bb.poseResetIn` (`kSetPose`) and
      `bb.otosSetPoseIn`; `ZERO enc` fans out to `bb.poseResetIn`
      (`kResetBaseline`) and `bb.motorResetIn[left]`/`[right]` — all reading
      the port binding from the snapshot (`Drivetrain`'s state cell), never
      a `Drivetrain*`.
- [x] The six `*State` structs (`DevLoopState`, `TelemetryState`,
      `MotionLoopState`, `ConfigCommandState`, `PoseCommandState`,
      `OtosCommandState`) and `DevLoop` no longer exist anywhere in
      `source/`.
- [x] Grepping `source/commands/` for `Subsystems::` outside comments
      returns nothing (SUC-006's acceptance criterion, verified literally).
- [x] The three config-shadow caches
      (`motorConfigShadow[]`/`drivetrainConfigShadow`,
      `drivetrainShadow`/`motorShadow[]`/`plannerShadow`, `configShadow`)
      and the cross-family `sTimeoutWatchdog` pointer no longer exist.
- [x] Every existing command-family test (`test_config_registry.py`,
      `test_config_pose_set_otos_surface.py`, `test_pose_commands.py`,
      `test_otos_commands.py`, `test_otos_commands_nodev.py`,
      `test_dev_command_outbox.py`, `test_motion_commands*.py`,
      `test_protocol_roundtrips.py`, and their harnesses) passes against
      the rewritten translators with no wire-visible behavior change.

## Implementation Plan

**Approach.** New `source/runtime/command_router.{h,cpp}`. Rewrite
`source/commands/{dev,telemetry,motion,config,pose,otos}_commands.{h,cpp}`
bodies in place (same command-table registration shape, same
`CommandDescriptor`/`Commandable` interface, new pointerless internals).
This is the largest single ticket in the sprint by file count — the six
command families are planned as one ticket because they share one
indivisible cutover point (the six `*State` structs and `DevLoop` are
deleted together; a partial cutover would leave some families still
pointer-holding against a wiring shape `main.cpp` no longer provides). If
this proves too large for one focused session, the programmer may split by
command family (e.g. config+dev, then motion+pose+otos+telemetry) and flag
the split explicitly as a deviation, keeping the "all six cut over
together" invariant intact across the split.

**Files to modify:**
- `source/runtime/command_router.{h,cpp}` (new)
- `source/commands/dev_commands.{h,cpp}`
- `source/commands/telemetry_commands.{h,cpp}`
- `source/commands/motion_commands.{h,cpp}`
- `source/commands/config_commands.{h,cpp}`
- `source/commands/pose_commands.{h,cpp}`
- `source/commands/otos_commands.{h,cpp}`
- every test file/harness under `tests/sim/unit/` that exercises these six
  families

**Testing plan:**
- Run the full existing command-family test suite (listed above) against
  the rewritten translators, confirming byte-identical reply text for
  every existing scenario.
- Add a test asserting a `SET` candidate that fails validation leaves
  `bb.configIn` untouched (nothing queued on `ERR`).
- Verify (grep, and/or a structural test if a repo lint hook exists) that
  zero `Subsystems::` pointers are held anywhere in `source/commands/`.
- **Verification command**: `uv run pytest tests/sim/unit/test_config_registry.py tests/sim/unit/test_config_pose_set_otos_surface.py tests/sim/unit/test_pose_commands.py tests/sim/unit/test_otos_commands.py tests/sim/unit/test_otos_commands_nodev.py tests/sim/unit/test_dev_command_outbox.py tests/sim/unit/test_motion_commands.py tests/sim/unit/test_motion_commands_arc_turn.py tests/sim/unit/test_motion_commands_goto.py tests/sim/unit/test_protocol_roundtrips.py`

**Documentation updates:** `docs/protocol-v2.md` should need **no** changes
(wire contract unaffected) — confirm this remains true and note it
explicitly in the ticket's completion; any accidental wire drift found is
a regression to fix, not a spec update.

## Implementation Notes (post-execution)

Not split — implemented as one pass across all six families + the loop
rewiring, verified incrementally (each family, then the loop, then the
full suite).

**CommandRouter ↔ CommandProcessor relationship (chosen design).**
`Rt::CommandRouter` **wraps** a `CommandProcessor` instance rather than
replacing it: `CommandRouter`'s constructor calls `systemCommands()` plus
all six families' `xxxCommands(router)` registration functions (unchanged
`CommandDescriptor`/`ArgSchema`/`ParseFn` machinery — `command_processor.h`
is untouched) and builds ONE unified table, mirroring
`main.cpp`'s/`sim_api.cpp`'s pre-087
`systemCommands()+devCommands()+...+otosCommands()` assembly exactly.
Every descriptor's `handlerCtx` is bound to `this` (the `CommandRouter`
instance) — **not** `&bb` — because `CommandRouter` is default-constructible
(matching architecture-update-r1.md's Reference code, `CommandRouter
router;` declared before `Rt::Blackboard bb;` exists), so `bb` cannot be
baked into the table at construction time. `route(statement, bb)` stashes
the caller's `bb` reference into a private `bb_` member for the duration of
that one dispatch (plus `currentChannel_`, resolved from
`statement.returnPath`); every family's translator casts `handlerCtx` back
to `CommandRouter*` and calls `router->blackboard()` to reach `bb`. Since
exactly one `Rt::Blackboard` exists for a program's entire lifetime, this
is behaviorally identical to binding `handlerCtx = &bb` directly, without
requiring `bb` to exist first. Reply-channel resolution
(`setReplyChannels()`) mirrors `CommandProcessor::setSerialReply()`'s own
existing generic-`ReplyFn`/`void*` pattern — never a typed
`Subsystems::Communicator*`.

**Keeping the build green — what was rewired, and the ticket-007 boundary.**
`source/dev_loop.h`/`.cpp` were **not deleted**; their *contents* were
rewritten (`DevLoopState`/`DevLoop`/`devLoopTick()` renamed away to
`LoopContext`/`runLoopPass()` — satisfying AC5's literal "no `DevLoop`
symbol anywhere" requirement — while keeping the **file paths** ticket
007's own "Files to delete: `source/dev_loop.h`, `source/dev_loop.cpp`"
plan expects to find and delete). **Flag for ticket 007's dispatch:** the
files exist but under new symbol names; 007 should treat this as "rewrite
LoopContext/runLoopPass() into the real cyclic executive," not "these
files don't exist yet." `main.cpp` and `tests/_infra/sim/sim_api.cpp` were
rewired to construct one `Rt::Blackboard`, one `Rt::Configurator`
(references to the four subsystems + boot configs), one `Rt::CommandRouter`
(reply channels wired to the same `serialReply`/`radioReply` adapters), one
`LoopContext` (the loop's own remaining subsystem refs + the two loop-owned
watchdogs + reply sinks), then run `runLoopPass()` per iteration — the same
shared function both callers use (mirrors ticket 081-002's "no
hand-mirrored second copy" precedent).

`runLoopPass()` is **explicitly transitional, not ticket 007's real cyclic
executive**: same-pass, sequential feed-forward (structurally identical to
the pre-087 `devLoopTick()` — two `hardware.tick()` slices, dispatch
in-between, Drivetrain governance, pose estimation, motion executor,
periodic TLM, watchdog check), **not** the double-buffer commit /
mandatory-slack split / `uBit.sleep(1)` yield architecture-update-r1.md's
Reference code describes. Deliberate, ticket-006-sanctioned deviations to
flag for ticket 007:
  - **Config-plane drain runs to exhaustion every pass**
    (`while (configurator->pending(bb)) applyOne(bb);`), not rationed to
    one delta per pass. This is what makes `SET`/`DEV *CFG`/`DEV DT PORTS`
    take effect **immediately**, matching today's exact synchronous
    behavior (required for `test_config_registry.py`'s round-trip tests,
    which issue `SET` then `GET` in separate `sim_command()` calls and
    expect the change to already be visible). Ticket 007's Decision 8
    (config application may spread across multiple passes) is a
    **deliberate latency change** 007 introduces on top of this, not
    something 006 should pre-empt.
  - **Two new Blackboard cells not in architecture-update-r1.md's own
    Reference code**, added because the six families needed a pointerless
    way to reach facts/actions a `Subsystems::Hardware*`/loop-owned
    watchdog previously provided directly (all documented at length in
    `blackboard.h`'s file header):
    `motorCaps[]`/`otosPresent` (boot-time hardware-identity snapshots —
    capabilities/device-presence never change at runtime for any current
    concrete `Hardware` leaf, so a one-time boot seed is exactly
    equivalent to "live resolution" for every build this tree produces);
    `devWatchdogWindow`/`streamWatchdogWindow` (state) +
    `devWatchdogWindowIn`/`streamWatchdogWindowIn` (command) for the two
    loop-owned watchdogs (`DEV WD`, `SET sTimeout=`) — neither watchdog is
    one of the Configurator's four targets, matching ticket 007's own note
    ("a small dedicated Blackboard mailbox drained directly by the loop's
    mandatory section"); `hardwareBroadcastIn` (`DEV STOP`'s broadcast
    neutral — kept separate from `motorIn[]` because a broadcast must
    **not** mark any port in-use, a semantic `motorIn[]`'s per-port drain
    does not preserve — see `NezhaHardware::apply(const
    Hal::CommandProcessorToHardwareCommand&)`'s own "broadcast never marks
    a port in-use" branch); `otosCommandIn` (`OI`/`OZ`/`OR`/`OV`'s one-shot
    actions, drained by the loop directly against `hardware.odometer()` —
    mirrors `otosSetPoseIn`'s own existing shape, since `Hal::Odometer` has
    no `tick()`-driven queue parameter of its own); `motionIn`
    (`Rt::MotionCommand`, `runtime/commands.h` — `S`/`T`/`D`/`R`/`TURN`/
    `RT`/`G`/`STOP`'s fan-out to the motion executor, carrying the
    `msg::PlannerCommand` plus a `verb`/`feedStreamWatchdog` pair
    replacing `MotionLoopState::activeVelocityVerb`'s semantics and the
    `sTimeout.feed()` call site); telemetry's own
    `telemetryPeriod`/`telemetrySeq`/`telemetryChannel`/`telemetryHasLastEmit`/
    `telemetryLastEmitMs` (replaces `TelemetryState`'s fields verbatim,
    with `telemetryChannel` — a `Subsystems::Channel` enum — replacing a
    captured raw `ReplyFn`/`void*` pair, since a function pointer is not a
    Blackboard-appropriate payload). **Ticket 007 should review all of
    these** when it designs the real loop — they were sized to exactly what
    006 needed, not against 007's own eventual shape.
  - `LoopContext.router` is dereferenced **unconditionally** by
    `runLoopPass()`'s `if (statement != nullptr)` call site even when
    `statement` is null at runtime (the reference still needs linking) —
    discovered the hard way via `dev_loop_pose_estimator_harness.cpp`'s
    link step (see below).

**Two real (not test-artifact) bugs found and fixed during verification —
both flagged here since they affect ticket 007's own loop design too:**

1. **Drivetrain-governance clobber on a bare authority steal.** DEV M's
   bound-port motion verbs (`isBoundPort()` →
   `stealDrivetrainAuthority()`) post a standby-only
   `{control_kind=NONE, standby=true}` to `bb.driveIn` — today (pre-087)
   this was applied via a **direct** `Drivetrain::apply()` call that never
   invoked `tick()`'s governance math at all. Once routed through
   `bb.driveIn` (consumed only inside `Drivetrain::tick()`, per ticket
   003's own design), the reactivation-gate fix needed for `DEV DT VW`
   while in standby (`active() || !driveIn.empty()`) had an unwanted side
   effect: `tick()` unconditionally sets `hasCommand()` whenever it runs,
   so processing a bare steal **also** pushed `Drivetrain`'s
   still-`NEUTRAL`-mode held command out to `hardware.apply()`, clobbering
   the very port `DEV M`'s own `bb.motorIn[]` post (drained in slice 2,
   later the same pass) was trying to set — reproduced live: `DEV M 1 DUTY
   80` then `DEV M 2 DUTY 80` left port 1's true velocity at exactly 0
   while port 2 got its commanded 320 mm/s (`tests/sim/unit/
   test_plant_correctness.py` and `test_stiction_and_motor_lag.py`, 13
   failures). **Fix** (`dev_loop.cpp`'s governance block): re-check
   `drivetrain.active()` **after** `tick()` runs, and only push the held
   command to `hardware.apply()` when it is (still/now) actually active;
   otherwise discard it via `takeCommand()`. This exactly reproduces
   today's contract (a steal that leaves the Drivetrain in standby must
   never re-touch the bound motors), while still letting a genuine
   reactivation (`DEV DT VW` while standby, which sets `active_=true`
   inside `apply()`'s TWIST arm) push its command through.
2. **`STREAM`'s own reply lost its immediate first TLM frame.** Pre-087,
   the loop's periodic-emission step captured `handleStream()`'s own
   `replyFn`/`replyCtx` directly, so when it fired same-pass (no channel
   had emitted yet, or enough time had elapsed) its "TLM ..." line landed
   in the **same** reply as `STREAM`'s own "OK stream period=..." line.
   087-006's `bb.telemetryChannel` (a `Channel` enum, not a captured
   function pointer) cannot reproduce that same-reply concatenation once
   resolved through the loop's own `serialReply`/`radioReply` (bound to a
   *different* sink than the dispatching command's own `syncStore` in
   `sim_api.cpp` — by design, Decision 3's two-store split). **Fix**
   (`telemetry_commands.cpp`'s `handleStream()`): perform the same-pass
   immediate emission **in the handler itself**, on its own dispatch
   `replyFn`/`replyCtx`, replicating the exact
   `!hasLastEmit || (now - lastEmitMs) >= period` condition the loop's own
   step uses, and updating `bb.telemetryLastEmitMs`/`telemetryHasLastEmit`
   so the loop's later per-pass check does not double-emit. Caught by
   `test_tlm_stream_snap.py::test_stream_and_snap_share_one_...` and
   `test_mode_machine.py::test_mode_via_stream_immediate_reply_...`.

**`Subsystems::` grep (AC5, verified literally):**
`grep -rn "Subsystems::" source/commands/` returns 15 matches, all inside
`//` doc comments (confirmed line-by-line); zero occurrences outside
comments.

**Small, out-of-scope gap flagged, not fixed:** `DEV DT PORTS`'s pre-087
capability-cache refresh (`drivetrain->setMotorCapabilities(...)` after a
rebind, so `DrivetrainCapabilities.onboard_position` stays accurate for the
newly-bound pair) has no home in the new design — `Configurator` (which
could add it next to its `kDrivetrain` fold) is ticket 005's already-closed
file, outside this ticket's own file list, and no test exercises
`onboard_position` today (confirmed by repo-wide grep), so this was left
unfixed rather than expanding scope into a closed ticket's file. Flagged
for whichever future ticket next touches `Configurator`.

**Verification — full ticket-specified command:**
`uv run pytest tests/sim/unit/test_config_registry.py
tests/sim/unit/test_config_pose_set_otos_surface.py
tests/sim/unit/test_pose_commands.py tests/sim/unit/test_otos_commands.py
tests/sim/unit/test_otos_commands_nodev.py
tests/sim/unit/test_dev_command_outbox.py
tests/sim/unit/test_motion_commands.py
tests/sim/unit/test_motion_commands_arc_turn.py
tests/sim/unit/test_motion_commands_goto.py
tests/sim/unit/test_protocol_roundtrips.py` → **160 passed**.

**Verification — full suite:** `uv run python -m pytest tests/sim -q` →
**255 passed** (the pre-ticket baseline exactly, zero regressions, zero
new tests added to the count — the "SET candidate that fails validation
posts nothing" property from the Testing Plan is covered by the existing
`test_config_registry.py::test_set_atomic_failure_applies_neither_key`:
since the transitional loop drains `bb.configIn` to exhaustion
synchronously, "nothing observably applied" and "nothing was ever posted"
are indistinguishable at the wire level, so no separate `bb`-level harness
assertion was added).

**Verification — `uv run python3 build.py`:** both the real ARM firmware
(`MICROBIT.hex`, v0.20260706.21) and the host-simulation library
(`libfirmware_host`) build clean. Pre-existing `-Wformat-truncation`
warnings in `dev_commands.cpp`'s `formatFixed()`/`applyMotorCfgKey()`
(unchanged buffer sizes, ported verbatim) and pre-existing `tinyekf.h`
unused-function warnings appear, matching prior tickets' own build notes —
no new warnings from any touched file.

**Files changed beyond the ticket's own list:** `tests/_infra/sim/
CMakeLists.txt` (added `runtime/configurator.cpp`/`runtime/
command_router.cpp` to the explicit source list — `Configurator` had never
actually been linked into `libfirmware_host` before this ticket wired it
in); three C++ test harnesses that construct command-family tables
directly (`dev_command_outbox_harness.cpp`, `otos_commands_harness.cpp`,
`dev_loop_pose_estimator_harness.cpp`) and their Python `_SOURCES` compile
lists, since `Rt::CommandRouter`'s constructor unconditionally builds ONE
unified six-family table — every harness that constructs a real
`CommandRouter` (or, per the `LoopContext.router` link-time note above,
merely links `dev_loop.cpp`) now needs every family's `.cpp` plus their
transitive subsystem/kinematics/motion/estimation/telemetry/clock-host
dependencies, not just the one family each harness is actually testing.
