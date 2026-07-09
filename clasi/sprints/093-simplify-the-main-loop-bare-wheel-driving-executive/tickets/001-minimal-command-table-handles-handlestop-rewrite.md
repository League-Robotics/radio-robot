---
id: '001'
title: Minimal command table + handleS/handleStop rewrite
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on: []
github-issue: ''
issue: simplify-the-main-loop-strip-it-to-bare-wheel-driving.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Minimal command table + handleS/handleStop rewrite

## Description

Reduce `Rt::CommandRouter`'s live command surface from ~30 verbs across six
families to exactly four: `PING`, `HELLO` (unchanged, from `systemCommands()`),
`S`, `STOP` (rewritten). This is the wire-surface half of the sprint 093 gut —
it must land before ticket 002 removes `Planner`/`PoseEstimator` from the
loop, because after that removal nothing drains `bb.motionIn` any more. `S`/
`STOP` need to already be posting to `bb.driveIn` directly (bypassing
`Planner` entirely) BEFORE ticket 002 makes `bb.motionIn` a dead mailbox,
or `S`/`STOP` would silently stop working the moment ticket 002 lands.

`T`/`D`/`R`/`TURN`/`RT`/`G` handler bodies, and every file in the `dev`/
`config`/`pose`/`otos`/`telemetry` command families, are left **source-
unchanged on disk** — only unregistered from `buildTable()`. This is the
sprint's "removed code is left un-wired, not deleted" decision
(architecture-update.md Step 5/Migration Concerns).

## Acceptance Criteria

- [x] `Rt::CommandRouter::buildTable()` (`source/runtime/command_router.cpp`)
      registers only `systemCommands()` + `motionCommands()`; the calls to
      `devCommands()`, `telemetryCommands()`, `configCommands()`,
      `poseCommands()`, `otosCommands()` are removed (the `#include`s for
      those families' headers may go too, but the family `.cpp`/`.h` files
      themselves are untouched).
- [x] `motionCommands()` (`source/commands/motion_commands.cpp`) still
      registers all of `S`/`T`/`D`/`R`/`TURN`/`RT`/`G`/`STOP` at the
      descriptor level (no `buildTable()`-level filtering of individual
      motion verbs) — but since `buildTable()` still calls `motionCommands()`
      wholesale, `T`/`D`/`R`/`TURN`/`RT`/`G` remain reachable at the wire
      unless a separate decision prunes them too. **Decide and document
      explicitly** (this ticket, not deferred): either (a) prune
      `motionCommands()`'s own returned vector down to just `S`/`STOP`
      inside this file (leaving the `T`/`D`/etc. handler functions/parsers
      defined-but-uncalled, same "unregistered not deleted" treatment as
      the other families), or (b) leave all eight motion verbs registered
      and rely on their handlers still posting to the now-dead
      `bb.motionIn` (silently inert once ticket 002 lands, but reachable
      and replying `OK` on the wire, which is misleading). **Recommended:
      (a)** — matches the issue's "four live verbs" decision literally;
      record the choice in this ticket's implementation notes.

      **Decision (team-lead, implemented): option (a).**
      `motionCommands()`'s own returned vector now pushes only `S` and
      `STOP`; `T`/`D`/`R`/`TURN`/`RT`/`G`'s `parse*`/`handle*` functions are
      left in place in `motion_commands.cpp`, source-unchanged, simply
      uncalled by the returned vector (they still compile — clangd/IDE
      diagnostics flag them `-Wunused-function`, which is expected and
      matches the "unregistered not deleted" treatment used for the other
      families; the project's build has no `-Werror` gate on this warning).
- [x] `handleS`/`parseS` rewritten: `S <left> <right>` parses two signed
      ints, `±1000` range-checked (`parseS`'s existing range check is
      reused verbatim). `handleS` drops `BodyKinematics::forward()` and the
      `bb.motionIn` post; builds `msg::WheelTargets`/`msg::DrivetrainCommand`
      inline and posts to `bb.driveIn`, mirroring `DEV DT WHEELS`'s own
      construction idiom exactly (`source/commands/dev_commands.cpp`
      lines ~846-862: `wt.w_[0].speed.has = true; wt.w_[0].speed.val = left;`
      pattern, `cmd.setWheels(wt)`, `b.driveIn.post(cmd)`). Reply stays
      `OK drive l=.. r=..`.
- [x] **Decide and document** whether `parseS` continues to accept (and
      silently ignore) `stop=`/`sensor=` kv tokens now that `S` no longer
      evaluates stop conditions, or rejects them as `badarg`. Recommended:
      reject as `badarg` — accepting and silently dropping a wire argument
      the caller believes will be honored is confusing and contradicts the
      "read like a shopping list" clarity goal. If accepted, `parseS`'s
      `packStopKVs()` call is removed along with `collectStopClauses()`'s
      call site in `handleS`.

      **Decision (team-lead, implemented): reject as `badarg`.** `parseS`
      now calls `kvFind(kvs, nkv, "stop")`/`kvFind(kvs, nkv, "sensor")` and
      returns `{ok=false, err.code="badarg", err.detail="stop"|"sensor"}`
      if either key is present, instead of calling `packStopKVs()`.
      `handleS` no longer calls `collectStopClauses()`. `packStopKVs()`/
      `collectStopClauses()`/`replyStopBadarg()` themselves are untouched —
      `T`/`D`/`R`/`TURN`/`RT` still call them.
- [x] `handleStop` rewritten: drops the `msg::PlannerCommand`/`bb.motionIn`
      post; posts a NEUTRAL `msg::DrivetrainCommand` to `bb.driveIn`. Reply
      stays `OK stop`.

      **Reopened/corrected (2026-07-09): `buildDrivetrainStop()` was the
      wrong shape.** The originally-implemented version posted
      `buildDrivetrainStop(msg::Neutral::BRAKE)` (dev_commands.h's
      `{NEUTRAL, standby=true}` helper) — this compiled and passed the
      manual sim check below (`STOP` → `OK stop`) but did **not** physically
      neutralize the wheels: `Subsystems::Drivetrain::apply()` processes
      `standby=true` AFTER the NEUTRAL arm, flipping `active_` back to
      `false` in the same call, and `Rt::MainLoop::routeOutputs()` only
      posts the computed wheel command to `bb.motorIn[]` when
      `drivetrain_.active()` — so the neutral command was silently dropped
      and the motors kept spinning at their last commanded speed. Fixed by
      building the `msg::DrivetrainCommand{NEUTRAL}` inline in `handleStop`
      WITHOUT setting `standby` (default `Opt<bool>{has=false}`), so the
      drivetrain stays active and `routeOutputs()` passes the neutral
      through to `bb.motorIn[]`. `buildDrivetrainStop()` itself is
      unchanged — `DEV STOP`/`DEV DT STOP`/the loop's watchdog-fire path
      still use its `standby=true` shape and are unaffected. Verified live
      in sim: `S 200 200` → pwm/vel ≈ (62, 250 mm/s) both wheels; `STOP` →
      pwm instantly (0.0, 0.0), vel decays 250 → ~0.29 mm/s over the next
      480 ms (never re-driven); a subsequent `S 200 200` re-spins both
      wheels (pwm/vel ≈ (61, 245) again) — the drivetrain was not wedged
      inactive.
- [x] `just build-sim` compiles cleanly (compile-level proof the rewrite is
      wired correctly; the full `tests/sim/` suite is NOT expected green
      after this ticket — see architecture-update.md's Impact section and
      ticket 003).
- [x] A quick manual/temporary sim check (not necessarily a committed test,
      since ticket 003 owns the committed focused suite) confirms:
      `sim.command("S 200 200")` → `OK drive l=200 r=200`,
      `sim.command("STOP")` → `OK stop`, `sim.command("PING")` → `OK`,
      `sim.command("HELLO")` → `DEVICE:...`.
- [x] `T`/`D`/`R`/`TURN`/`RT`/`G` handler/parser source in
      `motion_commands.cpp` is untouched (only reachability changes, per
      the decision above) — diff review confirms no logic edits to those
      functions.

## Implementation Plan

**Approach**: Wire-surface-only change. No loop/composition-root edits
(that's ticket 002) — `Planner`/`PoseEstimator`/watchdogs keep running
exactly as today for the duration of this ticket; `S`/`STOP` simply stop
feeding them.

**Files to modify**:
- `source/runtime/command_router.cpp` — `buildTable()`.
- `source/commands/motion_commands.cpp` — `parseS`, `handleS`, `handleStop`,
  and (per the decision above) possibly `motionCommands()`'s own returned
  table.
- `source/commands/motion_commands.h` — doc-comment updates only (the file
  header's description of what this family does no longer matches "posts
  to `bb.motionIn`" for `S`/`STOP` specifically).

**Testing plan**:
- Existing: `just build-sim` must succeed. Do not chase `tests/sim/` green
  here — ticket 003 owns that.
- New: none committed by this ticket (ticket 003 writes the committed
  focused suite); use ad hoc `sim.command(...)` calls to sanity-check the
  four verbs during development.

**Documentation updates**: none required by this ticket (protocol-v2.md
currency is explicitly deferred — architecture-update.md Step 7, item 2).
