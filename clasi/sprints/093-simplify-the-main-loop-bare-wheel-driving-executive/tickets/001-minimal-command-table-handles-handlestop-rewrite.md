---
id: '001'
title: Minimal command table + handleS/handleStop rewrite
status: open
use-cases: [SUC-001, SUC-002, SUC-003]
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

- [ ] `Rt::CommandRouter::buildTable()` (`source/runtime/command_router.cpp`)
      registers only `systemCommands()` + `motionCommands()`; the calls to
      `devCommands()`, `telemetryCommands()`, `configCommands()`,
      `poseCommands()`, `otosCommands()` are removed (the `#include`s for
      those families' headers may go too, but the family `.cpp`/`.h` files
      themselves are untouched).
- [ ] `motionCommands()` (`source/commands/motion_commands.cpp`) still
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
- [ ] `handleS`/`parseS` rewritten: `S <left> <right>` parses two signed
      ints, `±1000` range-checked (`parseS`'s existing range check is
      reused verbatim). `handleS` drops `BodyKinematics::forward()` and the
      `bb.motionIn` post; builds `msg::WheelTargets`/`msg::DrivetrainCommand`
      inline and posts to `bb.driveIn`, mirroring `DEV DT WHEELS`'s own
      construction idiom exactly (`source/commands/dev_commands.cpp`
      lines ~846-862: `wt.w_[0].speed.has = true; wt.w_[0].speed.val = left;`
      pattern, `cmd.setWheels(wt)`, `b.driveIn.post(cmd)`). Reply stays
      `OK drive l=.. r=..`.
- [ ] **Decide and document** whether `parseS` continues to accept (and
      silently ignore) `stop=`/`sensor=` kv tokens now that `S` no longer
      evaluates stop conditions, or rejects them as `badarg`. Recommended:
      reject as `badarg` — accepting and silently dropping a wire argument
      the caller believes will be honored is confusing and contradicts the
      "read like a shopping list" clarity goal. If accepted, `parseS`'s
      `packStopKVs()` call is removed along with `collectStopClauses()`'s
      call site in `handleS`.
- [ ] `handleStop` rewritten: drops the `msg::PlannerCommand`/`bb.motionIn`
      post; posts `buildDrivetrainStop(msg::Neutral::BRAKE)` (declared in
      `source/commands/dev_commands.h`, `#include`d here even though the
      `DEV` family stays unregistered from the table) to `bb.driveIn`.
      Reply stays `OK stop`.
- [ ] `just build-sim` compiles cleanly (compile-level proof the rewrite is
      wired correctly; the full `tests/sim/` suite is NOT expected green
      after this ticket — see architecture-update.md's Impact section and
      ticket 003).
- [ ] A quick manual/temporary sim check (not necessarily a committed test,
      since ticket 003 owns the committed focused suite) confirms:
      `sim.command("S 200 200")` → `OK drive l=200 r=200`,
      `sim.command("STOP")` → `OK stop`, `sim.command("PING")` → `OK`,
      `sim.command("HELLO")` → `DEVICE:...`.
- [ ] `T`/`D`/`R`/`TURN`/`RT`/`G` handler/parser source in
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
