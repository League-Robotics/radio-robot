---
id: '002'
title: Scope firmware motion-watchdog resets to keepalive and motion verbs only
status: done
use-cases:
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: stop-delivery-and-keepalive-watchdog-architecture.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Scope firmware motion-watchdog resets to keepalive and motion verbs only

## Description

CR-05a (part of CR-04/CR-05, high). `LoopScheduler::runCommsIn`
(`LoopScheduler.cpp:49-90`) calls `sched.resetWatchdog(now)` after **every**
inbound line, regardless of verb — `GET`, `SNAP`, and any other non-motion
query resets the same timestamp `+`/`VW` do. `LoopScheduler::run_test()` has
the identical pattern a second time, and `tests/_infra/sim/sim_api.cpp`'s
`sim_command()` has a third, explicitly-commented parallel copy ("mirrors
LoopScheduler's resetWatchdog"). This means a host that only polls telemetry
(no `+`, no motion commands) can silently mask a stalled motion-watchdog
check — the firmware cannot distinguish "host is alive and driving" from
"host is alive and merely polling."

Fix: classify at the command-descriptor table (the existing single source of
truth for per-command properties like `CMD_ACCESS_HARDWARE`), gate the
watchdog reset at its handful of call sites. See `architecture-update.md`
Step 4-5 item 2 for the full design.

Depends on ticket 001 for file-locality only (both touch
`MotionCommands.cpp`) — no functional dependency.

This ticket, together with tickets 003-005, fully addresses issue
`stop-delivery-and-keepalive-watchdog-architecture.md` (CR-04/CR-05); the
issue archives once all four are done.

## Acceptance Criteria

- [x] `source/types/CommandTypes.h` gains `static constexpr uint8_t
      CMD_MOTION_WATCHDOG = 2;` alongside the existing `CMD_ACCESS_HARDWARE`.
- [x] Every motion-verb descriptor in `source/commands/MotionCommands.cpp`
      (`S`, `T`, `D`, `G`, `R`, `TURN`, `RT`, `VW`, `_VW`, `X`, `STOP`) and
      the `+` descriptor in `source/commands/SystemCommands.cpp` have
      `CMD_MOTION_WATCHDOG` OR'd into their `flags` value. No other
      descriptor changes.
- [x] `source/commands/CommandProcessor.{h,cpp}` gains a private
      `_lastDispatchFlags` member (set in `dispatchTable()` immediately after
      a successful parse, before the enqueue-vs-immediate-dispatch branch)
      and a public accessor `bool lastCommandResetsWatchdog() const`.
- [x] `source/robot/LoopScheduler.cpp`'s `runCommsIn` (both the serial and
      radio branches) and `run_test` replace their unconditional
      `resetWatchdog(now)` with `if (cmd.lastCommandResetsWatchdog())
      resetWatchdog(now);`.
- [x] `tests/_infra/sim/sim_api.cpp`'s `sim_command()` applies the identical
      gate to its `s->_ts.watchdogMs = ...` line, reading the same
      `CommandProcessor` classification (no separate/duplicated
      "which commands count" logic in the sim).
- [x] `SystemCommands.cpp`'s `handleKeepalive` is left unchanged (its own
      explicit `resetWatchdog` call is now redundant-but-harmless with the
      gate for the `+` line specifically).
- [x] New sim test: an active open-ended `VW` session that receives only
      `GET`/`SNAP` traffic (no `+`, no fresh `VW`) for longer than
      `sTimeoutMs` safety-stops.
- [x] Regression: an active `VW` session kept alive by `+` alone (no `VW`
      resend) does NOT trip the watchdog — narrowing the classification must
      not break the legitimate keepalive path.
- [x] `tests/simulation/unit/test_watchdog_exemption.py` stays green (the
      TIME-stop exemption for `T`/`D`/`G`/`TURN`/`RT` is unaffected by this
      change).
- [x] Full default sim suite green.

## Implementation Plan

**Approach**: Add one bitmask flag, tag ~12 existing descriptors
(declarative, not logic duplication), thread one boolean accessor through
`CommandProcessor`, and gate the three existing `resetWatchdog`/
`_ts.watchdogMs` call sites (firmware `runCommsIn`, firmware `run_test`, sim
`sim_command`) on it. No new classes, no new files.

**Files to modify**:
- `source/types/CommandTypes.h` — new `CMD_MOTION_WATCHDOG` constant.
- `source/commands/MotionCommands.cpp` — OR the flag into the 11 motion-verb
  descriptors in `getMotionCommands()`.
- `source/commands/SystemCommands.cpp` — OR the flag into the `+` descriptor.
- `source/commands/CommandProcessor.h`, `CommandProcessor.cpp` — new private
  member + accessor, set in `dispatchTable()`.
- `source/robot/LoopScheduler.cpp` — gate `runCommsIn`'s two `resetWatchdog`
  calls and `run_test`'s one call.
- `tests/_infra/sim/sim_api.cpp` — gate `sim_command()`'s watchdog-reset
  line.

**Testing plan**:
- New sim test exercising "GET/SNAP-only traffic during open-ended VW ->
  watchdog still fires at sTimeoutMs" (this scenario was previously
  impossible to construct because any line reset the watchdog).
- Regression test confirming `+`-only keepalive still holds VW alive
  (guards against over-narrowing).
- Run `tests/simulation/unit/test_watchdog_exemption.py` and the full
  default suite.

**Documentation updates**: `architecture-update.md` already documents this
change (Step 4-5 item 2, Impact table). No wire-protocol change — this is
purely a firmware-internal timing-source narrowing; no new command, no
changed reply shape.
