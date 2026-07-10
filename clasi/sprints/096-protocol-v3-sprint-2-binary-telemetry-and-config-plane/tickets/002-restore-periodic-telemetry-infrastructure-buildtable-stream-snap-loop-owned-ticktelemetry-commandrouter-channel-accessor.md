---
id: '002'
title: Restore periodic telemetry infrastructure (buildTable STREAM/SNAP + loop-owned
  tickTelemetry + CommandRouter channel accessor)
status: open
use-cases: [SUC-002]
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Restore periodic telemetry infrastructure (buildTable STREAM/SNAP + loop-owned tickTelemetry + CommandRouter channel accessor)

## Description

Build the periodic-emission mechanism the STREAM verb's design has always
assumed but which sprint 093's loop rewrite deleted (Decision 1,
architecture-update.md). Independent of the wire schema (ticket 001) and
of any binary content — this ticket only makes TEXT periodic emission
work again, giving this sprint's later "text vs. binary TLM at matched
rates" bench criterion a live baseline, and lays the loop-owned plumbing
ticket 005's binary `stream` arm will hook into.

**Approach**:
1. Re-add `telemetryCommands()` to `Rt::CommandRouter::buildTable()`
   (`command_router.cpp`), restoring `STREAM`/`SNAP` to the live text
   table. Do NOT re-add `configCommands()` (SET/GET) — Decision 1 is
   explicit that config's binary arm (ticket 004) is the only live path
   this sprint; text SET/GET stays parked/unregistered.
2. Add a new free function `tickTelemetry(Rt::Blackboard& bb,
   Rt::CommandRouter& router, uint32_t now)` in `source/commands/
   telemetry_commands.{h,cpp}`, alongside the existing `telemetryEmit()`.
   It checks `bb.telemetryPeriod > 0` and elapsed time
   (`bb.telemetryLastEmitMs`/`bb.telemetryHasLastEmit`, the SAME fields
   `handleStream()` already maintains), resolves `bb.telemetryChannel`
   to a live `ReplyFn`/`void*` via the new `CommandRouter` accessor
   (below), and — for THIS ticket's scope — always calls the existing
   text path (`telemetryEmit()`/`buildTlmFrame()`). Add the
   `bb.telemetryBinary` field (bool, default false) to `Rt::Blackboard`
   now (`blackboard.h`) so `tickTelemetry()` has the branch point ready;
   the actual binary-path call is wired in by ticket 003 (formatter must
   exist first) and set to `true` by ticket 005 (the `stream` arm) — this
   ticket's own behavior is unconditionally text, since nothing sets
   `telemetryBinary` yet.
3. Add a small accessor to `Rt::CommandRouter` (`command_router.{h,cpp}`)
   that resolves a `Subsystems::Channel` (SERIAL/RADIO/NONE) to the
   matching `ReplyFn`/`void*` pair, reusing the SAME private
   `serialReply_`/`serialCtx_`/`radioReply_`/`radioCtx_` state `route()`
   already branches on — do not add new state, just a second entry point
   usable outside an active `route()` call.
4. Call `tickTelemetry(bb, router, now)` once per pass from BOTH
   `source/main.cpp`'s bare `for(;;)` loop (a peer of the existing
   `comm.tick(now)`/`router.route(...)` calls) AND `tests/_infra/sim/
   sim_api.cpp`'s advance step (alongside its existing `s->loop.tick(...)`
   call) — the same "both real hardware and sim call the identical
   function" invariant `Rt::MainLoop::tick()` already establishes for
   motion.

Per Open Question 5: this new periodic tick does NOT reproduce
`handleStream()`'s old "immediate first frame concatenated into the SAME
reply as the ACK" micro-optimization — the first frame now arrives one
pass later via the normal `!telemetryHasLastEmit` trigger. This is a
deliberate, documented behavior refinement; do not try to preserve the
old same-reply-concatenation timing.

**Files to modify**: `source/commands/telemetry_commands.{h,cpp}`,
`source/runtime/command_router.{h,cpp}`, `source/runtime/blackboard.h`,
`source/main.cpp`, `tests/_infra/sim/sim_api.cpp`.

## Acceptance Criteria

- [ ] `STREAM 50` followed by waiting >= 200ms yields >= 3 periodic
      `TLM ...` text frames with strictly increasing `seq=`, on the sim
      harness.
- [ ] `STREAM 0` stops periodic emission; `SNAP` still works standalone
      (one-shot, on its own dispatch channel, unaffected by the periodic
      tick).
- [ ] `tickTelemetry()` is called once per pass from both `source/main.cpp`
      and `tests/_infra/sim/sim_api.cpp`, with identical behavior.
- [ ] `Rt::CommandRouter`'s new channel-resolution accessor reuses the
      existing private reply-channel state — no new `ReplyFn`/`void*`
      storage duplicated.
- [ ] `bb.telemetryBinary` defaults to `false`; this ticket's own text
      emission behavior is unconditional (the binary branch is inert
      until tickets 003/005 land).
- [ ] The existing (pre-096) sim suite stays green — no existing text
      verb's behavior changes; `configCommands()` (SET/GET) remains
      unregistered.
- [ ] 095's differential codec gate is unaffected (this ticket touches no
      wire codec code).

## Testing

- **Existing tests to run**: full `tests/sim` suite (`uv run python -m
  pytest tests/sim`) — confirm no existing verb's behavior regresses.
- **New tests to write**: a sim-level test driving `STREAM <ms>` across
  multiple passes and asserting monotonically increasing `seq=` and
  correct on/off behavior (`STREAM 0`).
- **Verification command**: `just build-sim && uv run python -m pytest
  tests/sim`
