---
status: done
sprint: 093
tickets:
- 093-002
- 093-004
---

# Get wire-output (events + telemetry) out of the main loop

## Context

`Rt::MainLoop::tick()` — the cyclic executive in
[source/runtime/main_loop.cpp](source/runtime/main_loop.cpp) — currently builds
`msg::Event` values **inline with `std::strncpy`** and then calls
`CommandProcessor::emitEvent(...)` **synchronously** to format and send them, mid
control-pass. It also calls `telemetryEmit(...)` synchronously to sample, format,
and send a ~300-byte TLM frame. The control loop has no business copying strings
or driving the wire.

The fix is the pattern the codebase already uses everywhere else on the command
plane: a **message is a class that constructs itself** (its constructor/factory
does the byte copies), the loop **instantiates it and posts it on a queue**, and a
**wire-layer drain** — outside the loop — formats and sends. The queue primitives
([source/runtime/queue.h](source/runtime/queue.h)) already accept any `msg::` type,
`msg::Event` is hand-authored (safe to give a constructor), and `Blackboard` has no
include cycle back to the wire layer.

End state: `MainLoop` posts typed messages to two new outbound queues and does
**zero** string copying and **zero** emit/format/send. It drops all four
`ReplyFn`/`ctx` members entirely — it becomes pure control logic.

Decisions taken (this session): **static factory methods** for `msg::Event`
construction; **both events and telemetry** move off the loop.

## Changes

### 1. `msg::Event` — construct itself ([source/messages/event.h](source/messages/event.h))

Keep it trivially-copyable (must live in the queues): only add a defaulted default
ctor + static factories (implicit copy ctor/dtor stay trivial). Do the bounded
copies inside the class with a tiny inline copy loop (keep the header
dependency-free — no `<cstring>`, matching queue.h's ethos).

```cpp
Event() = default;
static Event named(const char* name, const char* reason = nullptr);   // NAMED shape
static Event goalDone(const char* verb, const char* reason, const char* corrId); // GOAL_DONE shape
```

- `named()` sets `kind = NAMED`, copies into `name[16]`/`reason[16]`.
- `goalDone()` sets `kind = GOAL_DONE`, copies into `verb[8]`/`reason[16]`/`corrId[64]`.
- A `nullptr` string leaves the buffer empty (preserves today's "empty reason omits
  the body" behavior for `dev_watchdog`).

### 2. New outbound queues + telemetry request type

- [source/runtime/commands.h](source/runtime/commands.h): add
  `struct TelemetryRequest { uint32_t now; Subsystems::Channel channel; };`  // [ms]
- [source/runtime/blackboard.h](source/runtime/blackboard.h): add
  `#include "messages/event.h"`, then two members:
  - `WorkQueue<msg::Event, 8> eventsOut;`      // loop → wire layer
  - `Mailbox<TelemetryRequest> telemetryOut;`  // loop → wire layer (latest-wins; drained each pass)

### 3. `MainLoop` — post, never emit ([source/runtime/main_loop.cpp](source/runtime/main_loop.cpp) + [.h](source/runtime/main_loop.h))

- **dev_watchdog** (serviceWatchdogs, ~L116-119): replace the `msg::Event ev; strncpy...; emitEvent(...)`
  block with `bb.eventsOut.post(msg::Event::named("dev_watchdog"));`
- **safety_stop** (~L286-290): `bb.eventsOut.post(msg::Event::named("safety_stop", "watchdog"));`
- **planner event** (~L296-303): `if (planner_.hasEvent()) bb.eventsOut.post(planner_.takeEvent());`
- **periodic telemetry** (~L316-324): keep the timing gate + last-emit bookkeeping, but
  replace the `replyFn` selection and `telemetryEmit(...)` call with
  `bb.telemetryOut.post(TelemetryRequest{now, bb.telemetryChannel});`
- **`activeVelocityVerb_`** (the last string copy, L155/264-265/277): replace the
  `char[8]` + `strncpy` with `bool activeVelocityVerbPresent_`. Set it
  `= (mc.verb[0] != '\0')`; the L277 gate becomes `!activeVelocityVerbPresent_`.
  (Reading one char is not a copy.) Update the member's doc comment in main_loop.h.
- Remove the four `serialReply_/serialCtx_/radioReply_/radioCtx_` members and the
  matching constructor parameters; drop the now-unused includes `<cstring>`,
  `commands/command_processor.h`, `commands/telemetry_commands.h`. Keep
  `hal/capability/hal_command.h` (still used by the broadcast/drivetrain edges).
  Update the stale "reply sink" doc comments in main_loop.h.

### 4. Wire-layer drain (new small module)

New `source/commands/loop_output.{h,cpp}` — the one place the loop's outbound queues
are drained and sent (includes `command_processor.h` for `emitEvent` and
`telemetry_commands.h` for `telemetryEmit`):

```cpp
void drainLoopOutputs(Rt::Blackboard& bb,
                      ReplyFn serialFn, void* serialCtx,
                      ReplyFn radioFn, void* radioCtx);
```

Drains `bb.eventsOut` FIFO → `CommandProcessor::emitEvent(ev, serialFn, serialCtx)`
(events are serial-only, as today), then `bb.telemetryOut` → `telemetryEmit(bb,
req.now, chosen-by-channel fn/ctx)`. `telemetryEmit` and `emitEvent` are unchanged —
they remain the sole owners of wire-text grammar; `Telemetry::tick`/`buildTlmFrame`
still do sampling/formatting, now at drain time (same committed `bb`, drain runs
immediately after `tick()` before the slack phase mutates anything).

### 5. Composition roots — construct without reply sinks, drain after tick

- [source/main.cpp](source/main.cpp): drop the reply args from the `Rt::MainLoop`
  ctor (~L196-197); after `loop.tick(bb, now)` (~L210) add
  `drainLoopOutputs(bb, serialReply, &comm, radioReply, &comm);`
- [tests/_infra/sim/sim_api.cpp](tests/_infra/sim/sim_api.cpp): drop the reply args
  from the `MainLoop` member ctor (~L233-234); after each `s->loop.tick(...)`
  (`sim_tick` ~L309 and `sim_command_on` ~L359) add
  `drainLoopOutputs(s->bb, storeReply, &s->asyncStore, storeReply, &s->asyncStore);`
  — so loop-originated events/telemetry still land in `asyncStore`
  (`sim_get_async_evts` unchanged). `sim_command` is a wrapper over `sim_command_on`,
  so no third site.

### 6. Consistency follow-through (recommended, same class)

- [source/subsystems/planner.cpp](source/subsystems/planner.cpp) `queueEvent()`:
  replace the three manual char-copy loops with
  `heldEvent_ = msg::Event::goalDone(verb_, reason, corrId_);` — same anti-pattern,
  now uses the new factory.
- STREAM/SNAP handlers in
  [source/commands/telemetry_commands.cpp](source/commands/telemetry_commands.cpp)
  keep calling `telemetryEmit` directly (command replies in the slack phase, not the
  loop) — out of scope.

## Notes / risks

- **Trivial-copyability**: adding only a defaulted default ctor + static factories
  keeps `msg::Event` trivially copyable, so `WorkQueue<msg::Event,8>` /
  `T buf_[N]={}` / `return T{}` still compile. Verify with a sim build.
- **Ordering preserved**: events post in the same intra-pass order (dev_watchdog →
  safety_stop → planner) and drain FIFO → identical wire order.
- **Timing preserved**: drain runs right after `tick()`, before slack mutates `bb`,
  so telemetry samples the same committed state it did when emitted inline.
- Grep tests for direct `Rt::MainLoop(` constructions and update signatures
  (e.g. any harness under `tests/sim/`); behavior-level assertions via the sim async
  store should be unaffected.
- Execution follows CLASI (sprint/ticket) unless run out-of-process.

## Verification

1. **Build sim**: `just build-sim` (must compile — proves trivial-copyability &
   include graph).
2. **Sim suite**: `uv run python -m pytest` (collects `tests/sim/` only). Confirm
   event/telemetry-related tests (via `sim_get_async_evts`) still pass.
3. **Standing hardware bench gate** (HAL/command-surface touch — required by
   `.claude/rules/hardware-bench-testing.md`): build + flash
   (`just build-clean` then `mbdeploy deploy <full-UID> --hex MICROBIT.hex`), drive
   on the stand, and confirm over the real link:
   - `EVT dev_watchdog` fires on serial-silence while motors run;
   - `EVT safety_stop reason=watchdog` fires on stream timeout;
   - a completed motion still emits `EVT done <verb> ...`;
   - periodic `TLM ...` frames still stream at the configured period on the selected
     channel (SERIAL and RADIO).
