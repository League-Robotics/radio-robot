---
status: pending
sprint: 090
---

# General subsystem event → reply mechanism (remove EVT formatting from the main loop)

Extracted from `sprint-089-omnibus.md` entry 3 (stakeholder design discussion 2026-07-07).

## Motivation

The main loop is the central debugging surface — the one place you read to
reason about *sequencing* and x[k]/commit timing. Every `snprintf` in it is
camouflage over that, and the defects it introduces (a missing space, a wrong
decimal count) are wire-formatting bugs with nothing to do with control flow.
Wire-format string assembly does not belong in `MainLoop::tick`.

Today the loop hand-assembles `EVT` wire grammar in two places:
- `main_loop.cpp:247-260` — the Planner "done" event: three `snprintf`s building
  `body` (`[#<corr> ]reason=<r>`) and `name` (`done <verb>`), then `replyEvt`.
- `main_loop.cpp:236-239` — the `safety_stop` watchdog event.

## The feature

Make it possible **in general** for subsystems to emit events that get turned
into replies, with formatting owned by the wire layer. The loop only routes.

The Planner already produces a typed `Subsystems::Planner::Event{reason,
corrId}` via `hasEvent()/takeEvent()` — the loop is merely the thing formatting
it. Generalize that into a first-class capability.

## Direction

- **`msg::Event` in `source/messages/`** — a typed event `{ name/verb, corrId,
  reason }` (generalizes `Subsystems::Planner::Event`). Lives in `messages/` so
  both subsystems and `CommandProcessor` depend on it, neither on the other.
- **Subsystems expose `hasEvent()/takeEvent()` → `msg::Event`** (Planner already
  does; retype + add the verb field).
- **One wire-layer emitter: `CommandProcessor::emitEvent(const msg::Event&,
  ReplyFn, ctx)`** owns the `EVT <name> <body>` grammar and the `[#<corr>
  ]reason=` shaping. ALL `snprintf` lives here.
- **Main loop collapses to routing:** for each event producer, if it has an
  event, hand it to `emitEvent` with the target channel. Zero formatting in the
  loop.

## Key decision: the event is self-describing (data in, formatting out)

- **Data goes into the event.** The Planner already carries `reason`/`corrId`;
  the only missing piece is the verb, which can ride the same path `corrId`
  already does — `motion_commands.cpp` has `mc.verb`; thread it through
  `PlannerCommand → Planner → Event`. Then the event names itself and the loop
  can drop `motionVerbForMode()`/`activeModeBeforeTick` for emission entirely
  (more loop-local state removed — a direct win).
- **Formatting does NOT go into the subsystem.** Subsystems emit a typed
  `msg::Event`, never wire text. The `EVT` grammar stays in `CommandProcessor`
  (the wire layer). A subsystem `snprintf`-ing wire strings would just move the
  smell one layer down and break the command(wire-inbound)/message(internal)
  boundary — the subsystem speaks typed messages; the wire layer speaks wire.

## Loop-originated events

The loop's own `safety_stop` event (line 236) is not subsystem-produced, but it
routes through the SAME `emitEvent` by constructing a `msg::Event` — so
formatting is uniform for loop-synthesized and subsystem-produced events alike.

## Possible bigger version (noted, not folded in)

A full blackboard output-event queue drained by a single `routeOutputs`-style
step, parallel to the command-input plane. The minimal version above does not
require it; note as a future option.

## Scope

- `source/messages/` (new `msg::Event`)
- `source/commands/CommandProcessor.{h,cpp}` (`emitEvent`)
- `source/subsystems/planner.{h,cpp}` (retype `Event`, carry verb)
- `source/runtime/main_loop.cpp` (drain→route; remove `snprintf` +
  `motionVerbForMode`-for-emission)
