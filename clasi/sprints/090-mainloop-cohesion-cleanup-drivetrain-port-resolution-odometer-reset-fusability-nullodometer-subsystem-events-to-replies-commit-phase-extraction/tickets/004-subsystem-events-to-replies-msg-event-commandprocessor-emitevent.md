---
id: '004'
title: "Subsystem events to replies \u2014 msg::Event + CommandProcessor::emitEvent"
status: in-progress
use-cases:
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: subsystem-events-to-replies.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Subsystem events to replies — msg::Event + CommandProcessor::emitEvent

## Description

Introduce `msg::Event` (`source/messages/`) and
`CommandProcessor::emitEvent()` as the single wire-layer authority for ALL
`EVT` formatting, replacing the `snprintf`s currently inline in
`main_loop.cpp` (the Planner "done `<verb>`" event and the loop's own
`safety_stop` event) and generalizing the existing `dev_watchdog` bare-name
emission. Planner's private `Event` struct retypes to `msg::Event` and
gains a persisted `verb` field, threaded through a new field on
`msg::PlannerCommand` (populated by `motion_commands.cpp`'s handlers,
which already compute the verb for `Rt::MotionCommand::verb`).
`MainLoop::activeVelocityVerb_` is **RETAINED** — it independently gates
the stream-watchdog's S-vs-R distinction, unrelated to event formatting —
only its use for event-NAME composition (`motionVerbForMode()`/
`activeModeBeforeTick`) is removed. Independent of the odometer work
(tickets 002/003) logically; depends on ticket 003 only by
file-serialization (all five tickets share `main_loop.cpp`).

**Formatting does NOT go into the subsystem.** `Subsystems::Planner` emits
a typed `msg::Event`, never wire text — the `EVT` grammar stays in
`CommandProcessor` (the wire layer). Do not let a `snprintf`/wire string
creep into `planner.{h,cpp}` — that would move the smell one layer down
and break the command(wire-inbound)/message(internal) boundary
(`.claude/rules/naming-and-style.md` §4).

## Acceptance Criteria

- [ ] `msg::Event` exists in `source/messages/` (hand-authored or
      `protos/`-generated — implementer's call) carrying a discriminant
      (e.g. `kind: GOAL_DONE | NAMED`), a verb (meaningful iff
      `GOAL_DONE`), a standalone name (meaningful iff `NAMED`, e.g.
      `"dev_watchdog"`/`"safety_stop"`), an optional reason token, and an
      optional correlation id.
- [ ] `msg::PlannerCommand` gains a `verb` field (`protos/planner.proto` +
      regenerated `source/messages/planner.h`);
      `source/commands/motion_commands.cpp`'s handlers populate it
      identically to how they already populate `Rt::MotionCommand::verb`
      (empty for S/T/D/G, `"R"`/`"TURN"`/`"RT"` otherwise).
- [ ] `Subsystems::Planner`'s private `Event` struct retypes to
      `msg::Event`; `Planner::apply()` persists the new `cmd.verb` field
      (mirrors how `corrId_` already persists across `apply()` →
      goal-completion); `hasEvent()/takeEvent()` return `msg::Event` with
      `kind = GOAL_DONE`, `verb` set from the persisted field, `reason`
      from the fired stop condition's token, `corrId` from `corrId_`.
- [ ] `CommandProcessor::emitEvent(const msg::Event&, ReplyFn, void* ctx)`
      is the ONE place `EVT` text is assembled: for `kind == GOAL_DONE`,
      composes the wire name as `"done " + verb` (exactly matching
      today's `motionVerbForMode()` output) and body as
      `[#<corrId> ]reason=<reason>`; for `kind == NAMED`, uses `name`
      verbatim and `reason=<reason>` (or no body) — built on the existing
      `replyEvt()` primitive (unchanged), the same way `replyOKf` is built
      on `replyOK`.
- [ ] `main_loop.cpp` contains ZERO `snprintf` calls building `EVT` text;
      the watchdog-fire and `safety_stop` events are constructed as
      `msg::Event` values (`kind = NAMED`) and routed through
      `emitEvent()`, identically to how Planner's own event is drained and
      routed.
- [ ] `motionVerbForMode()` and the `activeModeBeforeTick` local are
      removed from `main_loop.cpp`.
- [ ] `MainLoop::activeVelocityVerb_` is NOT removed — confirm by grep
      that it still exists and is still read by the stream-watchdog gate
      (`activeVelocityVerb_[0] == '\0'`) exactly as before.
- [ ] `Subsystems::Planner` gains no dependency on `CommandProcessor` or
      any wire-text-producing header (verified by grep: no
      `#include "commands/command_processor.h"` and no `snprintf` in
      `planner.{h,cpp}`).
- [ ] `uv run python -m pytest tests/sim` is green; every existing
      EVT-format assertion (watchdog-fire, motion-done reason tokens for
      every stop kind, safety_stop) produces BYTE-IDENTICAL wire text
      before and after this ticket.

## Implementation Plan

**Approach**:
1. Design `msg::Event`'s exact field layout (deliberately left
   ticket-owned by architecture-update.md Step 4/Open Question 2) — a
   `Kind` enum, `verb`/`name`/`reason`/`corrId` char arrays sized to match
   existing usage (`corrId[64]` mirrors `PlannerCommand.corr_id`;
   `verb[8]` mirrors `Rt::MotionCommand::verb`; `reason[16]` mirrors
   `Planner::Event::reason`; `name` sized to fit `"dev_watchdog"`/
   `"safety_stop"`, e.g. `char name[16]`).
2. Add the `verb` field to `protos/planner.proto`'s `PlannerCommand`
   message; regenerate `source/messages/planner.h`
   (`scripts/gen_messages.py`).
3. In `motion_commands.cpp`, set `cmd.verb` alongside the existing
   `mc.verb` population in every handler that currently sets the latter
   (S/T/D/R/TURN/RT/G).
4. In `planner.h`/`planner.cpp`: add a persisted `verb_` field (alongside
   `corrId_`), set from `cmd.verb` wherever `corrId_` is set; retype the
   private `Event` struct to `msg::Event`; update `queueEvent()` to
   populate `kind = GOAL_DONE`, `verb = verb_`, `reason`, `corrId =
   corrId_`.
5. Add `CommandProcessor::emitEvent()` to `command_processor.{h,cpp}`,
   built on the existing `replyEvt()`.
6. In `main_loop.cpp`: replace the Planner-event drain block's
   `snprintf`s with a call to `emitEvent(planner_.takeEvent(),
   serialReply_, serialCtx_)`; replace the `dev_watchdog`/`safety_stop`
   `replyEvt()` calls with constructing a `msg::Event{kind=NAMED,
   name="dev_watchdog"}` / `{kind=NAMED, name="safety_stop",
   reason="watchdog"}` and calling `emitEvent()` on each; remove
   `motionVerbForMode()` and the `activeModeBeforeTick` local entirely;
   confirm `activeVelocityVerb_` remains, used only by the stream-watchdog
   gate.

**Files to modify**: `protos/planner.proto`, `source/messages/planner.h`
(regenerated), `source/messages/` (new event message file — e.g.
`event.h`/`event.proto`, or hand-authored, per implementer's call),
`source/commands/command_processor.{h,cpp}`,
`source/commands/motion_commands.cpp`, `source/subsystems/planner.{h,cpp}`,
`source/runtime/main_loop.cpp`.

**Documentation updates**: none beyond the new type/method's own doc
comments (no wire format changes — `docs/protocol-v2.md`'s EVT grammar
section is unaffected).

## Testing

- **Existing tests to run**: full `tests/sim`, with explicit attention to
  any test asserting exact EVT wire text (watchdog-fire, `done <verb>
  reason=<token>` for every stop kind DISTANCE/TIMED/TURN/ROTATION/GOTO,
  `safety_stop reason=watchdog`) — grep `tests/sim/unit/` for `"EVT "`/
  `replyEvt`/`"done "` to find them.
- **New tests to write**: none required if existing EVT-format coverage
  is exhaustive; if any verb/reason combination lacks coverage, add a
  targeted assertion (implementer's call).
- **Verification command**: `uv run python -m pytest tests/sim`
