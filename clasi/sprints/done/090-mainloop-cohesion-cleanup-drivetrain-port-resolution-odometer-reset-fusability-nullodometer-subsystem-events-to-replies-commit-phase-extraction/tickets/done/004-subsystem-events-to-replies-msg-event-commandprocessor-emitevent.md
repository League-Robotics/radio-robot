---
id: '004'
title: "Subsystem events to replies \u2014 msg::Event + CommandProcessor::emitEvent"
status: done
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

- [x] `msg::Event` exists in `source/messages/` (hand-authored or
      `protos/`-generated — implementer's call) carrying a discriminant
      (e.g. `kind: GOAL_DONE | NAMED`), a verb (meaningful iff
      `GOAL_DONE`), a standalone name (meaningful iff `NAMED`, e.g.
      `"dev_watchdog"`/`"safety_stop"`), an optional reason token, and an
      optional correlation id.
- [x] `msg::PlannerCommand` gains a `verb` field (`protos/planner.proto` +
      regenerated `source/messages/planner.h`);
      `source/commands/motion_commands.cpp`'s handlers populate it
      identically to how they already populate `Rt::MotionCommand::verb`
      (empty for S/T/D/G, `"R"`/`"TURN"`/`"RT"` otherwise).
- [x] `Subsystems::Planner`'s private `Event` struct retypes to
      `msg::Event`; `Planner::apply()` persists the new `cmd.verb` field
      (mirrors how `corrId_` already persists across `apply()` →
      goal-completion); `hasEvent()/takeEvent()` return `msg::Event` with
      `kind = GOAL_DONE`, `verb` set from the persisted field, `reason`
      from the fired stop condition's token, `corrId` from `corrId_`.
- [x] `CommandProcessor::emitEvent(const msg::Event&, ReplyFn, void* ctx)`
      is the ONE place `EVT` text is assembled: for `kind == GOAL_DONE`,
      composes the wire name as `"done " + verb` (exactly matching
      today's `motionVerbForMode()` output) and body as
      `[#<corrId> ]reason=<reason>`; for `kind == NAMED`, uses `name`
      verbatim and `reason=<reason>` (or no body) — built on the existing
      `replyEvt()` primitive (unchanged), the same way `replyOKf` is built
      on `replyOK`.
- [x] `main_loop.cpp` contains ZERO `snprintf` calls building `EVT` text;
      the watchdog-fire and `safety_stop` events are constructed as
      `msg::Event` values (`kind = NAMED`) and routed through
      `emitEvent()`, identically to how Planner's own event is drained and
      routed.
- [x] `motionVerbForMode()` and the `activeModeBeforeTick` local are
      removed from `main_loop.cpp`.
- [x] `MainLoop::activeVelocityVerb_` is NOT removed — confirm by grep
      that it still exists and is still read by the stream-watchdog gate
      (`activeVelocityVerb_[0] == '\0'`) exactly as before.
- [x] `Subsystems::Planner` gains no dependency on `CommandProcessor` or
      any wire-text-producing header (verified by grep: no
      `#include "commands/command_processor.h"` and no `snprintf` in
      `planner.{h,cpp}`).
- [x] `uv run python -m pytest tests/sim` is green; every existing
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

## Completion Notes

Implemented as planned, with one refinement to the fallback resolution
(explained below — reconciles the Implementation Plan's literal wording
with AC4's "exactly matching today's `motionVerbForMode()` output"
requirement).

- `msg::Event` hand-authored at `source/messages/event.h` (not
  `protos/`-generated — its four char arrays need four DIFFERENT sizes,
  `verb[8]`/`name[16]`/`reason[16]`/`corrId[64]`, and
  `scripts/gen_messages.py`'s string-field rule always emits a flat
  `char[64]` with no per-field override).
- `protos/planner.proto`'s `PlannerCommand` gained `string verb = 13`,
  regenerated into `source/messages/planner.h` as `char verb[64]` (the
  generator's standard string-field shape, same as `corr_id`).
  `motion_commands.cpp`'s `handleR`/`handleTURN`/`handleRT` set
  `cmd.verb` alongside the existing `mc.verb` (empty for S/T/D/G, as
  specified).
- **Refinement**: AC4 requires emitEvent's `"done " + verb` composition to
  exactly match the OLD `motionVerbForMode(mode, activeVelocityVerb)`
  output — but that function's S/T/D/G fallback letters came from the
  DriveMode, which `cmd.verb` never carries (empty for those four verbs,
  per AC2). `CommandProcessor::emitEvent()` has no DriveMode to fall back
  on (its signature is `(const msg::Event&, ReplyFn, void*)` — no mode
  parameter), and the loop's own `motionVerbForMode()`/
  `activeModeBeforeTick` are removed per AC5/AC6. The only remaining place
  that still has the DriveMode at the right moment is `Planner` itself:
  `stageCommon()` already receives the goal's resolved `mode` as a
  parameter, so a new file-scope helper `verbFallbackFor(msg::DriveMode)`
  (`planner.cpp`, same style/placement as the existing `reasonTokenFor()`)
  resolves an empty `cmd.verb` to its DriveMode-implied letter ("S"/"T"/
  "D"/"G") right there, so `verb_` is never empty by the time a goal
  completes. This is a short DATA-token lookup — the same category as
  `reasonTokenFor()`'s reason strings — not wire-text assembly, so it does
  not violate the "Planner never formats wire text" boundary; `queueEvent()`
  still just copies `verb_` verbatim into `heldEvent_.verb`, matching the
  Implementation Plan's literal step 4 exactly. Verified this reproduces
  the old output bit-for-bit for every DriveMode/verb combination the
  dispatch table can actually produce (worked through by hand against
  `velocityShapedMode()`'s STREAMING/TIMED collapse for R/T/TURN/RT, and
  confirmed empirically by the full `tests/sim` EVT-assertion suite below).
- `CommandProcessor::emitEvent()` added to `command_processor.{h,cpp}`,
  built on `replyEvt()`; reuses the EXACT buffer sizes the old
  `main_loop.cpp` snprintfs used for the GOAL_DONE path (`body[64]`,
  `name[16]`, `wbuf[96]`) so any latent truncation behavior (never
  actually reachable — the wire parser's own `corr_id` buffer caps real
  correlation ids at 15 chars) stays byte-identical.
- `main_loop.cpp`: `motionVerbForMode()` and `activeModeBeforeTick` removed;
  all three EVT sites (`dev_watchdog`, `safety_stop`, Planner's own event)
  now construct/route a `msg::Event` through `emitEvent()`; zero
  `snprintf`/`"EVT"` text remains in the file (confirmed by grep, `<cstdio>`
  include also dropped as now-unused). `activeVelocityVerb_` untouched —
  still gates the stream-watchdog's S-vs-R distinction exactly as before.
- Two test call sites in `tests/sim/unit/planner_harness.cpp`
  (`Subsystems::Planner::Event evt = ...`) updated to `msg::Event evt = ...`
  (the type still exposes the same `.reason`/`.corrId` fields the
  assertions read).

**Test results**: `uv run python -m pytest tests/sim` → `309 passed, 2
xfailed in 98.42s` — identical counts to the pre-ticket baseline (309
passed / 2 xfailed). Every EVT-format assertion (`"EVT done D reason=dist"`,
`"EVT done T reason=time"`, `"EVT done G reason=pos"`, `"EVT done RT
reason=rot"`, `"EVT done TURN reason=heading"`, `"EVT done R reason=..."`,
`"EVT dev_watchdog"`, `"EVT safety_stop reason=watchdog"`, etc., across
`test_motion_commands.py`, `test_motion_commands_arc_turn.py`,
`test_motion_commands_goto.py`, `test_motion_verbs_full_sequence.py`,
`test_motion_overshoot_regression.py`, `test_pose_commands.py`,
`test_config_pose_set_otos_surface.py`, `test_watchdog_policy.py`) passed
byte-identically.

Grep confirmations:
- `activeVelocityVerb_` retained: `source/runtime/main_loop.h:136: char
  activeVelocityVerb_[8] = "";` and still read at
  `source/runtime/main_loop.cpp:221` in the stream-watchdog gate
  (`activeVelocityVerb_[0] == '\0'`), unchanged.
- `Subsystems::Planner` has no `CommandProcessor`/wire-text dependency:
  `grep -n '#include' source/subsystems/planner.{h,cpp}` shows no
  `commands/command_processor.h`; `grep -n snprintf
  source/subsystems/planner.{h,cpp}` returns no matches.

No deviations from ticket scope (ticket 005's `commit()` extraction
untouched).
