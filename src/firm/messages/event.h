// event.h -- msg::Event: a typed, self-describing "one pending wire-reportable
// occurrence" (090-004, sprint 090's architecture-update.md; generalizes
// Subsystems::Planner's former private `Event` struct so both a subsystem
// producer and CommandProcessor -- the wire layer -- can depend on the SAME
// type, neither depending on the other, per the source issue
// (subsystem-events-to-replies.md): "Lives in messages/ so both subsystems
// and CommandProcessor depend on it, neither on the other."
//
// Hand-authored, NOT scripts/gen_messages.py-generated (the ticket's own
// "implementer's call"): this type needs FOUR differently-sized char
// arrays, each sized to match an existing caller -- `verb[8]` mirrors
// Rt::MotionCommand::verb (runtime/commands.h), `reason[16]`/`corrId[64]`
// mirror the former Subsystems::Planner::Event fields (planner.h), `name[16]`
// fits the two loop-originated literals "dev_watchdog"/"safety_stop"
// (main_loop.cpp) -- but scripts/gen_messages.py's string-field rule always
// emits a flat `char[64]` with no per-field size override, so a
// protos/event.proto round-trip cannot produce this shape. Every other
// header in this directory backs a protos/*.proto file 1:1; this one does
// not, by design.
//
// Two shapes, selected by `kind` -- the discriminant
// CommandProcessor::emitEvent() (command_processor.h) reads to decide how to
// format the EVT wire text; see that method's own doc comment for the exact
// grammar. A producer of msg::Event NEVER assembles wire text itself -- data
// goes into the event, formatting stays in the wire layer
// (.claude/rules/naming-and-style.md sec 4, command(wire-inbound) vs
// message(internal)):
//   - GOAL_DONE: a completed Planner goal. `verb` (the wire verb that staged
//     the goal -- "S"/"T"/"D"/"R"/"TURN"/"RT"/"G") and `reason` (the fired
//     stop condition's token, e.g. "dist"/"time"/"heading"/"pos"/"rot") are
//     meaningful; `corrId` is optional (empty if the staging command carried
//     none). `name` is unused.
//   - NAMED: a loop-originated event with a fixed literal name
//     ("dev_watchdog", "safety_stop") -- `name` is used verbatim as the wire
//     name; `reason` is optional (an empty reason omits the body entirely,
//     e.g. dev_watchdog's bare "EVT dev_watchdog"). `verb`/`corrId` are
//     unused.
#pragma once

#include <stdint.h>

namespace msg {

struct Event {
  enum class Kind : uint8_t {
    GOAL_DONE = 0,  // a completed Planner goal -- verb/reason[/corrId] meaningful
    NAMED = 1,      // a loop-originated named event -- name[/reason] meaningful
  };

  Kind kind = Kind::GOAL_DONE;
  char verb[8] = {};     // meaningful iff kind == GOAL_DONE
  char name[16] = {};    // meaningful iff kind == NAMED (e.g. "dev_watchdog", "safety_stop")
  char reason[16] = {};  // optional
  char corrId[64] = {};  // optional, GOAL_DONE only
};

}  // namespace msg
