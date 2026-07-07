// command_router.h -- Rt::CommandRouter: sprint 087's command-tier
// translator (architecture-update-r1.md Step 3, ticket 006). Parses one wire
// statement (via the existing CommandProcessor/CommandDescriptor table
// machinery, source/commands/command_processor.*) and dispatches it to a
// pointerless per-family handler that reads/writes ONLY Rt::Blackboard --
// never a Subsystems::* pointer (SUC-006's acceptance criterion).
//
// Construction is decoupled from any particular Rt::Blackboard instance
// (matching architecture-update-r1.md's Reference code, `CommandRouter
// router;` declared before `Rt::Blackboard bb;`): the full descriptor table
// (liveness + the six command families) is built once, at construction,
// with every descriptor's handlerCtx set to `this` -- never `&bb` (bb does
// not exist yet at that point). route(statement, bb) stashes the CALLER's
// bb reference in bb_ for the duration of that one dispatch; every family's
// HandlerFn casts handlerCtx back to CommandRouter* and calls blackboard()
// to reach it. Since exactly one Rt::Blackboard exists for a program's
// entire lifetime (constructed once by the loop), this is equivalent to
// binding handlerCtx to &bb directly, without requiring bb to already exist
// when CommandRouter itself is constructed.
//
// Reply-channel resolution (Decision 10): `statement.returnPath` is a
// Subsystems::Channel (SERIAL/RADIO), not a ReplyFn/void* pair -- the caller
// (main.cpp/sim_api.cpp) wires the two concrete reply sinks once via
// setReplyChannels(), mirroring CommandProcessor::setSerialReply()'s own
// existing pattern (a generic ReplyFn/void* opaque-callback pair, not a
// typed Subsystems::Communicator* -- see command_processor.h). route()
// resolves which pair to use from statement.returnPath every call.
#pragma once

#include "commands/command_processor.h"
#include "runtime/blackboard.h"
#include "subsystems/statement.h"

namespace Rt {

class CommandRouter {
 public:
  CommandRouter();

  // Wires the two physical reply sinks (serial, radio) route() resolves
  // statement.returnPath against. Must be called before the first route()
  // (mirrors every command family's own "state must be wired before first
  // use" contract, e.g. dev_commands.h's devCommands()).
  void setReplyChannels(ReplyFn serialReply, void* serialCtx, ReplyFn radioReply, void* radioCtx);

  // Parse and dispatch one statement against `bb`. Resolves the reply sink
  // from statement.returnPath (see setReplyChannels()), tokenizes/dispatches
  // via the existing CommandProcessor machinery, and lets the matched
  // family's translator read/post against `bb`.
  void route(const Subsystems::CommunicatorToCommandProcessorStatement& statement, Blackboard& bb);

  // Accessor the six command-family translators use to reach the
  // currently-routed Blackboard from their HandlerFn's handlerCtx (cast to
  // CommandRouter*) -- see the class comment above. Only valid to call from
  // within a translator invoked by route() (i.e. during dispatch).
  Blackboard& blackboard() { return *bb_; }

  // The Channel the statement CURRENTLY being dispatched by route() arrived
  // on -- STREAM's handler (telemetry_commands.cpp) reads this to bind the
  // periodic-emission channel (bb.telemetryChannel), replacing the old raw
  // ReplyFn/void* capture (a function pointer is not itself a
  // Blackboard-appropriate payload -- see blackboard.h's own note on
  // telemetryChannel). Only meaningful during a route() call.
  Subsystems::Channel currentChannel() const { return currentChannel_; }

  // Forwards to CommandProcessor::listVerbs() -- the live registered verb
  // table HELP's handler reads (088-003, Decision 2: reuses this class's
  // existing "reach shared runtime state from handlerCtx" pattern instead
  // of a second, separately-maintained verb list). Keeps processor_
  // private; this accessor is the only read path onto it beyond route().
  int listVerbs(char* buf, int size) const { return processor_.listVerbs(buf, size); }

 private:
  CommandProcessor processor_;
  Blackboard* bb_ = nullptr;
  Subsystems::Channel currentChannel_ = Subsystems::Channel::NONE;

  ReplyFn serialReply_ = nullptr;
  void* serialCtx_ = nullptr;
  ReplyFn radioReply_ = nullptr;
  void* radioCtx_ = nullptr;
};

}  // namespace Rt
