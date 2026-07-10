// command_router.cpp -- see command_router.h for the class-level contract.
#include "runtime/command_router.h"

#include "commands/system_commands.h"

#include "commands/motion_commands.h"
#include "commands/telemetry_commands.h"

namespace Rt {

namespace {

// buildTable -- sprint 093's minimal command table, extended by 096-002
// (architecture-update.md Decision 1): liveness (systemCommands():
// PING/HELLO) + the motion family (motionCommands(): S/STOP, plus 094-006's
// MOVE/TLM -- see motion_commands.cpp's own trimmed registration for the
// full, current list) + the telemetry family (telemetryCommands():
// STREAM/SNAP, restored now that the loop-owned periodic-emission tick
// exists -- see telemetry_commands.h's tickTelemetry()). The text SET/GET
// config family (formerly `configCommands()`, source/commands/
// config_commands.{h,cpp}) is not registered here -- and, as of 097-007
// (architecture-update-r2.md Decision 9, pure-binary firmware), no longer
// exists as source at all: its file was deleted outright, not merely left
// unregistered as 093/096 left it. Binary `config`/`get` (BinaryChannel's
// CONFIG/GET oneof arms, ticket 004) is the only live config-plane path,
// same as the still-unregistered `dev`/`pose`/`otos` text families.
std::vector<CommandDescriptor> buildTable(CommandRouter& router) {
  std::vector<CommandDescriptor> all = systemCommands(router);
  std::vector<CommandDescriptor> motion = motionCommands(router);
  all.insert(all.end(), motion.begin(), motion.end());
  std::vector<CommandDescriptor> telemetry = telemetryCommands(router);
  all.insert(all.end(), telemetry.begin(), telemetry.end());
  return all;
}

}  // namespace

CommandRouter::CommandRouter() : processor_(buildTable(*this)) {
  // 095-007 (M6 Dispatcher Integration, architecture-update.md Decision 1):
  // wires BinaryChannel's path to the Blackboard the SAME way every
  // motionCommands(router)/systemCommands(router) call already threads
  // `&router` through as handlerCtx -- CommandProcessor forwards this
  // opaque pointer verbatim to BinaryChannel::handle(), never
  // dereferencing it itself.
  processor_.setBinaryContext(this);
}

void CommandRouter::setReplyChannels(ReplyFn serialReply, void* serialCtx, ReplyFn radioReply,
                                     void* radioCtx) {
  serialReply_ = serialReply;
  serialCtx_ = serialCtx;
  radioReply_ = radioReply;
  radioCtx_ = radioCtx;
  processor_.setSerialReply(serialReply, serialCtx);
}

void CommandRouter::route(const Subsystems::CommunicatorToCommandProcessorCommand& command,
                          Blackboard& bb) {
  bb_ = &bb;
  currentChannel_ = command.returnPath;
  bool radio = command.returnPath == Subsystems::Channel::RADIO;
  ReplyFn replyFn = radio ? radioReply_ : serialReply_;
  void* replyCtx = radio ? radioCtx_ : serialCtx_;
  processor_.process(command.line, replyFn, replyCtx);
}

}  // namespace Rt
