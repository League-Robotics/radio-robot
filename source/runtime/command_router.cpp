// command_router.cpp -- see command_router.h for the class-level contract.
#include "runtime/command_router.h"

#include "commands/system_commands.h"

#include "commands/motion_commands.h"

namespace Rt {

namespace {

// buildTable -- sprint 093's minimal command table: liveness
// (systemCommands(): PING/HELLO) + the motion family (motionCommands():
// S/STOP, plus 094-006's MOVE/TLM -- see motion_commands.cpp's own trimmed
// registration for the full, current list). The `dev`/`telemetry`/
// `config`/`pose`/`otos` families are left un-wired here -- their files,
// handlers, and includes are untouched on disk (clasi/sprints/093-.../
// architecture-update.md Step 5/Migration Concerns); buildTable() simply
// stops calling them.
std::vector<CommandDescriptor> buildTable(CommandRouter& router) {
  std::vector<CommandDescriptor> all = systemCommands(router);
  std::vector<CommandDescriptor> motion = motionCommands(router);
  all.insert(all.end(), motion.begin(), motion.end());
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
