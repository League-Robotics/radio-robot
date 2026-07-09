// command_router.cpp -- see command_router.h for the class-level contract.
#include "runtime/command_router.h"

#include "commands/system_commands.h"

#if ROBOT_DEV_BUILD
#include "commands/motion_commands.h"
#endif  // ROBOT_DEV_BUILD

namespace Rt {

namespace {

// buildTable -- sprint 093's minimal command table: liveness
// (systemCommands(): PING/HELLO) + the four-verb motion family
// (motionCommands(): S/STOP -- see motion_commands.cpp's own trimmed
// registration). The `dev`/`telemetry`/`config`/`pose`/`otos` families are
// left un-wired here -- their files, handlers, and includes are untouched on
// disk (clasi/sprints/093-.../architecture-update.md Step 5/Migration
// Concerns); buildTable() simply stops calling them.
std::vector<CommandDescriptor> buildTable(CommandRouter& router) {
  std::vector<CommandDescriptor> all = systemCommands(router);
#if ROBOT_DEV_BUILD
  std::vector<CommandDescriptor> motion = motionCommands(router);
  all.insert(all.end(), motion.begin(), motion.end());
#endif  // ROBOT_DEV_BUILD
  return all;
}

}  // namespace

CommandRouter::CommandRouter() : processor_(buildTable(*this)) {}

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
