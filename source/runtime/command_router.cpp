// command_router.cpp -- see command_router.h for the class-level contract.
#include "runtime/command_router.h"

#include "commands/system_commands.h"

#if ROBOT_DEV_BUILD
#include "commands/config_commands.h"
#include "commands/dev_commands.h"
#include "commands/motion_commands.h"
#include "commands/otos_commands.h"
#include "commands/pose_commands.h"
#include "commands/telemetry_commands.h"
#endif  // ROBOT_DEV_BUILD

namespace Rt {

namespace {

// buildTable -- assembles the full command table (liveness + the six
// pointerless command families, ROBOT_DEV_BUILD permitting) bound to
// `router`, mirroring main.cpp's/sim_api.cpp's own pre-087
// systemCommands()+devCommands()+...+otosCommands() assembly exactly (same
// family order, same table-concatenation shape).
std::vector<CommandDescriptor> buildTable(CommandRouter& router) {
  std::vector<CommandDescriptor> all = systemCommands(router);
#if ROBOT_DEV_BUILD
  std::vector<CommandDescriptor> dev = devCommands(router);
  all.insert(all.end(), dev.begin(), dev.end());
  std::vector<CommandDescriptor> telemetry = telemetryCommands(router);
  all.insert(all.end(), telemetry.begin(), telemetry.end());
  std::vector<CommandDescriptor> motion = motionCommands(router);
  all.insert(all.end(), motion.begin(), motion.end());
  std::vector<CommandDescriptor> config = configCommands(router);
  all.insert(all.end(), config.begin(), config.end());
  std::vector<CommandDescriptor> pose = poseCommands(router);
  all.insert(all.end(), pose.begin(), pose.end());
  std::vector<CommandDescriptor> otos = otosCommands(router);
  all.insert(all.end(), otos.begin(), otos.end());
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
