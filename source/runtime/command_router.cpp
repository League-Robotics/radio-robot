// command_router.cpp -- see command_router.h for the class-level contract.
#include "runtime/command_router.h"

#include "commands/text_channel.h"

namespace Rt {

namespace {

// buildTable -- 097-011: folds the sprint 093 minimal command table's three
// separate per-family builders (systemCommands(): PING/HELLO;
// motionCommands(): STOP; telemetryCommands(): registered ZERO commands
// post-097-008) into a single call to text_channel.{h,cpp}'s own
// textCommands() -- see that file's own doc comment for the fold. Resulting
// registered table is identical: STOP, PING, HELLO (plus the binary
// channel, untouched). The text SET/GET config family (formerly
// `configCommands()`, source/commands/config_commands.{h,cpp}) is not
// registered here -- and, as of 097-007 (architecture-update-r2.md
// Decision 9, pure-binary firmware), no longer exists as source at all:
// its file was deleted outright, not merely left unregistered as 093/096
// left it. Binary `config`/`get` (BinaryChannel's CONFIG/GET oneof arms,
// ticket 004) is the only live config-plane path, same as the
// still-unregistered `dev`/`pose`/`otos` text families (text_channel.h
// Sections 2/3).
std::vector<CommandDescriptor> buildTable(CommandRouter& router) {
  return textCommands(router);
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
