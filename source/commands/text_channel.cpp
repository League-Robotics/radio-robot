// text_channel.cpp -- HELP/HELLO/PING/ID/VER/STOP: the minimal hand-typeable
// text command channel (protocol v3's "text safety rump",
// docs/protocol-v3.md section 6). See text_channel.h for what this file is
// and is not: the pose/otos one-shot verbs and the DEV bench-diagnostic
// family that used to live here were deleted outright (stakeholder-
// directed cleanup) -- neither was ever registered by
// `Rt::CommandRouter::buildTable()`, so nothing on the wire changes. Their
// old handler bodies are in git history if a transcription reference is
// ever needed. HELP (stakeholder-directed, 2026-07-10) was re-added -- see
// text_channel.h's own header comment and handleHelp()'s doc comment below.
#include "commands/text_channel.h"

#include <cstdint>
#include <cstdio>

#include "commands/command_processor.h"
#include "messages/drivetrain.h"
#include "types/clock.h"

#ifndef HOST_BUILD
#include "MicroBit.h"   // microbit_friendly_name() / microbit_serial_number() (on-target identity)
#endif

// ---------------------------------------------------------------------------
// deviceIdentity -- the CODAL device-identity pair every identity-bearing
// reply (the DEVICE: banner, HELLO, ID) reports. #ifdef HOST_BUILD
// substitutes a fixed placeholder identity, so the sim build can format an
// identity reply with no hardware or sim board present. The single place
// this branch lives -- formatDeviceAnnouncement()/handleId() below and
// BinaryChannel's binary `id` handler all call it, so none of them ever
// drift from each other.
//
// External linkage (declared in text_channel.h): defined here, ahead of
// the anonymous namespace below, the same way formatDeviceAnnouncement()
// is defined below it -- both are this translation unit's two
// external-linkage identity entry points.
// ---------------------------------------------------------------------------
void deviceIdentity(const char** name, uint32_t* serial) {
#ifdef HOST_BUILD
  *name   = "HOST-SIM";
  *serial = 0;
#else
  *name   = microbit_friendly_name();
  *serial = microbit_serial_number();
#endif
}

namespace {

// ---------------------------------------------------------------------------
// handleStop -- posts a NEUTRAL msg::DrivetrainCommand straight to
// bb.driveIn, built inline WITHOUT the standby side-channel. Leaving
// standby unset keeps the drivetrain active, so the neutral reaches the
// motors (a prior standby=true shape was found to silently drop the
// neutral -- see docs/protocol-v3.md section 6 and this handler's own
// history for the full story). No EVT. Reply stays `OK stop`.
//
// PHYSICAL EFFECT: Subsystems::Drivetrain::dispatchEscapeHatch()
// (drivetrain.cpp) inspects whether a Motion::Segment is actively
// executing when a NEUTRAL arrives -- if so, it arms the owned
// Motion::SegmentExecutor's own presolved graceful decel-to-zero instead
// of zeroing the wheels instantly. Only when there is nothing in-flight to
// decelerate does STOP fall straight through to instant-neutral behavior.
// This handler itself needs no code change for that behavior switch --
// entirely a Drivetrain-level decision on the same NEUTRAL command shape
// this handler already builds.
// ---------------------------------------------------------------------------
void handleStop(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
                void* handlerCtx) {
  Rt::Blackboard& b = static_cast<Rt::CommandRouter*>(handlerCtx)->blackboard();

  msg::DrivetrainCommand cmd;
  cmd.setNeutral(msg::Neutral::BRAKE);
  b.driveIn.post(cmd);

  char rbuf[32];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stop", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// PING -- clock-sync probe.
//   prefix "PING"; parseFn nullptr.
//   Reply: OK pong t=<ms>
// ---------------------------------------------------------------------------
void handlePing(const ArgList& /*args*/, const char* corrId,
                ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/) {
  uint32_t now = Types::systemClockNow();  // [ms]
  char rbuf[64];
  char body[32];
  snprintf(body, sizeof(body), "t=%lu", (unsigned long)now);
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "pong", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// HELLO -- re-request the firmware's own identity banner.
//   prefix "HELLO"; parseFn nullptr.
//   Reply: DEVICE:NEZHA2:robot:<name>:<serial> -- a bare reply, like ID's
//   own taxonomy: no OK/ERR wrapper, DEVICE: is its own reply taxonomy.
//
// Re-emits on whichever channel HELLO arrived on: replyFn/replyCtx are
// already resolved to that channel by the time a handler runs, so no
// channel selection logic belongs here. The banner is byte-identical to
// main.cpp's boot-time announcement -- both call formatDeviceAnnouncement()
// -- so no corrId is echoed back into it.
// ---------------------------------------------------------------------------
void handleHello(const ArgList& /*args*/, const char* /*corrId*/,
                  ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/) {
  char banner[64];
  formatDeviceAnnouncement(banner, sizeof(banner));
  replyFn(banner, replyCtx);
}

// ---------------------------------------------------------------------------
// ID -- device identification.
//   prefix "ID"; parseFn nullptr.
//   Reply: ID model=<m> name=<n> serial=<s> fw=<ver> proto=<n> [#id] -- a
//   bare reply (no OK/ERR wrapper), the same taxonomy HELLO's DEVICE: line
//   uses, and the same fields the binary `id` arm's DeviceId reply carries
//   (docs/protocol-v3.md section 3).
//
// No caps= field: the original's caps list (otos/line/color) was read off
// a Robot type that does not exist in this tree.
// ---------------------------------------------------------------------------
void handleId(const ArgList& /*args*/, const char* corrId,
              ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/) {
  const char* name;
  uint32_t    serial;
  deviceIdentity(&name, &serial);

  char rbuf[160];
  if (corrId && corrId[0] != '\0') {
    snprintf(rbuf, sizeof(rbuf),
             "ID model=NEZHA2 name=%s serial=%lu fw=%s proto=%d #%s",
             name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION, corrId);
  } else {
    snprintf(rbuf, sizeof(rbuf),
             "ID model=NEZHA2 name=%s serial=%lu fw=%s proto=%d",
             name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION);
  }
  replyFn(rbuf, replyCtx);
}

// ---------------------------------------------------------------------------
// VER -- firmware/protocol version query.
//   prefix "VER"; parseFn nullptr.
//   Reply: OK ver fw=<ver> proto=<n>
// ---------------------------------------------------------------------------
void handleVer(const ArgList& /*args*/, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/) {
  char rbuf[64];
  char body[64];
  snprintf(body, sizeof(body), "fw=%s proto=%d", FIRMWARE_VERSION, PROTO_VERSION);
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "ver", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// HELP -- list every registered verb.
//   prefix "HELP"; parseFn nullptr.
//   Reply: OK help <space-separated verbs> [#id]
//
// Re-added (stakeholder-directed, 2026-07-10) from its pre-18ba84d8
// implementation (`git show 18ba84d8^:source/commands/system_commands.cpp`),
// unchanged: reads the LIVE registered command table via
// `CommandRouter::listVerbs()` (handlerCtx cast to Rt::CommandRouter*, the
// same pattern handleStop() above already uses to reach shared runtime
// state) -- never a hardcoded string, so the reply always matches whatever
// textCommands() below actually registers. The binary `help` arm
// (source/commands/binary_channel.cpp) reads the SAME listVerbs() text into
// its `HelpText.text` reply field, so the two planes never drift apart.
// ---------------------------------------------------------------------------
void handleHelp(const ArgList& /*args*/, const char* corrId,
                ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
  auto* router = static_cast<Rt::CommandRouter*>(handlerCtx);
  char verbs[64];
  router->listVerbs(verbs, sizeof(verbs));

  char rbuf[96];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "help", verbs,
                            corrId, replyFn, replyCtx);
}

}  // namespace

// ---------------------------------------------------------------------------
// formatDeviceAnnouncement -- see text_channel.h's doc comment. Defined
// here (external linkage, per the header declaration) rather than inside
// the anonymous namespace above, and reuses deviceIdentity() -- itself
// external linkage too, defined just above this file's own anonymous
// namespace.
// ---------------------------------------------------------------------------
int formatDeviceAnnouncement(char* buf, int size) {
  const char* name;
  uint32_t    serial;
  deviceIdentity(&name, &serial);
  return snprintf(buf, size, "DEVICE:NEZHA2:robot:%s:%lu",
                   name, (unsigned long)serial);
}

// ---------------------------------------------------------------------------
// textCommands -- the text safety rump's registered command table: HELP,
// HELLO, PING, ID, VER, STOP. The only table Rt::CommandRouter::buildTable()
// (command_router.cpp) assembles.
// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> textCommands(Rt::CommandRouter& router) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeCmd("HELP",  nullptr, handleHelp,  &router, "badarg"));
  cmds.push_back(makeCmd("HELLO", nullptr, handleHello, nullptr, "badarg"));
  cmds.push_back(makeCmd("PING",  nullptr, handlePing,  nullptr, "badarg"));
  cmds.push_back(makeCmd("ID",   nullptr, handleId,    nullptr, "badarg"));
  cmds.push_back(makeCmd("VER",  nullptr, handleVer,   nullptr, "badarg"));
  cmds.push_back(makeCmd("STOP", nullptr, handleStop, &router, "badarg", ForceReply::NONE,
                         CMD_ACCESS_HARDWARE));
  return cmds;
}
