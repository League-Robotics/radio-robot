// ---------------------------------------------------------------------------
// system_commands.cpp -- liveness command handlers: PING, VER, HELP, ECHO, ID.
//
// New file (077-001). Handler bodies ported from
// source_old/commands/SystemCommands.cpp, with the old Robot*/RobotSysCtx
// coupling removed -- this firmware has no Robot class. Device identity
// (microbit_friendly_name() / microbit_serial_number()) is a free CODAL
// vendor function, so none of the five handlers below need a handlerCtx.
//
// 081-002: the clock read moved behind Types::systemClockNow()
// (types/clock.h) -- this was the one remaining direct
// system_timer_current_time() call in the host-clean command set. handleId's
// identity calls are gated #ifdef HOST_BUILD (fixed host identity strings)
// so this file can compile host-side ahead of a future sim build; the
// on-target branch is unchanged.
// ---------------------------------------------------------------------------

#include "system_commands.h"
#include "command_processor.h"
#include "types/clock.h"

#ifndef HOST_BUILD
#include "MicroBit.h"   // microbit_friendly_name() / microbit_serial_number() (on-target identity)
#endif

#include <cstdio>

namespace {

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
// HELP -- list all verbs.
//   prefix "HELP"; parseFn nullptr.
//   Reply: OK help <verb list>
// ---------------------------------------------------------------------------
void handleHelp(const ArgList& /*args*/, const char* corrId,
                 ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/) {
  char rbuf[128];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "help", "PING VER HELP ECHO ID",
                             corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// ECHO -- echo payload tokens back.
//   prefix "ECHO"; variadic ArgSchema; stores tokens as STR args.
//   Reply: OK echo <joined tokens>
// ---------------------------------------------------------------------------
const ArgSchema kEchoSchema = { nullptr, 0, 0, true, nullptr };

void handleEcho(const ArgList& args, const char* corrId,
                ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/) {
  // Reassemble tokens into a single space-joined payload.
  char payload[512];
  int pos = 0;
  for (int i = 0; i < args.count && pos < (int)sizeof(payload) - 2; ++i) {
    if (i > 0) payload[pos++] = ' ';
    for (const char* c = args.args[i].sval;
         *c != '\0' && pos < (int)sizeof(payload) - 1; ++c)
      payload[pos++] = *c;
  }
  payload[pos] = '\0';

  char rbuf[520];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "echo", payload, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// ID -- device identification.
//   prefix "ID"; parseFn nullptr.
//   Reply: ID model=<m> name=<n> serial=<s> fw=<ver> proto=<n>
//
// No caps= field this ticket: the original's caps list (otos/line/color) was
// read off a Robot that does not exist here. A later ticket's HAL/Drivetrain
// tier can add a caps= token back once there is something to report.
// ---------------------------------------------------------------------------
void handleId(const ArgList& /*args*/, const char* corrId,
              ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/) {
#ifdef HOST_BUILD
  // No CODAL identity to read host-side -- fixed placeholder identity (see
  // this file's header note and architecture-update.md's Impact table).
  const char* name   = "HOST-SIM";
  uint32_t    serial = 0;
#else
  const char* name   = microbit_friendly_name();
  uint32_t    serial = microbit_serial_number();
#endif

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

}  // namespace

std::vector<CommandDescriptor> systemCommands() {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeCmd("PING", nullptr, handlePing, nullptr, "badarg"));
  cmds.push_back(makeCmd("VER",  nullptr, handleVer,  nullptr, "badarg"));
  cmds.push_back(makeCmd("HELP", nullptr, handleHelp, nullptr, "badarg"));
  cmds.push_back(makeSchemaCmd("ECHO", &kEchoSchema, handleEcho, nullptr, "badarg"));
  cmds.push_back(makeCmd("ID",   nullptr, handleId,   nullptr, "badarg"));
  return cmds;
}
