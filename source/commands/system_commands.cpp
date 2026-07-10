// ---------------------------------------------------------------------------
// system_commands.cpp -- liveness command handlers: PING, VER, HELP, ECHO,
// ID, HELLO.
//
// New file (077-001). Handler bodies ported from
// source_old/commands/SystemCommands.cpp, with the old Robot*/RobotSysCtx
// coupling removed -- this firmware has no Robot class. Device identity
// (microbit_friendly_name() / microbit_serial_number()) is a free CODAL
// vendor function, so none of the six handlers below need a handlerCtx.
//
// 081-002: the clock read moved behind Types::systemClockNow()
// (types/clock.h) -- this was the one remaining direct
// system_timer_current_time() call in the host-clean command set. handleId's
// identity calls are gated #ifdef HOST_BUILD (fixed host identity strings)
// so this file can compile host-side ahead of a future sim build; the
// on-target branch is unchanged.
//
// 088-005: re-adds HELLO and the `DEVICE:NEZHA2:robot:<name>:<serial>`
// identity banner (removed under v1->v2, host still parses/caches it --
// see clasi/issues/robot-device-announcement-on-connect-and-hello.md).
// handleId and the new formatDeviceAnnouncement() share one identity-fetch
// helper (deviceIdentity(), below) instead of duplicating the #ifdef
// HOST_BUILD branch a second time.
// ---------------------------------------------------------------------------

#include "system_commands.h"
#include "command_processor.h"
#include "types/clock.h"

#ifndef HOST_BUILD
#include "MicroBit.h"   // microbit_friendly_name() / microbit_serial_number() (on-target identity)
#endif

#include <cstdio>

// ---------------------------------------------------------------------------
// deviceIdentity -- the CODAL device-identity pair every identity-bearing
// reply (ID, the DEVICE: banner, HELLO) reports. #ifdef HOST_BUILD
// substitutes the same fixed placeholder identity handleId has always used
// (name/serial below), so the sim build can format an identity reply with
// no hardware or sim board present. The single place this branch lives --
// handleId and formatDeviceAnnouncement() both call it, so the two never
// drift apart.
//
// External linkage (095-007, declared in system_commands.h): moved out of
// the anonymous namespace below so BinaryChannel's binary `id` handler
// (source/commands/binary_channel.cpp) can reuse it too -- see the header
// declaration's own doc comment. Defined here, ahead of the anonymous
// namespace, the same way formatDeviceAnnouncement() is defined below it --
// both are this translation unit's two external-linkage identity entry
// points.
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
// HELP -- list every registered verb.
//   prefix "HELP"; parseFn nullptr.
//   Reply: OK help <space-separated verbs> [#id]
//
// Reads the LIVE registered command table via CommandRouter::listVerbs()
// (handlerCtx cast to Rt::CommandRouter*, the same pattern every other
// command family already uses to reach shared runtime state -- see
// dev_commands.cpp's own `bb(handlerCtx)` helper) -- not a hardcoded
// string. HELP is the only liveness handler with a non-null handlerCtx
// (see systemCommands() below); adding or removing a command family
// changes this reply with no edit here (088-003, Decision 2).
// ---------------------------------------------------------------------------
void handleHelp(const ArgList& /*args*/, const char* corrId,
                 ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
  auto* router = static_cast<Rt::CommandRouter*>(handlerCtx);
  char verbs[256];
  router->listVerbs(verbs, sizeof(verbs));

  char rbuf[320];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "help", verbs,
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
// HELLO -- re-request the firmware's own identity banner.
//   prefix "HELLO"; parseFn nullptr.
//   Reply: DEVICE:NEZHA2:robot:<name>:<serial> -- a bare reply, like ID's
//   own "ID ..." tag: no OK/ERR wrapper, DEVICE: is its own reply taxonomy.
//
// Re-emits on whichever channel HELLO arrived on: replyFn/replyCtx are
// already resolved to that channel by the time a handler runs (the same
// mechanism every other liveness handler's reply already uses), so no
// channel selection logic belongs here. The banner is byte-identical to
// main.cpp's boot-time announcement -- both call formatDeviceAnnouncement()
// (088-005, architecture-update.md Decision 3) -- so no corrId is echoed
// back into it.
// ---------------------------------------------------------------------------
void handleHello(const ArgList& /*args*/, const char* /*corrId*/,
                  ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/) {
  char banner[64];
  formatDeviceAnnouncement(banner, sizeof(banner));
  replyFn(banner, replyCtx);
}

}  // namespace

// ---------------------------------------------------------------------------
// formatDeviceAnnouncement -- see system_commands.h's doc comment. Defined
// here (external linkage, per the header declaration) rather than inside
// the anonymous namespace above, but reuses deviceIdentity() -- an
// anonymous-namespace helper with internal linkage, visible for the rest of
// this translation unit.
// ---------------------------------------------------------------------------
int formatDeviceAnnouncement(char* buf, int size) {
  const char* name;
  uint32_t    serial;
  deviceIdentity(&name, &serial);
  return snprintf(buf, size, "DEVICE:NEZHA2:robot:%s:%lu",
                   name, (unsigned long)serial);
}

std::vector<CommandDescriptor> systemCommands(Rt::CommandRouter& router) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeCmd("PING",  nullptr, handlePing,  nullptr, "badarg"));
  cmds.push_back(makeCmd("VER",   nullptr, handleVer,   nullptr, "badarg"));
  cmds.push_back(makeCmd("HELP",  nullptr, handleHelp,  &router, "badarg"));
  cmds.push_back(makeSchemaCmd("ECHO", &kEchoSchema, handleEcho, nullptr, "badarg"));
  cmds.push_back(makeCmd("ID",    nullptr, handleId,    nullptr, "badarg"));
  cmds.push_back(makeCmd("HELLO", nullptr, handleHello, nullptr, "badarg"));
  return cmds;
}
