// ---------------------------------------------------------------------------
// system_commands.cpp -- liveness command handlers: PING, HELLO.
//
// New file (077-001). 097-006 (architecture-update-r2.md Decision 9) deleted
// VER/HELP/ECHO/ID unconditionally -- see system_commands.h's own header
// comment for the rationale and each deleted verb's binary-plane parity
// pointer. PING/HELLO's own handler bodies are byte-for-byte unchanged by
// this ticket.
//
// Handler bodies ported from source_old/commands/SystemCommands.cpp, with
// the old Robot*/RobotSysCtx coupling removed -- this firmware has no Robot
// class. Device identity (microbit_friendly_name() / microbit_serial_number())
// is a free CODAL vendor function, so neither handler below needs a
// handlerCtx.
//
// 081-002: the clock read moved behind Types::systemClockNow()
// (types/clock.h) -- this was the one remaining direct
// system_timer_current_time() call in the host-clean command set. deviceIdentity()'s
// identity calls are gated #ifdef HOST_BUILD (fixed host identity strings)
// so this file can compile host-side ahead of a future sim build; the
// on-target branch is unchanged.
//
// 088-005: re-adds HELLO and the `DEVICE:NEZHA2:robot:<name>:<serial>`
// identity banner (removed under v1->v2, host still parses/caches it --
// see clasi/issues/robot-device-announcement-on-connect-and-hello.md).
// formatDeviceAnnouncement() is the single identity-fetch helper both
// handleHello and BinaryChannel's binary `id` reply share (deviceIdentity(),
// below), instead of duplicating the #ifdef HOST_BUILD branch a second time.
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
// reply (the DEVICE: banner, HELLO) reports. #ifdef HOST_BUILD substitutes
// the same fixed placeholder identity this file has always used (name/
// serial below), so the sim build can format an identity reply with no
// hardware or sim board present. The single place this branch lives --
// formatDeviceAnnouncement() below and BinaryChannel's binary `id` handler
// both call it, so neither ever drifts from the other.
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
// HELLO -- re-request the firmware's own identity banner.
//   prefix "HELLO"; parseFn nullptr.
//   Reply: DEVICE:NEZHA2:robot:<name>:<serial> -- a bare reply, like the
//   binary `id` reply's own taxonomy: no OK/ERR wrapper, DEVICE: is its own
//   reply taxonomy.
//
// Re-emits on whichever channel HELLO arrived on: replyFn/replyCtx are
// already resolved to that channel by the time a handler runs (the same
// mechanism PING's own reply already uses), so no channel selection logic
// belongs here. The banner is byte-identical to main.cpp's boot-time
// announcement -- both call formatDeviceAnnouncement() (088-005,
// architecture-update.md Decision 3) -- so no corrId is echoed back into
// it.
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
// the anonymous namespace above, and reuses deviceIdentity() -- itself
// external linkage too (095-007, declared in system_commands.h), defined
// just above this file's own anonymous namespace.
// ---------------------------------------------------------------------------
int formatDeviceAnnouncement(char* buf, int size) {
  const char* name;
  uint32_t    serial;
  deviceIdentity(&name, &serial);
  return snprintf(buf, size, "DEVICE:NEZHA2:robot:%s:%lu",
                   name, (unsigned long)serial);
}

std::vector<CommandDescriptor> systemCommands(Rt::CommandRouter& router) {
  (void)router;   // no descriptor in this trimmed table binds handlerCtx = &router any more
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeCmd("PING",  nullptr, handlePing,  nullptr, "badarg"));
  cmds.push_back(makeCmd("HELLO", nullptr, handleHello, nullptr, "badarg"));
  return cmds;
}
