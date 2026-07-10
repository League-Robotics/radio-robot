#pragma once
#include "command_types.h"
#include "runtime/command_router.h"
#include <vector>

// ---------------------------------------------------------------------------
// system_commands.h -- liveness command family (077-001).
//
// 097-006 (architecture-update-r2.md Decision 9, Eric's 2026-07-10 redirect
// to a pure-binary firmware): DELETES VER/HELP/ECHO/ID unconditionally --
// only PING and HELLO survive as this sprint's confirmed liveness rump
// (the 3-verb default alongside motion_commands.h's STOP -- see that
// file's header comment). Each deleted verb's information content is a
// strict subset of what the binary plane already carries: `echo`/`id`
// (095, hardware-bench-smoke-tested) are direct binary parity for ECHO/ID;
// VER's `fw`/`proto` pair is a strict subset of binary `id`'s `DeviceId`
// reply; HELP enumerated the live text table, which no longer has anything
// interesting left to enumerate. HELLO is the one exception kept beyond
// Eric's stated 2-verb (STOP+PING) default -- see the paragraph below.
//
// Handler bodies were ported from source_old/commands/SystemCommands.cpp,
// with the old Robot*/RobotSysCtx handlerCtx coupling removed -- this
// firmware has no Robot class. PING/HELLO read only free vendor functions
// (Types::systemClockNow(), microbit_friendly_name(), microbit_serial_number())
// and the wire constants in types/protocol.h.
//
// 088-005: HELLO re-requests the firmware's own identity banner (removed
// under v1->v2, re-added here). formatDeviceAnnouncement() is the single
// place that knows the `DEVICE:NEZHA2:robot:<name>:<serial>` wire format
// and the #ifdef HOST_BUILD identity source deviceIdentity() uses -- both
// main.cpp's boot-time announcement and handleHello call it, so the two
// call sites can never format the banner differently (architecture-update.md
// Decision 3: a shared free function here, not a revived Announcer class).
// 097-006 keeps HELLO specifically because communicator.cpp's own boot-
// announcement comment documents that a missed boot banner is "not a
// failure -- HELLO re-requests it", and host/robot_radio/io/serial_conn.py's
// connect()/_banner_classify() sends HELLO repeatedly on RECONNECT for
// exactly that reason -- deleting it would remove the one re-request path
// that handshake structurally depends on, a materially different risk than
// the other four (diagnostic-only) liveness verbs. See
// architecture-update-r2.md's "Open decision: the text safety rump" for the
// full evidence; flagged there as pending Eric's confirmation, not silently
// assumed.
//
// 095-007: deviceIdentity() has external linkage (declared here, defined in
// system_commands.cpp outside its anonymous namespace) so BinaryChannel's
// binary `id` reply (source/commands/binary_channel.cpp) can source
// model/name/serial/fw/proto from the SAME identity pair formatDeviceAnnouncement()
// uses, instead of duplicating the #ifdef HOST_BUILD branch a second time
// (architecture-update.md Decision 4). Zero behavior change to any existing
// caller.
// ---------------------------------------------------------------------------

// Formats the `DEVICE:NEZHA2:robot:<name>:<serial>` identity banner into
// buf[0..size-1]. <name>/<serial> are exactly microbit_friendly_name()/
// microbit_serial_number() -- the same identity pair deviceIdentity()
// exposes below (HOST_BUILD substitutes the same fixed "HOST-SIM"/0
// placeholder host-side). Buffer-writing convention, matching
// CommandProcessor::listVerbs(): returns the length written (snprintf
// semantics), truncates silently if buf is too small. Called once by
// main.cpp at boot (both channels) and once per HELLO request.
int formatDeviceAnnouncement(char* buf, int size);

// The raw identity pair formatDeviceAnnouncement() formats from: *name/
// *serial are exactly microbit_friendly_name()/microbit_serial_number() on-
// target, or the fixed "HOST-SIM"/0 placeholder under HOST_BUILD. External
// linkage (095-007) so BinaryChannel's binary `id` handler can build a
// msg::DeviceId from the SAME source, never a second #ifdef HOST_BUILD
// branch.
void deviceIdentity(const char** name, uint32_t* serial);

// Returns the liveness command table: PING, HELLO.
std::vector<CommandDescriptor> systemCommands(Rt::CommandRouter& router);
