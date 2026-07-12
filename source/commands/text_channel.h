#pragma once

// ---------------------------------------------------------------------------
// text_channel.h -- the minimal hand-typeable text command channel:
// HELP, HELLO, PING, ID, VER, STOP. This is protocol v3's deliberately tiny
// "text safety rump" (docs/protocol-v3.md section 6) -- a human with a bare
// serial terminal (screen, minicom), no host program, no protobuf tooling,
// and no base64 encoder can always identify and halt the robot.
//
// HELP (stakeholder-directed, 2026-07-10): re-added -- it was deleted along
// with the rest of `system_commands.cpp` at commit 18ba84d8 and is restored
// here from that commit's own `handleHelp()` (`git show 18ba84d8^:source/
// commands/system_commands.cpp`), ported to this file's handlerCtx idiom
// exactly the way `handleStop()` already reaches `Rt::CommandRouter&` below.
// Every one of these six verbs also has a binary `CommandEnvelope` arm now
// (source/commands/binary_channel.cpp) -- HELP/HELLO/VER newly so; PING/
// STOP/ID already did.
//
// Everything else that used to live in this file -- the pose/otos one-shot
// verbs (SI/ZERO/OI/OZ/OR/OP/OV/OL/OA) and the ROBOT_DEV_BUILD-gated DEV
// bench-diagnostic family -- was DELETED outright (stakeholder-directed
// cleanup), not parked. Neither was ever registered by
// `Rt::CommandRouter::buildTable()` while it existed here, so nothing on
// the wire changes: pose/otos become BINARY `CommandEnvelope` arms in
// sprint 098 (envelope.proto fields 7/8, docs/protocol-v3.md section 3/8);
// DEV has no binary counterpart planned and is simply gone from the text
// plane. If a transcription reference is ever needed, the old handler
// bodies are in git history (this file's own history prior to this
// cleanup commit).
//
// `formatDeviceAnnouncement()`/`deviceIdentity()` are external-linkage
// identity helpers shared with the binary plane -- `binary_channel.cpp`'s
// `id` arm calls `deviceIdentity()` directly, and `communicator.cpp`'s
// boot-time announcement calls `formatDeviceAnnouncement()`; this file's
// own `ID`/`HELLO`/`VER` handlers source their identity fields from the
// same pair. Do not change either signature without updating both callers.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "runtime/command_router.h"

// Formats the `DEVICE:NEZHA2:robot:<name>:<serial>` identity banner into
// buf[0..size-1]. <name>/<serial> are exactly microbit_friendly_name()/
// microbit_serial_number() -- the same identity pair deviceIdentity()
// exposes below (HOST_BUILD substitutes the same fixed "HOST-SIM"/0
// placeholder host-side). Buffer-writing convention, matching
// CommandProcessor::listVerbs(): returns the length written (snprintf
// semantics), truncates silently if buf is too small. Called once by
// main.cpp at boot (both channels), once by communicator.cpp's own
// boot-time announcement, and once per HELLO request.
int formatDeviceAnnouncement(char* buf, int size);

// The raw identity pair formatDeviceAnnouncement() formats from: *name/
// *serial are exactly microbit_friendly_name()/microbit_serial_number() on-
// target, or the fixed "HOST-SIM"/0 placeholder under HOST_BUILD. External
// linkage so BinaryChannel's binary `id` handler (source/commands/
// binary_channel.cpp) and this file's own `ID` handler can build their
// replies from the SAME source, never a second #ifdef HOST_BUILD branch.
void deviceIdentity(const char** name, uint32_t* serial);

// Returns the text safety rump's command table: HELP, HELLO, PING, ID,
// VER, STOP -- the only six verbs `Rt::CommandRouter::buildTable()`
// (command_router.cpp) registers.
std::vector<CommandDescriptor> textCommands(Rt::CommandRouter& router);
