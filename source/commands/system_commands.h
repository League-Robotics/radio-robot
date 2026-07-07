#pragma once
#include "command_types.h"
#include "runtime/command_router.h"
#include <vector>

// ---------------------------------------------------------------------------
// system_commands.h -- liveness command family (077-001).
//
// PING / VER / HELP / ECHO / ID: the five verbs a bare dev-loop build needs so
// host tooling can detect a live firmware over serial or the radio relay,
// independent of whatever hardware tier (Motor / Drivetrain / DEV) lands on
// top of this comms+commands scaffold in later tickets.
//
// Handler bodies are ported from source_old/commands/SystemCommands.cpp, with
// the old Robot*/RobotSysCtx handlerCtx coupling removed -- this firmware has
// no Robot class. Every handler reads only free vendor functions
// (microbit_friendly_name(), microbit_serial_number(), system_timer_current_time())
// and the wire constants in types/protocol.h.
//
// 088-003: HELP now enumerates the LIVE registered command table (every
// family CommandRouter::buildTable() assembles), via
// Rt::CommandRouter::listVerbs(), instead of a hardcoded string --
// joining the other six command families' `(Rt::CommandRouter&)`
// construction signature (see dev_commands.h). Only HELP's descriptor binds
// handlerCtx = &router; PING/VER/ECHO/ID are unchanged and still read only
// free vendor functions, so their handlerCtx stays nullptr.
//
// 088-005: re-adds HELLO (removed under v1->v2) as a re-request for the
// firmware's own identity banner. formatDeviceAnnouncement() is the single
// place that knows the `DEVICE:NEZHA2:robot:<name>:<serial>` wire format
// and the #ifdef HOST_BUILD identity source handleId already uses -- both
// main.cpp's boot-time announcement and handleHello call it, so the two
// call sites can never format the banner differently (architecture-update.md
// Decision 3: a shared free function here, not a revived Announcer class).
// ---------------------------------------------------------------------------

// Formats the `DEVICE:NEZHA2:robot:<name>:<serial>` identity banner into
// buf[0..size-1]. <name>/<serial> are exactly microbit_friendly_name()/
// microbit_serial_number() -- the same identity pair handleId's ID reply
// uses (HOST_BUILD substitutes the same fixed "HOST-SIM"/0 placeholder
// handleId uses host-side). Buffer-writing convention, matching
// CommandProcessor::listVerbs(): returns the length written (snprintf
// semantics), truncates silently if buf is too small. Called once by
// main.cpp at boot (both channels) and once per HELLO request.
int formatDeviceAnnouncement(char* buf, int size);

// Returns the liveness command table: PING, VER, HELP, ECHO, ID, HELLO.
// HELP's descriptor is bound to `router` so it can enumerate the live table
// (Rt::CommandRouter::listVerbs()); every other descriptor's handlerCtx
// stays nullptr, unchanged.
std::vector<CommandDescriptor> systemCommands(Rt::CommandRouter& router);
