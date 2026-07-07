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
// ---------------------------------------------------------------------------

// Returns the liveness command table: PING, VER, HELP, ECHO, ID. HELP's
// descriptor is bound to `router` so it can enumerate the live table
// (Rt::CommandRouter::listVerbs()); every other descriptor's handlerCtx
// stays nullptr, unchanged.
std::vector<CommandDescriptor> systemCommands(Rt::CommandRouter& router);
