#pragma once
#include "CommandTypes.h"
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
// and the wire constants in types/Protocol.h, so handlerCtx is unused
// (nullptr) for all five descriptors.
// ---------------------------------------------------------------------------

// Returns the liveness command table: PING, VER, HELP, ECHO, ID.
std::vector<CommandDescriptor> systemCommands();
