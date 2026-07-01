#pragma once
#include <vector>
#include "CommandTypes.h"

// ---------------------------------------------------------------------------
// MotionCommands — app-layer motion command parsing, conversion, and
// reply formatting.
//
// Dependency direction: app/ → control/.
// All CommandProcessor::replyOK/Err/Evt calls live here, not in control/.
// ---------------------------------------------------------------------------

class Planner;
class CommandQueue;
class Superstructure;
struct Robot;

// ---------------------------------------------------------------------------
// MotionCtx — context bundle used by Commandable-registered motion handlers.
// ---------------------------------------------------------------------------
struct MotionCtx {
    Planner*           mc;
    Superstructure*    superstructure;  // Seam 3 (042-001): queue-path begin* goes through requestGoal
    struct Robot*      robot;
    CommandQueue*      queue;    // command queue for VW converter push_front; may be null
    CommandDescriptor  vwDesc;   // stable VW descriptor used by converters to build ParsedCommand
};

// ---------------------------------------------------------------------------
// getMotionCommands — returns the full set of motion CommandDescriptors
// (S, T, D, G, R, TURN, RT, VW, _VW, X, STOP) bound to ctx.
//
// Called by Robot::buildCommandTable().
// ctx must remain live for the lifetime of the returned descriptors.
// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> getMotionCommands(MotionCtx* ctx);
