// BusDrain.h — command-queue bus drain and route (ticket 059-003).
//
// drainCommandBatch() is the mechanism that connects a subsystem tick()'s
// returned CommandBatch to the appropriate subsystem apply() calls.
//
// DESIGN: each OutCommand in the batch is dispatched by verb_id:
//
//   verb_id = kVerbDrivetrainTwist (1)
//       → reconstruct DrivetrainCommand{TWIST} from args_[0..2]
//       → call drive.apply(cmd)
//
//   verb_id = kVerbPlannerCommand (2)
//       → reconstruct PlannerCommand from args_ (not yet used; no-op)
//       → reserved for future planner re-injection
//
//   verb_id = anything else
//       → treated as a passthrough: synthesise a ParsedCommand with
//         desc=nullptr and enqueue it.  priority=true → push_front;
//         priority=false → push_back.  This lets ASCII-verb commands
//         round-trip through the bus without duplicating the
//         CommandProcessor verb table.
//
// BOUNDED CASCADE: drainCommandBatch() processes at most kBusDrainMaxIters
// commands per call, regardless of batch.cmds_count.  If the batch has more
// commands than the limit, the function returns the limit and discards the
// rest.  Callers that need EVT telemetry on overflow should check the
// return value against cmds_count.
//
// SAFETY PRIORITY: OutCommand.priority==true routes the command to the
// FRONT of the CommandQueue (push_front) rather than the back.  push_front
// failure (queue full) is handled by returning the count of commands
// successfully processed up to that point.  The caller is responsible for
// emitting EVT bus_overflow if needed; overflow of a safety command is
// treated as EVT, not assertion.
//
// ADDITIVE — drainCommandBatch() is NOT called from the live loopTickOnce
// until ticket 059-005 (the cutover).  It is exercised by host tests
// (test_059_bus_drain.py) via the C-ABI shim bus_drain_api.cpp.
//
// Constraints: C++11, no heap/STL/RTTI/exceptions.  RETURN model.
// Reuses msg:: types, CommandQueue, CommandProcessor, Drive.
#pragma once

#include "messages/common.h"        // msg::CommandBatch, msg::OutCommand
#include "messages/verb_ids.h"      // msg::kVerbDrivetrainTwist, kVerbPlannerCommand
#include <stdint.h>

// Forward declarations — resolved at link time.
namespace subsystems { class Drive; }
class Planner;
class CommandQueue;
class CommandProcessor;

// Maximum number of OutCommands to route per drainCommandBatch() call.
// Prevents unbounded run-to-completion loops inside a single tick.
static constexpr uint8_t kBusDrainMaxIters = 8u;

// ---------------------------------------------------------------------------
// drainCommandBatch — route every OutCommand in batch to its target.
//
// Parameters:
//   batch  — CommandBatch returned by a subsystem tick() (e.g. Planner)
//   drive  — Drive subsystem; receives DrivetrainCommand{TWIST} for verb_id=1
//   planner— Planner; receives PlannerCommand for verb_id=2 (reserved)
//   queue  — CommandQueue for passthrough / ASCII-verb commands
//   cmd    — CommandProcessor (unused in the current dispatch table; retained
//             for future dequeueOne integration)
//
// Returns: the number of commands actually routed (≤ kBusDrainMaxIters).
//          If batch.cmds_count > kBusDrainMaxIters, the excess is silently
//          discarded and kBusDrainMaxIters is returned.
//          If push_front fails (queue full on a priority command), the count
//          at the point of failure is returned.
// ---------------------------------------------------------------------------
uint8_t drainCommandBatch(
    const msg::CommandBatch& batch,
    subsystems::Drive&       drive,
    Planner&                 planner,
    CommandQueue&            queue,
    CommandProcessor&        cmd);
