// verb_ids.h — compile-time verb ID constants for the command bus.
//
// These constants define the mapping between OutCommand::verb_id values and
// the subsystem apply() calls they encode.  Both the producer (MotionController2::tick())
// and the consumer (drainCommandBatch()) must agree on these constants.
//
// Encoding convention (matches 059-001 MotionController2::tick() packing):
//   kVerbDrivetrainTwist = 1  — DrivetrainCommand{TWIST}
//                                args_[0]=vx_mmps, args_[1]=vy_mmps, args_[2]=omega_rads
//   kVerbPlannerCommand  = 2  — PlannerCommand (not yet used; reserved for future goals)
//
// Verb IDs 3+ are reserved for future subsystem commands.
// Verb ID 0 is unused/invalid — OutCommand default-initialises verb_id=0.
//
// C++11, no heap, no STL, no RTTI, no exceptions.
#pragma once
#include <stdint.h>

namespace msg {

static constexpr uint32_t kVerbDrivetrainTwist = 1u;  // DrivetrainCommand{TWIST}
static constexpr uint32_t kVerbPlannerCommand  = 2u;  // PlannerCommand (reserved)

}  // namespace msg
