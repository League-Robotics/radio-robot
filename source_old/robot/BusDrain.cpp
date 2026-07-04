// BusDrain.cpp — command-queue bus drain and route (ticket 059-003).
//
// See BusDrain.h for design notes and bounded-cascade / safety-priority policy.
//
// Dispatch table (verb_id → subsystem):
//   kVerbDrivetrainTwist (1) → drive.apply(DrivetrainCommand{TWIST})
//   kVerbPlannerCommand  (2) → planner.apply(PlannerCommand)  [reserved; no-op]
//   all other verb IDs       → queue.push_back / queue.push_front if priority
//
// For passthrough commands (unrecognised verb IDs routed via the queue), a
// ParsedCommand with desc=nullptr is enqueued.  The caller must NOT call
// CommandProcessor::dequeueOne() on such entries without first guarding for
// desc==nullptr.  This is consistent with the ticket's intent: unknown
// OutCommands are deposited on the queue as opaque tokens; a future router
// layer (ticket 059-005) will handle them explicitly.
//
// C++11, no heap/STL/RTTI/exceptions.  RETURN model.

// Sprint 050, Ticket 004: EKFTiny must be included BEFORE any header that
// transitively pulls in tinyekf.h.
#define EKF_N 5
#define EKF_M 2
#include "state/EKFTiny.h"

#include "robot/BusDrain.h"
#include "subsystems/drive/Drive.h"         // subsystems::Drive, apply()
#include "superstructure/Planner.h"        // Planner, apply()
#include "commands/CommandQueue.h"          // CommandQueue, ParsedCommand
#include "commands/CommandProcessor.h"      // CommandProcessor (signature only)
#include "messages/common.h"               // msg::CommandBatch, OutCommand
#include "messages/drivetrain.h"           // msg::DrivetrainCommand, BodyTwist3
#include "messages/planner.h"              // msg::PlannerCommand
#include "messages/verb_ids.h"             // kVerbDrivetrainTwist, kVerbPlannerCommand
#include <cstring>                         // memset

// ---------------------------------------------------------------------------
// drainCommandBatch — bounded route/dispatch loop.
// ---------------------------------------------------------------------------
uint8_t drainCommandBatch(
    const msg::CommandBatch& batch,
    subsystems::Drive&       drive,
    Planner&                 planner,
    CommandQueue&            queue,
    CommandProcessor&        /*cmd*/)   // retained for future dequeueOne integration
{
    // Determine how many commands to process this call.
    uint8_t limit = (batch.cmds_count < kBusDrainMaxIters)
                    ? batch.cmds_count
                    : kBusDrainMaxIters;

    uint8_t routed = 0;

    for (uint8_t i = 0; i < limit; ++i) {
        const msg::OutCommand& oc = batch.cmds_[i];

        if (oc.verb_id == msg::kVerbDrivetrainTwist) {
            // ----------------------------------------------------------------
            // TWIST → Drive.apply(DrivetrainCommand{TWIST})
            //
            // Packing convention (from Planner::tick()):
            //   args_[0] = vx_mmps
            //   args_[1] = vy_mmps
            //   args_[2] = omega_rads
            // ----------------------------------------------------------------
            msg::DrivetrainCommand drvCmd;
            msg::BodyTwist3 twist{};
            twist.v_x    = (oc.args_count >= 1) ? oc.args_[0] : 0.0f;
            twist.v_y    = (oc.args_count >= 2) ? oc.args_[1] : 0.0f;
            twist.omega = (oc.args_count >= 3) ? oc.args_[2] : 0.0f;
            drvCmd.setTwist(twist);
            drive.apply(drvCmd);
            ++routed;

        } else if (oc.verb_id == msg::kVerbPlannerCommand) {
            // ----------------------------------------------------------------
            // PLANNER → Planner.apply(PlannerCommand)
            //
            // Reserved for future use.  A full encoding of PlannerCommand
            // into OutCommand args_ is not yet defined.  This branch is a
            // placeholder so that verb_id=2 is handled without falling through
            // to the queue path.  No-op for now.
            // ----------------------------------------------------------------
            (void)planner;  // planner parameter kept for API stability
            ++routed;

        } else {
            // ----------------------------------------------------------------
            // PASSTHROUGH → CommandQueue
            //
            // For verb IDs not directly handled above, synthesise a
            // ParsedCommand with desc=nullptr and deposit it onto the queue.
            // priority=true → push_front (safety command: goes to head).
            // priority=false → push_back (normal command: goes to tail).
            //
            // push_front failure (queue full) is treated as overflow EVT:
            // return current count; caller is responsible for EVT emission.
            // ----------------------------------------------------------------
            ParsedCommand pc;
            memset(&pc, 0, sizeof(pc));
            // desc=nullptr signals a passthrough / unrouted command.
            // corrId[0]='\0' signals no correlation id.
            pc.desc    = nullptr;
            pc.replyFn = nullptr;
            pc.replyCtx = nullptr;

            bool ok;
            if (oc.priority) {
                ok = queue.push_front(pc);
            } else {
                ok = queue.push_back(pc);
            }

            if (!ok) {
                // Queue full — return routed count (caller emits EVT if needed).
                return routed;
            }
            ++routed;
        }
    }

    return routed;
}
