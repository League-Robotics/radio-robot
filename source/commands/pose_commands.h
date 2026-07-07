#pragma once

// ---------------------------------------------------------------------------
// pose_commands.h -- the pose-set command family (084-007/SUC-006,
// rewritten pointerless 087-006): `SI` (re-anchor the believed world pose)
// and `ZERO enc` (rezero the bound pair's encoders).
//
// **Decision 7 (architecture-update-r1.md), router-half:** SI/ZERO are
// one-shot commands whose effects are entangled with PoseEstimator's own
// integration (the phantom-jump coherence problem) -- so, unlike SET's
// config-plane deltas (which flow through the Configurator), SI/ZERO POST
// directly to the target-drained reset queues PoseEstimator/Hardware
// themselves consume (bb.poseResetIn, bb.motorResetIn[]), plus the
// odometer-directed fan-out the loop drains directly (bb.otosSetPoseIn):
//   - SI posts Rt::PoseResetCommand{kind=kSetPose, pose} to bb.poseResetIn
//     (drained by Subsystems::PoseEstimator::tick(), ticket 004) AND the
//     SAME pose to bb.otosSetPoseIn (a Mailbox<msg::SetPose>, drained by the
//     loop directly against hardware.odometer() -- mirrors the pre-087
//     two-call handleSI(), just posted instead of called).
//   - `ZERO enc` posts Rt::PoseResetCommand{kind=kResetBaseline} to
//     bb.poseResetIn AND sets bb.motorResetIn[left-1]/[right-1] = true
//     (drained by Subsystems::Hardware::tick(), ticket 004) -- the port
//     binding is read from bb.drivetrainConfig.left_port/right_port (the
//     published snapshot), never a Drivetrain*.
//
// Neither handler holds or dereferences a Subsystems::* pointer (SUC-006).
// SI/ZERO's own reply text is built directly from the parsed wire input
// (never a bb read-back), matching today's wire text exactly.
// ---------------------------------------------------------------------------

#if ROBOT_DEV_BUILD

#include <vector>

#include "command_types.h"
#include "runtime/command_router.h"

// Returns the pose-set command table (SI, ZERO), bound to `router`.
std::vector<CommandDescriptor> poseCommands(Rt::CommandRouter& router);

#endif  // ROBOT_DEV_BUILD
