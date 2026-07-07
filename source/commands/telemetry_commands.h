#pragma once

// ---------------------------------------------------------------------------
// telemetry_commands.h -- the STREAM/SNAP command family (082-004, rewritten
// pointerless 087-006): periodic TLM emission plus a synchronous one-shot
// snapshot, both built on telemetry/tlm_frame.h's pure
// Telemetry::buildTlmFrame(). This file is the IMPURE glue: it samples
// Rt::Blackboard's committed state cells, shapes a Telemetry::TlmFrameInput
// from that, and calls buildTlmFrame() -- never a Hardware/Drivetrain/
// PoseEstimator/Planner pointer (SUC-006).
//
//   STREAM <ms>   -- sets the periodic-emission period, clamped to a 20ms
//                    floor (STREAM 10 -> OK stream period=20). STREAM 0
//                    disables periodic emission. Binds the periodic-emission
//                    reply channel to whichever channel issued this STREAM
//                    command (docs/protocol-v2.md §8's channel-binding rule)
//                    -- bb.telemetryChannel, read from
//                    Rt::CommandRouter::currentChannel() (see command_router.h).
//   SNAP          -- one TLM line synchronously, replied on the SAME
//                    channel/replyFn the SNAP command itself arrived on
//                    (NOT necessarily the STREAM-bound channel -- only the
//                    seq= counter is shared between the two verbs, per the
//                    ticket's acceptance criteria).
//
// Deliberately minimal this sprint (Decision 5): no `STREAM fields=<csv>`
// subscription, no D10 idle-rate refinement, no channel-rebinding nuance
// beyond "the channel that most recently issued STREAM is the bound
// recipient." These are named, explicit deferrals -- do not reintroduce
// without a fresh, acceptance-bar-driven reason.
//
// Field sourcing (Decision 7, enforced by construction -- see
// telemetry_commands.cpp's telemetryEmit()), now read from bb:
//   enc=/vel=  -- bb.motor[port-1]'s position/velocity DIRECTLY for the
//                 Drivetrain's bound pair (bb.drivetrainConfig.left_port/
//                 right_port). NEVER bb.drivetrain's vel_[] (commanded
//                 targets, a different semantic).
//   pose=/encpose= -- bb.fusedPose/bb.encoderPose.
//   otos=      -- bb.otos, OMITTED (not zero-filled) when bb.otosPresent is
//                 false (a boot-time snapshot of whether any Hal::Odometer
//                 exists at all -- see blackboard.h's file header).
//   twist=     -- BodyKinematics::forward() applied to the SAME directly-read
//                 wheel velocities vel= uses, plus bb.drivetrainConfig.
//                 trackwidth (the SAME value PoseEstimator::configure() was
//                 given -- both share msg::DrivetrainConfig, ticket
//                 087-004/005) -- a pure kinematic transform, never
//                 bb.drivetrain, never EKF velocity-channel state.
//   mode=      -- bb.planner.mode (msg::DriveMode), mapped to a single wire
//                 character -- I/S/T/D/G, per docs/protocol-v2.md §8 and
//                 architecture-update.md (084) Decision 6.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "runtime/command_router.h"

#if ROBOT_DEV_BUILD

// telemetryEmit -- shared frame-assembly + emission path: samples bb's
// committed state cells for the bound wheel pair, the two pose readings, and
// the odometer (if present), shapes a Telemetry::TlmFrameInput per the
// field-sourcing rules above, formats it via Telemetry::buildTlmFrame(),
// advances bb.telemetrySeq, and calls replyFn(line, replyCtx). Used by BOTH
// the loop's periodic-emission step (passing the channel resolved from
// bb.telemetryChannel) and SNAP's handler (passing its own dispatch
// replyFn/replyCtx). A null replyFn is a silent no-op -- bb.telemetrySeq is
// NOT advanced in that case, since no frame was actually emitted.
void telemetryEmit(Rt::Blackboard& bb, uint32_t now, ReplyFn replyFn, void* replyCtx);

// Returns the STREAM/SNAP command table, bound to `router`.
std::vector<CommandDescriptor> telemetryCommands(Rt::CommandRouter& router);

#endif  // ROBOT_DEV_BUILD
