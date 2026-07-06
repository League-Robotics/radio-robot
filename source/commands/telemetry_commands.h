#pragma once

// ---------------------------------------------------------------------------
// telemetry_commands.h -- the STREAM/SNAP command family (082-004): periodic
// TLM emission plus a synchronous one-shot snapshot, both built on
// telemetry/tlm_frame.h's pure Telemetry::buildTlmFrame(). This file is the
// IMPURE glue: it samples Subsystems::Hardware/Drivetrain/PoseEstimator,
// shapes a Telemetry::TlmFrameInput from that live state, and calls
// buildTlmFrame() -- mirroring dev_commands.h's own split between a shared
// state struct + registration table (this file) and the pure per-field
// sourcing rules (architecture-update.md (082) Decision 7).
//
//   STREAM <ms>   -- sets the periodic-emission period, clamped to a 20ms
//                    floor (STREAM 10 -> OK stream period=20). STREAM 0
//                    disables periodic emission. Binds the periodic-emission
//                    reply channel to whichever channel issued this STREAM
//                    command (docs/protocol-v2.md §8's channel-binding rule)
//                    -- captured as replyFn/replyCtx below.
//   SNAP          -- one TLM line synchronously, replied on the SAME
//                    channel/replyFn the SNAP command itself arrived on
//                    (NOT necessarily the STREAM-bound channel -- only the
//                    seq= counter is shared between the two verbs, per the
//                    ticket's acceptance criteria).
//
// Deliberately minimal this sprint (Decision 5): no `STREAM fields=<csv>`
// subscription (always emit the full fixed field set -- there is no second
// field set to select between in this dev-bench tree yet), no D10 idle-rate
// refinement (`max(period, 500ms)` when idle), no channel-rebinding
// nuance beyond "the channel that most recently issued STREAM is the bound
// recipient." These are named, explicit deferrals -- do not reintroduce
// without a fresh, acceptance-bar-driven reason.
//
// Field sourcing (Decision 7, enforced by construction -- see
// telemetry_commands.cpp's telemetryEmit()):
//   enc=/vel=  -- hardware.motor(port).position()/.velocity() DIRECTLY for
//                 the Drivetrain's bound pair. NEVER Drivetrain::state()'s
//                 vel_[] (commanded targets, a different semantic) -- this
//                 file does not include or reference Drivetrain::state() at
//                 all for these two fields.
//   pose=/encpose= -- poseEstimator->fusedPose()/->encoderPose().
//   otos=      -- the raw sampled odometer pose (hardware.odometer()->pose()),
//                 OMITTED (not zero-filled) when hardware.odometer() is null.
//   twist=     -- BodyKinematics::forward() applied to the SAME directly-read
//                 wheel velocities vel= uses, plus poseEstimator->trackwidth()
//                 -- a pure kinematic transform of directly-measured rates,
//                 never Drivetrain::state(), never EKF velocity-channel state
//                 (EkfTiny implements none -- see estimation/ekf_tiny.h).
//   mode=      -- 'I' when !drivetrain.active(), 'S' when active. Exactly
//                 two values this sprint (no T/D/G -- sprint 083's motion
//                 verbs).
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/pose_estimator.h"

#if ROBOT_DEV_BUILD

// TelemetryState -- the STREAM/SNAP family's shared wiring, owned by
// main.cpp (a function-static instance, matching DevLoopState's own
// convention -- dev_commands.h) and handed to telemetryCommands() as the
// handlerCtx every descriptor shares, and to devLoopTick()'s
// periodic-emission step via DevLoop::telemetry (dev_loop.h).
struct TelemetryState {
  Subsystems::Hardware* hardware = nullptr;
  Subsystems::Drivetrain* drivetrain = nullptr;
  Subsystems::PoseEstimator* poseEstimator = nullptr;

  uint32_t periodMs = 0;      // [ms] 0 = disabled; set (clamped) by STREAM
  uint16_t seq = 0;           // shared by every STREAM-driven frame AND SNAP

  // Bound at STREAM-command time (docs/protocol-v2.md §8's channel-binding
  // rule) -- the periodic-emission step's ONLY reply sink. SNAP never reads
  // these; it replies on its own dispatch replyFn/replyCtx instead (see this
  // file's header comment).
  ReplyFn replyFn = nullptr;
  void* replyCtx = nullptr;

  // devLoopTick()'s periodic-emission gate (dev_loop.cpp): true once at
  // least one frame has been emitted; lastEmitMs is that frame's `now`.
  bool hasLastEmit = false;
  uint32_t lastEmitMs = 0;    // [ms]
};

// telemetryEmit -- shared frame-assembly + emission path: samples the bound
// wheel pair (state.drivetrain->ports()), the two PoseEstimator readings,
// and the active Hardware owner's odometer (if any), shapes a
// Telemetry::TlmFrameInput per the field-sourcing rules above, formats it via
// Telemetry::buildTlmFrame(), advances state.seq, and calls
// replyFn(line, replyCtx). Used by BOTH devLoopTick()'s periodic-emission
// step (passing state.replyFn/state.replyCtx, the STREAM-bound channel) and
// SNAP's handler (passing its own dispatch replyFn/replyCtx). A null replyFn
// is a silent no-op (mirrors dev_loop.cpp's own null-fn guard on the
// watchdog-fire EVT reply) -- state.seq is NOT advanced in that case, since
// no frame was actually emitted.
void telemetryEmit(TelemetryState& state, uint32_t now, ReplyFn replyFn, void* replyCtx);

// Returns the STREAM/SNAP command table, bound to the given shared state
// (every pointer field must be set before any call this table's handlers
// make -- mirrors devCommands()'s own contract, dev_commands.h).
std::vector<CommandDescriptor> telemetryCommands(TelemetryState& state);

#endif  // ROBOT_DEV_BUILD
