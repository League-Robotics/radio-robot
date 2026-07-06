#pragma once

// dev_loop.h -- the shared dev-loop body: DevLoopStatement, DevLoop, and
// devLoopTick() -- extracted from source/main.cpp's inline loop (sprint 079
// wrote that loop; this ticket, 081-002, is what pulls it into a shared,
// host-clean function so a future simulated caller (ticket 081-004's
// sim_api.cpp) can run the IDENTICAL body instead of hand-mirroring it --
// exactly the drift risk clasi/sprints/081-.../architecture-update.md's
// Step 2 responsibility table calls out for this module ("a hand-mirrored
// second copy is exactly the drift risk the design write-up itself names").
//
// Host-clean by construction: no #include "MicroBit.h", no
// Subsystems::Communicator dependency -- "how did a statement get fed in"
// (Communicator's held/taken pair vs. a future ctypes sim_command() call) is
// deliberately the CALLER's problem, never this file's -- see Decision 3
// below.
//
// See architecture-update.md (081) Decision 3 for:
//   - why DevLoopStatement is a plain, caller-owned, single-call-lifetime
//     pointer rather than a second held/taken state machine mirroring
//     Subsystems::Communicator's own (Communicator's
//     CommunicatorToCommandProcessorStatement.line aliases Communicator's
//     OWN buffer and unconditionally pulls in MicroBit.h -- it cannot be the
//     shared statement type this file depends on);
//   - why DevLoop carries its own defaultReply/defaultReplyCtx pair: the
//     watchdog-fire `EVT dev_watchdog` reply is not triggered by any inbound
//     statement, so it has no statement-supplied replyFn/replyCtx to reuse.
#include <stdint.h>

#include "commands/command_processor.h"
#include "commands/dev_commands.h"
#include "commands/motion_commands.h"
#include "commands/telemetry_commands.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/planner.h"
#include "subsystems/pose_estimator.h"

#if ROBOT_DEV_BUILD

// DevLoopStatement -- a plain, caller-owned, single-call-lifetime statement
// line plus its reply routing. Deliberately NOT a <Producer>To<Consumer>
// edge type (naming-and-style.md rule 4 does not fit -- see
// architecture-update.md Decision 3's "Consequences"): it has two
// legitimate, structurally different producers (main.cpp, copying out of
// Communicator's own held/taken CommunicatorToCommandProcessorStatement; a
// future ctypes sim_command() call, built directly from its line argument),
// not one edge between two named subsystems.
struct DevLoopStatement {
  const char* line;
  ReplyFn replyFn;
  void* replyCtx;
};

// DevLoop -- one pass's shared wiring, owned by the caller (main.cpp today;
// a future sim_api.cpp) and handed to devLoopTick() by reference every pass.
// defaultReply/defaultReplyCtx is the loop-originated reply sink (Decision
// 3): used for any reply devLoopTick() originates ITSELF -- today, only the
// watchdog-fire `EVT dev_watchdog` -- never for a reply to an inbound
// statement, which always carries its own replyFn/replyCtx via
// DevLoopStatement above. main.cpp sets this to serialReply/&comm,
// byte-identical to its pre-extraction behavior.
struct DevLoop {
  Subsystems::Hardware* hardware = nullptr;
  Subsystems::Drivetrain* drivetrain = nullptr;
  // 082-003: advanced exactly once per devLoopTick() pass, after the
  // second hardware.tick(now) slice -- see devLoopTick()'s own doc comment
  // and dev_loop.cpp for the exact call. Never dereferenced if null is ever
  // assigned here, so every caller (main.cpp; sim_api.cpp) MUST wire a real
  // Subsystems::PoseEstimator before its first devLoopTick() call, the same
  // way every caller already must wire hardware/drivetrain.
  Subsystems::PoseEstimator* poseEstimator = nullptr;
  // 082-004: read (never mutated by devLoopTick() itself beyond
  // lastEmitMs/hasLastEmit) by the periodic-emission step below -- every
  // caller (main.cpp; sim_api.cpp) must wire a real TelemetryState before
  // its first devLoopTick() call, the same way hardware/drivetrain/
  // poseEstimator already must be wired.
  TelemetryState* telemetry = nullptr;
  CommandProcessor* processor = nullptr;
  SerialSilenceWatchdog* watchdog = nullptr;
  DevLoopState* devState = nullptr;

  // 084-002: Subsystems::Planner -- the goal-closure engine S/T/D/STOP
  // (source/commands/motion_commands.cpp) stage a msg::PlannerCommand into
  // via motionState's outbox below. Ticked exactly once per pass,
  // unconditionally (mirrors poseEstimator's own always-run contract) --
  // see devLoopTick()'s own doc comment and dev_loop.cpp for the exact
  // sequencing relative to the pose-estimation step. Never dereferenced if
  // null, so every caller (main.cpp; sim_api.cpp) MUST wire a real
  // Subsystems::Planner before its first devLoopTick() call.
  Subsystems::Planner* planner = nullptr;
  // 084-002: MotionLoopState -- read for its staged msg::PlannerCommand
  // outbox (drained into planner->apply() before planner->tick() runs) and
  // its sTimeout streaming-drive watchdog (checked once per pass, fed only
  // by S's own handler -- see motion_commands.h's class comment). A second
  // new field alongside `planner` above: MotionLoopState's own per-pass
  // upkeep (the outbox drain, the sTimeout check) needs the SAME "one
  // caller, no hand-mirrored copy between main.cpp and sim_api.cpp"
  // guarantee devLoopTick() already gives poseEstimator/telemetry, so it is
  // wired here rather than via a second call site in main.cpp's own loop.
  MotionLoopState* motionState = nullptr;

  ReplyFn defaultReply = nullptr;
  void* defaultReplyCtx = nullptr;
};

// devLoopTick -- runs exactly one pass of the shared dev-loop body: the
// two-slice hardware tick, statement-triggered parse (only when statement !=
// nullptr), the outbox drain, Drivetrain governance, pose estimation
// (082-003 -- see below), periodic TLM emission (082-004 -- see below), and
// the watchdog check -- reproducing main.cpp's pre-081-002 loop body
// exactly, plus 082-003's and 082-004's additions (see dev_loop.cpp for the
// line-by-line correspondence). now: [ms]. statement: nullptr when no
// statement is being fed this pass.
//
// Pose estimation (082-003): after the second (freshest-read) hardware.tick()
// slice, this pass's bound wheel pair (drivetrain.ports(), queried
// UNCONDITIONALLY -- not gated on drivetrain.active(), since pose estimation
// passively OBSERVES the bound wheels rather than requiring authority over
// them) and the active Hardware owner's Hal::Odometer (hardware.odometer(),
// nullptr for Subsystems::NezhaHardware this sprint) feed EXACTLY ONE
// loop.poseEstimator->tick() call per pass -- never zero (unconditional),
// never twice (a single, unbranched call site) -- see pose_estimator.h's own
// class comment for what tick() does with these.
//
// Periodic TLM emission (082-004): the ONE new step, immediately after the
// pose-estimation call above. If loop.telemetry->periodMs > 0 and enough
// time has elapsed since loop.telemetry->lastEmitMs (or no frame has been
// emitted yet -- hasLastEmit), formats and sends exactly one TLM frame on
// the STREAM-bound loop.telemetry->replyFn/replyCtx, then updates
// lastEmitMs/hasLastEmit. Never runs when periodMs == 0 (STREAM 0 disabled
// it) or when no channel has ever issued STREAM (replyFn stays null --
// telemetryEmit() itself no-ops on a null replyFn). SNAP is unrelated to
// this step -- it is dispatched like any other command, synchronously,
// during the statement-parse beat above.
//
// Motion executor (084-002): the ONE new step, immediately after the
// pose-estimation call above (and before periodic TLM emission). Drains
// loop.motionState's staged msg::PlannerCommand outbox into
// loop.planner->apply(), checks/fires loop.motionState's sTimeout
// streaming-drive watchdog (gated on loop.planner->state().mode ==
// STREAMING -- see motion_commands.h), then ticks loop.planner
// unconditionally with this pass's leftObs/rightObs and
// loop.poseEstimator->fusedPose(), draining its held command into
// drivetrain.apply() and its held completion Event into an `EVT done
// <verb> ...` reply on loop.defaultReply/loop.defaultReplyCtx -- see
// dev_loop.cpp for the exact sequencing and the DriveMode -> verb mapping.
void devLoopTick(DevLoop& loop, uint32_t now, const DevLoopStatement* statement);

#endif  // ROBOT_DEV_BUILD
