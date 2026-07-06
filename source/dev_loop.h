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
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
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
  CommandProcessor* processor = nullptr;
  SerialSilenceWatchdog* watchdog = nullptr;
  DevLoopState* devState = nullptr;

  ReplyFn defaultReply = nullptr;
  void* defaultReplyCtx = nullptr;
};

// devLoopTick -- runs exactly one pass of the shared dev-loop body: the
// two-slice hardware tick, statement-triggered parse (only when statement !=
// nullptr), the outbox drain, Drivetrain governance, pose estimation
// (082-003 -- see below), and the watchdog check -- reproducing main.cpp's
// pre-081-002 loop body exactly, plus 082-003's one addition (see
// dev_loop.cpp for the line-by-line correspondence). now: [ms]. statement:
// nullptr when no statement is being fed this pass.
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
void devLoopTick(DevLoop& loop, uint32_t now, const DevLoopStatement* statement);

#endif  // ROBOT_DEV_BUILD
