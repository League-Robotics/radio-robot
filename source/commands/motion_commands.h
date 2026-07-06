#pragma once

// ---------------------------------------------------------------------------
// motion_commands.h -- the S/T/D/STOP wire family (084-002): top-level
// production protocol-v2 verbs (matching sprint 082's own STREAM/SNAP
// top-level precedent), registered alongside DEV/telemetry in main.cpp's
// (and sim_api.cpp's) command table.
//
// Thin wire-parsing layer over ticket 001's Subsystems::Planner
// (source/subsystems/planner.h): every handler here parses its verb's wire
// shape (grammar ported from source_old/commands/MotionCommands.cpp --
// parseS/parseT/parseD/mc_packStopKVs/mc_parseStopTokenInto), builds a
// msg::PlannerCommand, and STAGES it into MotionLoopState's outbox below --
// it never calls Subsystems::Planner::apply()/tick() itself. dev_loop.cpp
// (084-002's own new step) is the sole drainer, the same "one orchestrator
// drains every outbox" discipline DevLoopState's outbox already established
// (architecture-update.md (084) Decision 7: motion gets its OWN state
// struct, not DevLoopState's).
//
// MotionLoopState is NOT DevLoopState (Decision 7) -- production protocol-v2
// verbs must not be entangled with the DEV family's bench-diagnostic state,
// so a future non-dev production build can carry this struct forward
// unchanged. It holds three things:
//   - poseEstimator: read-only, for BodyKinematics::forward()'s trackwidth
//     argument -- S/T/D's wire args are per-wheel speeds (l, r), which every
//     handler here converts to a body twist (v, omega) the same way
//     `DEV DT VW`/telemetry_commands.cpp's own twist= field already do,
//     never duplicating Drivetrain's own (private) trackwidth copy.
//   - the staged msg::PlannerCommand outbox (hasCommand/command) --
//     dev_loop.cpp's new step drains this into Planner::apply() before
//     calling Planner::tick(), mirroring DevLoopState's
//     hasDrivetrainCommand/drivetrainCommand shape exactly.
//   - sTimeout: the streaming-drive watchdog, fed ONLY by S's own handler
//     (never by any other statement) and checked once per pass by
//     dev_loop.cpp's new step -- DISTINCT from dev_commands.h's
//     SerialSilenceWatchdog (`DEV WD`), which is a completely separate
//     instance fed by ANY statement on ANY channel regardless of content.
//     Conflating the two would defeat the point of either (architecture-
//     update.md (084) Design Rationale Risk 2) -- this is why sTimeout gets
//     its own small watchdog type (StreamingDriveWatchdog below) rather than
//     reusing SerialSilenceWatchdog's instance or even its type.
// ---------------------------------------------------------------------------

#if ROBOT_DEV_BUILD

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "messages/planner.h"
#include "subsystems/pose_estimator.h"

// ---------------------------------------------------------------------------
// StreamingDriveWatchdog -- fire-once-per-silence-episode timer, the same
// feed()/check()/setWindow() contract as dev_commands.h's
// SerialSilenceWatchdog, but an independent TYPE (not just a second
// instance) so this file has no compile-time dependency on dev_commands.h --
// Decision 7's whole point is that the four new command families never
// need to be untangled from the DEV family later.
// ---------------------------------------------------------------------------
class StreamingDriveWatchdog {
 public:
  static constexpr uint32_t kDefaultWindow = 500;   // [ms] docs/protocol-v2.md §10's sTimeout default

  explicit StreamingDriveWatchdog(uint32_t window = kDefaultWindow) : windowMs_(window) {}

  // Call once every time an `S` command arrives (never for T/D/STOP/any
  // other statement -- that is dev_commands.h's SerialSilenceWatchdog's job).
  void feed(uint32_t now) { lastFeedMs_ = now; fired_ = false; }

  void setWindow(uint32_t window) { windowMs_ = window; }
  uint32_t window() const { return windowMs_; }

  // Returns true exactly once per silence episode: the first check() call at
  // or after the window has elapsed since the last feed(). Subsequent calls
  // return false until the next feed() re-arms it.
  bool check(uint32_t now) {
    if (fired_) return false;
    if (now - lastFeedMs_ >= windowMs_) {
      fired_ = true;
      return true;
    }
    return false;
  }

 private:
  uint32_t windowMs_;
  uint32_t lastFeedMs_ = 0;
  bool fired_ = false;
};

// ---------------------------------------------------------------------------
// MotionLoopState -- see this file's header comment.
// ---------------------------------------------------------------------------
struct MotionLoopState {
  Subsystems::PoseEstimator* poseEstimator = nullptr;   // set by main.cpp/sim_api.cpp before first use

  // The outbox (mirrors DevLoopState's hasDrivetrainCommand/drivetrainCommand
  // shape): S/T/D/STOP's handlers stage here; dev_loop.cpp's new step drains
  // it into Subsystems::Planner::apply() once per pass. Latest-wins, same as
  // every other outbox in this tree.
  bool hasCommand = false;
  msg::PlannerCommand command = {};

  // Fed only by S's handler; checked once per pass by dev_loop.cpp's new
  // step, gated on Planner::state().mode == STREAMING (see dev_loop.cpp's
  // own comment for why that gate is what "armed" means here, rather than a
  // separate bool).
  StreamingDriveWatchdog sTimeout;
};

// Returns the S/T/D/STOP command table, bound to the given shared state
// (state.poseEstimator must be set before this table's handlers run --
// mirrors devCommands()'s/telemetryCommands()'s own contract).
std::vector<CommandDescriptor> motionCommands(MotionLoopState& state);

#endif  // ROBOT_DEV_BUILD
