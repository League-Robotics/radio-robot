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
  // step, gated on Planner::state().mode == STREAMING AND
  // activeVelocityVerb[0] == '\0' (see dev_loop.cpp's own comment). The
  // second half of that gate is 084-005's own addition: once Decision 6
  // made a bare `R` also report DriveMode::STREAMING, gating on mode alone
  // would let this S-only watchdog fire on an R-driven session that never
  // feeds it -- excluded by checking that activeVelocityVerb is empty
  // (i.e. the active goal is NOT R/TURN/RT).
  StreamingDriveWatchdog sTimeout;

  // activeVelocityVerb -- 084-003: disambiguates which wire verb (R, TURN,
  // RT) staged the currently-active goal, for dev_loop.cpp's "EVT done
  // <verb>" text (and, as of 084-005, for excluding a bare-R-driven
  // STREAMING session from the sTimeout gate above). Subsystems::Planner's
  // own msg::DriveMode::VELOCITY value was originally shared by all three
  // (planner.cpp's apply() staged the VELOCITY, TURN, and ROTATION goal
  // kinds identically as msg::DriveMode::VELOCITY), so DriveMode alone
  // could not disambiguate them -- this field is that disambiguation
  // mechanism, read by dev_loop.cpp's motionVerbForMode().
  //
  // 084-005 (Decision 6) update: planner.cpp's velocityShapedMode() now
  // folds VELOCITY/TURN/ROTATION into msg::DriveMode::STREAMING or ::TIMED
  // (never ::VELOCITY) depending on whether the staged command carries a
  // stop condition -- the SAME two DriveMode values plain `S`/`T` already
  // use. This means the "S/T/D/STOP stage their own unambiguous DriveMode
  // values" invariant this comment used to state no longer holds: a plain
  // `T` and a stop=-bearing `R`/`TURN`/`RT` can now both report
  // DriveMode::TIMED, and a plain `S` and a bare `R` can both report
  // DriveMode::STREAMING. To keep this field from leaking a STALE R/TURN/RT
  // verb into a later S/T/D/G completion's EVT text, handleS/handleT/
  // handleD/handleG (motion_commands.cpp) now explicitly CLEAR this field
  // (set it to "") when staging their own goal, in addition to handleR/
  // handleTURN/handleRT SETTING it -- so it is empty whenever the active
  // goal is NOT one of R/TURN/RT, and motionVerbForMode() falls back to the
  // mode's own plain verb ("S"/"T") in that case.
  char activeVelocityVerb[8] = "";
};

// Returns the S/T/D/STOP command table, bound to the given shared state
// (state.poseEstimator must be set before this table's handlers run --
// mirrors devCommands()'s/telemetryCommands()'s own contract).
std::vector<CommandDescriptor> motionCommands(MotionLoopState& state);

#endif  // ROBOT_DEV_BUILD
