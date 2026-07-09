#pragma once

// ---------------------------------------------------------------------------
// motion_commands.h -- the S/T/D/R/TURN/RT/G/STOP wire family (084-002..005,
// rewritten pointerless 087-006). 093-001: `motionCommands()` now registers
// only S/STOP at the wire (see motion_commands.cpp's registration comment);
// T/D/R/TURN/RT/G's parser/handler functions below are left source-unchanged
// but unregistered -- same "removed code is left un-wired, not deleted"
// treatment as the other command families (clasi/sprints/093-.../
// architecture-update.md Step 5/Migration Concerns).
//
// S/STOP (093-001): direct wheel-drive translators, no Planner involvement.
// `handleS` builds a msg::WheelTargets from the parsed l/r ints and posts a
// msg::DrivetrainCommand{WHEELS} to Rt::Blackboard::driveIn; `handleStop`
// builds a msg::DrivetrainCommand{NEUTRAL} inline (deliberately WITHOUT the
// standby side-channel -- see handleStop's own doc comment in
// motion_commands.cpp for why dev_commands.h's buildDrivetrainStop(), which
// sets standby=true, silently dropped the neutral instead of stopping the
// wheels) to the same mailbox. Neither touches bb.motionIn, msg::
// PlannerCommand, or Subsystems::Planner.
//
// T/D/R/TURN/RT/G (unaffected by 093-001, parked/unregistered): still a thin
// wire-parsing layer over Subsystems::Planner (source/subsystems/planner.h)
// -- each handler parses its verb's wire shape, builds a
// msg::PlannerCommand, and POSTS it (wrapped in a Rt::MotionCommand, source/
// runtime/commands.h) to Rt::Blackboard::motionIn -- never calling
// Subsystems::Planner::apply()/tick() itself, never holding a
// Subsystems::PoseEstimator*/Subsystems::Planner* (SUC-006). Since
// `buildTable()` no longer calls `motionCommands()`'s wholesale eight-verb
// output for these six, they are unreachable at the wire; the loop's own
// motion-executor drain of bb.motionIn is untouched by this ticket (that is
// ticket 002's concern).
//
// Rt::MotionCommand's `verb` field replaces the pre-087
// MotionLoopState::activeVelocityVerb field's SEMANTICS exactly (empty for
// S/T/D/G -- Planner's own msg::DriveMode already names the verb
// unambiguously; "R"/"TURN"/"RT" otherwise, since all three can share a
// DriveMode value with S/T -- planner.cpp's velocityShapedMode()) but not
// its STORAGE: the loop, not this file, is what remembers "which verb staged
// the CURRENTLY ACTIVE goal" across passes (this file only ever posts a
// FRESH command; persisting the disambiguation across passes with no new
// command staged is the loop's own bookkeeping, ticket 007).
//
// Rt::MotionCommand's `feedStreamWatchdog` flag replaces the pre-087
// MotionLoopState::sTimeout.feed() call inside handleS() -- fed ONLY by S's
// own handler (never T/D/G/R/TURN/RT/STOP), checked once per pass by the
// loop against its OWN loop-owned StreamingDriveWatchdog instance (DISTINCT
// from dev_commands.h's SerialSilenceWatchdog, `DEV WD`) -- see
// runtime/commands.h's own field doc comment.
// ---------------------------------------------------------------------------


#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "runtime/command_router.h"

// ---------------------------------------------------------------------------
// StreamingDriveWatchdog -- fire-once-per-silence-episode timer, the same
// feed()/check()/setWindow() contract as dev_commands.h's
// SerialSilenceWatchdog, but an independent TYPE (not just a second
// instance) so this file has no compile-time dependency on dev_commands.h.
// Loop-owned (087-006: no longer embedded in a deleted MotionLoopState) --
// see runtime/commands.h's Rt::MotionCommand::feedStreamWatchdog.
// ---------------------------------------------------------------------------
class StreamingDriveWatchdog {
 public:
  static constexpr uint32_t kDefaultWindow = 500;   // [ms] docs/protocol-v2.md §10's sTimeout default

  explicit StreamingDriveWatchdog(uint32_t window = kDefaultWindow) : windowMs_(window) {}

  // Call once every time an `S` command arrives (never for T/D/STOP/any
  // other command -- that is dev_commands.h's SerialSilenceWatchdog's job).
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

// Returns the S/T/D/R/TURN/RT/G/STOP command table, bound to `router`.
std::vector<CommandDescriptor> motionCommands(Rt::CommandRouter& router);

