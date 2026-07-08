// main_loop.h -- Rt::MainLoop: sprint 087 ticket 007's real cyclic executive
// (architecture-update-r1.md's Reference code), replacing ticket 006's
// TRANSITIONAL `LoopContext`/`runLoopPass()` (source/dev_loop.{h,cpp},
// deleted by this ticket).
//
// Owns the loop's own persistent, cross-pass bookkeeping that sits OUTSIDE
// Rt::Blackboard/Rt::Configurator/Rt::CommandRouter -- the composition
// root's own state, same legitimate-reference-holding status as
// Rt::Configurator (Decision 4):
//   - the four subsystem references (Hardware/Drivetrain/PoseEstimator/
//     Planner) this loop ticks every pass;
//   - the two loop-owned watchdogs (`DEV WD`'s SerialSilenceWatchdog,
//     `SET sTimeout=`'s StreamingDriveWatchdog) -- neither is one of the
//     Configurator's four targets (blackboard.h's file header);
//   - `activeVelocityVerb_`, the "which verb staged the currently active
//     goal" disambiguation Rt::MotionCommand::verb feeds (replaces the
//     pre-087 MotionLoopState::activeVelocityVerb field);
//   - the two reply sinks the loop uses for output it originates ITSELF
//     (watchdog-fire/motion-done/safety_stop EVTs, periodic telemetry) --
//     distinct from Rt::CommandRouter's own reply channels (a dispatched
//     command's OWN reply), matching ticket 006's Decision-3 split.
//
// tick(bb, now) is the MANDATORY-then-COMMIT half of the cyclic executive:
// reads the committed snapshot `bb` (x[k]), runs every subsystem's tick(),
// drains this loop's own one-shot command mailboxes (DEV STOP's broadcast,
// the two watchdog windows, OI/OZ/OR/OV/SI's odometer actions, S/T/D/R/
// TURN/RT/G/STOP's motionIn fan-out), THEN commits x[k+1] (bulk subsystem-
// state copy, routeOutputs, the periodic-telemetry call site) -- see
// main_loop.cpp for the exact sequencing and Decision 1/2/6's rationale at
// each step. As of 090-005, tick()'s body reads as a sequence of named
// private phases -- serviceWatchdogs() -> [control/plan, inline] ->
// commit() -> routeOutputs() -- each declared below beside this class's
// other members, with tick()'s own inline control/plan section (hardware/
// drivetrain/pose-estimator/planner ticks and the motion-executor drain)
// left in place per that ticket's scope.
//
// Command ingestion (CommandRouter::route()) and config application
// (Rt::Configurator::applyOne()/pending()) are deliberately NOT wrapped
// here: per architecture-update-r1.md's Reference code, `CommandRouter` and
// `Rt::Configurator` are their OWN top-level objects (declared beside
// `Rt::Blackboard`, not nested inside the loop) -- main.cpp's/sim_api.cpp's
// own slack phase calls them directly, exactly as the Reference code shows.
// feedWatchdog() is the one small hook the slack phase must call before
// that routing decision (see its own doc comment for why).
//
// SAFETY WATCHDOG -- non-negotiable (preserve-serial-silence-safety-
// watchdog-in-greenfield-loop.md). `check()` is the FIRST action tick()
// takes (inside serviceWatchdogs(), tick()'s first call), before
// `Hardware::tick()` even runs THIS pass: this is what lets a
// fire's neutralize be visible in the SAME control pass the window expired
// in (a queue-routed neutralize -- via bb.motorIn[]/bb.driveIn -- would
// need this pass's OWN hardware.tick() to have already drained the post,
// which is impossible if the post itself only happens after that call).
// Placing the check first costs nothing: check()/feed() depend only on
// `now` and the watchdog's own internal timestamp, never on anything this
// pass's mandatory tick computes, so "first" and "last" are equally
// "mandatory, every pass, same-pass deterministic" for the watchdog's OWN
// correctness -- but "first" is what gives genuine same-pass PWM
// visibility once the emergency neutral reaches the motor's own next
// tick() call. See estop()'s doc comment for the bypass
// mechanism itself.
#pragma once

#include "commands/dev_commands.h"      // SerialSilenceWatchdog, buildBroadcastNeutral/buildDrivetrainStop
#include "commands/motion_commands.h"   // StreamingDriveWatchdog
#include "runtime/blackboard.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/planner.h"
#include "subsystems/pose_estimator.h"
#include "types/protocol.h"             // ReplyFn

#if ROBOT_DEV_BUILD

namespace Rt {

class MainLoop {
 public:
  MainLoop(Subsystems::Hardware& hardware, 
           Subsystems::Drivetrain& drivetrain,
           Subsystems::PoseEstimator& poseEstimator, 
           Subsystems::Planner& planner,
           ReplyFn serialReply, void* serialCtx, 
           ReplyFn radioReply, void* radioCtx);

  // The mandatory control tick + commit -- see main_loop.cpp for the exact
  // per-subsystem sequencing and the file-header rationale on watchdog
  // check placement. Called once per outer pass by BOTH main.cpp (real
  // hardware) and tests/_infra/sim/sim_api.cpp (SimHardware) -- the
  // 1:1-mirror invariant (both wiring sites share this ONE function,
  // mirrors ticket 081-002's "no hand-mirrored second copy" precedent).
  void tick(Blackboard& bb, uint32_t now);

  // feedWatchdog -- call on arrival of ANY command, on ANY channel,
  // regardless of content, BEFORE the caller's slack routing/config-
  // priority branch decides what to do with it (the NON-NEGOTIABLE
  // safety-watchdog contract: feeding must never be delayed by routing).
  // Mirrors dev_commands.h's SerialSilenceWatchdog::feed() contract --
  // this is simply the loop's own instance, reached through the one hook
  // the slack phase needs (main.cpp's/sim_api.cpp's own ingest step calls
  // this, then Rt::CommandRouter::route(), in that order).
  void feedWatchdog(uint32_t now) { watchdog_.feed(now); }

 private:
  // serviceWatchdogs -- tick()'s FIRST call: the safety watchdog's
  // check-and-estop (the file header's same-pass rationale), then both
  // loop-owned watchdogs' config-plane upkeep -- drain the window
  // mailboxes (DEV WD / SET sTimeout=, posted last slack; neither
  // watchdog is one of the Configurator's four targets), then publish
  // both windows to bb for GET/telemetry reads. Ordering within: check
  // BEFORE drain, so a window posted last slack governs the NEXT pass's
  // check, never the pass it drains in. The whole block has no data
  // dependency on anything else tick() does: setWindow() is a bare field
  // write on both watchdog classes (no feed/re-arm), and no subsystem
  // tick reads the windows.
  void serviceWatchdogs(Blackboard& bb, uint32_t now);

  // commit -- the COMMIT clock edge (090-005): copies each subsystem's
  // freshly-ticked state into bb -> x[k+1] (bb.motors[]/bb.drivetrain/
  // bb.encoderPose/bb.fusedPose/bb.planner), ticks the odometer and
  // publishes its sample (bb.otos), and folds THIS pass's one sanctioned
  // Hal::Odometer::fusableThisPass() read (see that method's own doc
  // comment -- it may be called AT MOST ONCE per pass) into bb.otosValid.
  // `otosFusableThisPass` is threaded in as a parameter rather than
  // recomputed here for exactly that reason -- tick()'s earlier
  // poseEstimator_.tick() fusion gate is fusableThisPass()'s one sanctioned
  // caller, so commit() reuses that same call's result instead of polling
  // it again. Called from tick() at the exact point the inline COMMIT
  // block used to run, immediately before routeOutputs().
  void commit(Blackboard& bb, uint32_t now, bool otosFusableThisPass);

  // routeOutputs -- drains Drivetrain's and Planner's OWN output edges
  // (hasCommand()/takeCommand()) into their next consumer's input queue,
  // per Decision 1 (Planner's authority-gated arbitration with DEV DT for
  // driveIn) and Decision 2 (Drivetrain's one addressed output unpacked
  // into per-port bb.motorIn[]). See main_loop.cpp for the two gates'
  // exact rationale.
  void routeOutputs(Blackboard& bb, bool plannerEngagedThisPass);

  // estop -- the sanctioned bypass (Decision 6's one
  // exception to synchronous update): calls Hardware::apply()/
  // Drivetrain::apply() DIRECTLY -- the SAME narrow, immediate-write
  // methods every OTHER command-plane post eventually reaches, just
  // invoked here without a queue in between. Never touches
  // bb.driveIn/bb.motorIn/bb.hardwareBroadcastIn.
  void estop();

  Subsystems::Hardware& hardware_;
  Subsystems::Drivetrain& drivetrain_;
  Subsystems::PoseEstimator& poseEstimator_;
  Subsystems::Planner& planner_;

  SerialSilenceWatchdog watchdog_;
  StreamingDriveWatchdog streamWatchdog_;
  char activeVelocityVerb_[8] = "";

  ReplyFn serialReply_ = nullptr;
  void* serialCtx_ = nullptr;
  ReplyFn radioReply_ = nullptr;
  void* radioCtx_ = nullptr;
};

}  // namespace Rt

#endif  // ROBOT_DEV_BUILD
