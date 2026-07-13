// main_loop.h -- Rt::MainLoop: sprint 093's gutted cyclic executive
// (clasi/sprints/093-simplify-the-main-loop-bare-wheel-driving-executive/
// architecture-update.md Step 5), replacing the four-subsystem/watchdog/
// reply-sink design ticket 087-007 built and later tickets (088-092) grew.
// Reordered/shrunk again by sprint 094 ticket 094-005 (see below). 099-001
// reverses 094's "no MainLoop wrapper" decision: source/main.cpp now
// constructs one `Rt::MainLoop` and calls `tick(bb, now)` too, the SAME
// function tests/_infra/sim/sim_api.cpp's SimHandle already called -- both
// wiring sites share this one mandatory-tick implementation (the
// 1:1-mirror invariant), rather than each hand-rolling its own copy.
//
// This class owns exactly three subsystem references -- `Hardware&`,
// `Drivetrain&`, and (099-004) `PoseEstimator&` -- and nothing else: no
// watchdogs, no reply sinks, no planner reference, no per-verb bookkeeping.
// Safety supervision (the two loop-owned safety-watchdog classes plus the
// `estop()` bypass) and loop-originated wire output (`EVT`/periodic `TLM`)
// stay gone from the tick entirely -- not stubbed, deleted (their classes
// remain parked, un-wired, on disk; see the 093 architecture update's
// "Parked" list). Pose estimation (099-004) is back; OTOS fusion (099-007,
// "the one-token flip") is now live too -- `otosObs` is `&otosSample` when
// `hardware_.odometer()->fusableThisPass()` (called exactly once per pass)
// and `otosSample.stamp.valid` are both true, else `nullptr`. Motion-goal
// closure is back too, in a different shape: `Drivetrain` itself now owns a
// `Motion::SegmentExecutor`
// (094-004) and stages its own output directly through `hardware_` -- there
// is no longer a separate motion-executor subsystem for this loop to tick,
// nor an addressed output for it to route.
//
// tick(bb, now) (099-004: now a five-step, one-pass sequence -- see
// architecture-update.md D1's pass pseudocode): tick `Hardware` (flushes
// whatever Drivetrain staged onto the motor refs last pass, collects fresh
// encoders), tick `Drivetrain` (drains `bb.segmentIn`/`bb.driveIn`, runs the
// executor/escape-hatch dispatch, stages this pass's setpoints directly
// through `hardware_`'s motor refs), read the bound pair's FRESH
// `MotorState` and tick `PoseEstimator` (drains `bb.poseResetIn`, posts any
// re-anchored pose to `bb.otosSetPoseIn`), drain `bb.otosSetPoseIn` into
// `hardware_.odometer()->applySetPose(...)`, commit every subsystem's fresh
// state into `bb`. See main_loop.cpp for the exact sequencing and why
// `Hardware::tick()` stays first.
//
// Command ingestion (`CommandRouter::route()`) is, as before, NOT wrapped
// here -- `CommandRouter` is its own top-level object (declared beside
// `Rt::Blackboard`, not nested inside the loop); main.cpp's/sim_api.cpp's
// own slack phase calls it directly. There is no runtime config-application
// authority left to wire in either (093: boot config is applied once,
// directly, at construction) and no watchdog-feed hook (093 removes the
// watchdog itself).
//
// SAFETY NOTE (stakeholder-directed removal, 2026-07-08): this loop no
// longer runs a serial-silence safety watchdog or an `estop()` bypass. That
// is acceptable ONLY because the robot runs mounted on a stand with wheels
// off the ground for the whole of this loop's current operating envelope
// (`.claude/rules/hardware-bench-testing.md`) -- see architecture-update.md
// Decision 2. This is a standing risk the moment this firmware is ever run
// off the stand.
#pragma once

#include "runtime/blackboard.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/pose_estimator.h"


namespace Rt {

class MainLoop {
 public:
  MainLoop(Subsystems::Hardware& hardware, Subsystems::Drivetrain& drivetrain,
           Subsystems::PoseEstimator& poseEstimator);

  // The mandatory control tick + commit -- see main_loop.cpp for the exact
  // per-subsystem sequencing. Called once per outer pass by BOTH main.cpp
  // (real hardware) and tests/_infra/sim/sim_api.cpp (SimHardware) -- the
  // 1:1-mirror invariant (both wiring sites share this ONE function).
  void tick(Blackboard& bb, uint32_t now);

 private:
  // commit -- copies each subsystem's freshly-ticked state into bb ->
  // x[k+1] (bb.motors[]/bb.drivetrain, bb.encoderPose/bb.fusedPose/
  // bb.poseStepped/bb.bodyState, and (099-007) bb.otos/bb.otosValid/
  // bb.otosConnected). otosFusable/otosSample are THIS pass's already-read
  // fusableThisPass()/pose() values, threaded in from tick() rather than
  // re-read here -- fusableThisPass() is a one-shot read-and-clear signal
  // that may be called AT MOST ONCE per pass (hal/capability/odometer.h),
  // so commit() must never call it again itself. Called from tick() as its
  // last step (094-005: routeOutputs(), formerly called after this, is
  // deleted -- Drivetrain::tick() already staged its own wheel writes
  // directly through hardware_'s motor refs, ahead of commit()).
  void commit(Blackboard& bb, uint32_t now, bool otosFusable, const msg::PoseEstimate& otosSample);

  Subsystems::Hardware& hardware_;
  Subsystems::Drivetrain& drivetrain_;
  Subsystems::PoseEstimator& poseEstimator_;
};

}  // namespace Rt

