// main_loop.cpp -- Rt::MainLoop: see main_loop.h for the class-level
// contract and the watchdog-check-first rationale. This is the REAL
// cyclic executive (mandatory tick -> commit) architecture-update-r1.md's
// Reference code describes -- synchronous update (Decision 6): every
// subsystem reads the committed snapshot `bb` (x[k]) as it stood BEFORE
// this pass's own commit step runs, regardless of tick() call order within
// this function, so a subsystem's observable behavior never depends on
// where it happens to sit in this sequence.
#include "runtime/main_loop.h"

#if ROBOT_DEV_BUILD

#include <cstdio>
#include <cstring>

#include "commands/command_processor.h"
#include "commands/telemetry_commands.h"
#include "hal/capability/hal_command.h"

namespace Rt {

namespace {

// motionVerbForMode -- maps the msg::DriveMode a Planner goal was driving to
// its wire verb, for "EVT done <verb> ..." text. Sampled from
// Planner::state().mode BEFORE calling tick() each pass, since a goal that
// completes THIS pass transitions mode_ back to IDLE INSIDE that same
// tick() call. Ported verbatim from ticket 006's transitional dev_loop.cpp.
const char* motionVerbForMode(msg::DriveMode mode, const char* activeVelocityVerb) {
  switch (mode) {
    case msg::DriveMode::STREAMING:
      return (activeVelocityVerb[0] != '\0') ? activeVelocityVerb : "S";
    case msg::DriveMode::TIMED:
      return (activeVelocityVerb[0] != '\0') ? activeVelocityVerb : "T";
    case msg::DriveMode::DISTANCE: return "D";
    case msg::DriveMode::VELOCITY: return activeVelocityVerb;
    case msg::DriveMode::GO_TO: return "G";
    default: return "";
  }
}

}  // namespace

MainLoop::MainLoop(Subsystems::Hardware& hardware, Subsystems::Drivetrain& drivetrain,
                    Subsystems::PoseEstimator& poseEstimator, Subsystems::Planner& planner,
                    ReplyFn serialReply, void* serialCtx, ReplyFn radioReply, void* radioCtx)
    : hardware_(hardware),
      drivetrain_(drivetrain),
      poseEstimator_(poseEstimator),
      planner_(planner),
      serialReply_(serialReply),
      serialCtx_(serialCtx),
      radioReply_(radioReply),
      radioCtx_(radioCtx) {}

void MainLoop::emergencyNeutralize() {
  // The sanctioned bypass (Decision 6): Hardware::apply()/Drivetrain::
  // apply() are the SAME narrow, immediate-write methods every OTHER
  // command-plane post eventually reaches via a subsystem's own tick() --
  // called here directly, with no bb queue in between, so the neutral is
  // STAGED into every motor object THIS call, not next pass. See
  // main_loop.h's file header for why this must run before hardware_.tick()
  // this same pass.
  hardware_.apply(buildBroadcastNeutral(msg::Neutral::BRAKE));
  drivetrain_.apply(buildDrivetrainStop(msg::Neutral::BRAKE));
}

void MainLoop::routeOutputs(Blackboard& bb, bool plannerEngagedThisPass) {
  // Decision 2: Drivetrain's ONE addressed output, unpacked into per-port
  // bb.motorIn[] -- gated on drivetrain_.active() (queried AFTER
  // drivetrain_.tick() ran this pass, so it reflects whatever THIS pass's
  // driveIn pop/governance just decided): a bare authority-steal/standby
  // output must never reach hardware (ticket 006's bug-fix #1 -- a steal
  // that leaves Drivetrain in standby must not re-touch the bound motors).
  if (drivetrain_.hasCommand()) {
    Hal::DrivetrainToHardwareCommand cmd = drivetrain_.takeCommand();
    if (drivetrain_.active()) {
      bb.motorIn[cmd.wheel[0].port - 1].post(cmd.wheel[0].command);
      bb.motorIn[cmd.wheel[1].port - 1].post(cmd.wheel[1].command);
    }
  }

  // Decision 1: Planner's own output is driveIn's SECOND producer
  // (alongside CommandRouter's DEV DT path, which always posts
  // unconditionally -- ticket 006's confirmed contract). Planner::tick()
  // holds a command UNCONDITIONALLY every pass (even a zero twist once
  // idle) -- posting that unconditionally would let a stale, idle Planner
  // silently reclaim/clobber a live DEV DT override the next time Planner
  // ticks (exactly the ambiguity Decision 1 exists to remove). Gating on
  // `plannerEngagedThisPass` (a fresh bb.motionIn arrival THIS pass, the
  // stream watchdog firing a stop THIS pass, or an already-active goal --
  // ported from ticket 006's transitional `plannerEngagedThisPass`) means
  // an idle/completed goal stops posting the instant it goes idle, so any
  // subsequent DEV DT override is never re-clobbered by a stale zero twist.
  if (planner_.hasCommand()) {
    msg::DrivetrainCommand cmd = planner_.takeCommand();
    if (plannerEngagedThisPass) {
      bb.driveIn.post(cmd);
    }
  }
}

void MainLoop::tick(Blackboard& bb, uint32_t now) {
  // === SAFETY WATCHDOG -- mandatory, first, same-pass deterministic. ===
  // See main_loop.h's file header for why this runs before hardware_.tick().
  if (watchdog_.check(now)) {
    emergencyNeutralize();
    char wbuf[32];
    CommandProcessor::replyEvt(wbuf, sizeof(wbuf), "dev_watchdog", nullptr, serialReply_,
                               serialCtx_);
  }

  // === MANDATORY: control. Reads bb (x[k]); consumes commands routed
  //     during the previous slack; each subsystem writes its OWN cell. ===
  hardware_.tick(now, bb.motorIn, bb.motorResetIn);

  // DEV STOP's broadcast neutral (posted last slack) -- deliberately NOT
  // bb.motorIn[] (a broadcast must not mark any port in-use -- see
  // blackboard.h's file header / NezhaHardware::apply()'s own "broadcast
  // never marks a port in-use" branch).
  if (!bb.hardwareBroadcastIn.empty()) {
    msg::MotorCommand neutral = bb.hardwareBroadcastIn.take();
    Hal::CommandProcessorToHardwareCommand broadcast;
    broadcast.allPorts = true;
    broadcast.count = 0;
    broadcast.addressed[0].command = neutral;
    hardware_.apply(broadcast);
  }

  // The two loop-owned watchdogs' window mailboxes (DEV WD / SET sTimeout=,
  // posted last slack) -- drained directly (neither watchdog is one of the
  // Configurator's four targets), then published for GET/telemetry reads.
  if (!bb.devWatchdogWindowIn.empty()) {
    watchdog_.setWindow(bb.devWatchdogWindowIn.take());
  }
  if (!bb.streamWatchdogWindowIn.empty()) {
    streamWatchdog_.setWindow(bb.streamWatchdogWindowIn.take());
  }
  bb.devWatchdogWindow = watchdog_.window();
  bb.streamWatchdogWindow = streamWatchdog_.window();

  Subsystems::DrivetrainPorts p = drivetrain_.ports();   // bound pair, from config

  drivetrain_.tick(now, bb.motor[p.left - 1], bb.motor[p.right - 1], bb.driveIn);

  // Odometer one-shot actions (OI/OZ/OR/OV, SI's re-anchor) -- the loop
  // legitimately holds Hardware& (composition-root status, same as
  // Rt::Configurator's own exception -- Decision 4); Hal::Odometer has no
  // tick()-driven queue parameter of its own.
  Hal::Odometer* odometer = hardware_.odometer();
  // odometerResetThisPass -- true when OI/OZ/OR/OV/SI's one-shot odometer
  // action was JUST drained (below) this SAME pass. bb.otos/bb.otosValid
  // are state-plane cells refreshed only at THIS pass's own COMMIT step
  // (Decision 6, x[k] semantics) -- so on the exact pass a reset lands,
  // bb.otos still holds the STALE, pre-reset reading. Feeding that stale
  // value into poseEstimator_.tick() as if it were a fresh OTOS measurement
  // would fabricate a large false innovation against the fresh pose
  // setPose() (poseResetIn) just re-anchored the EKF to THIS SAME pass --
  // reproduced live via SI: encoderPose() landed exactly on the requested
  // pose while fusedPose() was dragged back toward the pre-SI reading. The
  // fix: skip OTOS fusion for exactly the one pass a reset was applied;
  // bb.otos is correct again (matching the reset) by the very next pass,
  // once COMMIT has refreshed it, so fusion resumes with zero innovation.
  bool odometerResetThisPass = false;
  if (odometer != nullptr) {
    if (!bb.otosCommandIn.empty()) {
      odometer->apply(bb.otosCommandIn.take());
      odometerResetThisPass = true;
    }
    if (!bb.otosSetPoseIn.empty()) {
      msg::SetPose pose = bb.otosSetPoseIn.take();
      msg::Pose2D otosPose;
      otosPose.x = pose.x;
      otosPose.y = pose.y;
      otosPose.h = pose.h;
      msg::OdometerCommand cmd;
      cmd.setSetPose(otosPose);
      odometer->apply(cmd);
      odometerResetThisPass = true;
    }
  } else {
    // No device -- discard rather than let either Mailbox look
    // perpetually "full" to its next post() (Mailbox::post() overwrites
    // anyway, but draining keeps behavior obviously inert either way).
    bb.otosCommandIn.take();
    bb.otosSetPoseIn.take();
  }

  // Pose estimation reads bb.otos/bb.otosValid as committed LAST pass
  // (x[k]) -- this pass's fresh sample is taken during COMMIT, below
  // (Decision 6: no same-pass read of a value this SAME pass will refresh)
  // -- EXCEPT when odometerResetThisPass, per this block's own comment.
  poseEstimator_.tick(now, bb.motor[p.left - 1], bb.motor[p.right - 1],
                      (bb.otosValid && !odometerResetThisPass) ? &bb.otos : nullptr,
                      bb.poseResetIn);

  // Motion executor: drain bb.motionIn (staged by source/commands/
  // motion_commands.cpp's S/T/D/R/TURN/RT/G/STOP handlers, posted last
  // slack) into Planner::apply() -- this loop, not a command handler, is
  // the sole per-pass orchestrator (Planner's apply()/tick() split needs a
  // staging step outside Planner itself).
  bool plannerEngagedThisPass = false;
  if (!bb.motionIn.empty()) {
    MotionCommand mc = bb.motionIn.take();
    planner_.apply(mc.command, now);
    // activeVelocityVerb_ persists across passes -- updated here exactly
    // when a fresh command is staged (mirrors the pre-087 MotionLoopState
    // field's own write sites).
    std::strncpy(activeVelocityVerb_, mc.verb, sizeof(activeVelocityVerb_) - 1);
    activeVelocityVerb_[sizeof(activeVelocityVerb_) - 1] = '\0';
    if (mc.feedStreamWatchdog) {
      streamWatchdog_.feed(now);
    }
    plannerEngagedThisPass = true;
  }

  // sTimeout: DISTINCT from watchdog_ (fed by ANY command). Gating on
  // `mode == STREAMING` alone is not sufficient once a bare `R` also
  // reports STREAMING -- the `activeVelocityVerb_[0] == '\0'` check
  // excludes an R-driven session (R's handler never sets
  // feedStreamWatchdog).
  if (planner_.state().mode == msg::DriveMode::STREAMING && activeVelocityVerb_[0] == '\0' &&
      streamWatchdog_.check(now)) {
    msg::PlannerCommand stopCmd;
    stopCmd.setStop(true);
    planner_.apply(stopCmd, now);
    plannerEngagedThisPass = true;
    char wbuf[40];
    CommandProcessor::replyEvt(wbuf, sizeof(wbuf), "safety_stop", "reason=watchdog", serialReply_,
                               serialCtx_);
  }

  plannerEngagedThisPass = plannerEngagedThisPass || planner_.hasActiveCommand();

  // mode is sampled BEFORE tick() -- see motionVerbForMode()'s own doc
  // comment for why.
  msg::DriveMode activeModeBeforeTick = planner_.state().mode;
  planner_.tick(now, bb.motor[p.left - 1], bb.motor[p.right - 1], bb.fusedPose);
  if (planner_.hasEvent()) {
    Subsystems::Planner::Event ev = planner_.takeEvent();
    char body[64];
    if (ev.corrId[0] != '\0') {
      snprintf(body, sizeof(body), "#%s reason=%s", ev.corrId, ev.reason);
    } else {
      snprintf(body, sizeof(body), "reason=%s", ev.reason);
    }
    char name[16];
    snprintf(name, sizeof(name), "done %s", motionVerbForMode(activeModeBeforeTick,
                                                              activeVelocityVerb_));
    char wbuf[96];
    CommandProcessor::replyEvt(wbuf, sizeof(wbuf), name, body, serialReply_, serialCtx_);
  }

  // === COMMIT (clock edge): copy each subsystem cell into bb -> x[k+1]. ===
  for (uint32_t port = 1; port <= kPortCount; ++port) {
    bb.motor[port - 1] = hardware_.state(port);
  }
  bb.drivetrain = drivetrain_.state();
  bb.encoderPose = poseEstimator_.encoderPose();
  bb.fusedPose = poseEstimator_.fusedPose();
  bb.planner = planner_.state();
  if (odometer != nullptr) {
    odometer->tick(now);
    bb.otos = odometer->pose();
    bb.otosValid = true;
  } else {
    bb.otosValid = false;
  }

  routeOutputs(bb, plannerEngagedThisPass);

  // Periodic TLM emission: gated on bb.telemetryPeriod > 0 and enough time
  // having elapsed since the last emission (or none yet). Telemetry's own
  // internals reading bb are ticket 008's scope -- this preserves the
  // existing call site verbatim.
  if (bb.telemetryPeriod > 0 &&
      (!bb.telemetryHasLastEmit || (now - bb.telemetryLastEmitMs) >= bb.telemetryPeriod)) {
    ReplyFn replyFn = (bb.telemetryChannel == Subsystems::Channel::RADIO) ? radioReply_
                                                                          : serialReply_;
    void* replyCtx = (bb.telemetryChannel == Subsystems::Channel::RADIO) ? radioCtx_ : serialCtx_;
    telemetryEmit(bb, now, replyFn, replyCtx);
    bb.telemetryLastEmitMs = now;
    bb.telemetryHasLastEmit = true;
  }
}

}  // namespace Rt

#endif  // ROBOT_DEV_BUILD
