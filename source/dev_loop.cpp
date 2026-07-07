// dev_loop.cpp -- runLoopPass(): see dev_loop.h for the full rationale.
//
// This is main.cpp's PRE-087 loop body (as it existed just before this
// ticket, itself a direct descendant of the ORIGINAL pre-081-002 inline
// loop), with every direct subsystem-outbox touch replaced by a
// Rt::Blackboard queue post/drain:
//   - the six (087-006-deleted) *State structs' outbox fields are gone;
//     Rt::CommandRouter::route() posts into bb's queues instead.
//   - the outbox-drain step becomes: drain bb.hardwareBroadcastIn (DEV
//     STOP's broadcast neutral -- exempt from motorIn[]'s per-port in-use
//     marking), bb.devWatchdogWindowIn/bb.streamWatchdogWindowIn (the two
//     loop-owned watchdogs' windows), bb.otosCommandIn/bb.otosSetPoseIn (the
//     odometer's one-shot actions -- the loop is the one place besides the
//     Configurator that legitimately holds Hardware&, so it drains these
//     directly rather than through any subsystem's own tick()), and ALL
//     pending bb.configIn deltas (drained to exhaustion, not rationed to
//     one-per-pass -- see dev_loop.h's file header: this preserves today's
//     "SET/DEV *CFG takes effect immediately" behavior; ticket 007's real
//     cyclic executive is what introduces Decision 8's deliberate
//     multi-pass config-application latency).
//   - Drivetrain governance is gated on `active() || !bb.driveIn.empty()`,
//     not `active()` alone: Subsystems::Drivetrain::tick() is now the ONLY
//     thing that pops bb.driveIn (drivetrain.h's own tick() doc comment), so
//     a reactivation request (e.g. DEV DT VW posted while standby) must
//     still get a tick() call to actually take effect, even though `active()`
//     was false the instant BEFORE this pass's post.
//   - the motion executor drains bb.motionIn (Rt::MotionCommand) instead of
//     MotionLoopState's outbox; loop.activeVelocityVerb/loop.streamWatchdog
//     are the loop's own persistent bookkeeping, fed exactly the way
//     MotionLoopState's fields were (see runtime/commands.h's doc comment).
//   - bb's committed state cells (motor[]/drivetrain/encoderPose/fusedPose/
//     planner) are populated here, at the same relative points a command
//     handler would have read the LIVE subsystem directly before this
//     ticket, so a query dispatched mid-pass sees the same freshness it
//     always did.
#include "dev_loop.h"

#if ROBOT_DEV_BUILD

#include <cstdio>
#include <cstring>

#include "commands/command_processor.h"
#include "commands/telemetry_commands.h"
#include "hal/capability/hal_command.h"

namespace {

// motionVerbForMode -- maps the msg::DriveMode a Planner goal was driving to
// its wire verb, for "EVT done <verb> ..." text. Sampled from
// Planner::state().mode BEFORE calling tick() each pass, since a goal that
// completes THIS pass transitions mode_ back to IDLE INSIDE that same
// tick() call. Unaffected by this rewrite -- ported verbatim.
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

// resolveTelemetryReply -- maps bb.telemetryChannel to the loop's own
// serial/radio reply sink (087-006: replaces the pre-087 captured
// ReplyFn/void* pair -- see blackboard.h's file header on telemetryChannel).
void resolveTelemetryReply(const LoopContext& loop, Subsystems::Channel channel,
                           ReplyFn* replyFn, void** replyCtx) {
  if (channel == Subsystems::Channel::RADIO) {
    *replyFn = loop.radioReply;
    *replyCtx = loop.radioCtx;
  } else {
    *replyFn = loop.serialReply;
    *replyCtx = loop.serialCtx;
  }
}

}  // namespace

void runLoopPass(LoopContext& loop, Rt::Blackboard& bb, uint32_t now,
                 const Subsystems::CommunicatorToCommandProcessorStatement* statement) {
    Subsystems::Hardware& hardware = *loop.hardware;
    Subsystems::Drivetrain& drivetrain = *loop.drivetrain;

    hardware.tick(now, bb.motorIn, bb.motorResetIn);   // slice 1: any due collect lands before this pass's dispatch reads state

    if (statement != nullptr) {
        // Feed on any statement, regardless of content or dispatch outcome --
        // see dev_commands.h's watchdog contract.
        loop.watchdog.feed(now);
        loop.router->route(*statement, bb);
    }

    // Drain bb.hardwareBroadcastIn -- DEV STOP's broadcast neutral,
    // deliberately NOT bb.motorIn[] (a broadcast must not mark any port
    // in-use -- see dev_commands.h's file header).
    if (!bb.hardwareBroadcastIn.empty()) {
        msg::MotorCommand neutral = bb.hardwareBroadcastIn.take();
        Hal::CommandProcessorToHardwareCommand broadcast;
        broadcast.allPorts = true;
        broadcast.count = 0;
        broadcast.addressed[0].command = neutral;
        hardware.apply(broadcast);
    }

    // Drain the two loop-owned watchdogs' window mailboxes, then publish
    // their current windows for GET/DEV WD-adjacent reads.
    if (!bb.devWatchdogWindowIn.empty()) {
        loop.watchdog.setWindow(bb.devWatchdogWindowIn.take());
    }
    if (!bb.streamWatchdogWindowIn.empty()) {
        loop.streamWatchdog.setWindow(bb.streamWatchdogWindowIn.take());
    }
    bb.devWatchdogWindow = loop.watchdog.window();
    bb.streamWatchdogWindow = loop.streamWatchdog.window();

    // Drain the odometer's one-shot command queues directly -- the loop
    // legitimately holds Hardware& (same composition-root status as
    // Rt::Configurator); Hal::Odometer has no tick()-driven queue parameter
    // of its own.
    Hal::Odometer* odometerForCmds = hardware.odometer();
    if (odometerForCmds != nullptr) {
        if (!bb.otosCommandIn.empty()) {
            odometerForCmds->apply(bb.otosCommandIn.take());
        }
        if (!bb.otosSetPoseIn.empty()) {
            msg::SetPose pose = bb.otosSetPoseIn.take();
            msg::Pose2D otosPose;
            otosPose.x = pose.x;
            otosPose.y = pose.y;
            otosPose.h = pose.h;
            msg::OdometerCommand cmd;
            cmd.setSetPose(otosPose);
            odometerForCmds->apply(cmd);
        }
    } else {
        // No device -- discard rather than let either Mailbox look
        // perpetually "full" to its next post() (Mailbox::post() overwrites
        // anyway, but draining keeps behavior obviously inert either way).
        bb.otosCommandIn.take();
        bb.otosSetPoseIn.take();
    }

    // Config-plane drain: apply EVERY pending Rt::ConfigDelta synchronously,
    // this SAME pass (see this file's header comment and dev_loop.h's own
    // note on why this is not rationed to one-per-pass in this transitional
    // loop).
    while (loop.configurator->pending(bb)) {
        loop.configurator->applyOne(bb);
    }

    if (drivetrain.active() || !bb.driveIn.empty()) {
        // Binding queried fresh -- bb.drivetrainConfig is the Configurator's
        // own published cell, kept in sync by the config-plane drain above.
        uint32_t governedLeft = bb.drivetrainConfig.left_port;
        uint32_t governedRight = bb.drivetrainConfig.right_port;
        drivetrain.tick(now, hardware.motor(governedLeft).state(),
                        hardware.motor(governedRight).state(), bb.driveIn);
        // active() is re-checked AFTER tick() (which may have just popped a
        // standby-only {NONE, standby=true} steal off bb.driveIn -- DEV M's
        // authority-steal, dev_commands.cpp) -- Drivetrain::tick() sets
        // hasCommand() UNCONDITIONALLY whenever it runs (its own doc
        // comment), so a mere steal (posted so THIS pass's governance
        // doesn't fight a bound-port DEV M command) would otherwise still
        // push its now-stale/idle held command out to hardware, clobbering
        // the very port(s) DEV M just addressed via bb.motorIn[] (which
        // slice 2, below, has not drained yet this pass). Only a Drivetrain
        // that is ACTUALLY active after this tick() (a real TWIST/WHEELS/
        // NEUTRAL command, not a bare steal) gets its output pushed to
        // hardware -- matches today's pre-087 contract, where the steal was
        // applied via a direct Drivetrain::apply() call that never invoked
        // tick()'s governance math at all.
        if (drivetrain.active() && drivetrain.hasCommand()) {
            hardware.apply(drivetrain.takeCommand());
        } else if (drivetrain.hasCommand()) {
            drivetrain.takeCommand();   // discard -- standby; nothing should reach hardware
        }
    }

    // Slice 2: whatever request/write this pass's dispatch (or the
    // Drivetrain's own re-governed target) just staged goes out now.
    hardware.tick(now, bb.motorIn, bb.motorResetIn);

    // Commit bb.motor[] a first time here -- a query dispatched mid-pass
    // (before this point) already saw slice-1-fresh state via the same
    // Hardware::state(port) reads a translator makes; this second commit is
    // what pose estimation/the motion executor/telemetry (all later in this
    // same pass) see, matching this pass's freshest (post-slice-2) reads.
    for (uint32_t port = 1; port <= Rt::kPortCount; ++port) {
        bb.motor[port - 1] = hardware.state(port);
    }

    // Pose estimation: ports() queried UNCONDITIONALLY -- pose estimation
    // passively OBSERVES the bound wheels rather than requiring authority
    // over them. leftObs/rightObs are this pass's FRESHEST reads
    // (post-slice-2).
    uint32_t left = bb.drivetrainConfig.left_port;
    uint32_t right = bb.drivetrainConfig.right_port;
    msg::MotorState leftObs = hardware.motor(left).state();
    msg::MotorState rightObs = hardware.motor(right).state();
    Hal::Odometer* odometer = hardware.odometer();
    msg::PoseEstimate sampledPose = {};
    if (odometer != nullptr) {
        odometer->tick(now);
        sampledPose = odometer->pose();
        bb.otos = sampledPose;
        bb.otosValid = true;
    } else {
        bb.otosValid = false;
    }
    loop.poseEstimator->tick(now, leftObs, rightObs, odometer != nullptr ? &sampledPose : nullptr,
                             bb.poseResetIn);

    // Motion executor: drains bb.motionIn (staged by
    // source/commands/motion_commands.cpp's S/T/D/R/TURN/RT/G/STOP
    // handlers) into Planner::apply() -- this file, not a command handler,
    // is the sole per-pass orchestrator.
    bool plannerEngagedThisPass = false;
    if (!bb.motionIn.empty()) {
        Rt::MotionCommand mc = bb.motionIn.take();
        loop.planner->apply(mc.command, now);
        // activeVelocityVerb persists across passes -- updated here exactly
        // when a fresh command is staged (mirrors the pre-087
        // MotionLoopState field's own write sites).
        std::strncpy(loop.activeVelocityVerb, mc.verb, sizeof(loop.activeVelocityVerb) - 1);
        loop.activeVelocityVerb[sizeof(loop.activeVelocityVerb) - 1] = '\0';
        if (mc.feedStreamWatchdog) {
            loop.streamWatchdog.feed(now);
        }
        plannerEngagedThisPass = true;
    }

    // sTimeout: DISTINCT from loop.watchdog (fed by ANY statement). Gating
    // on `mode == STREAMING` alone is not sufficient once a bare `R` also
    // reports STREAMING -- the `activeVelocityVerb[0] == '\0'` check
    // excludes an R-driven session (R's handler never sets
    // feedStreamWatchdog).
    if (loop.planner->state().mode == msg::DriveMode::STREAMING &&
        loop.activeVelocityVerb[0] == '\0' &&
        loop.streamWatchdog.check(now)) {
        msg::PlannerCommand stopCmd;
        stopCmd.setStop(true);
        loop.planner->apply(stopCmd, now);
        plannerEngagedThisPass = true;
        char wbuf[40];
        CommandProcessor::replyEvt(wbuf, sizeof(wbuf), "safety_stop", "reason=watchdog",
                                   loop.serialReply, loop.serialCtx);
    }

    plannerEngagedThisPass = plannerEngagedThisPass || loop.planner->hasActiveCommand();

    // mode is sampled BEFORE tick() -- see motionVerbForMode()'s own doc
    // comment for why.
    msg::DriveMode activeModeBeforeTick = loop.planner->state().mode;
    loop.planner->tick(now, leftObs, rightObs, loop.poseEstimator->fusedPose());
    if (plannerEngagedThisPass && loop.planner->hasCommand()) {
        drivetrain.apply(loop.planner->takeCommand());
    }
    if (loop.planner->hasEvent()) {
        Subsystems::Planner::Event ev = loop.planner->takeEvent();
        char body[64];
        if (ev.corrId[0] != '\0') {
            snprintf(body, sizeof(body), "#%s reason=%s", ev.corrId, ev.reason);
        } else {
            snprintf(body, sizeof(body), "reason=%s", ev.reason);
        }
        char name[16];
        snprintf(name, sizeof(name), "done %s",
                 motionVerbForMode(activeModeBeforeTick, loop.activeVelocityVerb));
        char wbuf[96];
        CommandProcessor::replyEvt(wbuf, sizeof(wbuf), name, body, loop.serialReply,
                                   loop.serialCtx);
    }

    // Commit the remaining observable state cells (motor[] was already
    // re-committed above, before pose estimation) for GET/telemetry/
    // CommandRouter reads next pass (and by the periodic-emission step
    // immediately below, this SAME pass).
    bb.drivetrain = drivetrain.state();
    bb.encoderPose = loop.poseEstimator->encoderPose();
    bb.fusedPose = loop.poseEstimator->fusedPose();
    bb.planner = loop.planner->state();

    // Periodic TLM emission: gated on bb.telemetryPeriod > 0 and enough time
    // having elapsed since the last emission (or none yet).
    if (bb.telemetryPeriod > 0 &&
        (!bb.telemetryHasLastEmit || (now - bb.telemetryLastEmitMs) >= bb.telemetryPeriod)) {
        ReplyFn replyFn = nullptr;
        void* replyCtx = nullptr;
        resolveTelemetryReply(loop, bb.telemetryChannel, &replyFn, &replyCtx);
        telemetryEmit(bb, now, replyFn, replyCtx);
        bb.telemetryLastEmitMs = now;
        bb.telemetryHasLastEmit = true;
    }

    if (loop.watchdog.check(now)) {
        // Applied IMMEDIATELY, not staged via any bb queue -- the loop is
        // the top of the call tree, already the visible mover of every
        // command; an emergency stop gains nothing from an extra pass of
        // queue latency.
        hardware.apply(buildBroadcastNeutral(msg::Neutral::BRAKE));
        drivetrain.apply(buildDrivetrainStop(msg::Neutral::BRAKE));
        char wbuf[32];
        CommandProcessor::replyEvt(wbuf, sizeof(wbuf), "dev_watchdog", nullptr, loop.serialReply,
                                   loop.serialCtx);
    }
}

#endif  // ROBOT_DEV_BUILD
