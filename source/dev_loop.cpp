// dev_loop.cpp -- devLoopTick(): see dev_loop.h for the full rationale.
//
// This is main.cpp's PRE-081-002 loop body, copied verbatim, with exactly
// two adaptations (both documented at their point of use below):
//   1. The statement comes from the `statement` parameter (a DevLoopStatement*
//      the caller already built) instead of reading directly from a
//      Subsystems::Communicator.
//   2. The watchdog-fire `EVT dev_watchdog` reply goes out via
//      `loop.defaultReply`/`loop.defaultReplyCtx` instead of the hardcoded
//      `serialReply`/`&comm` main.cpp used before this extraction.
// Every other line -- the two hardware.tick(now) slices, the outbox drain,
// the Drivetrain governance, and the watchdog check's neutralize-and-emit
// sequence -- is unchanged.
#include "dev_loop.h"

#if ROBOT_DEV_BUILD

#include <cstdio>

namespace {

// motionVerbForMode -- maps the msg::DriveMode a Planner goal was driving to
// its wire verb, for "EVT done <verb> ..." text (084-002). Sampled from
// Planner::state().mode BEFORE calling tick() each pass, since a goal that
// completes THIS pass transitions mode_ back to IDLE INSIDE that same
// tick() call (planner.cpp) -- reading state() after tick() would always
// see IDLE for a just-completed goal.
//
// Scoped to this ticket's S/T/D verbs only: STREAMING/TIMED/DISTANCE each
// uniquely identify their own verb. GO_TO/VELOCITY are deliberately
// unmapped (empty string) -- ticket 084-003/004's R/TURN/RT/G all map onto
// DriveMode::VELOCITY or GO_TO in planner.cpp's own apply() (e.g. TURN and
// ROTATION both stage DriveMode::VELOCITY), so DriveMode alone cannot
// disambiguate them; those tickets will need a different mechanism (e.g.
// tracking the actual verb string, not just the DriveMode) for their own
// EVT text, not an extension of this switch.
const char* motionVerbForMode(msg::DriveMode mode) {
  switch (mode) {
    case msg::DriveMode::STREAMING: return "S";
    case msg::DriveMode::TIMED: return "T";
    case msg::DriveMode::DISTANCE: return "D";
    default: return "";
  }
}

}  // namespace

void devLoopTick(DevLoop& loop, uint32_t now, const DevLoopStatement* statement) {
    Subsystems::Hardware& hardware = *loop.hardware;
    Subsystems::Drivetrain& drivetrain = *loop.drivetrain;
    DevLoopState& devState = *loop.devState;

    hardware.tick(now);   // slice 1: any due collect lands before this pass's dispatch reads state

    if (statement != nullptr) {
        // Feed on any statement, regardless of content or dispatch outcome --
        // see dev_commands.h's watchdog contract. (Adaptation 1: statement
        // comes from the parameter, not directly from a Communicator.)
        loop.watchdog->feed(now);
        // Parse happens inside process(): replies go out the statement's own
        // return path directly (unaffected by this ticket); setpoint-shaped
        // DEV commands land in devState's outbox instead of calling
        // Hal/Drivetrain write methods -- see dev_commands.h/.cpp's
        // pure-transformer reshape.
        loop.processor->process(statement->line, statement->replyFn, statement->replyCtx);
    }

    // Drain the outbox: the caller (main.cpp; a future sim_api.cpp) is the
    // sole caller of hardware.apply()/drivetrain.apply() for anything
    // DEV-sourced.
    if (devState.hasHardwareCommand) {
        hardware.apply(devState.hardwareCommand);
        devState.hasHardwareCommand = false;
    }
    if (devState.hasDrivetrainCommand) {
        drivetrain.apply(devState.drivetrainCommand);
        devState.hasDrivetrainCommand = false;
    }

    if (drivetrain.active()) {
        // Binding queried, not duplicated -- ports() reads straight from
        // DrivetrainConfig (sprint 079 decision 8; DevLoopState no longer
        // holds its own leftPort/rightPort copy).
        Subsystems::DrivetrainPorts p = drivetrain.ports();
        drivetrain.tick(now, hardware.motor(p.left).state(), hardware.motor(p.right).state());
        if (drivetrain.hasCommand()) {
            hardware.apply(drivetrain.takeCommand());
        }
    }

    // Slice 2: whatever request/write this pass's dispatch (or the
    // Drivetrain's own re-governed target) just staged goes out now -- the
    // sanctioned second hardware.tick() call (architecture-update.md (079)
    // decision 6).
    hardware.tick(now);

    // Pose estimation (082-003; dev_loop.h's own doc comment has the full
    // rationale): ports() is queried UNCONDITIONALLY -- unlike the
    // drivetrain.active() governance block above, pose estimation is a
    // passive observer of whatever the bound wheels are doing, never an
    // authority-gated actor. leftObs/rightObs are this pass's FRESHEST
    // reads (post-slice-2). hardware.odometer() is nullptr for
    // Subsystems::NezhaHardware (no real-hardware OTOS driver this sprint)
    // and non-null for Subsystems::SimHardware -- when non-null, its
    // tick(now) is called and its pose() sampled before
    // Subsystems::PoseEstimator::tick() runs, so the fresh sample is ready
    // the same pass it was produced. Exactly ONE loop.poseEstimator->tick()
    // call follows, unconditionally -- the single most important
    // correctness property in this step (see dev_loop_pose_estimator_harness.cpp).
    Subsystems::DrivetrainPorts p = drivetrain.ports();
    msg::MotorState leftObs = hardware.motor(p.left).state();
    msg::MotorState rightObs = hardware.motor(p.right).state();
    Hal::Odometer* odometer = hardware.odometer();
    msg::PoseEstimate sampledPose = {};
    if (odometer != nullptr) {
        odometer->tick(now);
        sampledPose = odometer->pose();
    }
    loop.poseEstimator->tick(now, leftObs, rightObs, odometer != nullptr ? &sampledPose : nullptr);

    // Motion executor (084-002; dev_loop.h's own doc comment has the full
    // rationale). Placed AFTER pose estimation (needs loop.poseEstimator->
    // fusedPose(), just produced above) and BEFORE periodic TLM emission
    // (so a mode/authority change this pass is reflected in the SAME pass's
    // telemetry, once ticket 005 reads Planner::state().mode there).
    MotionLoopState& motionState = *loop.motionState;

    // plannerEngagedThisPass -- gates the drivetrain.apply() drain below.
    // Planner::tick() unconditionally HOLDS a twist command every pass, even
    // while fully idle (a (0,0,0) hold -- see planner.cpp's holdTwistCommand()
    // call at the end of tick()), so draining hasCommand()/takeCommand() into
    // drivetrain.apply() UNCONDITIONALLY every pass would call
    // Drivetrain::setTwist() -- which ALWAYS (re)activates authority, even
    // for a zero twist -- forever after the very first devLoopTick() call,
    // regardless of whether any S/T/D/STOP was ever issued. That would
    // permanently steal Drivetrain's authority away from DEV DT/DEV M with
    // no wire command ever asking for it, breaking `mode=I` at rest (082-004)
    // and any DEV-driven test. The gate is true exactly when Planner has
    // something ACTUAL to say this pass: a fresh command was just staged
    // (S/T/D/STOP, including the sTimeout-fired synthetic STOP below), or a
    // goal was already active going into this tick (still running, or
    // completing on this very tick -- hasActiveCommand() only flips false
    // INSIDE tick()/apply(), so sampling it here, before tick() runs, still
    // catches the final pass). Once a goal goes fully idle with no further
    // command, this stays false and Planner's held zero-twist is simply
    // never drained again -- Drivetrain's authority is left exactly where
    // the last REAL drain (a running goal's last twist, or an explicit
    // STOP's zero twist) put it, matching Open Question 3's "whichever last
    // issued a command wins" contract.
    bool plannerEngagedThisPass = false;

    // Drain motionState's outbox (staged by source/commands/
    // motion_commands.cpp's S/T/D/STOP handlers) into Planner::apply() --
    // this file, not a command handler, is the sole per-pass orchestrator,
    // mirroring DevLoopState's own outbox-drain discipline above.
    if (motionState.hasCommand) {
        loop.planner->apply(motionState.command, now);
        motionState.hasCommand = false;
        plannerEngagedThisPass = true;
    }

    // sTimeout (084-002): the streaming-drive watchdog, DISTINCT from
    // loop.watchdog (SerialSilenceWatchdog, fed by ANY statement regardless
    // of content) -- sTimeout is fed ONLY by S's own handler and only
    // matters while a STREAMING goal is the one actually active; gating the
    // check on that mode is what "armed" means here (no separate bool is
    // needed -- a fresh S always re-feeds it on (re)entry to STREAMING, and
    // a later T/D/STOP simply stops this check from ever running again
    // until the next S). Firing applies a silent STOP directly to the
    // Planner (never through its stops_[]/Event mechanism -- that would
    // misreport reason=time instead of reason=watchdog) and emits the EVT
    // itself, on the loop-originated reply sink (mirrors the DEV WD
    // watchdog-fire path below: this EVT was not triggered by any inbound
    // statement, so it has no per-command replyFn/replyCtx to reuse).
    if (loop.planner->state().mode == msg::DriveMode::STREAMING && motionState.sTimeout.check(now)) {
        msg::PlannerCommand stopCmd;
        stopCmd.setStop(true);
        loop.planner->apply(stopCmd, now);
        plannerEngagedThisPass = true;
        char wbuf[40];
        CommandProcessor::replyEvt(wbuf, sizeof(wbuf), "safety_stop", "reason=watchdog",
                                   loop.defaultReply, loop.defaultReplyCtx);
    }

    plannerEngagedThisPass = plannerEngagedThisPass || loop.planner->hasActiveCommand();

    // mode is sampled BEFORE tick() -- see motionVerbForMode()'s own doc
    // comment for why (a goal completing THIS pass transitions mode_ to
    // IDLE inside tick() itself). tick() itself still runs unconditionally
    // every pass, mirroring PoseEstimator::tick()'s own always-run contract
    // (planner.h's tick() doc comment: "the caller does not gate this on
    // hasActiveCommand()") -- only the DRAIN into drivetrain.apply() below
    // is gated, not the tick() call itself.
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
        snprintf(name, sizeof(name), "done %s", motionVerbForMode(activeModeBeforeTick));
        char wbuf[96];
        CommandProcessor::replyEvt(wbuf, sizeof(wbuf), name, body, loop.defaultReply,
                                   loop.defaultReplyCtx);
    }

    // Periodic TLM emission (082-004; dev_loop.h's own doc comment has the
    // full rationale). Gated on periodMs > 0 (STREAM 0 disables this
    // entirely) and on enough time having elapsed since the last emission --
    // or no emission having happened yet (hasLastEmit), so the very first
    // pass after a channel issues STREAM emits immediately rather than
    // waiting a full period. telemetryEmit() itself no-ops on a null
    // replyFn (no channel has ever issued STREAM), so this is safe to call
    // unconditionally once the time gate opens.
    TelemetryState& telemetry = *loop.telemetry;
    if (telemetry.periodMs > 0 &&
        (!telemetry.hasLastEmit ||
         (now - telemetry.lastEmitMs) >= telemetry.periodMs)) {
        telemetryEmit(telemetry, now, telemetry.replyFn, telemetry.replyCtx);
        telemetry.lastEmitMs = now;
        telemetry.hasLastEmit = true;
    }

    if (loop.watchdog->check(now)) {
        // Applied IMMEDIATELY, not staged via the outbox -- the caller is
        // the top of the call tree, already the visible mover of every
        // command; an emergency stop gains nothing from an extra pass of
        // outbox latency (architecture-update.md's narrow, deliberate
        // exception to "never call apply() outside main/the HAL"). The SAME
        // buildBroadcastNeutral()/buildDrivetrainStop() construction path
        // `DEV STOP`'s handler stages is used here directly.
        hardware.apply(buildBroadcastNeutral(msg::Neutral::BRAKE));
        drivetrain.apply(buildDrivetrainStop(msg::Neutral::BRAKE));
        char wbuf[32];
        // Adaptation 2: the loop-originated default reply sink
        // (loop.defaultReply/loop.defaultReplyCtx) replaces main.cpp's
        // hardcoded serialReply/&comm -- see dev_loop.h's file header and
        // architecture-update.md Decision 3.
        CommandProcessor::replyEvt(wbuf, sizeof(wbuf), "dev_watchdog", nullptr,
                                   loop.defaultReply, loop.defaultReplyCtx);
    }
}

#endif  // ROBOT_DEV_BUILD
