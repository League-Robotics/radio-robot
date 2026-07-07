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

#include "runtime/queue.h"

namespace {

// motionVerbForMode -- maps the msg::DriveMode a Planner goal was driving to
// its wire verb, for "EVT done <verb> ..." text (084-002). Sampled from
// Planner::state().mode BEFORE calling tick() each pass, since a goal that
// completes THIS pass transitions mode_ back to IDLE INSIDE that same
// tick() call (planner.cpp) -- reading state() after tick() would always
// see IDLE for a just-completed goal.
//
// DISTANCE/GO_TO each still uniquely identify their own verb (D/G). STREAMING
// and TIMED, as of 084-005's Decision 6 (planner.cpp's velocityShapedMode()),
// are each now shared by MORE than one verb: STREAMING is `S` or a bare `R`
// (no stop=); TIMED is `T`, a stop=-bearing `R`, `TURN`, or `RT`. DriveMode
// alone cannot disambiguate any of these -- `activeVelocityVerb`
// (MotionLoopState, set by handleR/handleTURN/handleRT, and CLEARED by
// handleS/handleT/handleD/handleG -- see motion_commands.h's field doc
// comment) is the disambiguation mechanism: non-empty means the active goal
// was staged by R/TURN/RT, so it names the actual wire verb; empty falls
// back to the mode's own plain verb (S/T). `VELOCITY` itself is no longer
// ever emitted by planner.cpp (see velocityShapedMode()'s doc comment) --
// this switch keeps a defensive case for it rather than assuming that
// invariant holds forever.
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
        // 087-003: Drivetrain::tick() gained a driveIn Mailbox parameter
        // (source/runtime/queue.h; see drivetrain.h's tick() doc comment)
        // that it drains before running its setpoint-governance path.
        // dev_loop.cpp still stages setpoints via the direct
        // drivetrain.apply(devState.drivetrainCommand) call above (its own
        // outbox-drain mechanism, unaffected by this ticket) -- it has no
        // Blackboard-backed driveIn to source a real one from yet (that
        // wiring is ticket 007's job, which also deletes this whole file --
        // see dev_loop.h's file header). An always-empty local Mailbox here
        // is a mechanical compile fix only: it stays empty, so tick() takes
        // the "no new command" branch and governs the setpoint already
        // applied above, unchanged.
        Rt::Mailbox<msg::DrivetrainCommand> noDriveInYet;
        drivetrain.tick(now, hardware.motor(p.left).state(), hardware.motor(p.right).state(),
                         noDriveInYet);
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
    // matters while a STREAMING goal S/VW itself staged is the one actually
    // active; a later T/D/STOP simply stops this check from ever running
    // again until the next S.
    //
    // Gating on `mode == STREAMING` ALONE stopped being sufficient once
    // 084-005's Decision 6 (planner.cpp's velocityShapedMode()) made a bare
    // `R` (no stop=) ALSO report DriveMode::STREAMING: `R`'s own ticket
    // (084-003) acceptance is "no stop of its own" -- it must run
    // open-ended until an explicit STOP or its own stop= clause, NEVER
    // subject to S's sTimeout (which R's handler never feeds). The extra
    // `activeVelocityVerb[0] == '\0'` check restores that scoping: `handleS`
    // clears activeVelocityVerb when it stages a goal (motion_commands.cpp),
    // so a genuine S/VW-driven STREAMING session always has it empty, while
    // a bare-R-driven one always has it set to "R" -- excluding exactly the
    // one case this watchdog must never fire for.
    if (loop.planner->state().mode == msg::DriveMode::STREAMING &&
        motionState.activeVelocityVerb[0] == '\0' &&
        motionState.sTimeout.check(now)) {
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
        snprintf(name, sizeof(name), "done %s",
                 motionVerbForMode(activeModeBeforeTick, motionState.activeVelocityVerb));
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
