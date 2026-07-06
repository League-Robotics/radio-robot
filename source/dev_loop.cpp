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
