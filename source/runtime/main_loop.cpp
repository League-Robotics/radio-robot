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

#include <cstring>

#include "commands/command_processor.h"
#include "commands/telemetry_commands.h"
#include "hal/capability/hal_command.h"

namespace Rt {

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

void MainLoop::estop() {
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

namespace {

// motorsRunning -- 091-003 fire-gate predicate (architecture-update.md
// Decision 3): true iff bb's last-committed snapshot shows either the
// Drivetrain's own bound-pair output active, or any individual motor port
// under a non-neutral commanded mode. Reads only bb -- computes nothing
// new. `DrivetrainState.active` alone is NOT sufficient: isBoundPort()'s
// authority-steal in dev_commands.cpp drives it false the instant a
// bound-port DEV M motion verb lands, even though that port keeps
// spinning under the now-standalone command (see msg::MotorState.active's
// own proto doc comment) -- hence the OR across bb.motors[] too.
bool motorsRunning(const Blackboard& bb) {
  if (bb.drivetrain.active) return true;
  for (uint32_t port = 0; port < kPortCount; ++port) {
    if (bb.motors[port].active) return true;
  }
  return false;
}

}  // namespace

void MainLoop::serviceWatchdogs(Blackboard& bb, uint32_t now) {
  // === SAFETY WATCHDOG -- mandatory, first, same-pass deterministic. ===
  // See main_loop.h's file header for why this runs before hardware_.tick().
  // check() is called UNCONDITIONALLY every pass regardless of
  // motorsRunning(bb) -- only the ACTION (estop + EVT) is gated, so
  // SerialSilenceWatchdog's own fire-once/re-arm-on-feed bookkeeping is
  // never skipped (091-003: architecture-update.md Decision 3's "why gate
  // the action, not the check()").
  if (watchdog_.check(now) && motorsRunning(bb)) {
    estop();
    // 090-004: a loop-originated NAMED event -- routed through the SAME
    // emitEvent() a Planner-produced GOAL_DONE event uses (main_loop.h's own
    // "distinct from Rt::CommandRouter's own reply channels" doc comment) --
    // no snprintf/wire text assembled here (CommandProcessor::emitEvent()'s
    // own doc comment owns 100% of the "EVT ..." grammar).
    msg::Event ev;
    ev.kind = msg::Event::Kind::NAMED;
    std::strncpy(ev.name, "dev_watchdog", sizeof(ev.name) - 1);
    CommandProcessor::emitEvent(ev, serialReply_, serialCtx_);
  }

  // The two loop-owned watchdogs' window mailboxes (DEV WD / SET sTimeout=,
  // posted last slack) -- drained directly (neither watchdog is one of the
  // Configurator's four targets), then published for GET/telemetry reads.
  // Drained AFTER the check above, so a window posted last slack governs
  // the NEXT pass's check -- see this method's main_loop.h doc comment.
  if (!bb.devWatchdogWindowIn.empty()) {
    watchdog_.setWindow(bb.devWatchdogWindowIn.take());
  }
  if (!bb.streamWatchdogWindowIn.empty()) {
    streamWatchdog_.setWindow(bb.streamWatchdogWindowIn.take());
  }
  bb.devWatchdogWindow = watchdog_.window();
  bb.streamWatchdogWindow = streamWatchdog_.window();
}

void MainLoop::commit(Blackboard& bb, uint32_t now, bool otosFusableThisPass) {
  // === COMMIT (clock edge): copy each subsystem cell into bb -> x[k+1]. ===
  for (uint32_t port = 1; port <= kPortCount; ++port) {
    bb.motors[port - 1] = hardware_.state(port);
  }
  bb.drivetrain = drivetrain_.state();
  bb.encoderPose = poseEstimator_.encoderPose();
  bb.fusedPose = poseEstimator_.fusedPose();
  bb.planner = planner_.state();
  Hal::Odometer* odometer = hardware_.odometer();
  odometer->tick(now);
  bb.otos = odometer->pose();
  // (090-003) Reuses THIS PASS's one fusableThisPass() call (captured by
  // tick(), at the poseEstimator_.tick() read site, and threaded through
  // here as a parameter) rather than calling it again -- see that call
  // site's own comment for why a second call would be wrong.
  // A NullOdometer's override always returns false, so bb.otosValid
  // collapses to "false" exactly when there is no device, folding in the
  // same fact the old `!= nullptr` branch encoded, without a pointer check.
  bb.otosValid = otosFusableThisPass;
}

void MainLoop::tick(Blackboard& bb, uint32_t now) {
  // SAFETY WATCHDOG check + estop, then watchdog-window config upkeep --
  // FIRST, before hardware_.tick(), per main_loop.h's file header.
  serviceWatchdogs(bb, now);

  // === MANDATORY: control. Reads bb (x[k]); consumes commands routed
  //     during the previous slack; each subsystem writes its OWN cell. ===
  hardware_.tick(now, bb.motorIn, bb.motorResetIn);

  // DEV STOP's broadcast neutral (posted last slack) -- deliberately NOT
  // bb.motorIn[] (a broadcast needs the allPorts=true
  // Hal::CommandProcessorToHardwareCommand shape, applied through
  // Hardware::apply() below -- a structurally different distribution path
  // than bb.motorIn[]'s per-port drain; see blackboard.h's file header).
  if (!bb.hardwareBroadcastIn.empty()) {
    msg::MotorCommand neutral = bb.hardwareBroadcastIn.take();
    Hal::CommandProcessorToHardwareCommand broadcast;
    broadcast.allPorts = true;
    broadcast.count = 0;
    broadcast.addressed[0].command = neutral;
    hardware_.apply(broadcast);
  }

  Subsystems::DrivetrainPorts p = drivetrain_.ports();   // bound pair, from config

  // 090-001: the loop does no port-cell indexing of its own -- it passes
  // the whole committed per-port observation array (bb.motors) and lets
  // Drivetrain resolve its own bound pair (ports()) internally, with its own
  // range assert. `p` above is still needed for THIS function's other two
  // call sites below (poseEstimator_.tick()/planner_.tick()).
  drivetrain_.tick(now, bb.motors, kPortCount, bb.driveIn);

  // Odometer one-shot actions (OI/OZ/OR/OV, SI's re-anchor) -- the loop
  // legitimately holds Hardware& (composition-root status, same as
  // Rt::Configurator's own exception -- Decision 4); Hal::Odometer has no
  // tick()-driven queue parameter of its own. The loop still drains and
  // applies these -- it legitimately knows a reset happened because it just
  // applied one -- but (090-002) the SetPose -> Pose2D -> OdometerCommand
  // translation and the "is OTOS fusable this pass" decision now live on the
  // odometer itself (Hal::Odometer::applySetPose()/fusableThisPass()) rather
  // than as loop-local plumbing.
  //
  // (090-003) hardware_.odometer() is NEVER null (Hal::NullOdometer default,
  // subsystems/hardware.h) -- both drain arms below now run unconditionally;
  // the former "no device" else-arm (which existed only to keep either
  // Mailbox from looking perpetually "full" to its next post()) is
  // redundant now: a NullOdometer's apply()/applySetPose() already discard
  // inertly, and the mailboxes are drained by the `if (!empty())` guards
  // below either way.
  Hal::Odometer* odometer = hardware_.odometer();
  if (!bb.otosCommandIn.empty()) {
    odometer->apply(bb.otosCommandIn.take());
  }
  if (!bb.otosSetPoseIn.empty()) {
    odometer->applySetPose(bb.otosSetPoseIn.take());
  }

  // Pose estimation reads bb.otos as committed LAST pass (x[k]) -- this
  // pass's fresh sample is taken during COMMIT, below (Decision 6: no
  // same-pass read of a value this SAME pass will refresh) -- EXCEPT on the
  // exact pass a reset was JUST applied above, per Hal::Odometer::
  // fusableThisPass()'s own doc comment: bb.otos is an x[k] cell refreshed
  // only at COMMIT, so on a reset pass it still holds the STALE, pre-reset
  // reading, and fusing it against the freshly setPose'd EKF would fabricate
  // a large false innovation (reproduced live via SI -- see
  // fusableThisPass()'s doc comment for the full history).
  //
  // (090-003) This is fusableThisPass()'s ONE sanctioned call site this pass
  // -- see its own doc comment for why it must never be polled twice. The
  // result is captured ONCE, here, and reused verbatim at COMMIT below for
  // bb.otosValid rather than calling fusableThisPass() a second time. This
  // also replaces the former `bb.otosValid &&` short-circuit guard at this
  // read site -- that guard existed only to avoid dereferencing a null
  // odometer (dead code even before this ticket, since both concrete owners
  // already override odometer() to non-null), never as a deliberate
  // "otos not populated yet" check -- Subsystems::PoseEstimator::tick()
  // already re-checks `otosObs->stamp.valid` internally
  // (source/subsystems/pose_estimator.cpp) before actually fusing, so
  // passing `&bb.otos` here whenever fusableThisPass() is true is safe even
  // on the very first pass, before COMMIT has ever populated bb.otos (its
  // default-constructed stamp.valid is already false).
  bool otosFusableThisPass = odometer->fusableThisPass();
  poseEstimator_.tick(now, bb.motors[p.left - 1], bb.motors[p.right - 1],
                      otosFusableThisPass ? &bb.otos : nullptr,
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
    // 090-004: loop-originated NAMED event -- see the dev_watchdog site
    // above for why this is zero wire-text assembly, routed through the
    // SAME emitEvent() a Planner-produced GOAL_DONE event uses.
    msg::Event ev;
    ev.kind = msg::Event::Kind::NAMED;
    std::strncpy(ev.name, "safety_stop", sizeof(ev.name) - 1);
    std::strncpy(ev.reason, "watchdog", sizeof(ev.reason) - 1);
    CommandProcessor::emitEvent(ev, serialReply_, serialCtx_);
  }

  plannerEngagedThisPass = plannerEngagedThisPass || planner_.hasActiveCommand();

  planner_.tick(now, bb.motors[p.left - 1], bb.motors[p.right - 1], bb.fusedPose);
  if (planner_.hasEvent()) {
    // 090-004: Planner's own event is ALREADY a fully-formed msg::Event
    // (kind = GOAL_DONE, verb/reason/corrId all resolved by Planner itself
    // -- see planner.h's hasEvent()/takeEvent() doc comment) -- the loop
    // only routes it to emitEvent(), the SAME wire-layer authority the
    // loop's own NAMED events above use. Zero EVT formatting here.
    CommandProcessor::emitEvent(planner_.takeEvent(), serialReply_, serialCtx_);
  }

  // === COMMIT (clock edge): x[k] -> x[k+1] -- see commit()'s own doc
  //     comment (main_loop.h) for exactly what it copies and why
  //     otosFusableThisPass is threaded in rather than recomputed. ===
  commit(bb, now, otosFusableThisPass);

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
