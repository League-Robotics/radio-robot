// drivetrain.cpp -- Subsystems::Drivetrain implementation. See drivetrain.h
// for the class-level design notes (the 100-007 THE CUTOVER rewrite into
// the thin wafer adapter over source/drive/).
#include "subsystems/drivetrain.h"

#include <cassert>
#include <cmath>
#include <new>

#include "kinematics/body_kinematics.h"
#include "subsystems/drive_bridge.h"

namespace Subsystems {

Drivetrain::Drivetrain(Hardware& hardware) : hardware_(hardware) {}

void Drivetrain::setTwist(float v_x, float /*v_y*/, float omega) {
    // v_y is accepted for signature symmetry but never consumed: the
    // differential drivetrain is non-holonomic -- see commandedWheelTargets().
    mode_ = Mode::TWIST;
    v_x_ = v_x;
    omega_ = omega;
    active_ = true;
}

void Drivetrain::setWheelTargets(float left, float right) {
    mode_ = Mode::WHEELS;
    wheelTargetLeft_ = left;
    wheelTargetRight_ = right;
    active_ = true;
}

void Drivetrain::setNeutral(msg::Neutral mode) {
    mode_ = Mode::NEUTRAL;
    neutralMode_ = mode;
    active_ = true;
}

void Drivetrain::configure(const msg::DrivetrainConfig& config) {
    config_ = config;
    // THE single conversion point (0-based motor indices): config.
    // left_port/right_port are wire/serialized 1-based brick labels,
    // converted to 0-based Hardware motor indices here, exactly once.
    boundLeft_ = config.left_port - 1;
    boundRight_ = config.right_port - 1;
    rebuildDriveDrivetrain();
}

void Drivetrain::configureMotion(const msg::PlannerConfig& config) {
    plannerConfig_ = config;
    rebuildDriveDrivetrain();
}

void Drivetrain::rebuildDriveDrivetrain() {
    // Drive::Drivetrain IS copy-assignable (unlike Drive::MotionPlan --
    // Drivetrain itself holds no ruckig:: member, only a Limits value and a
    // trackwidth float, both trivially assignable) -- a plain assignment is
    // correct and sufficient here, no placement-new needed.
    driveDrivetrain_ = Drive::Drivetrain(driveLimitsFromConfig(plannerConfig_), config_.trackwidth);
}

void Drivetrain::apply(const msg::DrivetrainCommand& command) {
    switch (command.control_kind) {
        case msg::DrivetrainCommand::ControlKind::TWIST: {
            const msg::BodyTwist3& twist = command.control.twist;
            setTwist(twist.v_x, twist.v_y, twist.omega);
            break;
        }
        case msg::DrivetrainCommand::ControlKind::WHEELS: {
            const msg::WheelTargets& wheels = command.control.wheels;
            float left = 0.0f;
            float right = 0.0f;
            if (wheels.w_count_val() > 0 && wheels.w()[0].speed.has) {
                left = wheels.w()[0].speed.val;
            }
            if (wheels.w_count_val() > 1 && wheels.w()[1].speed.has) {
                right = wheels.w()[1].speed.val;
            }
            setWheelTargets(left, right);
            break;
        }
        case msg::DrivetrainCommand::ControlKind::NEUTRAL:
            setNeutral(command.control.neutral);
            break;
        case msg::DrivetrainCommand::ControlKind::POSE:
            // Ignored this sprint -- documented no-op, not a silent drop.
            break;
        case msg::DrivetrainCommand::ControlKind::NONE:
        default:
            break;
    }

    if (command.standby.has && command.standby.val) {
        standby();
    }
}

void Drivetrain::clearRing() {
    while (!ring_.empty()) {
        ring_.take();
    }
}

void Drivetrain::replacePlan(const Drive::MotionPlan& newPlan) {
    // Drive::MotionPlan is copy-CONSTRUCTIBLE but NOT copy-ASSIGNABLE
    // (master_profile.h's own ruckig::Ruckig<1> const members) -- placement
    // new is the no-heap "reassignment" idiom (see drivetrain.h's own class
    // comment). Well-defined even the FIRST time this is called: plan_'s
    // default constructor already ran at Drivetrain construction, so there
    // is always a live object here to destroy first.
    plan_.~MotionPlan();
    new (&plan_) Drive::MotionPlan(newPlan);
}

bool Drivetrain::dispatchEscapeHatch(const msg::DrivetrainCommand& command) {
    using Kind = msg::DrivetrainCommand::ControlKind;
    bool preempted = (command.control_kind == Kind::TWIST ||
                      command.control_kind == Kind::WHEELS ||
                      command.control_kind == Kind::NEUTRAL);

    if (preempted) {
        // (100-007) Instant preemption for every arm, including NEUTRAL
        // while a plan is in flight -- see drivetrain.h's own class comment
        // for the documented deviation from the pre-cutover graceful
        // decel-to-zero (Motion::SegmentExecutor::stop()) this replaces:
        // that mechanism has no source/drive/ equivalent in this ticket's
        // scope (planVelocity()-based decel is ticket 100-008's MOVER job).
        clearRing();
        planActive_ = false;
        haveAnchor_ = false;
        segmentMode_ = false;
    }

    apply(command);
    return preempted;
}

void Drivetrain::startNextPlan(uint32_t now, const msg::PoseEstimate& bodyState) {
    while (!ring_.empty()) {
        const Drive::Goal goal = ring_.take();
        const Drive::Pose start =
            haveAnchor_ ? plan_.goal() : driveBodyState(bodyState).pose;

        Drive::PlanRequest request;
        request.goal = goal;
        request.start = start;
        request.entrySpeed = nextEntrySpeed_;
        request.entryAccel = 0.0f;

        const Drive::PlanResult result = driveDrivetrain_.plan(request);
        if (result.verdict != Drive::Verdict::OK) {
            // Late plan() failure: admit()'s coarse, conservative
            // queue-time check (BinaryChannel, wire time) passed, but the
            // exact Ruckig solve did not -- rare (both checks fold the SAME
            // v_eff ceiling math), and there is no live wire corr_id left
            // to reply an ERR to any more (the segment's own ACK already
            // happened at admission time). Drop this ring entry and try
            // the next one, same tick -- diagnostic counter only, no
            // EventNotify (this is not one of the Drive::Status ABORT_*
            // cases the ticket's own EventNotify contract covers).
            ++lateSolveFailures_;
            continue;
        }

        replacePlan(result.plan);
        planStart_ = now;
        // A freshly popped ring entry is a genuinely NEW segment -- unlike
        // a same-segment REPLAN_DUE (which preserves state_ across
        // Drivetrain::replan(), see tick()'s own comment), starting the
        // NEXT queued goal resets the policy-timer history.
        state_ = Drive::StepState{};
        planActive_ = true;
        haveAnchor_ = true;
        nextEntrySpeed_ = 0.0f;
        ++segSeq_;
        return;
    }
}

void Drivetrain::abortAndFlush(Drive::ChainTail& chainTail, Drive::Status status,
                                const Drive::BodyState& measured,
                                const Drive::TrackRecord& record) {
    clearRing();
    planActive_ = false;
    haveAnchor_ = false;

    // Re-anchor ChainTail to the current measured pose -- whatever the wire
    // handler had predicted for the (now-flushed) queue is moot.
    chainTail = Drive::ChainTail{measured.pose, 0.0f, 0.0f};

    lastEvent_.seg_seq = segSeq_;
    lastEvent_.status = toMotionStatus(status);
    lastEvent_.e_final_pos = std::sqrt(record.eAlong * record.eAlong + record.eCross * record.eCross);
    lastEvent_.e_final_theta = record.eTheta;
}

void Drivetrain::commandedWheelTargets(float* targetLeft, float* targetRight) const {
    switch (mode_) {
        case Mode::TWIST:
            BodyKinematics::inverse(v_x_, omega_, config_.trackwidth,
                                     *targetLeft, *targetRight);
            break;
        case Mode::WHEELS:
            *targetLeft = wheelTargetLeft_;
            *targetRight = wheelTargetRight_;
            break;
        case Mode::NEUTRAL:
        default:
            *targetLeft = 0.0f;
            *targetRight = 0.0f;
            break;
    }
}

void Drivetrain::governRatio(float* targetLeft, float* targetRight,
                              const msg::MotorState& leftObs,
                              const msg::MotorState& rightObs) const {
    if (config_.sync_gain <= 0.0f) return;

    if (*targetLeft == 0.0f || *targetRight == 0.0f) return;
    if ((*targetLeft) * (*targetRight) <= 0.0f) return;

    if (!leftObs.velocity.has || !rightObs.velocity.has) return;

    float achievedLeft = leftObs.velocity.val / *targetLeft;
    float achievedRight = rightObs.velocity.val / *targetRight;
    float achievedMin = (achievedLeft < achievedRight) ? achievedLeft : achievedRight;

    if (achievedMin >= 1.0f) return;
    if (achievedMin < 0.0f) achievedMin = 0.0f;

    float scale = 1.0f - config_.sync_gain * (1.0f - achievedMin);
    if (scale < 0.0f) scale = 0.0f;
    *targetLeft *= scale;
    *targetRight *= scale;
}

void Drivetrain::tick(uint32_t now,
                       Rt::WorkQueue<Drive::Goal, 8>& segmentIn,
                       Rt::Mailbox<Rt::MoverRequest>& replaceIn,
                       Rt::WorkQueue<msg::DrivetrainCommand, 8>& driveIn,
                       const msg::PoseEstimate& bodyState,
                       const msg::PoseStep& poseStepped,
                       Drive::ChainTail& chainTail) {
    // 1. driveIn drained FIRST -- unchanged escape-hatch precedence.
    bool preempted = false;
    if (!driveIn.empty()) {
        preempted = dispatchEscapeHatch(driveIn.take());
    }

    // 1b. replaceIn -- MOVER (100-008): a fresh Rt::MoverRequest replaces
    // the held plan (latest-wins, Mailbox semantics -- no new queueing).
    // clearRing() unconditionally: a fresh MOVER always retires any queued
    // arc/pivot Goal, regardless of whether THIS planVelocity() call itself
    // succeeds (matches the dispatchEscapeHatch() preemption above, and
    // AC2's own "no new queueing behavior introduced"). planVelocity() has
    // no pose goal, so -- unlike startNextPlan()'s Goal-based plan() call --
    // this never reads/writes chainTail/haveAnchor_ as a chaining anchor.
    if (!preempted && !replaceIn.empty()) {
        const Rt::MoverRequest request = replaceIn.take();
        clearRing();
        const Drive::PlanResult result = driveDrivetrain_.planVelocity(
            request.target, request.deadman, driveBodyState(bodyState));
        if (result.verdict == Drive::Verdict::OK) {
            replacePlan(result.plan);
            planStart_ = now;
            state_ = Drive::StepState{};
            planActive_ = true;
            haveAnchor_ = false;  // velocity-mode plan has no chainable anchor pose
            nextEntrySpeed_ = 0.0f;
            ++segSeq_;
            segmentMode_ = true;
        }
        // else SOLVE_FAILED/CEILING_INFEASIBLE (e.g. a misconfigured
        // trimVMax >= v_body_max leaving no linear-channel headroom): the
        // ring is already cleared above, but plan_/planActive_/segmentMode_
        // are left untouched -- mirrors replan()'s own "the caller keeps
        // the old plan" contract (this same tick()'s REPLAN_DUE handling
        // below), rather than an unconditional stop for a single rejected
        // refresh.
    }

    // 2. Otherwise, drain segmentIn IN FULL into ring_ this tick.
    if (!preempted) {
        bool queuedAny = false;
        while (!segmentIn.empty()) {
            if (ring_.post(segmentIn.take())) {
                queuedAny = true;
            }
        }
        if (queuedAny) {
            segmentMode_ = true;
        }
    }

    DrivetrainPorts bound = ports();
    assert(bound.left < Hardware::kMotorCount);
    assert(bound.right < Hardware::kMotorCount);
    const msg::MotorState leftObs = hardware_.motorState(bound.left);
    const msg::MotorState rightObs = hardware_.motorState(bound.right);

    updateAccelEma(now, 0, leftObs);
    updateAccelEma(now, 1, rightObs);

    msg::MotorCommand leftCmd;
    msg::MotorCommand rightCmd;
    bool neutral = false;
    float targetLeft = 0.0f;
    float targetRight = 0.0f;

    if (segmentMode_) {
        if (!planActive_ && !ring_.empty()) {
            startNextPlan(now, bodyState);
        }

        if (planActive_) {
            const float elapsed =
                static_cast<float>(static_cast<int32_t>(now - planStart_)) * 0.001f;

            Drive::StepInput in;
            in.t = elapsed;
            in.measured = driveBodyState(bodyState);
            in.left = driveWheelState(leftObs);
            in.right = driveWheelState(rightObs);
            in.poseStep = poseStepped.pos;
            in.poseStepTheta = poseStepped.theta;

            const Drive::StepOutput out = plan_.step(in, &state_);
            targetLeft = out.command.left;
            targetRight = out.command.right;

            // lastRecord_ (100-009) -- captured EVERY pass step() runs,
            // regardless of out.status (RUNNING/SETTLING/REPLAN_DUE/
            // DONE_*/ABORT_*) -- MainLoop::commit() publishes this to
            // bb.motionTrace unconditionally every pass, mirroring
            // lastEvent_'s own "last-known value" semantics (never reset to
            // a default between passes; simply overwritten by the next
            // capture).
            lastRecord_ = Subsystems::driveMotionTrace(out.record, segSeq_);

            // Remaining master-DOF distance [mm] -- state()/rem=. 0 for a
            // pivot (the master DOF there is heading, radians, not a
            // translation -- see drivetrain.h's own field comment).
            remainingLinear_ =
                plan_.isPivot()
                    ? 0.0f
                    : fabsf(plan_.referenceAt(plan_.duration()).s - plan_.referenceAt(elapsed).s);

            switch (out.status) {
                case Drive::Status::REPLAN_DUE: {
                    const Drive::PlanResult result =
                        driveDrivetrain_.replan(plan_, in.measured, elapsed);
                    if (result.verdict == Drive::Verdict::OK) {
                        replacePlan(result.plan);
                        planStart_ = now;
                        // state_ intentionally NOT reset -- policy.cpp's
                        // attemptReplan() already reset sustainStart/
                        // dwellStart/settling as part of producing THIS
                        // tick's REPLAN_DUE status; replanCount/lastReplan
                        // must persist across the replan (the rate-limit/
                        // N-max history is for the whole segment).
                    }
                    // else SOLVE_FAILED: the caller keeps the OLD plan_/
                    // state_ (drivetrain.cpp's own replan() doc comment) --
                    // out.command (already computed above, against the
                    // pre-replan plan_) is still this tick's correct
                    // staged output.
                    break;
                }
                case Drive::Status::DONE_STOP:
                case Drive::Status::DONE_HANDOFF:
                    // Seed the NEXT startNextPlan() call from the
                    // REFERENCE's own exit speed on a flying handoff
                    // (policy.cpp's own "Seeding contract" comment: never
                    // from measured state), or from rest on an ordinary
                    // stop.
                    nextEntrySpeed_ =
                        (out.status == Drive::Status::DONE_HANDOFF) ? plan_.exitSpeed() : 0.0f;
                    planActive_ = false;
                    break;
                case Drive::Status::ABORT_TIMEOUT:
                case Drive::Status::ABORT_REPLAN_LIMIT:
                    abortAndFlush(chainTail, out.status, in.measured, out.record);
                    break;
                default:
                    break;  // RUNNING, SETTLING
            }
        }

        if (!planActive_ && ring_.empty()) {
            // Nothing queued, nothing in flight -- idle out of segment mode,
            // matching the pre-cutover "S ... STOP with no segment ever in
            // flight" idle shape.
            segmentMode_ = false;
            neutral = true;
        }
    } else if (mode_ == Mode::NEUTRAL) {
        neutral = true;
    } else {
        commandedWheelTargets(&targetLeft, &targetRight);
    }

    if (neutral) {
        leftCmd.setNeutral(neutralMode_);
        rightCmd.setNeutral(neutralMode_);
        if (!segmentMode_) {
            remainingLinear_ = 0.0f;
        }
    } else {
        governRatio(&targetLeft, &targetRight, leftObs, rightObs);
        leftCmd.setVelocity(targetLeft);
        rightCmd.setVelocity(targetRight);
    }

    using CK = msg::MotorCommand::ControlKind;
    cmdVel_[0] = (leftCmd.control_kind == CK::VELOCITY) ? leftCmd.control.velocity : 0.0f;
    cmdVel_[1] = (rightCmd.control_kind == CK::VELOCITY) ? rightCmd.control.velocity : 0.0f;
    hardware_.motor(bound.left).apply(leftCmd);
    hardware_.motor(bound.right).apply(rightCmd);
}

// updateAccelEma -- one wheel's measured-acceleration EMA. UNCHANGED this
// ticket -- see the pre-100-007 doc comment (git history): a new EMA term
// folds in only when the velocity VALUE actually changes (a fresh sample).
namespace {
constexpr float kAccelEmaAlpha = 0.25f;
}

void Drivetrain::updateAccelEma(uint32_t now, int wheel, const msg::MotorState& obs) {
    if (!obs.velocity.has) return;
    const float v = obs.velocity.val;
    if (!haveVelSample_[wheel]) {
        haveVelSample_[wheel] = true;
        lastVelSample_[wheel] = v;
        lastVelSampleMs_[wheel] = now;
        return;
    }
    if (v == lastVelSample_[wheel]) return;
    const float dt = static_cast<float>(static_cast<int32_t>(now - lastVelSampleMs_[wheel]))
                     * 0.001f;
    if (dt <= 0.0f) return;
    const float rawAccel = (v - lastVelSample_[wheel]) / dt;
    accelEma_[wheel] = kAccelEmaAlpha * rawAccel + (1.0f - kAccelEmaAlpha) * accelEma_[wheel];
    lastVelSample_[wheel] = v;
    lastVelSampleMs_[wheel] = now;
}

msg::DrivetrainState Drivetrain::state() const {
    msg::DrivetrainState s;

    DrivetrainPorts bound = ports();
    if (bound.left < Hardware::kMotorCount && bound.right < Hardware::kMotorCount) {
        msg::MotorState leftObs = hardware_.motorState(bound.left);
        msg::MotorState rightObs = hardware_.motorState(bound.right);

        s.enc_[0] = leftObs.position.has ? leftObs.position.val : 0.0f;
        s.enc_[1] = rightObs.position.has ? rightObs.position.val : 0.0f;
        s.enc_count = 2;

        s.vel_[0] = leftObs.velocity.has ? leftObs.velocity.val : 0.0f;
        s.vel_[1] = rightObs.velocity.has ? rightObs.velocity.val : 0.0f;
        s.vel_count = 2;

        s.cmd_[0] = cmdVel_[0];
        s.cmd_[1] = cmdVel_[1];
        s.cmd_count = 2;

        s.acc_[0] = accelEma_[0];
        s.acc_[1] = accelEma_[1];
        s.acc_count = 2;
    }

    // Authority mode -- (100-007) segmentMode_ && planActive_ replaces the
    // retired executor_.active() read; active_ itself is unchanged (set
    // only by a driveIn escape-hatch command).
    s.active = active_ || (segmentMode_ && planActive_);

    // busy -- MOTION in progress, the flag TLM's active= token reports.
    s.busy = segmentMode_ ? planActive_
                          : (mode_ == Mode::WHEELS || mode_ == Mode::TWIST);

    // Motion-queue depth: ring_ entries PLUS the one currently executing.
    s.queue = ring_.size() + ((segmentMode_ && planActive_) ? 1u : 0u);

    s.rem = remainingLinear_;

    return s;
}

msg::PlanRecord Drivetrain::activePlanRecord() const {
    // planActive_ == false leaves plan_ at its default-constructed
    // (invalid/empty) value -- drivePlanRecord() on that is well-defined
    // (motion_plan.h's own "every query returns a safe zero/default"
    // contract) but callers should check hasActivePlan() first (this
    // class's own header comment on this method).
    return Subsystems::drivePlanRecord(plan_, state_.replanCount);
}

uint32_t Drivetrain::ringGoals(Drive::Goal* out, uint32_t capacity) const {
    uint32_t n = ring_.size();
    if (n > capacity) n = capacity;
    for (uint32_t i = 0; i < n; ++i) {
        const Drive::Goal* g = ring_.peek(i);
        out[i] = g ? *g : Drive::Goal{};
    }
    return n;
}

DrivetrainPorts Drivetrain::ports() const {
    return {boundLeft_, boundRight_};
}

void Drivetrain::standby() { active_ = false; }

}  // namespace Subsystems
