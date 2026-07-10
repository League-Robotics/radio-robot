// drivetrain.cpp -- Subsystems::Drivetrain implementation. See drivetrain.h
// for the class-level design notes (the 094-004 motion-planner rewrite).
#include "subsystems/drivetrain.h"

#include <cassert>

#include "kinematics/body_kinematics.h"

namespace Subsystems {

Drivetrain::Drivetrain(Hardware& hardware) : hardware_(hardware) {}

void Drivetrain::setTwist(float v_x, float v_y, float omega) {
    mode_ = Mode::TWIST;
    v_x_ = v_x;
    v_y_ = v_y;   // stored but never consumed -- see commandedWheelTargets()
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
    // THE single conversion point (0-based motor indices, OOP refactor):
    // config.left_port/right_port are wire/serialized 1-based brick labels
    // (msg::DrivetrainConfig, unchanged) -- converted to 0-based Hardware
    // motor indices here, exactly once, and never again anywhere else in
    // this class (see drivetrain.h's own doc comments on configure()/
    // boundLeft_/boundRight_).
    boundLeft_ = config.left_port - 1;
    boundRight_ = config.right_port - 1;
}

void Drivetrain::configureMotion(const msg::PlannerConfig& config) {
    executor_.configure(config);
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
            // This Drivetrain has exactly two wheels (capabilities().wheel_count
            // == 2): index 0 is left, index 1 is right. A WheelTargets with
            // fewer than 2 entries leaves the missing side at 0 rather than
            // reading past w_count.
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
            // The pose (imperative re-anchor) arm is ignored this sprint:
            // this differential dev-bench Drivetrain has no odometry/EKF to
            // re-anchor. Explicitly a documented no-op, not a silent drop:
            // apply() takes no action for this arm rather than touching any
            // setter above.
            break;
        case msg::DrivetrainCommand::ControlKind::NONE:
        default:
            // No control arm set -- e.g. an authority-steal command whose
            // only payload is standby=true (see below). Nothing to dispatch
            // here; mode_/the last commanded target are left untouched, on
            // purpose (the class comment's "Authority arbitration" section).
            break;
    }

    // The standby side-channel rides beside the oneof exactly like
    // MotorCommand's feedforward/reset_position -- processed AFTER the oneof
    // above so `{control=NEUTRAL, standby=true}` sets mode_ AND drops
    // authority in the same call (see the class comment).
    if (command.standby.has && command.standby.val) {
        standby();
    }
}

void Drivetrain::clearRing() {
    while (!ring_.empty()) {
        ring_.take();
    }
}

bool Drivetrain::dispatchEscapeHatch(const msg::DrivetrainCommand& command, uint32_t now) {
    using Kind = msg::DrivetrainCommand::ControlKind;
    bool preempted = (command.control_kind == Kind::TWIST ||
                      command.control_kind == Kind::WHEELS ||
                      command.control_kind == Kind::NEUTRAL);

    if (preempted) {
        clearRing();

        if (command.control_kind == Kind::NEUTRAL && segmentMode_ && executor_.active()) {
            // A segment is actively executing (or already mid-graceful-stop)
            // -- NEUTRAL arms the executor's OWN presolved graceful
            // decel-to-zero instead of an instant zero-velocity command
            // (architecture-update.md Section 6's "STOP triggers the
            // graceful decel-to-zero"). segmentMode_ stays true: tick()
            // keeps riding executor_'s decel down to a literal 0.0f twist
            // over subsequent ticks, then idles.
            executor_.stop(now);
        } else {
            // Nothing in-flight to gracefully decelerate (DIRECT mode was
            // already active, or the executor was already idle) -- fall
            // through to the instant DIRECT-mode zero/target this command
            // sets below (apply()'s setNeutral()/setWheelTargets()/
            // setTwist()).
            segmentMode_ = false;
        }
    }

    apply(command);
    return preempted;
}

void Drivetrain::commandedWheelTargets(float* targetLeft, float* targetRight) const {
    switch (mode_) {
        case Mode::TWIST:
            // v_y_ is never read here: this Drivetrain is differential-only
            // (capabilities().holonomic == false this sprint). This
            // differential path calls the scalar (v_x, omega) overload
            // directly; BodyKinematics's msg::BodyTwist3 array overload is
            // for the holonomic/mecanum path.
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
    if (config_.sync_gain <= 0.0f) return;   // SET sync=0 -> independent (ported semantics)

    // Only couple when both wheels are commanded in the SAME direction
    // (straight or curve). A spin-in-place has opposite-signed targets;
    // coupling those would degenerate the spin toward a single wheel
    // (ported caveat from source_old/control/MotorController.cpp's
    // tgtSpeed product>0 gate). A zero target on either side also skips
    // governing -- there is no ratio to hold against a stopped wheel.
    if (*targetLeft == 0.0f || *targetRight == 0.0f) return;
    if ((*targetLeft) * (*targetRight) <= 0.0f) return;

    if (!leftObs.velocity.has || !rightObs.velocity.has) return;

    // Achievement fraction: how much of ITS OWN commanded target each wheel
    // is actually hitting. 1.0 = on target; < 1.0 = bogged down (or
    // reversing relative to its command, which clamps to 0 below).
    float achievedLeft = leftObs.velocity.val / *targetLeft;
    float achievedRight = rightObs.velocity.val / *targetRight;
    float achievedMin = (achievedLeft < achievedRight) ? achievedLeft : achievedRight;

    if (achievedMin >= 1.0f) return;   // neither wheel is bogged down
    if (achievedMin < 0.0f) achievedMin = 0.0f;

    // Blend the shared ceiling toward the bogged-down wheel's actual pace by
    // sync_gain (sync_gain=1 fully commits every tick; smaller values ease
    // in). Scaling BOTH targets by the SAME factor holds the commanded
    // left/right ratio exactly (curvature preserved) -- this is the
    // re-targeting of source_old's syncGain (which nudged only the leading
    // wheel's effective target toward a computed "coupled" value) onto
    // velocity targets: a single shared scale is the ratio-exact form of the
    // same idea, and never touches duty cycle.
    float scale = 1.0f - config_.sync_gain * (1.0f - achievedMin);
    if (scale < 0.0f) scale = 0.0f;
    *targetLeft *= scale;
    *targetRight *= scale;
}

void Drivetrain::tick(uint32_t now,
                       Rt::WorkQueue<Motion::Segment, 8>& segmentIn,
                       Rt::WorkQueue<msg::DrivetrainCommand, 8>& driveIn) {
    // 1. driveIn drained FIRST -- one command per tick (FIFO pop, matching
    // this class's pre-094 drain cadence), applied via the escape-hatch
    // dispatcher (see the class comment's precedence rules).
    bool preempted = false;
    if (!driveIn.empty()) {
        preempted = dispatchEscapeHatch(driveIn.take(), now);
    }

    // 2. Otherwise, drain segmentIn IN FULL into ring_ this tick -- queuing
    // at least one fresh segment (re)claims SEGMENT mode (see the class
    // comment).
    if (!preempted) {
        bool queuedAny = false;
        while (!segmentIn.empty()) {
            if (ring_.post(segmentIn.take())) {
                queuedAny = true;
            }
            // A post() failure (ring_ already at its 8-slot cap) silently
            // drops the excess -- should not occur in ordinary operation,
            // since segmentIn itself caps at the same depth and is drained
            // every tick.
        }
        if (queuedAny) {
            segmentMode_ = true;
        }
    }

    // 090-001 (moved, 094-004): resolve this Drivetrain's OWN bound wheel
    // pair -- boundLeft_/boundRight_ are already-converted 0-based indices
    // (configure()'s own single conversion point), and Hardware::state()/
    // motor() take a 0-based index and do their own out-of-range clamping
    // (see drivetrain.h's tick() doc comment), so there is no further `- 1`
    // conversion to perform here; the range assert below is kept as a
    // defensive guard against a misconfigured bound index.
    DrivetrainPorts bound = ports();
    assert(bound.left < Hardware::kMotorCount);
    assert(bound.right < Hardware::kMotorCount);
    const msg::MotorState leftObs = hardware_.motorState(bound.left);
    const msg::MotorState rightObs = hardware_.motorState(bound.right);

    // Measured per-wheel acceleration, EMA-filtered -- surfaced via
    // state()/TLM `acc=`. Raw d(vel)/dt of the (quantized, flip-flop-cadence)
    // velocity samples is dominated by noise; the EMA lives HERE in the
    // firmware so every consumer gets the same smooth signal instead of each
    // deriving its own (stakeholder direction, 2026-07-09).
    updateAccelEma(now, 0, leftObs);
    updateAccelEma(now, 1, rightObs);

    msg::MotorCommand leftCmd;
    msg::MotorCommand rightCmd;

    if (segmentMode_) {
        // Pop-on-completion, start-next: once the executor is idle (never
        // started, or the previous segment -- including its own trailing
        // graceful stop -- fully converged) and the ring is non-empty, hand
        // it the next queued segment.
        if (!executor_.active() && !ring_.empty()) {
            Motion::Segment seg = ring_.take();
            executor_.start(seg, now, config_.trackwidth);
        } else if (executor_.streaming() && !executor_.hasPending() && !ring_.empty() &&
                   ring_.peek(0) != nullptr && ring_.peek(0)->stream) {
            // Streaming merge feed: while a STREAM segment executes and the
            // next queued segment is also a stream one, top up the executor's
            // one-deep pending slot -- it merges mid-plan on the executor's
            // next tick (SegmentExecutor::offerNext()'s contract). Discrete
            // segments never enter this path: they wait for idle and execute
            // fully sequentially, exactly as before.
            executor_.offerNext(ring_.take());
        }

        msg::BodyTwist3 twist = executor_.tick(now, leftObs, rightObs);
        float targetLeft = 0.0f;
        float targetRight = 0.0f;
        BodyKinematics::inverse(twist.v_x, twist.omega, config_.trackwidth,
                                 targetLeft, targetRight);
        governRatio(&targetLeft, &targetRight, leftObs, rightObs);
        leftCmd.setVelocity(targetLeft);
        rightCmd.setVelocity(targetRight);
    } else if (mode_ == Mode::NEUTRAL) {
        leftCmd.setNeutral(neutralMode_);
        rightCmd.setNeutral(neutralMode_);
    } else {
        float targetLeft = 0.0f;
        float targetRight = 0.0f;
        commandedWheelTargets(&targetLeft, &targetRight);
        governRatio(&targetLeft, &targetRight, leftObs, rightObs);

        leftCmd.setVelocity(targetLeft);
        rightCmd.setVelocity(targetRight);
    }

    // Staged, not held -- flushed at hardware_'s own tick() cadence (see
    // drivetrain.h's class comment and architecture-update.md Section 5).
    // Nothing left to route. Remember this pass's post-governor commanded
    // wheel velocities so state()/TLM can surface cmd= (measured vel= vs the
    // setpoint the velocity PID is chasing).
    using CK = msg::MotorCommand::ControlKind;
    cmdVel_[0] = (leftCmd.control_kind == CK::VELOCITY) ? leftCmd.control.velocity : 0.0f;
    cmdVel_[1] = (rightCmd.control_kind == CK::VELOCITY) ? rightCmd.control.velocity : 0.0f;
    hardware_.motor(bound.left).apply(leftCmd);
    hardware_.motor(bound.right).apply(rightCmd);
}

// updateAccelEma -- one wheel's measured-acceleration EMA. Velocity samples
// refresh at the I2C flip-flop's cadence (~80ms per motor on hardware), while
// tick() runs every loop pass -- so a new EMA term is folded in only when the
// velocity VALUE actually changes (a fresh sample), with dt measured between
// those changes. Between samples the EMA holds. kAccelEmaAlpha trades lag for
// smoothness; raw d(vel)/dt is quantization hash (the whole reason this lives
// in firmware).
namespace {
constexpr float kAccelEmaAlpha = 0.25f;
}

void Drivetrain::updateAccelEma(uint32_t now, int wheel, const msg::MotorState& obs) {
    if (!obs.velocity.has) return;
    const float v = obs.velocity.val;   // [mm/s]
    if (!haveVelSample_[wheel]) {
        haveVelSample_[wheel] = true;
        lastVelSample_[wheel] = v;
        lastVelSampleMs_[wheel] = now;
        return;
    }
    if (v == lastVelSample_[wheel]) return;   // no fresh sample this pass
    const float dt = static_cast<float>(static_cast<int32_t>(now - lastVelSampleMs_[wheel]))
                     * 0.001f;   // [s]
    if (dt <= 0.0f) return;
    const float rawAccel = (v - lastVelSample_[wheel]) / dt;   // [mm/s^2]
    accelEma_[wheel] = kAccelEmaAlpha * rawAccel + (1.0f - kAccelEmaAlpha) * accelEma_[wheel];
    lastVelSample_[wheel] = v;
    lastVelSampleMs_[wheel] = now;
}

msg::DrivetrainState Drivetrain::state() const {
    msg::DrivetrainState s;

    // enc_[]/vel_[] are sourced from hardware_.motorState(i) -- MEASURED, not
    // commanded (094-004: replaces the pre-094 "reports the pre-governor
    // commanded target" behavior entirely). Pose/EKF fields (fused/encoder/
    // optical, otos, wheel_wedged[], connected, otos_status,
    // otos_fusion_blocked) stay at their zero defaults -- this differential
    // dev-bench Drivetrain has no odometry/EKF this sprint.
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

        s.cmd_[0] = cmdVel_[0];   // [mm/s] post-governor commanded (vs measured vel_)
        s.cmd_[1] = cmdVel_[1];
        s.cmd_count = 2;

        s.acc_[0] = accelEma_[0];   // [mm/s^2] EMA-filtered measured acceleration
        s.acc_[1] = accelEma_[1];
        s.acc_count = 2;
    }

    // Authority mode, readable from this state cell without a Drivetrain*,
    // OR'd with the owned Motion::SegmentExecutor's own active/idle status
    // (094-006): `active_` alone (the pre-079 authority-arbitration flag)
    // is set ONLY by setTwist()/setWheelTargets()/setNeutral() -- i.e. only
    // by a driveIn (S/STOP) escape-hatch command -- so a session driven
    // entirely by `MOVE`/segmentIn would report `active=false` throughout
    // even while a segment is actively executing (or riding its own
    // graceful decel-to-zero), which is useless for `TLM`'s active/idle
    // poll (architecture-update.md Section 7, "Telemetry and the deferred
    // completion event"). `segmentMode_ && executor_.active()` covers that
    // case without changing `active_`'s own pre-existing meaning for the
    // DIRECT-mode/authority path.
    s.active = active_ || (segmentMode_ && executor_.active());

    // busy -- MOTION in progress, the flag TLM's active= token actually
    // reports (094 OOP fix). `active_` is the pre-079 AUTHORITY flag:
    // setNeutral() sets it TRUE too (holding neutral IS governing the bound
    // pair), so it latches 1 after the first STOP ever sent and can never
    // mean "idle" -- the notebook/bench completion polls were reading a
    // permanently-1 flag. Segment mode: busy while the executor has a phase
    // in flight (incl. its trailing graceful decel). Direct mode: busy while
    // a WHEELS/TWIST drive command is the standing mode (S...STOP window).
    s.busy = segmentMode_ ? executor_.active()
                          : (mode_ == Mode::WHEELS || mode_ == Mode::TWIST);

    // Motion-queue depth: segments waiting in the ring PLUS the one
    // currently executing. Surfaced in the MOVE ack (`q=`) so a streaming
    // teleop client can flow-control its send rate against the real backlog.
    s.queue = ring_.size() + ((segmentMode_ && executor_.active()) ? 1u : 0u);

    return s;
}

msg::DrivetrainCapabilities Drivetrain::capabilities() const {
    msg::DrivetrainCapabilities caps;
    caps.holonomic = false;   // differential-only this sprint (Tovez) -- see setTwist()
    caps.wheel_count = 2;
    caps.onboard_position = leftMotorCaps_.position && rightMotorCaps_.position;
    return caps;
}

void Drivetrain::setMotorCapabilities(const msg::MotorCapabilities& left,
                                       const msg::MotorCapabilities& right) {
    leftMotorCaps_ = left;
    rightMotorCaps_ = right;
}

DrivetrainPorts Drivetrain::ports() const {
    // Already-converted 0-based indices (configure()'s single conversion
    // point) -- NOT config_.left_port/right_port (the 1-based wire labels)
    // directly.
    return {boundLeft_, boundRight_};
}

bool Drivetrain::active() const { return active_; }

void Drivetrain::standby() { active_ = false; }

}  // namespace Subsystems
