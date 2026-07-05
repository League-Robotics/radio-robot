// drivetrain.cpp -- Subsystems::Drivetrain implementation. See drivetrain.h
// for the class-level design notes.
#include "subsystems/drivetrain.h"

#include "kinematics/body_kinematics.h"

namespace Subsystems {

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
            // == 2): index 0 is left, index 1 is right, matching
            // Hal::DrivetrainToHardwareCommand's wheel[0]/wheel[1] fields. A
            // WheelTargets with fewer than 2 entries leaves the missing side
            // at 0 rather than reading past w_count.
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
            // re-anchor (those return in later tickets -- see
            // architecture-update.md "Later tickets"). Explicitly a
            // documented no-op, not a silent drop: apply() takes no action
            // for this arm rather than touching any setter above.
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

void Drivetrain::commandedWheelTargets(float* targetLeft, float* targetRight) const {
    switch (mode_) {
        case Mode::TWIST:
            // v_y_ is never read here: this Drivetrain is differential-only
            // (capabilities().holonomic == false this sprint). A future
            // mecanum ticket wires v_y in once holonomic can be true
            // (architecture-update.md Open Question 6) -- this is that
            // wiring's intended site. Scalar overload per this ticket's
            // acceptance criteria: kinematics/pose2d.h's BodyTwist3
            // (vx_mmps/vy_mmps/omega_rads) is a different, old-style-named
            // type from msg::BodyTwist3 (v_x/v_y/omega) -- the scalar
            // differential overload sidesteps any conversion between them.
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
                       const msg::MotorState& leftObs,
                       const msg::MotorState& rightObs) {
    // now: no clock read happens here -- this ticket's governor is a purely
    // per-tick algebraic correction with no timing-dependent behavior yet.
    // Kept as a parameter per the locked faceplate shape for a future ticket
    // that needs it (e.g. a governor ease-in rate).
    (void)now;

    msg::MotorCommand leftCmd;
    msg::MotorCommand rightCmd;

    if (mode_ == Mode::NEUTRAL) {
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

    // Held, not returned (architecture-update.md "The command-edge types"):
    // addressed via ports() so the wiring layer (ticket 079-005) can dispatch
    // straight through Subsystems::NezhaHardware::apply(const Hal::DrivetrainToHardwareCommand&)
    // without ever naming a port itself. Set unconditionally whenever tick()
    // runs -- see the class comment and hasCommand()'s doc comment.
    heldCommand_.wheel[0].port = config_.left_port;
    heldCommand_.wheel[0].command = leftCmd;
    heldCommand_.wheel[1].port = config_.right_port;
    heldCommand_.wheel[1].command = rightCmd;
    hasCommand_ = true;
}

bool Drivetrain::hasCommand() const { return hasCommand_; }

Hal::DrivetrainToHardwareCommand Drivetrain::takeCommand() {
    hasCommand_ = false;
    return heldCommand_;
}

msg::DrivetrainState Drivetrain::state() const {
    msg::DrivetrainState s;

    // Only the two wheel-velocity targets are populated -- pose/EKF/OTOS
    // fields (fused/encoder/optical, enc[], enc_stamp, otos, wheel_wedged[],
    // connected, otos_status, otos_fusion_blocked) stay at their zero
    // defaults: this differential dev-bench Drivetrain has no odometry/EKF
    // this sprint (those return in later tickets -- see
    // architecture-update.md "Later tickets"), and Drivetrain never retains
    // a Motor reference to ask about connectivity/wedge directly (see class
    // comment). vel_[]/vel_count report the CURRENT commanded (pre-governor)
    // targets, since the governor's live-observation-dependent output only
    // exists transiently inside tick().
    float left = 0.0f;
    float right = 0.0f;
    if (mode_ != Mode::NEUTRAL) {
        commandedWheelTargets(&left, &right);
    }
    s.vel_[0] = left;
    s.vel_[1] = right;
    s.vel_count = 2;

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
    return {config_.left_port, config_.right_port};
}

bool Drivetrain::active() const { return active_; }

void Drivetrain::standby() { active_ = false; }

}  // namespace Subsystems
