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
}

void Drivetrain::setWheelTargets(float left, float right) {
    mode_ = Mode::WHEELS;
    wheelTargetLeft_ = left;
    wheelTargetRight_ = right;
}

void Drivetrain::setNeutral(msg::Neutral mode) {
    mode_ = Mode::NEUTRAL;
    neutralMode_ = mode;
}

void Drivetrain::configure(const msg::DrivetrainConfig& config) {
    config_ = config;
}

void Drivetrain::apply(const msg::DrivetrainCommand& command) {
    switch (command.get_control_kind()) {
        case msg::DrivetrainCommand::ControlKind::TWIST: {
            const msg::BodyTwist3& twist = command.control.twist;
            setTwist(twist.v_x, twist.v_y, twist.omega);
            break;
        }
        case msg::DrivetrainCommand::ControlKind::WHEELS: {
            const msg::WheelTargets& wheels = command.control.wheels;
            // This Drivetrain has exactly two wheels (capabilities().wheel_count
            // == 2): index 0 is left, index 1 is right, matching
            // DrivetrainToMotorCommand's left/right fields. A WheelTargets
            // with fewer than 2 entries leaves the missing side at 0 rather
            // than reading past w_count.
            float left = 0.0f;
            float right = 0.0f;
            if (wheels.w_count_val() > 0 && wheels.w()[0].get_speed().has) {
                left = wheels.w()[0].get_speed().val;
            }
            if (wheels.w_count_val() > 1 && wheels.w()[1].get_speed().has) {
                right = wheels.w()[1].get_speed().val;
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
            break;
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
            BodyKinematics::inverse(v_x_, omega_, config_.get_trackwidth(),
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
    if (config_.get_sync_gain() <= 0.0f) return;   // SET sync=0 -> independent (ported semantics)

    // Only couple when both wheels are commanded in the SAME direction
    // (straight or curve). A spin-in-place has opposite-signed targets;
    // coupling those would degenerate the spin toward a single wheel
    // (ported caveat from source_old/control/MotorController.cpp's
    // tgtSpeed product>0 gate). A zero target on either side also skips
    // governing -- there is no ratio to hold against a stopped wheel.
    if (*targetLeft == 0.0f || *targetRight == 0.0f) return;
    if ((*targetLeft) * (*targetRight) <= 0.0f) return;

    if (!leftObs.get_velocity().has || !rightObs.get_velocity().has) return;

    // Achievement fraction: how much of ITS OWN commanded target each wheel
    // is actually hitting. 1.0 = on target; < 1.0 = bogged down (or
    // reversing relative to its command, which clamps to 0 below).
    float achievedLeft = leftObs.get_velocity().val / *targetLeft;
    float achievedRight = rightObs.get_velocity().val / *targetRight;
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
    float scale = 1.0f - config_.get_sync_gain() * (1.0f - achievedMin);
    if (scale < 0.0f) scale = 0.0f;
    *targetLeft *= scale;
    *targetRight *= scale;
}

DrivetrainToMotorCommand Drivetrain::tick(uint32_t now,
                                           const msg::MotorState& leftObs,
                                           const msg::MotorState& rightObs) {
    // now: no clock read happens here -- this ticket's governor is a purely
    // per-tick algebraic correction with no timing-dependent behavior yet.
    // Kept as a parameter per the locked faceplate shape for a future ticket
    // that needs it (e.g. a governor ease-in rate).
    (void)now;

    DrivetrainToMotorCommand out;

    if (mode_ == Mode::NEUTRAL) {
        out.left.setNeutral(neutralMode_);
        out.right.setNeutral(neutralMode_);
        return out;
    }

    float targetLeft = 0.0f;
    float targetRight = 0.0f;
    commandedWheelTargets(&targetLeft, &targetRight);
    governRatio(&targetLeft, &targetRight, leftObs, rightObs);

    out.left.setVelocity(targetLeft);
    out.right.setVelocity(targetRight);
    return out;
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
    caps.onboard_position = leftMotorCaps_.get_position() && rightMotorCaps_.get_position();
    return caps;
}

void Drivetrain::setMotorCapabilities(const msg::MotorCapabilities& left,
                                       const msg::MotorCapabilities& right) {
    leftMotorCaps_ = left;
    rightMotorCaps_ = right;
}

}  // namespace Subsystems
