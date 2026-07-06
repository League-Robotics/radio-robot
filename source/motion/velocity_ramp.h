// velocity_ramp.h -- Motion::VelocityRamp: a body-level (v, omega) motion
// profiler.
//
// Ported from source_old/control/BodyVelocityController.{h,cpp} MINUS its
// kinematics/saturate/motor-output tail (architecture-update.md (084)
// Decision 3): this class produces a ramped (v, omega) ONLY. It never calls
// BodyKinematics::inverse()/saturate() and never writes to a motor --
// Subsystems::Drivetrain already owns kinematics and the ratio governor for
// its TWIST arm (see source/subsystems/drivetrain.h); Subsystems::Planner
// (the sole caller of this class) hands the ramped twist straight to
// Drivetrain::setTwist()/msg::DrivetrainCommand{TWIST}, exactly the way
// `DEV DT VW` already does. Duplicating kinematics/saturation here would be
// the shotgun-surgery risk Decision 3 explicitly rejects.
//
// Unit note: msg::PlannerConfig's yaw_rate_max/yaw_acc_max/yaw_jerk_max carry
// no unit annotation in protos/planner.proto -- unlike source_old's
// RobotConfig, which stored the same three quantities in deg/s, deg/s^2,
// deg/s^3 and converted to rad at each BodyVelocityController::advance()
// call. This port treats all three as rad/s, rad/s^2, rad/s^3 DIRECTLY,
// matching every other omega-shaped quantity already in the new message
// schema (msg::BodyTwist3.omega, msg::VelocityGoal.omega, ... all rad/s per
// protos/common.proto's BodyTwist3 doc comment) instead of reintroducing a
// lone degrees-based field family with a conversion constant nothing else in
// this tree needs. Config authors (ticket 084-006's SET/GET wiring) must
// supply these three fields in rad/s-family units, not deg/s.
//
// configure()'s existence is an intentional, documented addition beyond
// source_old/control/BodyVelocityController.h's own public surface (which
// bound `const RobotConfig&` as a live reference at construction time,
// because pre-message-plane RobotConfig was one process-wide mutable
// struct). Every config-bearing class in the new message-based tree (
// Subsystems::Drivetrain::configure(), Subsystems::PoseEstimator::
// configure()) instead takes an owned COPY of its config message via its own
// configure() method -- VelocityRamp follows that same, already-established
// convention rather than reintroducing a live-reference binding pattern this
// tree's own architecture moved away from.
#pragma once

#include <stdint.h>

#include "messages/planner.h"

namespace Motion {

class VelocityRamp {
 public:
  // setTarget -- update the commanded body twist. Does not step the
  // profiler; call advance() to ramp toward the new target.
  void setTarget(float v, float omega);  // [mm/s] [rad/s]

  // advance -- step the profiler one control tick. Reads the motion limits
  // from the most recent configure() call. At j_max == 0 (default): pure
  // trapezoid (approach v/omega directly under the per-tick accel-limited
  // step). At j_max > 0 (yaw_jerk_max > 0 for the yaw channel): S-curve --
  // slews the live acceleration toward the demanded step under the jerk
  // bound, then integrates. Must be called exactly once per Subsystems::
  // Planner tick. dt <= 0 is a no-op (returns !atTarget()).
  //
  // Returns true while still ramping toward target; false once atTarget().
  bool advance(float dt);  // [s]

  // reset -- zero all profiler state (current v/omega, target v/omega, and
  // the live S-curve acceleration channels).
  void reset();

  // seedCurrent -- set the live profiler state directly (no ramp step). Use
  // when handing off from another mode so the next advance() ramps from the
  // current actual twist rather than from zero.
  void seedCurrent(float v, float omega);  // [mm/s] [rad/s]

  float currentV() const { return v_; }          // [mm/s] signed
  float currentOmega() const { return omega_; }  // [rad/s] signed

  // atTarget -- true once the profiler has converged on the (clamped)
  // target: |v - vTargetClamped| < 0.5 mm/s AND |omega - omegaTargetClamped|
  // < 0.001 rad/s. The clamped targets are re-derived from the live config
  // limits, matching the ported source's own convergence test.
  bool atTarget() const;

  // configure -- store the motion-limit config subsequently read by
  // advance()/atTarget() (a_max, a_decel, v_body_max, yaw_rate_max,
  // yaw_acc_max, j_max, yaw_jerk_max). See the class comment for why this
  // method exists despite no direct source_old equivalent.
  void configure(const msg::PlannerConfig& config);

 private:
  static float approach(float cur, float tgt, float step);
  static float clamp(float v, float lo, float hi);

  msg::PlannerConfig config_ = {};

  float v_ = 0.0f;           // [mm/s] live profiled body forward speed
  float omega_ = 0.0f;       // [rad/s] live profiled yaw rate
  float vTgt_ = 0.0f;        // [mm/s] commanded forward speed (pre-clamp)
  float omegaTgt_ = 0.0f;    // [rad/s] commanded yaw rate (pre-clamp)
  float aLive_ = 0.0f;       // [mm/s^2] live linear accel (S-curve channel)
  float omegaALive_ = 0.0f;  // [rad/s^2] live yaw accel (S-curve channel)
};

}  // namespace Motion
