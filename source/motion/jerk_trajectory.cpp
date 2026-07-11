// jerk_trajectory.cpp -- Motion::JerkTrajectory implementation. See
// jerk_trajectory.h for the class-level design notes (two solve modes, the
// seeding contract, retarget()/reanchor(), the jerk sentinel, and the
// direction-mirrored linear-channel acceleration bounds).
#include "motion/jerk_trajectory.h"

#include <algorithm>
#include <array>
#include <limits>

namespace Motion {

namespace {

// mapJerkSentinel -- PlannerConfig's existing 0.0 sentinel ("off --
// trapezoid, no S-curve") maps to Ruckig's own max_jerk = +infinity
// default (architecture-update.md (089) Decision 6), NOT a literal 0
// (which would forbid any acceleration change). A positive value passes
// straight through. A negative value is not a meaningful configuration;
// treated the same as the sentinel rather than fed to Ruckig unchecked.
float mapJerkSentinel(float configuredJerk) {
  return (configuredJerk <= 0.0f) ? std::numeric_limits<float>::infinity() : configuredJerk;
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

}  // namespace

void JerkTrajectory::configure(const msg::PlannerConfig& config, bool isRotational) {
  if (isRotational) {
    // Symmetric by construction (Decision 9): both directions are bounded
    // by the SAME yaw_acc_max, so accelBoundsForDirection()'s mirroring
    // formula collapses to an identical result regardless of direction --
    // no separate rotational branch is needed anywhere in this class.
    aMax_ = config.yaw_acc_max;
    aDecel_ = config.yaw_acc_max;
    maxJerk_ = mapJerkSentinel(config.yaw_jerk_max);
    velocityCeiling_ = config.yaw_rate_max;
  } else {
    aMax_ = config.a_max;
    aDecel_ = config.a_decel;
    maxJerk_ = mapJerkSentinel(config.j_max);
    velocityCeiling_ = config.v_body_max;
  }
}

void JerkTrajectory::reset() {
  lastPosition_ = 0.0f;
  lastVelocity_ = 0.0f;
  lastAcceleration_ = 0.0f;
}

void JerkTrajectory::seedCurrent(float position, float velocity, float acceleration) {
  lastPosition_ = position;
  lastVelocity_ = velocity;
  lastAcceleration_ = acceleration;
}

void JerkTrajectory::accelBoundsForDirection(bool positiveDirection, float* maxAccel,
                                              float* minAccel) const {
  // Open Question 2: Ruckig's max_acceleration/min_acceleration are
  // absolute-signed, not direction-of-travel-relative the way a_max
  // (accelerating)/a_decel (decelerating) are. Mirror the assignment for a
  // negative-direction solve so "accelerating"/"decelerating" keep meaning
  // the same physical thing regardless of travel direction.
  if (positiveDirection) {
    *maxAccel = aMax_;
    *minAccel = -aDecel_;
  } else {
    *maxAccel = aDecel_;
    *minAccel = -aMax_;
  }
}

bool JerkTrajectory::solvePositionControl(float targetPosition, float currentPosition,
                                           float currentVelocity, float currentAcceleration,
                                           float maxVelocity) {
  const float ceiling = std::min(maxVelocity, velocityCeiling_);
  const bool positiveDirection = (targetPosition - currentPosition) >= 0.0f;
  float maxAccel = 0.0f;
  float minAccel = 0.0f;
  accelBoundsForDirection(positiveDirection, &maxAccel, &minAccel);

  ruckig::InputParameter<1> input;
  input.control_interface = ruckig::ControlInterface::Position;
  input.current_position = {static_cast<double>(currentPosition)};
  input.current_velocity = {static_cast<double>(currentVelocity)};
  input.current_acceleration = {static_cast<double>(currentAcceleration)};
  input.target_position = {static_cast<double>(targetPosition)};
  input.target_velocity = {0.0};
  input.target_acceleration = {0.0};
  // Directional velocity band (Ruckig's own asymmetric-limit feature --
  // input_parameter.hpp min_velocity, consumed by the position interface
  // incl. its brake pre-trajectory): a position solve may never plan motion
  // AGAINST its own direction of travel. min_velocity = 0 for a positive-
  // direction solve (max_velocity = 0 for a negative one) makes "no
  // reversal" a CONSTRAINT OF THE SOLVE rather than anything downstream:
  // an ask that could only be met by reversing (e.g. a replan seeded past
  // its target) now fails cleanly -- and a failed solve leaves the previous
  // trajectory in place (see the temp-solve note below) -- instead of
  // producing Ruckig's otherwise-correct time-optimal overshoot-and-return.
  if (positiveDirection) {
    input.max_velocity = {static_cast<double>(ceiling)};
    input.min_velocity = std::array<double, 1>{0.0};
  } else {
    input.max_velocity = {0.0};
    input.min_velocity = std::array<double, 1>{static_cast<double>(-ceiling)};
  }
  input.max_acceleration = {static_cast<double>(maxAccel)};
  input.min_acceleration = std::array<double, 1>{static_cast<double>(minAccel)};
  input.max_jerk = {static_cast<double>(maxJerk_)};

  // Solve into a temporary: calculate() writes into its trajectory argument
  // DURING calculation, so a failed solve would otherwise corrupt the
  // in-flight trajectory the caller keeps sampling. Trajectory<1> is a
  // stack-based value type (std::array-backed, no heap) -- the copy is
  // cheap and only taken on success.
  ruckig::Trajectory<1> solved;
  const ruckig::Result result = otg_.calculate(input, solved);
  const bool ok = (result == ruckig::Result::Working);
  if (ok) {
    traj_ = solved;
    target_ = targetPosition;
    maxVelocity_ = maxVelocity;
    lastPosition_ = currentPosition;
    lastVelocity_ = currentVelocity;
    lastAcceleration_ = currentAcceleration;
    calculated_ = true;
  }
  return ok;
}

bool JerkTrajectory::solveToRest(float targetPosition, float maxVelocity) {
  return solvePositionControl(targetPosition, lastPosition_, lastVelocity_, lastAcceleration_,
                               maxVelocity);
}

bool JerkTrajectory::retarget(float newRemaining) {
  // Re-baseline to 0 (class comment); seed velocity/acceleration from this
  // channel's OWN last sample, never the position.
  return solvePositionControl(newRemaining, /*currentPosition=*/0.0f, lastVelocity_,
                               lastAcceleration_, maxVelocity_);
}

bool JerkTrajectory::reanchor(float position, float velocity) {
  // Re-solve to the SAME target this channel was already tracking; seed
  // from the CALLER-SUPPLIED position/velocity with acceleration forced to
  // 0 -- a deliberate exception to the normal seeding contract (class
  // comment, Decision 8's revision).
  return solvePositionControl(target_, position, velocity, /*currentAcceleration=*/0.0f,
                               maxVelocity_);
}

bool JerkTrajectory::solveToVelocity(float targetVelocity, float maxVelocity) {
  const float ceiling = std::min(maxVelocity, velocityCeiling_);
  const float clampedTarget = clampf(targetVelocity, -ceiling, ceiling);
  const bool positiveDirection =
      (lastVelocity_ != 0.0f) ? (lastVelocity_ > 0.0f) : (clampedTarget >= 0.0f);
  float maxAccel = 0.0f;
  float minAccel = 0.0f;
  accelBoundsForDirection(positiveDirection, &maxAccel, &minAccel);

  ruckig::InputParameter<1> input;
  input.control_interface = ruckig::ControlInterface::Velocity;
  // current_position/target_position are ignored by Ruckig in
  // Velocity-control mode (input_parameter.hpp's own ControlInterface
  // doc comment) -- set to a benign, defined value rather than left
  // uninitialized.
  input.current_position = {0.0};
  input.current_velocity = {static_cast<double>(lastVelocity_)};
  input.current_acceleration = {static_cast<double>(lastAcceleration_)};
  input.target_position = {0.0};
  input.target_velocity = {static_cast<double>(clampedTarget)};
  input.target_acceleration = {0.0};
  input.max_velocity = {static_cast<double>(ceiling)};
  input.max_acceleration = {static_cast<double>(maxAccel)};
  input.min_acceleration = std::array<double, 1>{static_cast<double>(minAccel)};
  input.max_jerk = {static_cast<double>(maxJerk_)};

  // Same temp-solve discipline as solvePositionControl() -- a failed solve
  // must never corrupt the in-flight trajectory.
  ruckig::Trajectory<1> solved;
  const ruckig::Result result = otg_.calculate(input, solved);
  const bool ok = (result == ruckig::Result::Working);
  if (ok) {
    traj_ = solved;
    // The position channel is not meaningful in velocity-control mode;
    // velocity/acceleration already hold the seed used above.
    lastPosition_ = 0.0f;
    calculated_ = true;
  }
  return ok;
}

JerkTrajectory::State JerkTrajectory::peek(float elapsed) const {
  // Same calculated_ UB guard as sample() below; no seed-state update.
  if (!calculated_) return State{};
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  traj_.at_time(static_cast<double>(elapsed), position, velocity, acceleration);
  return State{static_cast<float>(position), static_cast<float>(velocity),
               static_cast<float>(acceleration)};
}

JerkTrajectory::State JerkTrajectory::sample(float elapsed) {
  // Defensive guard (see class comment's calculated_ doc) -- never touch an
  // un-calculate()'d traj_; its default-constructed Profile holds
  // uninitialized std::array members, so reading it is UB.
  if (!calculated_) return State{};
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  traj_.at_time(static_cast<double>(elapsed), position, velocity, acceleration);
  lastPosition_ = static_cast<float>(position);
  lastVelocity_ = static_cast<float>(velocity);
  lastAcceleration_ = static_cast<float>(acceleration);
  return State{lastPosition_, lastVelocity_, lastAcceleration_};
}

float JerkTrajectory::duration() const {
  return calculated_ ? static_cast<float>(traj_.get_duration()) : 0.0f;
}

}  // namespace Motion
