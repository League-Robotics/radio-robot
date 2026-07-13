// master_profile.cpp -- Drive::MasterProfile implementation. See
// master_profile.h for the class-level design notes (the solveToExit
// generalization, the seeding contract, the jerk sentinel, and the
// direction-mirrored acceleration bounds).
#include "drive/master_profile.h"

#include <algorithm>
#include <array>
#include <limits>

namespace Drive {

namespace {

// mapJerkSentinel -- ProfileLimits.jerk's 0.0 sentinel ("off -- trapezoid,
// no S-curve") maps to Ruckig's own max_jerk = +infinity default
// (master_profile.h's class comment), NOT a literal 0 (which would forbid
// any acceleration change). A positive value passes straight through. A
// negative value is not a meaningful configuration; treated the same as
// the sentinel rather than fed to Ruckig unchecked.
float mapJerkSentinel(float configuredJerk) {
  return (configuredJerk <= 0.0f) ? std::numeric_limits<float>::infinity() : configuredJerk;
}

}  // namespace

void MasterProfile::configure(const ProfileLimits& limits) {
  accel_ = limits.accel;
  decel_ = limits.decel;
  maxJerk_ = mapJerkSentinel(limits.jerk);
  velocityCeiling_ = limits.velocity;
}

void MasterProfile::reset() {
  lastPosition_ = 0.0f;
  lastVelocity_ = 0.0f;
  lastAcceleration_ = 0.0f;
}

void MasterProfile::seedCurrent(float position, float velocity, float acceleration) {
  lastPosition_ = position;
  lastVelocity_ = velocity;
  lastAcceleration_ = acceleration;
}

void MasterProfile::accelBoundsForDirection(bool positiveDirection, float* maxAccel,
                                             float* minAccel) const {
  // Open Question 2 (jerk_trajectory.h): Ruckig's max_acceleration/
  // min_acceleration are absolute-signed, not direction-of-travel-relative
  // the way accel_ (accelerating)/decel_ (decelerating) are. Mirror the
  // assignment for a negative-direction solve so "accelerating"/
  // "decelerating" keep meaning the same physical thing regardless of
  // travel direction.
  if (positiveDirection) {
    *maxAccel = accel_;
    *minAccel = -decel_;
  } else {
    *maxAccel = decel_;
    *minAccel = -accel_;
  }
}

bool MasterProfile::solvePositionControl(float targetPosition, float currentPosition,
                                          float currentVelocity, float currentAcceleration,
                                          float exitVelocity, float maxVelocity) {
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
  // exitVelocity, generalized (master_profile.h's class comment): solveToRest()
  // passes 0.0f here; solveToExit() passes its own caller-supplied exit
  // velocity. The SAME directional band below (unchanged from jerk_
  // trajectory.h's solveToRest()) validates it -- no separate check needed.
  input.target_velocity = {static_cast<double>(exitVelocity)};
  input.target_acceleration = {0.0};
  // Directional velocity band (Ruckig's own asymmetric-limit feature --
  // input_parameter.hpp min_velocity, consumed by the position interface
  // incl. its brake pre-trajectory): a position solve may never plan motion
  // AGAINST its own direction of travel. min_velocity = 0 for a positive-
  // direction solve (max_velocity = 0 for a negative one) makes "no
  // reversal" a CONSTRAINT OF THE SOLVE rather than anything downstream --
  // AND, generalized here, makes "exitVelocity must share the direction's
  // sign and stay within the ceiling" a constraint of the SAME solve
  // (master_profile.h's class comment): an ask that could only be met by
  // reversing, or an exitVelocity of the wrong sign or magnitude, now fails
  // cleanly (Ruckig's own target-state validation, confirmed against
  // input_parameter.hpp/ruckig.hpp) -- and a failed solve leaves the
  // previous trajectory in place (see the temp-solve note below) -- instead
  // of producing an invalid or UB result.
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
    lastPosition_ = currentPosition;
    lastVelocity_ = currentVelocity;
    lastAcceleration_ = currentAcceleration;
    calculated_ = true;
  }
  return ok;
}

bool MasterProfile::solveToRest(float targetPosition, float maxVelocity) {
  return solveToExit(targetPosition, 0.0f, maxVelocity);
}

bool MasterProfile::solveToExit(float targetPosition, float exitVelocity, float maxVelocity) {
  return solvePositionControl(targetPosition, lastPosition_, lastVelocity_, lastAcceleration_,
                               exitVelocity, maxVelocity);
}

bool MasterProfile::solveToVelocity(float targetVelocity, float maxVelocity) {
  const float ceiling = std::min(maxVelocity, velocityCeiling_);
  const float clampedTarget = std::clamp(targetVelocity, -ceiling, ceiling);
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

MasterProfile::State MasterProfile::peek(float elapsed) const {
  // Same calculated_ UB guard as sample() below; no seed-state update.
  if (!calculated_) return State{};
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  traj_.at_time(static_cast<double>(elapsed), position, velocity, acceleration);
  return State{static_cast<float>(position), static_cast<float>(velocity),
               static_cast<float>(acceleration)};
}

MasterProfile::State MasterProfile::sample(float elapsed) {
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

float MasterProfile::duration() const {
  return calculated_ ? static_cast<float>(traj_.get_duration()) : 0.0f;
}

}  // namespace Drive
