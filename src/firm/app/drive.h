// drive.h -- App::Drive: converts a body twist into per-wheel velocity
// targets and stages them onto the two Devices::NezhaMotor leaves.
//
// Boundary: inside -- staging vL/vR from BodyKinematics::inverse() onto
// the two NezhaMotor leaves' own setVelocity() setter; outside -- the
// kinematics math itself (stays in BodyKinematics) and the deadman
// decision (the loop calls Drive::stop(); Drive never polls Deadman).
//
// Drive is a PURE velocity follower (115-005, gut S1): setTwist()/stop()
// only STAGE a target; tick() computes wheel targets via
// BodyKinematics::inverse() and stages them onto the leaves via
// setVelocity() -- it never calls NezhaMotor::tick() itself, and it never
// touches the bus or sleeps, so the loop can call it from anywhere in its
// own schedule. No acceleration-feedforward term (112-002's actuationLag_/
// a_x/alpha staging, and the deleted planner-config message type it read
// the gain from, were deleted by 115-005 -- the gut's motion-stack
// excision) -- this is now a bare inverse()-then-setVelocity() follower
// with nothing else in it.
//
// fwdSign/port convention: each NezhaMotor leaf applies its OWN
// config_.fwdSign correction internally, at both the encoder-decode and
// duty-write boundary (nezha_motor.cpp's collectEncoder()/writeRawDuty()).
// Drive therefore works entirely in logical "positive = forward"
// body-relative mm/s and never touches fwdSign or the port-to-side
// mapping itself -- which NezhaMotor instance is "left" vs "right" is
// main.cpp's own construction-time wiring.
#pragma once

#include "devices/motor.h"

namespace App {

class Drive {
 public:
  // left/right -- the two drive-wheel NezhaMotor leaves, in BodyKinematics'
  // own L/R convention (inverse()'s vL_out/vR_out order). trackWidth --
  // [mm], BodyKinematics::inverse()'s own `b` parameter.
  Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth);

  // Stages the next tick()'s body twist target. v_y is accepted and
  // IGNORED (115-005: wire-forward for sprint 116's MoveTwist -- see
  // sprint.md Decision 5 -- the legacy Twist wire message carries no v_y
  // yet, so every call site through S1 passes 0 here). Does not itself
  // reach into the leaves -- tick() is the only method that ever calls
  // setVelocity().
  void setTwist(float v_x, float v_y, float omega);  // [mm/s] [mm/s] [rad/s]

  // Stages a zero twist -- the next tick() call computes inverse(0, 0, ...)
  // (both outputs exactly 0) and stages it onto both leaves.
  void stop();

  // Computes vL/vR via BodyKinematics::inverse(v_x, omega, trackWidth, vL,
  // vR) from the last staged twist and stages them onto the two leaves via
  // their own setVelocity() -- no additional scaling/sign logic here beyond
  // what inverse() already computes, and no feedforward term. Bounded: one
  // inverse() call, two setVelocity() calls, no I2C traffic, no sleeps.
  void tick();

  // trackWidth -- read-only accessor onto the same `b` BodyKinematics::
  // inverse() uses above (109-009: RobotLoop::updateTlm() needs it to fuse
  // the two leaves' measured velocities into the primary frame's `twist`
  // field via BodyKinematics::forward() -- see that method's own call
  // site). No setter: trackWidth_ is fixed at construction, matching
  // Drive's own "no live-reconfigure" contract.
  float trackWidth() const { return trackWidth_; }  // [mm]

 private:
  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  float v_x_ = 0.0f;    // [mm/s]
  float omega_ = 0.0f;  // [rad/s]
};

}  // namespace App
