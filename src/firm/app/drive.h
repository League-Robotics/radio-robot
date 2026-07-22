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
  // setVelocity(). Last-wins: makes the twist path the one tick() computes
  // from, superseding whatever setWheels() staged before it.
  void setTwist(float v_x, float v_y, float omega);  // [mm/s] [mm/s] [rad/s]

  // 116-004: a SECOND, independent staging path alongside setTwist() --
  // stages v_left/v_right directly, bypassing BodyKinematics::inverse()
  // entirely (see this file's own header comment for the rationale: a
  // MoveWheels command tells the robot exactly what wheel speeds it wants,
  // and a forward/inverse round trip buys nothing on today's differential
  // base). Does not itself reach into the leaves -- tick() is the only
  // method that ever calls setVelocity(). Last-wins: makes the wheels path
  // the one tick() computes from, superseding whatever setTwist() staged
  // before it.
  void setWheels(float v_left, float v_right);  // [mm/s] [mm/s]

  // Stages a zero target on BOTH staging paths -- the next tick() call
  // stages exactly 0 onto both leaves regardless of which path (twist or
  // wheels) was last active.
  void stop();

  // Computes the two leaves' targets from whichever of setTwist()/
  // setWheels() was called most recently (last-wins) and stages them onto
  // the two leaves via their own setVelocity(). The twist path computes vL/
  // vR via BodyKinematics::inverse(v_x, omega, trackWidth, vL, vR) -- no
  // additional scaling/sign logic beyond what inverse() already computes,
  // and no feedforward term; the wheels path stages v_left/v_right
  // unchanged, with no inverse() call at all. Bounded: at most one
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
  // Which staging path tick() computes from next -- last-wins between
  // setTwist() and setWheels(). stop() zeroes both underlying targets but
  // leaves this mode untouched: with both paths at zero, either mode stages
  // (0, 0) onto the leaves, so which mode is "active" post-stop() has no
  // observable effect.
  enum class TargetKind { kTwist, kWheels };

  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  TargetKind targetKind_ = TargetKind::kTwist;

  float v_x_ = 0.0f;    // [mm/s] twist path
  float omega_ = 0.0f;  // [rad/s] twist path

  float vLeft_ = 0.0f;   // [mm/s] wheels path
  float vRight_ = 0.0f;  // [mm/s] wheels path
};

}  // namespace App
