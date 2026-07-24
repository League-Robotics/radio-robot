// drive.h -- App::Drive: the base's wheel-target sink. Stages a commanded
// per-wheel velocity target and applies it onto the two Devices::NezhaMotor
// leaves. Implements Motion::WheelSink (motion/wheel_sink.h) so
// Motion::MoveQueue can command it through that boundary interface rather
// than by concrete type.
//
// Boundary: inside -- staging vL/vR onto the two NezhaMotor leaves' own
// setVelocity() setter; outside -- the kinematics math (122-002: the twist
// decomposition -- BodyKinematics::inverse() -- moved to Motion::MoveQueue,
// which now calls the sink's setWheels() directly with already-decomposed
// wheel targets; Drive never sees a twist) and the deadman decision (the
// loop calls Drive::stop(); Drive never polls Deadman).
//
// Drive is a PURE velocity follower (115-005, gut S1; 122-002 narrows it
// further): setWheels()/stop() only STAGE a target; tick() stages the last
// setWheels() target onto the leaves via setVelocity() -- it never calls
// NezhaMotor::tick() itself, and it never touches the bus or sleeps, so the
// loop can call it from anywhere in its own schedule.
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
#include "motion/wheel_sink.h"

namespace App {

class Drive : public Motion::WheelSink {
 public:
  // left/right -- the two drive-wheel NezhaMotor leaves, in BodyKinematics'
  // own L/R convention (Motion::MoveQueue's own inverse()-derived vL/vR
  // order). trackWidth -- [mm], BodyKinematics::inverse()/forward()'s own
  // `b` parameter -- Drive no longer uses it for its own kinematics (122-002
  // moved that math to Motion::MoveQueue), but keeps holding/exposing it via
  // trackWidth() below, since App::RobotLoop::updateTlm() still needs the
  // SAME value to fuse the two leaves' measured velocities for telemetry
  // (BodyKinematics::forward()), and Drive is where that value has always
  // been constructed.
  Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth);

  // Stages v_left/v_right directly -- the ONE staging path left on Drive
  // (122-002: the twist path, setTwist()/BodyKinematics::inverse(), moved to
  // Motion::MoveQueue, which now calls this method with already-decomposed
  // wheel targets instead). Does not itself reach into the leaves -- tick()
  // is the only method that ever calls setVelocity().
  void setWheels(float v_left, float v_right) override;  // [mm/s] [mm/s]

  // Stages a zero target. The next tick() call stages exactly 0 onto both
  // leaves.
  void stop() override;

  // Stages the last setWheels()/stop() target onto the two leaves via their
  // own setVelocity(). Bounded: two setVelocity() calls, no I2C traffic, no
  // sleeps.
  void tick();

  // trackWidth -- read-only accessor (109-009: RobotLoop::updateTlm() needs
  // it to fuse the two leaves' measured velocities into the primary frame's
  // `twist` field via BodyKinematics::forward() -- see that method's own
  // call site). No setter: trackWidth_ is fixed at construction, matching
  // Drive's own "no live-reconfigure" contract.
  float trackWidth() const { return trackWidth_; }  // [mm]

 private:
  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  float vLeft_ = 0.0f;   // [mm/s]
  float vRight_ = 0.0f;  // [mm/s]
};

}  // namespace App
