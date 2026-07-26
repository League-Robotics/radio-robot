// drive.h -- App::Drive: the base's wheel-target sink. Stages a commanded
// per-wheel target and applies it onto the two Devices::NezhaMotor leaves.
// Implements Motion::WheelSink (motion/wheel_sink.h) so Motion::MoveQueue
// can command it through that boundary interface rather than by concrete
// type.
//
// Boundary: inside -- staging vL/vR onto the two NezhaMotor leaves' own
// setVelocity() setter; outside -- the kinematics math (122-002: the twist
// decomposition -- BodyKinematics::inverse() -- moved to Motion::MoveQueue,
// which now calls the sink's setDuty() directly with already-decomposed
// wheel targets; Drive never sees a twist) and the deadman decision (the
// loop calls Drive::stop(); Drive never polls Deadman).
//
// 125-002 (RETOOL, PLACEHOLDER): Motion::WheelSink's boundary changed from
// a velocity sink to a duty sink (setWheels(v_left, v_right) ->
// setDuty(left, right)) -- a pure type/interface rename, no real duty
// semantics land here yet. setDuty() below is a placeholder pass-through
// (still stages whatever it is given, unclamped, exactly as setWheels()
// did) so the tree keeps compiling AND behaving identically -- the real
// implementation (the |duty|<=1/NaN->0 plausibility clamp, the
// App::WheelObserver pair, motor-ownership reshuffle) is ticket 007. Until
// then, callers still hand this method raw mm/s velocity values under the
// new duty-shaped name; that mismatch is deliberate and documented, not a
// silent lie -- ticket 006 is what starts actually passing real duty.
//
// Drive is a PURE velocity follower (115-005, gut S1; 122-002 narrows it
// further): setDuty()/stop() only STAGE a target; tick() stages the last
// setDuty() target onto the leaves via setVelocity() -- it never calls
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

  // Stages left/right directly -- the ONE staging path left on Drive
  // (122-002: the twist path, setTwist()/BodyKinematics::inverse(), moved to
  // Motion::MoveQueue, which now calls this method with already-decomposed
  // wheel targets instead). Does not itself reach into the leaves -- tick()
  // is the only method that ever calls setVelocity(). 125-002 PLACEHOLDER:
  // renamed from setWheels() (Motion::WheelSink's own retool) but still a
  // straight pass-through stage, unclamped -- see this file's own header
  // for why, and ticket 007 for the real duty implementation.
  void setDuty(float left, float right) override;  // [-1,1] [-1,1] (placeholder: unclamped)

  // Stages a zero target. The next tick() call stages exactly 0 onto both
  // leaves.
  void stop() override;

  // Stages the last setDuty()/stop() target onto the two leaves via their
  // own setVelocity(). Bounded: two setVelocity() calls, no I2C traffic, no
  // sleeps.
  void tick();

  // trackWidth -- read-only accessor (109-009: RobotLoop::updateTlm() needs
  // it to fuse the two leaves' measured velocities into the primary frame's
  // `twist` field via BodyKinematics::forward() -- see that method's own
  // call site). No setter: trackWidth_ is fixed at construction, matching
  // Drive's own "no live-reconfigure" contract.
  float trackWidth() const { return trackWidth_; }  // [mm]

  // vLeft/vRight -- read-only accessors onto the last-staged setDuty()/
  // stop() target (124-009): Types::RobotState::Wheel::cmdVelocity's own
  // source (robot_state.h's own doc comment: "writer: App::Drive::tick()'s
  // own staged target") -- RobotLoop::cycle() reads these immediately after
  // both wheels' collects to publish the wheel section's commanded-velocity
  // field, mirroring trackWidth()'s own read-only-accessor shape. No
  // setter, same reasoning as trackWidth(): staging stays exclusively
  // through setDuty()/stop().
  float vLeft() const { return vLeft_; }    // [mm/s] signed
  float vRight() const { return vRight_; }  // [mm/s] signed

 private:
  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  float vLeft_ = 0.0f;   // [mm/s]
  float vRight_ = 0.0f;  // [mm/s]
};

}  // namespace App
