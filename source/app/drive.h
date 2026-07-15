// drive.h -- App::Drive: converts a body twist into per-wheel velocity
// targets and stages them onto the two Devices::NezhaMotor leaves.
//
// architecture-update.md (103) Step 3 "Drive" boundary: inside -- staging
// vL/vR from BodyKinematics::inverse() onto the two NezhaMotor leaves' own
// setVelocity() setter; outside -- the kinematics math itself (stays in
// BodyKinematics, unchanged this sprint) and the deadman decision (the loop
// calls Drive::stop(); Drive never polls Deadman). Serves SUC-006.
//
// Ticket 103-006. Drive is a PURE velocity follower: setTwist()/stop() only
// STAGE a target; tick() computes wheel targets and stages them onto the
// leaves via setVelocity() -- it never calls NezhaMotor::tick() itself. The
// leaves' own request/collect/PID cycle is serviced by the loop directly
// (ticket 008); Drive::tick() is a bounded, single-purpose staging step
// (one inverse() call, two setVelocity() calls) with no I2C traffic and no
// internal sleeps, so the loop can call it from anywhere in its own
// schedule.
//
// fwdSign/port convention (confirmed against nezha_motor.cpp during this
// ticket's own implementation -- see .clasi/knowledge/tovez-fwd-sign-and-
// port-swap.md): each NezhaMotor leaf applies its OWN config_.fwdSign
// correction internally, at both the encoder-decode and duty-write boundary
// (nezha_motor.cpp's collectEncoder()/writeRawDuty()). Drive therefore
// works entirely in logical "positive = forward" body-relative mm/s and
// never touches fwdSign or the port-to-side mapping itself -- which
// NezhaMotor instance is "left" vs "right" is main.cpp's own construction-
// time wiring (ticket 008). This is exactly what the ticket's own
// acceptance criterion means by "no additional scaling/sign logic
// duplicated in Drive beyond what inverse() already computes."
#pragma once

#include "devices/nezha_motor.h"

namespace App {

class Drive {
 public:
  // left/right -- the two drive-wheel NezhaMotor leaves, in BodyKinematics'
  // own L/R convention (inverse()'s vL_out/vR_out order). trackWidth --
  // [mm], BodyKinematics::inverse()'s own `b` parameter; the loop's own
  // construction (ticket 008) passes
  // Config::defaultDrivetrainConfig().trackwidth.
  Drive(Devices::NezhaMotor& left, Devices::NezhaMotor& right, float trackWidth);

  // Stages the next tick()'s body twist target. Does not itself reach into
  // the leaves -- tick() is the only method that ever calls setVelocity().
  void setTwist(float v_x, float omega);  // [mm/s] [rad/s]

  // Stages a zero twist -- the next tick() call computes inverse(0, 0, ...)
  // (both outputs exactly 0) and stages it onto both leaves.
  void stop();

  // Computes vL/vR via BodyKinematics::inverse(v_x, omega, trackWidth, vL,
  // vR) from the last staged twist and stages them onto the two leaves via
  // their own setVelocity() -- no additional scaling/sign logic here beyond
  // what inverse() already computes. Bounded: one inverse() call, two
  // setVelocity() calls, no I2C traffic, no sleeps.
  void tick();

 private:
  Devices::NezhaMotor& left_;
  Devices::NezhaMotor& right_;
  float trackWidth_;  // [mm]

  float v_x_ = 0.0f;    // [mm/s]
  float omega_ = 0.0f;  // [rad/s]
};

}  // namespace App
