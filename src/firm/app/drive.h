// drive.h -- App::Drive: converts a body twist into per-wheel velocity
// targets and stages them onto the two Devices::NezhaMotor leaves.
//
// Boundary: inside -- staging vL/vR from BodyKinematics::inverse() onto
// the two NezhaMotor leaves' own setVelocity() setter; outside -- the
// kinematics math itself (stays in BodyKinematics) and the deadman
// decision (the loop calls Drive::stop(); Drive never polls Deadman).
//
// Drive is a PURE velocity follower: setTwist()/stop() only STAGE a
// target; tick() computes wheel targets and stages them onto the leaves
// via setVelocity() -- it never calls NezhaMotor::tick() itself. The
// leaves' own request/collect/PID cycle is serviced by the loop directly;
// Drive::tick() is a bounded, single-purpose staging step (112-002: two
// inverse() calls -- velocity, then acceleration for the model feedforward
// term -- two setVelocity() calls) with no I2C traffic and no internal
// sleeps, so the loop can call it from anywhere in its own schedule.
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
#include "messages/planner.h"

namespace App {

class Drive {
 public:
  // left/right -- the two drive-wheel NezhaMotor leaves, in BodyKinematics'
  // own L/R convention (inverse()'s vL_out/vR_out order). trackWidth --
  // [mm], BodyKinematics::inverse()'s own `b` parameter; the loop's own
  // construction (ticket 008) passes
  // Config::defaultDrivetrainConfig().trackwidth.
  Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth);

  // configure -- 112-002: reads actuation_lag (the acceleration-
  // feedforward model gain tick() uses below). Mirrors Motion::Executor::
  // configure()/App::HeadingSource::configure()'s own "call once, before
  // first use" convention; main.cpp's boot wiring calls this once. Not
  // calling it at all leaves actuationLag_ at its 0.0f default, i.e. the
  // feedforward term is a no-op -- matching every other DEFAULTED,
  // additive-only piece of this ticket.
  void configure(const msg::PlannerConfig& config);

  // Stages the next tick()'s body twist target -- v_x/omega, the commanded
  // rate, PLUS (112-002, both DEFAULTED so every pre-existing 2-arg call
  // site, e.g. RobotLoop::handleTwist()'s raw TWIST path, compiles and
  // behaves unchanged) a_x/alpha, the SAME instant's already-solved
  // acceleration (Motion::Executor::Twist::aRef/alphaRef, forwarded via
  // App::Pilot::tick()) tick() below folds into a model feedforward term.
  // Does not itself reach into the leaves -- tick() is the only method that
  // ever calls setVelocity().
  void setTwist(float v_x, float omega, float a_x = 0.0f, float alpha = 0.0f);  // [mm/s] [rad/s] [mm/s^2] [rad/s^2]

  // Stages a zero twist -- the next tick() call computes inverse(0, 0, ...)
  // (both outputs exactly 0) and stages it onto both leaves.
  void stop();

  // Computes vL/vR via BodyKinematics::inverse(v_x, omega, trackWidth, vL,
  // vR) from the last staged twist and stages them onto the two leaves via
  // their own setVelocity() -- no additional scaling/sign logic here beyond
  // what inverse() already computes, PLUS (112-002) a model feedforward
  // term: aL/aR are computed via the SAME inverse() map from the last
  // staged a_x/alpha (kinematics is linear, so reusing inverse() for
  // acceleration is exact -- aL = a_x - alpha*b/2, aR = a_x + alpha*b/2),
  // and actuationLag_ * aL / actuationLag_ * aR are added onto vL/vR before
  // staging -- a lead-compensation term anticipating where the velocity
  // target will need to be actuationLag_ seconds from now, given the
  // already-solved reference acceleration. a_x/alpha default to 0 (see
  // setTwist() above), so this is a no-op for any caller that never passes
  // them. Bounded: two inverse() calls, two setVelocity() calls, no I2C
  // traffic, no sleeps.
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
  float a_x_ = 0.0f;    // [mm/s^2] 112-002
  float alpha_ = 0.0f;  // [rad/s^2] 112-002

  // actuationLag_ -- 112-002: msg::PlannerConfig.actuation_lag, the model
  // feedforward gain tick() multiplies aL/aR by. 0.0f default (configure()
  // never called) makes the feedforward term an exact no-op.
  float actuationLag_ = 0.0f;  // [s]
};

}  // namespace App
