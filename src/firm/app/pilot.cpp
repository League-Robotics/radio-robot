// pilot.cpp -- App::Pilot implementation. See pilot.h's file header for
// the module's boundary and cycle-placement contract.
#include "app/pilot.h"

namespace App {

void Pilot::tick(uint32_t now) {  // [ms]
  uint32_t dt = hasLastTick_ ? (now - lastTick_) : 0;
  lastTick_ = now;
  hasLastTick_ = true;
  float dtS = static_cast<float>(dt) / 1000.0f;  // [s]

  // HeadingSource is sampled every cycle, IDLE included -- see pilot.h's
  // own header comment.
  headingSource_.sample();

  Motion::Executor::Twist twist =
      executor_.tick(dt, odom_.lastDistance(), headingSource_.heading());

  float omega = twist.omega;
  if (twist.headingActive) {
    float thetaErr = twist.thetaRef - twist.thetaMeas;
    float omegaMeasEst =
        (hasPrevThetaMeas_ && dtS > 0.0f) ? (twist.thetaMeas - prevThetaMeas_) / dtS : 0.0f;
    omega += headingKp_ * thetaErr + headingKd_ * (twist.omegaDes - omegaMeasEst);
  }
  prevThetaMeas_ = twist.thetaMeas;
  hasPrevThetaMeas_ = true;

  if (executor_.state() != Motion::State::kIdle) {
    drive_.setTwist(twist.v, omega);
  }
}

}  // namespace App
