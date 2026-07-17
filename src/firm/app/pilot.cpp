// pilot.cpp -- App::Pilot implementation. See pilot.h's file header for
// the module's boundary and cycle-placement contract.
#include "app/pilot.h"

namespace App {

void Pilot::tick(uint32_t now, uint64_t nowUs) {  // [ms] [us]
  uint32_t dt = hasLastTick_ ? (now - lastTick_) : 0;
  lastTick_ = now;
  hasLastTick_ = true;
  float dtS = static_cast<float>(dt) / 1000.0f;  // [s]

  // HeadingSource is sampled every cycle, IDLE included -- see pilot.h's
  // own header comment. nowUs (109-010) lets HeadingSource's own
  // measurement-age tracker compute the REAL elapsed time since Devices::
  // Otos's own cached pose was actually sampled, without either class
  // needing a Devices::Clock dependency of its own.
  headingSource_.sample(nowUs);

  Motion::Executor::Twist twist = executor_.tick(dt, odom_.lastDistance(), headingSource_.heading(),
                                                  headingSource_.headingLead());

  float omega = twist.omega;
  if (twist.headingActive) {
    // 109-010 locus 1: the PD's own error term uses thetaMeasLead (the
    // measurement-age-projected heading), not the raw thetaMeas -- see
    // Motion::Executor::Twist::thetaMeasLead's own doc comment. The rate
    // estimate below (omegaMeasEst) deliberately stays on the RAW,
    // continuous thetaMeas sequence -- thetaMeasLead's own age-tracked
    // offset resets to 0 on every fresh OTOS sample (App::HeadingSource's
    // own ageMs_ bookkeeping), which would inject a sawtooth into a
    // finite-difference derivative computed across it.
    float thetaErr = twist.thetaRef - twist.thetaMeasLead;
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
