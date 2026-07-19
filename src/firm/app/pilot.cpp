// pilot.cpp -- App::Pilot implementation. See pilot.h's file header for
// the module's boundary and cycle-placement contract.
#include "app/pilot.h"

#include <cmath>

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

    // Minimum-command floor (2026-07-18, terminal stiction/deadband): once
    // the planned profile has ended (omegaDes ~ 0 -- pure PD phase), a
    // small residual error times kp can command a per-wheel speed BELOW
    // what actually moves the plant (the write shaping's output deadband
    // clamps sub-0.03 duty to zero; real motors add stiction) -- the PD
    // then stalls with the error frozen ABOVE the dwell tolerance and the
    // command runs to the STOP_TIME backstop instead of completing
    // (observed directly in sim: kp=1 froze 5.7deg out, kp=6 froze ~1deg
    // out). Floor the PD's output at the omega whose wheel speed is
    // PlannerConfig.min_speed (sign preserved) so it always closes into
    // tolerance; once within tolerance the terminal-decel gate turns
    // headingActive off and the floor with it. Gated on omegaDes ~ 0 so a
    // live profile's own smooth jerk-limited ramp is never floored.
    // Gated OFF inside the dwell tolerance band (twist.withinTolerance):
    // the floor drives the approach, then disengages so the plant coasts
    // to rest IN the band -- flooring inside the band bang-bangs straight
    // through it (floor speed x plant decay exceeds the band width) and
    // the dwell never settles (observed: 15+ sign flips, then timeout).
    if (minSpeed_ > 0.0f && std::fabs(twist.omegaDes) < 1e-3f &&
        !twist.withinTolerance && omega != 0.0f) {
      float trackWidth = drive_.trackWidth();
      if (trackWidth > 0.0f) {
        float minOmega = 2.0f * minSpeed_ / trackWidth;   // [rad/s]
        if (std::fabs(omega) < minOmega) omega = std::copysign(minOmega, omega);
      }
    }
  }
  prevThetaMeas_ = twist.thetaMeas;
  hasPrevThetaMeas_ = true;

  if (executor_.state() != Motion::State::kIdle) {
    drive_.setTwist(twist.v, omega);
  }
}

}  // namespace App
