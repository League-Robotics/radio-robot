// pilot.cpp -- App::Pilot implementation. See pilot.h's file header for
// the module's boundary and cycle-placement contract.
#include "app/pilot.h"

#include <cmath>

#include "kinematics/body_kinematics.h"

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

  // 111-003: captured BEFORE executor_.tick() so the twist-staging decision
  // below can tell "already idle before AND after this tick() call" (a
  // same-cycle flush -- see this method's own doc comment in pilot.h) apart
  // from "just transitioned running->idle INSIDE this tick() call" (a
  // natural completion, which must be zeroed exactly once).
  Motion::State stateBefore = executor_.state();

  Motion::Executor::Twist twist = executor_.tick(dt, odom_.lastDistance(), headingSource_.heading(),
                                                  headingSource_.headingLead());

  // 112-002: the PLANNED per-wheel reference -- BodyKinematics::inverse()
  // applied to twist.v/twist.omega EXACTLY as Executor emitted them, before
  // the heading-PD correction below (which only ever modifies the LOCAL
  // `omega` copy, never `twist.omega` itself) and before App::Drive's own
  // actuation-lag feedforward (Drive::tick(), a later, separate stage). See
  // refLeft()/refRight()'s own doc comment (pilot.h) for why this is a
  // live accessor rather than a wire telemetry field.
  BodyKinematics::inverse(twist.v, twist.omega, drive_.trackWidth(), refLeft_, refRight_);

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

  // 111-003 twist-staging decision (pilot.h's own tick() doc comment):
  //   - still running (or just started) -- stage the freshly-computed
  //     twist, unchanged existing behavior.
  //   - a natural running->idle transition happened INSIDE this tick()
  //     call (stateBefore was non-idle, executor_.state() is now kIdle) --
  //     stage a zero twist exactly once, so Drive stops commanding the
  //     PREVIOUS cycle's stale twist instead of creeping until the 300ms
  //     deadman lease force-stops it (robot_loop.cpp's kPilotDeadmanLease).
  //   - already idle BEFORE this tick() call (includes a same-cycle flush:
  //     RobotLoop::handleTwist()/handleStop() call Pilot::flush() BEFORE
  //     Pilot::tick() runs this same cycle, so stateBefore is already
  //     kIdle by the time it's sampled above) -- do nothing, matching
  //     today's "does nothing while kIdle" contract; a raw TWIST/STOP's
  //     own Drive::setTwist() call (already staged earlier this cycle by
  //     handleTwist()/handleStop()) must survive untouched.
  if (executor_.state() != Motion::State::kIdle) {
    // 112-002: aRef/alphaRef forward the SAME sample() result already
    // computed for v/omega above (never a separate solve) -- Drive::tick()
    // folds them into a model feedforward term (actuation_lag * a) on top
    // of the velocity target.
    drive_.setTwist(twist.v, omega, twist.aRef, twist.alphaRef);
  } else if (stateBefore != Motion::State::kIdle) {
    drive_.setTwist(0.0f, 0.0f);
  }
}

}  // namespace App
