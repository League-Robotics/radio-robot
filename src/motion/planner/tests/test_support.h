// test_support.h -- minimal CHECK macro plus the zero-error test plant
// (perfect velocity tracking, sketch §7 tier 2): each step integrates the
// staged wheel commands over one control interval and publishes fresh
// timestamped samples, exactly the discrete model the profiler plans
// against. Test-only scaffolding, never part of the planner library.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "planner.h"
#include "types/robot_state.h"

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
      std::exit(1);                                                       \
    }                                                                     \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                             \
  do {                                                                    \
    const double a_ = (a);                                                \
    const double b_ = (b);                                                \
    const double eps_ = (eps);                                            \
    if (!((a_ > b_ ? a_ - b_ : b_ - a_) <= eps_)) {                       \
      std::printf("FAIL %s:%d: |%s - %s| = %g > %g\n", __FILE__,          \
                  __LINE__, #a, #b, (a_ > b_ ? a_ - b_ : b_ - a_), eps_); \
      std::exit(1);                                                       \
    }                                                                     \
  } while (0)

namespace TestPlanner {

// Perfect-velocity plant: position advances exactly cmdVelocity * dt.
struct PerfectPlant {
  float positionLeft = 0.0f;   // [mm]
  float positionRight = 0.0f;  // [mm]

  // Jump the heading by `heading` without moving the body along its path
  // (equal and opposite wheel travel, so ds is exactly zero).
  void disturbHeading(float heading, float trackWidth) {  // [rad] [mm]
    positionLeft -= 0.5f * heading * trackWidth;
    positionRight += 0.5f * heading * trackWidth;
  }

  // Integrate the staged commands over one interval ending at `sampleTime`,
  // then publish fresh samples into the state.
  void step(Types::RobotState& state, float dt, uint32_t sampleTime) {  // [s] [ms]
    positionLeft += state.wheelLeft.cmdVelocity * dt;
    positionRight += state.wheelRight.cmdVelocity * dt;
    state.wheelLeft.position = positionLeft;
    state.wheelLeft.velocity = state.wheelLeft.cmdVelocity;
    state.wheelLeft.sampleTime = sampleTime;
    state.wheelLeft.connected = true;
    state.wheelRight.position = positionRight;
    state.wheelRight.velocity = state.wheelRight.cmdVelocity;
    state.wheelRight.sampleTime = sampleTime;
    state.wheelRight.connected = true;
  }
};

// Noisy/lagging plant (issue §7.1): the same perfect velocity tracking
// wrapped in the three defects a real drivetrain has, each independently
// switchable so a failure points at one cause.
//
//   noiseAmplitude / noiseWhite -- the PUBLISHED velocity sample is dirty;
//       the position sample stays honest (a real encoder's integral is far
//       cleaner than its derivative, which is the whole reason the planner
//       anchors distance to positions).
//   sampleDivisor -- a fresh sample lands only every N-th cycle; between
//       them the previous triple (SAME sampleTime) is re-seen, which is
//       what the WheelChannel's fresh-sample gate exists to survive.
//   delayedActuation -- the interval is driven by the PREVIOUS tick's
//       command, the one-cycle staging latency PlannerLimits::
//       actuationDelay compensates for.
//   trackingLag -- the wheel does not JUMP to the commanded velocity; it
//       approaches it first-order. This is the defect that actually breaks
//       exactness, because it is the one the planner's own plant model
//       (perfect velocity tracking) does not contain. Everything above it
//       the planner can, and does, reconstruct exactly.
//   positionQuantum -- the encoder reports whole ticks, not real numbers.
//   creepVelocity -- an unmodelled drive that never lets the body rest;
//       used to drive the settle window to expiry.
//   gainLeft / gainRight -- per-wheel scale between the COMMANDED velocity
//       and the velocity the wheel actually reaches. This is the one
//       defect a purely feedforward stack cannot correct and the velocity
//       trim exists for: the measured speed->duty study found the two
//       wheels ~2% apart in true gain, and any residual after calibration
//       lands here. Asymmetric values make the two wheels' remaining
//       distances genuinely diverge, which is what the ratio lock and the
//       trim are each tested against.
struct NoisyPlant {
  float noiseAmplitude = 0.0f;    // [mm/s] +- alternating, per published sample
  float noiseWhite = 0.0f;        // [mm/s] +- deterministic pseudo-random
  int sampleDivisor = 1;          // publish a fresh sample every N-th cycle
  bool delayedActuation = false;  // drive the interval with the previous command
  float trackingLag = 1.0f;       // [0..1] velocity response per interval; 1 = instant
  float positionQuantum = 0.0f;   // [mm] encoder resolution; 0 = continuous
  float creepVelocity = 0.0f;     // [mm/s] added to both wheels, always
  float gainLeft = 1.0f;          // [1] achieved / commanded, left wheel
  float gainRight = 1.0f;         // [1] achieved / commanded, right wheel

  float positionLeft = 0.0f;   // [mm] ground truth
  float positionRight = 0.0f;  // [mm] ground truth

  // Jump the heading by `heading` without moving the body along its path
  // (equal and opposite wheel travel, so ds is exactly zero).
  void disturbHeading(float heading, float trackWidth) {  // [rad] [mm]
    positionLeft -= 0.5f * heading * trackWidth;
    positionRight += 0.5f * heading * trackWidth;
  }

  void step(Types::RobotState& state, float dt, uint32_t sampleTime) {  // [s] [ms]
    const float targetLeft =
        (delayedActuation ? stagedLeft_ : state.wheelLeft.cmdVelocity) +
        creepVelocity;
    const float targetRight =
        (delayedActuation ? stagedRight_ : state.wheelRight.cmdVelocity) +
        creepVelocity;
    stagedLeft_ = state.wheelLeft.cmdVelocity;
    stagedRight_ = state.wheelRight.cmdVelocity;
    actualLeft_ += trackingLag * (targetLeft * gainLeft - actualLeft_);
    actualRight_ += trackingLag * (targetRight * gainRight - actualRight_);
    positionLeft += actualLeft_ * dt;
    positionRight += actualRight_ * dt;

    if (++cycles_ % sampleDivisor != 0) return;  // stale cycle: re-see the last
    ++publishes_;
    state.wheelLeft.position = quantize(positionLeft);
    state.wheelLeft.velocity = actualLeft_ + noise();
    state.wheelLeft.sampleTime = sampleTime;
    state.wheelLeft.connected = true;
    state.wheelRight.position = quantize(positionRight);
    state.wheelRight.velocity = actualRight_ + noise();
    state.wheelRight.sampleTime = sampleTime;
    state.wheelRight.connected = true;
  }

 private:
  float quantize(float position) const {  // [mm]
    if (positionQuantum <= 0.0f) return position;
    return std::floor(position / positionQuantum) * positionQuantum;
  }

  float noise() {  // [mm/s]
    const float zigZag = (publishes_ % 2 == 0) ? noiseAmplitude
                                               : -noiseAmplitude;
    lcg_ = lcg_ * 1103515245u + 12345u;  // deterministic; tests must repeat
    const float unit =
        static_cast<float>((lcg_ >> 16) & 0xFFFFu) / 32767.5f - 1.0f;
    return zigZag + unit * noiseWhite;
  }

  float stagedLeft_ = 0.0f;   // [mm/s] last command handed to the plant
  float stagedRight_ = 0.0f;  // [mm/s]
  float actualLeft_ = 0.0f;   // [mm/s] what the wheel is really doing
  float actualRight_ = 0.0f;  // [mm/s]
  int cycles_ = 0;
  int publishes_ = 0;
  uint32_t lcg_ = 1u;
};

// One closed-loop cycle: stamp time, tick (compute), update (save), then
// let the plant integrate the staged command over the interval and publish
// the samples the NEXT tick will see. Returns tick()'s completion event.
template <typename Plant>
inline Motion::TickResult cycle(Motion::Planner& planner, Types::RobotState& state,
                                Plant& plant, uint32_t& now,
                                float periodMs) {
  state.time.cycleStart = now;
  const Motion::TickResult result = planner.tick(state);
  planner.update(state);
  now += static_cast<uint32_t>(periodMs);
  plant.step(state, periodMs * 0.001f, now);
  return result;
}

inline Motion::PlannerLimits benchLimits() {
  Motion::PlannerLimits limits;
  limits.ceilings.vMax = 600.0f;         // [mm/s]
  limits.ceilings.aMax = 400.0f;         // [mm/s^2]
  limits.ceilings.aDecel = 300.0f;       // [mm/s^2]
  limits.ceilings.omegaMax = 8.0f;       // [rad/s]
  limits.ceilings.alphaMax = 12.0f;      // [rad/s^2]
  limits.ceilings.alphaDecel = 10.0f;    // [rad/s^2]
  limits.plant.trackWidth = 100.0f;      // [mm]
  limits.plant.controlPeriod = 50.0f;    // [ms]
  return limits;
}

}  // namespace TestPlanner
