// test_support.h -- minimal CHECK macro plus the zero-error test plant
// (perfect velocity tracking, sketch §7 tier 2): each step integrates the
// staged wheel commands over one control interval and publishes fresh
// timestamped samples, exactly the discrete model the profiler plans
// against. Test-only scaffolding, never part of the planner library.
#pragma once

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

  // Integrate the staged commands over one interval ending at `sampleTime`,
  // then publish fresh samples into the state.
  void step(RobotState& state, float dt, uint32_t sampleTime) {  // [s] [ms]
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

// One closed-loop cycle: stamp time, tick (compute), update (save), then
// let the plant integrate the staged command over the interval and publish
// the samples the NEXT tick will see. Returns tick()'s completion event.
inline Motion::TickResult cycle(Motion::Planner& planner, RobotState& state,
                                PerfectPlant& plant, uint32_t& now,
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
  limits.vMax = 600.0f;         // [mm/s]
  limits.aMax = 400.0f;         // [mm/s^2]
  limits.aDecel = 300.0f;       // [mm/s^2]
  limits.omegaMax = 8.0f;       // [rad/s]
  limits.alphaMax = 12.0f;      // [rad/s^2]
  limits.alphaDecel = 10.0f;    // [rad/s^2]
  limits.trackWidth = 100.0f;   // [mm]
  limits.controlPeriod = 50.0f; // [ms]
  return limits;
}

}  // namespace TestPlanner
