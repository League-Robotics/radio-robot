// test_support.h -- minimal CHECK macros plus an ideal test plant for
// Motion::Navigator ctests (135-003). Test-only scaffolding, never part
// of the navigator library.
//
// Self-contained: no dependency on src/firm/motion/planner/tests/test_support.h
// or on BodyKinematics/PoseTracker/Odometry -- this standalone project
// (motion_navigator) links Motion::Planner's own sources directly
// (CMakeLists.txt's motion_planner_lib) but keeps its OWN test harness
// independent, matching arc_solver_test.cpp's own "standalone, no reach
// into a sibling tests/ directory" convention.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "navigator.h"
#include "planner.h"
#include "types/robot_state.h"

#define CHECK(cond)                                                      \
  do {                                                                   \
    if (!(cond)) {                                                       \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
      std::exit(1);                                                      \
    }                                                                    \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                            \
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

namespace TestNav {

// Perfect-velocity wheel tracking (mirrors src/firm/motion/planner/tests/
// test_support.h's own PerfectPlant) PLUS an arc-exact world-pose
// integration (mirrors estimation.h's PoseTracker: "constant-curvature
// segment per step") -- self-contained, no BodyKinematics dependency.
// Publishes BOTH state.pose (encoder-sign -- the SUC-005 disconnect
// fallback source) and state.otos (WIRE/hardware-mounted sign: this
// negates `heading` on the way in, mirroring ticket 008's fix to
// TestSim::SimPlant::handleOtosRead() -- see navigator.h's own "OTOS sign
// convention" comment) every step. A scenario simulating a dropout
// overrides state.otos.connected = false AFTER calling step() -- step()
// itself always advances and publishes the ground truth.
struct IdealPlant {
  float positionLeft = 0.0f;   // [mm]
  float positionRight = 0.0f;  // [mm]
  float x = 0.0f;              // [mm] world, encoder-sign convention
  float y = 0.0f;              // [mm] world
  float heading = 0.0f;        // [rad] world, encoder-sign, unwrapped
  float trackWidth = 100.0f;   // [mm]

  void step(Types::RobotState& state, float dt, uint32_t sampleTime) {  // [s] [ms]
    const float vLeft = state.wheelLeft.cmdVelocity;
    const float vRight = state.wheelRight.cmdVelocity;
    positionLeft += vLeft * dt;
    positionRight += vRight * dt;

    const float ds = 0.5f * (vLeft + vRight) * dt;             // [mm]
    const float dtheta = (vRight - vLeft) / trackWidth * dt;   // [rad]
    // Threshold for "straight enough to skip the arc formula" is 1e-5 rad,
    // NOT arc_solver.cpp's own kMinBearing (1e-9) -- that guards a
    // DIFFERENT computation (a bearing used directly, no subtraction of
    // two close sin()/cos() evaluations of a LARGE angle). Here, the arc
    // branch computes sin(heading+dtheta)-sin(heading)/cos(...)-cos(...);
    // when `heading` is large (e.g. near PI after a pivot) and `dtheta` is
    // float-noise-scale (~1e-7, from an omega that is analytically zero
    // but numerically a few ULPs off), that subtraction is catastrophic
    // cancellation -- std::sin()/std::cos()'s own ~1e-7 relative error at
    // a large argument dwarfs a ~1e-9 dtheta, so the "arc-exact" branch
    // divides by dtheta (amplifying the cancellation noise by ~1/dtheta)
    // and returns garbage instead of the ~ds*cos(heading) any real
    // straight-enough step should give. Measured directly: this froze the
    // integrator's position update to (numerically) zero every tick
    // despite a genuine 150 mm/s commanded speed, in
    // testTargetBehindStopThenPivotThenArc's post-pivot cruise leg
    // (bearing/omega ~1.3e-7 rad/s, well inside float noise of a pivot
    // that lands heading at ~PI). 1e-5 is comfortably above that noise
    // floor and comfortably below any dtheta a genuine commanded turn
    // would ever produce in one 50 ms tick.
    if (std::fabs(dtheta) < 1e-5f) {
      x += ds * std::cos(heading);
      y += ds * std::sin(heading);
    } else {
      const float r = ds / dtheta;  // [mm]
      x += r * (std::sin(heading + dtheta) - std::sin(heading));
      y -= r * (std::cos(heading + dtheta) - std::cos(heading));
    }
    heading += dtheta;

    state.wheelLeft.position = positionLeft;
    state.wheelLeft.velocity = vLeft;
    state.wheelLeft.sampleTime = sampleTime;
    state.wheelLeft.connected = true;
    state.wheelRight.position = positionRight;
    state.wheelRight.velocity = vRight;
    state.wheelRight.sampleTime = sampleTime;
    state.wheelRight.connected = true;

    state.pose.x = x;
    state.pose.y = y;
    state.pose.heading = heading;

    state.otos.present = true;
    state.otos.connected = true;
    state.otos.x = x;
    state.otos.y = y;
    state.otos.heading = -heading;  // wire/hardware-mounted sign, ticket 008
    state.otos.sampleTime = sampleTime;
  }
};

// One closed-loop cycle: stamp time, tick the Navigator (which internally
// ticks/updates the Planner -- see navigator.h's own header comment), then
// let the plant integrate the staged wheel commands and publish fresh
// samples the NEXT cycle will see.
template <typename Plant>
inline Motion::NavResult cycle(Motion::Navigator& navigator, Types::RobotState& state,
                                Plant& plant, uint32_t& now, float periodMs) {
  state.time.cycleStart = now;
  const Motion::NavResult result = navigator.tick(state);
  now += static_cast<uint32_t>(periodMs);
  plant.step(state, periodMs * 0.001f, now);
  return result;
}

inline Motion::PlannerLimits defaultPlannerLimits() {
  Motion::PlannerLimits limits;
  limits.ceilings.vMax = 600.0f;        // [mm/s]
  limits.ceilings.aMax = 400.0f;        // [mm/s^2]
  limits.ceilings.aDecel = 300.0f;      // [mm/s^2]
  limits.ceilings.omegaMax = 8.0f;      // [rad/s]
  limits.ceilings.alphaMax = 12.0f;     // [rad/s^2]
  limits.ceilings.alphaDecel = 10.0f;   // [rad/s^2]
  limits.plant.trackWidth = 128.0f;     // [mm] matches defaultNavLimits()
  limits.plant.controlPeriod = 50.0f;   // [ms]
  return limits;
}

inline Motion::NavigatorLimits defaultNavLimits() {
  Motion::NavigatorLimits limits;
  limits.trackWidth = 128.0f;  // [mm]
  limits.speed = 150.0f;       // [mm/s]
  return limits;
}

}  // namespace TestNav
