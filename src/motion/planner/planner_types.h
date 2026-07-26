// planner_types.h -- the plain, motion-owned value types the Planner's
// public surface speaks: Move (what to do), PlannerLimits (construction
// config), TickResult (per-cycle completion event). No msg::*, no wire
// types -- the base's dispatch converts wire msg::Move -> Motion::Move at
// the boundary (motion-planner sketch §2). All structs are trivially
// copyable and C-layout so the ctypes harness mirrors them directly.
#pragma once

#include <cstdint>

namespace Motion {

// One bounded motion. Direction comes from the velocity sign; thresholds
// are positive magnitudes (protocol-v4 Move semantics, minus wire baggage).
struct Move {
  enum class Kind : uint8_t { Time, Distance, Angle };
  enum class VelocityKind : uint8_t { Twist, Wheels };

  uint32_t id = 0;
  Kind kind = Kind::Time;
  float threshold = 0.0f;  // [ms] Time / [mm] Distance / [rad] Angle; positive
  float timeout = 0.0f;    // [ms] safety backstop; 0 = none
  VelocityKind velocityKind = VelocityKind::Twist;
  float v_x = 0.0f;        // [mm/s] signed cruise, Twist
  float omega = 0.0f;      // [rad/s] signed cruise, Twist
  float vLeft = 0.0f;      // [mm/s] signed cruise, Wheels
  float vRight = 0.0f;     // [mm/s] signed cruise, Wheels
};

struct PlannerLimits {
  float vMax = 600.0f;          // [mm/s] linear velocity ceiling
  float aMax = 0.0f;            // [mm/s^2] linear accel-ramp ceiling
  float aDecel = 0.0f;          // [mm/s^2] linear decel-taper ceiling
  float omegaMax = 8.0f;        // [rad/s] angular velocity ceiling
  float alphaMax = 0.0f;        // [rad/s^2] angular accel-ramp ceiling
  float alphaDecel = 0.0f;      // [rad/s^2] angular decel-taper ceiling
  float trackWidth = 100.0f;    // [mm] wheel separation
  float controlPeriod = 50.0f;  // [ms] the loop interval one tick() plans for
  float actuationDelay = 0.0f;  // [ms] command-staged-to-wheels latency compensated in planning
  float velocityFilterWeight = 1.0f;  // EMA weight on fresh encoder velocity; 1 = unfiltered
  uint32_t otosStaleness = 200;       // [ms] max OTOS age still eligible to blend
  float headingOtosWeight = 0.0f;     // [0..1] complementary blend, fail-closed default
};

// The per-tick completion event, returned by tick() (never written into
// RobotState -- acks are protocol bookkeeping, not robot state). At most
// one Move can end per tick.
struct TickResult {
  bool completed = false;
  uint32_t moveId = 0;
  bool timedOut = false;
};

}  // namespace Motion
