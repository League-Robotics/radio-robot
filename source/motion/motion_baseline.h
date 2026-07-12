// motion_baseline.h -- Motion::MotionBaseline: a plain POD snapshot of the
// robot's state variables at the moment a Subsystems::Planner goal starts
// being evaluated against its stop conditions.
//
// Ported concept (field-for-field) from source_old/control/StopCondition.h's
// MotionBaseline. Captured by Subsystems::Planner on the FIRST tick() call
// after a goal is staged (see planner.cpp) -- NOT inside apply(), since
// apply()'s locked signature (`apply(const msg::PlannerCommand&, uint32_t
// now)`) carries no observations to snapshot from. This is a deliberate
// adaptation forced by the new tree's "no stored Hal::Motor/Drivetrain/
// PoseEstimator reference; arguments only" discipline (see
// source/subsystems/planner.h's class comment) -- source_old's Planner could
// snapshot a cached HardwareState pointer inside apply(); this tree's Planner
// cannot, so the snapshot moves to tick(), which does receive observations.
#pragma once

#include <stdint.h>

namespace Motion {

// MotionBaseline -- values latched once, at goal start, and read (never
// mutated) by Motion::evaluateStopCondition() on every subsequent tick.
struct MotionBaseline {
  uint32_t t0 = 0;         // [ms] system time at goal start
  float enc0 = 0.0f;       // [mm] (encLeft + encRight) * 0.5 at start
  float encDiff0 = 0.0f;   // [mm] (encRight - encLeft) at start -- ROTATION stop
  float heading0 = 0.0f;   // [rad] fused pose heading at start
  float pose0X = 0.0f;     // [mm] fused pose X at start
  float pose0Y = 0.0f;     // [mm] fused pose Y at start

  // Commanded-direction signs, latched from the goal's own (v, omega) target
  // at the moment the baseline is captured (ported concept from
  // source_old/commands/MotionCommand.cpp's start(), 072-002). +-1.0, or 0.0
  // if the corresponding commanded component is exactly zero -- STOP_DISTANCE/
  // STOP_ROTATION fall back to an undirected |delta| magnitude when 0.0 (see
  // motion/stop_condition.cpp).
  float vSign = 0.0f;      // dimensionless: +1/-1/0
  float omegaSign = 0.0f;  // dimensionless: +1/-1/0
};

}  // namespace Motion
