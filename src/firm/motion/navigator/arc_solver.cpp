// arc_solver.cpp -- Motion::ArcSolver::solve() (135-002). See arc_solver.h
// for the full contract, including the curvature slew-clamp budget's own
// first-principles derivation (kArcSolverSlewAccel/kArcSolverSolvePeriod/
// kArcSolverMaxWheelStepDefault) -- this file applies that budget and
// implements the tangent-circle geometry itself.
#include "arc_solver.h"

#include <cmath>

namespace Motion {
namespace ArcSolver {

namespace {

// Below this distance [mm] the target is (numerically) the robot's own
// position -- no direction to aim an arc at. Matches solver.py's
// _MIN_DISTANCE.
constexpr float kMinDistance = 1e-6f;

// Below this bearing magnitude [rad] treat the target as exactly on
// heading -- avoids a 0/0 in the general tangent-arc formula. Matches
// solver.py's _MIN_BEARING.
constexpr float kMinBearing = 1e-9f;

// Clamp `omega`'s change from `previousOmega` so the per-wheel command
// delta it induces (differential-drive kinematics: a wheel's speed
// changes by |domega| * trackWidth / 2 for a curvature-only change at
// fixed forward speed) never exceeds `maxWheelStep` -- the curvature
// slew-clamp budget derived in arc_solver.h. A non-positive trackWidth
// disables the clamp (no meaningful wheel-speed budget to convert).
// Ported from solver.py's _clampOmegaStep, unchanged in its own math.
float clampOmegaStep(float omega, float previousOmega, float trackWidth, float maxWheelStep) {
  if (trackWidth <= 0.0f) return omega;
  const float maxOmegaStep = 2.0f * maxWheelStep / trackWidth;  // [rad/s]
  const float delta = omega - previousOmega;
  if (delta > maxOmegaStep) return previousOmega + maxOmegaStep;
  if (delta < -maxOmegaStep) return previousOmega - maxOmegaStep;
  return omega;
}

// Approach-speed taper -- near-target slowdown for TRACKING ACCURACY, not
// feasibility (Motion::Planner's own shapeLimits() elsewhere handles
// feasibility; this is a separate, purely distance-based ramp new to this
// C++ port, not present in solver.py, which emits one constant `speed`
// per call by design). `limits.approachRadius <= 0` disables the taper
// entirely, matching PlannerLimits' own fail-open convention for an
// unconfigured bound. Linear ramp from `speed` (at or beyond
// approachRadius) down to `approachSpeed` (at zero distance).
//
// This changes only the COMMANDED SPEED along the tangent arc, never the
// arc's own geometry: the returned omega is always speed * curvature,
// where curvature (2*sin(bearing)/distance) is a function of geometry
// alone -- tapering speed scales omega down proportionally, which keeps
// the robot on the exact same circle, just traversing it more slowly.
float taperSpeed(const NavigatorLimits& limits, float distance) {
  if (limits.approachRadius <= 0.0f) return limits.speed;
  if (distance >= limits.approachRadius) return limits.speed;
  const float t = distance / limits.approachRadius;  // [0, 1]
  return limits.approachSpeed + t * (limits.speed - limits.approachSpeed);
}

}  // namespace

ArcSolution solve(const Pose& pose, const Pose& target, const NavigatorLimits& limits,
                   float previousOmega) {
  const float dx = target.x - pose.x;  // [mm]
  const float dy = target.y - pose.y;  // [mm]

  // World -> body frame (forward = +x_b, left = +y_b), Pose::heading's own
  // CCW-positive convention.
  const float cosH = std::cos(pose.heading);
  const float sinH = std::sin(pose.heading);
  const float bodyX = dx * cosH + dy * sinH;
  const float bodyY = -dx * sinH + dy * cosH;

  const float distance = std::hypot(bodyX, bodyY);  // [mm]
  if (distance < kMinDistance) {
    // Target is (numerically) the robot's own position -- no direction to
    // aim an arc at.
    return ArcSolution{0.0f, 0.0f, 0.0f, true, 0.0f};
  }

  const float bearing = std::atan2(bodyY, bodyX);  // [rad]
  if (std::fabs(bearing) > limits.behindAngle) {
    // No finite-radius tangent arc reaches a point beyond the behind-guard
    // angle -- signal stop rather than emit an extreme, unrealizable arc.
    return ArcSolution{0.0f, 0.0f, 0.0f, true, bearing};
  }

  const float speed = taperSpeed(limits, distance);

  // Tangent-circle identities: for a circle tangent to the forward
  // direction at the origin, the turn angle is 2*bearing and the radius is
  // distance / (2*sin(bearing)), giving omega = speed * 2*sin(bearing) /
  // distance and arcLength = distance * bearing / sin(bearing) -- both
  // continuous through bearing == 0, handled as an explicit on-heading
  // special case to avoid a 0/0.
  float omega;
  float arcLength;
  if (std::fabs(bearing) < kMinBearing) {
    omega = 0.0f;
    arcLength = distance;
  } else {
    const float sinBearing = std::sin(bearing);
    omega = speed * 2.0f * sinBearing / distance;
    arcLength = distance * bearing / sinBearing;
  }

  // Curvature slew clamp: note the returned arcLength is computed for the
  // UNCLAMPED tangent arc, not the clamped trajectory -- intentional, not
  // an oversight, matching solver.py's own documented rationale: the
  // caller (Navigator) continuously re-solves and replaces the in-flight
  // Move, so any geometric drift the clamp introduces self-corrects on the
  // very next solve rather than needing to be reconciled here.
  omega = clampOmegaStep(omega, previousOmega, limits.trackWidth, limits.maxWheelStep);
  return ArcSolution{speed, omega, arcLength, false, 0.0f};
}

}  // namespace ArcSolver
}  // namespace Motion
