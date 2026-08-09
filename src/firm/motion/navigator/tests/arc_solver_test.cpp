// arc_solver_test.cpp -- ctest coverage for Motion::ArcSolver::solve()
// (135-002), mirroring src/tests/unit/test_solver.py's own test surface
// (solveArcToPoint/_clampOmegaStep/SolverLimits/ArcSolution) closely enough
// to be a real parity check -- same representative poses/targets/
// tolerances, minus `pursuitTarget()` (out of scope for this ticket: it
// stays host-side, unchanged, per sprint 135's Out of Scope section).
//
// Standalone: no dependency on planner/tests/test_support.h (which pulls
// in planner.h/types/robot_state.h) -- this module has none of that, and
// its own test harness shouldn't either. Local CHECK/CHECK_NEAR macros
// only.
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "arc_solver.h"

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

using Motion::ArcSolution;
using Motion::NavigatorLimits;
using Motion::Pose;
using Motion::ArcSolver::solve;

namespace {

constexpr double kPi = 3.14159265358979323846;

const Pose kHere{0.0f, 0.0f, 0.0f};

// [mm] tovez.json drivetrain trackwidth (matches test_solver.py's own
// _TRACK_WIDTH -- same physical constant, ported test-for-test).
constexpr float kTrackWidth = 128.0f;
// [mm/s] matches ticket 001's own Edge-B measurement speed and
// test_solver.py's own _SPEED.
constexpr float kSpeed = 150.0f;

// Effectively-unclamped limits for tests that want to see the RAW arc
// geometry without the (separately, thoroughly tested) curvature slew
// clamp interfering. Mirrors test_solver.py's _UNCLAMPED.
NavigatorLimits unclampedLimits() {
  NavigatorLimits limits;
  limits.trackWidth = kTrackWidth;
  limits.speed = kSpeed;
  limits.maxWheelStep = 1.0e9f;
  return limits;
}

// The real, derived (default) limits -- used by the slew-limit tests.
// Mirrors test_solver.py's _DEFAULT, except maxWheelStep is THIS port's
// own derived default (125.0 mm/s per 50 ms solve), not solver.py's
// 250.0 mm/s per 100 ms solve -- see arc_solver.h's own derivation
// comment for why the two differ.
NavigatorLimits defaultLimits() {
  NavigatorLimits limits;
  limits.trackWidth = kTrackWidth;
  limits.speed = kSpeed;
  return limits;
}

// --- Derivation regression locks -------------------------------------------

void testMaxWheelStepDerivation() {
  CHECK_NEAR(Motion::kArcSolverMaxWheelStepDefault,
             Motion::kArcSolverSlewAccel * Motion::kArcSolverSolvePeriod, 1e-6);
  CHECK_NEAR(Motion::kArcSolverMaxWheelStepDefault, 125.0, 1e-6);
  // Still comfortably under ticket 001's measured Edge-B hazard figure
  // (433.3333 mm/s) -- the whole point of the derivation -- while sitting
  // above the firmware's own aMax = 300 mm/s^2 authority per solve
  // (300 * 0.05 = 15 mm/s), so aMax stays the binding constraint in
  // normal operation.
  CHECK(Motion::kArcSolverMaxWheelStepDefault < 433.3333f);
  CHECK(Motion::kArcSolverMaxWheelStepDefault > 300.0f * Motion::kArcSolverSolvePeriod);
}

void testDefaultBehindAngleIs90Degrees() {
  CHECK_NEAR(NavigatorLimits{}.behindAngle, kPi / 2.0, 1e-6);
}

void testDefaultTurnFirstAngleIs50Degrees() {
  CHECK_NEAR(NavigatorLimits{}.turnFirstAngle, 50.0 * kPi / 180.0, 1e-6);
}

// --- On-heading target ------------------------------------------------

void testOnHeadingTargetGivesZeroCurvature() {
  const Pose target{200.0f, 0.0f, 1.23f};  // 200mm straight ahead
  const ArcSolution result = solve(kHere, target, unclampedLimits(), 0.0f);
  CHECK(result.stop == false);
  CHECK_NEAR(result.omega, 0.0, 1e-6);
  CHECK_NEAR(result.arcLength, 200.0, 1e-3);
  CHECK_NEAR(result.v_x, kSpeed, 1e-6);
}

// --- 90-degree-offset target (hand-computed tangent-circle half-turn) --

void test90DegreeOffsetTarget() {
  // Target 100mm directly to the LEFT of the robot's heading. The tangent
  // circle through the origin and (0, 100mm) is a half-circle (radius
  // 50mm) -- "target at 90 degrees needs a 180-degree arc" (turn angle =
  // 2*bearing).
  const Pose target{0.0f, 100.0f, 0.0f};
  const ArcSolution result = solve(kHere, target, unclampedLimits(), 0.0f);
  CHECK(result.stop == false);
  CHECK_NEAR(result.omega, 2.0 * kSpeed / 100.0, 1e-4);  // kappa = 2/R, R = 50mm
  CHECK_NEAR(result.omega, 3.0, 1e-4);
  CHECK_NEAR(result.arcLength, 50.0 * kPi, 1e-3);  // R * turn_angle = 50 * pi
  CHECK(result.omega > 0.0f);  // target to the left -> positive (CCW) omega
}

// --- Moderate (30-degree) off-heading target, both sides ---------------

void testOffHeadingTargetModerateLeft() {
  const double bearing = kPi / 6.0;  // 30 degrees
  const double distance = 200.0;     // [mm]
  const Pose target{static_cast<float>(distance * std::cos(bearing)),
                     static_cast<float>(distance * std::sin(bearing)), 0.0f};
  const ArcSolution result = solve(kHere, target, unclampedLimits(), 0.0f);
  CHECK(result.stop == false);
  CHECK_NEAR(result.omega, kSpeed * 2.0 * std::sin(bearing) / distance, 1e-3);
  CHECK_NEAR(result.omega, 0.75, 1e-3);
  CHECK_NEAR(result.arcLength, distance * bearing / std::sin(bearing), 1e-2);
  CHECK_NEAR(result.arcLength, 209.4395102, 1e-2);
}

void testOffHeadingTargetSymmetricRightMirrorsLeft() {
  const double bearing = kPi / 6.0;
  const double distance = 200.0;
  const float dx = static_cast<float>(distance * std::cos(bearing));
  const Pose leftTarget{dx, static_cast<float>(distance * std::sin(bearing)), 0.0f};
  const Pose rightTarget{dx, static_cast<float>(-distance * std::sin(bearing)), 0.0f};
  const ArcSolution left = solve(kHere, leftTarget, unclampedLimits(), 0.0f);
  const ArcSolution right = solve(kHere, rightTarget, unclampedLimits(), 0.0f);
  CHECK_NEAR(right.omega, -left.omega, 1e-4);
  CHECK_NEAR(right.arcLength, left.arcLength, 1e-3);
  CHECK(left.arcLength > 0.0f);
  CHECK(right.arcLength > 0.0f);
}

// --- Target-behind guard -------------------------------------------------

void testTargetDirectlyBehindTriggersStop() {
  const Pose target{-200.0f, 0.0f, 0.0f};  // 200mm directly behind
  const ArcSolution result = solve(kHere, target, unclampedLimits(), 0.0f);
  CHECK(result.stop == true);
  CHECK_NEAR(result.v_x, 0.0, 1e-9);
  CHECK_NEAR(result.omega, 0.0, 1e-9);
  CHECK_NEAR(result.arcLength, 0.0, 1e-9);
  CHECK_NEAR(result.bearing, kPi, 1e-4);
}

void testTargetWellPastBehindThresholdTriggersStop() {
  // 120 degrees off-heading -- past the default 90-degree guard, still
  // short of directly behind. Must stop, not emit a degenerate arc.
  const double bearing = 120.0 * kPi / 180.0;
  const double distance = 200.0;
  const Pose target{static_cast<float>(distance * std::cos(bearing)),
                     static_cast<float>(distance * std::sin(bearing)), 0.0f};
  const ArcSolution result = solve(kHere, target, unclampedLimits(), 0.0f);
  CHECK(result.stop == true);
  CHECK_NEAR(result.bearing, bearing, 1e-4);
}

void testTargetJustInsideBehindThresholdDoesNotStop() {
  // 80 degrees off-heading -- still inside the default 90-degree guard.
  const double bearing = 80.0 * kPi / 180.0;
  const double distance = 200.0;
  const Pose target{static_cast<float>(distance * std::cos(bearing)),
                     static_cast<float>(distance * std::sin(bearing)), 0.0f};
  const ArcSolution result = solve(kHere, target, unclampedLimits(), 0.0f);
  CHECK(result.stop == false);
  CHECK(result.omega > 0.0f);
  CHECK(result.arcLength > 0.0f);
}

// --- Zero-distance degenerate case ----------------------------------------

void testTargetAtRobotsOwnPositionTriggersStop() {
  const Pose target{kHere.x, kHere.y, 2.5f};  // same position, any heading
  const ArcSolution result = solve(kHere, target, unclampedLimits(), 0.0f);
  CHECK(result.stop == true);
  CHECK_NEAR(result.v_x, 0.0, 1e-9);
  CHECK_NEAR(result.omega, 0.0, 1e-9);
  CHECK_NEAR(result.arcLength, 0.0, 1e-9);
  CHECK_NEAR(result.bearing, 0.0, 1e-9);
}

// --- Curvature slew limit -- ramp, never step -----------------------------
//
// maxOmegaStep = 2 * kArcSolverMaxWheelStepDefault / kTrackWidth
//              = 2 * 125.0 / 128.0 = 1.953125 rad/s
// A 90-degree-left target 25.6mm away has unclamped omega
// speed * 2*sin(90deg) / 25.6 = 150*2/25.6 = 11.71875 rad/s -- exactly
// 6x maxOmegaStep, chosen so the clamp visibly engages over an exact,
// verifiable number of steps.

const Pose kCloseLeftTarget{0.0f, 25.6f, 0.0f};
const Pose kCloseRightTarget{0.0f, -25.6f, 0.0f};
constexpr double kCloseUnclampedOmega = 11.71875;  // [rad/s]

void testSlewLimitClampsASingleLargeCurvatureStep() {
  const ArcSolution result = solve(kHere, kCloseLeftTarget, defaultLimits(), 0.0f);
  const double maxOmegaStep = 2.0 * Motion::kArcSolverMaxWheelStepDefault / kTrackWidth;
  CHECK_NEAR(result.omega, maxOmegaStep, 1e-6);
  // Nowhere near the naive/unclamped 11.71875 rad/s -- "ramp not step".
  CHECK(result.omega < kCloseUnclampedOmega);
}

void testSlewLimitRampsTowardTheTargetOverSuccessiveCalls() {
  const double maxOmegaStep = 2.0 * Motion::kArcSolverMaxWheelStepDefault / kTrackWidth;
  double previous = 0.0;
  int steps = 0;
  for (int i = 0; i < 64; ++i) {
    const ArcSolution result =
        solve(kHere, kCloseLeftTarget, defaultLimits(), static_cast<float>(previous));
    const double delta = result.omega - previous;
    CHECK(delta <= maxOmegaStep + 1e-9);
    CHECK(delta >= -1e-9);  // monotonically ramping toward the target here
    previous = result.omega;
    ++steps;
    if (previous >= kCloseUnclampedOmega - 1e-9) break;
  }
  CHECK_NEAR(previous, kCloseUnclampedOmega, 1e-3);
  // Converges in exactly the number of steps the budget predicts
  // (11.71875 / 1.953125 == 6), not in one jump.
  CHECK(steps == static_cast<int>(std::lround(kCloseUnclampedOmega / maxOmegaStep)));
  CHECK(steps > 1);
}

void testSlewLimitClampsAnAbruptReversal() {
  // Steady state at the close-left target's own unclamped omega (as if
  // converged toward it), then the target flips to the mirror-image right
  // target (naive unclamped omega is the negative of that). Must ramp
  // down, not jump straight to the naive reversed value.
  const double maxOmegaStep = 2.0 * Motion::kArcSolverMaxWheelStepDefault / kTrackWidth;
  const double previousOmega = kCloseUnclampedOmega;
  const ArcSolution result =
      solve(kHere, kCloseRightTarget, defaultLimits(), static_cast<float>(previousOmega));
  CHECK_NEAR(result.omega, previousOmega - maxOmegaStep, 1e-6);
  // Nowhere near the naive -11.71875 in a single call.
  CHECK(result.omega > 0.0f);
}

// --- Purity / final-heading-ignored ---------------------------------------

void testSameInputsProduceSameOutputs() {
  const Pose target{150.0f, 50.0f, 0.7f};
  const ArcSolution a = solve(kHere, target, unclampedLimits(), 0.1f);
  const ArcSolution b = solve(kHere, target, unclampedLimits(), 0.1f);
  CHECK_NEAR(a.v_x, b.v_x, 1e-9);
  CHECK_NEAR(a.omega, b.omega, 1e-9);
  CHECK_NEAR(a.arcLength, b.arcLength, 1e-9);
  CHECK(a.stop == b.stop);
  CHECK_NEAR(a.bearing, b.bearing, 1e-9);
}

void testTargetFinalHeadingIsIgnored() {
  const ArcSolution a = solve(kHere, Pose{150.0f, 50.0f, 0.0f}, unclampedLimits(), 0.0f);
  const ArcSolution b =
      solve(kHere, Pose{150.0f, 50.0f, static_cast<float>(kPi)}, unclampedLimits(), 0.0f);
  const ArcSolution c = solve(kHere, Pose{150.0f, 50.0f, -2.4f}, unclampedLimits(), 0.0f);
  CHECK_NEAR(a.omega, b.omega, 1e-9);
  CHECK_NEAR(a.omega, c.omega, 1e-9);
  CHECK_NEAR(a.arcLength, b.arcLength, 1e-9);
  CHECK_NEAR(a.arcLength, c.arcLength, 1e-9);
}

// --- Structural: never an Angle-stopped move ------------------------------
//
// A structured-binding decomposition compiles only if ArcSolution has
// EXACTLY these five public members, in this order. If a future edit adds
// a sixth field (e.g. a `stopAngle`), this line fails to COMPILE -- a
// stronger guard than test_solver.py's own runtime dataclasses.fields()
// introspection, which C++ has no equivalent of.
void testArcSolutionHasNoAngleStopField() {
  const ArcSolution solution{};
  const auto& [v_x, omega, arcLength, stop, bearing] = solution;
  (void)v_x;
  (void)omega;
  (void)arcLength;
  (void)stop;
  (void)bearing;
}

}  // namespace

int main() {
  testMaxWheelStepDerivation();
  testDefaultBehindAngleIs90Degrees();
  testDefaultTurnFirstAngleIs50Degrees();
  testOnHeadingTargetGivesZeroCurvature();
  test90DegreeOffsetTarget();
  testOffHeadingTargetModerateLeft();
  testOffHeadingTargetSymmetricRightMirrorsLeft();
  testTargetDirectlyBehindTriggersStop();
  testTargetWellPastBehindThresholdTriggersStop();
  testTargetJustInsideBehindThresholdDoesNotStop();
  testTargetAtRobotsOwnPositionTriggersStop();
  testSlewLimitClampsASingleLargeCurvatureStep();
  testSlewLimitRampsTowardTheTargetOverSuccessiveCalls();
  testSlewLimitClampsAnAbruptReversal();
  testSameInputsProduceSameOutputs();
  testTargetFinalHeadingIsIgnored();
  testArcSolutionHasNoAngleStopField();
  std::printf("PASS arc_solver_test (%d checks)\n", 17);
  return 0;
}
