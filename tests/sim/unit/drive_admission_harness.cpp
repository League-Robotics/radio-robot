// drive_admission_harness.cpp -- off-hardware acceptance harness for
// ticket 100-003 (SUC-003): exercises Drive::Drivetrain::admit()/plan()'s
// full Verdict enumerator table (all 8 values -- OK,
// EXIT_UNREACHABLE, JOINT_STEP_TOO_LARGE, JOINT_SIGN_REVERSAL,
// PIVOT_NONZERO_EXIT, RADIUS_TOO_TIGHT, CEILING_INFEASIBLE, SOLVE_FAILED),
// mirroring drive_master_profile_harness.cpp's/drive_arc_math_harness.cpp's
// compile-and-run pattern -- hand-rolled assertions, no gtest/pytest-native
// C++ framework, run via test_drive_admission.py.
//
// CEILING_INFEASIBLE and SOLVE_FAILED are produced by plan(), not admit()
// (admit() is a coarse, cheap queue-time feasibility estimate; plan() is
// the actual Ruckig solve under the folded ceiling) -- both are exercised
// here via Drivetrain::plan() so this ONE table covers every Verdict
// enumerator, per the ticket's own "admission-verdict table test exercises
// every Verdict enumerator" acceptance criterion.
//
// Every admit()-only scenario is constructed so exactly ONE check fires --
// admit()'s checks run in a fixed order (PIVOT_NONZERO_EXIT ->
// EXIT_UNREACHABLE -> [pivot short-circuits OK] -> JOINT_STEP_TOO_LARGE ->
// JOINT_SIGN_REVERSAL -> RADIUS_TOO_TIGHT), and each scenario below is
// deliberately built so every check BEFORE the one under test passes
// cleanly, per drivetrain.cpp's own comments at each check site.
#include <cmath>
#include <cstdio>
#include <string>

#include "drive/drivetrain.h"
#include "drive/types.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors drive_master_profile_harness.cpp) ---

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkVerdict(Drive::Verdict actual, Drive::Verdict expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected verdict %d, got %d", what.c_str(),
                  static_cast<int>(expected), static_cast<int>(actual));
    fail(buf);
  }
}

// --- Shared fixture: one representative, self-consistent Limits/Drivetrain ---

Drive::Limits defaultLimits() {
  Drive::Limits limits;
  limits.linear.velocity = 400.0f;      // [mm/s] vBodyMax
  limits.linear.accel = 800.0f;         // [mm/s^2]
  limits.linear.decel = 800.0f;         // [mm/s^2]
  limits.linear.jerk = 0.0f;            // trapezoid sentinel
  limits.rotational.velocity = 3.0f;    // [rad/s] omegaMax
  limits.rotational.accel = 15.0f;      // [rad/s^2]
  limits.rotational.decel = 15.0f;      // [rad/s^2]
  limits.rotational.jerk = 0.0f;
  limits.vWheelMax = 620.0f;            // [mm/s]
  limits.trimVMax = 120.0f;             // [mm/s]
  limits.trimOmegaMax = 1.0f;           // [rad/s]
  limits.wheelStepMax = 200.0f;         // [mm/s]
  return limits;
}

constexpr float kTrackwidth = 150.0f;  // [mm] -- halfTrack = 75mm

Drive::Goal straightGoal(float arcLength, float exitSpeed) {
  Drive::Goal goal;
  goal.arcLength = arcLength;
  goal.deltaHeading = 0.0f;
  goal.exitSpeed = exitSpeed;
  return goal;
}

Drive::ChainTail restTail() {
  Drive::ChainTail tail;
  tail.pose = Drive::Pose{0.0f, 0.0f, 0.0f};
  tail.exitSpeed = 0.0f;
  tail.kappa = 0.0f;
  return tail;
}

// --- Scenarios ---

void scenarioOkStraightFromRest() {
  beginScenario("OK: a feasible straight segment queued after a stopped tail");
  Drive::Drivetrain dt(defaultLimits(), kTrackwidth);
  Drive::Verdict v = dt.admit(straightGoal(500.0f, 0.0f), restTail());
  checkVerdict(v, Drive::Verdict::OK, "a plain 500mm stop segment from rest is admitted");
}

void scenarioOkPivotFromRest() {
  beginScenario("OK: a feasible pivot queued after a stopped tail");
  Drive::Drivetrain dt(defaultLimits(), kTrackwidth);
  Drive::Goal pivot;
  pivot.arcLength = 0.0f;
  pivot.deltaHeading = 1.0f;
  pivot.exitSpeed = 0.0f;
  Drive::Verdict v = dt.admit(pivot, restTail());
  checkVerdict(v, Drive::Verdict::OK, "a plain pivot from rest is admitted");
}

void scenarioPivotNonzeroExit() {
  beginScenario("PIVOT_NONZERO_EXIT: a pivot Goal with nonzero exitSpeed is rejected");
  Drive::Drivetrain dt(defaultLimits(), kTrackwidth);
  Drive::Goal pivot;
  pivot.arcLength = 0.0f;
  pivot.deltaHeading = 1.0f;
  pivot.exitSpeed = 50.0f;  // nonzero -- must be rejected, never silently clamped to 0
  Drive::Verdict v = dt.admit(pivot, restTail());
  checkVerdict(v, Drive::Verdict::PIVOT_NONZERO_EXIT,
               "a pivot with nonzero exitSpeed is PIVOT_NONZERO_EXIT");

  // Also via plan() directly -- plan() must independently reject this too
  // (it is not required to go through admit() first).
  Drive::PlanRequest req;
  req.goal = pivot;
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};
  Drive::PlanResult result = dt.plan(req);
  checkVerdict(result.verdict, Drive::Verdict::PIVOT_NONZERO_EXIT,
               "plan() also rejects a pivot with nonzero exitSpeed, never silently clamped");
}

void scenarioExitUnreachable() {
  beginScenario("EXIT_UNREACHABLE: a short arc cannot reach a large exit speed from rest");
  Drive::Drivetrain dt(defaultLimits(), kTrackwidth);
  // 50mm at 800 mm/s^2 accel cannot reach 400 mm/s from a standing start:
  // v^2 = 2*a*d => v_max = sqrt(2*800*50) ~= 283 mm/s < 400.
  Drive::Verdict v = dt.admit(straightGoal(50.0f, 400.0f), restTail());
  checkVerdict(v, Drive::Verdict::EXIT_UNREACHABLE,
               "50mm cannot reach 400mm/s exit speed from rest under 800mm/s^2 accel");
}

void scenarioJointStepTooLarge() {
  beginScenario("JOINT_STEP_TOO_LARGE: a sharp curvature jump at speed exceeds wheel_step_max");
  Drive::Drivetrain dt(defaultLimits(), kTrackwidth);

  Drive::ChainTail tail;
  tail.pose = Drive::Pose{0.0f, 0.0f, 0.0f};
  tail.exitSpeed = 300.0f;  // [mm/s] -- a flying joint
  tail.kappa = 0.0f;        // straight tail

  Drive::Goal goal;
  goal.arcLength = 300.0f;
  goal.deltaHeading = 0.02f * 300.0f;  // kappa = 0.02 [1/mm], radius 50mm
  goal.exitSpeed = 300.0f;             // same speed -- trivially reachable

  // jointStep = |300| * |0.02 - 0| * (150/2) = 300*0.02*75 = 450 > wheel_step_max(200).
  Drive::Verdict v = dt.admit(goal, tail);
  checkVerdict(v, Drive::Verdict::JOINT_STEP_TOO_LARGE,
               "300mm/s joint speed into kappa=0.02 exceeds the 200mm/s wheel step cap");
}

void scenarioJointSignReversal() {
  beginScenario("JOINT_SIGN_REVERSAL: a curvature jump flips a wheel's sign at LOW joint speed");
  Drive::Drivetrain dt(defaultLimits(), kTrackwidth);

  Drive::ChainTail tail;
  tail.pose = Drive::Pose{0.0f, 0.0f, 0.0f};
  tail.exitSpeed = 5.0f;  // deliberately small -- passes the step-size cap easily
  tail.kappa = 0.0f;      // straight tail: leftFactorOld = rightFactorOld = 1 (both positive)

  Drive::Goal goal;
  goal.arcLength = 300.0f;
  goal.deltaHeading = 0.02f * 300.0f;  // kappa = 0.02 [1/mm]: radius 50mm < halfTrack (75mm)
  goal.exitSpeed = 5.0f;               // same speed -- trivially reachable

  // leftFactorNew = 1 - 0.02*75 = -0.5 -- crosses zero from leftFactorOld=1: a sign flip.
  // jointStep = 5 * 0.02 * 75 = 7.5 <= 200 -- the step-size cap does NOT fire first.
  Drive::Verdict v = dt.admit(goal, tail);
  checkVerdict(v, Drive::Verdict::JOINT_SIGN_REVERSAL,
               "kappa 0 -> 0.02 (radius 50mm < halfTrack 75mm) flips the left wheel's sign");
}

void scenarioRadiusTooTight() {
  beginScenario("RADIUS_TOO_TIGHT: an arc entered at speed is tighter than the ~100mm floor");
  Drive::Drivetrain dt(defaultLimits(), kTrackwidth);

  // tail.kappa == newKappa: a continuation of the SAME curvature (no
  // step, no sign flip) -- isolates the radius-floor check on its own.
  const float kappa = 0.012f;  // radius = 1/0.012 ~= 83.3mm < 100mm floor
  Drive::ChainTail tail;
  tail.pose = Drive::Pose{0.0f, 0.0f, 0.0f};
  tail.exitSpeed = 50.0f;
  tail.kappa = kappa;

  Drive::Goal goal;
  goal.arcLength = 200.0f;
  goal.deltaHeading = kappa * 200.0f;
  goal.exitSpeed = 50.0f;  // same speed -- trivially reachable, no step, no sign flip

  Drive::Verdict v = dt.admit(goal, tail);
  checkVerdict(v, Drive::Verdict::RADIUS_TOO_TIGHT,
               "an 83.3mm-radius arc entered at 50mm/s is tighter than the 100mm floor");
}

void scenarioCeilingInfeasible() {
  beginScenario("CEILING_INFEASIBLE: plan() with a headroom that exceeds vWheelMax");
  Drive::Limits limits = defaultLimits();
  limits.trimVMax = 700.0f;  // headroom = 700 + 1.0*75 = 775 > vWheelMax(620)
  Drive::Drivetrain dt(limits, kTrackwidth);

  Drive::PlanRequest req;
  req.goal = straightGoal(500.0f, 0.0f);
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};
  Drive::PlanResult result = dt.plan(req);
  checkVerdict(result.verdict, Drive::Verdict::CEILING_INFEASIBLE,
               "a headroom (775) exceeding vWheelMax (620) leaves no wheel-speed budget");
}

void scenarioSolveFailed() {
  beginScenario("SOLVE_FAILED: plan() with an exit speed of the wrong sign for the direction");
  Drive::Drivetrain dt(defaultLimits(), kTrackwidth);

  Drive::PlanRequest req;
  // Positive-direction arc (arcLength > 0) with a NEGATIVE exit speed --
  // reachable only by reversing; the same-sign band (master_profile.h's
  // own class comment) rejects this cleanly.
  req.goal = straightGoal(300.0f, -100.0f);
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};
  Drive::PlanResult result = dt.plan(req);
  checkVerdict(result.verdict, Drive::Verdict::SOLVE_FAILED,
               "a positive-direction arc with a negative exit speed fails the same-sign band");
}

}  // namespace

int main() {
  scenarioOkStraightFromRest();
  scenarioOkPivotFromRest();
  scenarioPivotNonzeroExit();
  scenarioExitUnreachable();
  scenarioJointStepTooLarge();
  scenarioJointSignReversal();
  scenarioRadiusTooTight();
  scenarioCeilingInfeasible();
  scenarioSolveFailed();

  if (g_failureCount == 0) {
    std::printf("OK: all Drive::Drivetrain admission-verdict scenarios passed "
                "(all 8 Verdict enumerators exercised)\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Drive::Drivetrain admission-verdict scenarios\n",
              g_failureCount);
  return 1;
}
