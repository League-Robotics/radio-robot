// drive_plan_harness.cpp -- off-hardware acceptance harness for ticket
// 100-003 (SUC-001/SUC-003): exercises Drive::Drivetrain::plan()/replan()
// and Drive::MotionPlan::referenceAt(), mirroring drive_master_profile_
// harness.cpp's/drive_arc_math_harness.cpp's compile-and-run pattern --
// hand-rolled assertions, no gtest/pytest-native C++ framework, run via
// test_drive_plan.py.
//
// Scenarios:
//  (a) The v_eff/omega_eff fold invariant (SUC-003's core acceptance
//      criterion): across a sweep of curvatures, a pivot, and two
//      different Limits ("plateau x headroom" combinations), the max
//      sampled wheel speed of referenceAt(t) never exceeds
//      vWheelMax - headroom, for every t in [0, duration].
//  (b) plan() boundary conditions: a chain-inherited nonzero entrySpeed
//      seeds referenceAt(0); a nonzero exitSpeed arrives at referenceAt
//      (duration); a pivot's heading boundary lands exactly on
//      anchor.h + deltaHeading, at rest.
//  (c) referenceAt()'s closed-form composition matches an independent
//      dense NUMERICAL integration of the same sampled v/omega -- proving
//      the closed-form x/y/theta genuinely correspond to integrating the
//      master profile's own v(t)/omega(t), not an unrelated formula.
//  (d) replan() re-times the SAME anchored path from a measured mid-path
//      state (referenceAt(0) of the re-timed plan matches the measured
//      seed (s_meas, v_meas) within tolerance, same goal/anchor/kappa),
//      and fails cleanly (verdict != OK) on a backward ask (an overshoot
//      state reachable only by reversing).
#include <cmath>
#include <cstdio>
#include <string>

#include "drive/arc_math.h"
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

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

void checkVerdict(Drive::Verdict actual, Drive::Verdict expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected verdict %d, got %d", what.c_str(),
                  static_cast<int>(expected), static_cast<int>(actual));
    fail(buf);
  }
}

void checkLe(double actual, double bound, const std::string& what) {
  if (!(actual <= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected <= %g, got %g", what.c_str(), bound, actual);
    fail(buf);
  }
}

void checkNear(double actual, double expected, double tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected ~%g (tol %g), got %g", what.c_str(), expected,
                  tol, actual);
    fail(buf);
  }
}

constexpr float kTrackwidth = 150.0f;  // [mm]
constexpr float kHalfTrack = kTrackwidth * 0.5f;

Drive::Limits makeLimits(float vWheelMax, float trimVMax, float trimOmegaMax) {
  Drive::Limits limits;
  limits.linear.velocity = 400.0f;
  limits.linear.accel = 800.0f;
  limits.linear.decel = 800.0f;
  limits.linear.jerk = 0.0f;
  limits.rotational.velocity = 3.0f;
  limits.rotational.accel = 15.0f;
  limits.rotational.decel = 15.0f;
  limits.rotational.jerk = 0.0f;
  limits.vWheelMax = vWheelMax;
  limits.trimVMax = trimVMax;
  limits.trimOmegaMax = trimOmegaMax;
  limits.wheelStepMax = 200.0f;
  return limits;
}

// --- (a) v_eff / omega_eff fold invariant ---

void checkWheelInvariantForPlan(const Drive::MotionPlan& plan, float budget,
                                 const std::string& label) {
  constexpr int kSamples = 200;
  const float duration = plan.duration();
  double maxWheel = -1e18;
  for (int i = 0; i <= kSamples; ++i) {
    const float t = duration * static_cast<float>(i) / static_cast<float>(kSamples);
    const Drive::RefState ref = plan.referenceAt(t);
    double wheelLeft = 0.0;
    double wheelRight = 0.0;
    if (plan.isPivot()) {
      wheelLeft = -static_cast<double>(ref.omega) * kHalfTrack;
      wheelRight = static_cast<double>(ref.omega) * kHalfTrack;
    } else {
      const double kappa = plan.kappa();
      wheelLeft = ref.v * (1.0 - kappa * kHalfTrack);
      wheelRight = ref.v * (1.0 + kappa * kHalfTrack);
    }
    maxWheel = std::max(maxWheel, std::max(std::fabs(wheelLeft), std::fabs(wheelRight)));
  }
  checkLe(maxWheel, budget + 1.0, label + ": max |wheel(t)| stays within vWheelMax - headroom");
}

void scenarioVEffFoldInvariant() {
  beginScenario("v_eff/omega_eff fold invariant across a kappa x plateau x headroom sweep");

  struct LimitsCase {
    const char* name;
    float vWheelMax;
    float trimVMax;
    float trimOmegaMax;
  };
  const LimitsCase limitsCases[] = {
      {"A: vWheelMax=620, trimVMax=120, trimOmegaMax=1.0", 620.0f, 120.0f, 1.0f},
      {"B: vWheelMax=400, trimVMax=80, trimOmegaMax=0.5", 400.0f, 80.0f, 0.5f},
  };

  const float kappaSweep[] = {0.0f, 0.0015f, 0.004f, -0.002f};

  for (const LimitsCase& lc : limitsCases) {
    const Drive::Limits limits = makeLimits(lc.vWheelMax, lc.trimVMax, lc.trimOmegaMax);
    const float headroom = lc.trimVMax + lc.trimOmegaMax * kHalfTrack;
    const float budget = lc.vWheelMax - headroom;
    Drive::Drivetrain dt(limits, kTrackwidth);

    for (float kappa : kappaSweep) {
      Drive::PlanRequest req;
      req.goal.arcLength = 600.0f;
      req.goal.deltaHeading = kappa * 600.0f;
      req.goal.exitSpeed = 0.0f;
      req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

      Drive::PlanResult result = dt.plan(req);
      std::string label = std::string(lc.name) + " kappa=" + std::to_string(kappa);
      checkVerdict(result.verdict, Drive::Verdict::OK, label + ": plan() succeeds");
      if (result.verdict == Drive::Verdict::OK) {
        checkWheelInvariantForPlan(result.plan, budget, label);
      }
    }

    // Pivot case (kappa formally undefined -- exercised via the omega branch above).
    Drive::PlanRequest pivotReq;
    pivotReq.goal.arcLength = 0.0f;
    pivotReq.goal.deltaHeading = 2.0f;
    pivotReq.goal.exitSpeed = 0.0f;
    pivotReq.start = Drive::Pose{0.0f, 0.0f, 0.0f};
    Drive::PlanResult pivotResult = dt.plan(pivotReq);
    std::string pivotLabel = std::string(lc.name) + " pivot";
    checkVerdict(pivotResult.verdict, Drive::Verdict::OK, pivotLabel + ": plan() succeeds");
    if (pivotResult.verdict == Drive::Verdict::OK) {
      checkWheelInvariantForPlan(pivotResult.plan, budget, pivotLabel);
    }
  }
}

// --- (b) plan() boundary conditions ---

void scenarioBoundaryConditions() {
  beginScenario("plan() boundary conditions: entrySpeed seed, nonzero exitSpeed, pivot heading");

  Drive::Drivetrain dt(makeLimits(620.0f, 120.0f, 1.0f), kTrackwidth);

  // Chain-inherited entrySpeed seeds referenceAt(0).
  {
    Drive::PlanRequest req;
    req.goal.arcLength = 600.0f;
    req.goal.deltaHeading = 0.0f;
    req.goal.exitSpeed = 0.0f;
    req.start = Drive::Pose{0.0f, 0.0f, 0.0f};
    req.entrySpeed = 150.0f;
    req.entryAccel = 0.0f;

    Drive::PlanResult result = dt.plan(req);
    checkVerdict(result.verdict, Drive::Verdict::OK, "chain-inherited entrySpeed plan succeeds");
    if (result.verdict == Drive::Verdict::OK) {
      Drive::RefState ref0 = result.plan.referenceAt(0.0f);
      checkNear(ref0.v, 150.0, 1.0, "referenceAt(0).v matches the chain-inherited entrySpeed");
      checkNear(ref0.s, 0.0, 1e-2, "referenceAt(0).s starts at 0 (anchor)");
    }
  }

  // Nonzero exitSpeed arrives at referenceAt(duration).
  {
    Drive::PlanRequest req;
    req.goal.arcLength = 600.0f;
    req.goal.deltaHeading = 0.0f;
    req.goal.exitSpeed = 150.0f;
    req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

    Drive::PlanResult result = dt.plan(req);
    checkVerdict(result.verdict, Drive::Verdict::OK, "nonzero-exitSpeed plan succeeds");
    if (result.verdict == Drive::Verdict::OK) {
      Drive::RefState refEnd = result.plan.referenceAt(result.plan.duration());
      checkNear(refEnd.v, 150.0, 1.0, "referenceAt(duration).v arrives AT the exit speed");
      checkNear(refEnd.s, 600.0, 1.0, "referenceAt(duration).s arrives AT the arc length");
      checkNear(result.plan.exitSpeed(), 150.0, 1e-3, "MotionPlan::exitSpeed() matches the goal");
    }
  }

  // Pivot heading boundary: lands exactly on anchor.h + deltaHeading, at rest.
  {
    Drive::PlanRequest req;
    req.goal.arcLength = 0.0f;
    req.goal.deltaHeading = 1.2f;
    req.goal.exitSpeed = 0.0f;
    req.start = Drive::Pose{10.0f, -5.0f, 0.3f};

    Drive::PlanResult result = dt.plan(req);
    checkVerdict(result.verdict, Drive::Verdict::OK, "pivot plan succeeds");
    if (result.verdict == Drive::Verdict::OK) {
      checkTrue(result.plan.isPivot(), "MotionPlan::isPivot() is true for a pivot Goal");
      Drive::RefState refEnd = result.plan.referenceAt(result.plan.duration());
      checkNear(refEnd.theta, Drive::wrapAngle(0.3f + 1.2f), 1e-3,
                "pivot referenceAt(duration).theta lands on anchor.h + deltaHeading");
      checkNear(refEnd.omega, 0.0, 0.5, "pivot arrives at rest (omega ~ 0) at duration");
      checkNear(result.plan.goal().x, req.start.x, 1e-3, "pivot goal() keeps x unchanged");
      checkNear(result.plan.goal().y, req.start.y, 1e-3, "pivot goal() keeps y unchanged");
    }
  }
}

// --- (c) referenceAt() closed-form matches dense numerical integration ---

void scenarioReferenceAtMatchesNumericalIntegration() {
  beginScenario("referenceAt() closed-form matches a dense numerical integration of v/omega");

  Drive::Drivetrain dt(makeLimits(620.0f, 120.0f, 1.0f), kTrackwidth);
  Drive::PlanRequest req;
  req.goal.arcLength = 500.0f;
  req.goal.deltaHeading = 0.003f * 500.0f;  // a genuinely curved arc
  req.goal.exitSpeed = 0.0f;
  req.start = Drive::Pose{5.0f, 3.0f, 0.4f};

  Drive::PlanResult result = dt.plan(req);
  checkVerdict(result.verdict, Drive::Verdict::OK, "curved-arc plan succeeds");
  if (result.verdict != Drive::Verdict::OK) return;

  constexpr int kSteps = 4000;
  const float duration = result.plan.duration();
  const double dt_s = static_cast<double>(duration) / kSteps;

  double x = req.start.x;
  double y = req.start.y;
  double theta = req.start.h;
  for (int i = 0; i < kSteps; ++i) {
    const float t0 = duration * static_cast<float>(i) / kSteps;
    const float t1 = duration * static_cast<float>(i + 1) / kSteps;
    const Drive::RefState r0 = result.plan.referenceAt(t0);
    const Drive::RefState r1 = result.plan.referenceAt(t1);
    const double vAvg = (r0.v + r1.v) * 0.5;
    const double omegaAvg = (r0.omega + r1.omega) * 0.5;
    // Midpoint heading for a slightly better-than-Euler integration.
    const double thetaMid = theta + omegaAvg * dt_s * 0.5;
    x += vAvg * std::cos(thetaMid) * dt_s;
    y += vAvg * std::sin(thetaMid) * dt_s;
    theta += omegaAvg * dt_s;
  }

  const Drive::RefState refEnd = result.plan.referenceAt(duration);
  checkNear(x, refEnd.x, 2.0, "numerically-integrated x matches referenceAt(duration).x");
  checkNear(y, refEnd.y, 2.0, "numerically-integrated y matches referenceAt(duration).y");
  checkNear(Drive::wrapAngle(static_cast<float>(theta)), refEnd.theta, 1e-2,
            "numerically-integrated theta matches referenceAt(duration).theta");
}

// --- (d) replan() re-times to the same goal; fails cleanly on a backward ask ---

void scenarioReplanRoundTrip() {
  beginScenario("replan() re-times the same anchored path from a measured mid-path state");

  Drive::Drivetrain dt(makeLimits(620.0f, 120.0f, 1.0f), kTrackwidth);
  Drive::PlanRequest req;
  req.goal.arcLength = 600.0f;
  req.goal.deltaHeading = 0.002f * 600.0f;
  req.goal.exitSpeed = 0.0f;
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

  Drive::PlanResult original = dt.plan(req);
  checkVerdict(original.verdict, Drive::Verdict::OK, "original plan succeeds");
  if (original.verdict != Drive::Verdict::OK) return;

  const float elapsed = original.plan.duration() * 0.4f;
  const Drive::RefState ref = original.plan.referenceAt(elapsed);

  // Inject a known along-track offset (+12mm) and cross-track offset
  // (-4mm) via the reference's own tangent/normal frame -- mirrors
  // drive_arc_math_harness.cpp's own known-offset injection pattern.
  const float along = 12.0f;
  const float cross = -4.0f;
  const float cosT = std::cos(ref.theta);
  const float sinT = std::sin(ref.theta);
  Drive::BodyState measured;
  measured.pose.x = ref.x + along * cosT - cross * sinT;
  measured.pose.y = ref.y + along * sinT + cross * cosT;
  measured.pose.h = ref.theta;
  measured.twist.v_x = ref.v + 10.0f;  // a plausibly-different measured speed

  Drive::PlanResult replanned = dt.replan(original.plan, measured, elapsed);
  checkVerdict(replanned.verdict, Drive::Verdict::OK, "replan() succeeds from a mid-path state");
  if (replanned.verdict != Drive::Verdict::OK) return;

  checkNear(replanned.plan.goal().x, original.plan.goal().x, 1e-3,
            "replan() keeps the SAME frozen goal.x");
  checkNear(replanned.plan.goal().y, original.plan.goal().y, 1e-3,
            "replan() keeps the SAME frozen goal.y");
  checkNear(replanned.plan.kappa(), original.plan.kappa(), 1e-6,
            "replan() keeps the SAME curvature (never new geometry)");

  const float sMeas = ref.s + along;  // eAlong recovers `along` exactly (arc_math's own contract)
  const float vMeas = ref.v + 10.0f;
  Drive::RefState newRef0 = replanned.plan.referenceAt(0.0f);
  checkNear(newRef0.s, sMeas, 1e-1, "re-timed plan's referenceAt(0).s matches the measured seed");
  checkNear(newRef0.v, vMeas, 1e-1, "re-timed plan's referenceAt(0).v matches the measured seed");
}

void scenarioReplanBackwardAskFailsCleanly() {
  beginScenario("replan() fails cleanly (verdict != OK) on an ask reachable only by reversing");

  // A FLYING (nonzero-exitSpeed) segment, not a stop -- so the frozen exit
  // velocity itself (+100 mm/s, forward) is the thing that becomes
  // infeasible once the measured state has overshot the target: solving
  // from a NEGATIVE-direction seed (current position PAST the target)
  // toward a POSITIVE exit velocity violates master_profile.h's own
  // same-sign directional band (the SAME mechanism drive_admission_
  // harness.cpp's scenarioSolveFailed already proves for plan() itself --
  // here the direction flips at REPLAN time due to the overshoot, rather
  // than being mismatched from the start). Note: a still-forward SEED
  // velocity alone (with the target's own exit speed and sign otherwise
  // consistent, e.g. a stop segment's exitSpeed=0, which sits on the
  // boundary of BOTH direction bands) is not sufficient to trigger a
  // clean rejection -- Ruckig only validates the TARGET state against the
  // band, not the current/seed state -- confirmed empirically against
  // this harness before landing on the scenario below.
  Drive::Drivetrain dt(makeLimits(620.0f, 120.0f, 1.0f), kTrackwidth);
  Drive::PlanRequest req;
  req.goal.arcLength = 600.0f;
  req.goal.deltaHeading = 0.0f;
  req.goal.exitSpeed = 100.0f;  // a flying continuation, not a stop
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

  Drive::PlanResult original = dt.plan(req);
  checkVerdict(original.verdict, Drive::Verdict::OK, "original flying-segment plan succeeds");
  if (original.verdict != Drive::Verdict::OK) return;

  // A measured state that has OVERSHOT the 600mm target (by 50mm): the
  // direction (target - current) flips negative, but the frozen exit
  // velocity (+100) is still positive -- outside the flipped band.
  Drive::BodyState measured;
  measured.pose = Drive::composeArc(req.start, 0.0f, 650.0f);
  measured.twist.v_x = 80.0f;

  Drive::PlanResult replanned =
      dt.replan(original.plan, measured, original.plan.duration() * 0.5f);
  checkTrue(replanned.verdict != Drive::Verdict::OK,
            "an overshoot that flips the solve direction against the frozen exit speed fails "
            "(verdict != OK)");
}

}  // namespace

int main() {
  scenarioVEffFoldInvariant();
  scenarioBoundaryConditions();
  scenarioReferenceAtMatchesNumericalIntegration();
  scenarioReplanRoundTrip();
  scenarioReplanBackwardAskFailsCleanly();

  if (g_failureCount == 0) {
    std::printf("OK: all Drive::Drivetrain/MotionPlan plan scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Drive::Drivetrain/MotionPlan plan scenarios\n",
              g_failureCount);
  return 1;
}
