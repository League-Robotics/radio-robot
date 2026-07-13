// drive_tracker_harness.cpp -- off-hardware acceptance harness for ticket
// 100-004 (SUC-004): exercises Drive::track() (source/drive/tracker.{h,
// cpp}) in isolation, mirroring drive_arc_math_harness.cpp's/drive_plan_
// harness.cpp's compile-and-run pattern -- hand-rolled assertions, no
// gtest/pytest-native C++ framework, run via test_drive_tracker.py.
//
// Scenarios:
//  (a) trim signs across all four (eAlong, eCross) quadrants -- small,
//      non-saturating offsets injected via the SAME tangent/normal
//      technique as drive_arc_math_harness.cpp's scenario (d); verifies
//      vTrim/omegaTrim match the RECONCILED "errors reference-measured"
//      sign convention exactly (tracker.h's own class comment).
//  (b) reverse travel: a negative ref.v with a known eCross offset --
//      confirms the k_c*v_ref*e_cross cross term uses the SIGNED (not
//      absolute) v_ref.
//  (c) pivot mode: v_cmd is a LITERAL 0.0f (bit-exact, not merely near-
//      zero); omega's trim is UNCLAMPED even for a huge heading error
//      (trimSaturated stays false).
//  (d) trimSaturated exactly true iff a trim was clamped -- three cases:
//      only vTrim saturates, only omegaTrim saturates, neither saturates.
//  (e) property/fuzz: the one-sided forward-arc wheel clamp holds across
//      a wide deterministic grid of trim/error inputs (including
//      deliberately-saturating ones) -- neither wheel is ever negative on
//      a forward arc.
//  (f) closed-loop convergence: a minimal first-order plant stub, both for
//      an arc (lateral + heading offset against a straight reference) and
//      a pivot (heading-only offset) -- tracked error shrinks over time.
//      Ticket-scoped per the ticket's own "superseded once ticket 006
//      lands" note (ticket 006's real plant model has not landed yet).
#include <cmath>
#include <cstdio>
#include <string>

#include "drive/arc_math.h"
#include "drive/motion_plan.h"
#include "drive/tracker.h"
#include "drive/types.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors drive_arc_math_harness.cpp) ---

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

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkNear(double actual, double expected, double tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected ~%g (tol %g), got %g", what.c_str(), expected,
                  tol, actual);
    fail(buf);
  }
}

void checkExactly(double actual, double expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected EXACTLY %g, got %g", what.c_str(), expected,
                  actual);
    fail(buf);
  }
}

// --- Shared fixtures ---

const float kTrackwidth = 128.0f;  // [mm]

Drive::Limits makeLimits() {
  Drive::Limits limits;
  limits.vWheelMax = 600.0f;      // [mm/s]
  limits.trimVMax = 120.0f;       // [mm/s]      -- issue's control-law table
  limits.trimOmegaMax = 1.0f;     // [rad/s]     -- issue's control-law table (arc value)
  limits.trackKS = 2.0f;          // [1/s]       -- k_s
  limits.trackKTheta = 6.0f;      // [1/s]       -- k_theta
  limits.trackKCross = 1.5e-5f;   // [rad/mm^2]  -- k_c
  limits.minSpeed = 20.0f;        // [mm/s]      -- pivot-mode threshold
  return limits;
}

// injectOffset -- build a measured Pose that is EXACTLY (along, cross,
// dTheta) away from `reference` in the reference's own tangent/normal
// frame -- the exact inverse of arc_math's projectOntoPose() rotation.
// Same technique as drive_arc_math_harness.cpp's scenario (d).
Drive::Pose injectOffset(const Drive::Pose& reference, float along, float cross, float dTheta) {
  const float cosT = std::cos(reference.h);
  const float sinT = std::sin(reference.h);
  Drive::Pose measured;
  measured.x = reference.x + along * cosT - cross * sinT;
  measured.y = reference.y + along * sinT + cross * cosT;
  measured.h = reference.h + dTheta;
  return measured;
}

Drive::RefState makeRef(float x, float y, float theta, float v, float omega) {
  Drive::RefState ref;
  ref.x = x;
  ref.y = y;
  ref.theta = theta;
  ref.v = v;
  ref.omega = omega;
  return ref;
}

Drive::BodyState makeMeasured(const Drive::Pose& pose) {
  Drive::BodyState state;
  state.pose = pose;
  return state;
}

// --- Scenarios ---

// (a) All four (eAlong, eCross) quadrants, small non-saturating offsets:
// verify the RECONCILED sign convention (trim uses reference-measured,
// i.e. the NEGATIVE of arc_math's measured-reference eAlong/eCross/eTheta)
// exactly, unclamped.
void scenarioQuadrants() {
  beginScenario("trim signs across all four (eAlong, eCross) quadrants");

  const Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(10.0f, -30.0f, 0.4f, 200.0f, 0.02f);
  const Drive::Pose referencePose{ref.x, ref.y, ref.theta};

  struct Case {
    const char* name;
    float along, cross, dTheta;
  };
  const Case cases[] = {
      {"along+ cross+", 15.0f, 10.0f, 0.05f},
      {"along+ cross-", 15.0f, -10.0f, 0.05f},
      {"along- cross+", -15.0f, 10.0f, -0.05f},
      {"along- cross-", -15.0f, -10.0f, -0.05f},
  };

  for (const Case& c : cases) {
    const Drive::Pose measuredPose = injectOffset(referencePose, c.along, c.cross, c.dTheta);
    const Drive::BodyState measured = makeMeasured(measuredPose);
    const Drive::TrackerOutput out = Drive::track(ref, measured, limits, kTrackwidth);

    std::string label = std::string("[") + c.name + "] ";

    // Diagnostics report arc_math's native (measured - reference) sign,
    // unchanged.
    checkNear(out.eAlong, c.along, 1e-2, label + "eAlong (native arc_math sign)");
    checkNear(out.eCross, c.cross, 1e-2, label + "eCross (native arc_math sign)");
    checkNear(out.eTheta, c.dTheta, 1e-4, label + "eTheta (native arc_math sign)");

    // Trim law uses the reconciled (reference - measured) sign --
    // expected values computed independently here, negated relative to
    // the injected (along, cross, dTheta).
    const double expectedVTrim = limits.trackKS * (-c.along);
    const double expectedOmegaTrim =
        limits.trackKTheta * (-c.dTheta) + limits.trackKCross * ref.v * (-c.cross);

    checkNear(out.vTrim, expectedVTrim, 1e-2, label + "vTrim matches reconciled-sign formula");
    checkNear(out.omegaTrim, expectedOmegaTrim, 1e-4,
              label + "omegaTrim matches reconciled-sign formula");
    checkFalse(out.trimSaturated, label + "small offsets: trimSaturated false");
  }
}

// (b) Reverse travel: a negative ref.v with a known eCross offset -- the
// k_c*v_ref*e_cross term must use the SIGNED v_ref (not |v_ref|).
void scenarioReverseTravelSignedVRef() {
  beginScenario("reverse travel: k_c*v_ref*e_cross uses SIGNED v_ref");

  const Drive::Limits limits = makeLimits();
  const float vRef = -150.0f;  // [mm/s] backward, |vRef| > minSpeed -- arc mode, not pivot
  const Drive::RefState ref = makeRef(0.0f, 0.0f, 0.0f, vRef, 0.0f);
  const Drive::Pose referencePose{ref.x, ref.y, ref.theta};

  const float cross = 40.0f;  // [mm]
  const Drive::Pose measuredPose = injectOffset(referencePose, 0.0f, cross, 0.0f);
  const Drive::BodyState measured = makeMeasured(measuredPose);

  const Drive::TrackerOutput out = Drive::track(ref, measured, limits, kTrackwidth);

  // Reconciled formula: omegaTrim = k_theta*(-eTheta) + k_c*vRef*(-eCross).
  // eTheta == 0 here, so omegaTrim == k_c * vRef * (-cross).
  const double expectedOmegaTrim = limits.trackKCross * vRef * (-cross);
  checkNear(out.omegaTrim, expectedOmegaTrim, 1e-5,
            "omegaTrim uses signed (negative) v_ref in the cross term");

  // Sanity: the SAME |cross| offset with a POSITIVE v_ref of the same
  // magnitude produces the OPPOSITE-sign omegaTrim contribution from the
  // cross term (proves the term is not accidentally using fabsf(v_ref)).
  const Drive::RefState refForward = makeRef(0.0f, 0.0f, 0.0f, -vRef, 0.0f);
  const Drive::TrackerOutput outForward = Drive::track(refForward, measured, limits, kTrackwidth);
  checkTrue((out.omegaTrim > 0.0f) != (outForward.omegaTrim > 0.0f),
            "flipping v_ref's sign flips the cross term's contribution sign");
}

// (c) Pivot mode: v_cmd is a LITERAL 0.0f (bit-exact); omega's trim is
// UNCLAMPED even for a very large heading error (trimSaturated stays
// false).
void scenarioPivotMode() {
  beginScenario("pivot mode: literal v_cmd == 0.0f, omega trim unclamped");

  Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(5.0f, 5.0f, 0.1f, 0.0f, 0.0f);  // ref.v == 0 -- pivot
  const Drive::Pose referencePose{ref.x, ref.y, ref.theta};

  // A deliberately HUGE heading error -- would blow through trimOmegaMax
  // many times over if clamped.
  const float dTheta = 3.0f;  // [rad]
  const Drive::Pose measuredPose = injectOffset(referencePose, 0.0f, 0.0f, dTheta);
  const Drive::BodyState measured = makeMeasured(measuredPose);

  const Drive::TrackerOutput out = Drive::track(ref, measured, limits, kTrackwidth);

  checkExactly(out.vCmd, 0.0, "pivot mode vCmd is a LITERAL 0.0f");
  checkExactly(out.vTrim, 0.0, "pivot mode vTrim is a literal 0.0f (no along-track trim computed)");

  const double expectedOmegaTrim = limits.trackKTheta * (-dTheta);
  checkNear(out.omegaTrim, expectedOmegaTrim, 1e-4, "pivot omegaTrim == trackKTheta*(-eTheta), UNCLAMPED");
  checkTrue(std::fabs(out.omegaTrim) > limits.trimOmegaMax,
            "sanity: this omegaTrim genuinely exceeds trimOmegaMax (proves it was NOT clamped)");
  checkFalse(out.trimSaturated, "pivot mode never reports trimSaturated, even for a huge error");

  // Just above minSpeed -- confirms arc mode (not pivot) kicks in and
  // vCmd is v_ref + (zero, since along == 0 here) trim, not a forced 0.
  Drive::RefState refJustAbove = ref;
  refJustAbove.v = limits.minSpeed + 1.0f;  // clearly arc mode
  const Drive::TrackerOutput outArc = Drive::track(refJustAbove, measured, limits, kTrackwidth);
  checkNear(outArc.vCmd, refJustAbove.v, 1e-4,
            "just above minSpeed: arc mode computes vCmd == v_ref (along offset is 0 here), not a forced 0");
}

// (d) trimSaturated exactly true iff a trim was clamped -- three
// independent cases: only vTrim saturates, only omegaTrim saturates,
// neither saturates.
void scenarioTrimSaturatedExactness() {
  beginScenario("trimSaturated true iff a trim was clamped (independent v/omega cases)");

  const Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(0.0f, 0.0f, 0.0f, 200.0f, 0.0f);  // arc mode
  const Drive::Pose referencePose{ref.x, ref.y, ref.theta};

  // Case 1: only vTrim saturates -- huge along offset, tiny cross/theta.
  {
    const Drive::Pose measuredPose = injectOffset(referencePose, 5000.0f, 0.1f, 0.001f);
    const Drive::BodyState measured = makeMeasured(measuredPose);
    const Drive::TrackerOutput out = Drive::track(ref, measured, limits, kTrackwidth);

    checkExactly(std::fabs(out.vTrim), limits.trimVMax, "case1: vTrim pinned at +/-trimVMax");
    checkTrue(std::fabs(limits.trackKTheta * 0.001f + limits.trackKCross * ref.v * 0.1f) <
                  limits.trimOmegaMax,
              "case1 sanity: raw omegaTrim magnitude is within trimOmegaMax (not saturated)");
    checkTrue(out.trimSaturated, "case1: trimSaturated true (v alone saturated)");
  }

  // Case 2: only omegaTrim saturates -- huge heading offset, tiny along.
  {
    const Drive::Pose measuredPose = injectOffset(referencePose, 0.1f, 0.0f, 2.5f);
    const Drive::BodyState measured = makeMeasured(measuredPose);
    const Drive::TrackerOutput out = Drive::track(ref, measured, limits, kTrackwidth);

    checkTrue(std::fabs(limits.trackKS * 0.1f) < limits.trimVMax,
              "case2 sanity: raw vTrim magnitude is within trimVMax (not saturated)");
    checkExactly(std::fabs(out.omegaTrim), limits.trimOmegaMax,
                 "case2: omegaTrim pinned at +/-trimOmegaMax");
    checkTrue(out.trimSaturated, "case2: trimSaturated true (omega alone saturated)");
  }

  // Case 3: neither saturates -- small offsets both ways.
  {
    const Drive::Pose measuredPose = injectOffset(referencePose, 5.0f, 3.0f, 0.02f);
    const Drive::BodyState measured = makeMeasured(measuredPose);
    const Drive::TrackerOutput out = Drive::track(ref, measured, limits, kTrackwidth);

    checkFalse(out.trimSaturated, "case3: trimSaturated false (neither trim saturated)");
  }
}

// (e) Property/fuzz: the one-sided forward-arc wheel clamp holds across a
// wide DETERMINISTIC grid of trim/error inputs, including deliberately-
// saturating ones. On a forward arc (ref.v > minSpeed), neither wheel is
// ever negative.
void scenarioOneSidedForwardArcClampProperty() {
  beginScenario("property: one-sided forward-arc wheel clamp holds across a wide input grid");

  const Drive::Limits limits = makeLimits();
  const Drive::Pose referencePose{0.0f, 0.0f, 0.2f};

  const float vRefs[] = {21.0f, 50.0f, 150.0f, 400.0f, 900.0f};        // [mm/s] all > minSpeed
  const float omegaRefs[] = {-3.0f, -0.5f, 0.0f, 0.5f, 3.0f};          // [rad/s]
  const float alongs[] = {-5000.0f, -100.0f, 0.0f, 100.0f, 5000.0f};   // [mm]
  const float crosses[] = {-5000.0f, -100.0f, 0.0f, 100.0f, 5000.0f};  // [mm]
  const float dThetas[] = {-3.0f, -0.5f, 0.0f, 0.5f, 3.0f};            // [rad]

  long combos = 0;
  long violations = 0;
  for (float v : vRefs) {
    for (float omega : omegaRefs) {
      Drive::RefState ref = makeRef(referencePose.x, referencePose.y, referencePose.h, v, omega);
      for (float along : alongs) {
        for (float cross : crosses) {
          for (float dTheta : dThetas) {
            const Drive::Pose measuredPose = injectOffset(referencePose, along, cross, dTheta);
            const Drive::BodyState measured = makeMeasured(measuredPose);
            const Drive::TrackerOutput out = Drive::track(ref, measured, limits, kTrackwidth);
            ++combos;
            if (out.command.left < 0.0f || out.command.right < 0.0f) ++violations;
          }
        }
      }
    }
  }

  std::printf("  swept %ld forward-arc combinations\n", combos);
  checkTrue(combos > 1000, "swept a genuinely wide grid (>1000 combinations)");
  checkExactly(static_cast<double>(violations), 0.0,
               "zero forward-arc combinations produced a negative wheel command");
}

// (f) Closed-loop convergence: minimal first-order plant stub. Ticket-
// scoped per the ticket's own "superseded once ticket 006 lands" note.

struct PlantState {
  Drive::Pose pose;
  float v = 0.0f;      // [mm/s] current actual body speed (lags commanded)
  float omega = 0.0f;  // [rad/s] current actual body yaw rate (lags commanded)
};

// stepPlant -- forward-kinematics the commanded wheel speeds into a body
// twist, first-order-lags the plant's actual twist toward it, then Euler-
// integrates the plant's pose forward by `dt`. A MINIMAL stand-in for
// ticket 100-006's real plant model (not yet landed) -- exactly what this
// ticket's own Testing section calls for.
void stepPlant(PlantState* plant, float wheelLeft, float wheelRight, float trackwidth, float dt,
               float lagAlpha) {
  const float vCmd = (wheelLeft + wheelRight) * 0.5f;
  const float omegaCmd = (wheelRight - wheelLeft) / trackwidth;

  plant->v += lagAlpha * (vCmd - plant->v);
  plant->omega += lagAlpha * (omegaCmd - plant->omega);

  plant->pose.x += plant->v * std::cos(plant->pose.h) * dt;
  plant->pose.y += plant->v * std::sin(plant->pose.h) * dt;
  plant->pose.h = Drive::wrapAngle(plant->pose.h + plant->omega * dt);
}

void scenarioClosedLoopArcConvergence() {
  beginScenario("closed-loop convergence: arc (lateral + heading offset) shrinks toward zero");

  // This gain set is DELIBERATELY overdamped/slow on the cross-track axis
  // (the issue's own gain-table rationale: "omega_n = v*sqrt(k_c) <= 2.3
  // rad/s at plateau; zeta >= 1.3 everywhere" -- k_c = 1.5e-5 is tiny by
  // design, tuned for a real actuation lag this ticket-scoped plant stub
  // does not model). Linearizing the closed loop around a straight
  // reference (trace = -trackKTheta = -6, det = trackKCross*vRef^2 = 0.6
  // at vRef=200) gives characteristic roots ~-0.1/s and ~-5.9/s -- the
  // slow pole's own 10s time constant is why this test runs for 20s
  // (2000 ticks) and only requires a 5x reduction, not a fast, tightly-
  // bounded convergence: this is a stability/convergence smoke test, not
  // a tuned settling-time gate (ticket 100-006's real plant + ticket
  // 100-005's policy/terminal machine own that).
  const Drive::Limits limits = makeLimits();
  const float vRef = 200.0f;  // [mm/s] straight-line reference (kappa == 0)
  const float dt = 0.01f;     // [s]
  const int ticks = 2000;     // 20 s

  PlantState plant;
  plant.pose = Drive::Pose{0.0f, 30.0f, 0.25f};  // 30mm cross + 0.25 rad heading offset

  float initialErrorMag = 0.0f;
  float finalErrorMag = 0.0f;

  for (int i = 0; i < ticks; ++i) {
    const float t = static_cast<float>(i) * dt;
    const Drive::Pose refPose = Drive::poseAlongArc(Drive::Pose{0.0f, 0.0f, 0.0f}, 0.0f, vRef * t);
    Drive::RefState ref = makeRef(refPose.x, refPose.y, refPose.h, vRef, 0.0f);

    const Drive::BodyState measured = makeMeasured(plant.pose);
    const Drive::TrackerOutput out = Drive::track(ref, measured, limits, kTrackwidth);

    const float errorMag = std::fabs(out.eCross) + std::fabs(out.eTheta) * 100.0f;
    if (i == 0) initialErrorMag = errorMag;
    if (i == ticks - 1) finalErrorMag = errorMag;

    stepPlant(&plant, out.command.left, out.command.right, kTrackwidth, dt, 0.3f);
  }

  std::printf("  initial |eCross|+100*|eTheta| = %g, final = %g\n", (double)initialErrorMag,
              (double)finalErrorMag);
  checkTrue(finalErrorMag < initialErrorMag * 0.2f,
            "tracked error (cross + heading) shrinks by at least 5x over 20s");
}

void scenarioClosedLoopPivotConvergence() {
  beginScenario("closed-loop convergence: pivot (heading-only offset) shrinks toward zero");

  const Drive::Limits limits = makeLimits();
  const float dt = 0.01f;  // [s]
  const int ticks = 300;   // 3 s

  PlantState plant;
  plant.pose = Drive::Pose{0.0f, 0.0f, 1.0f};  // 1 rad heading offset, in place

  const Drive::RefState ref = makeRef(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // pivot target: heading 0

  float initialTheta = std::fabs(plant.pose.h);
  for (int i = 0; i < ticks; ++i) {
    const Drive::BodyState measured = makeMeasured(plant.pose);
    const Drive::TrackerOutput out = Drive::track(ref, measured, limits, kTrackwidth);
    stepPlant(&plant, out.command.left, out.command.right, kTrackwidth, dt, 0.3f);
  }
  const float finalTheta = std::fabs(plant.pose.h);

  std::printf("  initial |theta| = %g, final |theta| = %g\n", (double)initialTheta,
              (double)finalTheta);
  checkTrue(finalTheta < initialTheta * 0.1f, "pivot heading error shrinks by at least 10x over 3s");
}

}  // namespace

int main() {
  scenarioQuadrants();
  scenarioReverseTravelSignedVRef();
  scenarioPivotMode();
  scenarioTrimSaturatedExactness();
  scenarioOneSidedForwardArcClampProperty();
  scenarioClosedLoopArcConvergence();
  scenarioClosedLoopPivotConvergence();

  if (g_failureCount == 0) {
    std::printf("OK: all Drive:: tracker scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Drive:: tracker scenarios\n", g_failureCount);
  return 1;
}
