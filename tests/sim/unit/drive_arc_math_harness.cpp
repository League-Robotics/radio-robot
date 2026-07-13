// drive_arc_math_harness.cpp -- off-hardware acceptance harness for ticket
// 100-002 (SUC-002/SUC-008): exercises Drive:: arc_math (source/drive/
// arc_math.{h,cpp}) in isolation, mirroring jerk_trajectory_harness.cpp's/
// ruckig_smoke_harness.cpp's compile-and-run pattern -- hand-rolled
// assertions, no gtest/pytest-native C++ framework, run via
// test_drive_arc_math.py.
//
// Scenarios:
//  (a) straight-line (kappa == 0) composeArc/poseAlongArc matches the plain
//      cos/sin displacement, and wrapAngle's own identity.
//  (b) quarter-circle and half-circle composeArc against HAND-COMPUTED
//      expected endpoints (independent of this codebase's implementation --
//      classic constant-curvature geometry: a CCW quarter turn of radius R
//      starting at heading 0 ends at (R, R) facing +y; a CCW half turn ends
//      diametrically opposite the start, facing backward).
//  (c) round trip: composeArc(anchor, kappa, s) then projectOntoArc at the
//      SAME s recovers zero error (eAlong == eCross == eTheta == 0) within
//      float tolerance -- the ticket's own "compose then project recovers
//      the original" requirement.
//  (d) projectOntoArc's rotation is exact for a KNOWN local-frame offset:
//      constructing a measured pose by injecting a known (along, cross,
//      dTheta) offset into a reference pose's own tangent/normal frame
//      recovers that exact offset back out, for both a straight and a
//      curved reference arc.
//  (e) wrapAngle wraps a range of positive/negative multi-turn angles into
//      (-pi, pi] while preserving the sin/cos identity (same effective
//      angle).
#include <cmath>
#include <cstdio>
#include <string>

#include "drive/arc_math.h"
#include "drive/types.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors jerk_trajectory_harness.cpp) ---

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

void checkNear(double actual, double expected, double tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected ~%g (tol %g), got %g", what.c_str(), expected,
                  tol, actual);
    fail(buf);
  }
}

constexpr double kPi = 3.14159265358979323846;

// --- Scenarios ---

// (a) Straight-line (kappa == 0): composeArc/poseAlongArc match the plain
// cos/sin displacement; heading is unchanged.
void scenarioStraightLine() {
  beginScenario("straight line (kappa == 0): plain cos/sin displacement");

  Drive::Pose start{100.0f, -50.0f, 0.5f};  // [mm][mm][rad]
  const float length = 300.0f;              // [mm]

  Drive::Pose end = Drive::composeArc(start, 0.0f, length);
  checkNear(end.x, start.x + length * std::cos(start.h), 1e-2, "straight: x = start.x + L*cos(h)");
  checkNear(end.y, start.y + length * std::sin(start.h), 1e-2, "straight: y = start.y + L*sin(h)");
  checkNear(end.h, start.h, 1e-6, "straight: heading unchanged");

  // poseAlongArc at the same (anchor, kappa, s) must agree with composeArc
  // exactly -- both are the same underlying closed form (arc_math.h's own
  // doc comment: composeArc is poseAlongArc, named for a different call
  // site).
  Drive::Pose sampled = Drive::poseAlongArc(start, 0.0f, length);
  checkNear(sampled.x, end.x, 1e-6, "poseAlongArc agrees with composeArc (x)");
  checkNear(sampled.y, end.y, 1e-6, "poseAlongArc agrees with composeArc (y)");
  checkNear(sampled.h, end.h, 1e-6, "poseAlongArc agrees with composeArc (h)");
}

// (b) Quarter-circle: a CCW quarter turn of radius R starting at heading 0
// ends at world (R, R) facing +y (heading pi/2) -- classic constant-
// curvature geometry, hand-computed independently of this implementation.
void scenarioQuarterCircle() {
  beginScenario("quarter circle: CCW R=100, heading 0 -> (100, 100), heading pi/2");

  const float R = 100.0f;                              // [mm]
  const float kappa = 1.0f / R;                         // [1/mm] CCW+
  const float arcLength = static_cast<float>(kPi / 2.0 * R);  // [mm] quarter turn

  Drive::Pose start{0.0f, 0.0f, 0.0f};
  Drive::Pose end = Drive::composeArc(start, kappa, arcLength);

  checkNear(end.x, R, 0.05, "quarter circle: x == R");
  checkNear(end.y, R, 0.05, "quarter circle: y == R");
  checkNear(end.h, kPi / 2.0, 1e-4, "quarter circle: heading == pi/2");
}

// (b) Half-circle: a CCW half turn ends diametrically opposite the start,
// facing backward (heading pi).
void scenarioHalfCircle() {
  beginScenario("half circle: CCW R=50, heading 0 -> (0, 100), heading pi");

  const float R = 50.0f;                        // [mm]
  const float kappa = 1.0f / R;                  // [1/mm]
  const float arcLength = static_cast<float>(kPi * R);  // [mm] half turn

  Drive::Pose start{0.0f, 0.0f, 0.0f};
  Drive::Pose end = Drive::composeArc(start, kappa, arcLength);

  checkNear(end.x, 0.0, 0.05, "half circle: x == 0 (diametrically opposite)");
  checkNear(end.y, 2.0 * R, 0.05, "half circle: y == 2R");
  checkNear(std::fabs(end.h), kPi, 1e-4, "half circle: heading == +/-pi (facing backward)");
}

// (c) Round trip: composeArc(anchor, kappa, s) then projectOntoArc at the
// SAME s recovers zero error -- both for a straight and a curved arc.
void scenarioRoundTripZeroErrorAtMatchingS() {
  beginScenario("round trip: compose then project at the SAME s recovers zero error");

  struct Case {
    const char* name;
    float kappa;
    float s;
  };
  const Case cases[] = {
      {"straight", 0.0f, 250.0f},
      {"gentle-CCW", 0.003f, 400.0f},
      {"tight-CW", -0.02f, 120.0f},
  };

  Drive::Pose anchor{25.0f, -80.0f, 0.9f};
  for (const Case& c : cases) {
    Drive::Pose measured = Drive::composeArc(anchor, c.kappa, c.s);
    Drive::ArcError error = Drive::projectOntoArc(anchor, c.kappa, c.s, measured);

    std::string label = std::string("[") + c.name + "] ";
    checkNear(error.eAlong, 0.0, 1e-2, label + "eAlong ~= 0 at matching s");
    checkNear(error.eCross, 0.0, 1e-2, label + "eCross ~= 0 at matching s");
    checkNear(error.eTheta, 0.0, 1e-4, label + "eTheta ~= 0 at matching s");
  }
}

// (d) projectOntoArc's rotation is exact for a KNOWN local-frame offset:
// build a measured pose by injecting (along, cross, dTheta) into the
// reference pose's own tangent/normal frame, and confirm the recovered
// error matches exactly -- both for a straight and a curved reference.
void scenarioProjectionRecoversKnownLocalOffset() {
  beginScenario("projectOntoArc recovers a known (along, cross, dTheta) local offset exactly");

  struct Case {
    const char* name;
    float kappa;
  };
  const Case cases[] = {
      {"straight", 0.0f},
      {"curved", 0.0045f},
  };

  Drive::Pose anchor{10.0f, 20.0f, 0.2f};
  const float s = 350.0f;         // [mm] reference parameter
  const float along = 18.0f;      // [mm] known injected along-track offset
  const float cross = -7.0f;      // [mm] known injected cross-track offset
  const float dTheta = 0.05f;     // [rad] known injected heading offset

  for (const Case& c : cases) {
    Drive::Pose reference = Drive::poseAlongArc(anchor, c.kappa, s);
    const float cosT = std::cos(reference.h);
    const float sinT = std::sin(reference.h);

    // Inject the offset in the tangent/normal frame: tangent = (cosT,
    // sinT), left-normal = (-sinT, cosT) -- the exact inverse of
    // projectOntoArc's own rotation (arc_math.h's ArcError doc comment).
    Drive::Pose measured;
    measured.x = reference.x + along * cosT - cross * sinT;
    measured.y = reference.y + along * sinT + cross * cosT;
    measured.h = reference.h + dTheta;

    Drive::ArcError error = Drive::projectOntoArc(anchor, c.kappa, s, measured);

    std::string label = std::string("[") + c.name + "] ";
    checkNear(error.eAlong, along, 1e-2, label + "eAlong recovers the injected along offset");
    checkNear(error.eCross, cross, 1e-2, label + "eCross recovers the injected cross offset");
    checkNear(error.eTheta, dTheta, 1e-4, label + "eTheta recovers the injected heading offset");
  }
}

// (e) wrapAngle wraps multi-turn angles into (-pi, pi] while preserving the
// sin/cos identity (same effective angle) and the range bound.
void scenarioWrapAngle() {
  beginScenario("wrapAngle: multi-turn angles wrap into (-pi, pi], identity preserved");

  const double testAngles[] = {0.0,  0.1,        kPi / 2.0,  kPi - 0.01, kPi + 0.5,
                                -0.3, -kPi / 2.0, -kPi + 0.2, 2.0 * kPi + 0.4, -3.0 * kPi + 1.0,
                                5.5 * kPi};
  for (double a : testAngles) {
    float wrapped = Drive::wrapAngle(static_cast<float>(a));
    char label[64];
    std::snprintf(label, sizeof(label), "wrapAngle(%.3f)", a);

    checkTrue(wrapped > -static_cast<float>(kPi) - 1e-4f &&
                  wrapped <= static_cast<float>(kPi) + 1e-4f,
              std::string(label) + " is within (-pi, pi]");
    checkNear(std::sin(wrapped), std::sin(a), 1e-3, std::string(label) + " preserves sin()");
    checkNear(std::cos(wrapped), std::cos(a), 1e-3, std::string(label) + " preserves cos()");
  }
}

}  // namespace

int main() {
  scenarioStraightLine();
  scenarioQuarterCircle();
  scenarioHalfCircle();
  scenarioRoundTripZeroErrorAtMatchingS();
  scenarioProjectionRecoversKnownLocalOffset();
  scenarioWrapAngle();

  if (g_failureCount == 0) {
    std::printf("OK: all Drive:: arc_math scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Drive:: arc_math scenarios\n", g_failureCount);
  return 1;
}
