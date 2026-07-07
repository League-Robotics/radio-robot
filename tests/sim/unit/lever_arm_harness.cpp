// lever_arm_harness.cpp -- off-hardware acceptance harness for ticket 086-005
// (SUC-005/SUC-006): exercises LeverArm::sensorToCentre()/centreToSensor()
// (source/hal/lever_arm.h) in isolation -- a pure, stateless, header-only
// pair of functions, no fakes needed.
//
// Mirrors stop_condition_harness.cpp's shape exactly: #includes only
// hal/lever_arm.h (dependency-free -- no MicroBit.h, no I2CBus), header-only
// so nothing else needs linking, compiles with the plain system C++
// compiler -- no CMake, no ARM toolchain. Hand-rolled assertions, prints
// PASS/FAIL, exits nonzero on any failure. Run by test_lever_arm.py, which
// compiles and runs this binary via subprocess.

#include <cmath>
#include <cstdio>
#include <string>

#include "hal/lever_arm.h"

namespace {

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

void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(static_cast<double>(actual - expected)) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g (tol %g)",
                  what.c_str(), static_cast<double>(expected),
                  static_cast<double>(actual), static_cast<double>(tol));
    fail(buf);
  }
}

constexpr float kTol = 1e-3f;   // mm/rad -- float round-trip tolerance

// 1. Degenerate case: zero mounting offset is a no-op in both directions,
//    regardless of heading -- the trivial case the ticket explicitly says is
//    not sufficient on its own, but must still hold.
void scenarioZeroOffsetIsIdentity() {
  beginScenario("zero offset: sensorToCentre/centreToSensor are both no-ops");
  float cx = 0.0f, cy = 0.0f, sx = 0.0f, sy = 0.0f;

  LeverArm::centreToSensor(123.0f, -45.0f, 0.7f, 0.0f, 0.0f, sx, sy);
  checkNear(sx, 123.0f, kTol, "centreToSensor: x unchanged with zero offset");
  checkNear(sy, -45.0f, kTol, "centreToSensor: y unchanged with zero offset");

  LeverArm::sensorToCentre(200.0f, 300.0f, -1.1f, 0.0f, 0.0f, cx, cy);
  checkNear(cx, 200.0f, kTol, "sensorToCentre: x unchanged with zero offset");
  checkNear(cy, 300.0f, kTol, "sensorToCentre: y unchanged with zero offset");
}

// 2. Non-degenerate round-trip: real tovez.json mounting offset
//    (odometry_offset_mm x=-47.7, y=3.5) and a non-zero heading. Compute the
//    sensor pose for a given centre pose via centreToSensor(), then recover
//    the centre pose via sensorToCentre() using the SAME-INSTANT heading --
//    the two functions must be exact inverses (acceptance criterion: "at
//    least one non-zero offset + non-zero heading case").
void scenarioRoundTripNonDegenerate() {
  beginScenario("non-zero offset + non-zero heading: exact inverse round-trip");
  constexpr float kOffsetX = -47.7f;   // [mm] tovez.json geometry.odometry_offset_mm.x
  constexpr float kOffsetY = 3.5f;     // [mm] tovez.json geometry.odometry_offset_mm.y

  constexpr float centreX = 512.0f;    // [mm]
  constexpr float centreY = -238.0f;   // [mm]
  constexpr float heading = 0.9f;      // [rad] ~51.6 degrees -- deliberately not a multiple of pi/2

  float sensorX = 0.0f, sensorY = 0.0f;
  LeverArm::centreToSensor(centreX, centreY, heading, kOffsetX, kOffsetY, sensorX, sensorY);

  // Sanity: the sensor pose must actually differ from the centre pose (the
  // offset is non-zero) -- otherwise this "non-degenerate" scenario would
  // silently degrade into scenario 1's trivial case.
  bool moved = (std::fabs(static_cast<double>(sensorX - centreX)) > 1.0) ||
               (std::fabs(static_cast<double>(sensorY - centreY)) > 1.0);
  if (!moved) fail("centreToSensor did not actually displace the pose -- scenario is degenerate");

  float recoveredX = 0.0f, recoveredY = 0.0f;
  LeverArm::sensorToCentre(sensorX, sensorY, heading, kOffsetX, kOffsetY, recoveredX, recoveredY);

  checkNear(recoveredX, centreX, kTol, "round-trip recovers the original centre X");
  checkNear(recoveredY, centreY, kTol, "round-trip recovers the original centre Y");
}

// 3. Round-trip holds across a spread of headings (including negative and
//    beyond-pi/2 values) and a second, differently-signed offset pair --
//    proves the inverse relationship isn't an accident of one particular
//    angle or offset sign combination.
void scenarioRoundTripAcrossHeadings() {
  beginScenario("round-trip holds across a spread of headings and offsets");
  const float offsets[2][2] = {{35.0f, -12.0f}, {-8.0f, 60.0f}};
  const float headings[5] = {0.0f, 0.3f, -1.57f, 2.4f, -3.0f};

  for (const auto& offset : offsets) {
    for (float heading : headings) {
      float sensorX = 0.0f, sensorY = 0.0f;
      LeverArm::centreToSensor(10.0f, -20.0f, heading, offset[0], offset[1], sensorX, sensorY);

      float recoveredX = 0.0f, recoveredY = 0.0f;
      LeverArm::sensorToCentre(sensorX, sensorY, heading, offset[0], offset[1], recoveredX,
                                recoveredY);

      char label[128];
      std::snprintf(label, sizeof(label),
                    "round-trip at heading=%.3f offset=(%.1f,%.1f)",
                    static_cast<double>(heading), static_cast<double>(offset[0]),
                    static_cast<double>(offset[1]));
      checkNear(recoveredX, 10.0f, kTol, std::string(label) + " -- x");
      checkNear(recoveredY, -20.0f, kTol, std::string(label) + " -- y");
    }
  }
}

// 4. Regression guard for the exact db11b7c failure mode: feeding
//    sensorToCentre() a LAGGED heading (not the same-instant one used by
//    centreToSensor()) must NOT round-trip -- it must leave a residual error
//    proportional to the heading discrepancy. This documents *why* the
//    same-instant contract matters, rather than merely asserting it in a
//    comment -- a future change that silently threads a stale heading
//    through would still pass scenarios 1-3 (which always use one shared,
//    correct heading) but must visibly fail here.
void scenarioLaggedHeadingLeavesResidual() {
  beginScenario("db11b7c regression guard: a LAGGED heading breaks the round-trip");
  constexpr float kOffsetX = -47.7f;
  constexpr float kOffsetY = 3.5f;
  constexpr float centreX = 0.0f;
  constexpr float centreY = 0.0f;
  constexpr float trueHeading = 1.2f;    // [rad] the same-instant heading
  constexpr float laggedHeading = 0.8f;  // [rad] a stale heading from an earlier sample

  float sensorX = 0.0f, sensorY = 0.0f;
  LeverArm::centreToSensor(centreX, centreY, trueHeading, kOffsetX, kOffsetY, sensorX, sensorY);

  float recoveredX = 0.0f, recoveredY = 0.0f;
  LeverArm::sensorToCentre(sensorX, sensorY, laggedHeading, kOffsetX, kOffsetY, recoveredX,
                            recoveredY);

  double residual = std::sqrt(static_cast<double>((recoveredX - centreX) * (recoveredX - centreX) +
                                                    (recoveredY - centreY) * (recoveredY - centreY)));
  if (residual < 1.0) {
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "expected a real (>1mm) residual from the mismatched heading, got %g mm -- "
                  "the same-instant-heading contract would be unenforceable if this passed",
                  residual);
    fail(buf);
  }
}

}  // namespace

int main() {
  scenarioZeroOffsetIsIdentity();
  scenarioRoundTripNonDegenerate();
  scenarioRoundTripAcrossHeadings();
  scenarioLaggedHeadingLeavesResidual();

  if (g_failureCount == 0) {
    std::printf("OK: all LeverArm scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the LeverArm scenarios\n", g_failureCount);
  return 1;
}
