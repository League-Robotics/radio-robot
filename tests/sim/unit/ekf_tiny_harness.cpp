// ekf_tiny_harness.cpp — off-hardware acceptance harness for ticket 082-001
// (SUC-001): exercises EkfTiny (source/estimation/ekf_tiny.{h,cpp}) — the
// 3-state (x, y, heading) EKF core ported and trimmed from the parked
// source_old/state/EKFTiny.* (5-state x, y, theta, v, omega) — in isolation,
// with no encoder/odometer wiring (that wiring is ticket 002's job).
//
// Per motor_policy_harness.cpp / velocity_pid_harness.cpp's precedent
// (078-004 / 081-001), this #includes only estimation/ekf_tiny.h plus its
// own translation unit (ekf_tiny.cpp), so it compiles with the plain system
// C++ compiler — no CMake, no ARM toolchain — as long as
// libraries/tinyekf/ is also on the include path (tinyekf.h is header-only).
//
// Two required scenarios (see ticket 082-001's Acceptance Criteria):
//   (a) predict-only matches a hand-computed arc-integration reference
//       within floating-point tolerance.
//   (b) predict+correct demonstrably pulls the estimate TOWARD a
//       deliberately-offset observation (proves the correction step is not
//       a no-op).
//
// Plain C++ program, hand-rolled assertions (mirrors the existing harnesses'
// shape) — prints a PASS/FAIL line per scenario and exits nonzero if any
// assertion failed.
//
// Verification command (see ticket 082-001's Testing plan — no `uv run
// pytest` involvement for this ticket, pure C++, no Python surface yet):
//   c++ -std=c++11 -Wall -Wextra \
//       -I source -I libraries/tinyekf \
//       -o /tmp/ekf_tiny_harness \
//       tests/sim/unit/ekf_tiny_harness.cpp source/estimation/ekf_tiny.cpp
//   /tmp/ekf_tiny_harness

#include <cmath>
#include <cstdio>
#include <string>

#include "estimation/ekf_tiny.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors sim_hardware_harness.cpp /
// velocity_pid_harness.cpp) ---

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
  if (!condition) fail(what + " — expected true, got false");
}

void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %.6f +/- %.6f, got %.6f",
                  what.c_str(), static_cast<double>(expected),
                  static_cast<double>(tol), static_cast<double>(actual));
    fail(buf);
  }
}

// Wrap angle to (-pi, pi], same identity EkfTiny itself uses — kept
// independent here (not a call into the class under test) so the reference
// pose computed below is a genuinely separate re-derivation of the
// arc-segment motion model's equations, not a reuse of the implementation.
float wrapPiRef(float theta) {
  return std::atan2(std::sin(theta), std::cos(theta));
}

// One arc-segment motion-model step, hand-computed independently of
// EkfTiny::predict() — mirrors the equations documented in ekf_tiny.h /
// ekf_tiny.cpp (thetaMid = thetaBefore + dTheta/2; x += dCenter*cos(thetaMid);
// y += dCenter*sin(thetaMid); theta = wrapPi(theta + dTheta)).
struct RefPose {
  float x;
  float y;
  float theta;
};

RefPose referencePredictStep(RefPose prev, float dCenter, float dTheta) {
  float thetaMid = prev.theta + dTheta * 0.5f;
  RefPose next;
  next.x = prev.x + dCenter * std::cos(thetaMid);
  next.y = prev.y + dCenter * std::sin(thetaMid);
  next.theta = wrapPiRef(prev.theta + dTheta);
  return next;
}

// --- Scenarios ----------------------------------------------------------

// (a) predict-only: drive the filter through a short, varied sequence of
// arc segments (straight run, a turn, a reverse-curving turn) with no
// corrections at all, and assert the filter's (x, y, theta) matches an
// independently hand-computed arc-integration reference within tight
// floating-point tolerance.
void scenarioPredictOnlyMatchesArcIntegrationReference() {
  beginScenario("predict-only matches hand-computed arc-integration reference");

  EkfTiny filter;
  filter.init(/*qXy=*/4.0f, /*qTheta=*/0.001f, /*rOtosXy=*/25.0f,
              /*rOtosTheta=*/0.00076f);

  struct Step {
    float dCenter;   // [mm]
    float dTheta;    // [rad]
    float dt;        // [s]
  };
  const Step steps[] = {
      {100.0f, 0.0f, 0.02f},                  // straight run
      {50.0f, static_cast<float>(M_PI) / 2.0f, 0.02f},    // 90 deg turn
      {80.0f, -static_cast<float>(M_PI) / 4.0f, 0.03f},   // partial reverse turn
      {30.0f, static_cast<float>(M_PI) / 6.0f, 0.015f},   // small turn
  };

  RefPose ref{0.0f, 0.0f, 0.0f};
  for (const Step& s : steps) {
    // thetaBefore is the filter's own current theta() — the caller's
    // contract per ekf_tiny.h — read BEFORE calling predict(), which
    // mutates state.
    float thetaBefore = filter.theta();
    filter.predict(s.dCenter, s.dTheta, thetaBefore, s.dt);
    ref = referencePredictStep(ref, s.dCenter, s.dTheta);
  }

  checkNear(filter.x(), ref.x, 1e-3f, "x() matches arc-integration reference after 4 predict-only ticks");
  checkNear(filter.y(), ref.y, 1e-3f, "y() matches arc-integration reference after 4 predict-only ticks");
  checkNear(filter.theta(), ref.theta, 1e-4f, "theta() matches arc-integration reference after 4 predict-only ticks");
}

// (b1) predict+correct (position channel): drive the filter forward with a
// few predict() ticks, then apply updatePosition() with an observation
// deliberately offset from the filter's own belief. Assert the post-update
// state moved TOWARD the observation (strictly closer on both axes than the
// pre-update state), and that it actually moved (not a no-op).
void scenarioUpdatePositionPullsTowardOffsetObservation() {
  beginScenario("updatePosition() pulls the estimate toward an offset observation");

  EkfTiny filter;
  filter.init(/*qXy=*/4.0f, /*qTheta=*/0.001f, /*rOtosXy=*/25.0f,
              /*rOtosTheta=*/0.00076f);

  // A few straight-line predict ticks so P has grown off zero (P starts at
  // zero out of init(); a zero-P Kalman gain is identically zero, which
  // would make updatePosition() a trivial no-op for the wrong reason — the
  // predict ticks below are what make the correction step meaningful).
  for (int i = 0; i < 5; ++i) {
    float thetaBefore = filter.theta();
    filter.predict(/*dCenter=*/40.0f, /*dTheta=*/0.0f, thetaBefore, /*dt=*/0.02f);
  }

  float preX = filter.x();
  float preY = filter.y();

  // Deliberate offset: observation sits well away from the filter's belief
  // in both x and y.
  const float kOffsetX = 60.0f;   // [mm]
  const float kOffsetY = -45.0f;  // [mm]
  float xOtos = preX + kOffsetX;
  float yOtos = preY + kOffsetY;

  filter.updatePosition(xOtos, yOtos);

  float postX = filter.x();
  float postY = filter.y();

  checkTrue(postX != preX, "updatePosition() actually moved x() (not a no-op)");
  checkTrue(postY != preY, "updatePosition() actually moved y() (not a no-op)");

  checkTrue(std::fabs(postX - xOtos) < std::fabs(preX - xOtos),
            "post-update x() is strictly closer to the observation than pre-update x()");
  checkTrue(std::fabs(postY - yOtos) < std::fabs(preY - yOtos),
            "post-update y() is strictly closer to the observation than pre-update y()");

  // Moved in the correct direction (toward the observation, not away).
  checkTrue((postX - preX) * kOffsetX > 0.0f,
            "x() moved in the same direction as the offset (toward the observation)");
  checkTrue((postY - preY) * kOffsetY > 0.0f,
            "y() moved in the same direction as the offset (toward the observation)");
}

// (b2) predict+correct (heading channel): same shape as (b1) but for
// updateHeading() — a deliberately offset heading observation must pull
// theta() toward it, not away from it or leave it unchanged.
void scenarioUpdateHeadingPullsTowardOffsetObservation() {
  beginScenario("updateHeading() pulls the estimate toward an offset observation");

  EkfTiny filter;
  filter.init(/*qXy=*/4.0f, /*qTheta=*/0.001f, /*rOtosXy=*/25.0f,
              /*rOtosTheta=*/0.00076f);

  // A few turning predict ticks so P[2][2] has grown off zero.
  for (int i = 0; i < 5; ++i) {
    float thetaBefore = filter.theta();
    filter.predict(/*dCenter=*/20.0f, /*dTheta=*/0.05f, thetaBefore, /*dt=*/0.02f);
  }

  float preTheta = filter.theta();

  // Deliberate offset: ~11.5 deg away from the filter's current belief.
  const float kOffsetTheta = 0.2f;   // [rad]
  float thetaOtos = wrapPiRef(preTheta + kOffsetTheta);

  filter.updateHeading(thetaOtos);

  float postTheta = filter.theta();

  checkTrue(postTheta != preTheta, "updateHeading() actually moved theta() (not a no-op)");

  float preErr = std::fabs(wrapPiRef(thetaOtos - preTheta));
  float postErr = std::fabs(wrapPiRef(thetaOtos - postTheta));
  checkTrue(postErr < preErr,
            "post-update theta() is strictly closer to the observation than pre-update theta()");

  // Moved in the correct direction (toward the observation).
  float delta = wrapPiRef(postTheta - preTheta);
  checkTrue(delta * kOffsetTheta > 0.0f,
            "theta() moved in the same direction as the offset (toward the observation)");
}

}  // namespace

int main() {
  scenarioPredictOnlyMatchesArcIntegrationReference();
  scenarioUpdatePositionPullsTowardOffsetObservation();
  scenarioUpdateHeadingPullsTowardOffsetObservation();

  if (g_failureCount == 0) {
    std::printf("OK: all EkfTiny scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the EkfTiny scenarios\n", g_failureCount);
  return 1;
}
