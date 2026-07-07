// stop_condition_harness.cpp -- off-hardware acceptance harness for ticket
// 084-001 (SUC-001/SUC-002/SUC-003): exercises Motion::evaluateStopCondition
// (source/motion/stop_condition.{h,cpp}) in isolation against plain
// msg::StopCondition/Motion::MotionBaseline/msg::MotorState/
// msg::PoseEstimate fixtures -- a pure predicate, no fakes needed.
//
// Mirrors drivetrain_harness.cpp's shape exactly: #includes only
// motion/stop_condition.h + messages/*.h (all dependency-free -- no
// MicroBit.h, no I2CBus), links against motion/stop_condition.cpp, compiles
// with the plain system C++ compiler -- no CMake, no ARM toolchain.
// Hand-rolled assertions, prints PASS/FAIL, exits nonzero on any failure.
// Run by test_stop_condition.py, which compiles and runs this binary via
// subprocess.

#include <cmath>
#include <cstdio>
#include <string>

#include "messages/common.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "motion/motion_baseline.h"
#include "motion/stop_condition.h"

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

void checkResult(Motion::StopEvalResult actual, Motion::StopEvalResult expected,
                  const std::string& what) {
  if (actual != expected) {
    const char* names[3] = {"NOT_FIRED", "FIRED", "UNSUPPORTED"};
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %s, got %s", what.c_str(),
                  names[static_cast<int>(expected)], names[static_cast<int>(actual)]);
    fail(buf);
  }
}

msg::MotorState obsPosition(float position) {
  msg::MotorState s;
  s.position.has = true;
  s.position.val = position;
  return s;
}

msg::MotorState obsNoPosition() { return msg::MotorState{}; }

msg::PoseEstimate poseAt(float x, float y, float h) {
  msg::PoseEstimate p;
  p.pose.x = x;
  p.pose.y = y;
  p.pose.h = h;
  return p;
}

// --- STOP_TIME ---

void scenarioTimeFiresAtThreshold() {
  beginScenario("STOP_TIME fires exactly at the threshold, not before");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_TIME;
  c.a = 500.0f;  // [ms]
  Motion::MotionBaseline base;
  base.t0 = 1000;

  msg::MotorState left = obsNoPosition();
  msg::MotorState right = obsNoPosition();
  msg::PoseEstimate pose{};

  checkResult(Motion::evaluateStopCondition(c, base, 1499, left, right, pose),
              Motion::StopEvalResult::NOT_FIRED, "elapsed=499ms < 500ms threshold");
  checkResult(Motion::evaluateStopCondition(c, base, 1500, left, right, pose),
              Motion::StopEvalResult::FIRED, "elapsed=500ms == 500ms threshold");
  checkResult(Motion::evaluateStopCondition(c, base, 2000, left, right, pose),
              Motion::StopEvalResult::FIRED, "elapsed=1000ms > 500ms threshold");
}

// --- STOP_DISTANCE ---

void scenarioDistanceFiresAtThresholdSignedForward() {
  beginScenario("STOP_DISTANCE fires at threshold, signed forward travel");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_DISTANCE;
  c.a = 100.0f;  // [mm]
  Motion::MotionBaseline base;
  base.enc0 = 0.0f;
  base.vSign = 1.0f;
  msg::PoseEstimate pose{};

  checkResult(Motion::evaluateStopCondition(c, base, 0, obsPosition(50.0f), obsPosition(50.0f), pose),
              Motion::StopEvalResult::NOT_FIRED, "traveled 50mm < 100mm threshold");
  checkResult(Motion::evaluateStopCondition(c, base, 0, obsPosition(100.0f), obsPosition(100.0f), pose),
              Motion::StopEvalResult::FIRED, "traveled 100mm == 100mm threshold");
}

void scenarioDistanceRejectsWrongDirectionTravel() {
  beginScenario("STOP_DISTANCE with vSign=-1 rejects travel in the wrong direction");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_DISTANCE;
  c.a = 100.0f;  // [mm]
  Motion::MotionBaseline base;
  base.enc0 = 0.0f;
  base.vSign = -1.0f;  // commanded REVERSE
  msg::PoseEstimate pose{};

  // Raw encoder delta is +150mm (forward) despite a reverse command --
  // signedTraveled = 150 * -1 = -150, which never satisfies `>= 100`.
  checkResult(
      Motion::evaluateStopCondition(c, base, 0, obsPosition(150.0f), obsPosition(150.0f), pose),
      Motion::StopEvalResult::NOT_FIRED, "wrong-direction travel never fires the signed gate");
}

void scenarioDistanceZeroSignFallsBackToMagnitude() {
  beginScenario("STOP_DISTANCE with vSign=0.0 falls back to undirected |delta|");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_DISTANCE;
  c.a = 100.0f;  // [mm]
  Motion::MotionBaseline base;
  base.enc0 = 0.0f;
  base.vSign = 0.0f;  // no commanded direction (e.g. a direction-agnostic watch)
  msg::PoseEstimate pose{};

  checkResult(
      Motion::evaluateStopCondition(c, base, 0, obsPosition(-100.0f), obsPosition(-100.0f), pose),
      Motion::StopEvalResult::FIRED, "magnitude fallback fires on |−100| >= 100 even though negative");
}

void scenarioDistanceMissingObservationNeverFires() {
  beginScenario("STOP_DISTANCE with a missing encoder observation reports NOT_FIRED");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_DISTANCE;
  c.a = 1.0f;  // trivially small threshold -- would fire on any real reading
  Motion::MotionBaseline base;
  base.vSign = 1.0f;
  msg::PoseEstimate pose{};

  checkResult(Motion::evaluateStopCondition(c, base, 0, obsNoPosition(), obsPosition(1000.0f), pose),
              Motion::StopEvalResult::NOT_FIRED,
              "missing left observation never fabricates a phantom trigger");
}

// --- STOP_HEADING ---

void scenarioHeadingFiresWithinEps() {
  beginScenario("STOP_HEADING fires once within eps of the target delta");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_HEADING;
  c.a = 1.5707963f;  // [rad] target delta: +90 deg
  c.b = 0.05f;       // [rad] eps
  Motion::MotionBaseline base;
  base.heading0 = 0.0f;
  msg::MotorState left = obsNoPosition();
  msg::MotorState right = obsNoPosition();

  checkResult(Motion::evaluateStopCondition(c, base, 0, left, right, poseAt(0, 0, 0.0f)),
              Motion::StopEvalResult::NOT_FIRED, "still at the starting heading");
  checkResult(Motion::evaluateStopCondition(c, base, 0, left, right, poseAt(0, 0, 1.5707963f)),
              Motion::StopEvalResult::FIRED, "reached the target heading exactly");
  checkResult(Motion::evaluateStopCondition(c, base, 0, left, right, poseAt(0, 0, 0.78f)),
              Motion::StopEvalResult::NOT_FIRED, "halfway there -- outside eps");
}

// --- STOP_POSITION ---

void scenarioPositionFiresWithinRadius() {
  beginScenario("STOP_POSITION fires once within the arrival radius");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_POSITION;
  c.ax = 100.0f;  // target X, mm
  c.a = 200.0f;   // target Y, mm
  c.b = 10.0f;    // arrival radius, mm
  Motion::MotionBaseline base;
  msg::MotorState left = obsNoPosition();
  msg::MotorState right = obsNoPosition();

  checkResult(Motion::evaluateStopCondition(c, base, 0, left, right, poseAt(100.0f, 195.0f, 0.0f)),
              Motion::StopEvalResult::FIRED, "5mm from target -- inside the 10mm radius");
  checkResult(Motion::evaluateStopCondition(c, base, 0, left, right, poseAt(0.0f, 0.0f, 0.0f)),
              Motion::StopEvalResult::NOT_FIRED, "far from target -- outside the radius");
}

// --- STOP_ROTATION ---

void scenarioRotationFiresAtArcThreshold() {
  beginScenario("STOP_ROTATION fires once the per-wheel arc reaches the threshold");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_ROTATION;
  c.a = 50.0f;  // [mm] target per-wheel arc
  Motion::MotionBaseline base;
  base.encDiff0 = 0.0f;
  base.omegaSign = 1.0f;  // commanded CCW
  msg::PoseEstimate pose{};

  // diff = right - left; arc = |diff|/2.
  checkResult(Motion::evaluateStopCondition(c, base, 0, obsPosition(-20.0f), obsPosition(20.0f), pose),
              Motion::StopEvalResult::NOT_FIRED, "arc=20mm < 50mm threshold");
  checkResult(Motion::evaluateStopCondition(c, base, 0, obsPosition(-50.0f), obsPosition(50.0f), pose),
              Motion::StopEvalResult::FIRED, "arc=50mm == 50mm threshold");
}

// --- Unsupported kinds (architecture-update.md Decision 4) ---

void scenarioUnsupportedKindsAreDistinctFromNotFired() {
  beginScenario("STOP_SENSOR/STOP_COLOR/STOP_LINE_ANY report UNSUPPORTED, not NOT_FIRED");
  Motion::MotionBaseline base;
  msg::MotorState left = obsNoPosition();
  msg::MotorState right = obsNoPosition();
  msg::PoseEstimate pose{};

  msg::StopCondition sensor;
  sensor.kind = msg::StopKind::STOP_SENSOR;
  checkResult(Motion::evaluateStopCondition(sensor, base, 0, left, right, pose),
              Motion::StopEvalResult::UNSUPPORTED, "STOP_SENSOR is recognized-but-unsupported");

  msg::StopCondition color;
  color.kind = msg::StopKind::STOP_COLOR;
  checkResult(Motion::evaluateStopCondition(color, base, 0, left, right, pose),
              Motion::StopEvalResult::UNSUPPORTED, "STOP_COLOR is recognized-but-unsupported");

  msg::StopCondition lineAny;
  lineAny.kind = msg::StopKind::STOP_LINE_ANY;
  checkResult(Motion::evaluateStopCondition(lineAny, base, 0, left, right, pose),
              Motion::StopEvalResult::UNSUPPORTED, "STOP_LINE_ANY is recognized-but-unsupported");
}

void scenarioNoneNeverFires() {
  beginScenario("STOP_NONE always reports NOT_FIRED");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_NONE;
  Motion::MotionBaseline base;
  msg::MotorState left = obsNoPosition();
  msg::MotorState right = obsNoPosition();
  msg::PoseEstimate pose{};

  checkResult(Motion::evaluateStopCondition(c, base, 999999, left, right, pose),
              Motion::StopEvalResult::NOT_FIRED, "STOP_NONE never fires, at any time");
}

// --- remainingToStop (086-003) ---

void checkFloatNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g (+/- %g), got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(tol),
                  static_cast<double>(actual));
    fail(buf);
  }
}

void scenarioRemainingToStopDistanceShrinksToZero() {
  beginScenario("remainingToStop(STOP_DISTANCE) shrinks as encoder travel approaches the threshold");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_DISTANCE;
  c.a = 100.0f;  // [mm]
  Motion::MotionBaseline base;
  base.enc0 = 0.0f;
  base.vSign = 1.0f;
  msg::PoseEstimate pose{};

  float remaining = -1.0f;
  checkResult(Motion::remainingToStop(c, base, obsPosition(50.0f), obsPosition(50.0f), pose, &remaining),
              Motion::StopEvalResult::NOT_FIRED, "50mm traveled, still short of 100mm");
  checkFloatNear(remaining, 50.0f, 1e-4f, "50mm remain of the 100mm threshold");

  checkResult(Motion::remainingToStop(c, base, obsPosition(90.0f), obsPosition(90.0f), pose, &remaining),
              Motion::StopEvalResult::NOT_FIRED, "90mm traveled, still short of 100mm");
  checkFloatNear(remaining, 10.0f, 1e-4f, "10mm remain of the 100mm threshold");

  checkResult(Motion::remainingToStop(c, base, obsPosition(100.0f), obsPosition(100.0f), pose, &remaining),
              Motion::StopEvalResult::FIRED, "100mm traveled == the 100mm threshold");
  checkFloatNear(remaining, 0.0f, 1e-4f, "0mm remain once the threshold is reached");

  // Overshoot past the threshold clamps at 0.0f, never negative.
  checkResult(Motion::remainingToStop(c, base, obsPosition(150.0f), obsPosition(150.0f), pose, &remaining),
              Motion::StopEvalResult::FIRED, "150mm traveled, past the 100mm threshold");
  checkFloatNear(remaining, 0.0f, 1e-4f, "remaining clamps at 0.0f, never negative, past the threshold");
}

void scenarioRemainingToStopDistanceMissingObservationReportsFullRemaining() {
  beginScenario("remainingToStop(STOP_DISTANCE) with a missing encoder observation reports the full distance");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_DISTANCE;
  c.a = 100.0f;  // [mm]
  Motion::MotionBaseline base;
  base.vSign = 1.0f;
  msg::PoseEstimate pose{};

  float remaining = -1.0f;
  checkResult(Motion::remainingToStop(c, base, obsNoPosition(), obsPosition(1000.0f), pose, &remaining),
              Motion::StopEvalResult::NOT_FIRED, "missing left observation never fabricates a phantom delta");
  checkFloatNear(remaining, 100.0f, 1e-4f,
                 "conservatively reports the full 100mm threshold as still remaining");
}

void scenarioRemainingToStopRotationShrinksToZero() {
  beginScenario("remainingToStop(STOP_ROTATION) shrinks as the per-wheel arc approaches the threshold");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_ROTATION;
  c.a = 50.0f;  // [mm] target per-wheel arc
  Motion::MotionBaseline base;
  base.encDiff0 = 0.0f;
  base.omegaSign = 1.0f;  // commanded CCW
  msg::PoseEstimate pose{};

  // diff = right - left; arc = |diff|/2.
  float remaining = -1.0f;
  checkResult(
      Motion::remainingToStop(c, base, obsPosition(-20.0f), obsPosition(20.0f), pose, &remaining),
      Motion::StopEvalResult::NOT_FIRED, "arc=20mm, still short of the 50mm threshold");
  checkFloatNear(remaining, 30.0f, 1e-4f, "30mm of arc remain of the 50mm threshold");

  checkResult(
      Motion::remainingToStop(c, base, obsPosition(-50.0f), obsPosition(50.0f), pose, &remaining),
      Motion::StopEvalResult::FIRED, "arc=50mm reaches the threshold");
  checkFloatNear(remaining, 0.0f, 1e-4f, "0mm of arc remain once the threshold is reached");
}

void scenarioRemainingToStopHeadingShrinksToZero() {
  beginScenario("remainingToStop(STOP_HEADING) shrinks as the fused heading approaches the target delta");
  msg::StopCondition c;
  c.kind = msg::StopKind::STOP_HEADING;
  c.a = 1.5707963f;  // [rad] target delta: +90 deg
  c.b = 0.05f;        // [rad] eps
  Motion::MotionBaseline base;
  base.heading0 = 0.0f;
  msg::MotorState left = obsNoPosition();
  msg::MotorState right = obsNoPosition();

  float remaining = -1.0f;
  checkResult(Motion::remainingToStop(c, base, left, right, poseAt(0, 0, 0.0f), &remaining),
              Motion::StopEvalResult::NOT_FIRED, "still at the starting heading");
  checkFloatNear(remaining, 1.5707963f, 1e-4f, "full 90 deg of heading error remains at the start");

  checkResult(Motion::remainingToStop(c, base, left, right, poseAt(0, 0, 0.78f), &remaining),
              Motion::StopEvalResult::NOT_FIRED, "halfway there -- outside eps");
  checkFloatNear(remaining, 1.5707963f - 0.78f, 1e-4f, "heading error shrinks as the turn progresses");

  checkResult(Motion::remainingToStop(c, base, left, right, poseAt(0, 0, 1.5707963f), &remaining),
              Motion::StopEvalResult::FIRED, "reached the target heading exactly");
  checkFloatNear(remaining, 0.0f, 1e-4f, "0 rad of heading error remains once the target is reached");
}

void scenarioRemainingToStopUnsupportedForOtherKinds() {
  beginScenario("remainingToStop reports UNSUPPORTED for STOP_TIME/STOP_POSITION/STOP_NONE/etc");
  Motion::MotionBaseline base;
  msg::MotorState left = obsNoPosition();
  msg::MotorState right = obsNoPosition();
  msg::PoseEstimate pose{};
  float remaining = -1.0f;

  msg::StopCondition time;
  time.kind = msg::StopKind::STOP_TIME;
  checkResult(Motion::remainingToStop(time, base, left, right, pose, &remaining),
              Motion::StopEvalResult::UNSUPPORTED, "STOP_TIME has no remaining-distance/angle concept");

  msg::StopCondition position;
  position.kind = msg::StopKind::STOP_POSITION;
  checkResult(Motion::remainingToStop(position, base, left, right, pose, &remaining),
              Motion::StopEvalResult::UNSUPPORTED,
              "STOP_POSITION's remaining is pursueSteer()'s own bespoke dRemaining, not this query's");

  msg::StopCondition none;
  none.kind = msg::StopKind::STOP_NONE;
  checkResult(Motion::remainingToStop(none, base, left, right, pose, &remaining),
              Motion::StopEvalResult::UNSUPPORTED, "STOP_NONE is unsupported by this query");
}

}  // namespace

int main() {
  scenarioTimeFiresAtThreshold();
  scenarioDistanceFiresAtThresholdSignedForward();
  scenarioDistanceRejectsWrongDirectionTravel();
  scenarioDistanceZeroSignFallsBackToMagnitude();
  scenarioDistanceMissingObservationNeverFires();
  scenarioHeadingFiresWithinEps();
  scenarioPositionFiresWithinRadius();
  scenarioRotationFiresAtArcThreshold();
  scenarioUnsupportedKindsAreDistinctFromNotFired();
  scenarioNoneNeverFires();

  scenarioRemainingToStopDistanceShrinksToZero();
  scenarioRemainingToStopDistanceMissingObservationReportsFullRemaining();
  scenarioRemainingToStopRotationShrinksToZero();
  scenarioRemainingToStopHeadingShrinksToZero();
  scenarioRemainingToStopUnsupportedForOtherKinds();

  if (g_failureCount == 0) {
    std::printf("OK: all evaluateStopCondition scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the evaluateStopCondition scenarios\n",
              g_failureCount);
  return 1;
}
