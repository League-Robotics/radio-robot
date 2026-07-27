// wheel_trim_test.cpp -- the velocity-domain closed loop (wheel_trim.h):
// fail-closed at zero gains, the HOLD-ONLY integrator gate, the accel
// feedforward as a plain time constant, trim authority clamping, and the
// end-to-end payoff -- a wheel-gain asymmetry that a feedforward-only
// stack cannot correct.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "planner.h"
#include "shape.h"
#include "tests/test_support.h"
#include "wheel_trim.h"

using Motion::Move;
using Motion::MovePhase;
using Motion::MoveShape;
using Motion::Planner;
using Motion::PlannerLimits;
using Motion::shapeOf;
using Motion::TickResult;
using Motion::VelocityTrimGains;
using Motion::WheelTrim;
using TestPlanner::benchLimits;
using TestPlanner::cycle;
using TestPlanner::NoisyPlant;
using TestPlanner::PerfectPlant;

namespace {

Move distanceMove(uint32_t id, float threshold, float v_x) {
  Move m;
  m.id = id;
  m.kind = Move::Kind::Distance;
  m.threshold = threshold;
  m.timeout = 60000.0f;
  m.velocityKind = Move::VelocityKind::Twist;
  m.v_x = v_x;
  return m;
}

// Commissioning-shaped gains: kp well under 1 (measured wheel velocity is
// a raw difference quotient), kaff at half the plant time constant, and a
// trim ceiling that bounds how far feedback may pull the command off the
// profile.
PlannerLimits trimmedLimits() {
  PlannerLimits limits = benchLimits();
  limits.trimKp = 0.25f;     // [1]
  limits.trimKi = 0.5f;      // [1/s]
  limits.trimIMax = 60.0f;   // [mm/s]
  limits.trimKaff = 0.115f;  // [s] half of a ~230 ms plant tau
  limits.trimMax = 120.0f;   // [mm/s]
  return limits;
}

void testFailsClosedAtZeroGains() {
  // The direct assertion behind "every pre-existing test stays green": at
  // the default all-zero gains the staged cmdVelocity is BIT-FOR-BIT the
  // profiled command, on every tick, in every phase.
  const PlannerLimits limits = benchLimits();
  CHECK(limits.trimKp == 0.0f);
  CHECK(limits.trimKi == 0.0f);
  CHECK(limits.trimKaff == 0.0f);

  Planner planner(limits);
  Types::RobotState state{};
  NoisyPlant plant;
  plant.gainLeft = 0.9f;  // a real disturbance the trim WOULD react to
  plant.trackingLag = 0.4f;
  uint32_t now = 1000;
  CHECK(planner.move(distanceMove(1, 500.0f, 150.0f), false));
  for (int i = 0; i < 500; ++i) {
    const TickResult result =
        cycle(planner, state, plant, now, limits.controlPeriod);
    CHECK(planner.trimLeft() == 0.0f);
    CHECK(planner.trimRight() == 0.0f);
    CHECK(state.wheelLeft.cmdVelocity == planner.commandedLeft());
    CHECK(state.wheelRight.cmdVelocity == planner.commandedRight());
    if (result.completed) break;
  }
}

void testIntegratorEngagesOnlyInHold() {
  // The load-bearing gate. Against a persistently slow wheel the
  // integrator must stay EXACTLY zero through the accel ramp, grow during
  // hold, and then FREEZE (not reset) through decel.
  const PlannerLimits limits = trimmedLimits();
  Planner planner(limits);
  Types::RobotState state{};
  NoisyPlant plant;
  plant.gainLeft = 0.92f;
  plant.gainRight = 0.92f;
  plant.trackingLag = 0.4f;
  plant.delayedActuation = true;
  uint32_t now = 1000;
  // Long enough to genuinely cruise.
  CHECK(planner.move(distanceMove(1, 3000.0f, 300.0f), false));

  bool sawAccel = false, sawHold = false, sawDecel = false;
  float integralAtHoldEnd = 0.0f;
  float previousIntegral = 0.0f;
  bool holdGrew = false;
  for (int i = 0; i < 2000; ++i) {
    const TickResult result =
        cycle(planner, state, plant, now, limits.controlPeriod);
    const MovePhase phase = planner.phase();
    const float integral = planner.trimIntegralLeft();
    if (phase == MovePhase::Accel) {
      sawAccel = true;
      // Nothing has been integrated yet, and nothing may be.
      CHECK(integral == 0.0f);
    } else if (phase == MovePhase::Hold) {
      sawHold = true;
      if (integral > previousIntegral + 1e-6f) holdGrew = true;
      integralAtHoldEnd = integral;
    } else if (phase == MovePhase::Decel && sawHold) {
      if (!sawDecel) sawDecel = true;
      // Frozen, not reset: the value the hold learned is still there.
      CHECK_NEAR(integral, integralAtHoldEnd, 1e-5);
    }
    previousIntegral = integral;
    if (result.completed) break;
  }
  CHECK(sawAccel);
  CHECK(sawHold);
  CHECK(sawDecel);
  CHECK(holdGrew);
  // It did real work: a 8%-slow wheel needs a positive trim to hold speed.
  CHECK(integralAtHoldEnd > 0.0f);
}

void testTrimClosesAsymmetricGain() {
  // The payoff. A left wheel 8% slower than the right is exactly the
  // defect a feedforward-only stack cannot see: both wheels are commanded
  // the same speed, one delivers less, and the difference integrates into
  // heading error. Compare final heading with the trim off and on.
  auto runHeading = [](bool withTrim) {
    const PlannerLimits limits = withTrim ? trimmedLimits() : benchLimits();
    Planner planner(limits);
    Types::RobotState state{};
    NoisyPlant plant;
    plant.gainLeft = 0.92f;
    plant.trackingLag = 0.4f;
    plant.delayedActuation = true;
    uint32_t now = 1000;
    CHECK(planner.move(distanceMove(1, 1000.0f, 250.0f), false));
    for (int i = 0; i < 2000; ++i) {
      const TickResult result =
          cycle(planner, state, plant, now, limits.controlPeriod);
      if (result.completed) break;
    }
    // Ground-truth heading from the plant's own wheel travel.
    return (plant.positionRight - plant.positionLeft) / limits.trackWidth;
  };
  const double openLoop = std::fabs(runHeading(false));
  const double closedLoop = std::fabs(runHeading(true));
  // The trim must materially reduce the drift, not merely change it.
  if (!(closedLoop < openLoop * 0.6)) {
    std::printf("FAIL asymmetric gain: open %g rad, closed %g rad\n",
                openLoop, closedLoop);
    std::exit(1);
  }
}

void testWoundIntegratorDoesNotReleaseAsMotion() {
  // A trim learned during a hold must not push the robot after the Move
  // ends: the staged command has to reach exactly zero and stay there.
  const PlannerLimits limits = trimmedLimits();
  Planner planner(limits);
  Types::RobotState state{};
  NoisyPlant plant;
  plant.gainLeft = 0.9f;
  plant.gainRight = 0.9f;
  plant.trackingLag = 0.4f;
  uint32_t now = 1000;
  CHECK(planner.move(distanceMove(1, 2000.0f, 300.0f), false));
  bool completed = false;
  for (int i = 0; i < 3000; ++i) {
    const TickResult result =
        cycle(planner, state, plant, now, limits.controlPeriod);
    if (result.completed) completed = true;
    if (completed && planner.commandedLeft() == 0.0f &&
        planner.commandedRight() == 0.0f) {
      // Once the profile is at rest the staged command must be too --
      // no residual trim leaking out as creep.
      CHECK(state.wheelLeft.cmdVelocity == 0.0f);
      CHECK(state.wheelRight.cmdVelocity == 0.0f);
      CHECK(planner.trimLeft() == 0.0f);
      CHECK(planner.trimRight() == 0.0f);
      const float restLeft = plant.positionLeft;
      const float restRight = plant.positionRight;
      for (int j = 0; j < 60; ++j) {
        cycle(planner, state, plant, now, limits.controlPeriod);
        CHECK(state.wheelLeft.cmdVelocity == 0.0f);
        CHECK(state.wheelRight.cmdVelocity == 0.0f);
      }
      // The plant's own coast is bounded; nothing DRIVES it further.
      CHECK_NEAR(plant.positionLeft, restLeft, 1.0);
      CHECK_NEAR(plant.positionRight, restRight, 1.0);
      return;
    }
  }
  CHECK(false);  // never came to rest
}

void testStandingRobotDoesNotDrift() {
  // With no Move at all, the trim must stay silent -- an idle controller
  // chasing encoder noise would walk a parked robot.
  const PlannerLimits limits = trimmedLimits();
  Planner planner(limits);
  Types::RobotState state{};
  NoisyPlant plant;
  plant.noiseAmplitude = 6.0f;   // [mm/s] flicker at true rest
  plant.noiseWhite = 4.0f;
  plant.positionQuantum = 0.1f;  // [mm]
  uint32_t now = 1000;
  for (int i = 0; i < 400; ++i) {
    cycle(planner, state, plant, now, limits.controlPeriod);
    CHECK(state.wheelLeft.cmdVelocity == 0.0f);
    CHECK(state.wheelRight.cmdVelocity == 0.0f);
  }
  CHECK_NEAR(plant.positionLeft, 0.0, 1e-6);
  CHECK_NEAR(plant.positionRight, 0.0, 1e-6);
}

void testUnitBehaviorOfTheLaw() {
  // The law itself, away from the planner.
  VelocityTrimGains gains;
  gains.kp = 0.5f;
  gains.ki = 2.0f;
  gains.iMax = 50.0f;
  gains.kaff = 0.2f;
  gains.trimMax = 40.0f;
  WheelTrim trim;
  trim.configure(gains);

  // Proportional is dimensionless: 10 mm/s slow at kp 0.5 -> +5 mm/s.
  CHECK_NEAR(trim.compute(100.0f, 0.0f, 90.0f, 0.05f, MovePhase::Accel),
             5.0f, 1e-4);
  CHECK(trim.integral() == 0.0f);  // Accel never integrates

  // kaff is a time constant: at 200 mm/s^2 with kaff 0.2 s the lead is
  // 40 mm/s (clamped here by trimMax, which is the point of the clamp).
  trim.reset();
  const float lead =
      trim.compute(100.0f, 200.0f, 100.0f, 0.05f, MovePhase::Decel);
  CHECK_NEAR(lead, 40.0f, 1e-4);  // 0.2*200 = 40, exactly at the ceiling
  trim.reset();
  const float lead2 =
      trim.compute(100.0f, 100.0f, 100.0f, 0.05f, MovePhase::Decel);
  CHECK_NEAR(lead2, 20.0f, 1e-4);  // 0.2*100, unclamped

  // Hold integrates; the integral is clamped at iMax.
  trim.reset();
  for (int i = 0; i < 2000; ++i) {
    trim.compute(100.0f, 0.0f, 90.0f, 0.05f, MovePhase::Hold);
  }
  CHECK(trim.integral() <= gains.iMax + 1e-4f);
  CHECK(trim.integral() > 0.0f);
  // Total output respects trimMax even with a full integrator.
  const float out = trim.compute(100.0f, 0.0f, 0.0f, 0.05f, MovePhase::Hold);
  CHECK(std::fabs(out) <= gains.trimMax + 1e-4f);

  // Zero gains -> exactly zero, whatever the error.
  WheelTrim inert;
  CHECK(inert.compute(300.0f, 500.0f, 0.0f, 0.05f, MovePhase::Hold) == 0.0f);
}

void testRatioStillLockedWithTrimOn() {
  // The trim deliberately breaks the COMMANDED ratio (that is how it fixes
  // the ACTUAL one), so the profiled pair must remain exactly on the
  // shape's ray while the staged pair may differ from it -- and only
  // within the configured trim authority.
  const PlannerLimits limits = trimmedLimits();
  const Move move = distanceMove(1, 800.0f, 200.0f);
  const MoveShape shape = shapeOf(move, limits.trackWidth);
  Planner planner(limits);
  Types::RobotState state{};
  NoisyPlant plant;
  plant.gainLeft = 0.9f;
  plant.trackingLag = 0.4f;
  plant.delayedActuation = true;
  uint32_t now = 1000;
  CHECK(planner.move(move, false));
  bool sawTrim = false;
  for (int i = 0; i < 2000; ++i) {
    const TickResult result =
        cycle(planner, state, plant, now, limits.controlPeriod);
    const double cross =
        static_cast<double>(planner.commandedLeft()) * shape.unitRight -
        static_cast<double>(planner.commandedRight()) * shape.unitLeft;
    CHECK(std::fabs(cross) <= 1e-3);  // the PROFILE is untouched
    CHECK(std::fabs(planner.trimLeft()) <= limits.trimMax + 1e-3f);
    CHECK(std::fabs(planner.trimRight()) <= limits.trimMax + 1e-3f);
    CHECK_NEAR(state.wheelLeft.cmdVelocity,
               planner.commandedLeft() + planner.trimLeft(), 1e-4);
    if (std::fabs(planner.trimLeft()) > 1.0f) sawTrim = true;
    if (result.completed) break;
  }
  CHECK(sawTrim);
}

}  // namespace

int main() {
  testFailsClosedAtZeroGains();
  testIntegratorEngagesOnlyInHold();
  testTrimClosesAsymmetricGain();
  testWoundIntegratorDoesNotReleaseAsMotion();
  testStandingRobotDoesNotDrift();
  testUnitBehaviorOfTheLaw();
  testRatioStillLockedWithTrimOn();
  std::printf("wheel_trim_test: all checks passed\n");
  return 0;
}
