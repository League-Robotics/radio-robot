// planner_duty_scenarios_test.cpp -- the CO-LOCATED one-loop duty
// topology against the MEASURED plant (duty_plant.h): sense -> estimate ->
// plan -> WheelPid -> duty, one cycle, duty applied to the plant the same
// interval it was computed -- the structure the on-robot integration will
// run (sense/plan/act within one 50 ms cycle, zero transport in the loop).
//
// "Solid motion" gates (stakeholder 2026-07-26): every Move completes AND
// settles; distance error small and bounded on a lagging, quantized,
// noisy, L/R-asymmetric plant; turns accurate; chains carry; straight
// legs stay straight (heading hold trims the 7% gain asymmetry); no
// post-landing creep or oscillation; stop() means stopped.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "planner.h"
#include "tests/duty_plant.h"
#include "tests/test_support.h"

using Motion::Move;
using Motion::Planner;
using Motion::PlannerLimits;
using Motion::TickResult;
using TestPlanner::DutyPlant;

namespace {

constexpr float kPeriod = 50.0f;   // [ms]
constexpr float kTrack = 128.0f;   // [mm] tovez

PlannerLimits dutyLimits() {
  PlannerLimits limits;
  limits.vMax = 400.0f;
  limits.aMax = 300.0f;    // [mm/s^2]
  limits.aDecel = 250.0f;  // [mm/s^2]
  limits.omegaMax = 3.0f;
  limits.alphaMax = 6.0f;   // [rad/s^2]
  limits.alphaDecel = 5.0f;  // [rad/s^2]
  limits.trackWidth = kTrack;
  limits.controlPeriod = kPeriod;
  limits.actuationDelay = 50.0f;  // [ms] one cycle: sample age at decide time
  limits.velocityFilterWeight = 0.35f;  // reported velocity is noisy
  limits.requireSettle = true;
  limits.settleWindow = 2500.0f;  // [ms] -- the creep must walk the
                                  // hand-off residual to the epsilon AND
                                  // the lagging plant must decay to true
                                  // rest (tau ~230 ms)
  limits.headingHoldGain = 1.5f;  // [1/s] trims the L/R gain asymmetry
  // M4 duty stage, from the measured plant: kff = 1/gain(nominal 1370);
  // feedback sized for one-cycle dead time + tau 0.23 s.
  limits.velKff = 1.0f / 1370.0f;
  limits.velKp = 0.0009f;
  limits.velKi = 0.006f;
  limits.velIMax = 0.25f;
  limits.velIAccelGate = 50.0f;  // [mm/s^2] integral trims at steady state only
  return limits;
}

// One co-located cycle: stamp, tick (compute), update (save), then the
// plant integrates THIS tick's duty over the interval and publishes the
// samples the next tick will see.
TickResult cycle(Planner& planner, Types::RobotState& state,
                 DutyPlant& plant, uint32_t& now) {
  state.time.cycleStart = now;
  const TickResult result = planner.tick(state);
  planner.update(state);
  now += static_cast<uint32_t>(kPeriod);
  plant.step(state, planner.commandedDutyLeft(),
             planner.commandedDutyRight(), kPeriod * 0.001f, now);
  return result;
}

Move distanceMove(uint32_t id, float threshold, float v_x) {
  Move m;
  m.id = id;
  m.kind = Move::Kind::Distance;
  m.threshold = threshold;
  m.v_x = v_x;
  m.timeout = 30000.0f;
  return m;
}

Move angleMove(uint32_t id, float threshold, float omega) {
  Move m;
  m.id = id;
  m.kind = Move::Kind::Angle;
  m.threshold = threshold;
  m.omega = omega;
  m.timeout = 30000.0f;
  return m;
}

struct RunOutcome {
  bool completed = false;
  bool settled = false;
  int ticks = 0;
};

RunOutcome runToCompletion(Planner& planner, Types::RobotState& state,
                           DutyPlant& plant, uint32_t& now, int maxTicks,
                           bool drainAfter = true) {
  RunOutcome outcome;
  for (int i = 0; i < maxTicks; ++i) {
    const TickResult r = cycle(planner, state, plant, now);
    ++outcome.ticks;
    if (r.completed) {
      outcome.completed = true;
      outcome.settled = r.settled;
      if (!drainAfter) return outcome;
      break;
    }
  }
  if (drainAfter) {
    for (int i = 0; i < 40; ++i) cycle(planner, state, plant, now);
  }
  return outcome;
}

void testDistanceSolid() {
  Planner planner(dutyLimits());
  DutyPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  plant.step(state, 0.0f, 0.0f, 0.05f, now);  // seed samples

  CHECK(planner.move(distanceMove(1, 500.0f, 200.0f), false));
  const RunOutcome outcome =
      runToCompletion(planner, state, plant, now, 400);
  CHECK(outcome.completed);
  CHECK(outcome.settled);

  const float path = plant.truePath();
  const float headingDeg =
      plant.trueHeading(kTrack) * 57.29578f;
  std::printf("  distance 500: path %.2f mm (err %+.2f), heading %+.3f deg,"
              " %d ticks\n", path, path - 500.0f, headingDeg, outcome.ticks);
  CHECK_NEAR(path, 500.0f, 3.0f);        // solid: within 3 mm on real dynamics
  CHECK(std::fabs(headingDeg) < 1.0f);   // straight stays straight
  // No post-landing creep: hold for 2 s, position must not move.
  const float parked = plant.truePath();
  for (int i = 0; i < 40; ++i) cycle(planner, state, plant, now);
  CHECK_NEAR(plant.truePath(), parked, 0.5f);
}

void testDistanceBackward() {
  Planner planner(dutyLimits());
  DutyPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  plant.step(state, 0.0f, 0.0f, 0.05f, now);
  CHECK(planner.move(distanceMove(2, 350.0f, -200.0f), false));
  const RunOutcome outcome =
      runToCompletion(planner, state, plant, now, 400);
  CHECK(outcome.completed);
  CHECK(outcome.settled);
  std::printf("  distance -350: path %.2f mm (err %+.2f)\n",
              plant.truePath(), plant.truePath() + 350.0f);
  CHECK_NEAR(plant.truePath(), -350.0f, 3.0f);
}

void testTurnSolid() {
  Planner planner(dutyLimits());
  DutyPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  plant.step(state, 0.0f, 0.0f, 0.05f, now);
  const float quarter = 1.5707964f;
  CHECK(planner.move(angleMove(3, quarter, 1.5f), false));
  const RunOutcome outcome =
      runToCompletion(planner, state, plant, now, 400);
  CHECK(outcome.completed);
  CHECK(outcome.settled);
  const float headingDeg = plant.trueHeading(kTrack) * 57.29578f;
  std::printf("  turn 90: heading %.3f deg (err %+.3f), path drift %.2f mm\n"
              "    planner-believed heading %.3f deg | plant true %.3f deg\n",
              headingDeg, headingDeg - 90.0f, plant.truePath(),
              state.pose.heading * 57.29578f, headingDeg);
  CHECK_NEAR(headingDeg, 90.0f, 1.0f);       // within a degree
  CHECK(std::fabs(plant.truePath()) < 8.0f);  // pivot stays in place
}

void testChainSolid() {
  Planner planner(dutyLimits());
  DutyPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  plant.step(state, 0.0f, 0.0f, 0.05f, now);
  const float quarter = 1.5707964f;
  CHECK(planner.move(distanceMove(10, 300.0f, 200.0f), false));
  CHECK(planner.move(distanceMove(11, 200.0f, 200.0f), false));
  CHECK(planner.move(angleMove(12, quarter, 1.5f), false));
  CHECK(planner.move(distanceMove(13, 250.0f, 200.0f), false));

  int completions = 0;
  float minCarrySpeed = 1e9f;
  bool sawCruise = false;
  for (int i = 0; i < 900 && completions < 4; ++i) {
    const TickResult r = cycle(planner, state, plant, now);
    const float speed =
        0.5f * (plant.left.velocity + plant.right.velocity);
    if (speed > 190.0f) sawCruise = true;
    if (sawCruise && completions == 0) {
      minCarrySpeed = std::min(minCarrySpeed, speed);
    }
    if (r.completed) ++completions;
  }
  CHECK(completions == 4);
  for (int i = 0; i < 40; ++i) cycle(planner, state, plant, now);

  const float path = plant.truePath();
  const float headingDeg = plant.trueHeading(kTrack) * 57.29578f;
  std::printf("  chain: path %.2f mm (target 750 + pivot), heading %.3f deg"
              " (target 90), carry-min %.1f mm/s\n",
              path, headingDeg, minCarrySpeed);
  // Total distance: 300+200 pre-turn + 250 post-turn = 750.
  CHECK_NEAR(path, 750.0f, 8.0f);
  CHECK_NEAR(headingDeg, 90.0f, 1.5f);
  // Same-axis boundary crossed at speed: no land-at-zero dip between the
  // two forward legs (allow the plant's own ripple).
  CHECK(minCarrySpeed > 120.0f);
}

void testStopMeansStopped() {
  Planner planner(dutyLimits());
  DutyPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  plant.step(state, 0.0f, 0.0f, 0.05f, now);
  CHECK(planner.move(distanceMove(20, 5000.0f, 250.0f), false));
  for (int i = 0; i < 40; ++i) cycle(planner, state, plant, now);
  planner.estop();
  for (int i = 0; i < 30; ++i) cycle(planner, state, plant, now);
  // Rest damping, not a dead-duty clamp: the integral is reset/frozen at
  // rest but the P term stays engaged (duty = -kp*measured, far below any
  // motor deadband), so the residual duty tracks the plant's residual
  // measured velocity instead of snapping to exactly zero. "Stopped" is
  // proven by the parked-pose hold below, not by a zero duty word.
  CHECK(std::fabs(planner.commandedDutyLeft()) < 0.02f);
  CHECK(std::fabs(plant.left.velocity) < 5.0f);
  const float parked = plant.truePath();
  for (int i = 0; i < 20; ++i) cycle(planner, state, plant, now);
  CHECK_NEAR(plant.truePath(), parked, 0.5f);
}

}  // namespace

int main() {
  testDistanceSolid();
  testDistanceBackward();
  testTurnSolid();
  testChainSolid();
  testStopMeansStopped();
  std::printf("planner_duty_scenarios_test: all checks passed\n");
  return 0;
}
