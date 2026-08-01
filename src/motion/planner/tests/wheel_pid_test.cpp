// wheel_pid_test.cpp -- unit tests for the M4 duty output stage
// (wheel_pid.h): fail-closed zero gains, feedforward mapping, closed-loop
// tracking against a lagging first-order plant (must beat open-loop kff
// under plant-gain error), anti-windup bounds, and (128-015: EXPLICITLY
// invoked -- Planner::tick() no longer calls stageDuty() automatically,
// see planner.h's own doc comment on the method) the planner-level wiring
// (duty mirrors the staged velocity targets whenever stageDuty() runs).
#include <cmath>
#include <cstdio>

#include "planner.h"
#include "tests/test_support.h"
#include "wheel_pid.h"

using Motion::PidGains;
using Motion::WheelPid;

namespace {

constexpr float kDt = 0.05f;  // [s]

// First-order velocity plant: tau response toward duty/gain steady state.
struct LagPlant {
  float gain = 500.0f;  // [mm/s per duty] steady-state speed per unit duty
  float tau = 0.25f;    // [s] time constant
  float velocity = 0.0f;  // [mm/s]
  void step(float duty, float dt) {  // [s]
    const float target = duty * gain;
    velocity += (target - velocity) * (dt / tau);
  }
};

void testFailClosedZeroGains() {
  WheelPid pid;  // default all-zero gains
  CHECK_NEAR(pid.compute(300.0f, 0.0f, 0.0f, kDt), 0.0f, 1e-9);
}

void testFeedforwardMapping() {
  WheelPid pid;
  pid.configure(PidGains{0.002f, 0.0f, 0.0f, 0.0f});
  // Pure kff: duty = kff * target regardless of measurement.
  CHECK_NEAR(pid.compute(300.0f, 0.0f, 999.0f, kDt), 0.6f, 1e-6);
  CHECK_NEAR(pid.compute(-300.0f, 0.0f, 0.0f, kDt), -0.6f, 1e-6);
  // Output clamps at [-1, 1].
  CHECK_NEAR(pid.compute(1000.0f, 0.0f, 0.0f, kDt), 1.0f, 1e-6);
}

// The reason the PID exists: with the plant gain misestimated (kff 30%
// low), open-loop kff settles ~30% short; the closed loop must converge
// to the target anyway via kp + integral.
void testClosedLoopBeatsOpenLoop() {
  const float target = 300.0f;  // [mm/s]
  LagPlant openPlant, closedPlant;

  WheelPid openLoop;
  openLoop.configure(PidGains{0.0014f, 0.0f, 0.0f, 0.0f});  // kff 30% low
  WheelPid closedLoop;
  closedLoop.configure(PidGains{0.0014f, 0.0016f, 0.005f, 0.3f});

  for (int i = 0; i < 200; ++i) {  // 10 s
    openPlant.step(openLoop.compute(target, 0.0f, openPlant.velocity, kDt), kDt);
    closedPlant.step(closedLoop.compute(target, 0.0f, closedPlant.velocity, kDt),
                     kDt);
  }
  const float openError = std::fabs(openPlant.velocity - target);
  const float closedError = std::fabs(closedPlant.velocity - target);
  CHECK(openError > 50.0f);    // open loop visibly short (~30%)
  CHECK(closedError < 10.0f);  // closed loop converged
  CHECK(closedError < openError * 0.2f);
}

void testAntiWindup() {
  WheelPid pid;
  pid.configure(PidGains{0.002f, 0.0016f, 0.05f, 0.3f});
  // Demand far beyond the plant: output saturates; integrator must stay
  // clamped, and the output must recover promptly when the demand drops.
  for (int i = 0; i < 400; ++i) {
    (void)pid.compute(2000.0f, 0.0f, 100.0f, kDt);
  }
  CHECK(std::fabs(pid.integral()) <= 0.3f + 1e-6f);
  // Post-saturation: a modest, matched target must not be dragged far by
  // leftover integral.
  const float duty = pid.compute(100.0f, 0.0f, 100.0f, kDt);
  CHECK(duty <= 0.2f + 0.3f + 1e-3f);  // kff term + at most the clamp
}

// Planner wiring: duty outputs mirror the staged velocity command whenever
// stageDuty() runs, and go quiet on stop(). 128-015: stageDuty() is called
// EXPLICITLY here, right after TestPlanner::cycle()'s own tick()/update()
// -- that shared helper does not call it, and neither does Planner::tick()
// any more (see planner.h's own doc comment) -- so this test keeps proving
// the duty stage's own correctness without paying its cost in the live
// loop.
constexpr float kDutyStageDt = 0.05f;  // [s], matches TestPlanner::cycle()'s 50.0f ms

void testPlannerDutyStage() {
  Motion::PlannerLimits limits = TestPlanner::benchLimits();
  limits.velKff = 0.002f;
  limits.velKp = 0.0016f;
  limits.velKi = 0.005f;
  limits.velIMax = 0.3f;
  Motion::Planner planner(limits);
  TestPlanner::PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;

  Motion::Move move;
  move.id = 1;
  move.kind = Motion::Move::Kind::Distance;
  move.threshold = 300.0f;
  move.v_x = 150.0f;
  move.timeout = 30000.0f;
  CHECK(planner.move(move, false));

  bool sawDuty = false;
  for (int i = 0; i < 200; ++i) {
    TestPlanner::cycle(planner, state, plant, now, 50.0f);
    planner.stageDuty(kDutyStageDt);
    if (planner.commandedDutyLeft() > 0.05f) sawDuty = true;
    // Duty must track the sign/scale of the staged velocity command.
    if (state.wheelLeft.cmdVelocity > 50.0f) {
      CHECK(planner.commandedDutyLeft() > 0.0f);
    }
  }
  CHECK(sawDuty);
  planner.estop();
  CHECK_NEAR(planner.commandedDutyLeft(), 0.0f, 1e-9);
  CHECK_NEAR(planner.commandedDutyRight(), 0.0f, 1e-9);
}

}  // namespace

int main() {
  testFailClosedZeroGains();
  testFeedforwardMapping();
  testClosedLoopBeatsOpenLoop();
  testAntiWindup();
  testPlannerDutyStage();
  std::printf("wheel_pid_test: all checks passed\n");
  return 0;
}
