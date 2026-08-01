// capi_stage_duty_test.cpp -- regression test for the exact C ABI surface
// src/tests/bench/hil_drive.py (--duty mode) and square_tour_sim.py drive
// via ctypes (129-006, bench-duty-readers-see-zero-after-stageduty-park.md).
//
// Planner::stageDuty() was PARKED from the live tick by 128-015:
// plannerDuty() (capi.cpp) reflects only the last EXPLICIT stageDuty()
// call, 0.0f otherwise. Both bench scripts read plannerDuty() every cycle
// without ever calling the new plannerStageDuty() wrapper -- silently (and,
// for hil_drive.py's --duty mode specifically, not just cosmetically:
// square_tour_sim.py's own SimWheel plant is driven ENTIRELY off this
// value, so a permanent zero here means the whole sim never moves) reading
// zero forever. This test proves the fix at the SAME boundary those
// scripts call across (the extern "C" functions, not Planner's own C++
// methods -- already covered elsewhere by wheel_pid_test.cpp's
// testPlannerDutyStage() and planner_duty_scenarios_test.cpp): duty reads
// exactly zero before the caller ever invokes plannerStageDuty(), and
// nonzero once it does, mirroring the exact call sequence the fix now uses.
#include <cmath>
#include <cstdio>

#include "planner.h"
#include "tests/test_support.h"

// capi.cpp's own extern "C" surface (no header -- it exists purely for the
// ctypes harness, src/tests/bench/planner_harness.py). Declared here to
// call it exactly as the Python ctypes bindings do, not as Planner's own
// C++ methods.
extern "C" {
void* plannerCreate(const Motion::PlannerLimits* limits);
void plannerDestroy(void* planner);
bool plannerMove(void* planner, const Motion::Move* next, bool replace);
void plannerTick(void* planner, const Types::RobotState* state,
                 Motion::TickResult* result);
void plannerUpdate(void* planner, Types::RobotState* state);
void plannerDuty(void* planner, float* left, float* right);
void plannerStageDuty(void* planner, float dt);
}

namespace {

void testDutyReadsZeroUntilStageDutyCalled() {
  Motion::PlannerLimits limits = TestPlanner::benchLimits();
  limits.velKff = 0.002f;
  limits.velKp = 0.0016f;
  limits.velKi = 0.005f;
  limits.velIMax = 0.3f;
  void* planner = plannerCreate(&limits);

  TestPlanner::PerfectPlant plant;
  Types::RobotState state;
  Motion::TickResult result;
  uint32_t now = 0;

  Motion::Move move;
  move.id = 1;
  move.kind = Motion::Move::Kind::Distance;
  move.threshold = 300.0f;
  move.v_x = 150.0f;
  move.timeout = 30000.0f;
  CHECK(plannerMove(planner, &move, false));

  // Drive several cycles through tick()/update() WITHOUT ever calling
  // plannerStageDuty() -- the exact sequence hil_drive.py/square_tour_sim.py
  // used before this ticket's fix. commandedDutyLeft/Right() (and therefore
  // plannerDuty()) must stay EXACTLY zero the whole time: this is the
  // "silently reports zero" regression both scripts hit, reproduced here
  // at the same C ABI boundary.
  float left = -1.0f, right = -1.0f;
  for (int i = 0; i < 20; ++i) {
    state.time.cycleStart = now;
    plannerTick(planner, &state, &result);
    plannerUpdate(planner, &state);
    now += 50;
    plant.step(state, 0.050f, now);
    plannerDuty(planner, &left, &right);
    CHECK_NEAR(left, 0.0f, 1e-9f);
    CHECK_NEAR(right, 0.0f, 1e-9f);
  }
  // Sanity: the planner really is commanding motion this whole time (a
  // Move is active with a nonzero staged velocity target) -- the zero duty
  // reading above is the PARKED-stage artifact, not an idle planner.
  CHECK(state.wheelLeft.cmdVelocity > 50.0f);

  // Now call plannerStageDuty() explicitly, right where tick() used to
  // call it internally -- exactly what both bench scripts' fixed call
  // sites now do. Duty must become nonzero and track the commanded sign.
  plannerStageDuty(planner, 0.05f);
  plannerDuty(planner, &left, &right);
  CHECK(left > 0.05f);
  CHECK(right > 0.05f);

  plannerDestroy(planner);
}

}  // namespace

int main() {
  testDutyReadsZeroUntilStageDutyCalled();
  std::printf("capi_stage_duty_test: all checks passed\n");
  return 0;
}
