// capi.cpp -- flat C API over Motion::Planner for the Python ctypes
// harness (motion-planner sketch §7 tier 3). Same compute/save split as
// the C++ surface: plannerTick(const Types::RobotState*, TickResult*) computes,
// plannerUpdate(Types::RobotState*) saves. All structs are C-layout and mirrored
// field-for-field as ctypes.Structure in py/planner_harness.py.
#include "planner.h"

extern "C" {

void* plannerCreate(const Motion::PlannerLimits* limits) {
  return new Motion::Planner(*limits);
}

void plannerDestroy(void* planner) {
  delete static_cast<Motion::Planner*>(planner);
}

bool plannerMove(void* planner, const Motion::Move* next, bool replace) {
  return static_cast<Motion::Planner*>(planner)->move(*next, replace);
}

void plannerStop(void* planner) {
  static_cast<Motion::Planner*>(planner)->stop();
}

void plannerTick(void* planner, const Types::RobotState* state,
                 Motion::TickResult* result) {
  *result = static_cast<Motion::Planner*>(planner)->tick(*state);
}

void plannerUpdate(void* planner, Types::RobotState* state) {
  static_cast<const Motion::Planner*>(planner)->update(*state);
}

// Layout guard for the ctypes mirror: the harness compares these against
// ctypes.sizeof() before doing anything else, so a struct drift fails
// loudly instead of corrupting silently.
void plannerStructSizes(uint32_t* robotState, uint32_t* move,
                        uint32_t* limits, uint32_t* tickResult) {
  *robotState = sizeof(Types::RobotState);
  *move = sizeof(Motion::Move);
  *limits = sizeof(Motion::PlannerLimits);
  *tickResult = sizeof(Motion::TickResult);
}

}  // extern "C"
