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

// The PANIC stop (command-ingestion-...-two-stops.md §5) -- renamed from
// plannerStop() alongside Planner::stop() -> Planner::estop(), so the C ABI
// cannot leave the two stop meanings ambiguous either.
void plannerEstop(void* planner) {
  static_cast<Motion::Planner*>(planner)->estop();
}

// The PLANNED stop: enqueues a Move::Kind::Stop entry that executes in
// sequence and acks `moveId` on arrival. Returns false when the queue is
// full, mirroring plannerMove().
bool plannerPlannedStop(void* planner, uint32_t moveId) {
  return static_cast<Motion::Planner*>(planner)->plannedStop(moveId);
}

void plannerTick(void* planner, const Types::RobotState* state,
                 Motion::TickResult* result) {
  *result = static_cast<Motion::Planner*>(planner)->tick(*state);
}

void plannerUpdate(void* planner, Types::RobotState* state) {
  static_cast<const Motion::Planner*>(planner)->update(*state);
}

// M4 duty-plane outputs -- see Planner::commandedDutyLeft/Right().
void plannerDuty(void* planner, float* left, float* right) {
  const auto* p = static_cast<const Motion::Planner*>(planner);
  *left = p->commandedDutyLeft();
  *right = p->commandedDutyRight();
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
