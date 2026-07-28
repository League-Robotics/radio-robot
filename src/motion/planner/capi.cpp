// capi.cpp -- flat C API over Motion::Planner for the Python ctypes
// harness (motion-planner sketch §7 tier 3). Same compute/save split as
// the C++ surface: plannerTick(const Types::RobotState*, TickResult*) computes,
// plannerUpdate(Types::RobotState*) saves. All structs are C-layout and mirrored
// field-for-field as ctypes.Structure in py/planner_harness.py.
#include <cstddef>

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

// Everything the bench charts need that is NOT already in RobotState: the
// PROFILED command (state.wheel*.cmdVelocity carries profile + trim, so
// the two cannot be separated from the blackboard alone), the trim and its
// integrator, and the Move's phase. One call so a per-tick capture is one
// FFI crossing.
void plannerTrim(void* planner, float* profiledLeft, float* profiledRight,
                 float* trimLeft, float* trimRight, float* integralLeft,
                 float* integralRight, uint8_t* phase) {
  const auto* p = static_cast<const Motion::Planner*>(planner);
  *profiledLeft = p->commandedLeft();
  *profiledRight = p->commandedRight();
  *trimLeft = p->trimLeft();
  *trimRight = p->trimRight();
  *integralLeft = p->trimIntegralLeft();
  *integralRight = p->trimIntegralRight();
  *phase = static_cast<uint8_t>(p->phase());
}

void plannerApplyTrimGains(void* planner, float kp, float ki, float iMax,
                           float kaff, float trimMax) {
  static_cast<Motion::Planner*>(planner)->applyTrimGains(kp, ki, iMax, kaff,
                                                         trimMax);
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

// Per-field OFFSET guard for PlannerLimits -- the struct the harness sets
// by name and therefore the one a mid-struct insertion corrupts silently.
//
// A size-only check is NOT enough, and this is not hypothetical: adding
// five floats mid-struct in C++ and five floats at the END in the ctypes
// mirror leaves both sizes identical while every field after the
// insertion point reads a different member. The failure looked like a
// wildly mistuned controller, not a layout bug. Offsets catch it exactly.
//
// Writes min(count, field count) entries and returns the field count.
uint32_t plannerLimitsOffsets(uint32_t* out, uint32_t count) {
  using L = Motion::PlannerLimits;
  static const uint32_t kOffsets[] = {
      offsetof(L, vMax),                 offsetof(L, aMax),
      offsetof(L, aDecel),               offsetof(L, omegaMax),
      offsetof(L, alphaMax),             offsetof(L, alphaDecel),
      offsetof(L, trackWidth),           offsetof(L, controlPeriod),
      offsetof(L, actuationDelay),       offsetof(L, velocityFilterWeight),
      offsetof(L, otosStaleness),        offsetof(L, headingOtosWeight),
      offsetof(L, requireSettle),        offsetof(L, settleWindow),
      offsetof(L, headingHoldGain),      offsetof(L, velKff),
      offsetof(L, velKp),                offsetof(L, velKi),
      offsetof(L, velIMax),              offsetof(L, velKaff),
      offsetof(L, velIAccelGate),        offsetof(L, dutyFloor),
      offsetof(L, settleEpsilonLinear),  offsetof(L, settleEpsilonAngular),
      offsetof(L, jerkMax),              offsetof(L, yawJerkMax),
      offsetof(L, settleRestVelocity),   offsetof(L, settleRestOmega),
      offsetof(L, trimKp),               offsetof(L, trimKi),
      offsetof(L, trimIMax),             offsetof(L, trimKaff),
      offsetof(L, trimMax),             offsetof(L, decelPlanFraction),
  };
  const uint32_t total = sizeof(kOffsets) / sizeof(kOffsets[0]);
  const uint32_t n = count < total ? count : total;
  for (uint32_t i = 0; i < n; ++i) out[i] = kOffsets[i];
  return total;
}

}  // extern "C"
