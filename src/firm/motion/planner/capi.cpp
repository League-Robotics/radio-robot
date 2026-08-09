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

// Everything the bench charts need that is NOT already in RobotState: the
// PROFILED command (state.wheel*.cmdVelocity, bit-for-bit as of 130-005 --
// Motion::Planner no longer carries any wheel-actuation correction of its
// own, see planner.h's own header) and the Move's phase. One call so a
// per-tick capture is one FFI crossing.
//
// 130-005: this used to also return Motion::WheelTrim's trim/integrator
// (deleted -- the wheel-speed controller now lives entirely in Core::DifferentialDrive,
// see drive.h's own header; its bias/fast-PID observability is on Drive's
// own accessors, wired into the wire Telemetry frame instead of this
// bench-only C API).
void plannerTrim(void* planner, float* profiledLeft, float* profiledRight,
                 uint8_t* phase) {
  const auto* p = static_cast<const Motion::Planner*>(planner);
  *profiledLeft = p->commandedLeft();
  *profiledRight = p->commandedRight();
  *phase = static_cast<uint8_t>(p->phase());
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
// 130-009: PlannerLimits regrouped from 34 flat fields into 18 fields
// under four sub-structs (ceilings/plant/landing/tracking) -- offsetof()
// with a nested member-designator (e.g. `offsetof(L, ceilings.vMax)`)
// works the same way for a standard-layout aggregate; each entry below is
// still that field's absolute byte offset from the start of PlannerLimits.
//
// Writes min(count, field count) entries and returns the field count.
uint32_t plannerLimitsOffsets(uint32_t* out, uint32_t count) {
  using L = Motion::PlannerLimits;
  static const uint32_t kOffsets[] = {
      // ceilings
      offsetof(L, ceilings.vMax),        offsetof(L, ceilings.aMax),
      offsetof(L, ceilings.aDecel),      offsetof(L, ceilings.omegaMax),
      offsetof(L, ceilings.alphaMax),    offsetof(L, ceilings.alphaDecel),
      offsetof(L, ceilings.jerkMax),     offsetof(L, ceilings.yawJerkMax),
      // plant
      offsetof(L, plant.trackWidth),     offsetof(L, plant.controlPeriod),
      offsetof(L, plant.actuationDelay), offsetof(L, plant.velocityFilterWeight),
      // landing
      offsetof(L, landing.settleEpsilonLinear),
      offsetof(L, landing.settleEpsilonAngular),
      offsetof(L, landing.settleRestVelocity),
      offsetof(L, landing.settleRestOmega),
      offsetof(L, landing.decelPlanFraction),
      // 134-003 terminal fine-align. alignMaxNudges is int32, the first
      // non-float leaf in this table -- the guard compares OFFSETS, and
      // the int32 keeps Landing's 4-byte stride, so it needs no special
      // handling beyond the harness mirroring its type (ctypes.c_int32).
      offsetof(L, landing.alignTol),
      offsetof(L, landing.alignMaxNudges),
      // tracking
      offsetof(L, tracking.headingHoldGain),
  };
  const uint32_t total = sizeof(kOffsets) / sizeof(kOffsets[0]);
  const uint32_t n = count < total ? count : total;
  for (uint32_t i = 0; i < n; ++i) out[i] = kOffsets[i];
  return total;
}

}  // extern "C"
