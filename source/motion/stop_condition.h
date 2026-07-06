// stop_condition.h -- Motion::evaluateStopCondition: a pure predicate that
// tests one msg::StopCondition against a Motion::MotionBaseline and this
// tick's observations.
//
// Ported from source_old/control/StopCondition.cpp, scoped down to the five
// kinds architecture-update.md (sprint 084) Decision 4 keeps this sprint --
// STOP_TIME/STOP_DISTANCE/STOP_HEADING/STOP_POSITION/STOP_ROTATION -- the
// ones whose underlying observation (clock, encoders, fused pose) already
// exists in the new source/ tree. STOP_SENSOR/STOP_COLOR/STOP_LINE_ANY are
// recognized (the switch is exhaustive) but return StopEvalResult::
// UNSUPPORTED rather than silently never firing -- source/subsystems/
// planner.cpp treats UNSUPPORTED identically to NOT_FIRED (it never
// terminates a goal on its own); a future wire-layer ticket that adds a
// `stop=sensor:`/`color:`/`line:` clause is expected to reject it with
// `ERR badarg` BEFORE it ever reaches here, using this same distinction.
//
// source_old's D-mode-only STOP kinds -- SAFETY_MARGIN (runaway safety net)
// and ARRIVE (stall-forced-completion tag) -- have no equivalent value in
// msg::StopKind at all (the wire schema was never extended with them); they
// are out of scope for this sprint (ticket 084-001's Open Question 1), not
// omitted by oversight.
//
// Pure function: no device I/O, no stored state, host-testable with plain
// msg::* fixtures (see tests/sim/unit/stop_condition_harness.cpp).
#pragma once

#include <stdint.h>

#include "messages/common.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "motion/motion_baseline.h"

namespace Motion {

// StopEvalResult -- the three-way outcome evaluateStopCondition() can
// report. UNSUPPORTED is a distinct value from NOT_FIRED specifically so a
// caller can tell "this condition kind isn't implemented yet" apart from
// "this condition just hasn't fired yet" (architecture-update.md Decision 4).
enum class StopEvalResult : uint8_t {
  NOT_FIRED = 0,
  FIRED = 1,
  UNSUPPORTED = 2,
};

// evaluateStopCondition -- test whether `cond` is satisfied this tick.
//
//   cond       -- the stop condition to evaluate (kind + its a/b/ax/ay/sensor/
//                 cmp params -- see messages/planner.h's msg::StopCondition).
//   base       -- the MotionBaseline captured at goal start.
//   now        -- [ms] current system time (STOP_TIME).
//   leftObs/rightObs -- this tick's sampled MotorState for the two bound
//                 wheels (STOP_DISTANCE/STOP_ROTATION; .position is
//                 Opt<float> -- a condition that needs it and finds .has ==
//                 false reports NOT_FIRED rather than fabricating a phantom
//                 zero-baseline delta).
//   fusedPose  -- this tick's fused pose estimate (STOP_HEADING/
//                 STOP_POSITION).
StopEvalResult evaluateStopCondition(const msg::StopCondition& cond,
                                     const MotionBaseline& base, uint32_t now,
                                     const msg::MotorState& leftObs,
                                     const msg::MotorState& rightObs,
                                     const msg::PoseEstimate& fusedPose);

}  // namespace Motion
