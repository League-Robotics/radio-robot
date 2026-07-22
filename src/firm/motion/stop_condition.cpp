// stop_condition.cpp -- Motion::StopCondition implementation. See
// stop_condition.h's file header for the module's narrow boundary.
#include "motion/stop_condition.h"

#include <cmath>

namespace Motion {

namespace {

// Malformed-input-safety clamp shared by threshold and timeout -- see
// stop_condition.h's constructor doc comment ("Zero/negative threshold").
// NaN comparisons are always false, so a NaN input clamps to 0 exactly
// like a negative one (`value > 0.0f` is false for both).
float clampPositive(float value) { return (value > 0.0f) ? value : 0.0f; }

// [ms] -> [us], matching the deleted App::Deadman::arm()'s own conversion
// idiom: multiply while still float-typed so sub-millisecond fractions of
// `ms` aren't truncated before the unit conversion.
uint64_t millisToMicros(float ms) {
  return static_cast<uint64_t>(ms * 1000.0f);
}

}  // namespace

StopCondition::StopCondition(Kind kind, float threshold, float timeout,
                              uint64_t now, float pathLength, float theta)
    : kind_(kind),
      threshold_(clampPositive(threshold)),
      timeDeadlineUs_(now + millisToMicros(clampPositive(threshold))),
      timeoutDeadlineUs_(now + millisToMicros(clampPositive(timeout))),
      activationPathLength_(pathLength),
      activationTheta_(theta) {}

StopCondition::Outcome StopCondition::tick(uint64_t now, float pathLength,
                                            float theta) const {
  bool stopConditionMet = false;
  switch (kind_) {
    case Kind::Time:
      stopConditionMet = now >= timeDeadlineUs_;
      break;
    case Kind::Distance:
      stopConditionMet =
          std::fabs(pathLength - activationPathLength_) >= threshold_;
      break;
    case Kind::Angle:
      stopConditionMet = std::fabs(theta - activationTheta_) >= threshold_;
      break;
  }

  // Tie-break: the kind-specific result always wins -- see tick()'s own
  // doc comment in stop_condition.h.
  if (stopConditionMet) return Outcome::StopConditionMet;
  if (now >= timeoutDeadlineUs_) return Outcome::TimedOut;
  return Outcome::Continue;
}

}  // namespace Motion
