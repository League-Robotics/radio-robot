#include "wheel_trim.h"

#include <algorithm>
#include <cmath>

namespace Motion {

float WheelTrim::compute(float target, float targetAccel, float measured,
                         float dt, MovePhase phase) {
  const float error = target - measured;             // [mm/s]
  const float feed = gains_.kaff * targetAccel;      // [mm/s]
  const float proportional = gains_.kp * error;      // [mm/s]

  const bool clamped = gains_.trimMax > 0.0f;
  // Conditional integration: do not wind further into a clamp we are
  // already pinned against (classic anti-windup -- without it the
  // integrator keeps accumulating while saturated and has to unwind
  // before the output can come off the rail).
  const float provisional = feed + proportional + integral_;
  const bool pushingIntoClamp =
      clamped && ((provisional >= gains_.trimMax && error > 0.0f) ||
                  (provisional <= -gains_.trimMax && error < 0.0f));

  if (phase == MovePhase::Hold && gains_.iMax > 0.0f && gains_.ki != 0.0f &&
      !pushingIntoClamp && dt > 0.0f) {
    integral_ = std::clamp(integral_ + gains_.ki * error * dt, -gains_.iMax,
                           gains_.iMax);
  }

  float trim = feed + proportional + integral_;
  if (clamped) trim = std::clamp(trim, -gains_.trimMax, gains_.trimMax);
  if (!std::isfinite(trim)) return 0.0f;  // fail closed, never inject NaN
  return trim;
}

}  // namespace Motion
