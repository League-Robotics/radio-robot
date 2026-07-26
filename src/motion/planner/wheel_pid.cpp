#include "wheel_pid.h"

#include <algorithm>

namespace Motion {

float WheelPid::compute(float target, float measured, float dt) {
  const float error = target - measured;
  const float unsaturated =
      gains_.kff * target + gains_.kp * error + integral_;
  // Conditional integration (anti-windup): freeze the integrator whenever
  // the output is already saturated AND the error would push it further.
  const bool saturatedHigh = unsaturated >= 1.0f && error > 0.0f;
  const bool saturatedLow = unsaturated <= -1.0f && error < 0.0f;
  if (gains_.iMax > 0.0f && !saturatedHigh && !saturatedLow) {
    integral_ += gains_.ki * error * dt;
    integral_ = std::clamp(integral_, -gains_.iMax, gains_.iMax);
  }
  const float duty = gains_.kff * target + gains_.kp * error + integral_;
  return std::clamp(duty, -1.0f, 1.0f);
}

}  // namespace Motion
