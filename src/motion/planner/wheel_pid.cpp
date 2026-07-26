#include "wheel_pid.h"

#include <algorithm>
#include <cmath>
#ifdef PLANNER_PID_DEBUG
#include <cstdio>
#endif

namespace Motion {

float WheelPid::compute(float target, float targetAccel, float measured,
                        float dt) {
  const float error = target - measured;
  const float feed = gains_.kff * target + gains_.kaff * targetAccel;
  const float unsaturated = feed + gains_.kp * error + integral_;
  // Conditional integration (anti-windup): freeze the integrator whenever
  // the output is already saturated AND the error would push it further.
  const bool saturatedHigh = unsaturated >= 1.0f && error > 0.0f;
  const bool saturatedLow = unsaturated <= -1.0f && error < 0.0f;
  if (gains_.iMax > 0.0f && !saturatedHigh && !saturatedLow) {
    // Ramp gate: at steady state the integrator runs at full rate (its
    // job -- steady trim, e.g. per-wheel gain mismatch). During commanded
    // ramps it runs at a FRACTION of the rate: enough to keep trimming a
    // persistent asymmetry (which otherwise bends the whole ramp), far
    // too slow to absorb the transient tracking error (which, at full
    // rate, was measured as a +20% velocity hump at cruise entry).
    constexpr float kRampRate = 0.2f;
    const float rate =
        std::fabs(targetAccel) <= gains_.iAccelGate ? 1.0f : kRampRate;
    integral_ += rate * gains_.ki * error * dt;
    integral_ = std::clamp(integral_, -gains_.iMax, gains_.iMax);
  }
  const float duty = feed + gains_.kp * error + integral_;
#ifdef PLANNER_PID_DEBUG
  static int dbgCount = 0;
  if (dbgCount < 200 && (dbgCount++ % 2 == 0)) {
    std::printf("[pid] tgt %7.1f a %7.1f meas %7.1f | kff %+.4f kaff %+.4f "
                "kp %+.4f I %+.4f -> %+.4f\n",
                target, targetAccel, measured, gains_.kff * target,
                gains_.kaff * targetAccel, gains_.kp * error, integral_,
                duty);
  }
#endif
  return std::clamp(duty, -1.0f, 1.0f);
}

}  // namespace Motion
