// VelocityController.cpp — per-wheel PI + feed-forward velocity controller.
//
// See docs/kinematics-model.md §2.1 for control law derivation.
// Sprint 010, Ticket 003.

#include "VelocityController.h"
#include <math.h>

VelocityController::VelocityController(float kFF_, float kP_, float kI_,
                                         float iMax_, float minWheelMms_)
    : kFF(kFF_), kP(kP_), kI(kI_), iMax(iMax_), minWheelMms(minWheelMms_),
      integral(0.0f)
{
}

float VelocityController::update(float setpoint, float measured, float dt_s)
{
    if (dt_s <= 0.0f) return 0.0f;

    // Error: positive when measured is slower than setpoint.
    float err = setpoint - measured;

    // Feed-forward: drives proportional to |setpoint|, signed by setpoint direction.
    float spAbs  = fabsf(setpoint);
    float spSign = (setpoint >= 0.0f) ? 1.0f : -1.0f;
    float ff     = kFF * spAbs;

    // Compute raw output before clamping (to detect rail saturation for anti-windup).
    float rawPwm = spSign * ff + kP * err + integral;

    // Anti-windup: freeze integrator when output is rail-limited.
    bool saturated = (rawPwm >= 100.0f) || (rawPwm <= -100.0f);

    // Deadband: freeze integrator when commanded speed is below threshold.
    bool inDeadband = (spAbs < minWheelMms);

    if (!saturated && !inDeadband) {
        integral += kI * err * dt_s;
        integral = clamp(integral, -iMax, iMax);
    }

    // Recompute output with (possibly updated) integrator.
    float output = spSign * ff + kP * err + integral;

    return clamp(output, -100.0f, 100.0f);
}

void VelocityController::reset()
{
    integral = 0.0f;
}

float VelocityController::clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
