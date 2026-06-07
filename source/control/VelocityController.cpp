// VelocityController.cpp — per-wheel PI + feed-forward velocity controller.
//
// See docs/kinematics-model.md §2.1 for control law derivation.
// Sprint 010, Ticket 003.

#include "VelocityController.h"
#include <math.h>

VelocityController::VelocityController(float kFF_, float kP_, float kI_,
                                         float iMax_, float minWheelMms_, float kAw_)
    : kFF(kFF_), kP(kP_), kI(kI_), iMax(iMax_), minWheelMms(minWheelMms_),
      kAw(kAw_), integral(0.0f)
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

    // Raw (pre-clamp) command, then the actual rail-limited output.
    float rawPwm = spSign * ff + kP * err + integral;
    float output = clamp(rawPwm, -100.0f, 100.0f);

    // Deadband: don't accumulate at very low commanded speed.
    bool inDeadband = (spAbs < minWheelMms);

    if (!inDeadband) {
        // Integrate with BACK-CALCULATION anti-windup. The (output - rawPwm) term
        // is zero unless the output is saturated; when it is, it bleeds the
        // integrator back toward the un-saturated value at rate kAw. This stops a
        // load disturbance (e.g. a held wheel) from winding the integral up and
        // causing overshoot + a long slow recovery when the load is released.
        // kAw = 0 reduces to plain integration (with the clamp below as the only
        // bound — the legacy behaviour).
        integral += (kI * err + kAw * (output - rawPwm)) * dt_s;
        integral = clamp(integral, -iMax, iMax);
    }

    return output;
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
