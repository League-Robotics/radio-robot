#pragma once

#include "cmon-pid.h"

/**
 * VelocityController — per-wheel PI + feed-forward velocity controller.
 *
 * Implements the Layer-1 wheel velocity loop defined in
 * docs/kinematics-model.md §2.1.
 *
 * Control law (one tick):
 *   err    = setpoint - measured
 *   ff     = kFF * |setpoint|
 *   piOut  = _pid.Update(err)   [cmon-pid backcalculation<pid_bwe>, PI only]
 *                                (integrator bounded by ±iMax via back-calculation;
 *                                 deadband gate suppresses Update when |setpoint|
 *                                 < minWheelMms)
 *   rawPwm = sign(setpoint) * ff + piOut
 *   output = clamp(rawPwm, -100, +100)
 *
 * Integral/anti-windup core is delegated to a composed
 * backcalculation_t<pid_bwe_exposed> (Sprint 049-003).
 * kAw maps to the back-calculation tracking time: Tw = 1/kAw.
 * iMax is the PI-output limit passed to Backcalculation(); the ±100
 * PWM clamp is applied by the wrapper after adding the FF term.
 *
 * Thread safety: not thread-safe. Call from single tick loop only.
 */

// ---------------------------------------------------------------------------
// Thin shim: expose the protected integrator state for public inspection.
// pid_bwe::I is protected; pid_bwe_exposed adds a const accessor so
// VelocityController can sync the public `integral` field each tick.
// ---------------------------------------------------------------------------
class pid_bwe_exposed : public pid_bwe {
public:
    float getI() const { return I; }
};

class VelocityController {
public:
    /**
     * Construct with gain set and deadband.
     *
     * @param kFF        Feed-forward coefficient: FF term = kFF * |setpoint|
     * @param kP         Proportional gain
     * @param kI         Integral gain
     * @param iMax       Integrator anti-windup clamp (PWM% units, symmetric ±).
     *                   Passed as the output-saturation limit to backcalculation_t;
     *                   the ±100 PWM clamp is applied by the wrapper after FF.
     * @param minWheelMms Low-speed deadband: integrator frozen below this |setpoint|
     * @param kAw        Back-calculation anti-windup gain (1/s). When the PI output
     *                   saturates at ±iMax, the integrator is bled toward the
     *                   un-saturated value at this rate. 0 = no back-calculation
     *                   (cW becomes 0, integrator only bounded by iMax clamp).
     */
    VelocityController(float kFF, float kP, float kI, float iMax, float minWheelMms,
                       float kAw = 0.0f);

    /**
     * update — compute one control tick.
     *
     * @param setpoint  Wheel speed command in mm/s (signed; forward > 0)
     * @param measured  Measured wheel speed in mm/s (signed)
     * @param dt_s      Elapsed time since last call in seconds
     * @return          PWM% output in [-100, +100]
     */
    float update(float setpoint, float measured, float dt_s);

    /** reset — zero the integrator state. Call at command start. */
    void reset();

    /**
     * reconfigurePid — re-apply current public gain fields into the cmon-pid
     * instance. Call this after modifying kP / kI / iMax / kAw at runtime
     * (e.g. from MotorController::updateVelGains()). Uses the nominal dt.
     */
    void reconfigurePid();

    // Gains — public so MotorController can update them from config at runtime.
    float kFF;
    float kP;
    float kI;
    float iMax;
    float minWheelMms;
    float kAw;       // back-calculation anti-windup gain (1/s); 0 = no back-calculation

    // Integrator state — synced from cmon-pid internal state each tick for
    // public inspection. Do not write directly; use reset() to zero.
    float integral;

private:
    // cmon-pid PI + back-calculation anti-windup core (Sprint 049-003).
    // pid_bwe_exposed exposes the protected integrator `I` for the `integral`
    // public field. Configured by _configurePid(); reconfigured on each
    // updateVelGains() call.
    backcalculation_t<pid_bwe_exposed> _pid;

    // Reconfigure _pid parameters from the current public gain fields and dt_s.
    // Called from constructor (with a nominal dt) and from updateVelGains()
    // (via MotorController) whenever gains change at runtime.
    void _configurePid(float dt_s);

    static float clamp(float v, float lo, float hi);
};
