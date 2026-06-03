#pragma once

/**
 * VelocityController — per-wheel PI + feed-forward velocity controller.
 *
 * Implements the Layer-1 wheel velocity loop defined in
 * docs/kinematics-model.md §2.1.
 *
 * Control law (one tick):
 *   err    = setpoint - measured
 *   I     += kI * err * dt_s          (unless frozen by anti-windup or deadband)
 *   I      = clamp(I, -iMax, +iMax)
 *   rawPwm = kFF * |setpoint| + kP * err + I
 *   output = sign(setpoint) * clamp(|rawPwm|, 0, 100)
 *            (or clamp(rawPwm, -100, 100) for bidirectional)
 *
 * Anti-windup: integrator frozen when |rawPwm| >= 100 (output is rail-limited).
 * Deadband:    integrator not accumulated when |setpoint| < minWheelMms.
 * Direction:   sign of output follows sign of setpoint.
 *
 * Thread safety: not thread-safe. Call from single tick loop only.
 */
class VelocityController {
public:
    /**
     * Construct with gain set and deadband.
     *
     * @param kFF        Feed-forward coefficient: FF term = kFF * |setpoint|
     * @param kP         Proportional gain
     * @param kI         Integral gain
     * @param iMax       Integrator anti-windup clamp (PWM% units, symmetric ±)
     * @param minWheelMms Low-speed deadband: integrator frozen below this |setpoint|
     */
    VelocityController(float kFF, float kP, float kI, float iMax, float minWheelMms);

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

    // Gains — public so MotorController can update them from config at runtime.
    float kFF;
    float kP;
    float kI;
    float iMax;
    float minWheelMms;

    float integral;  // integrator state (public for inspection/testing)

private:
    static float clamp(float v, float lo, float hi);
};
