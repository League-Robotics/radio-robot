#pragma once
#include "MicroBit.h"
#include "NezhaV2.h"
#include "Config.h"

/**
 * MotorController — PI + feed-forward wheel speed control.
 *
 * Owns two independent PI integrators (left, right) and a ratio
 * cross-coupling correction. Sprint 5 replaces the tick() body with
 * a ratio PID; callers in CommandProcessor are unchanged.
 *
 * Thread safety: single-threaded tick loop only.
 */
class MotorController {
public:
    explicit MotorController(NezhaV2& motor, const CalibParams& cal);

    // Gains — public so CommandProcessor can update via K-commands.
    // Defaults: kFF=0.15, kP=0.05, kI=0.20, iClamp=60, kRatio=0.01
    struct Gains {
        float kFF;      // feed-forward coefficient
        float kP;       // proportional gain
        float kI;       // integral gain
        float iClamp;   // integral windup clamp (PWM units, ±)
        float kRatio;   // ratio cross-coupling gain (sprint 2 stub, small)
    } gains;

    // Set speed targets in mm/s. Zero both to coast (not brake).
    void setTarget(float leftMms, float rightMms);

    // Stop: zero targets and reset integrators.
    void stop();

    // Reset integrators only (called by CommandProcessor on mode change,
    // NOT on S-command watchdog refresh — integrators survive keepalives).
    void resetIntegrators();

    // Run one control tick. dt_s is elapsed seconds since last tick.
    // Reads encoders, runs PI+FF+ratio, clamps output, calls NezhaV2::setPwm().
    // Sprint 5 replaces this body only.
    void tick(float dt_s);

    // Read actual wheel velocities in mm/s (encoder delta since last tick).
    void getActualVelocity(float& leftMms, float& rightMms) const;

    // Read cumulative encoder positions in mm (sum since last resetEncoderAccumulators()).
    void getEncoderPositions(int32_t& leftMm, int32_t& rightMm) const;

    // Zero encoder accumulators — delegates to NezhaV2::resetEncoders().
    void resetEncoderAccumulators();

private:
    NezhaV2&           _motor;
    const CalibParams& _cal;

    float _targetL;    // mm/s
    float _targetR;    // mm/s
    float _integralL;  // PI integral accumulator, left wheel
    float _integralR;  // PI integral accumulator, right wheel

    // Cached encoder readings from the most recent tick() call.
    // Used to compute velocity and expose via getActualVelocity().
    int32_t _prevEncL;   // mm at start of last tick
    int32_t _prevEncR;
    float   _actualVelL; // mm/s computed in tick()
    float   _actualVelR;

    // clamp helper
    static float clamp(float v, float lo, float hi);
};
