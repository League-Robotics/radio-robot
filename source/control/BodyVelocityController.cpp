// BodyVelocityController.cpp — body-level (v, ω) motion profiler.
//
// See BodyVelocityController.h for full API documentation.
// Architecture reference: .clasi/sprints/017-.../architecture-update.md
// Sprint 017, Ticket 002.

#include "BodyVelocityController.h"
#include "MotorController.h"
// IKinematics.h provides the Kinematics:: alias and kWheelCount:
//   mecanum build  → Kinematics = MecanumKinematics, kWheelCount = 4
//   differential   → Kinematics = BodyKinematics,   kWheelCount = 2
// The differential advance() path still calls BodyKinematics:: directly
// (not via the alias) to avoid touching the shared code path.
#include "IKinematics.h"
#include <math.h>

// ---------------------------------------------------------------------------
// M_PI guard — micro:bit / ARMCC may not define M_PI by default.
// ---------------------------------------------------------------------------
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float kDegToRad = (float)(M_PI / 180.0);

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

BodyVelocityController::BodyVelocityController(MotorController& mc,
                                                 const RobotConfig& cfg)
    : _mc(mc), _cfg(cfg),
      _v(0.0f), _omega(0.0f), _vTgt(0.0f), _omegaTgt(0.0f),
      _aLive(0.0f), _omegaALive(0.0f)
#ifdef ROBOT_DRIVETRAIN_MECANUM
      , _vy(0.0f), _vyTgt(0.0f), _vyALive(0.0f)
      , _geom{ cfg.halfTrackMm, cfg.halfWheelbaseMm }
#endif
{
}

// ---------------------------------------------------------------------------
// Setters
// ---------------------------------------------------------------------------

#ifdef ROBOT_DRIVETRAIN_MECANUM
void BodyVelocityController::setTarget(float v_mms, float omega_rads, float vy_mms)
{
    _vTgt     = v_mms;
    _omegaTgt = omega_rads;
    _vyTgt    = vy_mms;
}
#else
void BodyVelocityController::setTarget(float v_mms, float omega_rads)
{
    _vTgt      = v_mms;
    _omegaTgt  = omega_rads;
}
#endif

// ---------------------------------------------------------------------------
// Profile step
// ---------------------------------------------------------------------------

bool BodyVelocityController::advance(float dt_s)
{
    if (dt_s <= 0.0f) {
        return !atTarget();
    }

    // ------------------------------------------------------------------
    // Linear channel — asymmetric accel/decel with optional jerk limit.
    //
    // At jMax == 0 (default): pure trapezoid — approach v directly under
    // the per-tick dv_max step (identical to pre-018 behaviour).
    //
    // At jMax > 0: S-curve — slew _aLive toward the demanded acceleration
    // under the jerk bound (jMax * dt_s), then integrate v += _aLive*dt_s.
    //
    // "Demanded accel" = +aMax when target is farther from zero than v
    // (speeding up), -aDecel otherwise (slowing down).
    // ------------------------------------------------------------------
    float vTgtClamped = clamp(_vTgt, -_cfg.vBodyMax, +_cfg.vBodyMax);

    if (_cfg.jMax > 0.0f) {
        // S-curve path: jerk-limit the acceleration, then integrate toward
        // the target using approach() so the integration cannot overshoot.
        //
        // aTarget: +aMax when v must increase to reach vTgtClamped, else -aDecel.
        float aTarget = (_v < vTgtClamped) ? _cfg.aMax
                      : (_v > vTgtClamped) ? -_cfg.aDecel
                      : 0.0f;
        float jerkStep = _cfg.jMax * dt_s;
        _aLive = approach(_aLive, aTarget, jerkStep);
        // Integrate, but cap the step so we never overshoot vTgtClamped.
        // fabsf guards against negative _aLive pointing away from target.
        _v = approach(_v, vTgtClamped, fabsf(_aLive * dt_s));
    } else {
        // Trapezoid path (jMax == 0): identical to pre-018 behaviour.
        // Use aDecel when the clamped target is closer to zero than current v
        // (i.e. we are decelerating), otherwise aMax.
        float dv_max = (fabsf(vTgtClamped) >= fabsf(_v) ? _cfg.aMax : _cfg.aDecel) * dt_s;
        _v = approach(_v, vTgtClamped, dv_max);
    }

    // ------------------------------------------------------------------
    // Yaw channel — symmetric trapezoid with optional jerk limit.
    //
    // yawRateMax and yawAccMax are stored in deg/s and deg/s²; convert
    // to rad/s and rad/s² at the use site.
    //
    // At yawJerkMax == 0 (default): pure trapezoid (identical to pre-018).
    // At yawJerkMax > 0: S-curve on omega via _omegaALive.
    // ------------------------------------------------------------------
    float yawRateMax_rad = _cfg.yawRateMax * kDegToRad;
    float yawAccMax_rad  = _cfg.yawAccMax  * kDegToRad;

    float omegaTgtClamped = clamp(_omegaTgt, -yawRateMax_rad, +yawRateMax_rad);

    if (_cfg.yawJerkMax > 0.0f) {
        // S-curve path for yaw.  Same approach-based integration as linear
        // channel: prevents overshoot while preserving jerk-limited ramp.
        float yawJerkMaxRad = _cfg.yawJerkMax * kDegToRad;
        float omegaATarget  = (_omega < omegaTgtClamped) ? +yawAccMax_rad
                            : (_omega > omegaTgtClamped) ? -yawAccMax_rad
                            : 0.0f;
        _omegaALive = approach(_omegaALive, omegaATarget, yawJerkMaxRad * dt_s);
        _omega = approach(_omega, omegaTgtClamped, fabsf(_omegaALive * dt_s));
    } else {
        // Trapezoid path (yawJerkMax == 0): identical to pre-018 behaviour.
        float domega_max = yawAccMax_rad * dt_s;
        _omega = approach(_omega, omegaTgtClamped, domega_max);
    }

    // ------------------------------------------------------------------
    // Per-tick ordering invariant:
    //   profile → inverse → saturate → setTarget
    // ------------------------------------------------------------------
#ifdef ROBOT_DRIVETRAIN_MECANUM
    // 046-005: Lateral (vy) channel — trapezoid/S-curve mirroring forward.
    float vyTgtClamped = clamp(_vyTgt, -_cfg.vyBodyMax, +_cfg.vyBodyMax);
    if (_cfg.jMaxY > 0.0f) {
        float aTargetY = (_vy < vyTgtClamped) ? _cfg.aMaxY
                       : (_vy > vyTgtClamped) ? -_cfg.aMaxY
                       : 0.0f;
        float jerkStepY = _cfg.jMaxY * dt_s;
        _vyALive = approach(_vyALive, aTargetY, jerkStepY);
        _vy = approach(_vy, vyTgtClamped, fabsf(_vyALive * dt_s));
    } else {
        float dvy_max = _cfg.aMaxY * dt_s;
        _vy = approach(_vy, vyTgtClamped, dvy_max);
    }

    // Mecanum inverse kinematics: body twist (vx, vy, omega) → 4 wheel speeds.
    //
    // 046-008: The kinematics works in the LOGICAL convention (forward-positive
    // per wheel). The per-wheel PHYSICAL forward sign (cfg.fwdSign*) is applied
    // by the HAL Motor layer — Motor::setSpeed multiplies the PWM by _fwdSign and
    // the encoder read applies it too — exactly as the differential drive does.
    // Applying cfg.fwdSign here as well double-applied it (±1 squared = +1),
    // cancelling the correction so every wheel was driven by the raw value (all
    // wheels spun the same way on a forward command). So use identity signs here
    // and let the Motor own the physical sign. (The sim's SimMotor ignores
    // fwdSign, so identity is also correct in simulation.)
    const int8_t signs[4] = { 1, 1, 1, 1 };
    float wheels[kWheelCount];
    float satWheels[kWheelCount];
    BodyTwist3 twist{ _v, _vy, _omega };
    Kinematics::inverse(twist, _geom, signs, wheels);
    Kinematics::saturate(wheels, _cfg.vWheelMax, satWheels);

    // Anti-windup: if any wheel was saturated, back-calculate the effective twist.
    bool saturated = false;
    for (int i = 0; i < kWheelCount; ++i) {
        if (satWheels[i] != wheels[i]) { saturated = true; break; }
    }
    if (saturated) {
        BodyTwist3 backCalc{};
        Kinematics::forward(satWheels, _geom, signs, backCalc);
        _v     = backCalc.vx_mmps;
        _vy    = backCalc.vy_mmps;
        _omega = backCalc.omega_rads;
    }
    _mc.setTarget(satWheels, kWheelCount);
#else
    float vL, vR, sL, sR;
    BodyKinematics::inverse(_v, _omega, _cfg.trackwidthMm, vL, vR);
    BodyKinematics::saturate(vL, vR, _cfg.vWheelMax, _cfg.steerHeadroom, sL, sR);
    // Anti-windup: if saturation clipped the output, back-calculate the effective
    // body velocity so _v never builds up past the saturation ceiling.  Without
    // this, _v silently overruns during ramp-up and produces a flat-spot plateau
    // at the start of deceleration while _v burns back down to the ceiling.
    if (sL != vL || sR != vR) {
        BodyKinematics::forward(sL, sR, _cfg.trackwidthMm, _v, _omega);
    }
    _mc.setTarget(sL, sR);
#endif  // ROBOT_DRIVETRAIN_MECANUM

    // Publish live and raw body twist to DesiredState (047-003).
    // _vy / _vyTgt are always 0 in the differential build (field absent in private
    // members); they are present in the mecanum build via the #ifdef block above.
    if (_ds) {
#ifdef ROBOT_DRIVETRAIN_MECANUM
        _ds->bodyTwist    = {_v,    _vy,    _omega};
        _ds->bodyTwistRaw = {_vTgt, _vyTgt, _omegaTgt};
#else
        _ds->bodyTwist    = {_v,    0.0f,   _omega};
        _ds->bodyTwistRaw = {_vTgt, 0.0f,   _omegaTgt};
#endif
    }

    return !atTarget();
}

// ---------------------------------------------------------------------------
// Reset / seed
// ---------------------------------------------------------------------------

void BodyVelocityController::reset()
{
    _v         = 0.0f;
    _omega     = 0.0f;
    _vTgt      = 0.0f;
    _omegaTgt  = 0.0f;
    _aLive     = 0.0f;
    _omegaALive = 0.0f;
#ifdef ROBOT_DRIVETRAIN_MECANUM
    _vy        = 0.0f;
    _vyTgt     = 0.0f;
    _vyALive   = 0.0f;
#endif
}

void BodyVelocityController::seedCurrent(float v_mms, float omega_rads)
{
    _v     = v_mms;
    _omega = omega_rads;
}

// ---------------------------------------------------------------------------
// Convergence test
// ---------------------------------------------------------------------------

bool BodyVelocityController::atTarget() const
{
    float vTgtClamped     = clamp(_vTgt, -_cfg.vBodyMax, +_cfg.vBodyMax);
    float yawRateMax_rad  = _cfg.yawRateMax * kDegToRad;
    float omegaTgtClamped = clamp(_omegaTgt, -yawRateMax_rad, +yawRateMax_rad);

#ifdef ROBOT_DRIVETRAIN_MECANUM
    float vyTgtClamped = clamp(_vyTgt, -_cfg.vyBodyMax, +_cfg.vyBodyMax);
    return (fabsf(_v     - vTgtClamped)     < 0.5f) &&
           (fabsf(_omega - omegaTgtClamped) < 0.001f) &&
           (fabsf(_vy    - vyTgtClamped)    < 0.5f);
#else
    return (fabsf(_v     - vTgtClamped)     < 0.5f) &&
           (fabsf(_omega - omegaTgtClamped) < 0.001f);
#endif
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

float BodyVelocityController::approach(float cur, float tgt, float step)
{
    float delta = tgt - cur;
    if (delta > +step) delta = +step;
    if (delta < -step) delta = -step;
    return cur + delta;
}

float BodyVelocityController::clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
