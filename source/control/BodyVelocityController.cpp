// BodyVelocityController.cpp — body-level (v, ω) motion profiler.
//
// See BodyVelocityController.h for full API documentation.
// Architecture reference: .clasi/sprints/017-.../architecture-update.md
// Sprint 017, Ticket 002.

#include "BodyVelocityController.h"
#include "MotorController.h"
#include "BodyKinematics.h"
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
      _v(0.0f), _omega(0.0f), _vTgt(0.0f), _omegaTgt(0.0f)
{
}

// ---------------------------------------------------------------------------
// Setters
// ---------------------------------------------------------------------------

void BodyVelocityController::setTarget(float v_mms, float omega_rads)
{
    _vTgt      = v_mms;
    _omegaTgt  = omega_rads;
}

// ---------------------------------------------------------------------------
// Profile step
// ---------------------------------------------------------------------------

bool BodyVelocityController::advance(float dt_s)
{
    if (dt_s <= 0.0f) {
        return !atTarget();
    }

    // TODO(017): S-curve when jMax > 0 (linear channel) or yawJerkMax > 0 (yaw channel).
    // For now, pure trapezoid for both channels.

    // ------------------------------------------------------------------
    // Linear channel — asymmetric accel / decel trapezoid.
    //
    // Choose acceleration based on whether we are speeding up or slowing
    // down.  "Speeding up" means moving toward larger |v|, which is true
    // when the error (clamped_tgt - _v) pushes v farther from zero.
    // ------------------------------------------------------------------
    float vTgtClamped = clamp(_vTgt, -_cfg.vBodyMax, +_cfg.vBodyMax);
    bool  accelerating = (vTgtClamped - _v) * _v >= 0.0f || _v == 0.0f;
    // Use aDecel when the clamped target is closer to zero than current v
    // (i.e. we are decelerating), otherwise aMax.
    float dv_max = (fabsf(vTgtClamped) >= fabsf(_v) ? _cfg.aMax : _cfg.aDecel) * dt_s;
    _v = approach(_v, vTgtClamped, dv_max);

    // ------------------------------------------------------------------
    // Yaw channel — symmetric trapezoid (single acc/decel limit).
    //
    // yawRateMax and yawAccMax are stored in deg/s and deg/s²; convert
    // to rad/s and rad/s² at the use site.
    // ------------------------------------------------------------------
    float yawRateMax_rad = _cfg.yawRateMax * kDegToRad;
    float yawAccMax_rad  = _cfg.yawAccMax  * kDegToRad;

    float omegaTgtClamped = clamp(_omegaTgt, -yawRateMax_rad, +yawRateMax_rad);
    float domega_max      = yawAccMax_rad * dt_s;
    _omega = approach(_omega, omegaTgtClamped, domega_max);

    // ------------------------------------------------------------------
    // Per-tick ordering invariant:
    //   profile → inverse → saturate → setTarget
    // ------------------------------------------------------------------
    float vL, vR, sL, sR;
    BodyKinematics::inverse(_v, _omega, _cfg.trackwidthMm, vL, vR);
    BodyKinematics::saturate(vL, vR, _cfg.vWheelMax, _cfg.steerHeadroom, sL, sR);
    _mc.setTarget(sL, sR);

    return !atTarget();
}

// ---------------------------------------------------------------------------
// Reset / seed
// ---------------------------------------------------------------------------

void BodyVelocityController::reset()
{
    _v      = 0.0f;
    _omega  = 0.0f;
    _vTgt   = 0.0f;
    _omegaTgt = 0.0f;
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

    return (fabsf(_v     - vTgtClamped)     < 0.5f) &&
           (fabsf(_omega - omegaTgtClamped) < 0.001f);
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
