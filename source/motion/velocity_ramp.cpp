// velocity_ramp.cpp -- Motion::VelocityRamp implementation. See
// velocity_ramp.h for the class-level design notes (in particular the unit
// note on yaw_rate_max/yaw_acc_max/yaw_jerk_max).
#include "motion/velocity_ramp.h"

#include <math.h>

namespace Motion {

void VelocityRamp::setTarget(float v, float omega) {
  vTgt_ = v;
  omegaTgt_ = omega;
}

void VelocityRamp::configure(const msg::PlannerConfig& config) { config_ = config; }

float VelocityRamp::approach(float cur, float tgt, float step) {
  float delta = tgt - cur;
  if (delta > step) delta = step;
  if (delta < -step) delta = -step;
  return cur + delta;
}

float VelocityRamp::clamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool VelocityRamp::advance(float dt) {
  if (dt <= 0.0f) {
    return !atTarget();
  }

  // ---- Linear channel: asymmetric accel/decel, optional jerk limit. ----
  float vTgtClamped = clamp(vTgt_, -config_.v_body_max, config_.v_body_max);

  if (config_.j_max > 0.0f) {
    float aTarget = (v_ < vTgtClamped) ? config_.a_max
                    : (v_ > vTgtClamped) ? -config_.a_decel
                                         : 0.0f;
    float jerkStep = config_.j_max * dt;
    aLive_ = approach(aLive_, aTarget, jerkStep);
    v_ = approach(v_, vTgtClamped, fabsf(aLive_ * dt));
  } else {
    float dvMax = (fabsf(vTgtClamped) >= fabsf(v_) ? config_.a_max : config_.a_decel) * dt;
    v_ = approach(v_, vTgtClamped, dvMax);
  }

  // ---- Yaw channel: symmetric trapezoid, optional jerk limit. ----
  float omegaTgtClamped = clamp(omegaTgt_, -config_.yaw_rate_max, config_.yaw_rate_max);

  if (config_.yaw_jerk_max > 0.0f) {
    float omegaATarget = (omega_ < omegaTgtClamped) ? config_.yaw_acc_max
                          : (omega_ > omegaTgtClamped) ? -config_.yaw_acc_max
                                                        : 0.0f;
    omegaALive_ = approach(omegaALive_, omegaATarget, config_.yaw_jerk_max * dt);
    omega_ = approach(omega_, omegaTgtClamped, fabsf(omegaALive_ * dt));
  } else {
    float domegaMax = config_.yaw_acc_max * dt;
    omega_ = approach(omega_, omegaTgtClamped, domegaMax);
  }

  return !atTarget();
}

void VelocityRamp::reset() {
  v_ = 0.0f;
  omega_ = 0.0f;
  vTgt_ = 0.0f;
  omegaTgt_ = 0.0f;
  aLive_ = 0.0f;
  omegaALive_ = 0.0f;
}

void VelocityRamp::seedCurrent(float v, float omega) {
  v_ = v;
  omega_ = omega;
}

bool VelocityRamp::atTarget() const {
  float vTgtClamped = clamp(vTgt_, -config_.v_body_max, config_.v_body_max);
  float omegaTgtClamped = clamp(omegaTgt_, -config_.yaw_rate_max, config_.yaw_rate_max);
  return (fabsf(v_ - vTgtClamped) < 0.5f) && (fabsf(omega_ - omegaTgtClamped) < 0.001f);
}

}  // namespace Motion
