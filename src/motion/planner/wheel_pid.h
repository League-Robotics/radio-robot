// wheel_pid.h -- Motion::WheelPid: the M4 duty-plane output stage (issue
// §7.6 / motion-planner sketch §5): one wheel's velocity-target -> duty
// control law, relocated INTO the motion library so one velocity estimate
// (the planner's filtered channel) feeds one controller, tunable on the
// host. duty = kff*target + kp*error + integral, integrator clamped and
// conditionally frozen while the output is saturated (anti-windup).
//
// Fail-closed: all-zero gains (the default) produce duty 0 -- the duty
// plane is OFF unless gains are configured, and every pre-existing
// velocity-plane consumer is untouched.
#pragma once

namespace Motion {

struct PidGains {
  float kff = 0.0f;   // [duty/(mm/s)] feedforward slope
  float kp = 0.0f;    // [duty/(mm/s)] proportional
  float ki = 0.0f;    // [duty/(mm/s)/s] integral rate
  float iMax = 0.0f;  // [duty] integrator clamp (0 disables integration)
  // Acceleration feedforward (~= plantTau * kff): a first-order plant
  // needs (v + tau*dv/dt)/gain of duty to TRACK a ramp. With this term
  // the ramp tracking error stays near zero, so the integral never winds
  // during ramps -- the cruise-entry overshoot spike was that windup
  // releasing. 0 = off.
  float kaff = 0.0f;  // [duty/(mm/s^2)]
  // Integral ramp gate: the integrator engages ONLY while the COMMANDED
  // acceleration is below this threshold -- i.e. at (or near) steady
  // state, where trimming is its job. During commanded ramps the
  // tracking error is transient work that belongs to kff/kaff/kp;
  // letting the integral absorb it was measured as a +20% velocity hump
  // at cruise entry (the wound integral over-delivering, then unwinding
  // for ~0.5 s). Error-magnitude separation cannot make this
  // distinction (ramp errors are small but persistent); the commanded
  // accel can. Default: wide open, preserving prior behavior.
  float iAccelGate = 1.0e9f;  // [mm/s^2]
};

class WheelPid {
 public:
  void configure(const PidGains& gains) { gains_ = gains; }

  // One control step: velocity target (+ the commanded accel behind it)
  // vs measured -> duty in [-1, 1].
  float compute(float target, float targetAccel, float measured,
                float dt);  // [mm/s] [mm/s^2] [mm/s] [s]

  void reset() { integral_ = 0.0f; }
  float integral() const { return integral_; }  // [duty] observability

 private:
  PidGains gains_;
  float integral_ = 0.0f;  // [duty]
};

}  // namespace Motion
