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
};

class WheelPid {
 public:
  void configure(const PidGains& gains) { gains_ = gains; }

  // One control step: velocity target vs measured -> duty in [-1, 1].
  float compute(float target, float measured, float dt);  // [mm/s] [mm/s] [s]

  void reset() { integral_ = 0.0f; }
  float integral() const { return integral_; }  // [duty] observability

 private:
  PidGains gains_;
  float integral_ = 0.0f;  // [duty]
};

}  // namespace Motion
