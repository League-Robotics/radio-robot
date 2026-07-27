// wheel_trim.h -- Motion::WheelTrim: the closed-loop half of the wheel
// controller, in the VELOCITY domain.
//
// The loop's one actuation contract is a wheel VELOCITY
// (Types::RobotState::Wheel::cmdVelocity); App::Drive turns that into duty
// through a per-wheel, per-direction affine map measured on the bench. So
// the planner's feedback correction is a velocity too:
//
//     cmdVelocity = profiledTarget + trim
//
// The profile says where the wheel should be going; the trim says how much
// MORE (or less) to ask for so it actually goes there. Drive's calibrated
// map remains the feedforward and the planner never has to model the
// plant's gain at all.
//
// THERE IS NO kff, DELIBERATELY -- not even defaulted to zero. In the
// older duty formulation `duty = kff*target + ...`, kff WAS the whole
// plant inverse. Here the feedforward has already happened twice: the
// profiled target is itself the "1 * target" term, and Drive then inverts
// its measured map on the sum. A kff*target term would be additive on top
// of a target that is already present -- kff = 1 would command double
// speed, and pasting the old duty-domain value across would be a silent
// scale error. A residual scale correction belongs in Drive's own measured
// gain, where it is per-wheel and per-direction; not here.
//
// kaff, by contrast, gets SIMPLER in this domain. To make a first-order
// wheel actually travel at v while ramping, command v + tau*dv/dt. So kaff
// is just the plant time constant in seconds -- directly measurable, and
// unchanged by any recalibration of the gain. In the duty domain the same
// term was tau*kff, a product that drifted with battery voltage and with
// every recalibration.
//
// Fail-closed: all-zero gains (the default) produce exactly zero trim, so
// cmdVelocity is bit-for-bit the profiled target and every pre-existing
// behavior is untouched.
#pragma once

#include "planner_types.h"

namespace Motion {

struct VelocityTrimGains {
  // DIMENSIONLESS: mm/s of extra command per mm/s of velocity error.
  // kp == 1 means "10 mm/s slow -> ask for 10 mm/s more". Measured wheel
  // velocity is a naive per-tick difference quotient, so a kp near 1 pumps
  // encoder noise straight through; start well below it.
  float kp = 0.0f;    // [1]
  float ki = 0.0f;    // [1/s]
  float iMax = 0.0f;  // [mm/s] integrator clamp; 0 disables integration
  // Acceleration feedforward ~= the plant time constant. 0 = off.
  float kaff = 0.0f;  // [s]
  // Total trim authority. Bounds how far the correction may pull the
  // command away from the profile -- the profile owns the trajectory, the
  // trim only closes the residual. 0 = unclamped.
  float trimMax = 0.0f;  // [mm/s]
};

class WheelTrim {
 public:
  void configure(const VelocityTrimGains& gains) { gains_ = gains; }

  // One control step -> the velocity correction [mm/s] to ADD to the
  // profiled target.
  //
  // The integrator engages ONLY in MovePhase::Hold. During accel and decel
  // the tracking error is transient work that belongs to kaff and kp;
  // letting the integral absorb it winds up and then releases at the
  // cruise corner (measured on the duty-domain predecessor as a +20%
  // velocity hump for ~0.5 s). Outside Hold the integrator is FROZEN, not
  // reset -- a chain that ramps between legs must not throw away the trim
  // it just learned and re-learn it every hold.
  float compute(float target, float targetAccel, float measured, float dt,
                MovePhase phase);  // [mm/s] [mm/s^2] [mm/s] [s]

  void reset() { integral_ = 0.0f; }
  float integral() const { return integral_; }  // [mm/s] observability

 private:
  VelocityTrimGains gains_;
  float integral_ = 0.0f;  // [mm/s]
};

}  // namespace Motion
