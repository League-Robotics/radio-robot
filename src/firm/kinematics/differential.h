// differential.h -- Kinematics::Differential: the two-wheel twist<->wheel-
// speed map, plus curvature-preserving saturation.
//
// This is the former `BodyKinematics` free-function namespace
// (src/motion/body_kinematics.h), unchanged math, behind
// Kinematics::Model. The static members below ARE those free functions,
// with the same signatures and the same bodies -- every existing call site
// swapped `BodyKinematics::` for `Differential::` and nothing
// else. The virtual overrides call the statics with the instance's own
// trackWidth_, so there is exactly one copy of each equation.
//
// Wheel order for the array forms: `wheels[0]` is LEFT, `wheels[1]` is
// RIGHT. `v_y` is always 0 -- a differential drive cannot strafe.
//
// All functions are pure: no I2C, no global state, no heap allocation. See
// DESIGN.md (this directory) for the subsystem contract and
// docs/kinematics-model.md §1.3/§1.7 for the math this implements.
#pragma once

#include "kinematics/kinematics.h"

namespace Kinematics {

class Differential : public Model {
 public:
  explicit Differential(float trackWidth)  // [mm]
      : trackWidth_(trackWidth) {}

  int wheelCount() const override { return 2; }

  void inverse(const Twist& twist, float wheels[]) const override;
  void forward(const float wheels[], Twist& twist) const override;

  float trackWidth() const { return trackWidth_; }  // [mm]

  // ---------------------------------------------------------------------
  // The math, as static functions taking the track width explicitly.
  // Identical in name, signature and body to the pre-reorganization
  // BodyKinematics:: free functions.
  // ---------------------------------------------------------------------

  // inverse -- body twist (v, omega) to wheel speeds (vL, vR):
  //   vL = v - omega * (b / 2)
  //   vR = v + omega * (b / 2)
  static void inverse(float v, float omega, float b,
                      float& vL_out, float& vR_out);  // [mm/s] [rad/s] [mm]

  // forward -- wheel speeds (vL, vR) to body twist (v, omega):
  //   v     = (vR + vL) / 2
  //   omega = (vR - vL) / b
  static void forward(float vL, float vR, float b,
                      float& v_out, float& omega_out);  // [mm/s] x2 [mm]

  // saturate -- curvature-preserving wheel-speed saturation. Effective
  // ceiling is (vWheelMax - steerHeadroom); when max(|vL|, |vR|) exceeds
  // it, both are scaled by ceiling / max(|vL|, |vR|), so the faster wheel
  // sits exactly at the ceiling and the ratio (hence arc curvature) is
  // preserved. Pass-through below the ceiling.
  static void saturate(float vL, float vR, float vWheelMax,
                       float steerHeadroom, float& vL_out,
                       float& vR_out);  // [mm/s]

  // The pre-reorganization msg::BodyTwist3 array overloads are gone -- see
  // Kinematics::Twist's own comment (kinematics.h) for what was wrong with
  // them and why nothing noticed. The Model::inverse()/forward() overrides
  // below are the array form now, on the float twist.

 private:
  float trackWidth_;  // [mm]
};

}  // namespace Kinematics
