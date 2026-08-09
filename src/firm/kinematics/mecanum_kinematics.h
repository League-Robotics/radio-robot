// mecanum_kinematics.h -- Kinematics::MecanumKinematics: the four-wheel
// holonomic twist<->wheel-speed map for a 45-degree-roller mecanum chassis.
//
// Wheel order for the array forms is FL, FR, BL, BR:
//   wheels[0] front-left   wheels[1] front-right
//   wheels[2] back-left    wheels[3] back-right
// This is an ARRAY-INDEX convention, not a wiring one. A robot's per-motor
// `fwd_sign` (data/robots/*.json) still corrects mirror-mounted wheels
// below this layer -- do not fold a mounting sign into these equations.
//
// NOT WIRED IN YET: nothing constructs this class. `Core::DifferentialDrive` is still
// differential-specific end to end (its constructor takes exactly two
// Hal::Motor& and a scalar track width), so there is no four-wheel
// drivetrain for a composition root to hand this to. It exists because an
// interface with exactly one implementation is indirection rather than
// abstraction -- this is what makes Kinematics::Model a real seam, and it
// is exercised by the mecanum scenarios in
// src/tests/sim/unit/kinematics_harness.cpp. Togov is a real mecanum
// chassis in this fleet (data/robots/togov.json, drivetrain_type
// "mecanum"), so this is the drivetrain that has to work next, not a
// hypothetical one.
//
// All functions are pure: no I2C, no global state, no heap allocation.
#pragma once

#include "kinematics/kinematics.h"

namespace Kinematics {

class MecanumKinematics : public Model {
 public:
  // trackWidth  -- [mm] left-to-right wheel centre distance.
  // wheelBase   -- [mm] front-to-back wheel centre distance.
  MecanumKinematics(float trackWidth, float wheelBase)
      : trackWidth_(trackWidth), wheelBase_(wheelBase) {}

  int wheelCount() const override { return 4; }

  void inverse(const Twist& twist, float wheels[]) const override;
  void forward(const float wheels[], Twist& twist) const override;

  float trackWidth() const { return trackWidth_; }  // [mm]
  float wheelBase() const { return wheelBase_; }    // [mm]

  // ---------------------------------------------------------------------
  // The math, as statics taking the geometry explicitly -- mirroring
  // DifferentialKinematics' own static/instance split.
  //
  // With lx = wheelBase/2, ly = trackWidth/2, and k = lx + ly:
  //   inverse:  vFL = v_x - v_y - k*omega    vFR = v_x + v_y + k*omega
  //             vBL = v_x + v_y - k*omega    vBR = v_x - v_y + k*omega
  //   forward:  v_x   = ( vFL + vFR + vBL + vBR) / 4
  //             v_y   = (-vFL + vFR + vBL - vBR) / 4
  //             omega = (-vFL + vFR - vBL + vBR) / (4k)
  // ---------------------------------------------------------------------
  static void inverse(float v_x, float v_y, float omega, float trackWidth,
                      float wheelBase,
                      float wheels[4]);  // [mm/s] [mm/s] [rad/s] [mm] [mm]

  static void forward(const float wheels[4], float trackWidth, float wheelBase,
                      float& v_x_out, float& v_y_out, float& omega_out);

 private:
  float trackWidth_;  // [mm]
  float wheelBase_;   // [mm]
};

}  // namespace Kinematics
