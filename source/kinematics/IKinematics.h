#pragma once
/**
 * IKinematics.h — compile-time kinematics namespace alias (046-002).
 *
 * Selects the active kinematics implementation at compile time based on
 * the ROBOT_DRIVETRAIN_MECANUM preprocessor define (set by CMake from the
 * robot JSON drivetrain_type field).
 *
 * Usage:
 *   #include "IKinematics.h"
 *   // Then call Kinematics::inverse(...), Kinematics::forward(...), etc.
 *   // kWheelCount gives the number of driven wheels.
 *
 * The alias ensures the control stack (BodyVelocityController, ticket 046-003)
 * can call the correct kinematics without an #ifdef at every call site.
 */
#ifdef ROBOT_DRIVETRAIN_MECANUM
  #include "MecanumKinematics.h"
  namespace Kinematics = MecanumKinematics;
  constexpr int kWheelCount = 4;
#else
  #include "BodyKinematics.h"
  namespace Kinematics = BodyKinematics;
  constexpr int kWheelCount = 2;
#endif
