#pragma once
/**
 * IKinematics.h — compile-time kinematics namespace alias (046-002).
 *
 * Differential-only build. The alias and kWheelCount are unconditional.
 *
 * Usage:
 *   #include "IKinematics.h"
 *   // Then call Kinematics::inverse(...), Kinematics::forward(...), etc.
 *   // kWheelCount gives the number of driven wheels.
 *
 * The alias ensures the control stack (BodyVelocityController, ticket 046-003)
 * can call the correct kinematics without an #ifdef at every call site.
 *
 * To build for a mecanum robot (see also source/main.cpp):
 *   1. Replace the include + alias + kWheelCount below with:
 *        #include "MecanumKinematics.h"
 *        namespace Kinematics = MecanumKinematics;
 *        constexpr int kWheelCount = 4;
 *   2. In source/main.cpp, replace NezhaHAL with MecanumHAL.
 * (git history preserves the full mecanum integration prior to sprint 048.)
 */
#include "BodyKinematics.h"
namespace Kinematics = BodyKinematics;
constexpr int kWheelCount = 2;
