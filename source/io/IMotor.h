#pragma once
// 039-001: IMotor is now an alias shim over the capability-typed interface.
// The canonical drive-wheel interface lives at source/io/capability/IVelocityMotor.h.
// This shim keeps every existing IMotor consumer (Motor, MockMotor,
// MotorController, Robot, Hardware) compiling unchanged during the Phase A
// transition; it is deleted in Phase F. The path-qualified include resolves in
// both the host build (source/ on the path) and the firmware build (source/
// added to INCLUDE_DIRS in CMakeLists.txt for 039-001).
#include "io/capability/IVelocityMotor.h"
using IMotor = IVelocityMotor;
