#pragma once
// 039-001: IServo is now an alias shim over the capability-typed interface.
// The canonical position/angle-control interface lives at
// source/io/capability/IPositionMotor.h. This shim keeps every existing IServo
// consumer (Servo, MockServo, ServoController, Hardware::gripper) compiling
// unchanged during the Phase A transition; it is deleted in Phase F.
#include "io/capability/IPositionMotor.h"
using IServo = IPositionMotor;
