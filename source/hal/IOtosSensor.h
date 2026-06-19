#pragma once
// 039-001: IOtosSensor is now an alias shim over the capability-typed interface.
// The canonical odometry interface lives at source/io/capability/IOdometer.h,
// and the SI value types live in source/io/capability/Pose2D.h. This shim keeps
// every existing IOtosSensor consumer (OtosSensor, MockOtosSensor,
// BenchOtosSensor, Odometry, Robot) compiling unchanged during the Phase A
// transition; it is deleted in Phase F.
//
// The old struct names map onto the new value types so existing declarations
// (OtosPose p; OtosVelocity v; OtosAccel a;) and the IOtosSensor read
// signatures continue to resolve.
#include "io/capability/IOdometer.h"
using IOtosSensor  = IOdometer;
using OtosPose     = Pose2D;
using OtosVelocity = BodyTwist;
using OtosAccel    = BodyAccel;
