#pragma once
#include <stdint.h>

// SI value types for the odometry capability (039-001).
//
// These replace the old OtosPose / OtosVelocity / OtosAccel structs that lived
// in IOtosSensor.h.  The old names remain as `using` aliases (see the
// IOtosSensor.h shim) so existing callers compile unchanged during the Phase A
// transition; the rename to the capability-typed names is completed in T4.
//
// Field semantics are byte-identical to the structs they replace:
//   Pose2D    : x, y in mm; h in radians (robot/world frame per caller).
//   BodyTwist : v_mmps forward body speed (mm/s); omega_rads yaw rate (rad/s).
//   BodyAccel : ax/ay body-frame linear acceleration (mm/s^2).
struct Pose2D    { float x, y, h; };              // mm, mm, rad
struct BodyTwist { float v_mmps, omega_rads; };   // mm/s, rad/s
struct BodyAccel { float ax_mmps2, ay_mmps2; };   // mm/s^2
