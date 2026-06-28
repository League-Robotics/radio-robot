#pragma once
#include "io/capability/Pose2D.h"  // Pose2D, BodyTwist3
#include "types/ValueSet.h"        // ValueSet

// ---------------------------------------------------------------------------
// PoseEstimate — one source's pose + twist + freshness envelope (047-001).
//
// Used uniformly across differential and mecanum builds: vy is always present
// (written as 0.0f on differential builds) so dump/telemetry code is #ifdef-free.
// ---------------------------------------------------------------------------
struct PoseEstimate {
    Pose2D     pose  = {0.0f, 0.0f, 0.0f};   // x mm, y mm, h rad
    BodyTwist3 twist = {0.0f, 0.0f, 0.0f};   // vx mm/s, vy mm/s, omega rad/s
    ValueSet   stamp = {};                    // lagMs / lastUpdMs / valid
};
