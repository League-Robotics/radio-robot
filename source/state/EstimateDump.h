#pragma once
#include <stdint.h>
#include "state/ActualState.h"
#include "hal/capability/Pose2D.h"  // Pose2D, BodyTwist3

// ---------------------------------------------------------------------------
// EstimateDump — snapshot of one pose estimate for diagnostic output (047-001).
//
// dumpEstimates() fills an out[3] array from an ActualState:
//   out[0] = encoder dead-reckoned estimate
//   out[1] = raw optical (OTOS) estimate
//   out[2] = EKF fused estimate
//
// ageMs = now_ms - stamp.lastUpdMs (clamped to UINT32_MAX when !stamp.valid).
// vy is always present (0 on differential builds) — no #ifdef needed in callers.
//
// Telemetry format (build-agnostic):
//   EST enc   x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
//   EST otos  x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
//   EST fuse  x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
// ---------------------------------------------------------------------------
struct EstimateDump {
    const char* source;   // "enc", "otos", "fuse"
    Pose2D      pose;     // x mm, y mm, h rad
    BodyTwist3  twist;    // vx mm/s, vy mm/s, omega rad/s
    uint32_t    ageMs;    // now_ms - stamp.lastUpdMs; UINT32_MAX if !valid
    bool        valid;
};

inline void dumpEstimates(const ActualState& a, uint32_t now_ms,
                           EstimateDump out[3])
{
    // Helper lambda-style: fill one slot from a PoseEstimate.
    auto fill = [now_ms](EstimateDump& d, const char* src,
                         const PoseEstimate& pe) {
        d.source = src;
        d.pose   = pe.pose;
        d.twist  = pe.twist;
        d.valid  = pe.stamp.valid;
        d.ageMs  = pe.stamp.valid
                   ? (now_ms - pe.stamp.lastUpdMs)
                   : UINT32_MAX;
    };

    fill(out[0], "enc",  a.encoder);
    fill(out[1], "otos", a.optical);
    fill(out[2], "fuse", a.fused);
}
