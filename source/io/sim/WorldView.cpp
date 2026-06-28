#include "WorldView.h"
#include <cmath>

// Heading wrap to [-pi, pi].  Uses atan2f(sinf, cosf) — the numerically robust,
// branch-free convention used by StopCondition::wrap_angle / Odometry::wrapPi.
static inline float wrapPi(float x)
{
    return atan2f(sinf(x), cosf(x));
}

// Euclidean distance (mm) between the plant-truth pose and the firmware's fused
// pose estimate (actual.fused.pose.x/y — 047-002; written by Odometry::predict/
// correctEKF into the structured PoseEstimate).
float WorldView::estimationErrorXY() const
{
    const float dx = _plant.truePoseX() - _actual.fused.pose.x;
    const float dy = _plant.truePoseY() - _actual.fused.pose.y;
    return sqrtf(dx * dx + dy * dy);
}

// Heading error (rad), wrapped to [-pi, pi].
float WorldView::estimationErrorH() const
{
    return wrapPi(_plant.truePoseH() - _actual.fused.pose.h);
}
