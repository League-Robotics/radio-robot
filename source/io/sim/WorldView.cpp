#include "WorldView.h"
#include "types/Inputs.h"   // HardwareState (poseX/poseY/poseHrad)
#include <cmath>

// Heading wrap to [-pi, pi].  Uses atan2f(sinf, cosf) — the numerically robust,
// branch-free convention used by StopCondition::wrap_angle / Odometry::wrapPi.
static inline float wrapPi(float x)
{
    return atan2f(sinf(x), cosf(x));
}

// Euclidean distance (mm) between the plant-truth pose and the firmware's fused /
// dead-reckoned pose estimate (state.inputs.poseX/poseY, written by Odometry).
float WorldView::estimationErrorXY() const
{
    const float dx = _plant.truePoseX() - _inputs.poseX;
    const float dy = _plant.truePoseY() - _inputs.poseY;
    return sqrtf(dx * dx + dy * dy);
}

// Heading error (rad), wrapped to [-pi, pi].
float WorldView::estimationErrorH() const
{
    return wrapPi(_plant.truePoseH() - _inputs.poseHrad);
}
