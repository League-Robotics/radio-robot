#include "app/fake_otos.h"

#include "kinematics/body_kinematics.h"

namespace App {

FakeOtos::FakeOtos(const Odometry& odom, Devices::Motor& left, Devices::Motor& right,
                    float trackWidth)
    : odom_(odom), left_(left), right_(right), trackWidth_(trackWidth)
{
}

void FakeOtos::getOffset(float& x, float& y, float& heading)
{
    x = 0.0f;
    y = 0.0f;
    heading = 0.0f;
}

void FakeOtos::tick(uint64_t /*nowUs*/)
{
    // Body twist from the wheel velocities -- the SAME BodyKinematics::forward()
    // fusion App::RobotLoop::updateTlm() uses to stage frame_.twist. v_y stays
    // 0: forward() reports only (v, omega) for a differential drive.
    float v_x = 0.0f;
    float omega = 0.0f;
    BodyKinematics::forward(left_.velocity(), right_.velocity(), trackWidth_, v_x, omega);

    cachedPose_.x = odom_.x();
    cachedPose_.y = odom_.y();
    cachedPose_.heading = odom_.theta();
    cachedPose_.v_x = v_x;
    cachedPose_.v_y = 0.0f;
    cachedPose_.omega = omega;
    poseFresh_ = true;
}

}  // namespace App
