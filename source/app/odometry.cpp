#include "app/odometry.h"

#include <cmath>

#include "kinematics/body_kinematics.h"

namespace App {

Odometry::Odometry(Devices::NezhaMotor& left, Devices::NezhaMotor& right, float trackWidth)
    : left_(left),
      right_(right),
      trackWidth_(trackWidth),
      lastLeft_(left.position()),
      lastRight_(right.position()) {}

void Odometry::integrate() {
  float posLeft = left_.position();
  float posRight = right_.position();
  float deltaLeft = posLeft - lastLeft_;
  float deltaRight = posRight - lastRight_;
  lastLeft_ = posLeft;
  lastRight_ = posRight;

  float distance = 0.0f;     // [mm] this cycle's body-frame forward travel
  float headingDelta = 0.0f; // [rad] this cycle's heading change
  BodyKinematics::forward(deltaLeft, deltaRight, trackWidth_, distance, headingDelta);

  // Midpoint-arc integration: use the heading halfway through this cycle's
  // turn (not the heading at the START of the cycle) so a simultaneous
  // forward+turn motion doesn't bias x_/y_ toward the pre-turn heading --
  // the standard differential-drive dead-reckoning update.
  float midTheta = theta_ + headingDelta * 0.5f;
  x_ += distance * cosf(midTheta);
  y_ += distance * sinf(midTheta);
  theta_ += headingDelta;
}

void applyOtosSample(Devices::Otos& otos, uint64_t now, Telemetry::Frame& frame) {
  otos.tick(now);
  frame.hasOtos = otos.present();
  frame.otosConnected = otos.connected();
  if (otos.present()) {
    Devices::PoseReading reading = otos.pose();
    frame.otos = {reading.x, reading.y, reading.heading};
  }
}

}  // namespace App
