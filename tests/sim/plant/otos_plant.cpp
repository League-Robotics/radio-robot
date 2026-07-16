#include "otos_plant.h"

#include <cmath>

#include "kinematics/body_kinematics.h"

namespace TestSim {

OtosPlant::OtosPlant(float trackWidth) : trackWidth_(trackWidth) {}

void OtosPlant::step(float leftPosition, float rightPosition) {
  float deltaLeft = leftPosition - lastLeft_;
  float deltaRight = rightPosition - lastRight_;
  lastLeft_ = leftPosition;
  lastRight_ = rightPosition;

  // The SAME BodyKinematics::forward() call + midpoint-arc accumulation
  // App::Odometry::integrate() performs (source/app/odometry.cpp) -- see
  // this file's header for why that duplication is deliberate, not a
  // second heading formula.
  float distance = 0.0f;       // [mm] this cycle's body-frame forward travel
  float headingDelta = 0.0f;   // [rad] this cycle's heading change
  BodyKinematics::forward(deltaLeft, deltaRight, trackWidth_, distance, headingDelta);

  float midHeading = heading_ + headingDelta * 0.5f;
  x_ += distance * std::cos(midHeading);
  y_ += distance * std::sin(midHeading);
  heading_ += headingDelta;
}

void OtosPlant::setDrift(float xDrift, float yDrift, float headingDrift) {
  driftX_ = xDrift;
  driftY_ = yDrift;
  driftHeading_ = headingDrift;
}

}  // namespace TestSim
