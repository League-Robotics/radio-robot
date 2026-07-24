// odometry.cpp -- Motion::Odometry implementation. See odometry.h's file
// header for the module's boundary and rationale. applyOtosSample() moved
// out to src/firm/app/otos_sample.cpp (122-002) -- see that file.
#include "motion/odometry.h"

#include <cmath>

#include "motion/body_kinematics.h"

namespace Motion {

Odometry::Odometry(float trackWidth, float initialLeftPosition, float initialRightPosition)
    : trackWidth_(trackWidth),
      lastLeft_(initialLeftPosition),
      lastRight_(initialRightPosition) {}

void Odometry::integrate(float leftPosition, float rightPosition) {
  float deltaLeft = leftPosition - lastLeft_;
  float deltaRight = rightPosition - lastRight_;
  lastLeft_ = leftPosition;
  lastRight_ = rightPosition;

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

  pathLength_ += fabsf(distance);
}

void Odometry::reset(float x, float y, float theta, float leftPosition, float rightPosition) {
  x_ = x;
  y_ = y;
  theta_ = theta;
  // Re-anchor the delta baseline to the leaves' CURRENT positions (handed in
  // by the caller) so the very next integrate() computes a zero delta
  // (mirrors the constructor's own "first integrate() sees zero delta"
  // anchoring).
  lastLeft_ = leftPosition;
  lastRight_ = rightPosition;
}

}  // namespace Motion
