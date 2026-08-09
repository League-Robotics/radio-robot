// odometry.cpp -- Motion::Odometry implementation. See odometry.h's file
// header for the module's boundary and rationale. applyOtosSample() moved
// out to src/firm/app/otos_sample.cpp (122-002) -- see that file.
#include "motion/odometry.h"

#include <cmath>

#include "kinematics/differential_kinematics.h"

namespace Motion {

Odometry::Odometry(float trackWidth, float initialLeftPosition, float initialRightPosition)
    : trackWidth_(trackWidth),
      lastLeft_(initialLeftPosition),
      lastRight_(initialRightPosition) {}

void Odometry::integrate(float leftPosition, float rightPosition, uint8_t leftEpoch,
                          uint8_t rightEpoch) {
  // 131-004: a per-wheel epoch change means THIS wheel's raw position just
  // jumped by a software rebaseline (Hal::Motor::rebaseline(), called
  // from App::RobotLoop::publishWheel()) -- re-anchor that wheel's own
  // baseline to the incoming (already-rebaselined) position instead of
  // differencing across the jump, crediting zero delta for it this call.
  // The other wheel, if its epoch is unchanged, diffs normally -- the two
  // sides are independent (each publishWheel() call rebaselines at most one
  // wheel at a time, but nothing here assumes that).
  const bool leftRebaselined = (leftEpoch != lastLeftEpoch_);
  const bool rightRebaselined = (rightEpoch != lastRightEpoch_);

  float deltaLeft = leftRebaselined ? 0.0f : (leftPosition - lastLeft_);
  float deltaRight = rightRebaselined ? 0.0f : (rightPosition - lastRight_);
  lastLeft_ = leftPosition;
  lastRight_ = rightPosition;
  lastLeftEpoch_ = leftEpoch;
  lastRightEpoch_ = rightEpoch;

  float distance = 0.0f;     // [mm] this cycle's body-frame forward travel
  float headingDelta = 0.0f; // [rad] this cycle's heading change
  Kinematics::DifferentialKinematics::forward(deltaLeft, deltaRight, trackWidth_, distance, headingDelta);

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
