#include "estimation.h"

#include <algorithm>
#include <cmath>

namespace Motion {

void WheelChannel::ingest(float position, float velocity,
                          uint32_t sampleTime) {
  if (valid_ && sampleTime == lastSampleTime_) return;  // stale re-see: no-op
  if (!valid_) {
    velocityFiltered_ = velocity;  // first fresh sample seeds the filter
    valid_ = true;
  } else {
    velocityFiltered_ += weight_ * (velocity - velocityFiltered_);
  }
  anchorPosition_ = position;
  anchorTime_ = sampleTime;
  lastSampleTime_ = sampleTime;
}

float WheelChannel::positionAt(uint32_t t) const {
  if (!valid_) return 0.0f;
  // Signed: a caller may legitimately ask for an instant slightly BEFORE
  // this anchor (the loop stamps cycleStart before the collects) -- treat
  // that as zero extrapolation, never as a wrapped ~50-day age.
  const float age = std::max(
      0.0f, static_cast<float>(static_cast<int32_t>(t - anchorTime_)) * 0.001f);  // [s]
  return anchorPosition_ + velocityFiltered_ * age;
}

void PoseTracker::integrate(float leftPosition, float rightPosition, uint8_t leftEpoch,
                             uint8_t rightEpoch) {
  if (!seeded_) {
    lastLeft_ = leftPosition;
    lastRight_ = rightPosition;
    lastLeftEpoch_ = leftEpoch;
    lastRightEpoch_ = rightEpoch;
    seeded_ = true;
    return;
  }
  // 131-004: see Motion::Odometry::integrate()'s own comment (odometry.cpp)
  // -- identical per-wheel re-anchor-on-epoch-change contract.
  const bool leftRebaselined = (leftEpoch != lastLeftEpoch_);
  const bool rightRebaselined = (rightEpoch != lastRightEpoch_);

  const float dLeft = leftRebaselined ? 0.0f : (leftPosition - lastLeft_);
  const float dRight = rightRebaselined ? 0.0f : (rightPosition - lastRight_);
  lastLeft_ = leftPosition;
  lastRight_ = rightPosition;
  lastLeftEpoch_ = leftEpoch;
  lastRightEpoch_ = rightEpoch;

  const float ds = 0.5f * (dLeft + dRight);
  const float dTheta = (dRight - dLeft) / trackWidth_;
  if (dTheta == 0.0f) {
    x_ += ds * std::cos(heading_);
    y_ += ds * std::sin(heading_);
  } else {
    // Constant-curvature arc: chord via the turn radius ds/dTheta.
    const float radius = ds / dTheta;
    x_ += radius * (std::sin(heading_ + dTheta) - std::sin(heading_));
    y_ += -radius * (std::cos(heading_ + dTheta) - std::cos(heading_));
  }
  heading_ += dTheta;
  pathLength_ += std::fabs(ds);
}

void PoseTracker::blendHeading(float otosHeading, float weight) {
  if (weight <= 0.0f) return;
  // Shortest-way residual so a wrap seam never drags the blend the long way.
  float residual = otosHeading - heading_;
  residual = std::remainder(residual, 2.0f * static_cast<float>(M_PI));
  heading_ += weight * residual;
}

void PoseTracker::reset(float x, float y, float heading) {
  x_ = x;
  y_ = y;
  heading_ = heading;
  // pathLength_ deliberately untouched (Odometry::reset()'s own precedent).
}

}  // namespace Motion
