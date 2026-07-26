// estimation.h -- the planner's sense-side modules (motion-planner sketch
// §4): WheelChannel (per-wheel EMA velocity filter + fresh-sample gating +
// ZOH predict-to-now) and PoseTracker (arc-exact differential-drive
// odometry over predicted wheel positions, plus the v1 complementary OTOS
// heading blend). Plain hand-fed values in, plain values out -- no devices,
// no clock, no RobotState dependency (the Planner is the one place the
// blackboard is read/written).
#pragma once

#include <cstdint>

namespace Motion {

// One wheel's measurement channel. Raw encoder velocities are very noisy
// (stakeholder 2026-07-25); the EMA advances ONLY when sampleTime changes
// (the encoder refreshes far slower than the loop, so most cycles re-see
// the same sample -- re-feeding it would silently re-weight old data).
// Filtered velocity feeds extrapolation and the profiler's current-speed
// input; traveled distance is always anchored to measured POSITIONS, so
// filter lag can never accumulate distance error.
class WheelChannel {
 public:
  // weight: EMA weight on the fresh sample's velocity; 1 = unfiltered.
  void configure(float weight) { weight_ = weight; }

  // Feed the state's current sensed triple; no-op unless sampleTime is new.
  void ingest(float position, float velocity, uint32_t sampleTime);  // [mm] [mm/s] [ms]

  // ZOH predict: anchor position + filtered velocity * (t - anchor time).
  float positionAt(uint32_t t) const;  // [ms] -> [mm]

  float velocity() const { return velocityFiltered_; }  // [mm/s] signed
  uint32_t basisTime() const { return anchorTime_; }    // [ms]
  float basisPosition() const { return anchorPosition_; }  // [mm]
  bool valid() const { return valid_; }

 private:
  float weight_ = 1.0f;
  float anchorPosition_ = 0.0f;    // [mm]
  uint32_t anchorTime_ = 0;        // [ms]
  float velocityFiltered_ = 0.0f;  // [mm/s]
  uint32_t lastSampleTime_ = 0;    // [ms]
  bool valid_ = false;
};

// Differential-drive pose from cumulative wheel positions, integrated
// arc-exactly (constant-curvature segment per step). pathLength() is the
// unsigned traveled path; heading is the wheel-difference integral.
class PoseTracker {
 public:
  void configure(float trackWidth) { trackWidth_ = trackWidth; }  // [mm]

  // Fold the delta since the previous call into the pose. First call seeds.
  void integrate(float leftPosition, float rightPosition);  // [mm]

  // v1 complementary blend: heading += weight * (otosHeading - heading),
  // shortest-way wrapped.
  void blendHeading(float otosHeading, float weight);  // [rad]

  float x() const { return x_; }              // [mm]
  float y() const { return y_; }              // [mm]
  float heading() const { return heading_; }  // [rad] unwrapped
  float pathLength() const { return pathLength_; }  // [mm] unsigned

  void reset(float x, float y, float heading);  // [mm] [mm] [rad]

 private:
  float trackWidth_ = 100.0f;  // [mm]
  float x_ = 0.0f;             // [mm]
  float y_ = 0.0f;             // [mm]
  float heading_ = 0.0f;       // [rad]
  float pathLength_ = 0.0f;    // [mm]
  float lastLeft_ = 0.0f;      // [mm]
  float lastRight_ = 0.0f;     // [mm]
  bool seeded_ = false;
};

}  // namespace Motion
