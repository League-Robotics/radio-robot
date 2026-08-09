// estimation.h -- the planner's sense-side modules (motion-planner sketch
// §4): WheelChannel (per-wheel EMA velocity filter + fresh-sample gating +
// ZOH predict-to-now) and PoseTracker (arc-exact differential-drive
// odometry over predicted wheel positions, plus the v1 complementary OTOS
// heading blend). Plain hand-fed values in, plain values out -- no devices,
// no clock, no RobotState dependency (the Planner is the one place the
// blackboard is read/written).
//
// 131-004 (position-rebaseline-destroys-the-pose.md): PoseTracker::
// integrate() now also takes each wheel's positionEpoch, mirroring
// Motion::Odometry::integrate() (src/firm/motion/odometry.h) -- the identical
// re-anchor-on-epoch-change contract, applied to the planner's own
// x_/y_/heading_ ledger (which otherwise fed the SAME raw, rebaseline-
// discontinuous wheel positions into an identical bare-delta computation).
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
  //
  // leftEpoch/rightEpoch (131-004): each wheel's current
  // Types::RobotState::Wheel::positionEpoch -- see Motion::Odometry::
  // integrate()'s own doc comment (odometry.h) for the full contract. A
  // per-wheel epoch change re-anchors THAT wheel's lastLeft_/lastRight_ to
  // the incoming position, crediting zero delta for it this call, tracked
  // independently per side. A caller with no rebaseline concept passes a
  // stable (0, 0).
  void integrate(float leftPosition, float rightPosition, uint8_t leftEpoch,
                 uint8_t rightEpoch);  // [mm] [mm]

  // v1 complementary blend: heading += weight * (otosHeading - heading),
  // shortest-way wrapped.
  void blendHeading(float otosHeading, float weight);  // [rad]

  // Hand the tracker this cycle's OTOS heading. While the chip is connected,
  // heading() reports the OTOS's own heading instead of the wheel-integrated
  // one -- which is the whole point: wheel heading is scrub-limited and is
  // what makes open-loop turns inaccurate.
  //
  // `fresh` and `connected` are DIFFERENT questions and conflating them
  // breaks this badly. The chip is read every 20ms, so `fresh` is false on
  // most control cycles; treating that as "no OTOS" makes the source flip
  // every cycle, re-seeding constantly, so no rotation ever accumulates.
  // Only `connected` going false is a real loss.
  //
  // Accumulates DELTAS, because the chip reports wrapped [-pi, pi] while
  // heading() is unwrapped and every caller depends on that continuity.
  // Seeds from the current wheel heading on the first sample and hands the
  // accumulated value back to the wheel path on loss, so neither transition
  // is a discontinuity.
  void applyOtosHeading(float otosHeading, bool fresh, bool connected);  // [rad]

  bool otosDriven() const { return otosActive_; }

  float x() const { return x_; }              // [mm]
  float y() const { return y_; }              // [mm]
  float heading() const { return otosActive_ ? otosHeading_ : heading_; }  // [rad] unwrapped
  float pathLength() const { return pathLength_; }  // [mm] unsigned

  // Does NOT touch lastLeftEpoch_/lastRightEpoch_ (131-004) -- see
  // Motion::Odometry::reset()'s own doc comment (odometry.h) for why: a
  // pose teleport is orthogonal to the hardware rebaseline epoch, and a
  // coincident mismatch costs at most one harmless zero-delta re-anchor to
  // the position already just reset to.
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

  bool otosActive_ = false;
  float otosHeading_ = 0.0f;      // [rad] unwrapped, OTOS-driven
  float lastOtosSample_ = 0.0f;   // [rad] wrapped, previous chip reading

  // 131-004: mirrors Motion::Odometry's own lastLeftEpoch_/lastRightEpoch_
  // (odometry.h) -- initialized to 0 to match a fresh
  // Types::RobotState::Wheel::positionEpoch's own starting value.
  uint8_t lastLeftEpoch_ = 0;
  uint8_t lastRightEpoch_ = 0;
};

}  // namespace Motion
