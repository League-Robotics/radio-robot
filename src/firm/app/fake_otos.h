// fake_otos.h -- App::FakeOtos: a bench implementation of the Hal::Otos
// interface that reports the robot's dead-reckoned Odometry pose AS IF it
// were a real OTOS chip, without touching the I2C bus. Selected at the
// main.cpp composition root under `#ifdef FAKE_OTOS` (the ONE place that
// macro appears); the loop holds a plain `Hal::Otos&` and neither knows
// nor cares which implementation it drives.
//
// The synthesis is pulled by this class's own tick(), not pushed into the
// real leaf. Because a fake needs App:: context (the Odometry pose and
// the wheel Motors' velocities), it lives in app/ rather than devices/ --
// the devices/ isolation invariant forbids that leaf layer from depending
// on App::Odometry, which is exactly why the interface (Hal::Otos) is
// the seam and this concrete fake sits above it.
//
// tick() synthesizes:
//   - pose x/y/heading   <- the just-integrated Odometry pose (odom_)
//   - body twist v_x/omega <- Kinematics::DifferentialKinematics::forward(vL, vR, trackWidth),
//     the SAME fusion App::RobotLoop::updateTlm() uses for frame_.twist;
//     v_y is 0 (a differential drive has no lateral body velocity).
// present()/connected() are always true (a fake is always "there"), and
// poseFresh() is true after the first tick() -- mirroring the freshness the
// old feedSyntheticSample() published so applyOtosSample()'s
// `present() && poseFresh()` gate behaves identically.
#pragma once

#include <cstdint>

#include "hal/motor.h"
#include "hardware/generic/real_otos.h"
#include "motion/odometry.h"

namespace App {

class FakeOtos : public Hal::Otos {
 public:
  // odom -- the pose source (read, never mutated). left/right -- the SAME
  // two Motor leaves Odometry integrates, used only for their velocity() to
  // fuse the body twist. trackWidth -- [mm], Kinematics::DifferentialKinematics::forward()'s `b`.
  FakeOtos(const Motion::Odometry& odom, Hal::Motor& left, Hal::Motor& right,
           float trackWidth);  // [mm]

  // No real chip: begin()/init()/calibration setters are all no-ops (a fake
  // has nothing to probe or configure). getOffset() reports a zero offset.
  void begin() override {}
  void init() override {}
  void setLinearScalar(float) override {}
  void setAngularScalar(float) override {}
  void setOffset(float, float, float) override {}                   // [mm] [mm] [rad]
  void getOffset(float& x, float& y, float& heading) override;      // [mm] [mm] [rad]

  // Refresh the synthetic reading from this cycle's Odometry pose + fused
  // wheel twist. No bus traffic; nowUs also becomes sampleTime()'s return —
  // a fake synthesizes a fresh reading every tick() call, so its sample
  // time is always exactly the calling tick's own nowUs (no rate limit to
  // gate, unlike RealOtos).
  void tick(uint64_t nowUs) override;  // [us]

  Hal::PoseReading pose() const override { return cachedPose_; }
  bool poseFresh() const override { return poseFresh_; }
  bool connected() const override { return true; }
  bool present() const override { return true; }
  uint64_t sampleTime() const override { return sampleTimeUs_; }  // [us]

 private:
  const Motion::Odometry& odom_;
  Hal::Motor& left_;
  Hal::Motor& right_;
  float trackWidth_;  // [mm]

  Hal::PoseReading cachedPose_{};
  bool poseFresh_ = false;
  uint64_t sampleTimeUs_ = 0;  // [us] sampleTime()'s backing field -- the last tick()'s own nowUs
};

}  // namespace App
