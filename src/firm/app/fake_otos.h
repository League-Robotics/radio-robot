// fake_otos.h -- App::FakeOtos: a bench implementation of the Devices::Otos
// interface that reports the robot's dead-reckoned Odometry pose AS IF it
// were a real OTOS chip, without touching the I2C bus. Selected at the
// main.cpp composition root under `#ifdef FAKE_OTOS` (the ONE place that
// macro appears); the loop holds a plain `Devices::Otos&` and neither knows
// nor cares which implementation it drives.
//
// This replaces the old call-site `#ifdef FAKE_OTOS` branch +
// Devices::Otos::feedSyntheticSample() seam (120-002): the synthesis that
// used to be pushed into the real leaf every cycle now lives here, pulled by
// this class's own tick(). Because a fake needs App:: context (the
// Odometry pose and the wheel Motors' velocities), it lives in app/ rather
// than devices/ -- the devices/ isolation invariant forbids that leaf layer
// from depending on App::Odometry, which is exactly why the interface
// (Devices::Otos) is the seam and this concrete fake sits above it.
//
// tick() synthesizes:
//   - pose x/y/heading   <- the just-integrated Odometry pose (odom_)
//   - body twist v_x/omega <- BodyKinematics::forward(vL, vR, trackWidth),
//     the SAME fusion App::RobotLoop::updateTlm() uses for frame_.twist;
//     v_y is 0 (a differential drive has no lateral body velocity).
// present()/connected() are always true (a fake is always "there"), and
// poseFresh() is true after the first tick() -- mirroring the freshness the
// old feedSyntheticSample() published so applyOtosSample()'s
// `present() && poseFresh()` gate behaves identically.
#pragma once

#include <cstdint>

#include "app/odometry.h"
#include "devices/motor.h"
#include "devices/otos.h"

namespace App {

class FakeOtos : public Devices::Otos {
 public:
  // odom -- the pose source (read, never mutated). left/right -- the SAME
  // two Motor leaves Odometry integrates, used only for their velocity() to
  // fuse the body twist. trackWidth -- [mm], BodyKinematics::forward()'s `b`.
  FakeOtos(const Odometry& odom, Devices::Motor& left, Devices::Motor& right,
           float trackWidth);  // [mm]

  // No real chip: begin()/init()/calibration setters are all no-ops (a fake
  // has nothing to probe or configure). getOffset() reports a zero offset.
  void begin() override {}
  void init() override {}
  void setLinearScalar(float scalar) override {}
  void setAngularScalar(float scalar) override {}
  void setOffset(float x, float y, float heading) override {}       // [mm] [mm] [rad]
  void getOffset(float& x, float& y, float& heading) override;      // [mm] [mm] [rad]

  // Refresh the synthetic reading from this cycle's Odometry pose + fused
  // wheel twist. No bus traffic; nowUs is accepted for interface parity but
  // unused (a fake has no rate limit to gate).
  void tick(uint64_t nowUs) override;  // [us]

  Devices::PoseReading pose() const override { return cachedPose_; }
  bool poseFresh() const override { return poseFresh_; }
  bool connected() const override { return true; }
  bool present() const override { return true; }

 private:
  const Odometry& odom_;
  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  Devices::PoseReading cachedPose_{};
  bool poseFresh_ = false;
};

}  // namespace App
