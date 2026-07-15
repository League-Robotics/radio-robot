#include "otos_plant.h"

#include <cmath>

#include "kinematics/body_kinematics.h"

namespace TestSim {

namespace {
// Duplicated from source/devices/otos.h's private constants (this
// codebase's established per-file convention -- devices_otos_harness.cpp's
// own kPosMmPerLsb/kHdgRadPerLsb precedent).
constexpr float kPosMmPerLsb = 0.305f;                             // [mm/LSB]
constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);  // [rad/LSB]
}  // namespace

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

void OtosPlant::scriptPoseResponse(Devices::I2CBus& bus, uint16_t wireAddr) const {
  int16_t rx = static_cast<int16_t>(std::lround(x_ / kPosMmPerLsb));
  int16_t ry = static_cast<int16_t>(std::lround(y_ / kPosMmPerLsb));
  int16_t rh = static_cast<int16_t>(std::lround(heading_ / kHdgRadPerLsb));

  uint8_t raw[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  raw[0] = static_cast<uint8_t>(rx & 0xFF);
  raw[1] = static_cast<uint8_t>((rx >> 8) & 0xFF);
  raw[2] = static_cast<uint8_t>(ry & 0xFF);
  raw[3] = static_cast<uint8_t>((ry >> 8) & 0xFF);
  raw[4] = static_cast<uint8_t>(rh & 0xFF);
  raw[5] = static_cast<uint8_t>((rh >> 8) & 0xFF);
  // raw[6..11] (velocity registers) left zero -- no scenario in this ticket
  // asserts on OTOS's twist.

  bus.scriptWrite(wireAddr, /*status=*/0);
  bus.scriptRead(wireAddr, raw, 12, /*status=*/0);
}

}  // namespace TestSim
