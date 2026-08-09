// otos.h -- Hal::Otos: the optical tracking odometry sensor interface every
// consumer takes. Pure abstract; no bus, no registers, no chip knowledge.
//
// Implementations:
//   - Hardware::RealOtos (hardware/generic/real_otos.h) -- the SparkFun
//     OTOS chip over I2C. A commercial, publicly-documented part, hence
//     hardware/generic/ rather than a named-family directory.
//   - App::FakeOtos (app/fake_otos.h) -- the FAKE_OTOS build variant, which
//     reports the dead-reckoned Odometry pose as if it were the chip. It is
//     NOT a sim-only construct: it is how a real micro:bit robot built
//     WITHOUT a physical OTOS runs.
//   - TestSim's OtosPlant path, via SimPlant answering RealOtos's own bus
//     transactions (the sim substitutes the PLATFORM, not this interface).
//
// Extracted from the former devices/otos.h, which held this interface and
// RealOtos in one file. Splitting them is what lets a robot with no OTOS
// (or a different absolute-pose sensor entirely) link without dragging the
// SparkFun register map in behind it.
#pragma once

#include <cstdint>

#include "hal/device_types.h"

namespace Hal {

class Otos {
 public:
  // Inline `= default`, matching Hal::Motor / Platform::I2CBus / Platform::
  // Clock. It was an out-of-line vtable anchor in otos.cpp while the
  // interface and RealOtos shared one file; keeping that would have made
  // the HAL layer's own symbol live in a hardware translation unit, and
  // giving it a hal/otos.cpp of its own would have added a new file to the
  // host CMake source list and every pytest compile harness for one
  // defaulted destructor. The vtable is emitted weakly per TU and merged
  // by the linker instead -- the same deal every other interface here takes.
  virtual ~Otos() = default;

  virtual void begin() = 0;
  virtual void tick(uint64_t nowUs) = 0;  // [us]
  virtual PoseReading pose() const = 0;
  virtual bool poseFresh() const = 0;
  virtual bool connected() const = 0;
  virtual bool present() const = 0;

  virtual uint64_t sampleTime() const = 0;  // [us]

  // Raw product-id byte from the last probe: 0x5F is the chip, 0x00/0xFF mean
  // no ACK. Distinguishes "wrong device" from "nothing there" at diagnosis.
  virtual uint8_t lastProbeId() const { return 0; }

  // Seed the chip's world pose (robot CENTRE, lever arm applied by the impl).
  // Default no-op: a fake synthesizes its pose and has nothing to seed.
  virtual void setPose(float x, float y, float heading) {}  // [mm] [mm] [rad]

  virtual void setLinearScalar(float scalar) = 0;
  virtual void setAngularScalar(float scalar) = 0;
  virtual void setOffset(float x, float y, float heading) = 0;     // [mm] [mm] [rad]
  virtual void getOffset(float& x, float& y, float& heading) = 0;  // [mm] [mm] [rad]
  virtual void init() = 0;

  // Re-run the gyro bias calibration ONLY -- tracking and pose survive,
  // unlike init(), which resets both. The chip averages `samples` gyro
  // readings (~2.4ms each; 255 = ~612ms) into its bias estimate, so the
  // robot MUST be still for the duration -- the caller (RobotLoop's
  // CALIBRATE handler) enforces that with an encoder-stillness check.
  // Exists because the boot-time calibration silently poisons heading
  // when the robot boots in someone's hands (measured 2026-08-08:
  // +1.44 deg/s standstill drift; still recalibration -> -0.006 deg/s).
  // Default no-op: fakes synthesize their pose and have no gyro bias.
  virtual void calibrateImu(uint8_t samples) {}
};

}  // namespace Hal
