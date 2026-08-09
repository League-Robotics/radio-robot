// otos.h -- Hardware::RealOtos: the SparkFun OTOS chip (I2C 0x17) behind the
// Hal::Otos interface (hal/otos.h). The interface itself lived in this file
// until the platform/hardware/hal reorganization split them; everything
// below this line is chip knowledge -- the register map, the LSB scales,
// the bus-clearance and read-period budgets, the lever-arm transform.
#pragma once

#include <cstdint>

#include "hal/device_config.h"
#include "hal/device_types.h"
#include "hal/otos.h"
#include "platform/i2c_bus.h"

namespace Hardware {

constexpr uint8_t kOtosDeviceAddr = 0x17;

int8_t scaleToRegister(float scale);

class RealOtos : public Hal::Otos {
 public:
  RealOtos(Platform::I2CBus& bus, const Hal::OtosConfig& config);

  void begin() override;

  Hal::PoseReading pose() const override;

  bool poseFresh() const override;

  bool connected() const override;

  bool present() const override;

  uint8_t lastProbeId() const override { return lastProbeId_; }

  uint64_t sampleTime() const override { return lastReadUs_; }  // [us]

  bool readDue(uint64_t nowUs) const;  // [us]

  void tick(uint64_t nowUs) override;  // [us]

  void setPose(float x, float y, float heading) override;  // [mm] [mm] [rad]

  void resetTracking();  // OR — reset Kalman tracking

  void setLinearScalar(float scalar) override;   // OL
  void setAngularScalar(float scalar) override;  // OA

  void setOffset(float x, float y, float heading) override;       // [mm] [mm] [rad]
  void getOffset(float& x, float& y, float& heading) override;    // [mm] [mm] [rad]

  void setSignalProcessConfig(uint8_t config);
  uint8_t signalProcessConfig();

  uint8_t imuCalibrationSamplesRemaining();

  void calibrateImu(uint8_t samples) override;

  void init() override;

 private:
  Platform::I2CBus& bus_;
  Hal::OtosConfig config_;

  bool initialized_ = false;

  bool connected_ = false;
  uint8_t lastProbeId_ = 0;   // last product-ID byte read by begin()

  Hal::PoseReading cachedPose_{};
  bool poseFresh_ = false;   // poseFresh()'s backing field

  uint64_t lastReadUs_ = 0;  // [us] time of the most recent REAL bus read
  bool hasRead_ = false;     // true once at least one real bus read has been attempted

  bool posePending_ = false;
  float pendingX_ = 0.0f;        // [mm]
  float pendingY_ = 0.0f;        // [mm]
  float pendingHeading_ = 0.0f;  // [rad]

  static constexpr uint8_t kRegProductId        = 0x00;
  static constexpr uint8_t kRegLinearScalar     = 0x04;
  static constexpr uint8_t kRegAngularScalar    = 0x05;
  static constexpr uint8_t kRegImuCalibration   = 0x06;
  static constexpr uint8_t kRegReset            = 0x07;
  static constexpr uint8_t kRegSignalProcessCfg = 0x0E;
  static constexpr uint8_t kRegOffsetXl         = 0x10;
  static constexpr uint8_t kRegPositionXl       = 0x20;
  static constexpr uint8_t kRegVelocityXl       = 0x26;

  static constexpr uint8_t kExpectedProductId = 0x5F;

  static constexpr uint8_t kImuCalibSamples = 255;

  static constexpr float kPosMmPerLsb = 0.305f;                             // [mm/LSB]
  static constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);  // [rad/LSB]

  static constexpr uint32_t kBusClearance = 4000;  // [us]

  static constexpr uint64_t kReadPeriod = 20000;  // [us]

  void applyPendingPose();

  static void sensorToCentre(float sensorX, float sensorY, float sensorHeading,
                              float offsetX, float offsetY,
                              float& centreXOut, float& centreYOut);
  static void centreToSensor(float centreX, float centreY, float centreHeading,
                              float offsetX, float offsetY,
                              float& sensorXOut, float& sensorYOut);

  void writeReg8(uint8_t reg, uint8_t val);
  uint8_t readReg8(uint8_t reg);
  bool readPositionVelocity(int16_t& x, int16_t& y, int16_t& h,
                             int16_t& vx, int16_t& vy, int16_t& vh);
  bool readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h);
  void writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h);
  void writePoseMm(uint8_t startReg, float xF, float yF, float hF);
};

}  // namespace Hardware
