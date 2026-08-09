#pragma once

#include <cstdint>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "platform/i2c_bus.h"

namespace Devices {

constexpr uint8_t kOtosDeviceAddr = 0x17;

int8_t scaleToRegister(float scale);

class Otos {
 public:
  virtual ~Otos();

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

class RealOtos : public Otos {
 public:
  RealOtos(Platform::I2CBus& bus, const OtosConfig& config);

  void begin() override;

  PoseReading pose() const override;

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
  OtosConfig config_;

  bool initialized_ = false;

  bool connected_ = false;
  uint8_t lastProbeId_ = 0;   // last product-ID byte read by begin()

  PoseReading cachedPose_{};
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

}  // namespace Devices
