#pragma once

#include <cstdint>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"

namespace Devices {

constexpr uint8_t kOtosDeviceAddr = 0x17;

// scaleToRegister -- convert a calibration scale MULTIPLIER (e.g. 1.05, 1.0 =
// no correction) to the chip's signed int8 register scalar (0.1%-per-LSB,
// -127..127; `scalar = clamp(round((scale - 1.0) / 0.001), -127, 127)`).
//
// A free function, not a RealOtos member (132-010, the-configuration-
// object.md trap 3): RealOtos::begin() (otos.cpp, the BOOT path) and
// App::configureOtos() (app/boot_calibration.cpp, the LIVE wire-push path)
// both need this EXACT conversion before calling setLinearScalar()/
// setAngularScalar() below -- those two setters take the register domain
// directly, never the multiplier. Trap 3 WAS exactly this conversion
// existing on only the boot call site: the live path passed the multiplier
// straight through, so a pushed `linearScale = 1.0` installed register value
// 1 (a real +0.1% scalar) instead of register 0 (true unity). Living here,
// at namespace scope, keeps it callable from both without requiring the
// caller to hold a RealOtos instance (App::configureOtos() only ever holds
// the abstract Devices::Otos&) and without duplicating the formula a second
// time (a prior duplicate, `src/tests/sim/system/faults/
// sim_fidelity_harness.cpp`'s own file-scope copy, predates this move and is
// retired by the same ticket). Devices::-scoped rather than App::-scoped
// because it is pure chip-register math with zero App::/Config::
// dependency -- exposing it here does not weaken the devices/ isolation
// invariant (src/firm/DESIGN.md §5) the way pulling in a Config:: type
// would.
int8_t scaleToRegister(float scale);

// Otos — the abstract OTOS interface App:: drives (see this file's header
// for the interface/implementation-split rationale). Two implementations:
// RealOtos (below) and App::FakeOtos (app/fake_otos.h). Every method here
// is exactly one App:: call site drives through a `Devices::Otos&`: the
// per-cycle read path (begin/tick/pose/poseFresh/connected/present, via the
// loop + Preamble + applyOtosSample()) and the CONFIG calibration path
// (setLinearScalar/setAngularScalar/setOffset/getOffset/init, via
// RobotLoop's OTOS-config patch handler). Implementation-only primitives
// (setPose, resetTracking, readDue, the raw-register setters, …) stay on
// RealOtos and are NOT part of this contract.
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

  virtual void setLinearScalar(float scalar) = 0;
  virtual void setAngularScalar(float scalar) = 0;
  virtual void setOffset(float x, float y, float heading) = 0;  // [mm] [mm] [rad]
  virtual void getOffset(float& x, float& y, float& heading) = 0;  // [mm] [mm] [rad]
  virtual void init() = 0;

  // No Config::Robot-consuming configure() on THIS interface (132-007,
  // the-configuration-object.md) -- this file's own "Scope changes"
  // header section already establishes why: Devices-local
  // Devices::OtosConfig exists instead of reusing a Config:: type
  // specifically "so this leaf never includes config/" (the devices
  // isolation invariant). App::configureOtos(Devices::Otos&, const
  // Config::Robot&) (app/boot_calibration.h) is the Config::Robot-
  // consuming entry point instead, calling setLinearScalar()/
  // setAngularScalar()/setOffset() above.
};

class RealOtos : public Otos {
 public:
  RealOtos(I2CBus& bus, const OtosConfig& config);

  void begin() override;

  PoseReading pose() const override;

  bool poseFresh() const override;

  bool connected() const override;

  bool present() const override;

  uint8_t lastProbeId() const { return lastProbeId_; }

  uint64_t sampleTime() const override { return lastReadUs_; }  // [us]

  bool readDue(uint64_t nowUs) const;  // [us]

  void tick(uint64_t nowUs) override;  // [us]

  void setPose(float x, float y, float heading);  // [mm] [mm] [rad]


  void resetTracking();

  void setLinearScalar(float scalar) override;
  void setAngularScalar(float scalar) override;

  void setOffset(float x, float y, float heading) override;  // [mm] [mm] [rad]
  void getOffset(float& x, float& y, float& heading) override;  // [mm] [mm] [rad]

  void setSignalProcessConfig(uint8_t config);
  uint8_t signalProcessConfig();

  uint8_t imuCalibrationSamplesRemaining();

  void init() override;

 private:
  I2CBus& bus_;
  OtosConfig config_;

  bool initialized_ = false;

  bool connected_ = false;
  uint8_t lastProbeId_ = 0;

  PoseReading cachedPose_{};
  bool poseFresh_ = false;

  uint64_t lastReadUs_ = 0;  // [us] time of the most recent REAL bus read
  bool hasRead_ = false;

  bool posePending_ = false;
  float pendingX_ = 0.0f;  // [mm]
  float pendingY_ = 0.0f;  // [mm]
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

  static constexpr float kPosMmPerLsb = 0.305f;  // [mm/LSB]
  static constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);  // [rad/LSB]

  static constexpr uint32_t kBusClearance = 4000;  // [us]

  static constexpr uint64_t kReadPeriod = 20000;  // [us]

  // scaleToRegister() -- MOVED (132-010) to a Devices::-namespace-scope free
  // function, above `class Otos`: begin() below calls it unqualified (same
  // namespace); see that declaration's own comment for why it is no longer
  // a RealOtos member.

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

}
