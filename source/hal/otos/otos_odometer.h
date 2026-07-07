// otos_odometer.h — Hal::OtosOdometer: the real-hardware Hal::Odometer leaf
// for the SparkFun Optical Tracking Odometry Sensor (OTOS), I2C address
// 0x17 (ticket 086-006).
//
// This is the real-hardware counterpart to Hal::SimOdometer (source/hal/
// sim/sim_odometer.h, 081-003) — the first CONCRETE Hal::Odometer this
// program can actually reach hardware through (Subsystems::NezhaHardware::
// odometer() has returned the base's nullptr default since sprint 081-002;
// see subsystems/hardware.h's own file header for that history).
//
// Lives at source/hal/otos/ — a NEW top-level HAL device directory,
// parallel to hal/nezha/, hal/sim/, hal/capability/ (NOT nested under
// hal/nezha/): the OTOS sensor is not a Nezha-brand device, it just happens
// to be orchestrated by the same Subsystems::NezhaHardware owner in this
// single-hardware-owner tree (see nezha_hardware.h's own file header for why
// that owner class lives where it does).
//
// Register map / read sequencing ported (concept/math, not verbatim syntax)
// from source_old/hal/real/OtosSensor.{h,cpp}:
//   0x00  PRODUCT_ID       (read; expected 0x5F)
//   0x04  LINEAR_SCALAR    (signed int8, 0.1% resolution)
//   0x05  ANGULAR_SCALAR   (signed int8, 0.1% resolution)
//   0x06  IMU_CALIBRATION  (write N to start a background bias calibration)
//   0x07  RESET            (bit 0: reset Kalman tracking)
//   0x0E  SIGNAL_PROCESS_CFG (LUT=0x01, Accel=0x02, Rotation=0x04, Variance=0x08)
//   0x20  POSITION_XL      (6 bytes: X_L X_H Y_L Y_H H_L H_H, signed int16 LE)
//   0x26  VELOCITY_XL      (6 bytes, same layout)
// REG_OFFSET (0x10-0x15) is deliberately NEVER written — verified on this
// hardware to ACK the write and silently keep reading back 0 (source_old's
// own finding, restated in ticket 086-005's issue). The mounting-offset
// (lever-arm) compensation is applied HOST-SIDE instead, via source/hal/
// lever_arm.h's LeverArm::sensorToCentre()/centreToSensor() — see tick()'s
// own comment for the same-instant-heading contract that math relies on.
//
// Deliberate deviation from source_old: OtosSensor::init() BLOCK-POLLED
// (fiber_sleep-based busy-wait, up to ~0.77s) for the IMU bias calibration
// to finish before returning. This tree's main loop has no fiber_sleep-style
// scheduler-yield primitive (HOST_BUILD has none at all), and blocking the
// whole dev loop — comms, motor control, telemetry — for the better part of
// a second on every OI command (not just boot) is exactly the class of
// stall sprints 078/079 spent real stand time eliminating from the Nezha
// path (see nezha_motor.cpp's requestEncoder()/writeMotorRun() comments).
// This leaf instead only WRITES REG_IMU_CALIBRATION (a fire-and-forget
// kick-off, matching the chip's own documented async behavior — source_old's
// OtosSensor.h: "Calibration runs asynchronously") and does not poll for
// completion. The chip finishes calibrating in its own background timer
// regardless of whether anything waits for it.
//
// Bus safety: reuses I2CBus's existing per-device preClear/postClear lazy-
// clearance mechanism (already generic over any 7-bit address) for every
// read/write here — no second, hand-rolled busy-wait is introduced. OTOS
// (0x17) and the Nezha motor bus (0x10, Subsystems::NezhaHardware's brick
// flip-flop) are different device-slot addresses, so this leaf's own I2C
// traffic never contends with or is scheduled by the flip-flop sequencer —
// dev_loop.cpp drives this leaf's tick()/pose() on its own, once per pass,
// entirely outside NezhaHardware::tick().
//
// Construction: takes the ticket 086-005 boot-config values
// (Config::OtosBootConfig — mounting offset + linear/angular scale
// multipliers) directly. There is no msg::-shaped equivalent (unlike
// msg::MotorConfig for Hal::NezhaMotor) because this data is deliberately
// NOT a live wire surface — see boot_config.h's own OtosBootConfig doc
// comment. Config:: has no Hal:: dependency of its own, so this one-
// directional Hal -> Config include introduces no cycle.
#pragma once

#include <stdint.h>

#include "com/i2c_bus.h"
#include "config/boot_config.h"
#include "hal/capability/odometer.h"
#include "messages/common.h"
#include "messages/odometer.h"

namespace Hal {

// 7-bit I2C address of the SparkFun OTOS chip — a different device slot
// from kNezhaDeviceAddr (0x10, nezha_motor.h), so this leaf's own I2CBus
// clearance timers never interact with the Nezha flip-flop's.
constexpr uint8_t kOtosDeviceAddr = 0x17;

class OtosOdometer : public Odometer {
 public:
  OtosOdometer(I2CBus& bus, const Config::OtosBootConfig& config);

  // Detect (PRODUCT_ID read) and, if found: enable signal processing +
  // reset Kalman tracking + kick off IMU bias calibration (this class's
  // init(), the OI primitive's effect — see file header for why this does
  // NOT block-poll for calibration completion), apply the boot-config
  // linear/angular scale multipliers (converted to the chip's raw int8
  // register scalar — scaleToRegister()), and zero the OTOS position AND
  // heading (mirrors OtosSensor::begin(): the chip retains its tracked pose
  // across a micro:bit reset/reflash, so without this the very first tick()
  // would report a stale pose against the encoders' fresh (0,0,0) origin).
  // Sets connected() accordingly; a failed product-ID detect leaves this
  // leaf permanently un-initialized (no further bus traffic — mirrors
  // source_old's is_initialized() gate) since there is nothing to recover
  // from an absent/never-detected chip.
  void begin() override;

  // Returns the cached pose computed by the most recent tick() — a cheap
  // accessor, never issues I2C traffic (mirrors Hal::SimOdometer::pose() /
  // NezhaMotor::position()'s tick()-caches-then-getters-read-cache
  // contract). Defaults to a zero pose with stamp.valid == false before the
  // first successful tick().
  msg::PoseEstimate pose() const override;

  // True once PRODUCT_ID was detected at begin() AND the most recent tick()
  // (or begin()'s own probe, before the first tick()) completed its I2C
  // burst without error.
  bool connected() const override;

  // Burst-reads POSITION_XL then VELOCITY_XL, applies the mounting-yaw
  // rotation (config's offsetYaw) and the lever-arm compensation (source/
  // hal/lever_arm.h) using the SAME-INSTANT heading from THIS burst — never
  // a heading left over from a previous tick (see lever_arm.h's own
  // same-instant-heading contract; a stale heading here is the exact
  // db11b7c phantom-translation failure mode). Caches the result for
  // pose(); a burst failure holds the previously-cached pose but marks it
  // stale (stamp.valid = false) so Subsystems::PoseEstimator::tick() skips
  // fusion this pass (pose_estimator.cpp checks otosObs->stamp.valid). Every
  // tick() call attempts the read regardless of the previous call's
  // outcome — a transient bus glitch does not permanently disable further
  // attempts (mirrors Hal::NezhaMotor::tick()'s own always-retry
  // connected_ semantics). No-op (no bus traffic) if begin() never detected
  // the chip. now: [ms].
  void tick(uint32_t now) override;

  // --- Hal::Odometer's primitive setters — each a no-op if begin() never
  // detected the chip (mirrors source_old's is_initialized() guard on every
  // one of these). ---
  void init() override;                              // OI
  void resetTracking() override;                      // OR
  void setPose(const msg::Pose2D& pose) override;     // OZ (all-zero) / OV
  // OL/OA operate on the chip's raw int8 register scalar directly
  // (docs/protocol-v2.md §11 — "int8_t" register value, not a 1.0-based
  // multiplier), matching the wire contract otos_commands.cpp's handleOL/
  // handleOA already implement. The boot-config linear/angular SCALE
  // multipliers (Config::OtosBootConfig) are a different domain, converted
  // once at begin() via scaleToRegister() before being handed to these same
  // setters — see begin()'s own comment.
  void setLinearScalar(float scalar) override;        // OL
  void setAngularScalar(float scalar) override;        // OA

 private:
  I2CBus& bus_;
  Config::OtosBootConfig config_;

  // True once PRODUCT_ID matched at begin() — gates ALL further bus traffic
  // (mirrors source_old's is_initialized()); never re-probed after begin().
  bool initialized_ = false;

  // Live per-tick bus-health flag — see tick()'s doc comment for why this
  // is retried every call rather than latching permanently false.
  bool connected_ = false;

  msg::PoseEstimate cachedPose_{};

  // Register addresses — ported from source_old/hal/real/OtosSensor.h.
  static constexpr uint8_t kRegProductId        = 0x00;
  static constexpr uint8_t kRegLinearScalar     = 0x04;
  static constexpr uint8_t kRegAngularScalar    = 0x05;
  static constexpr uint8_t kRegImuCalibration   = 0x06;
  static constexpr uint8_t kRegReset            = 0x07;
  static constexpr uint8_t kRegSignalProcessCfg = 0x0E;
  static constexpr uint8_t kRegPositionXl       = 0x20;
  static constexpr uint8_t kRegVelocityXl       = 0x26;

  static constexpr uint8_t kExpectedProductId = 0x5F;

  // IMU calibration sample count written to REG_IMU_CALIBRATION by init()
  // (fire-and-forget — see file header). Matches source_old's
  // kImuCalibSamples (255 samples ≈ 0.77 s at ~3 ms/sample, chip-internal).
  static constexpr uint8_t kImuCalibSamples = 255;

  // LSB scale factors — position, velocity, and (unread by this leaf)
  // acceleration all share the same register layout (see source_old/hal/
  // real/OtosSensor.cpp's "OTOS LSB Scale Factors" comment block for the
  // SparkFun-library derivation this restates).
  static constexpr float kPosMmPerLsb = 0.305f;                            // [mm/LSB]
  static constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f); // [rad/LSB]

  // Convert a calibration scale multiplier (e.g. 1.05) to the chip's signed
  // int8 register scalar (0.1% per LSB), clamped to [-127, 127]. Ported
  // from OtosSensor::scaleToInt8().
  static int8_t scaleToRegister(float scale);

  void writeReg8(uint8_t reg, uint8_t val);
  uint8_t readReg8(uint8_t reg);
  // Burst-reads 6 bytes from a triple-register block (X_L X_H Y_L Y_H H_L
  // H_H) starting at startReg. Returns true iff both the register-address
  // write and the data read succeeded.
  bool readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h);
  // Burst-writes three signed int16 to a triple-register block.
  void writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h);
};

}  // namespace Hal
