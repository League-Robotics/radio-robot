// device_config.h — Devices-local configuration/calibration types.
//
// See device_types.h's file header for the isolation invariant these types
// exist to satisfy (Devices-local counterparts of the equivalent
// `msg::*`/`Config::*` types, since `devices/` may never
// `#include "messages/..."` or `#include "config/..."`). Every type below
// is, like device_types.h's, a plain aggregate — default-constructible, no
// virtuals/pointers/user-declared special member functions.
#pragma once

#include <cstdint>

namespace Hal {

// MotorConfig — one motor channel's calibration plus the armor tuning
// MotorArmor::configureArmor() caches. Devices-local counterpart to
// msg::MotorConfig (messages/motor.h) plus its two base-class-owned armor
// fields.
struct MotorConfig {
  // wheelTravelCalib is GONE (exploratory kernel, 2026-08-15): the leaf is
  // counts-native now — position()/velocity() report shaft encoder counts
  // (1 count = 0.1 deg, the Nezha 0x46 register's own unit). The mm
  // conversion (`travel_calib` in the robot JSON) belongs to the
  // APPLICATION layer; no mm value exists at or below this layer.

  // +1 or -1: corrects a mirror-mounted wheel's encoder/duty sign
  // (dimensionless — no unit tag).
  int32_t fwdSign = 0;

  // Maximum |duty write step| per tick, in the leaf's own raw hardware
  // write domain (e.g. Nezha's int8 PWM-percent register) — a device-write
  // primitive, not itself a physical quantity, so no `// [unit]` tag
  // applies.
  float slewRate = 0.0f;

  // 1-based port label (wire/config convention) — dimensionless.
  uint32_t port = 0;

  float reversalDwell = 0.0f;    // [ms]
  float outputDeadband = 0.0f;   // [-1, 1] fraction

  // Minimum interval between non-stop duty writes. Replaces the leaf's
  // old hand-synced kMinWriteIntervalUs = 27000 literal (which encoded
  // "kCycle - 5 ms jitter margin" and silently drifted whenever the loop
  // period changed — the devices-isolation invariant forbids the leaf
  // reading the loop's own constant, so the value must ARRIVE via config).
  // The composition root derives it from the kernel's cycle period at
  // compose time. <= 0 disables the throttle (write-on-change + slew still
  // apply). Stop writes always bypass it.
  float writeThrottle = 0.0f;    // [us]

  bool polled = false;
};

// OtosConfig — the OTOS lever-arm mounting offset plus linear/angular scale
// multipliers. Devices-local counterpart to Config::OtosBootConfig
// (config/boot_config.h), which the isolation invariant forbids including
// directly.
struct OtosConfig {
  float offsetX = 0.0f;       // [mm] mounting offset from chassis centre to sensor
  float offsetY = 0.0f;       // [mm]
  float offsetYaw = 0.0f;     // [rad] mounting yaw offset
  float linearScale = 1.0f;   // OTOS linear scale multiplier; 1.0 = no correction
  float angularScale = 1.0f;  // OTOS angular scale multiplier; 1.0 = no correction
};

// ColorConfig — Devices-local counterpart to msg::ColorSensorConfig
// (messages/sensors.h).
struct ColorConfig {
  uint32_t lagColor = 0;     // [ms] acceptable reading-staleness threshold
  uint32_t integration = 0;  // raw sensor integration-time register value
  uint32_t gain = 0;         // raw sensor gain register value
};

// LineConfig — Devices-local counterpart to msg::LineSensorConfig
// (messages/sensors.h).
struct LineConfig {
  uint32_t lagLine = 0;      // [ms] acceptable reading-staleness threshold
  uint32_t calMin[4] = {};   // raw ADC counts, per-channel calibration floor
  uint32_t calMax[4] = {};   // raw ADC counts, per-channel calibration ceiling
  float filtAlpha = 0.0f;    // dimensionless EMA smoothing coefficient
};

}  // namespace Hal
