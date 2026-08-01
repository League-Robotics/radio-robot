// device_config.h — Devices-local configuration/calibration types.
//
// See device_types.h's file header for the isolation invariant these types
// exist to satisfy (Devices-local counterparts of the equivalent
// `msg::*`/`Config::*` types, since `devices/` may never
// `#include "messages/..."` or `#include "config/..."`). Every type below
// is, like device_types.h's, a plain aggregate — default-constructible, no
// virtuals/pointers/user-declared special member functions.
//
// 125-003 (sprint 125 Decision 2): `Gains`/`Opt<T>` and MotorConfig's three
// PID-related fields (the gain set, the EMA smoothing coefficient, and the
// integrator-freeze deadband) are DELETED — the closed-loop velocity
// control law they fed relocated to the motion library (its own
// motion-local gain-set type, per the isolation invariant that motion may
// not depend on devices/), and that relocated class was itself deleted
// outright by 128-015 (zero instantiations -- App::Drive holds no
// controller of its own; see src/motion/DESIGN.md's "wheel control
// generations" note).
// `Devices::Motor::applyTravelCalib(float)` replaced the old gain-apply
// method — the one field this leaf still live-applies — so `Opt<T>`'s one
// remaining caller is gone too.
#pragma once

#include <cstdint>

namespace Devices {

// MotorConfig — one motor channel's calibration plus the armor tuning
// MotorArmor::configureArmor() caches. Devices-local counterpart to
// msg::MotorConfig (messages/motor.h) plus its two base-class-owned armor
// fields.
struct MotorConfig {
  float wheelTravelCalib = 0.0f;  // [mm/deg] wheel linear travel per motor-shaft degree of rotation

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

}  // namespace Devices
