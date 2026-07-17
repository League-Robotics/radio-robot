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

namespace Devices {

// Opt<T> — nullable wrapper for an optional config field. Devices-local
// counterpart to msg::Opt<T> (messages/common.h): same bool-plus-value
// shape, so it stays trivially copyable, but declared here instead of
// included from messages/common.h (isolation invariant). Needed wherever an
// explicit value (including an explicit 0) must stay distinguishable from
// "not configured, substitute the ship default" — see MotorConfig::
// reversalDwell/outputDeadband below; a zero-sentinel cannot serve the same
// purpose because an explicit 0 must remain valid and distinct.
template <typename T>
struct Opt {
  bool has = false;
  T val = T{};
};

// Gains — a generic PI(+feedforward+anti-windup) gain set. Devices-local
// counterpart to msg::Gains (messages/common.h). Dimensionless (gain
// coefficients carry no `// [unit]` tag).
struct Gains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kff = 0.0f;
  float iMax = 0.0f;
  float kaw = 0.0f;
};

// MotorConfig — one motor channel's calibration plus the armor tuning
// MotorArmor::configureArmor() caches. Devices-local counterpart to
// msg::MotorConfig (messages/motor.h) plus its two base-class-owned armor
// fields.
struct MotorConfig {
  float wheelTravelCalib = 0.0f;  // [mm/deg] wheel linear travel per motor-shaft degree of rotation

  // +1 or -1: corrects a mirror-mounted wheel's encoder/duty sign
  // (dimensionless — no unit tag).
  int32_t fwdSign = 0;

  Gains velGains = {};

  // EMA smoothing coefficient applied to the raw velocity sample
  // (dimensionless).
  float velFiltAlpha = 0.0f;

  // [mm/s] |target velocity| at/below this freezes the embedded PID's
  // integrator. Named for what it actually gates, not the wire field it
  // mirrors: msg::MotorConfig's wire key is `min_duty`, but this field
  // thresholds the VELOCITY target, never a duty — wire keys are excluded
  // from the renaming convention (coding-standards.md); this Devices-local
  // field is not a wire key, so it is named for what it is.
  float velDeadband = 0.0f;

  // Maximum |duty write step| per tick, in the leaf's own raw hardware
  // write domain (e.g. Nezha's int8 PWM-percent register) — a device-write
  // primitive, not itself a physical quantity, so no `// [unit]` tag
  // applies.
  float slewRate = 0.0f;

  // 1-based port label (wire/config convention) — dimensionless.
  uint32_t port = 0;

  Opt<float> reversalDwell = {};   // [ms]
  Opt<float> outputDeadband = {};  // [-1, 1] fraction

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
