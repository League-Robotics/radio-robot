// device_types.h — Devices-local reading/value types.
//
// Ticket DB-001 (device-bus-tickets.md). Part of the greenfield
// `source/devices/` subsystem (namespace `Devices`) described in
// clasi/issues/device-bus-fiber-owned-self-contained-device-subsystem.md.
//
// The standing isolation invariant (device-bus-tickets.md, "Standing
// isolation invariant") forbids `source/devices/*.{h,cpp}` from including
// anything outside its own `devices/` headers, the C/C++ standard library,
// and CODAL/micro:bit — in particular NOT `messages/*`. Every value this
// subsystem's handles publish or accept is therefore a Devices-local
// counterpart of the equivalent `msg::*` type, not the `msg::*` type
// itself (resolves the "OQ1 leftover" / "Neutral vocabulary" note in
// device-bus-tickets.md).
//
// Every type below is a plain aggregate: default-constructible with
// in-class initializers, no virtuals, no pointers, no user-declared special
// member functions. That is what the issue's "Concurrency contract" rule 2
// ("plain struct stores/copies") and DB-001's acceptance criteria (every
// type `std::is_trivially_copyable` and `std::is_standard_layout`) require
// — a `MeasurementRing<T>` (DB-002) publish is just a memcpy-equivalent
// struct assignment, never a constructor/destructor call.
//
// `Sample<T>`/timestamp scaffolding is deliberately NOT defined here — it is
// not trivially part of these value types (it wraps them, generically, one
// level up) and device-bus-tickets.md's "Resolved open questions" assigns it
// to DB-002 (`uint64_t` [us] stamp width, OQ3).
#pragma once

#include <cstdint>

namespace Devices {

// MotorReading — one motor's latest published sample. Devices-local
// counterpart to msg::MotorState's position/velocity/applied fields (issue
// "The public surface").
struct MotorReading {
  float position = 0.0f;     // [mm]
  float velocity = 0.0f;     // [mm/s] signed
  float appliedDuty = 0.0f;  // [-1, 1]
};

// ColorReading — one r/g/b/c color-sensor sample. Devices-local counterpart
// to msg::ColorSensorState's four raw channel counts (raw ADC counts — no
// physical unit).
struct ColorReading {
  uint32_t r = 0;
  uint32_t g = 0;
  uint32_t b = 0;
  uint32_t c = 0;
};

// LineReading — one 4-channel line-sensor sample, raw ADC counts plus their
// calibrated-normalized counterparts. Devices-local counterpart to
// msg::LineSensorState's raw_[4]/normalized_[4].
struct LineReading {
  uint32_t raw[4] = {};
  uint32_t normalized[4] = {};
};

// PoseReading — pose plus body-frame twist. Devices-local counterpart to
// msg::PoseEstimate's Pose2D{x,y,h} + BodyTwist3{v_x,v_y,omega}. A twist is
// never a bare directionless `v` here — the drivetrain may be holonomic, so
// the two linear components are carried separately (naming-and-style.md
// rule 2). msg::PoseEstimate's freshness ValueSet is not carried here: the
// ring's own Sample<T> wrapper (DB-002) supplies the stamp/valid bits that
// would otherwise duplicate it.
struct PoseReading {
  float x = 0.0f;        // [mm]
  float y = 0.0f;        // [mm]
  float heading = 0.0f;  // [rad]
  float v_x = 0.0f;      // [mm/s]
  float v_y = 0.0f;      // [mm/s]
  float omega = 0.0f;    // [rad/s]
};

// Neutral — coast vs brake. Devices-local counterpart to msg::Neutral
// (messages/common.h), which the isolation invariant forbids including
// directly (device-bus-tickets.md's resolved "OQ1 leftover"). Distinct from
// commanding a motor's velocity target to zero (Motor::setVelocity(0) means
// "PID actively chases zero"; Neutral means "stop driving the output
// entirely," per the issue's public-surface Motor::setNeutral() doc
// comment).
enum class Neutral : uint8_t {
  Coast,
  Brake,
};

}  // namespace Devices
