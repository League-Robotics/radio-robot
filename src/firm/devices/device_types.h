// device_types.h — Devices-local reading/value types.
//
// The isolation invariant (see DESIGN.md §3) forbids `devices/*.{h,cpp}`
// from including anything outside its own `devices/` headers, the C/C++
// standard library, and CODAL/micro:bit — in particular NOT `messages/*`.
// Every value this subsystem's leaves publish or accept is therefore a
// Devices-local counterpart of the equivalent `msg::*` type, not the
// `msg::*` type itself.
//
// Every type below is a plain aggregate: default-constructible with
// in-class initializers, no virtuals, no pointers, no user-declared special
// member functions — every type here is trivially copyable and standard
// layout, so a `MeasurementRing<T>` publish (measurement_ring.h) is just a
// memcpy-equivalent struct assignment, never a constructor/destructor call.
//
// `Sample<T>`/timestamp scaffolding is deliberately NOT defined here — it is
// not trivially part of these value types (it wraps them, generically, one
// level up) — see measurement_ring.h.
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
// the two linear components are carried separately. msg::PoseEstimate's
// freshness ValueSet is not carried here: the ring's own Sample<T> wrapper
// (measurement_ring.h) supplies the stamp/valid bits that would otherwise
// duplicate it.
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
// directly. Distinct from commanding a motor's velocity target to zero
// (NezhaMotor::setVelocity(0) means "PID actively chases zero"; Neutral
// means "stop driving the output entirely").
enum class Neutral : uint8_t {
  Coast,
  Brake,
};

}  // namespace Devices
