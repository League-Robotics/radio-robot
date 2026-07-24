// wheel_sink.h -- Motion::WheelSink: the ONE narrow boundary interface
// Motion drives every cycle to command wheel velocities, plus the
// per-wheel state Motion reads back and the plain config Motion is
// constructed with. This is the "boundary interface" sprint 122's
// architecture (Step 3, Design Rationale Decision 1) calls for -- the one
// piece of NEW design work in an otherwise pure mechanical move.
//
// Boundary: a VELOCITY sink -- setWheels(v_left, v_right)/stop() -- never
// a duty sink (Decision 1: the duty-sink rewrite, folding sprint 2's
// PID-placement decision in, is explicitly deferred). Concrete
// implementations of WheelSink live in the BASE (src/firm/app -- ticket
// 002 makes App::Drive implement this interface); NOTHING in src/motion
// implements it. Not yet load-bearing this ticket (122-001): no
// src/firm/app code depends on this header yet, and no src/motion code
// depends on it either -- Motion::MoveQueue (ticket 002) is the first
// consumer, holding a WheelSink& rather than a concrete App::Drive&.
//
// Zero dependency on src/firm (this header has no #include at all beyond
// <cstdint>) -- WheelState/WheelSinkConfig are plain structs, not aliases
// of any msg::* wire type or Devices::* type. See sprint.md Step 3's
// "Boundary interface" row for the normative In/Out shape; class/struct
// names here are a naming suggestion (Open Question 1), not requirements.
//
// Design/rationale: DESIGN.md (this directory).
#pragma once

#include <cstdint>

namespace Motion {

// WheelState -- one wheel's reading, as Motion consumes it every cycle.
// Mirrors what Devices::Motor already exposes (position()/velocity(),
// src/firm/devices/motor.h) without depending on that header -- a plain
// struct, not an adapter/wrapper around Devices::Motor itself.
struct WheelState {
  float position;      // [mm] cumulative wheel-linear travel
  float velocity;       // [mm/s] signed, filtered
  uint64_t sampleTime;   // [us] Devices::Clock::nowMicros() at this reading
};

// WheelSinkConfig -- plain config Motion is constructed with: track width
// plus the VelocityShaper limits (velocity_shaper.h's own next() params)
// for both the linear and angular axes. Supplied once, at construction,
// by whichever composition root wires Motion to the base (main.cpp/
// SimHarness, ticket 002) -- never mutated by Motion itself.
struct WheelSinkConfig {
  float trackWidth;  // [mm] BodyKinematics::inverse()/forward()'s own `b`

  float linearAMax;     // [mm/s^2] linear accel-ramp ceiling
  float linearADecel;   // [mm/s^2] linear decel-taper ceiling
  float linearJMax;     // [mm/s^3] linear jerk ceiling

  float angularAMax;     // [rad/s^2] angular accel-ramp ceiling
  float angularADecel;   // [rad/s^2] angular decel-taper ceiling
  float angularJMax;     // [rad/s^3] angular jerk ceiling
};

// WheelSink -- the abstract wheel-velocity command sink Motion drives
// every cycle. Implemented by the base (src/firm/app -- ticket 002);
// NEVER implemented inside src/motion. No concrete implementation lives
// in this header.
class WheelSink {
 public:
  virtual ~WheelSink() = default;

  // Commands the next cycle's per-wheel velocity targets.
  virtual void setWheels(float v_left, float v_right) = 0;  // [mm/s] [mm/s]

  // Commands both wheels to 0 velocity.
  virtual void stop() = 0;
};

}  // namespace Motion
