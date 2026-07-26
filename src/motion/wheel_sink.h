// wheel_sink.h -- Motion::WheelSink: the ONE narrow boundary interface
// Motion drives every cycle to command wheel actuation, plus the per-wheel
// estimate Motion reads back and the plain config Motion is constructed
// with. This is the "boundary interface" sprint 122's architecture (Step 3,
// Design Rationale Decision 1) calls for -- the one piece of NEW design work
// in an otherwise pure mechanical move.
//
// Boundary: a DUTY sink -- setDuty(left, right)/stop() -- never a velocity
// sink (125-002 retools this: the base's command primitive becomes per-wheel
// duty [-1,1]; the stakeholder's rate argument, 2026-07-24, settles that the
// closed-loop velocity PID buys nothing from base residency, so it moves up
// into src/motion instead -- sprint.md Step 3's "Motion::WheelSink" entry
// and Design Rationale Decision 1). Concrete implementations of WheelSink
// live in the BASE (src/firm/app -- App::Drive implements this interface);
// NOTHING in src/motion implements it. Motion::MoveQueue is the one
// consumer, holding a WheelSink& rather than a concrete App::Drive&.
//
// Zero dependency on src/firm (this header has no #include at all beyond
// <cstdint>) -- WheelEstimate/WheelSinkConfig are plain structs, not
// aliases of any msg::* wire type or Devices::* type. See sprint.md Step 3's
// "Boundary interface" row for the normative In/Out shape; class/struct
// names here are a naming suggestion (Open Question 1), not requirements.
//
// Design/rationale: DESIGN.md (this directory) -- NOT YET updated for this
// retool (125-002's own acceptance criteria: that reconciliation is ticket
// 017's job, once the whole duty-boundary/observer core has landed).
#pragma once

#include <cstdint>

namespace Motion {

// WheelEstimate -- one wheel's reading, as Motion consumes it every cycle.
// Sourced from App::Drive's published RobotState wheel section (the base's
// own per-wheel command observer, App::WheelObserver -- sprint 125 ticket
// 004) rather than a live Devices::Motor read; a plain struct, not an
// adapter/wrapper around either. The normative boundary-struct shape from
// docs/design/base-explicit-loop-sketch.md ("Boundary structs" section).
// Hand-fed into Motion::MoveQueue::tick(), matching MoveQueue's existing
// hand-fed (now, odom) convention rather than a live cross-tree reference
// (ticket 006 wires the actual parameter; this header only introduces the
// type).
struct WheelEstimate {
  float position;    // [mm] cumulative wheel-linear travel
  float velocity;    // [mm/s] signed, observer-corrected estimate
  float age;         // [s] time since this estimate's last fresh-sample correction
  uint32_t rejects;  // cumulative innovation-rejected samples (observer's own count)
  bool wedged;       // true iff the observer's innovation logic flags this wheel wedged
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

// WheelSink -- the abstract wheel-DUTY command sink Motion drives every
// cycle (125-002 retools this from a velocity sink). Implemented by the
// base (src/firm/app -- App::Drive); NEVER implemented inside src/motion.
// No concrete implementation lives in this header. The base's own two
// safety properties on this boundary (sprint.md SUC-001): zero-on-silence
// (a cycle with no setDuty()/stop() call defaults to 0 -- inherited
// structurally from MoveQueue's own unconditional per-cycle tick, no
// watchdog needed here) and a plausibility clamp (|duty| <= 1, NaN -> 0,
// App::Drive's own job, ticket 007).
class WheelSink {
 public:
  virtual ~WheelSink() = default;

  // Commands the next cycle's per-wheel duty targets.
  virtual void setDuty(float left, float right) = 0;  // [-1,1] [-1,1]

  // Commands both wheels to 0 duty.
  virtual void stop() = 0;
};

}  // namespace Motion
