// kinematics.h -- Kinematics::Model: the swappable twist<->wheel-speed map.
//
// A drivetrain's kinematics is the ONE place chassis geometry (track width,
// wheelbase) is allowed to live. Everything above states its intent as a
// body twist (v_x, v_y, omega) and lets the model turn that into per-wheel
// linear speeds; everything below deals in wheel speeds and knows nothing
// about the chassis.
//
// Implementations:
//   - Kinematics::Differential (differential.h) -- two
//     wheels, no lateral velocity. Every robot in this project today.
//   - Kinematics::Mecanum (mecanum.h) -- four wheels,
//     holonomic. Togov is a mecanum chassis (data/robots/togov.json), which
//     is why this is the second implementation and not a hypothetical.
//
// A robot's composition root picks one at construction, the same way it
// picks Serial vs. Radio transports.
//
// Naming: the class is `Model`, so a caller writes `Kinematics::Model&`
// rather than the `Kinematics::Kinematics&` a same-name class would force.
#pragma once

namespace Kinematics {

// Twist -- a body-frame velocity, in real units. Plain floats on purpose:
// msg::BodyTwist3 (messages/common.h) looks like the same thing and is NOT
// -- its v_x/v_y/omega are int32_t RAW wire values that only mean anything
// through its own packVX()/unpackVX() 0.1 scale. The pre-reorganization
// BodyKinematics array overloads took msg::BodyTwist3 and assigned raw
// floats straight into those int32 fields, bypassing the pack helpers: a
// 10x scale error plus truncation to whole raw counts, latent only because
// nothing ever called them (every real call site used the scalar forms).
// Those overloads are not carried forward, and this type is why -- the
// kinematics layer deals in real units and never touches the wire schema.
struct Twist {
  float v_x = 0.0f;    // [mm/s] body-frame forward, signed
  float v_y = 0.0f;    // [mm/s] body-frame lateral, signed; 0 for a
                       //        drivetrain that cannot strafe
  float omega = 0.0f;  // [rad/s] yaw rate, CCW-positive
};

// The widest drivetrain this project models. Mecanum is 4; X-drive, if it
// ever appears, is also 4. Callers size their own wheel arrays with this so
// a Model& can be swapped without resizing anything.
constexpr int kMaxWheels = 4;

class Model {
 public:
  virtual ~Model() = default;

  // How many entries of a `wheels[]` array this model reads and writes.
  // Wheel ORDER is each implementation's own documented convention.
  virtual int wheelCount() const = 0;

  // inverse -- body twist to per-wheel linear speeds. A model that cannot
  // realize a requested component (a differential drive asked for v_y)
  // ignores it rather than failing: the caller's own limits, not this map,
  // are what refuse an impossible motion.
  virtual void inverse(const Twist& twist,
                       float wheels[]) const = 0;  // [mm/s]

  // forward -- per-wheel linear speeds to the body twist they produce.
  // Components the drivetrain cannot produce are written as 0.
  virtual void forward(const float wheels[],
                       Twist& twist) const = 0;  // [mm/s]

  // saturate -- curvature-preserving uniform scaling. When the fastest
  // wheel exceeds (wheelSpeedMax - steerHeadroom), scale EVERY wheel by the
  // same factor so the fastest sits exactly at that ceiling and the
  // wheel-speed ratios -- and therefore the path -- are preserved. Below
  // the ceiling this is pass-through.
  //
  // Concrete, not virtual: the rule is a property of "preserve the ratio",
  // which is drivetrain-independent. Only wheelCount() varies, and this
  // reads it. `out` may alias `wheels`.
  void saturate(const float wheels[], float wheelSpeedMax,
                float steerHeadroom, float out[]) const;  // [mm/s] x3
};

}  // namespace Kinematics
