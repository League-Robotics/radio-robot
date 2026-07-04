// gripper.h — the Gripper faceplate. Declaration only this ticket: no
// concrete leaf exists yet (later ticket, out of sprint 077's scope), so
// apply()/state() are declared but not defined here — see the mechanism
// note below. Ticket 3 acceptance criteria: "one faceplate header each,
// following the same primitive-setters/-getters + configure/tick/
// capabilities + shared apply/state shape... Do not implement apply()/
// state() for these five if there's no primitive surface to base them on
// yet."
//
// Deviation note: protos/gripper.proto (regenerated in ticket 002) has no
// GripperCapabilities message — Gripper's command surface is a single
// optional `angle` field, not a oneof of control modes, so there is nothing
// to capability-gate the way Motor's five-way oneof needs. capabilities()
// is therefore omitted from this faceplate rather than invented.
//
// Mechanism for "declared, not defined": apply()/state() are ordinary
// (non-virtual) member function declarations with no body in this header
// and no gripper.cpp anywhere in the tree. Gripper is abstract (its
// setter/getter/configure/tick are pure virtual), so it cannot be
// instantiated and nothing can call apply()/state() through an instance —
// the header compiles standalone (any TU may include it), and there is no
// link error because the undefined functions are never ODR-used. The first
// ticket that adds a concrete Gripper leaf defines these two once, exactly
// as capability/motor.h does today.
#pragma once

#include <stdint.h>

#include "messages/gripper.h"

namespace Hal {

class Gripper {
 public:
  virtual ~Gripper() = default;
  virtual void begin() {}

  // Primitive setter/getter — the one degree of freedom this device has.
  virtual void setAngle(float angle) = 0;   // [deg]
  virtual float angle() const = 0;          // [deg]
  virtual bool connected() const = 0;

  // Faceplate verbs (no Capabilities message exists yet — see file header).
  virtual void configure(const msg::GripperConfig& config) = 0;
  virtual void tick(uint32_t now) = 0;   // [ms]

  // Message plane — declared, not defined (no concrete leaf this sprint).
  // bool return mirrors capability/motor.h's apply() shape for consistency
  // across faceplates; the implementing ticket may revisit it once a real
  // leaf exists to validate the shape against.
  bool apply(const msg::GripperCommand& command);
  msg::GripperState state() const;
};

}  // namespace Hal
