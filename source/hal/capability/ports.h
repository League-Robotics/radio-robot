// ports.h — the Ports faceplate (digital/analog RJ11 breakout). Declaration
// only this ticket — see capability/gripper.h's file header for the
// "declared, not defined" mechanism and rationale; the same applies here.
//
// Deviation notes:
//  - protos/ports.proto has no PortCapabilities message (single command
//    surface, no per-mode gating), so capabilities() is omitted here too.
//  - msg::PortState's digital_in/analog_in are fixed-size arrays
//    (uint8_t[4]/int32_t[4]) with no per-channel scalar accessors in the
//    generated message type, so the primitive getter here returns the
//    whole PortState directly (read()) rather than decomposing into N
//    scalar getters the way Motor's position()/velocity() do — there is no
//    natural per-channel primitive to expose. state() trivially forwards
//    read(). setDigitalOut()/setAnalogOut() mirror PortCommand's two oneof
//    arms directly for the same reason.
//  - msg::PortState has no `connected` field (unlike Gripper/LineSensor/
//    ColorSensorState, which do) — so, unlike those three, this faceplate
//    has no connected() primitive; there is nowhere in PortState to
//    assemble one into.
#pragma once

#include <stdint.h>

#include "messages/ports.h"

namespace Hal {

class Ports {
 public:
  virtual ~Ports() = default;
  virtual void begin() {}

  // Primitive setters — one per PortCommand oneof arm (see file header).
  virtual void setDigitalOut(const msg::DigitalOut& out) = 0;
  virtual void setAnalogOut(const msg::AnalogOut& out) = 0;

  // Primitive getter — the whole read-side state (see file header).
  virtual msg::PortState read() const = 0;

  // Faceplate verbs (no Capabilities message exists yet — see file header).
  virtual void configure(const msg::PortConfig& config) = 0;
  virtual void tick(uint32_t now) = 0;   // [ms]

  // Message plane — declared, not defined (no concrete leaf this sprint).
  bool apply(const msg::PortCommand& command);
  msg::PortState state() const;
};

}  // namespace Hal
