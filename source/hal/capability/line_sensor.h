// line_sensor.h — the LineSensor faceplate. Declaration only this ticket —
// see capability/gripper.h's file header for the "declared, not defined"
// mechanism; the same applies here.
//
// Deviation notes:
//  - protos/sensors.proto defines no LineSensorCommand — a line sensor is
//    read-only, so there is no command-in channel and therefore no
//    apply()/Command pairing on this faceplate at all (unlike Motor/
//    Gripper/Ports). Only the observation (State) and config channels
//    exist. There is also no LineSensorCapabilities message.
//  - msg::LineSensorState's raw/normalized fields are fixed-size arrays
//    (uint32_t[4]) with no per-channel scalar accessors, so the primitive
//    getter returns the whole LineSensorState directly (read()) rather
//    than decomposing into per-channel scalar getters — same reasoning as
//    capability/ports.h. state() trivially forwards read().
#pragma once

#include <stdint.h>

#include "messages/sensors.h"

namespace Hal {

class LineSensor {
 public:
  virtual ~LineSensor() = default;
  virtual void begin() {}

  // Primitive getter — the whole read-side state (see file header).
  virtual msg::LineSensorState read() const = 0;

  // Faceplate verbs (no Command/Capabilities message exists — see file
  // header).
  virtual void configure(const msg::LineSensorConfig& config) = 0;
  virtual void tick(uint32_t now) = 0;   // [ms]

  // Message plane — declared, not defined (no concrete leaf this sprint).
  // No apply(): this proto has no Command message (read-only sensor).
  msg::LineSensorState state() const;
};

}  // namespace Hal
