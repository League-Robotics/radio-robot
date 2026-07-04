// color_sensor.h — the ColorSensor faceplate. Declaration only this ticket
// — see capability/gripper.h's file header for the "declared, not defined"
// mechanism; the same applies here.
//
// Deviation notes:
//  - protos/sensors.proto defines no ColorSensorCommand — a color sensor
//    is read-only, so there is no command-in channel and no apply()/
//    Command pairing on this faceplate (same as LineSensor). There is also
//    no ColorSensorCapabilities message.
//  - Unlike LineSensor/Ports, msg::ColorSensorState's fields (r/g/b/c,
//    connected) are all plain scalars, so this faceplate DOES decompose
//    into real primitive getters (r()/g()/b()/c()/connected()), matching
//    Motor's shape more closely than the array-heavy faceplates do.
#pragma once

#include <stdint.h>

#include "messages/sensors.h"

namespace Hal {

class ColorSensor {
 public:
  virtual ~ColorSensor() = default;
  virtual void begin() {}

  // Primitive getters — the real reads, served from what tick() last
  // sampled.
  virtual uint32_t r() const = 0;
  virtual uint32_t g() const = 0;
  virtual uint32_t b() const = 0;
  virtual uint32_t c() const = 0;
  virtual bool connected() const = 0;

  // Faceplate verbs (no Command/Capabilities message exists — see file
  // header).
  virtual void configure(const msg::ColorSensorConfig& config) = 0;
  virtual void tick(uint32_t now) = 0;   // [ms]

  // Message plane — declared, not defined (no concrete leaf this sprint).
  // No apply(): this proto has no Command message (read-only sensor).
  msg::ColorSensorState state() const;
};

}  // namespace Hal
