// color_sensor.h -- Hal::ColorSensor: the RGBC colour-sensing interface the
// loop takes. Pure abstract; no bus, no registers, no chip knowledge.
//
// Implementation today: Hardware::ColorSensorLeaf
// (hardware/planetx/color_sensor.h), which handles both the PlanetX chip at
// 0x43 and an APDS9960 at 0x39.
//
// The surface is exactly what Core::Preamble and Core::RobotLoop already call,
// no more: a non-blocking detection step, the present/connected pair, a
// non-blocking per-cycle tick(), and the published reading. Extracting it
// costs nothing at runtime (the leaf was already driven through these seven
// methods) and is what lets a robot with a different colour sensor link.
#pragma once

#include <cstdint>

#include "hal/device_types.h"

namespace Hal {

class ColorSensor {
 public:
  virtual ~ColorSensor() = default;

  // Non-blocking single detection step, called once per fiber cycle until
  // detectDone(). No implementation may sleep or block -- see
  // hardware/DESIGN.md's "No leaf sleeps or blocks" invariant.
  virtual void beginStep(uint64_t nowUs) = 0;  // [us]
  virtual bool detectDone() const = 0;

  // present(): set once by detection and never re-evaluated.
  // connected(): the live, per-tick() bus-health result.
  virtual bool present() const = 0;
  virtual bool connected() const = 0;

  // The one steady-state sampling entry point. Non-blocking, and free to
  // rate-limit itself against its own polling budget.
  virtual void tick(uint64_t nowUs) = 0;  // [us]

  virtual ColorReading reading() const = 0;
  virtual bool readingFresh() const = 0;

  // Full-scale count for this chip/integration setting -- what a saturated
  // channel would read. Lets a consumer normalize without knowing the part.
  virtual uint32_t fullScale() const = 0;
};

}  // namespace Hal
