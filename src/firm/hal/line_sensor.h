// line_sensor.h -- Hal::LineSensor: the N-cell reflectance line-sensor
// interface the loop takes. Pure abstract; no bus, no registers, no chip
// knowledge.
//
// Implementation today: Hardware::LineSensorLeaf
// (hardware/planetx/line_sensor.h), the PlanetX 4-channel array at 0x1A.
//
// --- On the centreline position this interface deliberately does NOT expose ---
//
// The reorganization proposal asks this interface to expose "a computed
// centerline position, not just raw channel values", absorbing the centroid
// currently computed host-side in src/tests/bench/line_follow.py's
// line_error(). It is not here, and the reason is a project rule rather
// than an oversight.
//
// That centroid is not pure math over the reading: it needs the array's
// CHANNEL GEOMETRY (line_follow.py's CHANNEL_Y = 32/8/-8/-32 mm from
// centre) and two signal thresholds (OFF_LEVEL, MIN_SIGNAL) -- five numbers
// that differ per sensor array and per mounting. Under
// .claude/rules/configuration-discipline.md every value the robot uses at
// production boot must come from the robot's own configuration file, and
// every value in that file must have a consumer ("A value in the file that
// nothing consumes breaks invariant 2 as badly as a missing one. Delete it,
// don't wire it."). Nothing on the robot follows a line today -- the
// follower is host-side -- so wiring five new configuration fields now
// would add exactly the unconsumed configuration that rule forbids.
//
// The centroid therefore belongs to the on-robot LineFollower work
// (clasi/issues/proposal-on-robot-linefollower-subsystem.md), which brings
// the consumer with it. It should land here, as a `position()` method on
// this interface, at that point -- reading its geometry and thresholds from
// LineConfig, not from constants in this tree.
#pragma once

#include <cstdint>

#include "hal/device_types.h"

namespace Hal {

class LineSensor {
 public:
  virtual ~LineSensor() = default;

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

  // Raw counts plus their calibrated-normalized counterparts, per cell.
  virtual LineReading reading() const = 0;
  virtual bool readingFresh() const = 0;
};

}  // namespace Hal
