// deadman.h -- App::Deadman: the ONE staleness rule gating every actuation
// source in the new single-loop design (architecture-update.md (103) Step
// 3 "Deadman" boundary: inside -- arm(duration)/disarm()/expired() and the
// one timer they operate on; outside -- what happens when it expires (the
// loop calls Drive::stop(); Deadman itself never touches Drive or any
// other module)). No second ad hoc watchdog timer belongs anywhere else in
// source/app/. Serves SUC-004.
#pragma once

#include <cstdint>

#include "devices/clock.h"

namespace App {

class Deadman {
 public:
  explicit Deadman(const Devices::Clock& clock);

  // duration -- [ms], Twist.duration's own wire unit (envelope.proto:
  // "float duration = 3; // [ms] deadman arm window"). Negative/NaN input
  // is malformed-wire-safety-clamped to 0 (immediate expiry) -- not a
  // general min/max bound (the ticket doesn't specify one; no additional
  // clamping is invented here). Every call sets a FRESH deadline from now,
  // unconditionally (re-arming, not stacking) -- matches "every actuation
  // command arms the deadman."
  void arm(float duration);

  // Cancels unconditionally.
  void disarm();

  // Reads clock_.nowMicros() internally -- Deadman's only dependency
  // beyond its own state, per the ticket's "no dependencies beyond the
  // clock seam". False while disarmed or never armed.
  bool expired() const;

 private:
  const Devices::Clock& clock_;
  bool armed_ = false;
  uint64_t deadline_ = 0;  // [us]
};

}  // namespace App
