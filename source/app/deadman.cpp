// deadman.cpp -- App::Deadman implementation. See deadman.h's file header
// for the module's narrow boundary.
#include "app/deadman.h"

namespace App {

Deadman::Deadman(const Devices::Clock& clock) : clock_(clock) {}

void Deadman::arm(float duration) {
  // Malformed-wire-safety clamp: negative values and NaN both fail
  // `duration > 0.0f` (NaN comparisons are always false), so both clamp to
  // 0 -- immediate expiry, not a general min/max bound.
  float clamped = (duration > 0.0f) ? duration : 0.0f;

  // [ms] -> [us]. Multiply while still float-typed so sub-millisecond
  // fractions of `clamped` are not truncated before the unit conversion.
  const uint64_t deltaMicros = static_cast<uint64_t>(clamped * 1000.0f);

  deadlineMicros_ = clock_.nowMicros() + deltaMicros;
  armed_ = true;
}

void Deadman::disarm() { armed_ = false; }

bool Deadman::expired() const {
  if (!armed_) return false;
  return clock_.nowMicros() >= deadlineMicros_;
}

}  // namespace App
