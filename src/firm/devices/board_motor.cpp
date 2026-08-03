// board_motor.cpp -- see board_motor.h; reference documentation at
// docs/hiwonder/hiwonder-motor-board.md (cited as "doc s1.5" etc.).
#include "devices/board_motor.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace Devices {

void BoardMotor::begin() {
  zeroPending_ = true;  // real zero captured on the first tick with data
  offset_ = board_.total(channel_);
  position_ = 0.0f;
  velocity_ = 0.0f;
  lastCounts_ = 0;
  deltaRingN_ = 0;
}

void BoardMotor::setDuty(float duty) {
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  appliedDuty_ = duty;
  commanded_ = commanded_ || duty != 0.0f;
  // duty IS the normalized fraction the board interface wants; the
  // board owns the mapping onto its native encoding.
  board_.stageSpeed(channel_, duty * static_cast<float>(config_.fwdSign));
}

void BoardMotor::setNeutral(Neutral) {
  // An explicit zero write, never silence: speed commands LATCH on
  // these boards and there is no watchdog, so a crashed or quiet host
  // leaves the wheel driving forever (doc s1.3). The zero is staged
  // here and flushed by the next exchange (write-on-change fires on
  // the transition; a failed write retries until it lands).
  appliedDuty_ = 0.0f;
  board_.stageSpeed(channel_, 0.0f);
}

bool BoardMotor::reconfigure(const MotorConfig& config) {
  // Guard per the interface contract: refuse when genuinely in motion.
  if (commanded_ && std::fabs(velocity_) > 5.0f) return false;  // [mm/s]
  config_ = config;
  return true;
}

void BoardMotor::tick(uint64_t nowUs) {
  // First channel of the cycle pays the exchange; the rest serve from
  // the same cached read (guarded inside the MotorBoard base).
  board_.exchangeOncePerCycle(nowUs);
  if (!board_.connected()) return;
  const int32_t counts = board_.total(channel_);
  if (zeroPending_) {
    offset_ = counts;
    zeroPending_ = false;
    lastCounts_ = counts;
  }
  position_ = countsToMm(counts - offset_);

  // ---- velocity: tick-attributed, not clock-divided (doc s1.5) ----
  const int32_t delta = counts - lastCounts_;
  if (delta != 0) {
    const int32_t mag = std::abs(delta);
    // windowed median of recent per-tick magnitudes
    int n = deltaRingN_ < kDeltaRing ? deltaRingN_ : kDeltaRing;
    int32_t sorted[kDeltaRing];
    std::memcpy(sorted, deltaRing_, sizeof(int32_t) * n);
    for (int i = 1; i < n; ++i) {  // insertion sort; n <= 9
      const int32_t key = sorted[i];
      int j = i - 1;
      for (; j >= 0 && sorted[j] > key; --j) sorted[j + 1] = sorted[j];
      sorted[j + 1] = key;
    }
    const int32_t median = (n > 0) ? sorted[n / 2] : 0;
    const bool isDouble = mag >= kDoubleFloor && median > 0 &&
                          static_cast<float>(mag) >
                              kDoubleRatio * static_cast<float>(median);
    const float ticks = isDouble ? 2.0f : 1.0f;
    velocity_ = countsToMm(delta) / (ticks * board_.encoderTick());
    // The ring stores the PER-TICK magnitude, so a split double does
    // not drag the median it will be compared against next time.
    deltaRing_[deltaRingN_ % kDeltaRing] =
        static_cast<int32_t>(static_cast<float>(mag) / ticks);
    ++deltaRingN_;
    lastCounts_ = counts;
  } else if (appliedDuty_ == 0.0f) {
    // No fresh counts and nothing commanded: decay toward stopped
    // rather than latching the last moving estimate forever.
    velocity_ *= 0.5f;
    if (std::fabs(velocity_) < 1.0f) velocity_ = 0.0f;  // [mm/s]
  }
}

void BoardMotor::resetPosition() {
  offset_ = board_.total(channel_);
  position_ = 0.0f;
}

void BoardMotor::rebaseline() {
  // Contract (robot_loop::publishWheel): software-only re-anchor that
  // brings position() back inside the wire bound. The zero on these
  // boards is purely a host-side offset, so the re-anchor IS the
  // offset fold -- identical to resetPosition(). Velocity is
  // untouched: the estimator differences raw board counts, not
  // position.
  resetPosition();
}

}  // namespace Devices
