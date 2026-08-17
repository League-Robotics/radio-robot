// motor_driver_channel.h -- Hardware::MotorDriverChannel: one channel of ANY
// Hal::MotorDriver, presented as a Hal::Motor. Driver-agnostic by
// construction: everything register-shaped lives in the driver class.
//
// NOT WIRED IN YET: nothing constructs this class today. Wiring
// instructions: docs/hiwonder/hiwonder-motor-board.md section 3.
// Reference documentation (cited below as "doc s1.5" etc.): that same
// file. Ported from the hiwonder-spike branch, 2026-08-02.
//
// Because the driver runs its own closed loop, setDuty()'s duty IS the
// normalized speed fraction the driver wants -- Drive's calibrated
// velocity->duty map stays above this seam unchanged. Note the driver's
// own loop makes duty<->speed far closer to linear than the Nezha
// plant; Nezha-tuned trim/PID gains are wrong here (doc s2).
#pragma once

#include <cstdint>

#include "hal/device_config.h"
#include "hal/device_types.h"
#include "hal/motor.h"
#include "hal/motor_driver.h"

namespace Hardware {

class MotorDriverChannel : public Hal::Motor {
 public:
  MotorDriverChannel(Hal::MotorDriver& driver, int channel, const Hal::MotorConfig& config)
      : driver_(driver), channel_(channel), config_(config) {}

  void begin() override;
  void requestSample() override {}  // no split-phase latch on these boards
  void setDuty(float duty) override;  // [-1, 1] -> driver speed command
  void setNeutral(Hal::Neutral) override;
  // applyTravelCalib() is GONE from Hal::Motor with the counts-native leaf
  // (DifferentialDrive kernel rework): travel calibration belongs to the
  // application now, and no mm value exists at or below this layer.
  [[nodiscard]] bool reconfigure(const Hal::MotorConfig& config) override;
  void tick(uint64_t nowUs) override;  // [us]

  float position() const override { return position_; }   // [counts]
  float velocity() const override { return velocity_; }   // [counts/s] signed
  float appliedDuty() const override { return appliedDuty_; }
  bool connected() const override { return driver_.connected(); }
  uint64_t sampleTime() const override { return driver_.sampleTime(); }

  void resetPosition() override;
  void rebaseline() override;

 private:
  // Velocity estimation must cope with the board's own encoder tick
  // (~9.56 ms on HiWonder) beating against the read cycle: every
  // ~220 ms one read spans TWO board updates, and a naive difference
  // quotient reports a +100% spike of physically impossible motion
  // (+190,000 counts/s^2 against a measured achievable ~16,000).
  // Rule, validated against ground truth (14/14 doubles found, zero
  // false positives, plateau sd 792 -> 100 c/s -- doc s1.5): a positive
  // count delta greater than kDoubleRatio times the windowed MEDIAN of
  // recent per-tick magnitudes is one read that swallowed two ticks --
  // keep the counts, credit TWO tick periods. The median (not mean:
  // the mean is dragged by the outlier being hunted) rides a small
  // ring; the floor guards near-zero speed, where a tick is 1-3 counts
  // and +-1 count of quantization legitimately doubles a delta.
  static constexpr int kDeltaRing = 9;
  static constexpr float kDoubleRatio = 1.6f;
  static constexpr int32_t kDoubleFloor = 15;  // [counts]

  // Counts-native leaf: the only thing left to apply here is the mirror-
  // mount sign. The mm scale that used to be folded in (wheelTravelCalib,
  // [mm/count] on these boards) is deleted from Hal::MotorConfig -- the
  // application holds travel calibration now.
  //
  // NOTE for whoever finally wires this class up (nothing constructs it
  // today): these boards count at a DIFFERENT resolution than the Nezha
  // (whose count is 0.1 deg of shaft). The kernel's counts/s config --
  // fullDutyVelocity, the PID gains, the stall/speed floors -- is all
  // expressed in the leaf's own count unit, so it must be rebaked for
  // this board, not copied from a Nezha robot JSON. The old comment here
  // warned about exactly this hazard for the mm scale; it applies just as
  // sharply now that counts go all the way up to the control law.
  float signedCounts(int32_t counts) const {
    return static_cast<float>(counts) * static_cast<float>(config_.fwdSign);
  }

  Hal::MotorDriver& driver_;
  int channel_;
  Hal::MotorConfig config_;
  int32_t offset_ = 0;      // [counts] software zero (resetPosition)
  float position_ = 0.0f;   // [counts]
  float velocity_ = 0.0f;   // [counts/s]
  float appliedDuty_ = 0.0f;
  bool commanded_ = false;  // reconfigure() guard: never-commanded is safe
  // begin() runs BEFORE the first bus exchange, so driver_.total() is
  // still the constructor's 0 -- capturing the offset there leaves the
  // driver's large cumulative total in the reported position (measured:
  // it pinned at the +-32000 mm wire bound within one session -- doc
  // s2). Defer the zero to the first tick that has real data.
  bool zeroPending_ = false;
  int32_t lastCounts_ = 0;              // [counts] last raw total seen
  int32_t deltaRing_[kDeltaRing] = {};  // [counts] recent per-tick magnitudes
  int deltaRingN_ = 0;
};

}  // namespace Hardware
