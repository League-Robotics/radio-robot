// board_motor.h -- Devices::BoardMotor: one channel of ANY
// Devices::MotorBoard, presented as a Devices::Motor. Board-agnostic by
// construction: everything register-shaped lives in the board class.
//
// NOT WIRED IN YET: nothing constructs this class today. Wiring
// instructions: docs/hiwonder/hiwonder-motor-board.md section 3.
// Reference documentation (cited below as "doc s1.5" etc.): that same
// file. Ported from the hiwonder-spike branch, 2026-08-02.
//
// Because the board runs its own closed loop, setDuty()'s duty IS the
// normalized speed fraction the board wants -- Drive's calibrated
// velocity->duty map stays above this seam unchanged. Note the board's
// own loop makes duty<->speed far closer to linear than the Nezha
// plant; Nezha-tuned trim/PID gains are wrong here (doc s2).
#pragma once

#include <cstdint>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/motor.h"
#include "devices/motor_board.h"

namespace Devices {

class BoardMotor : public Motor {
 public:
  BoardMotor(MotorBoard& board, int channel, const MotorConfig& config)
      : board_(board), channel_(channel), config_(config) {}

  void begin() override;
  void requestSample() override {}  // no split-phase latch on these boards
  void setDuty(float duty) override;  // [-1, 1] -> board speed command
  void setNeutral(Neutral) override;
  void applyTravelCalib(float travelCalib) override {
    config_.wheelTravelCalib = travelCalib;
  }
  [[nodiscard]] bool reconfigure(const MotorConfig& config) override;
  void tick(uint64_t nowUs) override;  // [us]

  float position() const override { return position_; }   // [mm]
  float velocity() const override { return velocity_; }   // [mm/s] signed
  float appliedDuty() const override { return appliedDuty_; }
  bool connected() const override { return board_.connected(); }
  uint64_t sampleTime() const override { return board_.sampleTime(); }

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

  float countsToMm(int32_t counts) const {
    // wheelTravelCalib is [mm/count] on these boards. A Nezha-era
    // mm/deg value in a robot JSON is the WRONG SCALE entirely and
    // produces garbage positions until recalibrated (doc s3 item 4).
    return static_cast<float>(counts) * config_.wheelTravelCalib *
           static_cast<float>(config_.fwdSign);
  }

  MotorBoard& board_;
  int channel_;
  MotorConfig config_;
  int32_t offset_ = 0;      // [counts] software zero (resetPosition)
  float position_ = 0.0f;   // [mm]
  float velocity_ = 0.0f;   // [mm/s]
  float appliedDuty_ = 0.0f;
  bool commanded_ = false;  // reconfigure() guard: never-commanded is safe
  // begin() runs BEFORE the first bus exchange, so board_.total() is
  // still the constructor's 0 -- capturing the offset there leaves the
  // board's large cumulative total in the reported position (measured:
  // it pinned at the +-32000 mm wire bound within one session -- doc
  // s2). Defer the zero to the first tick that has real data.
  bool zeroPending_ = false;
  int32_t lastCounts_ = 0;              // [counts] last raw total seen
  int32_t deltaRing_[kDeltaRing] = {};  // [counts] recent per-tick magnitudes
  int deltaRingN_ = 0;
};

}  // namespace Devices
