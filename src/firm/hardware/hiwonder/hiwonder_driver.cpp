// hiwonder_driver.cpp -- see hiwonder_driver.h; reference documentation at
// docs/hiwonder/hiwonder-motor-board.md (cited as "doc s1.2" etc.).
#include "hardware/hiwonder/hiwonder_driver.h"

#include <cmath>
#include <cstring>

namespace Hardware {

bool HiwonderDriver::init(uint8_t motorType) {
  uint8_t typeFrame[2] = {kRegMotorType, motorType};
  int rc = bus_.write(kAddr << 1, typeFrame, sizeof(typeFrame));
  uint8_t polFrame[2] = {kRegPolarity, 0};
  rc |= bus_.write(kAddr << 1, polFrame, sizeof(polFrame));
  uint8_t stopFrame[5] = {kRegSpeed, 0, 0, 0, 0};
  rc |= bus_.write(kAddr << 1, stopFrame, sizeof(stopFrame));
  connected_ = (rc == 0);
  for (int i = 0; i < kMaxChannels; ++i) {
    staged_[i] = 0;
    written_[i] = 0;
    moved_[i] = false;
  }
  return connected_;
}

void HiwonderDriver::stageSpeed(int channel, float fraction) {
  if (channel < 0 || channel >= kMaxChannels) return;
  if (fraction > 1.0f) fraction = 1.0f;
  if (fraction < -1.0f) fraction = -1.0f;
  // 0x33 is PULSES PER 10 ms with a usable range of ~+-50, NOT int8
  // full scale -- commanding 127 saturates silently (doc s1.2).
  staged_[channel] =
      static_cast<int8_t>(std::lround(fraction * kSpeedFullScale));
  pwmMode_ = false;
}

void HiwonderDriver::stagePwm(int channel, float fraction) {
  if (channel < 0 || channel >= kMaxChannels) return;
  if (fraction > 1.0f) fraction = 1.0f;
  if (fraction < -1.0f) fraction = -1.0f;
  // Vendor range is -100..100, not int8 full scale (doc s1.2).
  staged_[channel] =
      static_cast<int8_t>(std::lround(fraction * kPwmFullScale));
  pwmMode_ = true;
}

void HiwonderDriver::readTotals() {
  // One auto-incrementing 16-byte read returns all four totals (this
  // board auto-increments; its Yahboom sibling does not). Motion
  // confirmation for the doc s1.4 write policy falls out of the same
  // read: a channel "moved" iff its total changed.
  uint8_t reg = kRegEncoderTotals;
  uint8_t raw[16];
  if (bus_.write(kAddr << 1, &reg, 1, true) == 0 &&
      bus_.read(kAddr << 1, raw, sizeof(raw)) == 0) {
    for (int i = 0; i < kMaxChannels; ++i) {
      int32_t v;
      std::memcpy(&v, raw + 4 * i, 4);
      moved_[i] = (v != totals_[i]);
      totals_[i] = v;
    }
  } else {
    connected_ = false;
  }
}

void HiwonderDriver::exchange(int32_t* totalsOut) {
  // Command write policy (doc s1.4, from the measured latch behavior in
  // doc s1.3):
  //   - speed commands LATCH while the motor is moving, so a steady
  //     cruise needs no command traffic at all;
  //   - a change is written once (and retried until a write succeeds);
  //   - a channel commanded nonzero that has not moved since the last
  //     exchange keeps being written every cycle -- a single write
  //     NEVER starts a motor from rest (measured: 1 write never, 2-5
  //     flaky, 20+ reliable); writing until the encoder confirms motion
  //     is the reliable form of that burst, and it also covers a
  //     stalled wheel;
  //   - the PWM plane is non-latching and always writes.
  bool needWrite = pwmMode_ ||
                   std::memcmp(staged_, written_, sizeof(staged_)) != 0;
  if (!needWrite) {
    for (int i = 0; i < kMaxChannels; ++i) {
      if (staged_[i] != 0 && !moved_[i]) {
        needWrite = true;
        break;
      }
    }
  }
  if (needWrite) {
    uint8_t frame[5] = {static_cast<uint8_t>(pwmMode_ ? kRegPwm : kRegSpeed),
                        static_cast<uint8_t>(staged_[0]),
                        static_cast<uint8_t>(staged_[1]),
                        static_cast<uint8_t>(staged_[2]),
                        static_cast<uint8_t>(staged_[3])};
    if (bus_.write(kAddr << 1, frame, sizeof(frame)) == 0) {
      std::memcpy(written_, staged_, sizeof(staged_));
      connected_ = true;
    } else {
      connected_ = false;  // change stays pending; retried next cycle
    }
  } else {
    connected_ = true;  // refreshed by the totals read below regardless
  }

  readTotals();

  // Supply reading on a divisor -- it moves slowly and costs a second
  // transaction, so it must never ride the per-cycle path.
  if (++cycles_ % kBatteryDivisor == 0) {
    uint8_t breg = kRegBattery;
    uint8_t braw[2];
    if (bus_.write(kAddr << 1, &breg, 1, true) == 0 &&
        bus_.read(kAddr << 1, braw, sizeof(braw)) == 0) {
      battery_ = static_cast<uint16_t>(braw[0] | (braw[1] << 8));
    }
  }

  if (totalsOut != nullptr) std::memcpy(totalsOut, totals_, sizeof(totals_));
}

void HiwonderDriver::readEncoders(int32_t* totalsOut) {
  readTotals();
  if (totalsOut != nullptr) std::memcpy(totalsOut, totals_, sizeof(totals_));
}

}  // namespace Hardware
