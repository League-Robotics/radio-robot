// hiwonder_board.h -- Hardware::HiwonderBoard: the HiWonder 4-channel
// encoder motor driver (I2C 0x34) behind the Hal::MotorBoard
// interface.
//
// NOT WIRED IN YET: nothing constructs this class today (the firmware
// build's recursive source glob compiles it, but no composition root
// instantiates it). Wiring instructions:
// docs/hiwonder/hiwonder-motor-board.md section 3.
//
// Reference documentation -- register map, measured command scales,
// latch behavior, encoder-tick timing, plant characterization, and
// source provenance -- lives at docs/hiwonder/hiwonder-motor-board.md
// (cited below by section as "doc s1.2" etc.). Ported from the
// hiwonder-spike branch, characterized on the vogop rig 2026-08-02.
//
// The board runs its own onboard closed-loop speed control on an
// internal ~9.56 ms tick (doc s1.5). Register map (doc s1.1):
//   0x00 battery ADC (u16 LE, reads directly in mV)
//   0x14 motor type (3 = JGB)          0x15 encoder polarity
//   0x1F fixed PWM (open loop, 4x int8, range -100..100, NON-LATCHING)
//   0x33 fixed speed (closed loop, 4x int8, PULSES PER 10 ms,
//        usable range ~+-50 -- NOT int8 full scale; doc s1.2)
//   0x3C encoder totals (4x int32 LE, auto-incrementing 16-byte read)
#pragma once

#include <cstdint>

#include "platform/i2c_bus.h"
#include "hal/motor_board.h"

namespace Hardware {

class HiwonderBoard : public Hal::MotorBoard {
 public:
  static constexpr uint8_t kAddr = 0x34;
  static constexpr uint8_t kRegBattery = 0x00;
  static constexpr uint8_t kRegMotorType = 0x14;
  static constexpr uint8_t kRegPolarity = 0x15;
  static constexpr uint8_t kRegPwm = 0x1F;
  static constexpr uint8_t kRegSpeed = 0x33;
  static constexpr uint8_t kRegEncoderTotals = 0x3C;
  static constexpr uint8_t kMotorTypeJgb37 = 3;

  // Full scales are the MEASURED usable ranges, not the field widths:
  // commanding int8 127 on 0x33 saturates silently at ~57 pulses/10 ms
  // and invalidated a whole characterization session (doc s1.2).
  static constexpr int kSpeedFullScale = 50;  // [pulses/10 ms]
  static constexpr int kPwmFullScale = 100;   // vendor-documented range

  // The board's OWN encoder-update period -- measured 9.56 ms, not the
  // 10 ms its speed unit implies (doc s1.5). Consumed by BoardMotor's
  // velocity estimator as the tick timebase. Measured on one board at
  // bench temperature; error scales velocity estimates proportionally.
  static constexpr float kEncoderTick = 0.00956f;  // [s]

  explicit HiwonderBoard(Platform::I2CBus& bus) : bus_(bus) {}

  // Hal::Motor type + polarity + all-stop. Returns false when the board
  // never acks (wiring/power) -- callers treat that as "board absent".
  // NOTE a live board with a flat pack acks but will not move: check
  // supplyMillivolts() before debugging anything else (doc s1.6).
  bool init() override { return init(kMotorTypeJgb37); }
  bool init(uint8_t motorType);

  void stageSpeed(int channel, float fraction) override;
  void stagePwm(int channel, float fraction) override;

  // Per-cycle exchange implementing the doc s1.4 command policy:
  // write-on-change, EXCEPT (a) any channel commanded nonzero whose
  // encoder has not advanced since the previous exchange keeps being
  // written every cycle (the from-rest start burst -- a single write
  // never starts the motor, doc s1.3 -- self-sustaining until motion
  // confirms, and it also covers a stalled wheel), and (b) the PWM
  // plane always writes (non-latching). A failed write leaves the
  // change pending and retries next cycle. Then one 16-byte totals
  // read, and a battery read every kBatteryDivisor cycles.
  void exchange(int32_t* totalsOut) override;

  // Totals + battery only, provably no command write (latch testing).
  void readEncoders(int32_t* totalsOut) override;

  int32_t total(int channel) const override { return totals_[channel]; }
  bool connected() const override { return connected_; }
  // The HiWonder ADC register reads directly in millivolts (measured
  // 8053 at rest against a nominally 7.4 V pack).
  uint16_t supplyMillivolts() const override { return battery_; }
  const char* name() const override { return "hiwonder-4ch"; }
  float encoderTick() const override { return kEncoderTick; }  // [s]

 private:
  static constexpr uint32_t kBatteryDivisor = 50;

  void readTotals();

  Platform::I2CBus& bus_;
  bool pwmMode_ = false;
  int8_t staged_[kMaxChannels] = {0, 0, 0, 0};
  int8_t written_[kMaxChannels] = {0, 0, 0, 0};
  int32_t totals_[kMaxChannels] = {0, 0, 0, 0};
  // Per-channel "encoder advanced since the previous exchange" -- the
  // motion confirmation that ends the start burst (doc s1.4).
  bool moved_[kMaxChannels] = {false, false, false, false};
  bool connected_ = false;
  uint16_t battery_ = 0;  // [mV]
  uint32_t cycles_ = 0;
};

}  // namespace Hardware
