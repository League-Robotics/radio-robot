// motor_board.h -- Devices::MotorBoard: the interface every multi-channel
// smart motor-driver board implements.
//
// NOT WIRED IN YET: nothing constructs a MotorBoard today. Wiring
// instructions: docs/hiwonder/hiwonder-motor-board.md section 3.
// Reference documentation for the whole family: that same file.
//
// Candidate boards (HiWonder 4-channel encoder driver, Yahboom quad
// encoder module, others) all do the same job through different
// registers: take per-channel speed commands, run their OWN closed
// loop, and report cumulative encoder counts. This interface is exactly
// that contract, so swapping boards is one new implementation class
// plus one line in the composition root -- no change to
// Devices::BoardMotor, App::Drive, the planner, or anything above.
//
// Deliberately NOT in the interface: register maps, I2C addresses,
// motor-type configuration, unit scaling. Those are each board's own
// business. The one shape this interface fixes is the COMMAND: a
// normalized fraction of the board's full-scale speed, because that is
// the only unit every candidate board agrees on.
#pragma once

#include <cstdint>

namespace Devices {

class MotorBoard {
 public:
  static constexpr int kMaxChannels = 4;

  virtual ~MotorBoard() = default;

  // Configure and zero the board. False means the board never acked --
  // callers treat that as "absent" and must not drive.
  virtual bool init() = 0;

  // Stage one channel's speed command as a NORMALIZED FRACTION of this
  // board's full-scale closed-loop command, -1.0..+1.0. Takes effect on
  // the next exchange(); staging is free.
  //
  // Normalized, not raw, because the candidate boards disagree wildly
  // on the native unit: HiWonder takes int8 pulses/10ms (usable ~+-50),
  // Yahboom takes int16 BE mm/s (+-1000). A raw int8 interface would
  // silently clip Yahboom to 12% of its range. Each board maps this
  // fraction onto its own encoding.
  virtual void stageSpeed(int channel, float fraction) = 0;

  // Stage an OPEN-LOOP command (raw PWM), same normalized -1..1 range.
  // Bypasses the board's own speed controller entirely -- the
  // comparison that proves whether the closed loop is doing anything.
  virtual void stagePwm(int channel, float fraction) = 0;

  // THE per-cycle bus transaction: flush staged commands per the
  // board's write policy, pull every channel's cumulative encoder
  // count. One call per control cycle, whose cost is the loop's real
  // floor. `totalsOut` may be null.
  virtual void exchange(int32_t* totalsOut) = 0;

  // Encoder/battery read WITHOUT any command write, for latch testing:
  // proving whether a command persists requires observing the plant
  // while provably not re-sending it (see
  // docs/hiwonder/hiwonder-motor-board.md section 1.3 for what that
  // test found). Default falls back to a full exchange for boards that
  // have not implemented the split.
  virtual void readEncoders(int32_t* totalsOut) { exchange(totalsOut); }

  // Last-read cumulative count for a channel (served from the cache the
  // most recent exchange() filled -- never its own bus traffic).
  virtual int32_t total(int channel) const = 0;

  virtual bool connected() const = 0;

  // Cycle-guarded exchange: the FIRST caller in a given cycle pays the
  // bus, every later caller in the same cycle is a no-op served from
  // the cache. Concrete on purpose -- both wheels call it and neither
  // should have to know which one is first.
  void exchangeOncePerCycle(uint64_t nowUs) {  // [us]
    if (nowUs == lastExchangeUs_ && lastExchangeUs_ != 0) return;
    lastExchangeUs_ = nowUs;
    exchange(nullptr);
  }

  // [us] timestamp of the most recent exchangeOncePerCycle().
  uint64_t sampleTime() const { return lastExchangeUs_; }

  // [s] The board's OWN encoder-update period -- the timebase for any
  // velocity estimate derived from total(). These boards refresh their
  // readable totals on an internal tick that is NOT the host's loop
  // period and NOT necessarily the nominal 10 ms (HiWonder measures
  // 9.56 ms; Yahboom measures 10.00 ms) -- see
  // docs/hiwonder/hiwonder-motor-board.md section 1.5 for why dividing
  // count deltas by host time instead produces phantom velocity
  // spikes.
  virtual float encoderTick() const { return 0.010f; }

  // Supply reading in MILLIVOLTS (a real unit, not each board's raw
  // register: HiWonder reports a u16 LE ADC already in mV, Yahboom a
  // u16 BE in tenths of a volt). 0 when the board cannot report one.
  // Boards that DO expose it make the Pybricks-style battery-normalized
  // command possible -- see
  // .clasi/knowledge/pybricks-motion-control-study.md.
  virtual uint16_t supplyMillivolts() const { return 0; }

  // Human-readable board identity for the boot banner / telemetry.
  virtual const char* name() const = 0;

 private:
  uint64_t lastExchangeUs_ = 0;  // [us] cycle guard
};

}  // namespace Devices
