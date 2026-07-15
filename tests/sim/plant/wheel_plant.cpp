#include "wheel_plant.h"

#include <cmath>

namespace TestSim {

WheelPlant::WheelPlant(float dutyVelMax, float tau)
    : dutyVelMax_(dutyVelMax), tau_(tau) {}

void WheelPlant::step(float appliedDuty, float dt) {
  // Exact discretization of the continuous first-order lag
  // dv/dt = (target - v) / tau_ over one step of length dt -- see
  // wheel_plant.h's own comment for the derivation.
  float target = dutyVelMax_ * appliedDuty;
  float alpha = 1.0f - std::exp(-dt / tau_);
  velocity_ += (target - velocity_) * alpha;
  position_ += velocity_ * dt;
}

void WheelPlant::scriptEncoderResponse(Devices::I2CBus& bus, uint16_t wireAddr,
                                        int writeCount) {
  int status = disconnected_ ? kNakStatus : 0;
  for (int i = 0; i < writeCount; ++i) {
    bus.scriptWrite(wireAddr, status);
  }

  // Fault-knob precedence: freeze wins outright (an explicitly frozen
  // reading is never itself subject to dropout-driven staleness -- there is
  // nothing "fresher" to fall back to while frozen). Otherwise the dropout
  // accumulator decides fresh-vs-held for this call. Neither knob ever
  // touches position_/velocity_ -- step()'s own integration is untouched.
  float reportPosition;
  if (freezePosition_) {
    reportPosition = frozenPosition_;
  } else if (dropoutRate_ > 0.0f) {
    dropoutAccum_ += dropoutRate_;
    if (dropoutAccum_ >= 1.0f) {
      dropoutAccum_ -= 1.0f;
      reportPosition = lastReportedPosition_;   // hold: stale-not-fresh
    } else {
      reportPosition = position_;
    }
  } else {
    reportPosition = position_;
  }
  lastReportedPosition_ = reportPosition;

  // wheelTravelCalib=1.0, fwdSign=+1 convention (matches every existing
  // scriptEncoderRequestCollect()-style helper in this codebase): raw ==
  // reportPosition in tenths of a millimeter, exactly.
  int32_t raw = static_cast<int32_t>(std::lround(reportPosition * 10.0f));
  uint8_t data[4] = {
      static_cast<uint8_t>(raw & 0xFF),
      static_cast<uint8_t>((raw >> 8) & 0xFF),
      static_cast<uint8_t>((raw >> 16) & 0xFF),
      static_cast<uint8_t>((raw >> 24) & 0xFF),
  };
  bus.scriptRead(wireAddr, data, 4, status);
}

void WheelPlant::freezePosition(bool freeze) {
  if (freeze && !freezePosition_) {
    frozenPosition_ = position_;   // capture only on the rising edge
  }
  freezePosition_ = freeze;
}

void WheelPlant::setDropoutRate(float fraction) {
  dropoutRate_ = fraction;
  dropoutAccum_ = 0.0f;   // fresh phase -- a rate change never inherits a stale one
}

}  // namespace TestSim
