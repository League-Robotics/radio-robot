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
                                        int writeCount) const {
  for (int i = 0; i < writeCount; ++i) {
    bus.scriptWrite(wireAddr, /*status=*/0);
  }

  // wheelTravelCalib=1.0, fwdSign=+1 convention (matches every existing
  // scriptEncoderRequestCollect()-style helper in this codebase): raw ==
  // position() in tenths of a millimeter, exactly.
  int32_t raw = static_cast<int32_t>(std::lround(position_ * 10.0f));
  uint8_t data[4] = {
      static_cast<uint8_t>(raw & 0xFF),
      static_cast<uint8_t>((raw >> 8) & 0xFF),
      static_cast<uint8_t>((raw >> 16) & 0xFF),
      static_cast<uint8_t>((raw >> 24) & 0xFF),
  };
  bus.scriptRead(wireAddr, data, 4, /*status=*/0);
}

}  // namespace TestSim
