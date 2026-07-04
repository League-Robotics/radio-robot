#include "hal/nezha/nezha_hal.h"

namespace Hal {

NezhaHal::NezhaHal(I2CBus& bus, const msg::MotorConfig configs[kPortCount])
    : motor1_(bus, configs[0]),
      motor2_(bus, configs[1]),
      motor3_(bus, configs[2]),
      motor4_(bus, configs[3])
{
}

void NezhaHal::begin()
{
    motor1_.begin();
    motor2_.begin();
    motor3_.begin();
    motor4_.begin();
}

void NezhaHal::tick(uint32_t now)
{
    // Fixed, deterministic ascending-port order — see nezha_hal.h.
    motor1_.tick(now);
    motor2_.tick(now);
    motor3_.tick(now);
    motor4_.tick(now);
}

Motor& NezhaHal::motor(uint32_t port)
{
    switch (port) {
        case 1: return motor1_;
        case 2: return motor2_;
        case 3: return motor3_;
        default: return motor4_;
    }
}

}  // namespace Hal
