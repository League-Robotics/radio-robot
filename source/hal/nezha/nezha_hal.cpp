#include "hal/nezha/nezha_hal.h"

namespace Hal {

NezhaHal::NezhaHal(I2CBus& bus, const msg::MotorConfig configs[kPortCount])
    : bus_(bus),
      motor1_(bus, configs[0]),
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

// The brick flip-flop sequencer — implemented exactly per architecture-
// update.md's "The flip-flop and the 078 base-class contract" code block.
void NezhaHal::tick(uint32_t now)
{
    if (!anyPortInUse()) return;                    // idle schedule (decision 1)
    if (!portInUse_[activePort_ - 1]) {
        activePort_ = nextPortInUse(activePort_);    // defensive resync
    }
    switch (phase_) {
        case Phase::REQUEST_DUE:
            motorAt(activePort_).requestSample();    // 0x46 write, postClear=4000 [us]
            phase_ = Phase::COLLECT_DUE;
            break;
        case Phase::COLLECT_DUE:
            if (!bus_.clear(kNezhaDeviceAddr)) return;   // settle window still open -- pass
            motorAt(activePort_).tick(now);              // the 5-step contract (base/leaf split)
            activePort_ = nextPortInUse(activePort_);
            phase_ = Phase::REQUEST_DUE;
            break;
    }
}

Motor& NezhaHal::motor(uint32_t port)
{
    return motorAt(port);
}

void NezhaHal::apply(const CommandProcessorToHalCommand& cmd)
{
    if (cmd.allPorts) {
        for (uint32_t p = 1; p <= kPortCount; ++p) {
            motorAt(p).apply(cmd.addressed[0].command);
        }
        return;   // broadcast never marks a port in-use -- see Design Rationale 5
    }
    for (uint8_t i = 0; i < cmd.count; ++i) {
        portInUse_[cmd.addressed[i].port - 1] = true;
        motorAt(cmd.addressed[i].port).apply(cmd.addressed[i].command);
    }
}

void NezhaHal::apply(const DrivetrainToHalCommand& cmd)
{
    for (int i = 0; i < 2; ++i) {
        portInUse_[cmd.wheel[i].port - 1] = true;
        motorAt(cmd.wheel[i].port).apply(cmd.wheel[i].command);
    }
}

NezhaMotor& NezhaHal::motorAt(uint32_t port)
{
    switch (port) {
        case 1: return motor1_;
        case 2: return motor2_;
        case 3: return motor3_;
        default: return motor4_;
    }
}

uint32_t NezhaHal::nextPortInUse(uint32_t cur) const
{
    for (uint32_t i = 1; i <= kPortCount; ++i) {
        uint32_t candidate = ((cur - 1 + i) % kPortCount) + 1;
        if (portInUse_[candidate - 1]) return candidate;
    }
    return cur;   // no in-use port found -- defensive only, see header comment
}

bool NezhaHal::anyPortInUse() const
{
    for (uint32_t i = 0; i < kPortCount; ++i) {
        if (portInUse_[i]) return true;
    }
    return false;
}

}  // namespace Hal
