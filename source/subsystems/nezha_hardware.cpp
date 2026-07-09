#include "subsystems/nezha_hardware.h"

namespace Subsystems {

NezhaHardware::NezhaHardware(I2CBus& bus, const msg::MotorConfig configs[kMotorCount],
                              const Config::OtosBootConfig& otosConfig)
    : bus_(bus),
      motors_{{ {bus, configs[0]}, {bus, configs[1]}, {bus, configs[2]}, {bus, configs[3]} }},
      otosOdometer_(bus, otosConfig)
{
    for (uint32_t i = 0; i < kMotorCount; ++i) {
        config_[i] = configs[i];   // 087-004: config()'s backing store
        polled_[i] = configs[i].polled;   // 091-002: the configured poll-set, established ONCE here
    }
}

void NezhaHardware::begin()
{
    for (uint32_t i = 0; i < kMotorCount; ++i) {
        motors_[i].begin();
    }
    otosOdometer_.begin();
}

// The brick flip-flop sequencer — implemented exactly per architecture-
// update.md's "The flip-flop and the 078 base-class contract" code block.
// (093/094 teardown) motorIn[]/motorResetIn[] consumption is gone -- see
// hardware.h's own tick() doc comment for the full contract; this class's
// tick() now runs ONLY the flip-flop's own scheduling decision below.
void NezhaHardware::tick(uint32_t now)
{
    if (!anyPolled()) return;                    // idle schedule (decision 1)
    if (!polled_[activeIndex_]) {
        activeIndex_ = nextPolled(activeIndex_);  // defensive resync
    }
    switch (phase_) {
        case Phase::REQUEST_DUE:
            motors_[activeIndex_].requestSample();    // 0x46 write, postClear=4000 [us]
            phase_ = Phase::COLLECT_DUE;
            break;
        case Phase::COLLECT_DUE:
            if (!bus_.clear(Hal::kNezhaDeviceAddr)) return;   // settle window still open -- pass
            motors_[activeIndex_].tick(now);              // the 5-step contract (base/leaf split)
            activeIndex_ = nextPolled(activeIndex_);
            phase_ = Phase::REQUEST_DUE;
            break;
    }
}

Hal::Motor& NezhaHardware::motor(uint32_t i)
{
    return motors_[clampIndex(i)];
}

void NezhaHardware::apply(const Hal::CommandProcessorToHardwareCommand& cmd)
{
    if (cmd.allPorts) {
        for (uint32_t i = 0; i < kMotorCount; ++i) {
            motors_[i].apply(cmd.addressed[0].command);
        }
        return;
    }
    for (uint8_t i = 0; i < cmd.count; ++i) {
        motors_[clampIndex(cmd.addressed[i].port)].apply(cmd.addressed[i].command);
    }
}

void NezhaHardware::apply(const Hal::DrivetrainToHardwareCommand& cmd)
{
    for (int i = 0; i < 2; ++i) {
        motors_[clampIndex(cmd.wheel[i].port)].apply(cmd.wheel[i].command);
    }
}

msg::MotorConfig NezhaHardware::config(uint32_t i) const
{
    return config_[clampIndex(i)];
}

msg::MotorState NezhaHardware::state(uint32_t i) const
{
    return motors_[clampIndex(i)].state();
}

Hal::Odometer* NezhaHardware::odometer()
{
    return &otosOdometer_;
}

uint32_t NezhaHardware::nextPolled(uint32_t cur) const
{
    for (uint32_t step = 1; step <= kMotorCount; ++step) {
        uint32_t candidate = (cur + step) % kMotorCount;
        if (polled_[candidate]) return candidate;
    }
    return cur;   // no polled motor found -- defensive only, see header comment
}

bool NezhaHardware::anyPolled() const
{
    for (uint32_t i = 0; i < kMotorCount; ++i) {
        if (polled_[i]) return true;
    }
    return false;
}

void NezhaHardware::setPolled(uint32_t i, bool polled)
{
    polled_[clampIndex(i)] = polled;
}

}  // namespace Subsystems
