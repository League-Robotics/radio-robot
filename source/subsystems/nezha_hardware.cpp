#include "subsystems/nezha_hardware.h"

namespace Subsystems {

NezhaHardware::NezhaHardware(I2CBus& bus, const msg::MotorConfig configs[kPortCount],
                              const Config::OtosBootConfig& otosConfig)
    : bus_(bus),
      motor1_(bus, configs[0]),
      motor2_(bus, configs[1]),
      motor3_(bus, configs[2]),
      motor4_(bus, configs[3]),
      otosOdometer_(bus, otosConfig)
{
    for (uint32_t i = 0; i < kPortCount; ++i) {
        config_[i] = configs[i];   // 087-004: config()'s backing store
    }
}

void NezhaHardware::begin()
{
    motor1_.begin();
    motor2_.begin();
    motor3_.begin();
    motor4_.begin();
    otosOdometer_.begin();
}

// The brick flip-flop sequencer — implemented exactly per architecture-
// update.md's "The flip-flop and the 078 base-class contract" code block,
// with 087-004's motorIn[]/motorResetIn[] consumption folded in at the top
// (see hardware.h's own tick() doc comment and this class's own tick() doc
// comment for the full contract).
void NezhaHardware::tick(uint32_t now, Rt::Mailbox<msg::MotorCommand> motorIn[kPortCount],
                          bool motorResetIn[kPortCount])
{
    // 087-004: uniform per-port consumption, no addressed-dispatch branch --
    // drained BEFORE the flip-flop's own scheduling decision below, so a
    // port newly brought in-use by this call's motorIn[] is eligible for
    // the SAME call's bus action.
    for (uint32_t i = 0; i < kPortCount; ++i) {
        if (!motorIn[i].empty()) {
            uint32_t port = i + 1;
            motorAt(port).apply(motorIn[i].take());
            portInUse_[i] = true;   // brings the port into the flip-flop schedule (decision 1)
        }
        if (motorResetIn[i]) {
            motorAt(i + 1).resetPosition();
            motorResetIn[i] = false;   // idempotent -- "reset twice = reset once"
        }
    }

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
            if (!bus_.clear(Hal::kNezhaDeviceAddr)) return;   // settle window still open -- pass
            motorAt(activePort_).tick(now);              // the 5-step contract (base/leaf split)
            activePort_ = nextPortInUse(activePort_);
            phase_ = Phase::REQUEST_DUE;
            break;
    }
}

Hal::Motor& NezhaHardware::motor(uint32_t port)
{
    return motorAt(port);
}

void NezhaHardware::apply(const Hal::CommandProcessorToHardwareCommand& cmd)
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

void NezhaHardware::apply(const Hal::DrivetrainToHardwareCommand& cmd)
{
    for (int i = 0; i < 2; ++i) {
        portInUse_[cmd.wheel[i].port - 1] = true;
        motorAt(cmd.wheel[i].port).apply(cmd.wheel[i].command);
    }
}

msg::MotorConfig NezhaHardware::config(uint32_t port) const
{
    switch (port) {
        case 1: return config_[0];
        case 2: return config_[1];
        case 3: return config_[2];
        default: return config_[3];   // out-of-range clamps to port 4 -- mirrors motorAt()'s own convention
    }
}

msg::MotorState NezhaHardware::state(uint32_t port) const
{
    switch (port) {
        case 1: return motor1_.state();
        case 2: return motor2_.state();
        case 3: return motor3_.state();
        default: return motor4_.state();   // out-of-range clamps to port 4 -- mirrors motorAt()'s own convention
    }
}

Hal::Odometer* NezhaHardware::odometer()
{
    return &otosOdometer_;
}

Hal::NezhaMotor& NezhaHardware::motorAt(uint32_t port)
{
    switch (port) {
        case 1: return motor1_;
        case 2: return motor2_;
        case 3: return motor3_;
        default: return motor4_;
    }
}

uint32_t NezhaHardware::nextPortInUse(uint32_t cur) const
{
    for (uint32_t i = 1; i <= kPortCount; ++i) {
        uint32_t candidate = ((cur - 1 + i) % kPortCount) + 1;
        if (portInUse_[candidate - 1]) return candidate;
    }
    return cur;   // no in-use port found -- defensive only, see header comment
}

bool NezhaHardware::anyPortInUse() const
{
    for (uint32_t i = 0; i < kPortCount; ++i) {
        if (portInUse_[i]) return true;
    }
    return false;
}

}  // namespace Subsystems
