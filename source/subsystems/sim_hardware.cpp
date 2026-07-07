#include "subsystems/sim_hardware.h"

#include "hal/sim/sim_setters.h"

namespace Subsystems {

SimHardware::SimHardware(const msg::MotorConfig configs[kPortCount])
    : plant_(),
      motor1_(plant_, Hal::SimMotor::Side::LEFT, configs[0]),
      motor2_(plant_, Hal::SimMotor::Side::RIGHT, configs[1]),
      motor3_(configs[2]),
      motor4_(configs[3]),
      odometer_(plant_)
{
    for (uint32_t i = 0; i < kPortCount; ++i) {
        config_[i] = configs[i];   // 087-004: config()'s backing store
    }
}

void SimHardware::begin()
{
    motor1_.begin();
    motor2_.begin();
    motor3_.begin();
    motor4_.begin();
}

// The dt=0 re-entry guard (architecture-update.md (081) Decision 4) — see
// file header. A call with an unchanged `now` is a complete no-op: no
// Hal::SimMotor::tick() call, no Hal::PhysicsWorld::update() call, for ANY
// port. 087-004's motorIn[]/motorResetIn[] consumption (below) runs on
// EVERY call, including a re-entrant same-now one -- apply()/
// resetPosition() only stage a command/reset, they do not themselves
// advance the plant, so there is no double-integration hazard here (unlike
// the guard below, which exists specifically to protect Hal::PhysicsWorld::
// update()/Hal::SimMotor::tick() from running twice).
void SimHardware::tick(uint32_t now, Rt::Mailbox<msg::MotorCommand> motorIn[kPortCount],
                        bool motorResetIn[kPortCount])
{
    // 087-004: uniform per-port consumption, no addressed-dispatch branch,
    // no in-use bookkeeping (every port ticks every pass regardless).
    for (uint32_t i = 0; i < kPortCount; ++i) {
        if (!motorIn[i].empty()) {
            motorAt(i + 1).apply(motorIn[i].take());
        }
        if (motorResetIn[i]) {
            motorAt(i + 1).resetPosition();
            motorResetIn[i] = false;   // idempotent -- "reset twice = reset once"
        }
    }

    if (hasAdvanced_ && now == lastAdvancedNow_) {
        return;
    }
    uint32_t dt = hasAdvanced_ ? (now - lastAdvancedNow_) : 0;   // [ms]

    // Every port ticks first (each Hal::SimMotor samples the plant's STILL-
    // stale reported encoder from the previous pass, decides its new
    // command, and stages it via writeRawDuty()) — THEN the plant advances
    // exactly once with the freshly-staged actuators, and the odometer
    // samples the just-advanced true pose. This one-tick latency mirrors
    // real hardware's own request/collect split (NezhaHardware's brick
    // flip-flop).
    motor1_.tick(now);
    motor2_.tick(now);
    motor3_.tick(now);
    motor4_.tick(now);

    plant_.update(dt);
    odometer_.tick(now);

    lastAdvancedNow_ = now;
    hasAdvanced_ = true;
}

Hal::Motor& SimHardware::motor(uint32_t port)
{
    return motorAt(port);
}

void SimHardware::apply(const Hal::CommandProcessorToHardwareCommand& cmd)
{
    if (cmd.allPorts) {
        for (uint32_t p = 1; p <= kPortCount; ++p) {
            motorAt(p).apply(cmd.addressed[0].command);
        }
        return;
    }
    for (uint8_t i = 0; i < cmd.count; ++i) {
        motorAt(cmd.addressed[i].port).apply(cmd.addressed[i].command);
    }
}

void SimHardware::apply(const Hal::DrivetrainToHardwareCommand& cmd)
{
    for (int i = 0; i < 2; ++i) {
        motorAt(cmd.wheel[i].port).apply(cmd.wheel[i].command);
    }
}

msg::MotorConfig SimHardware::config(uint32_t port) const
{
    switch (port) {
        case 1: return config_[0];
        case 2: return config_[1];
        case 3: return config_[2];
        default: return config_[3];   // out-of-range clamps to port 4 -- mirrors motorAt()'s own convention
    }
}

msg::MotorState SimHardware::state(uint32_t port) const
{
    switch (port) {
        case 1: return motor1_.state();
        case 2: return motor2_.state();
        case 3: return motor3_.state();
        default: return motor4_.state();   // out-of-range clamps to port 4 -- mirrors motorAt()'s own convention
    }
}

Hal::SimMotor& SimHardware::simMotor(uint32_t port)
{
    return motorAt(port);
}

void SimHardware::rebindPlantPorts(uint32_t leftPort, uint32_t rightPort)
{
    if (leftPort == leftPort_ && rightPort == rightPort_) {
        return;   // already bound this way -- no-op
    }
    // Unbind whichever ports currently hold the plant channels, then bind
    // the new pair. If the new pair happens to reuse one of the old ports,
    // that motor's tick cache is rebaselined too (Hal::SimMotor::
    // bindToPlant()'s documented behavior) -- a harmless one-tick velocity-
    // read blip on this rare administrative operation, not a correctness
    // issue (the alternative, skipping the rebaseline for an "unchanged"
    // port, risks a spurious velocity spike whenever the OTHER port's
    // change alone would otherwise warrant one).
    Hal::unbindSimMotorFromPlant(motorAt(leftPort_));
    Hal::unbindSimMotorFromPlant(motorAt(rightPort_));

    Hal::bindSimMotorToPlant(motorAt(leftPort), plant_, Hal::SimMotor::Side::LEFT);
    Hal::bindSimMotorToPlant(motorAt(rightPort), plant_, Hal::SimMotor::Side::RIGHT);

    leftPort_ = leftPort;
    rightPort_ = rightPort;
}

Hal::SimMotor& SimHardware::motorAt(uint32_t port)
{
    switch (port) {
        case 1: return motor1_;
        case 2: return motor2_;
        case 3: return motor3_;
        default: return motor4_;
    }
}

}  // namespace Subsystems
