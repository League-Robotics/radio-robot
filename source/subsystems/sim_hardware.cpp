#include "subsystems/sim_hardware.h"

#include "hal/sim/sim_setters.h"

namespace Subsystems {

SimHardware::SimHardware(const msg::MotorConfig configs[kMotorCount])
    : plant_(),
      motors_{{ {plant_, Hal::SimMotor::Side::LEFT, configs[0]},
                {plant_, Hal::SimMotor::Side::RIGHT, configs[1]},
                Hal::SimMotor(configs[2]),
                Hal::SimMotor(configs[3]) }},
      odometer_(plant_)
{
    for (uint32_t i = 0; i < kMotorCount; ++i) {
        config_[i] = configs[i];   // 087-004: config()'s backing store
    }
}

void SimHardware::begin()
{
    for (uint32_t i = 0; i < kMotorCount; ++i) {
        motors_[i].begin();
    }
}

// The dt=0 re-entry guard (architecture-update.md (081) Decision 4) — see
// file header. A call with an unchanged `now` is a complete no-op: no
// Hal::SimMotor::tick() call, no Hal::PhysicsWorld::update() call, for ANY
// motor. (093/094 teardown) motorIn[]/motorResetIn[] consumption is gone --
// see hardware.h's own tick() doc comment for the full contract.
void SimHardware::tick(uint32_t now)
{
    if (hasAdvanced_ && now == lastAdvancedNow_) {
        return;
    }
    uint32_t dt = hasAdvanced_ ? (now - lastAdvancedNow_) : 0;   // [ms]

    // Every motor ticks first (each Hal::SimMotor samples the plant's STILL-
    // stale reported encoder from the previous pass, decides its new
    // command, and stages it via writeRawDuty()) — THEN the plant advances
    // exactly once with the freshly-staged actuators, and the odometer
    // samples the just-advanced true pose. This one-tick latency mirrors
    // real hardware's own request/collect split (NezhaHardware's brick
    // flip-flop).
    for (uint32_t i = 0; i < kMotorCount; ++i) {
        motors_[i].tick(now);
    }

    plant_.update(dt);
    odometer_.tick(now);

    lastAdvancedNow_ = now;
    hasAdvanced_ = true;
}

Hal::Motor& SimHardware::motor(uint32_t i)
{
    return motors_[clampIndex(i)];
}

void SimHardware::apply(const Hal::CommandProcessorToHardwareCommand& cmd)
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

void SimHardware::apply(const Hal::DrivetrainToHardwareCommand& cmd)
{
    for (int i = 0; i < 2; ++i) {
        motors_[clampIndex(cmd.wheel[i].port)].apply(cmd.wheel[i].command);
    }
}

msg::MotorConfig SimHardware::config(uint32_t i) const
{
    return config_[clampIndex(i)];
}

msg::MotorState SimHardware::state(uint32_t i) const
{
    return motors_[clampIndex(i)].state();
}

Hal::SimMotor& SimHardware::simMotor(uint32_t i)
{
    return motors_[clampIndex(i)];
}

void SimHardware::rebindPlantPorts(uint32_t leftIndex, uint32_t rightIndex)
{
    if (leftIndex == leftIndex_ && rightIndex == rightIndex_) {
        return;   // already bound this way -- no-op
    }
    // Unbind whichever indices currently hold the plant channels, then bind
    // the new pair. If the new pair happens to reuse one of the old
    // indices, that motor's tick cache is rebaselined too (Hal::SimMotor::
    // bindToPlant()'s documented behavior) -- a harmless one-tick velocity-
    // read blip on this rare administrative operation, not a correctness
    // issue (the alternative, skipping the rebaseline for an "unchanged"
    // index, risks a spurious velocity spike whenever the OTHER index's
    // change alone would otherwise warrant one).
    Hal::unbindSimMotorFromPlant(motors_[leftIndex_]);
    Hal::unbindSimMotorFromPlant(motors_[rightIndex_]);

    Hal::bindSimMotorToPlant(motors_[leftIndex], plant_, Hal::SimMotor::Side::LEFT);
    Hal::bindSimMotorToPlant(motors_[rightIndex], plant_, Hal::SimMotor::Side::RIGHT);

    leftIndex_ = leftIndex;
    rightIndex_ = rightIndex;
}

}  // namespace Subsystems
