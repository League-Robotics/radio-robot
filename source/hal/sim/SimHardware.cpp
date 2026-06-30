#include "SimHardware.h"
#include "types/Config.h"             // RobotConfig, defaultRobotConfig()
#include "types/Inputs.h"       // MotorCommands
#include <cmath>

// ---------------------------------------------------------------------------
// Constructor — own the plant, construct each observation model against it, and
// wire robot geometry so the OTOS sim model integrates correctly.
// ---------------------------------------------------------------------------
SimHardware::SimHardware(const RobotConfig& cfg)
    : _plant()
    , _motorL(_plant, _plant, SimMotor::Side::LEFT)
    , _motorR(_plant, _plant, SimMotor::Side::RIGHT)
    , _odom(_plant)
    , _line(_plant)
    , _color(_plant)
    , _portIO(_plant)
    , _servo()
{
    _trackwidthMm = cfg.trackwidthMm;
    _plant.setTrackwidth(cfg.trackwidthMm);
}

// tick(now) — sensor tick.  Promotes each sim motor's plant reported-encoder
// position into its positionMm() accessor (mirrors MockHAL::tick).  RIGHT before
// LEFT to match the retired MockHAL ordering (immaterial to values — no I2C —
// but kept consistent).  Does NOT integrate the plant; integration happens in
// tick(now,cmds) via advance() (the single integration site).
void SimHardware::tick(uint32_t now_ms) {
    _motorR.tick(now_ms);
    _motorL.tick(now_ms);
}

// tick(now, cmds) — the firmware loop's actuator-state tick.  Drives the ONE
// plant integration step and the OTOS/line/color advances (mirrors MockHAL::tick
// → advance with the bench-OTOS branch removed — there is no bench OTOS in SIM).
void SimHardware::tick(uint32_t now_ms, const MotorCommands& cmds) {
    advance(now_ms, cmds);
}

void SimHardware::advance(uint32_t now_ms, const MotorCommands& cmds) {
    int32_t dt = static_cast<int32_t>(now_ms - _lastTickMs);
    if (dt > 0) {
        uint32_t udt = static_cast<uint32_t>(dt);

        // Compute turn rate from the current PWM commands and feed it to the
        // plant before update() so the reported-encoder slip model sees the
        // correct turn intensity.
        // Array convention: [0]=R (FR), [1]=L (FL) — see OutputState.h.
        float aL = fabsf(static_cast<float>(cmds.pwm[1]));
        float aR = fabsf(static_cast<float>(cmds.pwm[0]));
        float turnRate = (aL + aR > 0.5f)
            ? fabsf(static_cast<float>(cmds.pwm[0] - cmds.pwm[1])) / (aL + aR)
            : 0.0f;
        _plant.setTurnRate(turnRate);

        // ONE ordered integration step.  setActuators uses the SAME rounded PWM
        // the control law produced this tick (cmds.pwm[1]=FL=L, cmds.pwm[0]=FR=R).
        _plant.setActuators(static_cast<int8_t>(cmds.pwm[1]),
                            static_cast<int8_t>(cmds.pwm[0]));
        _plant.update(udt);

        // OTOS sim model: integrate the TRUE (pre-slip) plant velocities into the
        // odometer accumulator (mirrors MockHAL::advance → MockOtosSensor::tick).
        _odom.tick(_plant.trueVelLMms(), _plant.trueVelRMms(), _trackwidthMm, udt);

        // Auxiliary sensor schedules advance on the actuator tick only.
        _line.tick(udt);
        _color.tick(udt);
    }
    _lastTickMs = now_ms;
}
