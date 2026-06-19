#include "MockHAL.h"
#include "control/RobotState.h"   // MotorCommands (commanded wheel velocity)
#include <cmath>

// tick(now) — sensor tick (039-002).  Promotes each mock motor's integrated
// encoder position into its positionMm() accessor (mirrors NezhaHAL::tick, which
// drives the real split-phase read).  RIGHT before LEFT to match NezhaHAL — the
// mock ordering is immaterial to values (no I2C) but kept consistent.
//
// This does NOT integrate the plant; integration happens in tick(now,cmds) via
// advance() → MockMotor::integrate (the single integration site, OQ-2).  The
// old advance(now,nullptr) plant-integration on this path is removed: the sim
// now calls hal.tick(now,cmds) (plant) then hal.tick(now) (sensor) each step,
// and loopTickOnce also calls hal.tick(now,cmds) — the dt==0 guard already made
// repeat plant integrations no-ops.
void MockHAL::tick(uint32_t now_ms) {
    _motorR.tick(now_ms);
    _motorL.tick(now_ms);
}

// tick(now, cmds) — the firmware loop's actuator-state tick.  Drives the same
// plant AND, when bench mode is active, feeds the BenchOtosSensor the commanded
// wheel velocity (mirrors NezhaHAL::tick).
void MockHAL::tick(uint32_t now_ms, const MotorCommands& cmds) {
    advance(now_ms, &cmds);
}

void MockHAL::advance(uint32_t now_ms, const MotorCommands* cmds) {
    int32_t dt = static_cast<int32_t>(now_ms - _lastTickMs);
    if (dt > 0) {
        uint32_t udt = static_cast<uint32_t>(dt);

        // Compute turn rate from current motor commands and feed to each motor
        // before ticking so the slip model sees the correct turn intensity.
        float aL = fabsf(static_cast<float>(_motorL.cmdSpeed()));
        float aR = fabsf(static_cast<float>(_motorR.cmdSpeed()));
        float turnRate = (aL + aR > 0.5f)
            ? fabsf(static_cast<float>(_motorR.cmdSpeed() - _motorL.cmdSpeed())) / (aL + aR)
            : 0.0f;
        _motorL.setTurnRate(turnRate);
        _motorR.setTurnRate(turnRate);

        _motorL.integrate(udt);
        _motorR.integrate(udt);

        // Update oracle ground-truth pose from pre-slip true velocities.
        if (_trackwidthMm > 0.0f) {
            _exactPose.update(
                _motorL.trueVelocityMms(),
                _motorR.trueVelocityMms(),
                _trackwidthMm,
                udt);
        }

        _otos.tick(_motorL.trueVelocityMms(), _motorR.trueVelocityMms(), _trackwidthMm, udt);

        // Bench OTOS: when active, integrate the COMMANDED wheel velocity into
        // the synthetic pose — the same device + input the firmware uses on
        // hardware.  otos() returns this sensor while bench mode is on, so
        // Robot::otosCorrect() fuses it into the EKF exactly as on the bench.
        if (cmds != nullptr && _trackwidthMm > 0.0f &&
                _otosActive == static_cast<IOdometer*>(&_benchOtos)) {
            _benchOtos.tick(cmds->tgtLMms, cmds->tgtRMms, _trackwidthMm, udt);
        }

        _line.tick(udt);
        _color.tick(udt);
    }
    _lastTickMs = now_ms;
}
