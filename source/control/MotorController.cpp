#include "MotorController.h"
#include <math.h>

MotorController::MotorController(Motor& left, Motor& right, const RobotConfig& cal)
    : _motorL(left), _motorR(right), _cal(cal),
      _vcL(cal.velKff, cal.velKp, cal.velKi, 60.0f, cal.minWheelMms),
      _vcR(cal.velKff, cal.velKp, cal.velKi, 60.0f, cal.minWheelMms),
      _pid(cal.ratioPidKp, cal.ratioPidKi, cal.ratioPidKd, cal.ratioPidMax),
      _cmdEncStartL(0.0f), _cmdEncStartR(0.0f),
      _cmdRatio(1.0f), _fasterIsRight(false),
      _tgtLMms(0.0f), _tgtRMms(0.0f),
      _prevEncL(0.0f), _prevEncR(0.0f)
{
    gains.kFF     = 0.15f;
    gains.kP      = 0.05f;
    gains.kI      = 0.20f;
    gains.iClamp  = 60.0f;
    gains.kRatio  = 0.01f;
}

void MotorController::setTarget(float leftMms, float rightMms)
{
    _tgtLMms = leftMms;
    _tgtRMms = rightMms;
}

void MotorController::startDriveClean(float leftMms, float rightMms)
{
    _tgtLMms = leftMms;
    _tgtRMms = rightMms;
    _fasterIsRight = (fabsf(rightMms) >= fabsf(leftMms));
    float fasterAbs = _fasterIsRight ? fabsf(rightMms) : fabsf(leftMms);
    float slowerAbs = _fasterIsRight ? fabsf(leftMms)  : fabsf(rightMms);
    _cmdRatio = (slowerAbs > 0.0f) ? (fasterAbs / slowerAbs) : 1.0f;
    _cmdEncStartL = _motorL.readEncoderMmF(_cal);
    _cmdEncStartR = _motorR.readEncoderMmF(_cal);
    _pid.reset();
    _vcL.reset();
    _vcR.reset();
}

void MotorController::startDrive(float leftMms, float rightMms)
{
    _tgtLMms = leftMms;
    _tgtRMms = rightMms;

    bool newFasterIsRight = (fabsf(rightMms) >= fabsf(leftMms));
    float newFasterAbs = newFasterIsRight ? fabsf(rightMms) : fabsf(leftMms);
    float newSlowerAbs = newFasterIsRight ? fabsf(leftMms)  : fabsf(rightMms);
    float newRatio = (newSlowerAbs > 0.0f) ? (newFasterAbs / newSlowerAbs) : 1.0f;

    float curL = _motorL.readEncoderMmF(_cal);
    float curR = _motorR.readEncoderMmF(_cal);
    float curFaster   = newFasterIsRight ? curR : curL;
    float curSlower   = newFasterIsRight ? curL : curR;
    float startFaster = newFasterIsRight ? _cmdEncStartR : _cmdEncStartL;
    float prevDeltaFaster = fabsf(curFaster - startFaster);

    float seedFaster = fmaxf(prevDeltaFaster, newFasterAbs);
    float seedSlower = (newRatio > 0.0f) ? (seedFaster / newRatio) : 0.0f;

    float signFaster = ((newFasterIsRight ? rightMms : leftMms) >= 0.0f) ? 1.0f : -1.0f;
    float signSlower = ((newFasterIsRight ? leftMms  : rightMms) >= 0.0f) ? 1.0f : -1.0f;

    if (newFasterIsRight) {
        _cmdEncStartR = curFaster - signFaster * seedFaster;
        _cmdEncStartL = curSlower - signSlower * seedSlower;
    } else {
        _cmdEncStartL = curFaster - signFaster * seedFaster;
        _cmdEncStartR = curSlower - signSlower * seedSlower;
    }

    if (newFasterIsRight != _fasterIsRight) _pid.reset();
    _fasterIsRight = newFasterIsRight;
    _cmdRatio = newRatio;
}

void MotorController::stop()
{
    _tgtLMms = 0.0f;
    _tgtRMms = 0.0f;
    _pid.reset();
    _vcL.reset();
    _vcR.reset();
    _cmdEncStartL = _motorL.readEncoderMmF(_cal);
    _cmdEncStartR = _motorR.readEncoderMmF(_cal);
    _motorL.setSpeed(0);
    _motorR.setSpeed(0);
}

void MotorController::resetIntegrators()
{
    _pid.reset();
    _vcL.reset();
    _vcR.reset();
}

void MotorController::updatePidGains(float kP, float kI, float kD, float iClamp)
{
    _pid.updateGains(kP, kI, kD, iClamp);
}

void MotorController::controlTick(HardwareState& inputs, MotorCommands& cmds, float dt_s)
{
    if (dt_s <= 0.0f) return;

    // Transitional stub (014-003): mirror the private targets into cmds so the
    // VelocityController reads from the authoritative MotorCommands struct.
    // Ticket 007 removes _tgtLMms/R and has the command processor write
    // cmds.tgtLMms/R directly; at that point this sync block is deleted.
    cmds.tgtLMms = _tgtLMms;
    cmds.tgtRMms = _tgtRMms;

    // Step 1: Read encoder positions (mm) from HardwareState — written by
    // Robot::controlCollect() before this call.
    float encLMm = inputs.encLMm;
    float encRMm = inputs.encRMm;

    // Encoder-delta velocity — sole feedback source (chip readSpeed disabled in 013).
    float encVelL = (encLMm - _prevEncL) / dt_s;
    float encVelR = (encRMm - _prevEncR) / dt_s;
    _prevEncL = encLMm;   // float — no 1 mm truncation (was a velocity-throb source)
    _prevEncR = encRMm;

    // Write derived velocities back to HardwareState.
    inputs.velLMms = encVelL;
    inputs.velRMms = encVelR;

    // If no drive command active, ensure motors are stopped.
    if (cmds.tgtLMms == 0.0f && cmds.tgtRMms == 0.0f) {
        cmds.pwmL = 0;
        cmds.pwmR = 0;
        _motorL.setSpeed(0);
        _motorR.setSpeed(0);
        return;
    }

    // Step 2: Per-wheel velocity PID (Sprint 010 inner loop).
    // VelocityController::update(setpoint, measured, dt_s) → PWM% in [-100, +100].
    float uL = _vcL.update(cmds.tgtLMms, inputs.velLMms, dt_s);
    float uR = _vcR.update(cmds.tgtRMms, inputs.velRMms, dt_s);

    cmds.pwmL = static_cast<int16_t>(roundf(uL));
    cmds.pwmR = static_cast<int16_t>(roundf(uR));
    _motorL.setSpeed(static_cast<int8_t>(roundf(uL)));
    _motorR.setSpeed(static_cast<int8_t>(roundf(uR)));
}

void MotorController::getVelocitySourceFlags(bool& leftChip, bool& rightChip) const
{
    // The chip readSpeed (0x47) path was disabled in sprint 013 (motor throb fix).
    // Encoder-delta is the sole velocity source. Always report false.
    leftChip  = false;
    rightChip = false;
}

void MotorController::getEncoderPositions(int32_t& leftMm, int32_t& rightMm) const
{
    // Read directly from the motor hardware. These are synchronous I2C reads
    // (legacy blocking path, same as before the split-phase API). Used by
    // DriveController for D-mode distance tracking until ticket 007 migrates
    // that path to HardwareState.
    leftMm  = _motorL.readEncoder(_cal);
    rightMm = _motorR.readEncoder(_cal);
}

void MotorController::resetEncoderAccumulators()
{
    _motorL.resetEncoder();
    _motorR.resetEncoder();
    _prevEncL = 0.0f;
    _prevEncR = 0.0f;
}

float MotorController::clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
