#include "SimMotor.h"
#include "types/Config.h"

float SimMotor::reportedEncMm() const {
    return (_side == Side::LEFT) ? _plant.reportedEncLMm()
                                 : _plant.reportedEncRMm();
}

void SimMotor::setSpeed(int8_t pct) {
    _cmdSpeed = pct;
    // Forward to the plant for this one wheel.  The authoritative plant tick in
    // SimHardware uses setActuators(cmds.pwmL, cmds.pwmR) with the SAME rounded
    // PWM (cmds.pwmL == roundf(uL) == this int8_t for |uL| <= 100), so this
    // forward keeps the plant consistent for any single-wheel injection path.
    _mut.setActuator(sideIdx(), pct);
}

int32_t SimMotor::collectEncoder() const {
    return static_cast<int32_t>(reportedEncMm());
}

float SimMotor::readEncoderMmF(const RobotConfig& /*cfg*/) const {
    return reportedEncMm();
}

float SimMotor::readEncoderMmFAtomic(const RobotConfig& /*cfg*/) const {
    return reportedEncMm();
}

float SimMotor::readEncoderMmFSettle(const RobotConfig& /*cfg*/) const {
    return reportedEncMm();
}

void SimMotor::resetEncoder() {
    // Mirror MockMotor::resetEncoder: zero this side's reported accumulator and
    // the cmd speed, then realign the tick() cache so the accessor + outlier
    // baseline stay in lockstep.  The plant's TRUE accumulator (ground truth) is
    // not reset here.
    _mut.resetReportedEncoder(sideIdx());
    _cmdSpeed         = 0;
    _lastPositionMm   = 0.0f;
    _lastVelocityMmps = 0.0f;
    _hasLastTick      = false;
}

void SimMotor::tick(uint32_t now_ms) {
    // Sensor tick — promote the plant's reported encoder into the accessor cache.
    // COPY only (no re-integration); bit-identical to MockMotor::tick.  A simple
    // position-difference velocity is cached for velocityMmps() (not consumed by
    // the PID, so it does not affect the golden-TLM frame).
    if (_frozen) {
        // Frozen encoder: hold the last cached value, do NOT advance the
        // timestamp baseline (so velocity differentiation resumes cleanly on
        // unfreeze).  Forward-compat: default-off, so MockMotor parity holds.
        return;
    }
    float pos = reportedEncMm();
    if (_hasLastTick) {
        float elapsed_s = static_cast<float>(now_ms - _lastTickMs) / 1000.0f;
        if (elapsed_s > 0.0f) {
            _lastVelocityMmps = (pos - _lastPositionMm) / elapsed_s;
        }
    } else {
        _hasLastTick = true;
    }
    _lastPositionMm = pos;
    _lastTickMs     = now_ms;
}

void SimMotor::setNoiseSigma(float sigmaMm) {
    _mut.setEncoderNoise(sideIdx(), sigmaMm);
}
