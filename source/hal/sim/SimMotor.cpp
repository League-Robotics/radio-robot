#include "SimMotor.h"
#include "types/Config.h"

float SimMotor::reportedEnc() const {
    return (_side == Side::LEFT) ? _plant.reportedEncL()
                                 : _plant.reportedEncR();
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
    // (064-005) Injected read failure: hold the last tick()-cached value
    // instead of a live plant read, mirroring the real Motor's hold-last
    // -value fix (CR-03).
    if (_readFailure) return static_cast<int32_t>(_lastPosition);
    return static_cast<int32_t>(reportedEnc());
}

float SimMotor::readEncoderMmF(const RobotConfig& /*cfg*/) const {
    if (_readFailure) return _lastPosition;
    return reportedEnc();
}

float SimMotor::readEncoderMmFAtomic(const RobotConfig& /*cfg*/) const {
    if (_readFailure) return _lastPosition;
    return reportedEnc();
}

float SimMotor::readEncoderMmFSettle(const RobotConfig& /*cfg*/) const {
    if (_readFailure) return _lastPosition;
    return reportedEnc();
}

void SimMotor::resetEncoder() {
    // Mirror MockMotor::resetEncoder: zero this side's reported accumulator and
    // the cmd speed, then realign the tick() cache so the accessor + outlier
    // baseline stay in lockstep.  The plant's TRUE accumulator (ground truth) is
    // not reset here.
    _mut.resetReportedEncoder(sideIdx());
    _cmdSpeed         = 0;
    _lastPosition   = 0.0f;
    _lastVelocityMmps = 0.0f;
    _hasLastTick      = false;
    ++_hardResetCount;
}

void SimMotor::rebaselineSoft() {
    // (064-003) The sim has no I2C timing race to avoid (no real bus, no
    // atomic-read burst to latch), so this performs the SAME effect
    // resetEncoder() already does above — zero the reported accumulator.
    // Only the reset-kind counter differs (_softResetCount, not
    // _hardResetCount), so a full-pipeline sim test can verify the at-rest
    // DECISION made by MotorController::resetEncoderAccumulators() (which
    // path was taken) without any behavioral difference in the resulting
    // encoder position.
    _mut.resetReportedEncoder(sideIdx());
    _cmdSpeed         = 0;
    _lastPosition   = 0.0f;
    _lastVelocityMmps = 0.0f;
    _hasLastTick      = false;
    ++_softResetCount;
}

void SimMotor::tick(uint32_t now_ms) {
    // Sensor tick — promote the plant's reported encoder into the accessor cache.
    // COPY only (no re-integration); bit-identical to MockMotor::tick.  A simple
    // position-difference velocity is cached for velocityMmps() (not consumed by
    // the PID, so it does not affect the golden-TLM frame).
    if (_frozen || _readFailure) {
        // Frozen encoder / injected read failure (064-005): hold the last
        // cached value, do NOT advance the timestamp baseline (so velocity
        // differentiation resumes cleanly once unfrozen/cleared).
        // Forward-compat: both default-off, so MockMotor parity holds.
        return;
    }
    float pos = reportedEnc();
    if (_hasLastTick) {
        float elapsed_s = static_cast<float>(now_ms - _lastTickMs) / 1000.0f;
        if (elapsed_s > 0.0f) {
            _lastVelocityMmps = (pos - _lastPosition) / elapsed_s;
        }
    } else {
        _hasLastTick = true;
    }
    _lastPosition = pos;
    _lastTickMs     = now_ms;
}

void SimMotor::setNoiseSigma(float sigma) {
    _mut.setEncoderNoise(sideIdx(), sigma);
}

// Encoder error injection (ticket 058-001): forward to the plant's per-wheel
// reported-encoder error model.  The true accumulator is untouched.
void SimMotor::setScaleError(float err) {
    _mut.setEncoderScaleError(sideIdx(), err);
}

void SimMotor::setSlip(float fraction) {
    _mut.setEncoderSlip(sideIdx(), fraction);
}
