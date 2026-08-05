#include "devices/nezha_motor.h"

#include <cmath>
#ifdef HOST_BUILD
#include <iostream>
#endif

// [3..7].

namespace Devices {

namespace {
constexpr int kOk = 0;

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int8_t clampStep(int8_t lastWritten, int8_t target, uint8_t maxDelta) {
    int16_t delta = static_cast<int16_t>(target) - static_cast<int16_t>(lastWritten);
    if (delta > static_cast<int16_t>(maxDelta)) {
        return static_cast<int8_t>(static_cast<int16_t>(lastWritten) + static_cast<int16_t>(maxDelta));
    }
    if (delta < -static_cast<int16_t>(maxDelta)) {
        return static_cast<int8_t>(static_cast<int16_t>(lastWritten) - static_cast<int16_t>(maxDelta));
    }
    return target;
}

int32_t median3(int32_t a, int32_t b, int32_t c) {
    int32_t lo = a, mid = b, hi = c;
    if (lo > hi) { int32_t tmp = lo; lo = hi; hi = tmp; }
    if (lo > mid) { int32_t tmp = lo; lo = mid; mid = tmp; }
    if (mid > hi) { mid = hi; }
    return mid;
}

int32_t decodeRawEncoder(const uint8_t resp[4]) {
    return static_cast<int32_t>(
        (static_cast<uint32_t>(resp[3]) << 24) |
        (static_cast<uint32_t>(resp[2]) << 16) |
        (static_cast<uint32_t>(resp[1]) << 8) |
        static_cast<uint32_t>(resp[0]));
}
}

NezhaMotor::NezhaMotor(I2CBus& bus, const MotorConfig& config)
    : bus_(bus)
{
    (void)reconfigure(config);
}

bool NezhaMotor::reconfigure(const MotorConfig& config)
{
    bool atRest = std::fabs(velocity_) < kReconfigureRestVelocity &&
                  appliedDuty() == 0.0f;
    if (mode_ != Mode::None && !atRest) {
        return false;
    }

    config_ = config;
    if (config_.slewRate <= 0.0f) {
        config_.slewRate = kDefaultSlewRate;
    }
    reversalDwell_ = config.reversalDwell;
    outputDeadband_ = config.outputDeadband;
    return true;
}

void NezhaMotor::begin()
{
    hardReset();
}

void NezhaMotor::resetPosition() { hardReset(); }

void NezhaMotor::rebaseline() { softRebaseline(); }


void NezhaMotor::setDuty(float duty)
{
    dutyTarget_ = duty;
    mode_ = Mode::Active;
}

void NezhaMotor::setNeutral(Neutral mode)
{
    neutralTarget_ = mode;
    mode_ = Mode::Neutral;
}

void NezhaMotor::applyTravelCalib(float travelCalib)
{
    config_.wheelTravelCalib = travelCalib;
}


float NezhaMotor::position() const { return lastPosition_; }
float NezhaMotor::velocity() const { return velocity_; }

float NezhaMotor::appliedDuty() const
{
    if (lastWrittenPct_ == -128) return 0.0f;
    return static_cast<float>(lastWrittenPct_) / 100.0f;
}

void NezhaMotor::tick(uint64_t nowUs)
{
    uint32_t nowMs = static_cast<uint32_t>(nowUs / 1000);

    int32_t raw = collectEncoder();
    float pos = (static_cast<float>(raw) / 10.0f)
              * config_.wheelTravelCalib * static_cast<float>(config_.fwdSign);

    if (hasLastTick_) {
        float elapsedTime = static_cast<float>(nowUs - lastTickUs_) / 1e6f;  // [s]
        if (elapsedTime > 0.0f) {
            velocity_ = (pos - lastPosition_) / elapsedTime;
        }
    } else {
        hasLastTick_ = true;
    }
    lastPosition_ = pos;
    lastTickUs_ = nowUs;

    if (connected_) {
        lastFreshUs_ = nowUs;
    }

    switch (mode_) {
        case Mode::Active:
            writeShapedDuty(dutyTarget_, nowMs);
            break;
        case Mode::Neutral:
            (void)neutralTarget_;
            writeShapedDuty(0.0f, nowMs);
            break;
        case Mode::None:
        default:
            break;
    }
}

void NezhaMotor::writeShapedDuty(float duty, uint32_t now)
{
    if (duty == 0.0f) {
        dwelling_ = false;
        lastRequestedDuty_ = 0.0f;
        writeRawDuty(0.0f);
        return;
    }

    if (fabsf(duty) < outputDeadband_) {
        duty = std::copysign(outputDeadband_, duty);
    }

    if (dwelling_) {
        if (now < dwellDeadline_) {
            lastRequestedDuty_ = 0.0f;
            writeRawDuty(0.0f);
            return;
        }
        dwelling_ = false;
    } else if (reversalDwell_ > 0.0f && lastRequestedDuty_ != 0.0f &&
               ((duty > 0.0f) != (lastRequestedDuty_ > 0.0f))) {
        dwelling_ = true;
        dwellDeadline_ = now + static_cast<uint32_t>(reversalDwell_);
        lastRequestedDuty_ = 0.0f;
        writeRawDuty(0.0f);
        return;
    }

    lastRequestedDuty_ = duty;
    writeRawDuty(duty);
}

void NezhaMotor::writeRawDuty(float duty)
{
    duty = clampf(duty, -1.0f, 1.0f);

    // SIGMA-DELTA on the rounding residual (133-002; measured on vevov
    // 2026-08-03). The brick takes an INTEGER PERCENT, and at
    // App::Drive::kDutyPerSpeed = 0.001182 one count is 8.46 mm/s -- 5.6%
    // of a 150 mm/s command. Measured: plateau velocity clusters at
    // exactly -1/0/+1 counts (sd 8.0 mm/s against an 8.46 mm/s step), and
    // the residual left/right distance imbalance being chased was 0.43 of
    // ONE COUNT. That is below the actuator's resolution, which is why
    // sweeping ki, kff and aSteady all failed to move it: no gain can
    // command a value the output cannot represent.
    //
    // Carrying the rounding residual into the next tick makes the
    // TIME-AVERAGED percent equal the fractional duty actually wanted, so
    // the loop gets sub-count resolution out of the same integer wire
    // field. Same idea as App::Drive::crawlDuty()'s Bresenham
    // accumulator, applied to EVERY duty rather than only sub-deadband
    // ones.
    //
    // The carry is DISCARDED on a commanded zero: a residual left over
    // from the last nonzero duty would otherwise round to +-1 and creep a
    // stopped wheel -- re-creating exactly the runaway class this file's
    // own header and 133-001 exist to prevent. Losing sub-count fidelity
    // across a stop is the correct trade. Stop stays stop.
    int8_t pct;
    if (duty == 0.0f) {
        dutyCarry_ = 0.0f;
        pct = 0;
    } else {
        const float wanted = clampf(duty * 100.0f + dutyCarry_, -100.0f, 100.0f);
        pct = static_cast<int8_t>(lroundf(wanted));
        // Bounded so a saturated command cannot accumulate an unbounded
        // debt that later dumps into the output as a lurch.
        dutyCarry_ = clampf(wanted - static_cast<float>(pct), -1.0f, 1.0f);
    }
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;

    const bool stopNotTaken = pct == 0 && fabsf(velocity_) > kStopConfirmVelocity;
    if (pct == lastWrittenPct_ && !stopNotTaken) {
        return;
    }

    static constexpr uint64_t kMinWriteIntervalUs = 45000;  // [us] kCycle(50ms) - 5ms jitter margin
    bool stopping = (pct == 0);
    uint64_t now = lastTickUs_;  // [us] this tick's timestamp (see file-header note)
    if (!stopping && (now - lastWriteTimeUs_) < kMinWriteIntervalUs) {
        return;
    }

    bool firstWrite = (lastWrittenPct_ == -128);
    int8_t written = (stopping || firstWrite)
        ? pct
        : clampStep(lastWrittenPct_, pct, static_cast<uint8_t>(config_.slewRate));

    int16_t effective = static_cast<int16_t>(config_.fwdSign) * static_cast<int16_t>(written);
    int status;
    if (effective == 0) {
        status = writeMotorRun(kDirCw, 0);
    } else {
        uint8_t dir = (effective > 0) ? kDirCw : kDirCcw;
        uint8_t speed = (effective > 0) ? static_cast<uint8_t>(effective)
                                         : static_cast<uint8_t>(-effective);
        status = writeMotorRun(dir, speed);
    }

    if (status == kOk) {
        lastWriteTimeUs_ = now;
        lastWrittenPct_ = written;
    }
}

int NezhaMotor::writeMotorRun(uint8_t direction, uint8_t speed)
{
    uint8_t buf[8] = {
        0xFF, 0xF9,
        static_cast<uint8_t>(config_.port),
        direction,
        0x60,
        speed,
        0xF5,
        0x00
    };
    return bus_.write(static_cast<uint16_t>(kNezhaDeviceAddr << 1), buf, 8, false, /*preClear=*/0, /*postClear=*/4000);
}


int32_t NezhaMotor::readEncoderAtomicRaw()
{
    static constexpr uint32_t kDelayUs = 4000;

    uint8_t cmd[8] = { 0xFF, 0xF9, static_cast<uint8_t>(config_.port), 0x00, 0x46, 0x00, 0xF5, 0x00 };
    int writeResult = bus_.write(static_cast<uint16_t>(kNezhaDeviceAddr << 1), cmd, 8, false,
                                 /*preClear=*/kDelayUs, /*postClear=*/kDelayUs);

    uint8_t resp[4] = { 0, 0, 0, 0 };
    int readResult = bus_.read(static_cast<uint16_t>(kNezhaDeviceAddr << 1), resp, 4, false);

    connected_ = (writeResult == kOk && readResult == kOk);
    if (!connected_) {
        return lastGoodRawEnc_;
    }

    int32_t result = decodeRawEncoder(resp) - encOffset_;
    lastGoodRawEnc_ = result;
    return result;
}

void NezhaMotor::requestSample()
{
    requestEncoder();
}

void NezhaMotor::requestEncoder()
{
    uint8_t cmd[8] = { 0xFF, 0xF9, static_cast<uint8_t>(config_.port), 0x00, 0x46, 0x00, 0xF5, 0x00 };
    int writeResult = bus_.write(static_cast<uint16_t>(kNezhaDeviceAddr << 1), cmd, 8, false,
                                 /*preClear=*/4000, /*postClear=*/4000);
    pendingEncRequestOk_ = (writeResult == kOk);
}

int32_t NezhaMotor::collectEncoder()
{
    uint8_t resp[4] = { 0, 0, 0, 0 };
    int readResult = bus_.read(static_cast<uint16_t>(kNezhaDeviceAddr << 1), resp, 4, false);

    connected_ = pendingEncRequestOk_ && (readResult == kOk);

    if (!connected_) {
        return lastGoodRawEnc_;
    }

    int32_t result = decodeRawEncoder(resp) - encOffset_;
    lastGoodRawEnc_ = result;
    return result;
}

void NezhaMotor::hardReset()
{
    static constexpr int kMaxRetries = 2;
    static constexpr int32_t kReadbackThreshold = 2;

    for (int attempt = 0; attempt <= kMaxRetries; ++attempt) {
        int32_t snapshot = median3(readEncoderAtomicRaw(), readEncoderAtomicRaw(), readEncoderAtomicRaw());
        encOffset_ += snapshot;

        int32_t readback = readEncoderAtomicRaw();
        if (readback >= -kReadbackThreshold && readback <= kReadbackThreshold) {
            lastPosition_ = 0.0f;
            velocity_ = 0.0f;
            hasLastTick_ = false;
            lastGoodRawEnc_ = 0;
            return;
        }
        encOffset_ -= snapshot;
    }

    encOffset_ += median3(readEncoderAtomicRaw(), readEncoderAtomicRaw(), readEncoderAtomicRaw());
    lastPosition_ = 0.0f;
    velocity_ = 0.0f;
    hasLastTick_ = false;
    lastGoodRawEnc_ = 0;
}

void NezhaMotor::softRebaseline()
{
    if (config_.wheelTravelCalib != 0.0f) {
        float rawDeltaF = (lastPosition_ / (config_.wheelTravelCalib * static_cast<float>(config_.fwdSign))) * 10.0f;
        encOffset_ += static_cast<int32_t>(rawDeltaF);
    }
    lastPosition_ = 0.0f;
    hasLastTick_ = false;
    lastGoodRawEnc_ = 0;
}

}
