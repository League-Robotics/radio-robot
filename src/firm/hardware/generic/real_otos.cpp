#include "hardware/generic/real_otos.h"

#include <cmath>

namespace Hardware {

namespace {
constexpr int kOk = 0;

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
}  // namespace

int8_t scaleToRegister(float scale)
{
    float raw = roundf((scale - 1.0f) / 0.001f);
    if (raw >  127.0f) raw =  127.0f;
    if (raw < -127.0f) raw = -127.0f;
    return static_cast<int8_t>(raw);
}

RealOtos::RealOtos(Hal::I2CBus& bus, const Hal::OtosConfig& config)
    : bus_(bus), config_(config)
{
}

void RealOtos::begin()
{
    uint8_t id = readReg8(kRegProductId);
    lastProbeId_ = id;   // captured for ODIAG bench triage
    initialized_ = (id == kExpectedProductId);
    connected_ = initialized_;
    if (!initialized_) return;

    init();

    setLinearScalar(static_cast<float>(scaleToRegister(config_.linearScale)));
    setAngularScalar(static_cast<float>(scaleToRegister(config_.angularScale)));

    // Zero the chip's offset register on EVERY boot. The lever arm is applied
    // in software (sensorToCentre()); leaving a stale arm in the chip too
    // double-corrects. This cannot be left to setOffset(), because the chip is
    // never power-cycled by an nRF reset -- it keeps whatever offset an
    // earlier session wrote, so a boot that never calls configureOtos() would
    // silently inherit it. Measured 2026-08-05: a pure pivot traced a 42.7mm
    // circle instead of holding the centre still.
    writePoseMm(kRegOffsetXl, 0.0f, 0.0f, 0.0f);

    writeXYH(kRegPositionXl, 0, 0, 0);
}

Hal::PoseReading RealOtos::pose() const { return cachedPose_; }

bool RealOtos::poseFresh() const { return poseFresh_; }

bool RealOtos::connected() const { return initialized_ && connected_; }

bool RealOtos::present() const { return initialized_; }

bool RealOtos::readDue(uint64_t nowUs) const
{
    return !hasRead_ || (nowUs - lastReadUs_) >= kReadPeriod;
}

void RealOtos::setPose(float x, float y, float heading)
{
    pendingX_ = x;
    pendingY_ = y;
    pendingHeading_ = heading;
    posePending_ = true;
}

void RealOtos::tick(uint64_t nowUs)
{
    if (!initialized_) return;   // never detected at begin() -- no bus traffic

    if (posePending_) {
        applyPendingPose();
        posePending_ = false;
        poseFresh_ = false;   // no read performed this tick; pose() unchanged
        return;
    }

    if (hasRead_ && (nowUs - lastReadUs_) < kReadPeriod) {
        poseFresh_ = false;
        return;
    }

    int16_t rx = 0, ry = 0, rh = 0;
    int16_t rvx = 0, rvy = 0, rvh = 0;
    bool ok = readPositionVelocity(rx, ry, rh, rvx, rvy, rvh);

    lastReadUs_ = nowUs;
    hasRead_ = true;

    connected_ = ok;

    if (!ok) {
        poseFresh_ = false;
        return;
    }

    float xF = static_cast<float>(rx) * kPosMmPerLsb;    // [mm]
    float yF = static_cast<float>(ry) * kPosMmPerLsb;    // [mm]
    float hF = static_cast<float>(rh) * kHdgRadPerLsb;   // [rad]

    float vxF = static_cast<float>(rvx) * kVelocityPerLsb;  // [mm/s]
    float vyF = static_cast<float>(rvy) * kVelocityPerLsb;  // [mm/s]
    float whF = static_cast<float>(rvh) * kOmegaPerLsb;     // [rad/s]

    float ang = -config_.offsetYaw;
    float c = cosf(ang);
    float s = sinf(ang);
    float rotX = c * xF - s * yF;
    float rotY = s * xF + c * yF;
    float rotVx = c * vxF - s * vyF;
    float rotVy = s * vxF + c * vyF;

    float centreX = 0.0f, centreY = 0.0f;
    sensorToCentre(rotX, rotY, hF, config_.offsetX, config_.offsetY, centreX, centreY);

    cachedPose_.x = centreX;
    cachedPose_.y = centreY;
    cachedPose_.heading = hF;   // heading takes no mounting offset
    cachedPose_.v_x = rotVx;
    cachedPose_.v_y = rotVy;
    cachedPose_.omega = whF;
    poseFresh_ = true;
}

void RealOtos::applyPendingPose()
{
    float sensorX = 0.0f, sensorY = 0.0f;
    centreToSensor(pendingX_, pendingY_, pendingHeading_, config_.offsetX, config_.offsetY,
                    sensorX, sensorY);

    float ang = -config_.offsetYaw;
    float c = cosf(ang);
    float s = sinf(ang);
    float xF =  c * sensorX + s * sensorY;
    float yF = -s * sensorX + c * sensorY;
    float hF = pendingHeading_;   // heading takes no mounting offset

    writePoseMm(kRegPositionXl, xF, yF, hF);
}

void RealOtos::init()
{
    if (!initialized_) return;
    setSignalProcessConfig(0x0F);
    writeReg8(kRegReset, 0x01);
    writeReg8(kRegImuCalibration, kImuCalibSamples);
}

void RealOtos::resetTracking()
{
    if (!initialized_) return;
    writeReg8(kRegReset, 0x01);
}

void RealOtos::setOffset(float x, float y, float heading)
{
    // The lever arm is applied HERE, in software, by sensorToCentre() on every
    // read and centreToSensor() on every seed. The chip's own offset register
    // is deliberately held at ZERO.
    //
    // Applying it in both places double-corrects. Measured 2026-08-05: with a
    // 47.8mm configured arm written to the chip AND subtracted in software, a
    // pure in-place pivot made the reported centre trace a circle of radius
    // 41.8mm instead of holding still (the camera, whose tag is registered at
    // the centre of rotation, held it to 8.1mm through the same pivot). That
    // was the dominant term in every OTOS-vs-camera disagreement, and it hid
    // whenever a run happened to end on its starting heading.
    //
    // This also updates config_, which the previous version did not: a live
    // offset push wrote the chip and left the software transform on the boot
    // value, so the two silently disagreed.
    config_.offsetX = x;
    config_.offsetY = y;
    config_.offsetYaw = heading;
    if (!initialized_) return;
    writePoseMm(kRegOffsetXl, 0.0f, 0.0f, 0.0f);
}

void RealOtos::getOffset(float& x, float& y, float& heading)
{
    // The EFFECTIVE lever arm, which lives in config_ -- not the chip's own
    // offset register, which setOffset() holds at zero on purpose.
    x = config_.offsetX;             // [mm]
    y = config_.offsetY;             // [mm]
    heading = config_.offsetYaw;     // [rad]
}

void RealOtos::setSignalProcessConfig(uint8_t config)
{
    if (!initialized_) return;
    writeReg8(kRegSignalProcessCfg, config);
}

uint8_t RealOtos::signalProcessConfig()
{
    if (!initialized_) return 0;
    return readReg8(kRegSignalProcessCfg);
}

uint8_t RealOtos::imuCalibrationSamplesRemaining()
{
    if (!initialized_) return 0;
    return readReg8(kRegImuCalibration);
}

void RealOtos::calibrateImu(uint8_t samples)
{
    // Bias calibration only -- deliberately NOT the reset+calibrate pair
    // init() does, so tracking and a seeded pose survive. 0 means the boot
    // default (kImuCalibSamples = 255, ~612ms of stillness).
    if (!initialized_) return;
    writeReg8(kRegImuCalibration, samples == 0 ? kImuCalibSamples : samples);
}

void RealOtos::setLinearScalar(float scalar)
{
    if (!initialized_) return;
    int8_t clamped = static_cast<int8_t>(clampf(scalar, -127.0f, 127.0f));
    writeReg8(kRegLinearScalar, static_cast<uint8_t>(clamped));
}

void RealOtos::setAngularScalar(float scalar)
{
    if (!initialized_) return;
    int8_t clamped = static_cast<int8_t>(clampf(scalar, -127.0f, 127.0f));
    writeReg8(kRegAngularScalar, static_cast<uint8_t>(clamped));
}

void RealOtos::sensorToCentre(float sensorX, float sensorY, float sensorHeading,
                           float offsetX, float offsetY,
                           float& centreXOut, float& centreYOut)
{
    float c = cosf(sensorHeading);
    float s = sinf(sensorHeading);
    float offsetXWorld = c * offsetX - s * offsetY;
    float offsetYWorld = s * offsetX + c * offsetY;
    centreXOut = sensorX - offsetXWorld;
    centreYOut = sensorY - offsetYWorld;
}

void RealOtos::centreToSensor(float centreX, float centreY, float centreHeading,
                           float offsetX, float offsetY,
                           float& sensorXOut, float& sensorYOut)
{
    float c = cosf(centreHeading);
    float s = sinf(centreHeading);
    sensorXOut = centreX + (c * offsetX - s * offsetY);
    sensorYOut = centreY + (s * offsetX + c * offsetY);
}

void RealOtos::writeReg8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    bus_.write(static_cast<uint16_t>(kOtosDeviceAddr << 1), buf, 2, false,
               /*preClear=*/0, /*postClear=*/kBusClearance);
}

uint8_t RealOtos::readReg8(uint8_t reg)
{
    uint8_t result = 0;
    bus_.write(static_cast<uint16_t>(kOtosDeviceAddr << 1), &reg, 1, false,
               /*preClear=*/0, /*postClear=*/kBusClearance);
    bus_.read(static_cast<uint16_t>(kOtosDeviceAddr << 1), &result, 1, false,
              /*preClear=*/kBusClearance, /*postClear=*/0);
    return result;
}

bool RealOtos::readPositionVelocity(int16_t& x, int16_t& y, int16_t& h,
                                 int16_t& vx, int16_t& vy, int16_t& vh)
{
    uint8_t reg = kRegPositionXl;
    uint8_t raw[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int writeStatus = bus_.write(static_cast<uint16_t>(kOtosDeviceAddr << 1), &reg, 1, false,
                                  /*preClear=*/0, /*postClear=*/kBusClearance);
    int readStatus = bus_.read(static_cast<uint16_t>(kOtosDeviceAddr << 1), raw, 12, false,
                                /*preClear=*/kBusClearance, /*postClear=*/0);

    x  = static_cast<int16_t>(raw[0]  | (static_cast<uint16_t>(raw[1])  << 8));
    y  = static_cast<int16_t>(raw[2]  | (static_cast<uint16_t>(raw[3])  << 8));
    h  = static_cast<int16_t>(raw[4]  | (static_cast<uint16_t>(raw[5])  << 8));
    vx = static_cast<int16_t>(raw[6]  | (static_cast<uint16_t>(raw[7])  << 8));
    vy = static_cast<int16_t>(raw[8]  | (static_cast<uint16_t>(raw[9])  << 8));
    vh = static_cast<int16_t>(raw[10] | (static_cast<uint16_t>(raw[11]) << 8));

    return (writeStatus == kOk && readStatus == kOk);
}

void RealOtos::writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h)
{
    uint8_t buf[7];
    buf[0] = startReg;
    buf[1] = static_cast<uint8_t>(x & 0xFF);
    buf[2] = static_cast<uint8_t>((x >> 8) & 0xFF);
    buf[3] = static_cast<uint8_t>(y & 0xFF);
    buf[4] = static_cast<uint8_t>((y >> 8) & 0xFF);
    buf[5] = static_cast<uint8_t>(h & 0xFF);
    buf[6] = static_cast<uint8_t>((h >> 8) & 0xFF);
    bus_.write(static_cast<uint16_t>(kOtosDeviceAddr << 1), buf, 7, false,
               /*preClear=*/0, /*postClear=*/kBusClearance);
}

bool RealOtos::readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h)
{
    uint8_t reg = startReg;
    uint8_t raw[6] = {0, 0, 0, 0, 0, 0};
    int writeStatus = bus_.write(static_cast<uint16_t>(kOtosDeviceAddr << 1), &reg, 1, false,
                                  /*preClear=*/0, /*postClear=*/kBusClearance);
    int readStatus = bus_.read(static_cast<uint16_t>(kOtosDeviceAddr << 1), raw, 6, false,
                                /*preClear=*/kBusClearance, /*postClear=*/0);

    x = static_cast<int16_t>(raw[0] | (static_cast<uint16_t>(raw[1]) << 8));
    y = static_cast<int16_t>(raw[2] | (static_cast<uint16_t>(raw[3]) << 8));
    h = static_cast<int16_t>(raw[4] | (static_cast<uint16_t>(raw[5]) << 8));

    return (writeStatus == kOk && readStatus == kOk);
}

void RealOtos::writePoseMm(uint8_t startReg, float xF, float yF, float hF)
{
    long rx = lroundf(xF / kPosMmPerLsb);
    long ry = lroundf(yF / kPosMmPerLsb);
    long rh = lroundf(hF / kHdgRadPerLsb);
    if (rx >  32767) rx =  32767;
    if (rx < -32767) rx = -32767;
    if (ry >  32767) ry =  32767;
    if (ry < -32767) ry = -32767;
    if (rh >  32767) rh =  32767;
    if (rh < -32767) rh = -32767;
    writeXYH(startReg, static_cast<int16_t>(rx), static_cast<int16_t>(ry),
             static_cast<int16_t>(rh));
}

}  // namespace Hardware
