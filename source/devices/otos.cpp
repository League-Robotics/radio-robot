#include "devices/otos.h"

#include <cmath>

namespace Devices {

namespace {
// CODAL's well-known convention: 0 == success (mirrors nezha_motor.cpp's
// identical local kOk — I2CBus::write()/read() already shield this leaf from
// any real-vs-HOST_BUILD MicroBit.h split, so no MICROBIT_OK include is
// needed here either).
constexpr int kOk = 0;

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
}  // namespace

Otos::Otos(I2CBus& bus, const OtosConfig& config)
    : bus_(bus), config_(config)
{
}

// ---------------------------------------------------------------------------
// begin() — product-ID detect, then init() + config scalars + zero pose.
// ---------------------------------------------------------------------------

void Otos::begin()
{
    uint8_t id = readReg8(kRegProductId);
    initialized_ = (id == kExpectedProductId);
    connected_ = initialized_;
    if (!initialized_) return;

    init();

    setLinearScalar(static_cast<float>(scaleToRegister(config_.linearScale)));
    setAngularScalar(static_cast<float>(scaleToRegister(config_.angularScale)));

    // Zero the OTOS position AND heading at boot so it starts at the same
    // origin as the freshly-zeroed encoders — the chip retains its tracked
    // pose across a micro:bit reset/reflash.
    writeXYH(kRegPositionXl, 0, 0, 0);
}

// ---------------------------------------------------------------------------
// pose() / poseFresh() / connected() / present() — cheap accessors, no I2C.
// ---------------------------------------------------------------------------

PoseReading Otos::pose() const { return cachedPose_; }

bool Otos::poseFresh() const { return poseFresh_; }

bool Otos::connected() const { return initialized_ && connected_; }

bool Otos::present() const { return initialized_; }

// ---------------------------------------------------------------------------
// readDue() -- pure scheduling query, no I2C traffic. Deliberately NOT
// gated on present()/initialized_ here -- that is the caller's own,
// separate conjunct (see this method's declaration comment, otos.h).
// ---------------------------------------------------------------------------

bool Otos::readDue(uint64_t nowUs) const
{
    return !hasRead_ || (nowUs - lastReadUs_) >= kReadPeriod;
}

// ---------------------------------------------------------------------------
// setPose() -- stages a re-anchor request; touches no bus. See tick()'s
// "Drain order" comment (otos.h) for how/when this is applied.
// ---------------------------------------------------------------------------

void Otos::setPose(float x, float y, float heading)
{
    pendingX_ = x;
    pendingY_ = y;
    pendingHeading_ = heading;
    posePending_ = true;
}

// ---------------------------------------------------------------------------
// tick() -- drain a staged setPose(), else rate-limited burst-read +
// transform + cache. See otos.h's declaration comment for the full contract.
// ---------------------------------------------------------------------------

void Otos::tick(uint64_t nowUs)
{
    if (!initialized_) return;   // never detected at begin() -- no bus traffic

    if (posePending_) {
        // A staged re-anchor always takes this tick's bus slot ahead of the
        // periodic read below -- see otos.h's "Drain order" comment.
        applyPendingPose();
        posePending_ = false;
        poseFresh_ = false;   // no read performed this tick; pose() unchanged
        return;
    }

    if (hasRead_ && (nowUs - lastReadUs_) < kReadPeriod) {
        // Too soon since the last real read -- no bus traffic, mark stale so
        // a downstream ring publish (DB-007) does not re-fuse the same
        // reading every cycle.
        poseFresh_ = false;
        return;
    }

    int16_t rx = 0, ry = 0, rh = 0;
    int16_t rvx = 0, rvy = 0, rvh = 0;
    bool ok = readPositionVelocity(rx, ry, rh, rvx, rvy, rvh);

    lastReadUs_ = nowUs;
    hasRead_ = true;

    // Live per-tick bus health -- always re-evaluated (a transient failure
    // does not latch permanently; see connected()'s own comment).
    connected_ = ok;

    if (!ok) {
        // Hold the previously-cached pose but mark THIS sample stale.
        poseFresh_ = false;
        return;
    }

    float xF = static_cast<float>(rx) * kPosMmPerLsb;    // [mm]
    float yF = static_cast<float>(ry) * kPosMmPerLsb;    // [mm]
    float hF = static_cast<float>(rh) * kHdgRadPerLsb;   // [rad]

    // 092-003 finding (carried forward unchanged, out of this ticket's
    // scope): the VELOCITY registers reuse the SAME kPosMmPerLsb/
    // kHdgRadPerLsb constants as position/offset despite the chip
    // documenting a different native velocity LSB scale -- see otos.h's
    // kPosMmPerLsb declaration comment.
    float vxF = static_cast<float>(rvx) * kPosMmPerLsb;    // [mm/s]
    float vyF = static_cast<float>(rvy) * kPosMmPerLsb;    // [mm/s]
    float whF = static_cast<float>(rvh) * kHdgRadPerLsb;   // [rad/s]

    // Mounting-yaw rotation (config_.offsetYaw) applied to the LINEAR
    // components only -- heading and yaw rate pass through unrotated (a
    // constant mounting rotation has zero time-derivative).
    float ang = -config_.offsetYaw;
    float c = cosf(ang);
    float s = sinf(ang);
    float rotX = c * xF - s * yF;
    float rotY = s * xF + c * yF;
    float rotVx = c * vxF - s * vyF;
    float rotVy = s * vxF + c * vyF;

    // Lever-arm compensation using hF -- the SAME-INSTANT heading from THIS
    // burst (see sensorToCentre()'s own comment, otos.h).
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

// ---------------------------------------------------------------------------
// applyPendingPose() -- exact inverse of tick()'s read transform: finds the
// sensor-frame pose that reads back as the given world CENTRE pose. Ported
// from the pre-port file's setPose() write-side math, unchanged.
// ---------------------------------------------------------------------------

void Otos::applyPendingPose()
{
    float sensorX = 0.0f, sensorY = 0.0f;
    centreToSensor(pendingX_, pendingY_, pendingHeading_, config_.offsetX, config_.offsetY,
                    sensorX, sensorY);

    // Undo the mounting-yaw rotation tick() applies going the other way:
    // tick() computes (rotX,rotY) = R(ang)*(xF,yF) with ang = -offsetYaw, so
    // the inverse is (xF,yF) = R(-ang)*(rotX,rotY) = R(ang)^T*(rotX,rotY).
    float ang = -config_.offsetYaw;
    float c = cosf(ang);
    float s = sinf(ang);
    float xF =  c * sensorX + s * sensorY;
    float yF = -s * sensorX + c * sensorY;
    float hF = pendingHeading_;   // heading takes no mounting offset

    writePoseMm(kRegPositionXl, xF, yF, hF);
}

// ---------------------------------------------------------------------------
// Remaining primitive setters/getters -- each a no-op if never initialized.
// ---------------------------------------------------------------------------

void Otos::init()
{
    if (!initialized_) return;
    // Enable all signal processing: LUT | Accel | Rotation | Variance = 0x0F.
    setSignalProcessConfig(0x0F);
    // Reset Kalman tracking (bit 0 = 1) -- the SAME write resetTracking() performs.
    writeReg8(kRegReset, 0x01);
    // Kick off IMU bias calibration -- fire-and-forget, no blocking poll for
    // completion (see file header for why this deliberately does not
    // block-poll).
    writeReg8(kRegImuCalibration, kImuCalibSamples);
}

void Otos::resetTracking()
{
    if (!initialized_) return;
    writeReg8(kRegReset, 0x01);
}

void Otos::setOffset(float x, float y, float heading)
{
    if (!initialized_) return;
    // Direct write -- REG_OFFSET holds the mounting-offset VALUE ITSELF, not
    // a world/chassis-centre pose to be converted through the lever arm the
    // way applyPendingPose() converts one.
    writePoseMm(kRegOffsetXl, x, y, heading);
}

void Otos::getOffset(float& x, float& y, float& heading)
{
    x = 0.0f; y = 0.0f; heading = 0.0f;
    if (!initialized_) return;

    int16_t rx = 0, ry = 0, rh = 0;
    readXYH(kRegOffsetXl, rx, ry, rh);

    x = static_cast<float>(rx) * kPosMmPerLsb;       // [mm]
    y = static_cast<float>(ry) * kPosMmPerLsb;       // [mm]
    heading = static_cast<float>(rh) * kHdgRadPerLsb; // [rad]
}

void Otos::setSignalProcessConfig(uint8_t config)
{
    if (!initialized_) return;
    writeReg8(kRegSignalProcessCfg, config);
}

uint8_t Otos::signalProcessConfig()
{
    if (!initialized_) return 0;
    return readReg8(kRegSignalProcessCfg);
}

uint8_t Otos::imuCalibrationSamplesRemaining()
{
    if (!initialized_) return 0;
    return readReg8(kRegImuCalibration);
}

void Otos::setLinearScalar(float scalar)
{
    if (!initialized_) return;
    int8_t clamped = static_cast<int8_t>(clampf(scalar, -127.0f, 127.0f));
    writeReg8(kRegLinearScalar, static_cast<uint8_t>(clamped));
}

void Otos::setAngularScalar(float scalar)
{
    if (!initialized_) return;
    int8_t clamped = static_cast<int8_t>(clampf(scalar, -127.0f, 127.0f));
    writeReg8(kRegAngularScalar, static_cast<uint8_t>(clamped));
}

// ---------------------------------------------------------------------------
// scaleToRegister -- ported from OtosSensor::scaleToInt8().
// ---------------------------------------------------------------------------

int8_t Otos::scaleToRegister(float scale)
{
    // scalar = clamp(round((scale - 1.0) / 0.001), -127, 127).
    float raw = roundf((scale - 1.0f) / 0.001f);
    if (raw >  127.0f) raw =  127.0f;
    if (raw < -127.0f) raw = -127.0f;
    return static_cast<int8_t>(raw);
}

// ---------------------------------------------------------------------------
// sensorToCentre()/centreToSensor() -- OTOS lever-arm compensation, ported
// unchanged -- see otos.h's declaration comments for the full same-instant-
// heading contract this relies on.
// ---------------------------------------------------------------------------

void Otos::sensorToCentre(float sensorX, float sensorY, float sensorHeading,
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

void Otos::centreToSensor(float centreX, float centreY, float centreHeading,
                           float offsetX, float offsetY,
                           float& sensorXOut, float& sensorYOut)
{
    float c = cosf(centreHeading);
    float s = sinf(centreHeading);
    sensorXOut = centreX + (c * offsetX - s * offsetY);
    sensorYOut = centreY + (s * offsetX + c * offsetY);
}

// ---------------------------------------------------------------------------
// Private register-map helpers -- ported byte-for-byte.
// ---------------------------------------------------------------------------

void Otos::writeReg8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    bus_.write(static_cast<uint16_t>(kOtosDeviceAddr << 1), buf, 2, false,
               /*preClear=*/0, /*postClear=*/kBusClearance);
}

uint8_t Otos::readReg8(uint8_t reg)
{
    uint8_t result = 0;
    bus_.write(static_cast<uint16_t>(kOtosDeviceAddr << 1), &reg, 1, false,
               /*preClear=*/0, /*postClear=*/kBusClearance);
    bus_.read(static_cast<uint16_t>(kOtosDeviceAddr << 1), &result, 1, false,
              /*preClear=*/kBusClearance, /*postClear=*/0);
    return result;
}

bool Otos::readPositionVelocity(int16_t& x, int16_t& y, int16_t& h,
                                 int16_t& vx, int16_t& vy, int16_t& vh)
{
    // ONE 12-byte burst read covers kRegPositionXl (6 bytes) followed
    // immediately by the CONTIGUOUS kRegVelocityXl (6 bytes).
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

void Otos::writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h)
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

bool Otos::readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h)
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

void Otos::writePoseMm(uint8_t startReg, float xF, float yF, float hF)
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

}  // namespace Devices
