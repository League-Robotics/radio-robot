#include "hal/otos/otos_odometer.h"

#include "hal/lever_arm.h"

#ifndef HOST_BUILD
#include "MicroBit.h"   // MICROBIT_OK (vendor SDK; excluded from the no-units-in-identifiers
                        // rename per .claude/rules/coding-standards.md)
#else
// HOST_BUILD (086-006 host harness, tests/sim/unit/otos_odometer_harness.cpp):
// no CODAL, so no MicroBit.h. Mirrors nezha_motor.cpp's own identical guard —
// MICROBIT_OK is CODAL's well-known convention (0 == success).
namespace {
constexpr int MICROBIT_OK = 0;
}  // namespace
#endif

#include <math.h>

namespace Hal {

namespace {
float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
}  // namespace

OtosOdometer::OtosOdometer(I2CBus& bus, const Config::OtosBootConfig& config)
    : bus_(bus), config_(config)
{
}

// ---------------------------------------------------------------------------
// begin() — product-ID detect, then init() + boot-config scalars + zero pose.
// ---------------------------------------------------------------------------

void OtosOdometer::begin()
{
    uint8_t id = readReg8(kRegProductId);
    initialized_ = (id == kExpectedProductId);
    connected_ = initialized_;
    if (!initialized_) return;

    // Enable signal processing, reset Kalman tracking, kick off IMU bias
    // calibration (fire-and-forget — see file header).
    init();

    // Mounting offset (lever-arm) is applied HOST-SIDE (source/hal/
    // lever_arm.h), never via the chip's own REG_OFFSET (silently
    // unwritable on this hardware — see file header). The linear/angular
    // SCALE MULTIPLIERS from the boot config, however, DO go to the chip's
    // own registers, converted to its raw int8 domain once here — matching
    // OtosSensor::begin()'s scaleToInt8() conversion. OL/OA's own live wire
    // calls (setLinearScalar()/setAngularScalar() below) operate on that
    // same raw register domain directly, never re-deriving from a
    // multiplier.
    setLinearScalar(static_cast<float>(scaleToRegister(config_.linearScale)));
    setAngularScalar(static_cast<float>(scaleToRegister(config_.angularScale)));

    // Zero the OTOS position AND heading at boot so it starts at the same
    // origin as the freshly-zeroed encoders — the chip retains its tracked
    // pose across a micro:bit reset/reflash (only a power cycle or an
    // explicit write clears it), so without this the first tick() would
    // report a stale pose against the encoders' fresh (0,0,0) origin
    // (mirrors OtosSensor::begin()'s identical rationale).
    writeXYH(kRegPositionXl, 0, 0, 0);
}

// ---------------------------------------------------------------------------
// pose() / connected() — cheap accessors, no I2C traffic.
// ---------------------------------------------------------------------------

msg::PoseEstimate OtosOdometer::pose() const { return cachedPose_; }

bool OtosOdometer::connected() const { return initialized_ && connected_; }

// ---------------------------------------------------------------------------
// tick() — burst-read position + velocity, transform, cache.
// ---------------------------------------------------------------------------

void OtosOdometer::tick(uint32_t now)
{
    if (!initialized_) return;   // never detected at begin() -- no bus traffic

    // 086-007 rate limiting: dev_loop.cpp calls tick() unconditionally every
    // main-loop pass (~470 Hz observed on hardware) -- the OTOS does not
    // need reading anywhere near that often. A tick() call that arrives
    // sooner than kReadPeriod since the last REAL bus read is a no-op on
    // the bus; mark THIS sample stale so Subsystems::PoseEstimator::tick()
    // does not re-fuse the same reading every pass (over-weighting the
    // EKF) -- see tick()'s own doc comment and the file header's "086-007
    // HITL fix" section for the CODAL NRF52I2C::waitForStop() stall this
    // (paired with the clearance added below) eliminates.
    if (hasRead_ && (now - lastReadMs_) < kReadPeriod) {
        cachedPose_.stamp.valid = false;
        return;
    }

    int16_t rx = 0, ry = 0, rh = 0;
    int16_t rvx = 0, rvy = 0, rvh = 0;
    bool ok = readPositionVelocity(rx, ry, rh, rvx, rvy, rvh);

    lastReadMs_ = now;
    hasRead_ = true;

    // Live per-tick bus health -- always re-evaluated (see tick()'s own doc
    // comment for why a transient failure does not latch permanently).
    connected_ = ok;

    if (!ok) {
        // Hold the previously-cached pose (mirrors Hal::NezhaMotor's
        // lastGoodRawEnc_ failure-hold) but mark THIS sample stale so
        // Subsystems::PoseEstimator::tick() skips fusion this pass.
        cachedPose_.stamp.valid = false;
        return;
    }

    float xF = static_cast<float>(rx) * kPosMmPerLsb;    // [mm]
    float yF = static_cast<float>(ry) * kPosMmPerLsb;    // [mm]
    float hF = static_cast<float>(rh) * kHdgRadPerLsb;   // [rad]

    float vxF = static_cast<float>(rvx) * kPosMmPerLsb;    // [mm/s]
    float vyF = static_cast<float>(rvy) * kPosMmPerLsb;    // [mm/s]
    float whF = static_cast<float>(rvh) * kHdgRadPerLsb;   // [rad/s]

    // Mounting-yaw rotation (config_.offsetYaw -- the OTOS chip's own
    // rotation relative to the robot's forward axis, geometry.
    // odometry_offset_mm.yaw_rad) applied to the LINEAR components only;
    // heading and yaw rate pass through unrotated -- a constant mounting
    // rotation has zero time-derivative, so it does not appear in omega,
    // and the chip's own heading channel already tracks the robot's world
    // heading directly (ported rationale from OtosSensor::readTransformed()/
    // readVelocityTransformed3()).
    float ang = -config_.offsetYaw;
    float c = cosf(ang);
    float s = sinf(ang);
    float rotX = c * xF - s * yF;
    float rotY = s * xF + c * yF;
    float rotVx = c * vxF - s * vyF;
    float rotVy = s * vxF + c * vyF;

    // Lever-arm compensation (source/hal/lever_arm.h) using hF -- the
    // SAME-INSTANT heading from THIS burst, never a heading left over from
    // a previous tick (the same-instant-heading contract; see lever_arm.h
    // and this method's own doc comment).
    float centreX = 0.0f, centreY = 0.0f;
    LeverArm::sensorToCentre(rotX, rotY, hF, config_.offsetX, config_.offsetY, centreX, centreY);

    cachedPose_.pose.x = centreX;
    cachedPose_.pose.y = centreY;
    cachedPose_.pose.h = hF;   // heading takes no mounting offset
    cachedPose_.twist.v_x = rotVx;
    cachedPose_.twist.v_y = rotVy;
    cachedPose_.twist.omega = whF;
    cachedPose_.stamp.valid = true;
    cachedPose_.stamp.last_upd = now;
    cachedPose_.stamp.lag = 0;
}

// ---------------------------------------------------------------------------
// Hal::Odometer's primitive setters -- each a no-op if never initialized.
// ---------------------------------------------------------------------------

void OtosOdometer::init()
{
    if (!initialized_) return;
    // Enable all signal processing: LUT | Accel | Rotation | Variance = 0x0F.
    writeReg8(kRegSignalProcessCfg, 0x0F);
    // Reset Kalman tracking (bit 0 = 1) -- the SAME write resetTracking() performs.
    writeReg8(kRegReset, 0x01);
    // Kick off IMU bias calibration -- fire-and-forget, no blocking poll for
    // completion (see file header for why this deliberately does not port
    // OtosSensor::init()'s fiber_sleep-based busy-wait).
    writeReg8(kRegImuCalibration, kImuCalibSamples);
}

void OtosOdometer::resetTracking()
{
    if (!initialized_) return;
    writeReg8(kRegReset, 0x01);
}

void OtosOdometer::setPose(const msg::Pose2D& pose)
{
    if (!initialized_) return;

    // Exact inverse of tick()'s read transform: find the sensor-frame pose
    // that reads back as the given world CENTRE pose (OZ, via an all-zero
    // Pose2D from Hal::Odometer::apply()'s ZERO arm, or OV). Used to anchor
    // the OTOS to a camera fix (SI) so its absolute observations agree with
    // the controller pose instead of dragging the EKF toward the boot
    // frame. Ported from OtosSensor::setWorldPose().
    float sensorX = 0.0f, sensorY = 0.0f;
    LeverArm::centreToSensor(pose.x, pose.y, pose.h, config_.offsetX, config_.offsetY,
                              sensorX, sensorY);

    // Undo the mounting-yaw rotation tick() applies going the other way:
    // tick() computes (rotX,rotY) = R(ang)*(xF,yF) with ang = -offsetYaw, so
    // the inverse is (xF,yF) = R(-ang)*(rotX,rotY) = R(ang)^T*(rotX,rotY).
    float ang = -config_.offsetYaw;
    float c = cosf(ang);
    float s = sinf(ang);
    float xF =  c * sensorX + s * sensorY;
    float yF = -s * sensorX + c * sensorY;
    float hF = pose.h;   // heading takes no mounting offset (tick()'s own convention)

    long rx = lroundf(xF / kPosMmPerLsb);
    long ry = lroundf(yF / kPosMmPerLsb);
    long rh = lroundf(hF / kHdgRadPerLsb);
    if (rx >  32767) rx =  32767;
    if (rx < -32767) rx = -32767;
    if (ry >  32767) ry =  32767;
    if (ry < -32767) ry = -32767;
    if (rh >  32767) rh =  32767;
    if (rh < -32767) rh = -32767;
    writeXYH(kRegPositionXl, static_cast<int16_t>(rx), static_cast<int16_t>(ry),
             static_cast<int16_t>(rh));
}

void OtosOdometer::setLinearScalar(float scalar)
{
    if (!initialized_) return;
    int8_t clamped = static_cast<int8_t>(clampf(scalar, -127.0f, 127.0f));
    writeReg8(kRegLinearScalar, static_cast<uint8_t>(clamped));
}

void OtosOdometer::setAngularScalar(float scalar)
{
    if (!initialized_) return;
    int8_t clamped = static_cast<int8_t>(clampf(scalar, -127.0f, 127.0f));
    writeReg8(kRegAngularScalar, static_cast<uint8_t>(clamped));
}

// ---------------------------------------------------------------------------
// scaleToRegister -- ported from OtosSensor::scaleToInt8().
// ---------------------------------------------------------------------------

int8_t OtosOdometer::scaleToRegister(float scale)
{
    // scalar = clamp(round((scale - 1.0) / 0.001), -127, 127).
    // E.g. 1.067 -> +67; 0.987 -> -13.
    float raw = roundf((scale - 1.0f) / 0.001f);
    if (raw >  127.0f) raw =  127.0f;
    if (raw < -127.0f) raw = -127.0f;
    return static_cast<int8_t>(raw);
}

// ---------------------------------------------------------------------------
// Private register-map helpers.
// ---------------------------------------------------------------------------

void OtosOdometer::writeReg8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    // 086-007: postClear=kBusClearance -- see file header's "086-007 HITL
    // fix" section and nezha_motor.cpp's writeMotorRun()/requestEncoder()
    // for the proven precedent this mirrors.
    bus_.write(static_cast<uint16_t>(kOtosDeviceAddr << 1), buf, 2, false,
               /*preClear=*/0, /*postClear=*/kBusClearance);
}

uint8_t OtosOdometer::readReg8(uint8_t reg)
{
    uint8_t result = 0;
    // 086-007: the register-select write gets postClear=kBusClearance; the
    // following read gets preClear=kBusClearance so it waits out the
    // write's own settle window before issuing -- see file header.
    bus_.write(static_cast<uint16_t>(kOtosDeviceAddr << 1), &reg, 1, false,
               /*preClear=*/0, /*postClear=*/kBusClearance);
    bus_.read(static_cast<uint16_t>(kOtosDeviceAddr << 1), &result, 1, false,
              /*preClear=*/kBusClearance, /*postClear=*/0);
    return result;
}

bool OtosOdometer::readPositionVelocity(int16_t& x, int16_t& y, int16_t& h,
                                         int16_t& vx, int16_t& vy, int16_t& vh)
{
    // 086-007: ONE 12-byte burst read replaces the former two separate
    // 6-byte readXYH() bursts (position, then velocity) -- kRegPositionXl
    // (0x20, 6 bytes) and kRegVelocityXl (0x26, 6 bytes) are CONTIGUOUS
    // registers, so a single register-select write to kRegPositionXl
    // followed by a 12-byte auto-increment read covers both, halving this
    // leaf's per-tick transaction count (and thus its clearance cost). The
    // register-select write gets postClear=kBusClearance; the following
    // read gets preClear=kBusClearance so it waits out the write's own
    // settle window before issuing -- see file header's "086-007 HITL fix"
    // section and nezha_motor.cpp's writeMotorRun()/requestEncoder() for
    // the proven precedent this mirrors.
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

    return (writeStatus == MICROBIT_OK && readStatus == MICROBIT_OK);
}

void OtosOdometer::writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h)
{
    uint8_t buf[7];
    buf[0] = startReg;
    buf[1] = static_cast<uint8_t>(x & 0xFF);
    buf[2] = static_cast<uint8_t>((x >> 8) & 0xFF);
    buf[3] = static_cast<uint8_t>(y & 0xFF);
    buf[4] = static_cast<uint8_t>((y >> 8) & 0xFF);
    buf[5] = static_cast<uint8_t>(h & 0xFF);
    buf[6] = static_cast<uint8_t>((h >> 8) & 0xFF);
    // 086-007: postClear=kBusClearance -- see writeReg8()'s own comment.
    bus_.write(static_cast<uint16_t>(kOtosDeviceAddr << 1), buf, 7, false,
               /*preClear=*/0, /*postClear=*/kBusClearance);
}

}  // namespace Hal
