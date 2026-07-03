#include "OtosSensor.h"
#include "Config.h"
#include "hal/capability/OtosLeverArm.h"
#include <cmath>

OtosSensor::OtosSensor(I2CBus& i2c, const RobotConfig& cfg)
    : _i2c(i2c), _cfg(cfg)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

int8_t OtosSensor::scaleToInt8(float scale)
{
    // scalar = clamp(round((scale - 1.0) / 0.001), -127, 127).
    // E.g. 1.05 → +50; 0.987 → -13.
    float raw = roundf((scale - 1.0f) / 0.001f);
    if (raw >  127.0f) raw =  127.0f;
    if (raw < -127.0f) raw = -127.0f;
    return static_cast<int8_t>(raw);
}

bool OtosSensor::begin()
{
    // Detect via product-ID read.
    uint8_t id = readReg8(REG_PRODUCT_ID);
    _initialized = (id == EXPECTED_PRODUCT_ID);
    if (!_initialized) return false;

    // Enable signal processing + reset Kalman tracking, then apply the
    // linear/angular calibration scalars from config.
    init();
    setLinearScalar(scaleToInt8(_cfg.otosLinearScale));
    setAngularScalar(scaleToInt8(_cfg.otosAngularScale));

    // Mounting offset (lever-arm) is applied HOST-SIDE in readTransformed(), NOT
    // via the chip's REG_OFFSET.  This OTOS unit silently ignores writes to the
    // offset register block 0x10-0x15 (the I2C write ACKs, but the register reads
    // back 0 — verified directly, while position 0x20 and scalars 0x04/0x05 write
    // and hold fine).  So we cannot offload the lever-arm to the chip; do not write
    // REG_OFFSET here (it would be a silent no-op).  See readTransformed().

    // Zero the OTOS position AND heading at boot so it starts at the same origin
    // as the freshly-zeroed encoders.  The OTOS chip retains its tracked pose
    // across a micro:bit reset/reflash (only a power cycle or an explicit write
    // clears it), so without this the EKF fuses a STALE OTOS pose (seen: −728mm,
    // −92°) against the encoders' (0,0,0) origin — corrupting the initial heading
    // and sending GO_TO off in the wrong direction.  Done after init() so the
    // zeroed heading is taken with the IMU bias already calibrated.
    setPositionRaw(0, 0, 0);
    return true;
}

void OtosSensor::init()
{
    if (!is_initialized()) return;
    // Enable all signal processing: LUT | Accel | Rotation | Variance = 0x0F.
    writeReg8(REG_SIGNAL_PROCESS_CFG, 0x0F);
    // Reset Kalman tracking (bit 0 = 1).
    writeReg8(REG_RESET, 0x01);

    // Calibrate the IMU bias (gyro + accelerometer) — the robot MUST be still.
    // Writing the sample count to REG_IMU_CALIBRATION starts a background
    // calibration; the OTOS decrements the register to 0 as it collects samples
    // (~3 ms each, 255 → ~0.77 s).  Without this the gyro/accel bias is
    // uncorrected: the heading drifts and warnTiltAngle (STATUS bit 0) stays set
    // (which the fusion-health gate rejects).  Block until done or a generous
    // timeout.  Runs at boot via begin() (after the 2.5 s sensor settle, robot
    // still) and on the OI command (deliberate re-calibration — keep it still).
    calibrateImu(kImuCalibSamples);
    for (uint32_t waited = 0; waited < kImuCalibTimeout; waited += 4u) {
        if (readReg8(REG_IMU_CALIBRATION) == 0) break;  // calibration complete
        fiber_sleep(4);
    }
}

void OtosSensor::calibrateImu(uint8_t samples)
{
    if (!is_initialized()) return;
    writeReg8(REG_IMU_CALIBRATION, samples);
}

void OtosSensor::resetTracking()
{
    if (!is_initialized()) return;
    writeReg8(REG_RESET, 0x01);
}

bool OtosSensor::readTransformed(OtosPose& poseOut,
                                  float heading) const  // [rad]
{
    if (!is_initialized()) {
        poseOut = {0.0f, 0.0f, 0.0f};
        return false;
    }

    int16_t rx = 0, ry = 0, rh = 0;
    readXYH(REG_POSITION_XL, rx, ry, rh);

    // N9 (030-008): _lastReadOk is updated by readXYH above (same tick).
    // Return false immediately if the burst read failed — caller must not fuse
    // a zero-filled {0,0,0} pose into the EKF.
    if (!_lastReadOk) {
        poseOut = {0.0f, 0.0f, 0.0f};
        return false;
    }

    constexpr float kPosMmPerLsb  = 0.305f;
    constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);

    float xF = static_cast<float>(rx) * kPosMmPerLsb;
    float yF = static_cast<float>(ry) * kPosMmPerLsb;
    float hF = static_cast<float>(rh) * kHdgRadPerLsb;

    if (_cfg.odomUpsideDown) {
        xF = -xF;
        yF = -yF;
        hF = -hF;
    }

    float ang = -_cfg.odomYaw * (3.14159265f / 180.0f);  // [rad]
    float c = cosf(ang);
    float s = sinf(ang);

    // Lever-arm correction (HOST-SIDE, source/hal/capability/OtosLeverArm.h).
    // The chip-side REG_OFFSET approach is unusable: this OTOS unit silently
    // ignores writes to the offset register 0x10-0x15 (ACKs them, reads back
    // 0), while position/scalars write fine — so the chip reports the SENSOR
    // pose, not the tracking-CENTER.  Recover the robot centre via
    // sensorToCentre(), which subtracts the mounting offset rotated by the
    // SAME-INSTANT OTOS heading hF (read in the same I2C burst as rx,ry):
    //   sensor  = pivot + R(hF)*odomOff      (the chip reports the sensor arc)
    //   centre  = sensor - R(hF)*odomOff = pivot   (exact cancellation)
    // hF (not the caller's heading) is required: heading is the fused pose
    // heading from the PREVIOUS tick and lags the spin by a constant ~ω*dt (≈29°
    // over the slow relay), leaving a residual lever-arm circle (the corner
    // "D-hook").  hF is same-instant so the arc cancels regardless of spin rate.
    // (An earlier hF test "spiralled" only because the offset was wrong (-24 vs the
    // measured -47.7) AND the run was cramped against the fence — not hF's fault.)
    (void)heading;
    float rotX = c * xF - s * yF;
    float rotY = s * xF + c * yF;
    float centreX = 0.0f, centreY = 0.0f;
    sensorToCentre(rotX, rotY, hF, _cfg.odomOffX, _cfg.odomOffY, centreX, centreY);
    poseOut.x = centreX;
    poseOut.y = centreY;
    poseOut.h = hF;   // heading takes no mounting offset
    return true;
}

// ---------------------------------------------------------------------------
// OTOS LSB Scale Factors (Sprint 023)
//
// The SparkFun OTOS chip uses the same signed-int16 register layout for
// position, velocity, and acceleration.  All three register banks share
// identical LSB resolutions:
//
//   Linear:  1 LSB = 1/32768 m  =  ~0.030518 mm  ≈ 0.305 mm / 10
//            BUT the chip operates on a 10-LSB-per-unit scale internally,
//            so effective linear scale = 0.305 mm/LSB  (= 1/32768 m * 10).
//   Angular: 1 LSB = 0.00549 deg  (= 1/32768 rev * 65.536, ~360/65536 deg)
//
// Sources:
//   • SparkFun OTOS Arduino library (SparkFun_Optical_Tracking_Odometry_Sensor.h):
//       kMeterToInt16 = 32768/meter → 1 LSB = 1/32768 m ≈ 0.030518 mm
//       The library further scales by INT_TO_FLOAT_METERS = 1/kMeterToInt16
//       and the internal factor of 10 used by readPosition/readVelocity, so
//       effective per-register scale = 0.305 mm/LSB (matches kPosMmPerLsb).
//   • The velocity registers (0x26) and acceleration registers (0x2C) use the
//     SAME bit-for-bit LSB encoding as the position registers (0x20), so
//     kVelMmpsPerLsb = kPosMmPerLsb = 0.305 mm/s/LSB and
//     kAccMmps2PerLsb = kPosMmPerLsb = 0.305 mm/s²/LSB.
//   • Angular rate uses kHdgDegPerLsb = 0.00549 deg/LSB → same value for
//     omega (deg/s per LSB); converted to rad/s below.
//
// Body-frame v derivation:
//   After applying the mounting rotation (odomYaw flip+rotation), the
//   OTOS x-axis points forward in the robot body frame.  Body speed is
//   taken as vx_body (the forward-axis component) rather than the vector
//   magnitude |v|.  For a differential-drive robot vy should be near zero;
//   using vx avoids sign ambiguity from sqrtf and correctly handles both
//   forward and reverse motion.  omega_rads comes directly from the heading
//   channel of the velocity register (deg/s → rad/s), sign-preserved.
// ---------------------------------------------------------------------------

bool OtosSensor::readVelocityTransformed(OtosVelocity& velOut,
                                          float heading) const  // [rad]
{
    // Delegate to the 3-DOF read and discard the lateral (vy) component.
    // A differential-drive robot cannot act on vy, but the OTOS measures it
    // regardless; dropping it here keeps the 2-DOF caller contract unchanged
    // while sharing one read/scale/flip/rotate path (no duplicated logic).
    BodyTwist3 t3;
    if (!readVelocityTransformed3(t3, heading)) {
        velOut = {0.0f, 0.0f};
        return false;
    }

    velOut.v_mmps     = t3.vx_mmps;
    velOut.omega_rads = t3.omega_rads;
    return true;
}

// ---------------------------------------------------------------------------
// readVelocityTransformed3 — 3-DOF velocity read (046-006).
//
// Reads the REG_VELOCITY_XL burst and returns all three body-frame components:
// vx (forward), vy (lateral), omega (yaw).  Applies the upside-down flip and
// odomYaw mounting rotation.  This is the OTOS driver's full-fidelity
// velocity read; the OTOS measures all three DOF independently of the robot's
// drivetrain, so this method is unconditional (no drivetrain dependency).
// Whether the lateral channel is *used* is a decision for the estimator/EKF
// upstream (see Robot.cpp), not for the sensor layer.
//
// The OTOS chip's internal mount-offset compensation (written to REG_OFFSET in
// begin()) correctly handles the lever arm, so no extra lever-arm code is
// needed here.  omega passes through unchanged (the mounting-offset derivative
// is zero — only the rotation angle matters for the linear components).
// ---------------------------------------------------------------------------
bool OtosSensor::readVelocityTransformed3(BodyTwist3& velOut,
                                           float /*heading*/) const  // [rad]
{
    if (!is_initialized()) {
        velOut = {0.0f, 0.0f, 0.0f};
        return false;
    }

    int16_t rvx = 0, rvy = 0, rvh = 0;
    readXYH(REG_VELOCITY_XL, rvx, rvy, rvh);

    if (!_lastReadOk) {
        velOut = {0.0f, 0.0f, 0.0f};
        return false;
    }

    // Same LSB resolution as position (see comment block above readVelocityTransformed).
    constexpr float kVelMmpsPerLsb    = 0.305f;
    constexpr float kOmegaRadpsPerLsb = 0.00549f * (3.14159265f / 180.0f);

    float vxF = static_cast<float>(rvx) * kVelMmpsPerLsb;
    float vyF = static_cast<float>(rvy) * kVelMmpsPerLsb;
    float whF = static_cast<float>(rvh) * kOmegaRadpsPerLsb;

    if (_cfg.odomUpsideDown) {
        vxF = -vxF;
        vyF = -vyF;
        whF = -whF;
    }

    float ang = -_cfg.odomYaw * (3.14159265f / 180.0f);  // [rad]
    float c = cosf(ang);
    float s = sinf(ang);

    // Rotate chip-native (vxF, vyF) into robot body frame.
    // odomYaw is a constant mounting offset; its derivative is zero,
    // so omega passes through unchanged (same reasoning as readVelocityTransformed).
    float vxBody = c * vxF - s * vyF;
    float vyBody = s * vxF + c * vyF;

    velOut.vx_mmps    = vxBody;
    velOut.vy_mmps    = vyBody;
    velOut.omega_rads = whF;
    return true;
}

OtosAccel OtosSensor::readAccelTransformed() const
{
    if (!is_initialized()) return {0.0f, 0.0f};

    int16_t rax = 0, ray = 0, rah = 0;
    readXYH(REG_ACCELERATION_XL, rax, ray, rah);

    // Same LSB resolution as position (see comment block above).
    constexpr float kAccMmps2PerLsb = 0.305f;  // mm/s² per LSB

    float axF = static_cast<float>(rax) * kAccMmps2PerLsb;
    float ayF = static_cast<float>(ray) * kAccMmps2PerLsb;
    // rah (angular acceleration) is discarded — only linear acceleration is used.
    (void)rah;

    if (_cfg.odomUpsideDown) {
        axF = -axF;
        ayF = -ayF;
    }

    float ang = -_cfg.odomYaw * (3.14159265f / 180.0f);  // [rad]
    float c = cosf(ang);
    float s = sinf(ang);

    OtosAccel accel;
    accel.ax_mmps2 = c * axF - s * ayF;
    accel.ay_mmps2 = s * axF + c * ayF;
    return accel;
}

void OtosSensor::getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const
{
    if (!is_initialized()) { x = 0; y = 0; h = 0; return; }
    readXYH(REG_POSITION_XL, x, y, h);
}

void OtosSensor::setPositionRaw(int16_t x, int16_t y, int16_t h)
{
    if (!is_initialized()) return;
    writeXYH(REG_POSITION_XL, x, y, h);
}

void OtosSensor::setWorldPose(float x, float y, float h)  // [mm], [mm], [rad]
{
    // Exact inverse of readTransformed(): find the chip-frame raw pose that reads
    // back as world (x, y, h).  Used to anchor the OTOS to a camera fix
    // (SI) so its absolute observations agree with the controller pose instead of
    // dragging the EKF toward the boot frame.
    //
    // Forward (readTransformed): poseOut = R(ang)*(xF,yF) - R(h)*offset,
    //   ang = -odomYaw ; offset = [odomOffX, odomOffY] (host-side lever-arm).
    // Inverse: add the lever-arm back (the chip POSITION reg stores the SENSOR
    // pose), then un-rotate by R(-ang), undo upside-down, convert -> raw LSBs.
    if (!is_initialized()) return;

    // Host-side lever-arm (chip can't store REG_OFFSET on this unit): the SENSOR
    // sits at center + R(h)*offset, so to make readTransformed report the world
    // CENTER (x,y) we must set the chip POSITION to the sensor point.
    // centreToSensor() (source/hal/capability/OtosLeverArm.h) is the exact
    // inverse of readTransformed()'s sensorToCentre() call above.
    float px = 0.0f, py = 0.0f;
    centreToSensor(x, y, h, _cfg.odomOffX, _cfg.odomOffY, px, py);

    float ang = -_cfg.odomYaw * (3.14159265f / 180.0f);  // [rad]
    float c = cosf(ang);
    float s = sinf(ang);
    // (xF,yF) = R(-ang) * (px,py)
    float xF =  c * px + s * py;
    float yF = -s * px + c * py;
    float hF = h;

    if (_cfg.odomUpsideDown) { xF = -xF; yF = -yF; hF = -hF; }

    constexpr float kPosMmPerLsb  = 0.305f;
    constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);
    long rx = lroundf(xF / kPosMmPerLsb);
    long ry = lroundf(yF / kPosMmPerLsb);
    long rh = lroundf(hF / kHdgRadPerLsb);
    if (rx >  32767) rx =  32767;
    if (rx < -32767) rx = -32767;
    if (ry >  32767) ry =  32767;
    if (ry < -32767) ry = -32767;
    if (rh >  32767) rh =  32767;
    if (rh < -32767) rh = -32767;
    writeXYH(REG_POSITION_XL, (int16_t)rx, (int16_t)ry, (int16_t)rh);
}

void OtosSensor::getVelocityRaw(int16_t& x, int16_t& y, int16_t& h) const
{
    if (!is_initialized()) { x = 0; y = 0; h = 0; return; }
    readXYH(REG_VELOCITY_XL, x, y, h);
}

int8_t OtosSensor::getLinearScalar() const
{
    if (!is_initialized()) return 0;
    return (int8_t)readReg8(REG_LINEAR_SCALAR);
}

void OtosSensor::setLinearScalar(int8_t val)
{
    if (!is_initialized()) return;
    writeReg8(REG_LINEAR_SCALAR, (uint8_t)val);
}

int8_t OtosSensor::getAngularScalar() const
{
    if (!is_initialized()) return 0;
    return (int8_t)readReg8(REG_ANGULAR_SCALAR);
}

void OtosSensor::setAngularScalar(int8_t val)
{
    if (!is_initialized()) return;
    writeReg8(REG_ANGULAR_SCALAR, (uint8_t)val);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void OtosSensor::writeReg8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    _i2c.write((ADDR << 1), (uint8_t*)buf, 2, false);
}

uint8_t OtosSensor::readReg8(uint8_t reg) const
{
    uint8_t result = 0;
    // repeated=true → no STOP after the register write (repeated-start read).
    _i2c.write((ADDR << 1), (uint8_t*)&reg, 1, false);
    _i2c.read((ADDR << 1), (uint8_t*)&result, 1, false);
    return result;
}

void OtosSensor::readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h) const
{
    uint8_t raw[6] = {0};
    // repeated=true → no STOP after the register write (repeated-start read).
    int wStatus = _i2c.write((ADDR << 1), (uint8_t*)&startReg, 1, false);
    int rStatus = _i2c.read((ADDR << 1), (uint8_t*)raw, 6, false);
    // MICROBIT_OK == 0 (CODAL convention).
    _lastReadOk = (wStatus == 0 && rStatus == 0);
    x = (int16_t)(raw[0] | ((uint16_t)raw[1] << 8));
    y = (int16_t)(raw[2] | ((uint16_t)raw[3] << 8));
    h = (int16_t)(raw[4] | ((uint16_t)raw[5] << 8));
}

bool OtosSensor::readStatus(uint8_t& out) const
{
    if (!is_initialized()) { out = 0xFF; return false; }
    // Write the register address.
    uint8_t reg = REG_STATUS;
    int wStatus = _i2c.write((ADDR << 1), &reg, 1, false);
    int rStatus = _i2c.read((ADDR << 1), &out, 1, false);
    return (wStatus == 0 && rStatus == 0);
}

void OtosSensor::writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h)
{
    uint8_t buf[7];
    buf[0] = startReg;
    buf[1] = (uint8_t)(x & 0xFF);
    buf[2] = (uint8_t)((x >> 8) & 0xFF);
    buf[3] = (uint8_t)(y & 0xFF);
    buf[4] = (uint8_t)((y >> 8) & 0xFF);
    buf[5] = (uint8_t)(h & 0xFF);
    buf[6] = (uint8_t)((h >> 8) & 0xFF);
    _i2c.write((ADDR << 1), (uint8_t*)buf, 7, false);
}
