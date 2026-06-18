#include "OtosSensor.h"
#include "Config.h"
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

    // Tell the CHIP its mounting offset (REG_OFFSET 0x10) so it attributes the
    // rotation-induced arc to yaw.  A sensor mounted off the center of rotation
    // sees lateral optical flow during a turn; without telling the chip, its
    // fusion mistakes that for translation, under-reports heading, and the robot
    // over-turns.  With the offset set, the chip subtracts the expected arc
    // (omega x r) and reports the TRACKING-CENTER pose directly.  The chip is
    // mounted aligned to the robot frame (silicon X=fwd, Y=left -> odomYawDeg=0),
    // so the offset is odomOffX/Y rotated into chip coords by odomYawDeg; the
    // angular offset is 0 (sensor aligned with its own frame).  Raw LSB = mm *
    // (kMeterToInt16/1000) = mm * 3.2768.  This replaces the host-side lever-arm.
    {
        float a   = _cfg.odomYawDeg * (3.14159265f / 180.0f);
        float ca  = cosf(a), sa = sinf(a);
        float chipX = ca * _cfg.odomOffX - sa * _cfg.odomOffY;   // robot -> chip
        float chipY = sa * _cfg.odomOffX + ca * _cfg.odomOffY;
        constexpr float kMmToInt16 = 32768.0f / 10.0f / 1000.0f;  // 3.2768 LSB/mm
        long ox = lroundf(chipX * kMmToInt16);
        long oy = lroundf(chipY * kMmToInt16);
        if (ox >  32767) ox =  32767;  if (ox < -32767) ox = -32767;
        if (oy >  32767) oy =  32767;  if (oy < -32767) oy = -32767;
        writeXYH(REG_OFFSET_XL, (int16_t)ox, (int16_t)oy, 0);
    }

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
    for (uint32_t waited = 0; waited < kImuCalibTimeoutMs; waited += 4u) {
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

bool OtosSensor::readTransformed(const RobotConfig& cfg, OtosPose& poseOut,
                                  float headingRad) const
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

    if (cfg.odomUpsideDown) {
        xF = -xF;
        yF = -yF;
        hF = -hF;
    }

    float angRad = -cfg.odomYawDeg * (3.14159265f / 180.0f);
    float c = cosf(angRad);
    float s = sinf(angRad);

    // Mounting offset (lever-arm) is now applied IN THE CHIP via REG_OFFSET (set in
    // begin()), so the chip already reports the TRACKING-CENTER pose.  Do NOT
    // subtract a host-side lever-arm here too — that would double-correct.
    // odomYawDeg still rotates the chip-native frame into the robot frame (a no-op
    // when the chip is mounted aligned, odomYawDeg=0).  headingRad is unused now.
    (void)headingRad;
    poseOut.x = c * xF - s * yF;
    poseOut.y = s * xF + c * yF;
    poseOut.h = hF;
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
//   After applying the mounting rotation (odomYawDeg flip+rotation), the
//   OTOS x-axis points forward in the robot body frame.  Body speed is
//   taken as vx_body (the forward-axis component) rather than the vector
//   magnitude |v|.  For a differential-drive robot vy should be near zero;
//   using vx avoids sign ambiguity from sqrtf and correctly handles both
//   forward and reverse motion.  omega_rads comes directly from the heading
//   channel of the velocity register (deg/s → rad/s), sign-preserved.
// ---------------------------------------------------------------------------

bool OtosSensor::readVelocityTransformed(const RobotConfig& cfg, OtosVelocity& velOut,
                                          float /*headingRad*/) const
{
    if (!is_initialized()) {
        velOut = {0.0f, 0.0f};
        return false;
    }

    int16_t rvx = 0, rvy = 0, rvh = 0;
    readXYH(REG_VELOCITY_XL, rvx, rvy, rvh);

    // N9 (030-008): _lastReadOk updated by readXYH above.  Return false on
    // I2C failure so the caller does not fuse a zero velocity into the EKF.
    if (!_lastReadOk) {
        velOut = {0.0f, 0.0f};
        return false;
    }

    // Same LSB resolution as position (see comment block above).
    constexpr float kVelMmpsPerLsb   = 0.305f;           // mm/s per LSB
    constexpr float kOmegaRadpsPerLsb = 0.00549f * (3.14159265f / 180.0f); // rad/s per LSB

    float vxF = static_cast<float>(rvx) * kVelMmpsPerLsb;
    float vyF = static_cast<float>(rvy) * kVelMmpsPerLsb;
    float whF = static_cast<float>(rvh) * kOmegaRadpsPerLsb;

    if (cfg.odomUpsideDown) {
        vxF = -vxF;
        vyF = -vyF;
        whF = -whF;
    }

    float angRad = -cfg.odomYawDeg * (3.14159265f / 180.0f);
    float c = cosf(angRad);
    float s = sinf(angRad);

    // Rotate into body frame; vx_body is the forward-axis projection.
    // odomYawDeg is a constant mounting offset to heading; its derivative is
    // zero, so omega passes through unchanged (after the flip above).
    float vxBody = c * vxF - s * vyF;

    velOut.v_mmps     = vxBody;
    velOut.omega_rads = whF;
    return true;
}

OtosAccel OtosSensor::readAccelTransformed(const RobotConfig& cfg) const
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

    if (cfg.odomUpsideDown) {
        axF = -axF;
        ayF = -ayF;
    }

    float angRad = -cfg.odomYawDeg * (3.14159265f / 180.0f);
    float c = cosf(angRad);
    float s = sinf(angRad);

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

void OtosSensor::setWorldPose(const RobotConfig& cfg,
                              float x_mm, float y_mm, float h_rad)
{
    // Exact inverse of readTransformed(): find the chip-frame raw pose that reads
    // back as world (x_mm, y_mm, h_rad).  Used to anchor the OTOS to a camera fix
    // (SI) so its absolute observations agree with the controller pose instead of
    // dragging the EKF toward the boot frame.
    //
    // Forward (readTransformed), with angRad = -odomYawDeg (lever-arm now in-chip):
    //   poseOut.x = c*xF - s*yF ;  poseOut.y = s*xF + c*yF ;  poseOut.h = hF
    //   (xF,yF,hF already negated when odomUpsideDown)
    // Inverse: un-rotate by R(-angRad), undo upside-down, convert mm/rad -> raw LSBs.
    if (!is_initialized()) return;

    // No host-side lever-arm: the chip applies REG_OFFSET, so its POSITION register
    // is the tracking-CENTER pose.  Write the world center pose directly; only the
    // odomYawDeg frame rotation below remains (a no-op when the chip is aligned).
    float px = x_mm;
    float py = y_mm;

    float angRad = -cfg.odomYawDeg * (3.14159265f / 180.0f);
    float c = cosf(angRad);
    float s = sinf(angRad);
    // (xF,yF) = R(-angRad) * (px,py)
    float xF =  c * px + s * py;
    float yF = -s * px + c * py;
    float hF = h_rad;

    if (cfg.odomUpsideDown) { xF = -xF; yF = -yF; hF = -hF; }

    constexpr float kPosMmPerLsb  = 0.305f;
    constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);
    long rx = lroundf(xF / kPosMmPerLsb);
    long ry = lroundf(yF / kPosMmPerLsb);
    long rh = lroundf(hF / kHdgRadPerLsb);
    if (rx >  32767) rx =  32767;  if (rx < -32767) rx = -32767;
    if (ry >  32767) ry =  32767;  if (ry < -32767) ry = -32767;
    if (rh >  32767) rh =  32767;  if (rh < -32767) rh = -32767;
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
