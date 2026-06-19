#pragma once
#include <stdint.h>
#include "Sensor.h"
#include "Pose2D.h"

struct RobotConfig;

/**
 * IOdometer — odometry capability (039-001).
 *
 * Phase A introduces this header as the canonical name for the odometry-sensor
 * interface.  During the transition (T1) the body is the verbatim former
 * IOtosSensor interface (still extending Sensor, still taking RobotConfig& in
 * its read signatures), and `source/hal/IOtosSensor.h` becomes a shim:
 *   using IOtosSensor  = IOdometer;
 *   using OtosPose     = Pose2D;
 *   using OtosVelocity = BodyTwist;
 *   using OtosAccel    = BodyAccel;
 * so every existing consumer (OtosSensor, MockOtosSensor, BenchOtosSensor,
 * Odometry, Robot) compiles unchanged.  The RobotConfig& seal (constructor
 * injection, cfg removed from public read signatures) lands in T4 — bodies are
 * not changed here.
 *
 * The value types Pose2D / BodyTwist / BodyAccel (from Pose2D.h) replace the
 * old OtosPose / OtosVelocity / OtosAccel structs.  Field layout is identical.
 *
 * Extends Sensor so that begin() and is_initialized() are provided by the
 * Sensor base; concrete classes inherit from IOdometer only (not both
 * Sensor and IOdometer) to avoid diamond inheritance.
 *
 * Includes the full sensor API (not just the minimal read/begin set) so that
 * Odometry command handlers (OI, OZ, OR, OP, OV, OL, OA) can reach calibration
 * and raw-position methods through the interface pointer without downcasting
 * to the concrete type.
 */
class IOdometer : public Sensor {
public:
    virtual ~IOdometer() = default;

    // Read position registers, apply transform from cfg, write result to poseOut.
    // Returns true if the underlying I2C burst read succeeded; false on I2C error
    // (poseOut receives {0,0,0} on failure — do NOT fuse a false return).
    // headingRad: current robot heading used for the lever-arm offset rotation.
    // No-op for zero offsets (as in tovez.json). Default 0.0f for callers that
    // do not yet supply heading.
    // N9 (030-008): return value enables the same-tick failure gate in
    // Robot::otosCorrect — callers must check the bool and skip fusion on false.
    virtual bool readTransformed(const RobotConfig& cfg, Pose2D& poseOut,
                                 float headingRad = 0.0f) const = 0;

    // Read velocity registers, apply transform from cfg, write result to velOut.
    // Returns true if the underlying I2C burst read succeeded; false on I2C error.
    // headingRad: see readTransformed (velocity lever-arm is near-zero in practice).
    virtual bool readVelocityTransformed(const RobotConfig& cfg, BodyTwist& velOut,
                                         float headingRad = 0.0f) const = 0;

    // Read the OTOS STATUS register (0x1F).
    // Returns true on I2C success; out receives the raw status byte (0 = valid).
    // Non-zero status means the OTOS tracking is invalid (e.g. sensor lifted).
    virtual bool readStatus(uint8_t& out) const = 0;

    // Returns true if the most recent burst read (readTransformed / readVelocityTransformed
    // / readAccelTransformed) completed without I2C error.
    virtual bool lastReadOk() const = 0;

    // Read acceleration registers, apply transform from cfg, and return BodyAccel.
    // ax_mmps2/ay_mmps2 are body-frame linear accelerations (angular discarded).
    virtual BodyAccel readAccelTransformed(const RobotConfig& cfg) const = 0;

    // Re-run device init (signal processing + Kalman reset). No-op if not inited.
    virtual void init() = 0;

    // Write N to the IMU calibration register.
    virtual void calibrateImu(uint8_t samples) = 0;

    // Reset Kalman tracking filters.
    virtual void resetTracking() = 0;

    // Raw position register access (signed int16 LSBs).
    virtual void getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const = 0;
    virtual void setPositionRaw(int16_t x, int16_t y, int16_t h) = 0;

    // Re-anchor the odometer to a WORLD-frame pose (a camera fix) so its absolute
    // position+heading observations AGREE with the controller pose instead of
    // dragging the EKF back toward the boot frame.  This is the exact inverse of
    // readTransformed() (un-rotates the mount angle, adds the lever-arm offset
    // back).  Units: x_mm/y_mm millimetres, h_rad radians (world frame).
    // Default no-op (mocks); the real and bench sensors override.
    virtual void setWorldPose(const RobotConfig& cfg,
                              float x_mm, float y_mm, float h_rad) {
        (void)cfg; (void)x_mm; (void)y_mm; (void)h_rad;
    }

    // Linear and angular scalar access (signed int8, 0.1% per LSB).
    virtual int8_t getLinearScalar() const = 0;
    virtual void   setLinearScalar(int8_t val) = 0;
    virtual int8_t getAngularScalar() const = 0;
    virtual void   setAngularScalar(int8_t val) = 0;
};
