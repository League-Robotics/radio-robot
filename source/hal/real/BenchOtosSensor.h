#pragma once
#include <stdint.h>
#include "hal/capability/IOdometer.h"

struct RobotConfig;

/**
 * BenchOtosSensor — IOtosSensor implementation for bench testing on a robot stand.
 *
 * The real OtosSensor sees no motion when the robot is on a stand.  This class
 * synthesises plausible OTOS output by integrating COMMANDED wheel velocity into
 * an ideal pose, then optionally adding sensor noise and slow yaw drift.  It lets
 * the full firmware stack (EKF, distance stops, TLM) run on the bench without I2C.
 *
 * Two independent accumulators:
 *   _idealX/Y/H  — noiseless arc-integration (ground truth for DBG OTOS BENCH)
 *   _otosX/Y/H   — errored integration (what readTransformed() returns)
 *
 * Error model (applied per tick):
 *   - Gaussian noise on the arc-distance dC  (sigma = _noiseXY mm/mm of motion)
 *   - Gaussian noise on the heading change dTh (sigma = _noiseH rad/rad of turn)
 *   - Slow yaw drift: _driftRadPerSec * dt_s added to _otosH each tick
 *
 * PRNG:
 *   HOST_BUILD — deterministic Box-Muller with a fixed-seed LCG (reproducible).
 *   Firmware   — microbit_random(int max) approximation via sum-of-uniforms
 *                (central-limit Gaussian; no std:: needed).
 *
 * WHY THIS LIVES IN source/hal/ AND NOT source/hal/mock/:
 *   It is a synthetic ("near-mock") sensor, but unlike the true mocks
 *   (MockOtosSensor, MockColorSensor, ...) it is NOT host-only.  It is compiled
 *   INTO the firmware (gated by BENCH_OTOS_ENABLED) and DEPENDS ON CODAL: the
 *   firmware PRNG path uses microbit_random() from MicroBitDevice.h.  NezhaHAL
 *   owns one as a member and `DBG OTOS BENCH` drives it live on the real robot
 *   stand.  The mock/ directory is deliberately host-only / CODAL-free and is
 *   FILTERED OUT of the firmware build — CMakeLists.txt drops every path under
 *   hal/mock/ via a list(FILTER SOURCE_FILES EXCLUDE REGEX ...) on that dir — so
 *   moving this file there would drop it from the firmware and break the
 *   NezhaHAL link.  It therefore lives here alongside the real OtosSensor; the
 *   host-sim build (which globs mock/) picks it up explicitly via
 *   tests/sim/CMakeLists.txt precisely because it is not under mock/.
 *
 * No I2C dependency.  begin() always succeeds; is_initialized() returns true.
 */
class BenchOtosSensor : public IOtosSensor {
public:
    BenchOtosSensor();

    // -------------------------------------------------------------------------
    // IOtosSensor interface
    // -------------------------------------------------------------------------

    // Returns the current errored pose (_otosX/Y/H) in poseOut.
    // Always returns true (bench sensor is always valid).
    // headingRad is ignored (no lever-arm correction in bench mode).
    bool readTransformed(OtosPose& poseOut,
                         float headingRad = 0.0f) const override;

    // Returns body-frame velocity derived from the last tick's errored arc step.
    // Always returns true.
    bool readVelocityTransformed(OtosVelocity& velOut,
                                 float headingRad = 0.0f) const override;

    // Returns true; out = 0 (always valid, never lifted).
    bool readStatus(uint8_t& out) const override;

    // Always returns true.
    bool lastReadOk() const override;

    // Returns {0, 0} — no accelerometer in bench mode.
    OtosAccel readAccelTransformed() const override;

    // Calibration stubs — no-ops (no IMU / tracking engine in bench mode).
    void   init() override {}
    void   calibrateImu(uint8_t samples) override { (void)samples; }
    void   resetTracking() override {}

    // Raw position access with register-scale parity to the real chip:
    // getPositionRaw reads back the errored accumulator; setPositionRaw
    // re-references BOTH accumulators (so `OZ` actually re-anchors the bench
    // frame — mirrors SimOdometer 063-006; see BenchOtosSensor.cpp).
    void   getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const override;
    void   setPositionRaw(int16_t x, int16_t y, int16_t h) override;
    void   setWorldPose(float x, float y, float h) override;  // [mm], [mm], [rad]

    int8_t getLinearScalar()         const override { return 0; }
    void   setLinearScalar(int8_t)         override {}
    int8_t getAngularScalar()        const override { return 0; }
    void   setAngularScalar(int8_t)        override {}

    // -------------------------------------------------------------------------
    // Sensor interface
    // -------------------------------------------------------------------------
    // Sets _initialized = true; always returns true (no I2C probe).
    bool begin() override;

    // -------------------------------------------------------------------------
    // Bench-specific interface (not on IOtosSensor)
    // -------------------------------------------------------------------------

    /**
     * tick — integrate one control step into both accumulators.
     *
     * velLeft, velRight : left/right wheel velocities, mm/s.
     * trackwidth        : wheel-to-wheel track width, mm.
     * dt_ms             : elapsed time for this step, ms.
     *
     * No-op when dt_ms == 0, trackwidth <= 0, or is_initialized() is false.
     */
    void tick(float velLeft, float velRight, float trackwidth, uint32_t dt_ms);  // [mm/s], [mm/s], [mm], [ms]

    /**
     * tickEncoder — integrate one control step from MEASURED cumulative wheel
     * travel (encoder mm), the preferred feed for bench mode.
     *
     * Integrating COMMANDED tgtSpeed made the bench pose blind to what the
     * wheels actually did — PID lag on ramp-up and, critically, ~10 mm/wheel
     * of coast past the stop-condition cutoff.  Measured on tovez
     * (2026-07-03, rotSlip=0): encoders executed ~92°/RT 9000 while the
     * commanded integral counted only ~82°; the EKF follows the OTOS heading,
     * so tours lost ~8°/turn and could not close.  Feeding encoder deltas
     * makes the bench OTOS an errored copy of encoder truth — the same
     * relationship SimOdometer has to plant truth in sim (ticket 066-001) —
     * so the two observation streams agree by construction and the injected
     * noise/drift model is the ONLY disagreement.
     *
     * encLeft/encRight are cumulative positions; this method owns the
     * previous-value baseline.  A per-tick step exceeding kMaxWheelMmps
     * × dt is treated as an encoder RESET (ZERO enc, per-drive-start
     * resetEncoder — events the real OTOS never sees): the baseline is
     * re-based and the step is NOT integrated.  The same clamp self-heals
     * the stale baseline on the first call after bench mode is re-enabled.
     *
     * encLeft, encRight : cumulative wheel travel, mm (Motor/SimMotor
     *                     position() — the value cached by this tick's
     *                     sensor read; no I2C).
     * trackwidth        : wheel-to-wheel track width, mm.
     * dt_ms             : elapsed time for this step, ms.
     */
    // holdHeading: integrate distance but hold heading this step — set while
    // a wheel-freeze (wedge latch) is detected, mirroring Odometry::predict's
    // dTheta hold so the bench frame cannot accumulate phantom rotation from
    // a frozen differential.
    void tickEncoder(float encLeft, float encRight, float trackwidth, uint32_t dt_ms,
                     bool holdHeading = false);  // [mm], [mm], [mm], [ms]

    /**
     * enable / enabled — gate whether tick() advances the accumulators.
     * When disabled the sensor is still initialized and returns the last pose.
     */
    void enable(bool on) { _enabled = on; }
    bool enabled()  const { return _enabled; }

    /**
     * setNoise — update the error model parameters at runtime.
     *
     * noiseXY       : per-tick linear noise sigma (fraction of arc distance).
     * noiseH        : per-tick yaw noise sigma (fraction of heading change).
     * driftRadPerSec: slow additive yaw drift added to _otosH every second.
     */
    void setNoise(float noiseXY, float noiseH, float driftRadPerSec);

    /** idealPose — fill out with the noiseless accumulator for DBG OTOS queries. */
    void idealPose(OtosPose& out) const;

    /** reset — zero both accumulators and the velocity/accel state. */
    void reset();

    // Noiseless accumulator accessors (convenience for tests).
    float idealX() const { return _idealX; }
    float idealY() const { return _idealY; }
    float idealH() const { return _idealH; }

    // Errored accumulator accessors (convenience for tests).
    float otosX() const { return _otosX; }
    float otosY() const { return _otosY; }
    float otosH() const { return _otosH; }

private:
    // Noiseless accumulator (ground truth).
    float _idealX = 0.0f;
    float _idealY = 0.0f;
    float _idealH = 0.0f;

    // Errored accumulator (what readTransformed returns).
    float _otosX  = 0.0f;
    float _otosY  = 0.0f;
    float _otosH  = 0.0f;

    // Error model parameters.
    float _noiseXY       = 0.0f;  // linear noise sigma (fraction of dC)
    float _noiseH        = 0.0f;  // yaw noise sigma (fraction of dTh)
    float _driftRadPerSec = 0.0f; // slow additive yaw drift, rad/s

    // Last-tick body-frame velocity/accel (updated by tick, returned by readVelocityTransformed).
    float _velV      = 0.0f;  // forward speed, mm/s
    float _velOmega  = 0.0f;  // yaw rate, rad/s
    float _accAx     = 0.0f;  // forward accel, mm/s^2
    float _prevVelV  = 0.0f;  // for finite-difference accel

    // tickEncoder baseline: previous cumulative wheel positions [mm].
    // _encBaselineValid gates the very first call (no delta to integrate).
    float _lastEncL         = 0.0f;
    float _lastEncR         = 0.0f;
    bool  _encBaselineValid = false;

    // Per-tick step ceiling for tickEncoder's reset clamp: no physical wheel
    // exceeds this speed, so a larger step is an encoder reset, not motion.
    static constexpr float kMaxWheelMmps = 2000.0f;  // [mm/s]

    bool _enabled = true;

    // HOST_BUILD PRNG state — deterministic LCG for reproducible simulation.
#ifdef HOST_BUILD
    mutable uint32_t _lcgState = 12345u;
    // Advance LCG and return a uniform float in [0, 1).
    float lcgRand() const;
    // Box-Muller normal variate with zero mean, given sigma.
    float gaussRand(float sigma) const;
#else
    // Firmware PRNG — sum-of-uniforms approximation via microbit_random.
    float gaussRandFW(float sigma) const;
#endif

    // Wrap angle to [-pi, pi].
    static float wrapAngle(float a);
};
