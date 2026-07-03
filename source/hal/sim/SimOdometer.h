#pragma once
#include <stdint.h>
#include "hal/capability/IOdometer.h"
#include "PhysicsWorld.h"

#ifdef HOST_BUILD
#include <random>
#endif

struct RobotConfig;   // fwd decl — reads odomOffX/odomOffY for the lever-arm round-trip

/**
 * SimOdometer — observation model for the OTOS odometer (Sprint 040 Phase B,
 * 040-002).
 *
 * Implements IOdometer.  Holds a `const PhysicsWorld&` (ground truth) and owns the
 * sensor error.  Replaces BOTH the retired MockOtosSensor (dual-accumulator OTOS +
 * injected pose) and the BenchOtosSensor's role in SIM mode.
 *
 * Behaviour preservation (OQ-1 / golden-TLM): the sim-model integration path is
 * the EXACT MockOtosSensor::tick body — same per-tick true-velocity arc, same
 * std::mt19937{43u} noise stream, same heading wrap.  readTransformed returns the
 * injected pose by default, or the accumulated noisy odom pose when the sim model
 * is enabled (enableSimModel(true)) — matching MockOtosSensor exactly so the OTOS
 * fusion / sensor-freshness tests pass unchanged.  (No golden-TLM command sequence
 * calls OZ/OV, so this preservation guarantee is unaffected by the setPositionRaw
 * fix below — see ticket 063-006.)
 *
 * setPositionRaw() re-references the `_odomX/_odomY/_odomH` accumulator (not just
 * the `_rawX/_rawY/_rawH` shadow), at parity with the real OtosSensor chip: on
 * hardware, setPositionRaw() writes the chip's POSITION registers directly, and
 * readTransformed() re-reads and rescales those SAME registers every tick, so
 * there is no separate accumulator to fall out of sync there.  `OZ`
 * (setPositionRaw(0,0,0)) therefore actually zeroes the pose the EKF fuses via
 * Robot::otosCorrect(), matching hardware (ticket 063-006).
 *
 * Every error setter defaults to a no-op, so a fresh SimOdometer is PERFECT.
 *
 * Ground-truth sampling + lever arm (ticket 066-001, CR-07/CR-08): tick() no
 * longer re-integrates commanded wheel speeds (which made the sim OTOS
 * structurally incapable of disagreeing with the encoders — see the retired
 * comment this replaces). It instead samples PhysicsWorld::truePoseX/Y/H()
 * each tick, computes the delta since the previous sample, and applies the
 * SAME noise/drift/scale-error knobs to that delta as before — only the
 * delta's SOURCE changed (plant-pose differencing instead of wheel-velocity
 * kinematics), so any chassis-truth slip configured via sim_set_motor_slip
 * now shows up in the OTOS accumulator exactly as it does in plant truth,
 * matching real hardware (the OTOS IS the ground-truth-tracking sensor).
 * readTransformed() projects the accumulated centre estimate through
 * centreToSensor() then sensorToCentre() (source/hal/capability/OtosLeverArm.h
 * — the SAME shared math OtosSensor::readTransformed() uses) before returning,
 * so the host-side lever-arm compensation a past hardware regression
 * (db11b7c) broke is now sim-reachable.  The constructor takes a
 * `const RobotConfig&` (mirrors OtosSensor) to read odomOffX/odomOffY for
 * that round-trip.
 *
 * No CODAL dependency.  Compiles with plain clang++ -std=c++11 -I source.
 */
class SimOdometer : public IOdometer {
public:
    SimOdometer(const PhysicsWorld& plant, const RobotConfig& cfg)
        : _plant(plant), _cfg(cfg) {}

    // IOdometer interface ----------------------------------------------------
    bool readTransformed(Pose2D& poseOut, float headingRad = 0.0f) const override;
    bool readVelocityTransformed(BodyTwist& velOut,
                                 float headingRad = 0.0f) const override;
    BodyAccel readAccelTransformed() const override;

    bool readStatus(uint8_t& out) const override {
        // LIFT (robot lifted) → INVALID status (0xFF). Read failure → failure too.
        if (_lift || _readFailure) { out = 0xFF; return false; }
        // WARN (readable-but-degraded, 065-006): warnOpticalTracking (bit 1),
        // matching the real OTOS chip's STATUS register — the reading is
        // still readable (return true) but callers gate fusion on it.
        if (_warnOptical) { out = 0x02; return true; }
        out = 0;
        return true;
    }
    bool lastReadOk() const override { return !_readFailure; }

    void init() override {}
    void calibrateImu(uint8_t samples) override { (void)samples; }
    void resetTracking() override {}

    void getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const override;

    // Writes the raw-register shadow (_rawX/_rawY/_rawH, read back by
    // getPositionRaw()) AND re-references the _odomX/_odomY/_odomH
    // accumulator that readTransformed() returns to the EKF, using the same
    // LSB scale (kPosMmPerLsb / kHdgRadPerLsb) as the real OtosSensor chip.
    // This is parity with hardware: the real chip's POSITION registers ARE
    // its accumulator (readTransformed() re-reads/rescales them every tick),
    // so writing them re-references the reported pose immediately. `OZ`
    // (setPositionRaw(0,0,0)) therefore zeroes the fused OTOS pose, not just
    // the shadow registers (ticket 063-006).
    void setPositionRaw(int16_t x, int16_t y, int16_t h) override;

    int8_t getLinearScalar()  const override { return _linearScalar; }
    void   setLinearScalar(int8_t val) override { _linearScalar = val; }
    int8_t getAngularScalar() const override { return _angularScalar; }
    void   setAngularScalar(int8_t val) override { _angularScalar = val; }

    // Sensor interface -------------------------------------------------------
    bool begin() override { _initialized = true; return true; }

    // Test injection / back-compat -------------------------------------------

    // Override the plant-truth read (back-compat for sim_set_otos_pose).
    void setInjectedPose(float x, float y, float h);

    // Deterministic read failure (back-compat for sim_set_otos_read_failure).
    void setReadFailure(bool fail) { _readFailure = fail; }

    float injectedX() const { return _injectedX; }
    float injectedY() const { return _injectedY; }
    float injectedH() const { return _injectedH; }

    // Error setters (all default no-op → a fresh sensor is PERFECT) -----------
    void enableSimModel(bool on)          { _useSimModel = on; }
    void setLinearNoiseSigma(float sigma) { _linearNoiseSigma = sigma; }
    void setYawNoiseSigma(float sigma)    { _yawNoiseSigma = sigma; }
    // Back-compat aliases (match the retired MockOtosSensor setter names).
    void setLinearNoise(float sigma)      { _linearNoiseSigma = sigma; }
    void setYawNoise(float sigma)         { _yawNoiseSigma = sigma; }
    // OTOS LIFT status (robot lifted; sensor returns INVALID).
    void setLift(bool on)                 { _lift = on; }
    // OTOS WARN status (065-006): readable-but-degraded (e.g. persistently
    // lifted / on the stand / freshly placed — warnOpticalTracking). Mirrors
    // setLift's shape but stays READABLE (readStatus returns true, out=0x02)
    // instead of INVALID; tick() freezes the pose accumulator and zeros
    // velocity/accel while set, modeling "frozen pose, near-zero velocity".
    void setWarnOptical(bool on)          { _warnOptical = on; }

    // Deterministic drift error: accumulated offset added per tick.
    // A fresh SimOdometer has zero drift (perfect sensor).
    // _driftPerTickMm is added to the linear odometry accumulator each tick;
    // _driftPerTickRad is added to the heading accumulator each tick.
    void setDriftPerTickMm(float mm)   { _driftPerTickMm = mm; }
    void setDriftPerTickRad(float rad) { _driftPerTickRad = rad; }

    // Scale error: multiplies the reported delta by (1 + error).
    // 0.0 = perfect, 0.05 = 5% scale error.
    void setLinearScaleError(float err)  { _linearScaleErr = err; }
    void setAngularScaleError(float err) { _angularScaleErr = err; }

    // Error-state accessors (069-004) — mirror the six setters immediately
    // above (setDriftPerTickMm/Rad, setLinearScaleError/setAngularScaleError,
    // setLinearNoiseSigma/setYawNoiseSigma). Write-only was fine when only
    // ctypes test code set these (057-005/058-001); the SIMSET/SIMGET wire
    // surface needs to read them back.
    float driftPerTickMm()    const { return _driftPerTickMm; }
    float driftPerTickRad()   const { return _driftPerTickRad; }
    float linearScaleError()  const { return _linearScaleErr; }
    float angularScaleError() const { return _angularScaleErr; }
    float linearNoiseSigma()  const { return _linearNoiseSigma; }
    float yawNoiseSigma()     const { return _yawNoiseSigma; }

    // Control-tick period (ms), read from the live RobotConfig this odometer
    // was constructed with (069-004) — NOT a copy, so a runtime `SET
    // ctrlPeriod=…` is reflected immediately (067's live-reference rule).
    // tick() adds the FULL _driftPerTickMm/_driftPerTickRad once per call,
    // and tick() fires once per RobotConfig::controlPeriod
    // (source/types/Config.h) — so this is "how many ms is one tick,"
    // used by SimCommands to convert the wire's per-second
    // otosLinDriftMmS/otosYawDriftDegS keys to/from this class's internal
    // per-tick representation. Out-of-line (SimOdometer.cpp) because the
    // header only forward-declares RobotConfig.
    int32_t controlPeriodMs() const;

    // Accumulated OTOS odometry (sim-model output; back-compat sim_get_otos_*).
    float odomX() const { return _odomX; }
    float odomY() const { return _odomY; }
    float odomH() const { return _odomH; }

    // Advance the OTOS integration model by one step (driven by SimHardware).
    // Samples PhysicsWorld::truePoseX/Y/H() (ground truth) and integrates the
    // delta since the previous sample into the noisy centre-frame accumulator
    // (ticket 066-001 — no longer takes wheel velocities; see class comment).
    void tick(uint32_t dt_ms);

private:
    const PhysicsWorld& _plant;   // ground-truth read access
    const RobotConfig&  _cfg;     // odomOffX/odomOffY for the lever-arm round-trip

    float   _injectedX     = 0.0f;
    float   _injectedY     = 0.0f;
    float   _injectedH     = 0.0f;
    bool    _readFailure   = false;
    bool    _lift          = false;
    bool    _warnOptical   = false;
    int16_t _rawX          = 0;
    int16_t _rawY          = 0;
    int16_t _rawH          = 0;
    int8_t  _linearScalar  = 0;
    int8_t  _angularScalar = 0;

    // Sim-model state (identical layout to MockOtosSensor).
    bool  _useSimModel      = false;
    float _linearNoiseSigma = 0.0f;
    float _yawNoiseSigma    = 0.0f;
    float _odomX            = 0.0f;
    float _odomY            = 0.0f;
    float _odomH            = 0.0f;

    // Ground-truth sampling baseline (ticket 066-001): the plant truePose*()
    // value as of the previous tick() call, used to compute the world-frame
    // delta to integrate this tick.  Rebaselined every tick() call (even when
    // the sim model is disabled or WARN-frozen) so re-enabling never produces
    // a single-tick "catch up" jump for motion that happened while disabled.
    float _prevTrueX = 0.0f;
    float _prevTrueY = 0.0f;
    float _prevTrueH = 0.0f;

    // Deterministic error model (ticket 057-005).
    // All zero by default → a fresh SimOdometer is perfect (no behaviour change).
    float _driftPerTickMm   = 0.0f;   // linear drift added to odomX accumulator per tick
    float _driftPerTickRad  = 0.0f;   // heading drift added to odomH per tick
    float _linearScaleErr   = 0.0f;   // fractional scale error on linear delta (0 = perfect)
    float _angularScaleErr  = 0.0f;   // fractional scale error on angular delta (0 = perfect)

    float _velV             = 0.0f;
    float _velOmega         = 0.0f;
    float _accAx            = 0.0f;
    float _accAy            = 0.0f;
    float _prevVelV         = 0.0f;

#ifdef HOST_BUILD
    std::mt19937 _rng{43u};
#endif
};
