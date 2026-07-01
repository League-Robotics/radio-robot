#pragma once
#include <stdint.h>
#include "hal/capability/IOdometer.h"
#include "PhysicsWorld.h"

#ifdef HOST_BUILD
#include <random>
#endif

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
 * No CODAL dependency.  Compiles with plain clang++ -std=c++11 -I source.
 */
class SimOdometer : public IOdometer {
public:
    explicit SimOdometer(const PhysicsWorld& plant) : _plant(plant) {}

    // IOdometer interface ----------------------------------------------------
    bool readTransformed(Pose2D& poseOut, float headingRad = 0.0f) const override;
    bool readVelocityTransformed(BodyTwist& velOut,
                                 float headingRad = 0.0f) const override;
    BodyAccel readAccelTransformed() const override;

    bool readStatus(uint8_t& out) const override {
        // LIFT (robot lifted) → INVALID status (0xFF). Read failure → failure too.
        if (_lift || _readFailure) { out = 0xFF; return false; }
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

    // Accumulated OTOS odometry (sim-model output; back-compat sim_get_otos_*).
    float odomX() const { return _odomX; }
    float odomY() const { return _odomY; }
    float odomH() const { return _odomH; }

    // Advance the OTOS integration model by one step (driven by SimHardware).
    // velLMms/velRMms: true (pre-slip) wheel velocities, mm/s; tw: trackwidth mm.
    // Bit-identical to the retired MockOtosSensor::tick.
    void tick(float velLMms, float velRMms, float trackwidthMm, uint32_t dt_ms);

private:
    const PhysicsWorld& _plant;   // ground-truth read access (forward-compat)

    float   _injectedX     = 0.0f;
    float   _injectedY     = 0.0f;
    float   _injectedH     = 0.0f;
    bool    _readFailure   = false;
    bool    _lift          = false;
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
