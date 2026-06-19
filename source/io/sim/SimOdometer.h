#pragma once
#include <stdint.h>
#include "io/capability/IOdometer.h"
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
 * fusion / sensor-freshness tests pass unchanged.
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

    float _velV             = 0.0f;
    float _velOmega         = 0.0f;
    float _accAx            = 0.0f;
    float _accAy            = 0.0f;
    float _prevVelV         = 0.0f;

#ifdef HOST_BUILD
    std::mt19937 _rng{43u};
#endif
};
