#pragma once
#include <stdint.h>
#include "../IMotor.h"

#ifdef HOST_BUILD
#include <random>
#endif

struct RobotConfig;

/**
 * MockMotor — host-compilable IMotor implementation for unit tests.
 *
 * Integrates commanded speed into an encoder accumulator on each tick(dt_ms).
 * Physics: encoderMm += (cmdSpeed / 100.0) * kNominalMaxMms * offsetFactor * (dt_ms / 1000.0)
 *
 * Slip model: encoder under-reports by (slipStraight + slipTurnExtra * turnRate).
 * Gaussian noise: std::normal_distribution<float> applied per tick (host-only).
 *
 * No CODAL dependency. Compiles with plain clang++ -std=c++11 -I source.
 */
class MockMotor : public IMotor {
public:
    static constexpr float kNominalMaxMms = 400.0f;

    // IMotor interface -------------------------------------------------------
    void    setSpeed(int8_t pct) override;
    void    requestEncoder() override;
    int32_t collectEncoder() const override;
    float   readEncoderMmF(const RobotConfig& cfg) const override;
    float   readEncoderMmFAtomic(const RobotConfig& cfg) const override;
    float   readEncoderMmFSettle(const RobotConfig& cfg) const override;
    void    resetEncoder() override;

    // Per-loop sensor tick (039-002): promote the integrated _encoderMm into the
    // positionMm() accessor.  This is a COPY only — the encoder integration lives
    // in integrate(dt_ms) (driven by MockHAL::tick(now,cmds)); tick(now_ms) must
    // NOT re-integrate or the sim would double-count (OQ-2).  No-op otherwise.
    void    tick(uint32_t now_ms) override;
    float   positionMm()   const override { return _lastPositionMm; }
    float   velocityMmps() const override { return _lastVelocityMmps; }

    // Simulation control -----------------------------------------------------

    // Advance physics by dt_ms milliseconds (encoder integration).  Renamed from
    // tick(dt_ms) in 039-002 to free the tick() name for the IVelocityMotor
    // sensor-tick override above; called by MockHAL::advance.
    void integrate(uint32_t dt_ms);

    // Test accessors
    float  encoderMm()        const { return _encoderMm; }
    int8_t cmdSpeed()         const { return _cmdSpeed; }
    float  trueVelocityMms()  const { return _trueVelMms; }

    // Inject a per-wheel speed offset factor (default 1.0 = symmetric).
    void setOffsetFactor(float f) { _offsetFactor = f; }

    // Set fractional slip: encoder under-reports by (straight + turnExtra * turnRate).
    void setSlip(float straight, float turnExtra) {
        _slipStraight   = straight;
        _slipTurnExtra  = turnExtra;
    }

    // Set Gaussian encoder noise standard deviation (mm per tick).
    void setEncoderNoise(float sigmaMm) { _encoderNoiseSigma = sigmaMm; }

    // Set current turn rate in [0, 1], updated by MockHAL before each tick().
    void setTurnRate(float r) { _turnRate = r; }

private:
    int8_t  _cmdSpeed          = 0;
    float   _encoderMm         = 0.0f;
    float   _offsetFactor      = 1.0f;

    // ---- tick() cache (039-002) ----
    // Promoted from _encoderMm by tick(now_ms); read by positionMm()/velocityMmps().
    float    _lastPositionMm   = 0.0f;
    float    _lastVelocityMmps = 0.0f;
    uint32_t _lastTickMs       = 0;
    bool     _hasLastTick      = false;

    float   _turnRate          = 0.0f;
    float   _slipStraight      = 0.0f;
    float   _slipTurnExtra     = 0.0f;
    float   _encoderNoiseSigma = 0.0f;
    float   _trueVelMms        = 0.0f;

#ifdef HOST_BUILD
    std::mt19937 _rng{42u};
#endif
};
