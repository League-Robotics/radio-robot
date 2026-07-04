#pragma once
#include <stdint.h>
#include "hal/capability/IVelocityMotor.h"

// ---------------------------------------------------------------------------
// NoopVelocityMotor — do-nothing IVelocityMotor for stubs and default HAL
// slots.
//
// Refactored from ReplayHAL.h (046-003) into a shared header so that
// Hardware.h and MecanumHAL can use it without pulling in ReplayHAL.
// ReplayHAL.h includes this header and uses it via inheritance (no callers
// need to change their own includes).
// ---------------------------------------------------------------------------
class NoopVelocityMotor : public IVelocityMotor {
public:
    void    setSpeed(int8_t pct) override           { (void)pct; }
    float   position()     const override           { return 0.0f; }
    float   velocityMmps() const override           { return 0.0f; }
    void    requestEncoder() override               {}
    int32_t collectEncoder() const override         { return 0; }
    float   readEncoder(const RobotConfig& cfg) const override {
        (void)cfg; return 0.0f;
    }
    float   readEncoderAtomic(const RobotConfig& cfg) const override {
        (void)cfg; return 0.0f;
    }
    float   readEncoderSettle(const RobotConfig& cfg) const override {
        (void)cfg; return 0.0f;
    }
    void    resetEncoder() override                 {}
    void    rebaselineSoft() override                {}
};
