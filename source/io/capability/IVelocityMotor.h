#pragma once
#include <stdint.h>

struct RobotConfig;

/**
 * IVelocityMotor — drive-wheel capability (039-001).
 *
 * Phase A introduces this header as the canonical name for the drive-wheel
 * interface.  During the transition (T1) the body is the verbatim former
 * IMotor interface, and `source/hal/IMotor.h` becomes a `using IMotor =
 * IVelocityMotor;` shim so every existing consumer (Motor, MockMotor,
 * MotorController, Robot) compiles unchanged.  The capability-typed method
 * rename (setOutput / positionMm / velocityMmps / tick) and the split-phase
 * move into the Motor impl land in T2/T3 — bodies are not changed here.
 *
 * Allows MotorController and Robot to be written against the interface
 * rather than the concrete Motor class, enabling test doubles and future
 * alternative hardware back-ends.
 */
class IVelocityMotor {
public:
    virtual ~IVelocityMotor() = default;

    // Prime/initialize the motor at boot (e.g. encoder readback). Default no-op.
    // Concrete implementations (Motor) override this to prime the hardware encoder
    // so the first read after boot returns valid data, not the frozen-at-zero
    // value the Nezha 0x46 register exhibits before its first atomic read.
    virtual void begin() {}

    // Set speed as signed percentage (-100..100). Positive = logical forward.
    virtual void setSpeed(int8_t pct) = 0;

    // Split-phase encoder I/O, phase 1: issue the 0x46 write and return.
    virtual void requestEncoder() = 0;

    // Split-phase encoder I/O, phase 2: read back the 4-byte response.
    virtual int32_t collectEncoder() const = 0;

    // High-resolution encoder read in mm as float (used by velocity loop).
    // readEncoderMmF is the generic name; concrete variants below for
    // specific timing contexts.
    virtual float readEncoderMmF(const RobotConfig& cfg) const = 0;

    // Atomic single-shot encoder read (~8 ms, safe outside control loop).
    virtual float readEncoderMmFAtomic(const RobotConfig& cfg) const = 0;

    // Settle-only encoder read (~4 ms, safe inside fixed-rate control loop).
    virtual float readEncoderMmFSettle(const RobotConfig& cfg) const = 0;

    // Zero this motor's encoder accumulator (software offset reset).
    virtual void resetEncoder() = 0;
};
