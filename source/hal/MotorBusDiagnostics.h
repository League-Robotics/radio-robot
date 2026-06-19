#pragma once
#include "io/capability/IBusDiagnostics.h"

class I2CBus;

/**
 * MotorBusDiagnostics — IBusDiagnostics adapter over the shared I2CBus,
 * bound to the Nezha motor-controller device address 0x10 (039-001).
 *
 * Forwards the three diagnostic getters the MotorController needs when emitting
 * EVT enc_wedged, so the controller no longer references I2CBus directly:
 *   errorCount()        -> I2CBus::errCount(0x10)
 *   reentryViolations() -> I2CBus::reentryViolations()
 *   lastError()         -> I2CBus::lastErr(0x10)  (cast int -> uint32_t)
 *
 * Owned by NezhaHAL as a value member (no heap), constructed from the same
 * I2CBus the motors use.  The reported values are byte-identical to what
 * MotorController previously read directly from the I2CBus* it held.
 */
class MotorBusDiagnostics : public IBusDiagnostics {
public:
    explicit MotorBusDiagnostics(I2CBus& bus);
    uint32_t errorCount() const override;
    uint32_t reentryViolations() const override;
    uint32_t lastError() const override;

private:
    // Nezha motor controller I2C device address (7-bit), matching the address
    // MotorController previously passed to errCount/lastErr directly.
    static constexpr uint16_t kMotorAddr = 0x10;
    I2CBus& _bus;
};
