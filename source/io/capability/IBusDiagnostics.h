#pragma once
#include <stdint.h>

/**
 * IBusDiagnostics — bus-health capability (039-001).
 *
 * Exposes the per-bus diagnostic counters the control layer needs when
 * emitting EVT enc_wedged, WITHOUT leaking the concrete I2CBus (or any other
 * vendor bus type) above the IO boundary.  A MotorBusDiagnostics adapter
 * (source/hal/MotorBusDiagnostics.*) forwards these to the live I2CBus for the
 * motor controller's device address (0x10).
 *
 * The three getters return the same values the MotorController previously read
 * directly from I2CBus:
 *   errorCount()        -> I2CBus::errCount(0x10)
 *   reentryViolations() -> I2CBus::reentryViolations()
 *   lastError()         -> I2CBus::lastErr(0x10)
 *
 * Any future bus type (SPI, CAN) that implements this interface works with
 * MotorController without further changes.
 */
class IBusDiagnostics {
public:
    virtual ~IBusDiagnostics() = default;
    virtual uint32_t errorCount() const = 0;
    virtual uint32_t reentryViolations() const = 0;
    virtual uint32_t lastError() const = 0;
};
