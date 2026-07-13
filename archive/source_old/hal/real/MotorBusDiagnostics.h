#pragma once
#include "hal/capability/IBusDiagnostics.h"

class I2CBus;

/**
 * MotorBusDiagnostics — IBusDiagnostics adapter over the shared I2CBus
 * (039-001; extended 044-003 Phase F).
 *
 * Forwards the original three diagnostic getters the MotorController needs when
 * emitting EVT enc_wedged, bound to the Nezha motor-controller device 0x10, so
 * the controller no longer references I2CBus directly:
 *   errorCount()        -> I2CBus::errCount(0x10)
 *   reentryViolations() -> I2CBus::reentryViolations()
 *   lastError()         -> I2CBus::lastErr(0x10)  (cast int -> uint32_t)
 *
 * Sprint 044 (Phase F) extends the adapter to forward the full diagnostic
 * surface the DebugCommands DBG handlers (DBG I2C / DBG I2CLOG / DBG IRQGUARD)
 * use, so DebugCommands holds an IBusDiagnostics* instead of an I2CBus*.
 * These take an explicit 7-bit address and forward to the same-named I2CBus
 * methods, so the DBG replies are byte-identical to the prior direct I2CBus reads.
 *
 * Owned by NezhaHAL as a value member (no heap), constructed from the same
 * I2CBus the motors use.
 */
class MotorBusDiagnostics : public IBusDiagnostics {
public:
    explicit MotorBusDiagnostics(I2CBus& bus);

    // --- Original 039-001 surface (bound to the motor controller 0x10) ---
    uint32_t errorCount() const override;
    uint32_t reentryViolations() const override;
    uint32_t lastError() const override;

    // --- Added 044-003 (Phase F): full diagnostic surface for DebugCommands ---
    uint32_t txnCount(uint8_t addr7) const override;
    uint32_t errCount(uint8_t addr7) const override;
    int8_t   lastErr(uint8_t addr7) const override;
    void     resetStats() override;
    void     setLogging(bool on) override;
    void     dumpRecent(DumpFn fn, void* ctx) const override;
    bool     irqGuard() const override;
    void     setIrqGuard(bool on) override;

private:
    // Nezha motor controller I2C device address (7-bit), matching the address
    // MotorController previously passed to errCount/lastErr directly.
    static constexpr uint16_t kMotorAddr = 0x10;
    I2CBus& _bus;
};
