#pragma once
#include <stdint.h>

/**
 * IBusDiagnostics — bus-health capability (039-001; extended 044-003 Phase F).
 *
 * Exposes the per-bus diagnostic counters the layers above source/io/ need
 * WITHOUT leaking the concrete I2CBus (or any other vendor bus type) above the
 * IO boundary.  A MotorBusDiagnostics adapter (source/io/real/) forwards these
 * to the live I2CBus.
 *
 * The original three getters (039-001) return the values MotorController reads
 * when emitting EVT enc_wedged, bound to the motor-controller device (0x10):
 *   errorCount()        -> I2CBus::errCount(0x10)
 *   reentryViolations() -> I2CBus::reentryViolations()
 *   lastError()         -> I2CBus::lastErr(0x10)
 *
 * Sprint 044 (Phase F) extends the interface with the full diagnostic surface
 * the DebugCommands DBG handlers (DBG I2C / DBG I2CLOG / DBG IRQGUARD) need,
 * so DebugCommands no longer references I2CBus directly.  These take an
 * explicit 7-bit device address (the addr the DBG I2C stats line iterates over):
 *   txnCount(addr7)  -> I2CBus::txnCount(addr7)
 *   errCount(addr7)  -> I2CBus::errCount(addr7)
 *   lastErr(addr7)   -> I2CBus::lastErr(addr7)
 *   resetStats()     -> I2CBus::resetStats()
 *   setLogging(on)   -> I2CBus::setLogging(on)
 *   dumpRecent(fn,c) -> I2CBus::dumpRecent(fn, c)
 *   irqGuard()       -> I2CBus::irqGuard()
 *   setIrqGuard(on)  -> I2CBus::setIrqGuard(on)
 *
 * Any future bus type (SPI, CAN) that implements this interface works with the
 * control layer and DebugCommands without further changes.
 */
class IBusDiagnostics {
public:
    // dumpRecent reply callback — a bare function-pointer typedef, matching the
    // ReplyFn shape from types/Protocol.h, declared inline so this capability
    // header takes no dependency on the command-dispatch layer (044-003 T3).
    using DumpFn = void (*)(const char* msg, void* ctx);

    virtual ~IBusDiagnostics() = default;

    // --- Original 039-001 surface (read by MotorController via _busDiag) ---
    virtual uint32_t errorCount() const = 0;
    virtual uint32_t reentryViolations() const = 0;
    virtual uint32_t lastError() const = 0;

    // --- Added in Sprint 044 (Phase F) to seal DebugCommands's I2CBus leak ---
    virtual uint32_t txnCount(uint8_t addr7) const = 0;
    virtual uint32_t errCount(uint8_t addr7) const = 0;
    virtual int8_t   lastErr(uint8_t addr7) const = 0;
    virtual void     resetStats() = 0;
    virtual void     setLogging(bool on) = 0;
    virtual void     dumpRecent(DumpFn fn, void* ctx) const = 0;
    virtual bool     irqGuard() const = 0;
    virtual void     setIrqGuard(bool on) = 0;
};
