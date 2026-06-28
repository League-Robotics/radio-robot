#include "MotorBusDiagnostics.h"
#include "I2CBus.h"

MotorBusDiagnostics::MotorBusDiagnostics(I2CBus& bus) : _bus(bus) {}

// --- Original 039-001 surface (bound to the motor controller 0x10) ---

uint32_t MotorBusDiagnostics::errorCount() const
{
    return _bus.errCount(kMotorAddr);
}

uint32_t MotorBusDiagnostics::reentryViolations() const
{
    return _bus.reentryViolations();
}

uint32_t MotorBusDiagnostics::lastError() const
{
    // I2CBus::lastErr returns int (CODAL status); cast to the interface's
    // uint32_t. MotorController casts it back to int at the snprintf %d site so
    // the EVT enc_wedged line is byte-identical to the prior direct read.
    return static_cast<uint32_t>(_bus.lastErr(kMotorAddr));
}

// --- Added 044-003 (Phase F): full diagnostic surface for DebugCommands ---
// Each method forwards verbatim to the same-named I2CBus method so the DBG
// replies are byte-identical to the prior direct I2CBus reads.

uint32_t MotorBusDiagnostics::txnCount(uint8_t addr7) const
{
    return _bus.txnCount(addr7);
}

uint32_t MotorBusDiagnostics::errCount(uint8_t addr7) const
{
    return _bus.errCount(addr7);
}

int8_t MotorBusDiagnostics::lastErr(uint8_t addr7) const
{
    // I2CBus::lastErr returns int (small CODAL status); narrow to int8_t. The
    // DBG I2C stats line prints it with %d (int promotion), so the value is
    // byte-identical to the prior direct ctx.bus->lastErr(addr) read.
    return static_cast<int8_t>(_bus.lastErr(addr7));
}

void MotorBusDiagnostics::resetStats()
{
    _bus.resetStats();
}

void MotorBusDiagnostics::setLogging(bool on)
{
    _bus.setLogging(on);
}

void MotorBusDiagnostics::dumpRecent(DumpFn fn, void* ctx) const
{
    _bus.dumpRecent(fn, ctx);
}

bool MotorBusDiagnostics::irqGuard() const
{
    return _bus.irqGuard();
}

void MotorBusDiagnostics::setIrqGuard(bool on)
{
    _bus.setIrqGuard(on);
}
