#include "MotorBusDiagnostics.h"
#include "I2CBus.h"

MotorBusDiagnostics::MotorBusDiagnostics(I2CBus& bus) : _bus(bus) {}

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
