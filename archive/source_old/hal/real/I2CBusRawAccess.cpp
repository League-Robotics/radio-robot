#include "I2CBusRawAccess.h"
#include "I2CBus.h"

I2CBusRawAccess::I2CBusRawAccess(I2CBus& bus) : _bus(bus) {}

int I2CBusRawAccess::write(uint16_t addr8, const uint8_t* data, int len,
                           bool repeated)
{
    // I2CBus::write takes a non-const uint8_t* (mirroring MicroBitI2C); the
    // payload is never mutated by a write transaction, so the const_cast is
    // safe and preserves byte-identical behavior with the prior direct call.
    return _bus.write(addr8, const_cast<uint8_t*>(data), len, repeated);
}

int I2CBusRawAccess::read(uint16_t addr8, uint8_t* buf, int len)
{
    return _bus.read(addr8, buf, len);
}
