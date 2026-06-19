#pragma once
#include <stdint.h>

/**
 * IRawBusAccess — raw I2C byte-level read/write access (044-003, Phase F).
 *
 * Implemented by I2CBusRawAccess (source/io/real/) wrapping the shared I2CBus.
 * Used ONLY by DebugCommandable for the I2CW / I2CR diagnostic handlers.
 *
 * Separating raw read/write from IBusDiagnostics keeps each capability cohesive:
 * IBusDiagnostics exposes read-only statistics; IRawBusAccess mutates the bus.
 * This seals the final vendor (I2CBus) leak above source/io/ — DebugCommandable
 * holds an IRawBusAccess* instead of an I2CBus*.
 *
 * Address convention matches I2CBus::write/read: addr8 is the 8-bit wire address
 * (7-bit address left-shifted by one), as the callers pass it.  Return values are
 * the underlying CODAL status int (0 == MICROBIT_OK on success).
 */
class IRawBusAccess {
public:
    virtual ~IRawBusAccess() = default;

    // Write len bytes from data to addr8 (8-bit shifted wire address).
    // If repeated=true, issue a repeated-start instead of a stop.
    // Returns 0 on success, non-zero CODAL status on error.
    virtual int write(uint16_t addr8, const uint8_t* data, int len,
                      bool repeated = false) = 0;

    // Read len bytes from addr8 into buf.
    // Returns 0 on success, non-zero CODAL status on error.
    virtual int read(uint16_t addr8, uint8_t* buf, int len) = 0;
};
