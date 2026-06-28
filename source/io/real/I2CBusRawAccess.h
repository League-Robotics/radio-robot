#pragma once
#include "io/capability/IRawBusAccess.h"

class I2CBus;

/**
 * I2CBusRawAccess — IRawBusAccess adapter over the shared I2CBus (044-003,
 * Phase F).
 *
 * Wraps the same I2CBus the motors/sensors use and exposes raw byte-level
 * read/write for the DebugCommands I2CW / I2CR handlers, so DebugCommands
 * holds an IRawBusAccess* instead of an I2CBus*.  This is the adapter that
 * seals the final vendor leak above source/io/.
 *
 * write/read forward verbatim to I2CBus::write/read (same 8-bit wire address,
 * same repeated-start semantics, same CODAL status return) so the I2CW / I2CR
 * replies are byte-identical to the prior direct I2CBus access.
 *
 * Owned by NezhaHAL as a value member (no heap), constructed from the same
 * I2CBus.  Firmware-only (wraps the CODAL-backed I2CBus); not compiled in the
 * host build.
 */
class I2CBusRawAccess : public IRawBusAccess {
public:
    explicit I2CBusRawAccess(I2CBus& bus);

    int write(uint16_t addr8, const uint8_t* data, int len,
              bool repeated = false) override;
    int read(uint16_t addr8, uint8_t* buf, int len) override;

private:
    I2CBus& _bus;
};
