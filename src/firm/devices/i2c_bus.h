// i2c_bus.h — Devices::I2CBus: pure abstract I2C bus interface. Plain
// virtual base (no preprocessor forks); the real ARM implementation lives
// in microbit_i2c_bus.h/.cpp (Devices::MicroBitI2CBus).
//
// Surface: exactly the 3 methods any command handler or device leaf calls
// — write(), read(), and clearanceSafetyNetCount(). The diagnostic/stats
// surface (txnCount/errCount/lastErr/clear/reentryViolations/resetStats/
// dumpRecent/setLogging/setIrqGuard/irqGuard) stays on the concrete
// Devices::MicroBitI2CBus class, not this interface.
//
// Design/rationale: DESIGN.md.
#pragma once
#include <cstdint>

namespace Devices {

class I2CBus {
 public:
  virtual ~I2CBus() = default;

  // ---------------------------------------------------------------------
  // I2C forwarding — mirror MicroBitI2C signatures exactly.
  //
  // address: 8-bit wire address (7-bit addr << 1), as the callers pass it.
  // Returns: CODAL status int (MICROBIT_OK == 0 on success).
  //
  // preClear/postClear (// [us], default 0): lazy per-device clearance
  // timers implemented by the concrete class. Defaults collapse the entry
  // deadline to lastEnd, already in the past by the next call, so every
  // 4-argument call site is byte-identical to before this parameter pair
  // existed.
  // ---------------------------------------------------------------------

  virtual int write(uint16_t address, uint8_t* data, int len,
                     bool repeated = false, uint32_t preClear = 0,
                     uint32_t postClear = 0) = 0;
  virtual int read(uint16_t address, uint8_t* data, int len,
                    bool repeated = false, uint32_t preClear = 0,
                    uint32_t postClear = 0) = 0;

  // ---------------------------------------------------------------------
  // Clearance safety-net diagnostics. Total number of times write()/
  // read() found itself called BEFORE a device's clearance deadline
  // (readyAt/preClear) had elapsed — should never fire if the loop
  // schedule is right. Read by app/robot_loop.cpp each cycle to populate
  // Telemetry.fault_bits bit 0.
  // ---------------------------------------------------------------------

  virtual uint32_t clearanceSafetyNetCount() const = 0;
};

}  // namespace Devices
