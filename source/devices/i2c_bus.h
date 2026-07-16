// i2c_bus.h — Devices::I2CBus: pure abstract I2C bus interface.
//
// Sprint 108 ticket 001 (clasi/issues/plan-pure-i2cbus-clock-interfaces-a-
// real-simplant-simulator.md, Stage 1). This used to be a concrete class
// with two build-mode-conditional forks (real MicroBitI2C machinery vs. a
// scripted-FIFO test fake). It is now a plain virtual base — same style as
// `App::Transport` (source/app/comms.h) and `Devices::MotorArmor`
// (source/devices/motor_armor.h) — so this header never drags in
// MicroBit.h and has zero preprocessor forks. The real ARM
// implementation lives in `source/devices/microbit_i2c_bus.h/.cpp`
// (`Devices::MicroBitI2CBus`).
//
// Surface: exactly the 3 methods any command handler or device leaf calls
// today (grep-confirmed against source/) — write(), read(), and
// clearanceSafetyNetCount(). Every other member the old concrete class
// exposed (txnCount/errCount/lastErr/clear/reentryViolations/
// reentryInFlightAddr/reentryNewAddr/resetStats/dumpRecent/setLogging/
// setIrqGuard/irqGuard) is only ever called from i2c_bus.cpp/.h themselves
// (now microbit_i2c_bus.cpp/.h) and stays on the concrete
// `Devices::MicroBitI2CBus` class, not this interface.
//
// All four device leaves this subsystem owns (Motor, Otos, LineSensorLeaf,
// ColorSensorLeaf) route their bus traffic through an `I2CBus&` reference —
// they are unaffected by this split.
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
  // Clearance safety-net diagnostics (103-002, M1 fix — 2026-07-13 code
  // review). Total number of times write()/read() found itself called
  // BEFORE a device's clearance deadline (readyAt/preClear) had elapsed.
  // This is the narrow signal ticket 001 (103) numbered as
  // Telemetry.fault_bits bit 0 ("I2CBus readyAt clearance safety-net trip")
  // — it "should never fire if the loop schedule is right." Read by
  // source/app/robot_loop.cpp each cycle to populate that fault bit.
  // ---------------------------------------------------------------------

  virtual uint32_t clearanceSafetyNetCount() const = 0;
};

}  // namespace Devices
