// nezha_hal.h — NezhaHal: owns the shared I2CBus plus one NezhaMotor per
// port (up to four), and orchestrates the split-phase bus schedule across
// them.
//
// Design Rationale 3 (architecture-update.md): NezhaHal is a small, dumb
// owner/factory over up to four ports — no left/right pairing, no port-
// role special-casing. That belongs one tier up (Drivetrain / the DEV
// command family's PORTS binding, both later tickets). NezhaHal only knows
// about ports.
#pragma once

#include <stdint.h>

#include "com/i2c_bus.h"
#include "hal/capability/motor.h"
#include "hal/nezha/nezha_motor.h"
#include "messages/motor.h"

namespace Hal {

class NezhaHal {
 public:
  static constexpr uint32_t kPortCount = 4;

  // configs must supply exactly kPortCount entries; configs[i].port should
  // equal i+1 (1..4) — the constructing caller's (main.cpp, ticket 5's)
  // responsibility. NezhaHal does not itself validate or force this,
  // consistent with "no NezhaHal-level special-casing."
  NezhaHal(I2CBus& bus, const msg::MotorConfig configs[kPortCount]);

  // Primes all four ports' encoders (see NezhaMotor::begin()).
  void begin();

  // Ticks all four ports in a fixed, deterministic ascending-port order
  // (1, 2, 3, 4). source_old/robot/NezhaHAL.h's "right-before-left"
  // convention existed for determinism, not a specific priority (per this
  // ticket's acceptance criteria); ascending port order preserves that
  // determinism at a tier with no L/R concept.
  void tick(uint32_t now);   // [ms]

  // Port-indexed accessor, port in [1, kPortCount]. Always returns the
  // Hal::Motor faceplate — callers (DEV commands, Drivetrain; both later
  // tickets) never see NezhaMotor's raw register verbs. Out-of-range ports
  // clamp to port 4 rather than trapping, since a bad port from a DEV
  // command should surface as ERR at the command layer, not crash the
  // firmware.
  Motor& motor(uint32_t port);

 private:
  NezhaMotor motor1_;
  NezhaMotor motor2_;
  NezhaMotor motor3_;
  NezhaMotor motor4_;
};

}  // namespace Hal
