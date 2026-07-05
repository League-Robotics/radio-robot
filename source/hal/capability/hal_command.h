// hal_command.h — the shared HAL command-in edge types (sprint 079,
// architecture-update.md "The command-edge types"): the addressed-motor
// shape both of the HAL's command-in edges share, and the two
// <Producer>To<Consumer>Command structs (naming-and-style.md rule 4) that
// carry it from the two producers who address Hal::NezhaHal's ports.
//
// Placement (Design Rationale 1 in architecture-update.md — do not
// relitigate here): these live in Hal::capability, NOT next to either
// producer (subsystems/drivetrain.h, commands/dev_commands.h), specifically
// so NezhaHal (the consumer, ticket 004) never has to #include
// "subsystems/drivetrain.h" to name DrivetrainToHalCommand in its own
// apply() overload — that would be a Hal -> Subsystems dependency, which
// does not exist anywhere else in this tree and inverts the established
// direction (Subsystems depends on Hal, never the reverse). Putting these
// in commands/ instead would be an even worse inversion, since commands/
// sits above both Hal and Subsystems. Hal::capability is the shared, stable
// tier both Hal::nezha and Subsystems::Drivetrain already depend on (via
// capability/motor.h and messages/*.h) — this is a data-only addition to
// that same tier, not a new kind of coupling.
//
// Headers-only, no hal_command.cpp — consistent with the rest of
// capability/ (see capability/motor.h's file header, capability/ports.h's
// "declared, not defined" note).
#pragma once

#include <stdint.h>

#include "messages/motor.h"

namespace Hal {

// A MotorCommand plus which port it targets. Shared shape for both of the
// HAL's command-in edges: the processor's addressed single-motor traffic
// (CommandProcessorToHalCommand) and the Drivetrain's governed wheel pair
// (DrivetrainToHalCommand) both address the HAL's ports through this same
// pair.
struct AddressedMotorCommand {
  uint32_t port = 0;   // 1..NezhaHal::kPortCount
  msg::MotorCommand command;
};

// CommandProcessorToHalCommand -- <Producer>To<Consumer>Command (rule 4).
// count in {0,1,2}: `DEV M <n>` stages 1; `DEV DT STOP` stages 2 (the bound
// pair, addressed -- NOT the same as allPorts); `DEV STOP` / the watchdog
// use allPorts (count ignored, addressed[0].command applied to every port
// NezhaHal owns; the port field is unused for a broadcast). A broadcast does
// NOT mark any port in-use (ticket 004's NezhaHal::apply() — see
// architecture-update.md Design Rationale on why broadcast is exempt).
struct CommandProcessorToHalCommand {
  bool allPorts = false;
  uint8_t count = 0;
  AddressedMotorCommand addressed[2];
};

// DrivetrainToHalCommand -- <Producer>To<Consumer>Command (rule 4). The
// Drivetrain's governed two-wheel target, addressed via DrivetrainConfig's
// port binding (Subsystems::Drivetrain::ports()). wheel[0] is left,
// wheel[1] is right — differential today; mecanum's 4 fits the same shape
// (a future ticket, not this sprint).
struct DrivetrainToHalCommand {
  AddressedMotorCommand wheel[2];   // [0]=left, [1]=right
};

}  // namespace Hal
