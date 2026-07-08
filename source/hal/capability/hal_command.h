// hal_command.h — the shared hardware command-in edge types (sprint 079,
// architecture-update.md "The command-edge types"): the addressed-motor
// shape both of the hardware subsystem's command-in edges share, and the two
// <Producer>To<Consumer>Command structs (naming-and-style.md rule 4) that
// carry it from the two producers who address Subsystems::NezhaHardware's
// ports.
//
// Placement: these live in Hal::capability, NOT next to either producer
// (subsystems/drivetrain.h, commands/dev_commands.h) and NOT beside the
// consumer (subsystems/nezha_hardware.h). Keeping the edge structs in the
// shared, data-only Hal::capability tier — the tier every higher layer
// already depends on (via capability/motor.h and messages/*.h) — means
// neither producer nor the consumer has to #include the other: the consumer
// Subsystems::NezhaHardware names DrivetrainToHardwareCommand in its own
// apply() overload without pulling in subsystems/drivetrain.h, so no
// Drivetrain <-> NezhaHardware mutual include is created between the two
// sibling subsystems. (Putting these in commands/ instead would be a worse
// coupling, since commands/ sits above both Subsystems and Hal.) This is a
// data-only addition to Hal::capability, not a new kind of dependency:
// Subsystems already depends on Hal, never the reverse.
//
// Historical note: NezhaHardware used to be Hal::NezhaHal, and this placement
// originally also avoided a Hal -> Subsystems include (the aggregator lived
// in namespace Hal then). It moved to Subsystems, so that specific inversion
// is no longer the reason — but the sibling-mutual-include reason above still
// makes Hal::capability the right, stable home.
//
// Headers-only, no hal_command.cpp — consistent with the rest of
// capability/ (see capability/motor.h's file header, capability/ports.h's
// "declared, not defined" note).
#pragma once

#include <stdint.h>

#include "messages/motor.h"

namespace Hal {

// A MotorCommand plus which port it targets. Shared shape for both of
// NezhaHardware's command-in edges: the processor's addressed single-motor
// traffic (CommandProcessorToHardwareCommand) and the Drivetrain's governed
// wheel pair (DrivetrainToHardwareCommand) both address NezhaHardware's ports
// through this same pair.
struct AddressedMotorCommand {
  uint32_t port = 0;   // 1..Subsystems::NezhaHardware::kPortCount
  msg::MotorCommand command;
};

// CommandProcessorToHardwareCommand -- <Producer>To<Consumer>Command (rule 4).
// count in {0,1,2}: `DEV M <n>` stages 1; `DEV DT STOP` stages 2 (the bound
// pair, addressed -- NOT the same as allPorts); `DEV STOP` / the watchdog
// use allPorts (count ignored, addressed[0].command applied to every port
// NezhaHardware owns; the port field is unused for a broadcast). A broadcast
// reaches every port's Hal::Motor setter unconditionally, regardless of that
// port's own I2C flip-flop poll-schedule membership (091-002:
// NezhaHardware::apply() no longer touches poll state in any branch — see
// nezha_hardware.h's own file header).
struct CommandProcessorToHardwareCommand {
  bool allPorts = false;
  uint8_t count = 0;
  AddressedMotorCommand addressed[2];
};

// DrivetrainToHardwareCommand -- <Producer>To<Consumer>Command (rule 4). The
// Drivetrain's governed two-wheel target, addressed via DrivetrainConfig's
// port binding (Subsystems::Drivetrain::ports()). wheel[0] is left,
// wheel[1] is right — differential today; mecanum's 4 fits the same shape
// (a future ticket, not this sprint).
struct DrivetrainToHardwareCommand {
  AddressedMotorCommand wheel[2];   // [0]=left, [1]=right
};

}  // namespace Hal
