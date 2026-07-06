#pragma once

// ---------------------------------------------------------------------------
// otos_commands.h -- the seven-verb OTOS command family (084-008, SUC-007):
// OI/OZ/OR/OP/OV/OL/OA, already fully specified in docs/protocol-v2.md §11
// (grammar, reply shapes, ERR nodev behavior). Resolves Subsystems::
// Hardware::odometer() LIVE on every dispatch, never a construction-time-
// bound pointer -- mirrors source_old/commands/OtosCommands.h's own
// documented rationale for live resolution (a future odometer swap, e.g. a
// bench-vs-real re-seat, must not require touching this file).
//
// hardware.odometer() is nullptr on Subsystems::NezhaHardware (no real-
// hardware OTOS driver this program -- clasi/issues/
// nezha-hardware-otos-driver-for-new-source-tree.md) and non-null on
// Subsystems::SimHardware (Hal::SimOdometer, 081-003): every verb replies
// "ERR nodev <verb>" against the former, "OK ..." against the latter.
//
// OP is the one verb that does NOT set CMD_ACCESS_HARDWARE: it reads
// Hal::Odometer::pose(), a cheap accessor over SimOdometer's own
// accumulator -- never tick() -- so it is a read, not a hardware write, the
// same read/write distinction telemetry_commands.cpp's SNAP handler already
// draws for the identical pose() call (also CMD_NONE there) and
// source_old/commands/OtosCommands.cpp's own handleOP made explicitly
// ("This is the only OTOS command that does NOT access hardware").
//
// OL/OA maintain their own read-modify-write shadow (OtosCommandState::
// configShadow), mirroring config_commands.cpp's ConfigCommandState shadow
// pattern: Hal::Odometer::configure() is a write-only message-plane call
// (like Hal::Motor::configure()) with no matching getter, so the CURRENT
// value for an OL/OA *read*, or for the field an OL/OA *set* did NOT touch,
// comes from this shadow -- never a read-back through the device.
// ---------------------------------------------------------------------------

#if ROBOT_DEV_BUILD

#include <vector>

#include "command_types.h"
#include "messages/odometer.h"
#include "subsystems/hardware.h"

// OtosCommandState -- see this file's header comment. hardware must be set
// before otosCommands()'s handlers are called -- mirrors poseCommands()'s/
// configCommands()'s own contract.
struct OtosCommandState {
  Subsystems::Hardware* hardware = nullptr;

  // OL/OA's own config-plane shadow, seeded to the proto zero-default
  // (linear_scalar = angular_scalar = 0.0f). No boot-config generator feeds
  // this (no robot-JSON/sim_prefs field exists for it this sprint --
  // Decision 5's "store-and-echo, no physical effect" already made the
  // actual boot value inconsequential).
  msg::OdometerConfig configShadow;
};

// Returns the OTOS command table (OI, OZ, OR, OP, OV, OL, OA), bound to the
// given shared state.
std::vector<CommandDescriptor> otosCommands(OtosCommandState& state);

#endif  // ROBOT_DEV_BUILD
