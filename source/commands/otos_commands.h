#pragma once

// ---------------------------------------------------------------------------
// otos_commands.h -- the seven-verb OTOS command family (084-008/SUC-007,
// rewritten pointerless 087-006): OI/OZ/OR/OP/OV/OL/OA, fully specified in
// docs/protocol-v2.md §11 (grammar, reply shapes, ERR nodev behavior).
//
// Never holds or dereferences a Subsystems::Hardware*/Hal::Odometer*
// (SUC-006). Device presence is read from bb.otosPresent -- a boot-time
// snapshot of `hardware.odometer() != nullptr` (see blackboard.h's file
// header): every current concrete Hardware leaf's odometer() either always
// returns the same non-null leaf or always returns nullptr for its whole
// lifetime, so a one-time boot snapshot is equivalent to the pre-087 "live
// resolution on every dispatch" for every build this tree actually produces
// (NezhaHardware always non-null since 086-006; SimHardware always
// non-null) -- flagged explicitly since the pre-087 file header's own
// rationale for LIVE resolution ("a future odometer swap... must not
// require touching this file") assumed a Hardware reference this rewrite
// can no longer hold; a genuine hot-swap capability, if ever added, would
// need bb.otosPresent refreshed by whatever performs the swap (the loop),
// not by this file.
//
// OI/OZ/OR/OV post one msg::OdometerCommand to bb.otosCommandIn (a
// Mailbox<msg::OdometerCommand>, drained by the loop directly against
// hardware.odometer() -- mirrors bb.otosSetPoseIn's own "the loop drains
// this against the odometer directly" shape, since Hal::Odometer has no
// tick()-driven queue parameter of its own). OP reads bb.otos/bb.otosPresent
// directly (a state-cell read, matching its pre-087 "reads Hal::Odometer::
// pose() directly... not tick()" CMD_NONE shape). OL/OA read/write
// bb.odometerConfig (the Configurator's own published config cell,
// replacing the pre-087 OtosCommandState::configShadow) and post a
// field-masked Rt::ConfigDelta (kOdometer) to bb.configIn on a set --
// mirrors config_commands.h's SET/DEV *CFG pattern exactly, since OL/OA are
// genuinely config-plane (read-modify-write persistent register), unlike
// the other five one-shot verbs.
// ---------------------------------------------------------------------------

#if ROBOT_DEV_BUILD

#include <vector>

#include "command_types.h"
#include "runtime/command_router.h"

// Returns the OTOS command table (OI, OZ, OR, OP, OV, OL, OA), bound to
// `router`.
std::vector<CommandDescriptor> otosCommands(Rt::CommandRouter& router);

#endif  // ROBOT_DEV_BUILD
