#pragma once

// ---------------------------------------------------------------------------
// config_commands.h -- the top-level SET/GET command family (084-006,
// rewritten pointerless 087-006): production protocol-v2 config-plane verbs
// (docs/protocol-v2.md §7).
//
// SET's validate-then-ERR still happens SYNCHRONOUSLY in the handler
// (architecture-update-r1.md Decision 3): it reads the CURRENT published
// config cells (bb.drivetrainConfig/bb.motorConfig[]/bb.plannerConfig/
// bb.streamWatchdogWindow -- replacing the pre-087 drivetrainShadow/
// motorShadow[]/plannerShadow read-modify-write shadows entirely, since
// these ARE now the single published source of truth, per the
// architecture's "Replaces every shadow" design), folds+validates a
// CANDIDATE copy, and replies ERR immediately with NOTHING posted on
// failure. Only the accepted path posts: one Rt::ConfigDelta per touched
// target (kDrivetrain/kMotor-left/kMotor-right/kPlanner) to bb.configIn --
// the Configurator (ticket 005) folds+applies each -- plus, for the one key
// that is NOT one of the Configurator's four targets (sTimeout -- the
// loop-owned StreamingDriveWatchdog's window, motion_commands.h), a post to
// bb.streamWatchdogWindowIn.
//
// Approved key table (architecture-update.md (084) Decision 2 -- unchanged
// by this rewrite):
//   tw                                         -> DrivetrainConfig.trackwidth
//   ml / mr                                    -> bound-pair motors' MotorConfig.travel_calib
//   pid.kp / pid.ki / pid.kff / pid.iMax / pid.kaw
//                                               -> both bound motors' MotorConfig.vel_gains
//   rotSlip                                    -> DrivetrainConfig.rotational_slip
//   ekfQxy / ekfQtheta / ekfROtosXy / ekfROtosTheta
//                                               -> DrivetrainConfig's matching ekf_* fields
//   minSpeed                                   -> PlannerConfig.min_speed
//   sTimeout                                   -> the loop-owned StreamingDriveWatchdog window
//
// `ml`/`mr`/`pid.*` always resolve against the CURRENTLY bound pair, read
// via bb.drivetrainConfig.left_port/right_port at SET/GET-time -- never a
// hardcoded port, never a Drivetrain* -- so a `DEV DT PORTS` rebind
// immediately changes which motor's config `ml`/`mr`/`pid.*` read and write
// next (once the Configurator has applied the rebind's own ConfigDelta).
//
// SET is atomic all-or-nothing (docs/protocol-v2.md §7): every key=value
// pair in one SET line is parsed and validated against a CANDIDATE copy of
// every published cell it touches; if any key is unknown, non-numeric, or
// fails an invariant, NO ConfigDelta is posted and nothing is applied. Only
// on full success are the touched targets' deltas posted.
//
// This file depends on commands/motion_commands.h for exactly one type,
// StreamingDriveWatchdog -- a small, stateless-behavior value class
// (feed/check/setWindow/window) -- unaffected by this rewrite; this file
// never reads/writes any OTHER motion_commands.h symbol.
// ---------------------------------------------------------------------------

#if ROBOT_DEV_BUILD

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "commands/motion_commands.h"  // StreamingDriveWatchdog -- see file header
#include "runtime/command_router.h"

// Returns the SET/GET command table, bound to `router`.
std::vector<CommandDescriptor> configCommands(Rt::CommandRouter& router);

#endif  // ROBOT_DEV_BUILD
