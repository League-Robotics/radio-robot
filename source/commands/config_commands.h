#pragma once

// ---------------------------------------------------------------------------
// config_commands.h -- the top-level SET/GET command family (084-006):
// production protocol-v2 config-plane verbs (docs/protocol-v2.md §7),
// registered alongside DEV/telemetry/motion in main.cpp's (and
// sim_api.cpp's) command table.
//
// Config-plane, not command-plane (architecture-update.md (084) "Config-
// plane vs. command-plane" precedent, sprint 079): SET/GET handlers call
// Subsystems::Drivetrain::configure()/Subsystems::PoseEstimator::configure()/
// Subsystems::Planner::configure()/the bound pair's Hal::Motor::configure()
// directly and synchronously -- no outbox, no staging, nothing for
// dev_loop.cpp to drain. This is why ConfigCommandState is NOT added to
// DevLoop (source/dev_loop.h) the way MotionLoopState/TelemetryState are --
// see architecture-update.md (084) Decision 7's Consequences.
//
// ConfigCommandState is NOT DevLoopState's motorConfigShadow[]/
// drivetrainConfigShadow (Decision 7): a deliberately independent
// read-modify-write shadow, seeded from the SAME boot configs main.cpp/
// sim_api.cpp pass to NezhaHardware/SimHardware's constructor and to
// Drivetrain::configure()/Planner::configure() at boot, but never read from
// or written by the DEV family. The two shadows CAN diverge if a `DEV M`/
// `DEV DT CFG` write and a `SET` write touch the same field independently --
// an accepted consequence of two independent command families each owning
// their own config-plane shadow (Decision 7).
//
// Approved key table (architecture-update.md (084) Decision 2 -- the
// complete, closed set this file registers; every other §7 key, including
// every explicitly-dropped one, is simply not registered here and so
// correctly surfaces as `ERR badkey`, identical wire behavior to any
// never-existed key):
//   tw                                         -> DrivetrainConfig.trackwidth
//   ml / mr                                    -> bound-pair motors' MotorConfig.travel_calib
//   pid.kp / pid.ki / pid.kff / pid.iMax / pid.kaw
//                                               -> both bound motors' MotorConfig.vel_gains
//   rotSlip                                    -> DrivetrainConfig.rotational_slip
//   ekfQxy / ekfQtheta / ekfROtosXy / ekfROtosTheta
//                                               -> DrivetrainConfig's matching ekf_* fields
//   minSpeed                                   -> PlannerConfig.min_speed
//   sTimeout                                   -> ticket 002's StreamingDriveWatchdog
//                                                  window (MotionLoopState::sTimeout --
//                                                  a plain field, no message, mirroring
//                                                  `DEV WD` reading SerialSilenceWatchdog
//                                                  directly)
//
// `ml`/`mr`/`pid.*` always resolve against the CURRENTLY bound pair, read
// via Subsystems::Drivetrain::ports() at SET/GET-time -- never a hardcoded
// port -- so a `DEV DT PORTS` rebind immediately changes which motor's
// shadow `ml`/`mr`/`pid.*` read and write next.
//
// SET is atomic all-or-nothing (docs/protocol-v2.md §7): every key=value
// pair in one SET line is parsed and validated against a CANDIDATE copy of
// every shadow it touches; if any key is unknown, non-numeric, or fails an
// invariant, NO candidate is committed and the live config (shadows AND the
// real subsystems/motors) is unchanged. Only on full success are the
// touched shadows copied back and configure() called on each touched
// subsystem/motor.
//
// Known consequence, not a bug: any drivetrain-scoped key (`tw`/`rotSlip`/
// `ekf*`) re-propagates the FULL candidate msg::DrivetrainConfig to BOTH
// Drivetrain::configure() and PoseEstimator::configure() (mirroring
// main.cpp's/sim_api.cpp's own boot wiring, which configures both from one
// shared source) -- and PoseEstimator::configure() calls EkfTiny::init()
// unconditionally (that method's own documented boot-only reset), so any
// one of these keys also re-zeroes the fused pose/covariance. This is the
// architecture's own explicit design (084 Grounding fact 3 / Decision 2),
// not something this file works around.
//
// This file depends on commands/motion_commands.h for exactly one type,
// StreamingDriveWatchdog -- a small, stateless-behavior value class (feed/
// check/setWindow/window) ticket 002 already defined. This is a deliberate,
// narrow, data-only type dependency (like hal/capability/hal_command.h's
// shared edge types), not a functional coupling on the rest of
// MotionLoopState: config_commands.cpp never reads/writes any OTHER
// MotionLoopState field, and never includes dev_commands.h/
// telemetry_commands.h -- this file's sibling command families never
// include each other either.
// ---------------------------------------------------------------------------

#if ROBOT_DEV_BUILD

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "commands/motion_commands.h"  // StreamingDriveWatchdog -- see file header
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/planner.h"
#include "subsystems/pose_estimator.h"

// ---------------------------------------------------------------------------
// ConfigCommandState -- see this file's header comment.
// ---------------------------------------------------------------------------
struct ConfigCommandState {
  Subsystems::Hardware* hardware = nullptr;
  Subsystems::Drivetrain* drivetrain = nullptr;
  Subsystems::PoseEstimator* poseEstimator = nullptr;
  Subsystems::Planner* planner = nullptr;
  // Ticket 002's production streaming-drive watchdog window -- see file
  // header. Points at the SAME instance dev_loop.cpp checks every pass
  // (MotionLoopState::sTimeout); main.cpp/sim_api.cpp wire this to
  // &motionState.sTimeout.
  StreamingDriveWatchdog* sTimeoutWatchdog = nullptr;

  // Own config-plane shadow (Decision 7) -- seeded by main.cpp/sim_api.cpp
  // from the SAME boot configs passed to NezhaHardware/SimHardware's
  // constructor and to Drivetrain::configure()/Planner::configure() at
  // boot, mirroring DevLoopState's own seeding contract (dev_commands.h).
  msg::DrivetrainConfig drivetrainShadow = {};
  msg::MotorConfig motorShadow[Subsystems::Hardware::kPortCount] = {};
  msg::PlannerConfig plannerShadow = {};
};

// Returns the SET/GET command table, bound to the given shared state (every
// pointer field must be set, and the three shadow fields seeded, before any
// call this table's handlers make -- mirrors devCommands()'s/
// telemetryCommands()'s/motionCommands()'s own contract).
std::vector<CommandDescriptor> configCommands(ConfigCommandState& state);

#endif  // ROBOT_DEV_BUILD
