#pragma once

// dev_loop.h -- LoopContext / runLoopPass(): sprint 087 ticket 006's
// TRANSITIONAL cyclic-executive-predecessor loop body. This is main.cpp's/
// tests/_infra/sim/sim_api.cpp's SHARED per-pass function (mirrors this
// file's own pre-087 role, ticket 081-002 -- "a future simulated caller can
// run the IDENTICAL body instead of hand-mirroring it") -- rewired so
// statement dispatch goes through Rt::CommandRouter and every command
// family's outbox is now a Rt::Blackboard queue, replacing the six
// (087-006-deleted) *State structs' direct outbox fields.
//
// NOT ticket 007's real cyclic executive: no double-buffer commit, no
// mandatory/slack split, no uBit.sleep(1) yield. Same-pass, sequential
// feed-forward, structurally IDENTICAL to this file's pre-087 body (see
// dev_loop.cpp for the line-by-line correspondence) -- a deliberate,
// ticket-006-sanctioned deviation from architecture-update-r1.md's Reference
// code loop shape, explicitly permitted so the build stays green and every
// existing test's wire-visible behavior (reply text/timing) is unchanged.
// Ticket 007 replaces this file's CONTENTS (and likely its exported names)
// with the real cyclic executive -- see this ticket's Implementation Notes
// for what to adjust.
//
// Host-clean by construction: no #include "MicroBit.h", no
// Subsystems::Communicator dependency -- "how did a statement get fed in"
// is the caller's problem (unchanged from before this ticket).
#include <stdint.h>

#include "commands/dev_commands.h"       // SerialSilenceWatchdog, buildBroadcastNeutral/buildDrivetrainStop
#include "commands/motion_commands.h"    // StreamingDriveWatchdog
#include "runtime/command_router.h"
#include "runtime/configurator.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/planner.h"
#include "subsystems/pose_estimator.h"
#include "subsystems/statement.h"

#if ROBOT_DEV_BUILD

// LoopContext -- the loop-owned instances that sit OUTSIDE Rt::Blackboard/
// Rt::Configurator/Rt::CommandRouter: the four subsystem references (the
// loop is the composition root -- same legitimate-pointer-holding status as
// Rt::Configurator, Decision 4), the two watchdogs (loop-owned, not one of
// the Configurator's four targets -- blackboard.h's file header), the
// persistent "which verb staged the active goal" bookkeeping (087-006:
// replaces the deleted MotionLoopState::activeVelocityVerb field -- see
// runtime/commands.h's Rt::MotionCommand doc comment), and the reply sinks.
//
// serialReply/serialCtx doubles as the loop-originated "default" reply sink
// (watchdog-fire EVT, motion-done EVT, safety_stop EVT) AND the periodic-
// telemetry-emission channel when bb.telemetryChannel is SERIAL/NONE --
// byte-identical to the pre-087 DevLoop::defaultReply/defaultReplyCtx, which
// was always bound to serial in both main.cpp and sim_api.cpp. radioReply/
// radioCtx is used only for periodic-telemetry emission bound to RADIO.
// These are intentionally NOT the same ReplyFn/void* pair
// Rt::CommandRouter::setReplyChannels() was given -- sim_api.cpp's own
// Decision 3 (two independent reply stores: syncStore for a dispatched
// statement's own reply, asyncStore for anything the loop originates
// itself) needs them to be independently bindable; main.cpp binds both to
// the same physical sink since it has no such split to preserve.
struct LoopContext {
  Subsystems::Hardware* hardware = nullptr;
  Subsystems::Drivetrain* drivetrain = nullptr;
  Subsystems::PoseEstimator* poseEstimator = nullptr;
  Subsystems::Planner* planner = nullptr;
  Rt::CommandRouter* router = nullptr;
  Rt::Configurator* configurator = nullptr;

  SerialSilenceWatchdog watchdog;
  StreamingDriveWatchdog streamWatchdog;
  char activeVelocityVerb[8] = "";

  ReplyFn serialReply = nullptr;
  void* serialCtx = nullptr;
  ReplyFn radioReply = nullptr;
  void* radioCtx = nullptr;
};

// runLoopPass -- runs exactly one pass of the shared, transitional loop
// body: the two-slice hardware tick, statement-triggered route() (only when
// statement != nullptr), the broadcast-neutral/watchdog-window/odometer
// one-shot queue drains, the config-plane drain (ALL pending deltas, not
// rationed to one-per-pass -- see this ticket's Implementation Notes),
// Drivetrain governance (gated on active() OR a fresh bb.driveIn post, so a
// reactivation request is never stranded), pose estimation, the motion
// executor (drains bb.motionIn into Planner::apply(), the sTimeout check,
// Planner::tick()), the bb state-cell commit, periodic TLM emission, and the
// serial-silence watchdog check -- reproducing this file's pre-087 body
// (dev_loop.cpp, then named devLoopTick()) line-for-line in STRUCTURE, with
// every direct subsystem-outbox touch replaced by a Rt::Blackboard queue
// drain. now: [ms]. statement: nullptr when no statement is being fed this
// pass.
void runLoopPass(LoopContext& loop, Rt::Blackboard& bb, uint32_t now,
                  const Subsystems::CommunicatorToCommandProcessorStatement* statement);

#endif  // ROBOT_DEV_BUILD
