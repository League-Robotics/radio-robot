#pragma once

// ---------------------------------------------------------------------------
// telemetry_commands.h -- 097-008 (architecture-update-r2.md Decision 9,
// pure-binary firmware): the text STREAM/SNAP command family (082-004,
// rewritten pointerless 087-006, frame-assembly moved out to
// Telemetry::tick() by 087-008) is DELETED outright -- handleStream()/
// handleSnap(), kStreamSchema, and telemetryEmit() (the text-emission
// helper they shared) no longer exist anywhere in this file; see git
// history for that prior code. Their binary-plane parity
// (`stream`/StreamControl, source/commands/binary_channel.cpp, 096-005) is
// unaffected -- it already fully replaced them (sim-exhaustive, 096).
//
// What remains: tickTelemetry() (096-002/096-003, below), the loop-owned
// periodic-emission mechanism, now driving ONLY the binary path
// (telemetryEmitBinary(), .cpp-local, built on telemetry/tlm_frame.h's
// Telemetry::tick()/Telemetry::buildTelemetryMessage()) -- there is no text
// path left to branch against. telemetryCommands() (below) still exists,
// still called from Rt::CommandRouter::buildTable() (command_router.cpp,
// out of this ticket's file scope), but now registers ZERO commands -- kept
// as a stable no-op entry point rather than also touching command_router.cpp.
//
// tickTelemetry() (096-002, architecture-update.md M2) -- the loop-owned
// periodic-emission mechanism STREAM's own design has always assumed but
// which sprint 093's loop rewrite deleted along with the rest of the old
// per-pass loop step (dev_loop.cpp, since removed; see
// source/runtime/main_loop.h's own file header). Called once per pass by
// BOTH source/main.cpp and tests/_infra/sim/sim_api.cpp, as a peer of their
// existing router.route(...)/loop.tick(...) calls -- the same
// "hardware and sim call the identical function" invariant Rt::MainLoop::
// tick() already establishes for motion. Per Open Question 5
// (architecture-update.md): this does NOT reproduce the deleted text
// handleStream()'s old "immediate first frame concatenated into the SAME
// reply as the ACK" micro-optimization -- the first periodic frame arrives
// one pass later, via the normal !telemetryHasLastEmit trigger, uniformly
// for every channel. This is a deliberate, documented behavior refinement,
// not a regression.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "runtime/command_router.h"


// tickTelemetry -- the loop-owned periodic-emission step (096-002): checks
// bb.telemetryPeriod > 0 and elapsed time (bb.telemetryLastEmitMs/
// bb.telemetryHasLastEmit, the SAME fields the deleted text handleStream()
// used to maintain for its own immediate-first-frame emission); if a frame
// is due, resolves bb.telemetryChannel to a live ReplyFn/void* pair via
// router.replySink() (command_router.h) and emits one binary frame via
// telemetryEmitBinary() (.cpp-local). Unconditional since 097-008 -- there
// is no more text sibling to branch against, so bb.telemetryBinary
// (blackboard.h, still written by binary_channel.cpp's `stream` arm) is no
// longer read here; see this file's own header comment. A no-op when no
// frame is due (bb.telemetryPeriod == 0, or not enough time has elapsed
// since the last emission).
void tickTelemetry(Rt::Blackboard& bb, Rt::CommandRouter& router, uint32_t now);

// Returns the telemetry command table, bound to `router`. Empty since
// 097-008 -- STREAM/SNAP's text registrations are gone; see this file's own
// header comment for why the function itself is kept.
std::vector<CommandDescriptor> telemetryCommands(Rt::CommandRouter& router);

