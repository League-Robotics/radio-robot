#pragma once

// ---------------------------------------------------------------------------
// telemetry_commands.h -- the STREAM/SNAP command family (082-004, rewritten
// pointerless 087-006; frame-assembly moved out to Telemetry::tick() by
// 087-008; loop-owned periodic emission restored by 096-002 after sprint
// 093's loop rewrite deleted it -- see tickTelemetry() below): periodic TLM
// emission plus a synchronous one-shot snapshot, both built on
// telemetry/tlm_frame.h's Telemetry::tick() (bb -> TlmFrameInput) and
// Telemetry::buildTlmFrame() (TlmFrameInput -> wire line). This file is
// the IMPURE glue: it resolves the reply channel, calls Telemetry::tick()
// to sample Rt::Blackboard's committed state cells, advances the shared
// seq= counter, formats via buildTlmFrame(), and replies -- never a
// Hardware/Drivetrain/PoseEstimator/Planner pointer (SUC-006).
//
//   STREAM <ms>   -- sets the periodic-emission period, clamped to a 20ms
//                    floor (STREAM 10 -> OK stream period=20). STREAM 0
//                    disables periodic emission. Binds the periodic-emission
//                    reply channel to whichever channel issued this STREAM
//                    command (docs/protocol-v2.md §8's channel-binding rule)
//                    -- bb.telemetryChannel, read from
//                    Rt::CommandRouter::currentChannel() (see command_router.h).
//   SNAP          -- one TLM line synchronously, replied on the SAME
//                    channel/replyFn the SNAP command itself arrived on
//                    (NOT necessarily the STREAM-bound channel -- only the
//                    seq= counter is shared between the two verbs, per the
//                    ticket's acceptance criteria).
//
// Deliberately minimal this sprint (Decision 5): no `STREAM fields=<csv>`
// subscription, no D10 idle-rate refinement, no channel-rebinding nuance
// beyond "the channel that most recently issued STREAM is the bound
// recipient." These are named, explicit deferrals -- do not reintroduce
// without a fresh, acceptance-bar-driven reason.
//
// Field sourcing (Decision 7) moved to Telemetry::tick() by 087-008 --
// source/telemetry/tlm_frame.h documents the full per-field rule table
// (enc=/vel=/pose=/encpose=/otos=/twist=/mode=) at that function's own doc
// comment. This file no longer contains that logic at all.
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
// (architecture-update.md): this does NOT reproduce handleStream()'s old
// "immediate first frame concatenated into the SAME reply as the ACK"
// micro-optimization below -- the first periodic frame now arrives one pass
// later, via the normal !telemetryHasLastEmit trigger, uniformly for every
// channel. This is a deliberate, documented behavior refinement, not a
// regression.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "runtime/command_router.h"


// telemetryEmit -- shared emission path: calls Telemetry::tick(now, bb) to
// assemble a TlmFrameInput (the actual field-sourcing logic -- see
// tlm_frame.h), advances the shared bb.telemetrySeq counter (Telemetry::
// tick() itself only READS it), formats via Telemetry::buildTlmFrame(), and
// calls replyFn(line, replyCtx). Used by BOTH the loop's periodic-emission
// step (passing the channel resolved from bb.telemetryChannel) and SNAP's
// handler (passing its own dispatch replyFn/replyCtx). A null replyFn is a
// silent no-op -- bb.telemetrySeq is NOT advanced in that case, since no
// frame was actually emitted.
void telemetryEmit(Rt::Blackboard& bb, uint32_t now, ReplyFn replyFn, void* replyCtx);

// tickTelemetry -- the loop-owned periodic-emission step (096-002): checks
// bb.telemetryPeriod > 0 and elapsed time (bb.telemetryLastEmitMs/
// bb.telemetryHasLastEmit, the SAME fields handleStream() already
// maintains for its own immediate-first-frame emission); if a frame is due,
// resolves bb.telemetryChannel to a live ReplyFn/void* pair via
// router.replySink() (command_router.h) and emits one frame. bb.telemetryBinary
// (blackboard.h) is the branch point (096-003): true selects the binary
// path (telemetryEmitBinary()/Telemetry::buildTelemetryMessage(), .cpp-local),
// false (the default) selects the pre-existing text path
// (telemetryEmit()/Telemetry::buildTlmFrame()). Nothing sets
// bb.telemetryBinary true until ticket 005 (the binary `stream` arm), so
// this ticket's own observable behavior is still unconditionally text. A
// no-op when no frame is due (bb.telemetryPeriod == 0, or not enough time
// has elapsed since the last emission).
void tickTelemetry(Rt::Blackboard& bb, Rt::CommandRouter& router, uint32_t now);

// Returns the STREAM/SNAP command table, bound to `router`.
std::vector<CommandDescriptor> telemetryCommands(Rt::CommandRouter& router);

