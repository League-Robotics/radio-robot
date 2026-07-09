#pragma once

// ---------------------------------------------------------------------------
// telemetry_commands.h -- the STREAM/SNAP command family (082-004, rewritten
// pointerless 087-006; frame-assembly moved out to Telemetry::tick() by
// 087-008): periodic TLM emission plus a synchronous one-shot snapshot, both
// built on telemetry/tlm_frame.h's Telemetry::tick() (bb -> TlmFrameInput)
// and Telemetry::buildTlmFrame() (TlmFrameInput -> wire line). This file is
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

// Returns the STREAM/SNAP command table, bound to `router`.
std::vector<CommandDescriptor> telemetryCommands(Rt::CommandRouter& router);

