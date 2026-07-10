// telemetry_commands.cpp -- STREAM/SNAP command handlers + telemetryEmit().
// See telemetry_commands.h for the full vocabulary, the field-sourcing
// rules (Decision 7), and the ROBOT_DEV_BUILD gating rationale.
#include "commands/telemetry_commands.h"


#include "commands/command_processor.h"
#include "telemetry/tlm_frame.h"
#include "types/clock.h"

namespace {

Rt::Blackboard& bb(void* handlerCtx) { return static_cast<Rt::CommandRouter*>(handlerCtx)->blackboard(); }

// ---------------------------------------------------------------------------
// STREAM <ms> -- pure fixed-shape `<verb> <int>` command, so it uses
// ArgSchema (same mixed hand-rolled/schema approach dev_commands.h's Open
// Question 3 documents for DEV WD <window>). The 20ms floor is enforced by
// handleStream() below, not the schema -- STREAM 10 must be ACCEPTED and
// clamped to 20, not rejected as out-of-range.
// ---------------------------------------------------------------------------
const ArgDef kStreamArgs[] = {
    { "period", ArgKind::INT, true, 0, 60000 },
};
const ArgSchema kStreamSchema = { kStreamArgs, 1, 1, false, nullptr };

constexpr uint32_t kStreamFloorMs = 20;   // [ms] docs/protocol-v2.md §8's documented minimum

void handleStream(const ArgList& args, const char* corrId,
                   ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
  Rt::CommandRouter* router = static_cast<Rt::CommandRouter*>(handlerCtx);
  Rt::Blackboard& b = router->blackboard();
  uint32_t requested = static_cast<uint32_t>(args.args[0].ival);
  uint32_t period = (requested == 0) ? 0
                    : (requested < kStreamFloorMs ? kStreamFloorMs : requested);

  b.telemetryPeriod = period;
  // Channel binding (docs/protocol-v2.md §8): the periodic-emission reply
  // channel is whichever channel issued the most recently accepted STREAM
  // command -- rebound unconditionally, even for STREAM 0 (disabling still
  // records "this channel asked last"). Resolved via CommandRouter's
  // currentChannel() (the command CommandRouter::route() is currently
  // dispatching), never a captured ReplyFn/void* pair -- see
  // telemetry_commands.h's file header.
  b.telemetryChannel = router->currentChannel();

  char rbuf[48];
  CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "stream", corrId, replyFn, replyCtx,
                             "period=%u", static_cast<unsigned>(period));

  // 096-002 (architecture-update.md Open Question 5): pre-087 (and every
  // version through the 093 loop rewrite) this handler ALSO emitted an
  // immediate first frame here, concatenated into THIS SAME dispatch's own
  // reply -- a same-pass, same-reply optimization made possible only
  // because a captured ReplyFn/void* pair was still available at this call
  // site. That optimization is DELIBERATELY NOT reproduced now that a real
  // loop-owned periodic tick exists again (tickTelemetry(), this file):
  // emission -- first frame AND every later one -- is entirely
  // tickTelemetry()'s job, firing one pass after this handler sets
  // bb.telemetryPeriod/bb.telemetryChannel, via its own normal
  // !bb.telemetryHasLastEmit trigger. This handler no longer touches
  // bb.telemetryLastEmitMs/bb.telemetryHasLastEmit at all -- see
  // telemetry_commands.h's file header for the full rationale.
}

// ---------------------------------------------------------------------------
// SNAP -- no arguments, no schema (parseFn = nullptr), mirroring how
// PING/VER register (system_commands.cpp). Replies on its OWN dispatch
// replyFn/replyCtx (the channel SNAP itself arrived on) -- NOT
// bb.telemetryChannel (the STREAM-bound channel); only bb.telemetrySeq is
// shared between the two verbs -- see telemetry_commands.h's header comment.
// ---------------------------------------------------------------------------
void handleSnap(const ArgList& /*args*/, const char* /*corrId*/,
                ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  uint32_t now = Types::systemClockNow();
  telemetryEmit(b, now, replyFn, replyCtx);
}

}  // namespace

void telemetryEmit(Rt::Blackboard& b, uint32_t now, ReplyFn replyFn, void* replyCtx) {
  // A channel that never bound a replyFn must not dereference a null
  // function pointer. bb.telemetrySeq is left untouched -- no frame was
  // emitted.
  if (replyFn == nullptr) return;

  // Field sourcing (Decision 7) is entirely Telemetry::tick()'s own
  // internals now (source/telemetry/tlm_frame.{h,cpp}, 087-008) -- this
  // function's remaining job is just the shared seq= bookkeeping and the
  // actual wire emission.
  Telemetry::TlmFrameInput in = Telemetry::tick(now, b);
  b.telemetrySeq++;   // shared by every STREAM-driven frame AND SNAP --
                       // advances AFTER this frame captured the
                       // PRE-increment value via Telemetry::tick().

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);
  replyFn(buf, replyCtx);
}

void tickTelemetry(Rt::Blackboard& bb, Rt::CommandRouter& router, uint32_t now) {
  if (bb.telemetryPeriod == 0) return;
  if (bb.telemetryHasLastEmit && (now - bb.telemetryLastEmitMs) < bb.telemetryPeriod) return;

  ReplyFn replyFn = nullptr;
  void* replyCtx = nullptr;
  router.replySink(bb.telemetryChannel, replyFn, replyCtx);

  // bb.telemetryBinary is the branch point for the binary formatter a later
  // ticket wires in (see this function's own doc comment in
  // telemetry_commands.h); unconditionally text this ticket, since nothing
  // sets it true yet.
  telemetryEmit(bb, now, replyFn, replyCtx);

  // Mirrors handleStream()'s own immediate-first-frame bookkeeping: update
  // unconditionally (even if telemetryEmit() was a silent no-op because
  // replyFn resolved null) so a channel with no wired reply sink does not
  // retry every single pass.
  bb.telemetryLastEmitMs = now;
  bb.telemetryHasLastEmit = true;
}

std::vector<CommandDescriptor> telemetryCommands(Rt::CommandRouter& router) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeSchemaCmd("STREAM", &kStreamSchema, handleStream, &router,
                               "badarg", ForceReply::NONE, CMD_NONE));
  cmds.push_back(makeCmd("SNAP", nullptr, handleSnap, &router,
                         "badarg", ForceReply::NONE, CMD_NONE));
  return cmds;
}

