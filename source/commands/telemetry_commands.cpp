// telemetry_commands.cpp -- STREAM/SNAP command handlers + telemetryEmit().
// See telemetry_commands.h for the full vocabulary, the field-sourcing
// rules (Decision 7), and the ROBOT_DEV_BUILD gating rationale.
#include "commands/telemetry_commands.h"


#include "commands/command_processor.h"
#include "messages/wire.h"
#include "messages/wire_runtime.h"
#include "telemetry/tlm_frame.h"
#include "types/clock.h"

namespace {

Rt::Blackboard& bb(void* handlerCtx) { return static_cast<Rt::CommandRouter*>(handlerCtx)->blackboard(); }

// kArmoredBufSize (096-003) -- "*B" (2) + base64(kReplyEnvelopeMaxEncodedSize)
// + NUL, rounded up with headroom; the SAME sizing argument and 256-byte
// budget commands/binary_channel.cpp's own kArmoredBufSize uses (matches
// Subsystems::CommunicatorToCommandProcessorCommand::line's 256-byte
// transport budget, wire_command.h).
constexpr size_t kArmoredBufSize = 256;

// telemetryEmitBinary -- the binary-plane sibling of telemetryEmit() below
// (096-003): the SAME tick()-then-advance-seq flow, but formats via
// Telemetry::buildTelemetryMessage() into a msg::ReplyEnvelope{tlm}, then
// encode+armor+send exactly like commands/binary_channel.cpp's own
// sendReply() (msg::wire::encode -> "*B" + WireRuntime::base64Encode).
// corr_id = 0 -- this is unsolicited PUSH telemetry, not a reply to any
// particular CommandEnvelope (envelope.proto's own forward-looking doc
// comment; matches ticket 005's own acceptance criterion for the binary
// `stream` arm this function's caller is gated on). Only ever called from
// tickTelemetry() below -- SNAP (handleSnap()) always uses the text
// telemetryEmit(), regardless of bb.telemetryBinary (this file's own
// header note: only bb.telemetrySeq is shared between STREAM and SNAP).
void telemetryEmitBinary(Rt::Blackboard& b, uint32_t now, ReplyFn replyFn, void* replyCtx) {
  if (replyFn == nullptr) return;

  Telemetry::TlmFrameInput in = Telemetry::tick(now, b);
  b.telemetrySeq++;   // shared with the text path -- see telemetryEmit()'s own comment

  msg::ReplyEnvelope reply;
  reply.corr_id = 0;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  Telemetry::buildTelemetryMessage(reply.body.tlm, in);

  uint8_t rawBuf[msg::wire::kReplyEnvelopeMaxEncodedSize];
  const uint16_t n = msg::wire::encode(reply, rawBuf, static_cast<uint16_t>(sizeof(rawBuf)));
  if (n == 0) {
    // Unreachable in practice -- rawBuf is sized from the SAME generated
    // kReplyEnvelopeMaxEncodedSize constant encode() itself is budgeted
    // against (wire.h's own static_assert), so every ReplyEnvelope this
    // function builds fits. No frame is sent rather than a malformed one.
    return;
  }

  char armored[kArmoredBufSize];
  armored[0] = '*';
  armored[1] = 'B';
  size_t b64Len = 0;
  if (!WireRuntime::base64Encode(rawBuf, n, armored + 2, sizeof(armored) - 3, &b64Len)) {
    return;   // same unreachable-in-practice sizing argument as above
  }
  armored[2 + b64Len] = '\0';
  replyFn(armored, replyCtx);
}

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

  // bb.telemetryBinary (096-002's branch point, wired by 096-003): a binary
  // ReplyEnvelope{tlm} frame via telemetryEmitBinary() when a binary stream
  // client asked for it (ticket 005's `stream` arm sets this true), the
  // pre-existing text TLM line via telemetryEmit() otherwise. Both share
  // the SAME Telemetry::tick()-then-advance-bb.telemetrySeq flow -- only
  // the wire framing differs. Nothing sets bb.telemetryBinary true until
  // ticket 005 lands, so this ticket's own observable behavior is still
  // unconditionally text.
  if (bb.telemetryBinary) {
    telemetryEmitBinary(bb, now, replyFn, replyCtx);
  } else {
    telemetryEmit(bb, now, replyFn, replyCtx);
  }

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

