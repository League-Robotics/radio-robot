// telemetry_commands.cpp -- 097-008: the text STREAM/SNAP command handlers
// and telemetryEmit() (their shared text-emission helper) are DELETED; see
// telemetry_commands.h's own file header for the full rationale and git
// history for the prior code. What remains is the binary-plane machinery
// (telemetryEmitBinary(), tickTelemetry()) plus the now-empty
// telemetryCommands() registrar.
#include "commands/telemetry_commands.h"


#include "messages/wire.h"
#include "messages/wire_runtime.h"
#include "telemetry/tlm_frame.h"

namespace {

// kArmoredBufSize (096-003) -- "*B" (2) + base64(kReplyEnvelopeMaxEncodedSize)
// + NUL, rounded up with headroom; the SAME sizing argument and 256-byte
// budget commands/binary_channel.cpp's own kArmoredBufSize uses (matches
// Subsystems::CommunicatorToCommandProcessorCommand::line's 256-byte
// transport budget, wire_command.h).
constexpr size_t kArmoredBufSize = 256;

// telemetryEmitBinary -- the binary-plane emission path (096-003): the
// tick()-then-advance-seq flow, formats via Telemetry::buildTelemetryMessage()
// into a msg::ReplyEnvelope{tlm}, then encode+armor+send exactly like
// commands/binary_channel.cpp's own sendReply() (msg::wire::encode -> "*B"
// + WireRuntime::base64Encode). corr_id = 0 -- this is unsolicited PUSH
// telemetry, not a reply to any particular CommandEnvelope (envelope.proto's
// own forward-looking doc comment). Only ever called from tickTelemetry()
// below -- 097-008 deleted this function's text-emission sibling
// (telemetryEmit()) and its only other caller (SNAP's handler), so this is
// now the sole emission path, unconditional (see tickTelemetry()'s own
// comment).
void telemetryEmitBinary(Rt::Blackboard& b, uint32_t now, ReplyFn replyFn, void* replyCtx) {
  if (replyFn == nullptr) return;

  Telemetry::TlmFrameInput in = Telemetry::tick(now, b);
  b.telemetrySeq++;   // advances AFTER this frame captured the PRE-increment
                       // value via Telemetry::tick()

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

}  // namespace

void tickTelemetry(Rt::Blackboard& bb, Rt::CommandRouter& router, uint32_t now) {
  if (bb.telemetryPeriod == 0) return;
  if (bb.telemetryHasLastEmit && (now - bb.telemetryLastEmitMs) < bb.telemetryPeriod) return;

  ReplyFn replyFn = nullptr;
  void* replyCtx = nullptr;
  router.replySink(bb.telemetryChannel, replyFn, replyCtx);

  // 097-008: unconditionally binary now -- the text sibling this used to
  // branch against (bb.telemetryBinary ? telemetryEmitBinary() :
  // telemetryEmit()) is gone along with telemetryEmit() itself. bb.telemetryBinary
  // (blackboard.h) is still WRITTEN by binary_channel.cpp's `stream` arm
  // (StreamControl.binary is still a real wire field a legacy-proxy client
  // could set false) but is no longer READ anywhere -- a known, accepted
  // vestige (mirrors ticket 006's own bb.motionIn precedent, architecture-
  // update-r2.md Open Question 1), not touched here since blackboard.h and
  // binary_channel.cpp are outside this ticket's file scope.
  telemetryEmitBinary(bb, now, replyFn, replyCtx);

  // Mirrors the deleted text handleStream()'s own immediate-first-frame
  // bookkeeping: update unconditionally (even if telemetryEmitBinary() was
  // a silent no-op because replyFn resolved null) so a channel with no
  // wired reply sink does not retry every single pass.
  bb.telemetryLastEmitMs = now;
  bb.telemetryHasLastEmit = true;
}

std::vector<CommandDescriptor> telemetryCommands(Rt::CommandRouter& /*router*/) {
  // 097-008: STREAM/SNAP's text registrations are deleted -- see this
  // file's own header comment for why the function itself (and its
  // Rt::CommandRouter& call signature, unused now) is kept.
  return {};
}

