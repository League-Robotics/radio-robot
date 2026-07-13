// telemetry_tick.cpp -- tickTelemetry() + file-local telemetryEmitBinary().
// See telemetry_tick.h for the module contract. Relocated verbatim from
// commands/binary_channel.cpp by this cleanup (itself relocated verbatim
// from the now-deleted telemetry_commands.cpp by 097-011) -- pure move,
// zero behavior change; see telemetry_tick.h's own header comment for why
// this pair now lives here instead of the command dispatcher.
#include "telemetry/telemetry_tick.h"

#include "messages/wire.h"
#include "messages/wire_runtime.h"
#include "telemetry/tlm_frame.h"
#include "types/protocol.h"

namespace {

// kArmoredBufSize (096-003) -- "*B" (2) + base64(kReplyEnvelopeMaxEncodedSize)
// + NUL, rounded up with headroom; the SAME sizing argument and 256-byte
// budget commands/binary_channel.cpp's own (BinaryChannel-namespaced)
// kArmoredBufSize uses (matches
// Subsystems::CommunicatorToCommandProcessorCommand::line's 256-byte
// transport budget, wire_command.h). A DIFFERENT symbol from
// BinaryChannel's own anonymous-namespace kArmoredBufSize (this one lives
// in this file's own global-scope anonymous namespace) -- no redefinition.
constexpr size_t kArmoredBufSize = 256;

// telemetryEmitBinary -- the binary-plane emission path (096-003): the
// tick()-then-advance-seq flow, formats via Telemetry::buildTelemetryMessage()
// into a msg::ReplyEnvelope{tlm}, then encode+armor+send exactly like
// commands/binary_channel.cpp's own BinaryChannel::sendReply() (msg::wire::encode
// -> "*B" + WireRuntime::base64Encode). corr_id = 0 -- this is unsolicited PUSH
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

// telemetryEmitTrace -- (100-009) the MotionTrace push path, sourced from
// bb.motionTrace (committed every pass by MainLoop::commit() from the
// wafer adapter's own lastRecord() getter -- see blackboard.h's own doc
// comment). Same encode+armor+send shape as telemetryEmitBinary() above,
// a SEPARATE ReplyEnvelope arm (`trace`, never a Telemetry extension --
// envelope.proto's own MotionTrace doc comment), corr_id = 0 (unsolicited
// PUSH, same reasoning as telemetryEmitBinary()). Only called from
// tickTelemetry() below, and only when bb.telemetryTrace is armed
// (StreamControl.trace, BinaryChannel's `stream` arm handler).
void telemetryEmitTrace(Rt::Blackboard& b, ReplyFn replyFn, void* replyCtx) {
  if (replyFn == nullptr) return;

  msg::ReplyEnvelope reply;
  reply.corr_id = 0;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::TRACE;
  reply.body.trace = b.motionTrace;

  uint8_t rawBuf[msg::wire::kReplyEnvelopeMaxEncodedSize];
  const uint16_t n = msg::wire::encode(reply, rawBuf, static_cast<uint16_t>(sizeof(rawBuf)));
  if (n == 0) {
    // Unreachable in practice -- same sizing argument as telemetryEmitBinary().
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
  // update-r2.md Open Question 1), not touched here since blackboard.h is
  // outside this ticket's file scope.
  telemetryEmitBinary(bb, now, replyFn, replyCtx);

  // (100-009) MotionTrace push -- ADDITIONAL to the Telemetry push above,
  // same period, only when StreamControl.trace armed it (bb.telemetryTrace).
  if (bb.telemetryTrace) {
    telemetryEmitTrace(bb, replyFn, replyCtx);
  }

  // Mirrors the deleted text handleStream()'s own immediate-first-frame
  // bookkeeping: update unconditionally (even if telemetryEmitBinary() was
  // a silent no-op because replyFn resolved null) so a channel with no
  // wired reply sink does not retry every single pass.
  bb.telemetryLastEmitMs = now;
  bb.telemetryHasLastEmit = true;
}
