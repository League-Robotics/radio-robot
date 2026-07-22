// telemetry.cpp -- App::Telemetry implementation. See telemetry.h's file
// header for the module's boundary, its two send paths, and the flags-bit
// layout.
#include "app/telemetry.h"

#include "messages/wire.h"
#include "messages/wire_runtime.h"

namespace App {

Telemetry::Telemetry(Comms& comms, Transport& serialLink, Transport& radioLink)
    : comms_(comms), serialLink_(serialLink), radioLink_(radioLink) {}

void Telemetry::setFrame(const Frame& frame) { frame_ = frame; }

void Telemetry::setSecondaryFrame(const SecondaryFrame& frame) { secondaryFrame_ = frame; }

void Telemetry::setFlag(uint32_t bit, bool active) {
  if (active) {
    flags_ |= bit;
  } else {
    flags_ &= ~bit;
  }
}

void Telemetry::ack(uint32_t corrId, uint32_t errCode) {
  ackCorr_ = corrId;
  ackErr_ = errCode;
  ackPending_ = true;
}

bool Telemetry::primaryDue(uint32_t now) const {
  if (!everEmittedPrimary_) return true;  // always on from boot -- no arming
  return (now - lastPrimaryEmit_) >= kPrimaryPeriod;
}

bool Telemetry::secondaryDue(uint32_t now) const {
  if (!everEmittedSecondary_) return true;
  return (now - lastSecondaryEmit_) >= kSecondaryPeriod;
}

void Telemetry::emit(uint32_t now) {
  bool pDue = primaryDue(now);
  bool sDue = secondaryDue(now);

  // Tie-detection uses a STRICTER "genuinely due" test for secondary's
  // pre-first-ever-emission window: secondaryDue()'s own "!everEmitted
  // Secondary_ -> true" boot bypass (unchanged, still governs the
  // non-tied branch below exactly as before) makes secondary look "due"
  // from t=0, long before a real kSecondaryPeriod has ever elapsed --
  // harmless under a "primary always wins" tie rule (that bypass value is
  // never reached whenever primary is also due), but WOULD spuriously
  // tie-alternate a caller's SECOND-ever call (e.g. exactly
  // kPrimaryPeriod after the first) onto secondary, well before any real
  // starvation exists. Substituting a real elapsed-time check
  // (`now >= kSecondaryPeriod`) for that ONE pre-first-emission window
  // preserves every existing short-run caller's expectation that early
  // calls are primary-only, while still guaranteeing secondary its first
  // slot (via a tie, same as any later one) once genuine time has passed.
  bool sDueForTie = everEmittedSecondary_ ? sDue : (now >= kSecondaryPeriod);

  // Tie: both genuinely due in the same call -- alternate rather than
  // always favoring primary (see telemetry.h's emit() comment: at a real
  // loop period at/above kPrimaryPeriod, primary is due every call, so an
  // unconditional primary-wins rule starves secondary to 0 Hz forever).
  if (pDue && sDueForTie) {
    if (tieFavorsSecondary_) {
      emitSecondary(now);
    } else {
      emitPrimary(now);
    }
    tieFavorsSecondary_ = !tieFavorsSecondary_;
    return;
  }

  if (pDue) {
    emitPrimary(now);
    return;
  }
  if (sDue) {
    emitSecondary(now);
  }
}

void Telemetry::emitPrimary(uint32_t now) {
  msg::Telemetry tlm;

  tlm.now = now;
  tlm.seq = seq_++;
  tlm.mode = frame_.mode;

  // flags -- the single assembly point: OR the caller-staged bits (every
  // status/fault/event/presence bit RobotLoop already computed into
  // flags_ via setFlag()) with Telemetry's OWN internally-tracked
  // ack_fresh bit (kFlagAckFresh, bit 5) -- the one bit no caller ever
  // sets directly (see kFlagAckFresh's own doc comment in telemetry.h).
  uint32_t flags = flags_;
  if (ackPending_) flags |= kFlagAckFresh;
  tlm.flags = flags;
  ackPending_ = false;

  tlm.ack_corr = ackCorr_;
  tlm.ack_err = ackErr_;

  tlm.enc_left = frame_.encLeft;
  tlm.enc_right = frame_.encRight;
  tlm.otos = frame_.otos;
  tlm.pose = frame_.pose;
  tlm.twist = frame_.twist;
  tlm.line = frame_.line;
  tlm.color = frame_.color;

  msg::ReplyEnvelope env;
  env.corr_id = 0;  // unsolicited push -- envelope.proto's own convention
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = tlm;

  comms_.sendReply(env);

  everEmittedPrimary_ = true;
  lastPrimaryEmit_ = now;
  ++primaryEmitCount_;
}

void Telemetry::emitSecondary(uint32_t now) {
  msg::TelemetrySecondary sec;
  sec.now = now;
  sec.has_cmd_vel = secondaryFrame_.hasCmdVel;
  sec.cmd_vel_left = secondaryFrame_.cmdVelLeft;
  sec.cmd_vel_right = secondaryFrame_.cmdVelRight;
  sec.acc_left = secondaryFrame_.accLeft;
  sec.acc_right = secondaryFrame_.accRight;
  sec.glitch_left = secondaryFrame_.glitchLeft;
  sec.glitch_right = secondaryFrame_.glitchRight;
  sec.ts_left = secondaryFrame_.tsLeft;
  sec.ts_right = secondaryFrame_.tsRight;

  // Own top-level armored payload -- same encode+armor sequence as
  // Comms::sendReply(), reused here via
  // App::kArmoredBufSize/WireRuntime::base64Encode() rather than
  // duplicated in a private helper, since Comms's own send path only
  // accepts a ReplyEnvelope (TelemetrySecondary is not one of its oneof
  // arms).
  uint8_t rawBuf[msg::wire::kTelemetrySecondaryMaxEncodedSize];
  const uint16_t n = msg::wire::encode(sec, rawBuf, static_cast<uint16_t>(sizeof(rawBuf)));
  if (n == 0) {
    // Unreachable in practice -- rawBuf is sized from the same generated
    // kTelemetrySecondaryMaxEncodedSize constant encode() itself is
    // budgeted against (mirrors Comms::sendReply()'s own guard). Still
    // count the cycle as "handled" so cadence pacing doesn't retry this
    // frame every subsequent call.
    everEmittedSecondary_ = true;
    lastSecondaryEmit_ = now;
    return;
  }

  char armored[kArmoredBufSize];
  armored[0] = '*';
  armored[1] = 'B';
  size_t b64Len = 0;
  if (!WireRuntime::base64Encode(rawBuf, n, armored + 2, sizeof(armored) - 3, &b64Len)) {
    everEmittedSecondary_ = true;
    lastSecondaryEmit_ = now;
    return;  // same unreachable-in-practice sizing argument as above
  }
  armored[2 + b64Len] = '\0';

  // Broadcast on both transports, async/drop-on-full -- same discipline as
  // Comms::sendReply(): telemetry is always-on and must never stall the
  // loop on backpressure.
  serialLink_.send(armored);
  radioLink_.send(armored);

  everEmittedSecondary_ = true;
  lastSecondaryEmit_ = now;
  ++secondaryEmitCount_;
}

}  // namespace App
