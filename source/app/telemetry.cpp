// telemetry.cpp -- App::Telemetry implementation. See telemetry.h's file
// header for the module's boundary, its two send paths, and the
// fault_bits/event_bits layout.
#include "app/telemetry.h"

#include "messages/wire.h"
#include "messages/wire_runtime.h"

namespace App {

Telemetry::Telemetry(Comms& comms, Transport& serialLink, Transport& radioLink)
    : comms_(comms), serialLink_(serialLink), radioLink_(radioLink) {}

void Telemetry::setFrame(const Frame& frame) { frame_ = frame; }

void Telemetry::setSecondaryFrame(const SecondaryFrame& frame) { secondaryFrame_ = frame; }

void Telemetry::setFault(uint32_t bit, bool active) {
  if (active) {
    faultBits_ |= bit;
  } else {
    faultBits_ &= ~bit;
  }
}

void Telemetry::setEvent(uint32_t bit, bool active) {
  if (active) {
    eventBits_ |= bit;
  } else {
    eventBits_ &= ~bit;
  }
}

void Telemetry::ack(uint32_t corrId, msg::AckStatus status, uint32_t errCode) {
  msg::AckEntry entry;
  entry.corr_id = corrId;
  entry.status = status;
  entry.err_code = errCode;

  if (ringCount_ < 3) {
    ring_[ringCount_++] = entry;
    return;
  }
  // Ring full -- evict the oldest (index 0), shift the remaining two down,
  // append the new entry at the end. ring_[] stays chronological
  // (oldest-first, newest-last) at all times.
  ring_[0] = ring_[1];
  ring_[1] = ring_[2];
  ring_[2] = entry;
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
  // Primary checked first, unconditionally sent when due -- secondary can
  // never delay it (this file's own scheduling note).
  if (primaryDue(now)) {
    emitPrimary(now);
    return;
  }
  if (secondaryDue(now)) {
    emitSecondary(now);
  }
}

void Telemetry::emitPrimary(uint32_t now) {
  msg::Telemetry tlm;
  for (uint8_t i = 0; i < ringCount_; ++i) tlm.acks_[i] = ring_[i];
  tlm.acks_count = ringCount_;

  tlm.now = now;
  tlm.mode = frame_.mode;
  tlm.seq = seq_++;

  tlm.has_enc = frame_.hasEnc;
  tlm.enc_left = frame_.encLeft;
  tlm.enc_right = frame_.encRight;

  tlm.has_vel = frame_.hasVel;
  tlm.vel_left = frame_.velLeft;
  tlm.vel_right = frame_.velRight;

  tlm.has_pose = frame_.hasPose;
  tlm.pose = frame_.pose;

  tlm.has_otos = frame_.hasOtos;
  tlm.otos = frame_.otos;
  tlm.otos_connected = frame_.otosConnected;

  tlm.has_twist = frame_.hasTwist;
  tlm.twist = frame_.twist;

  tlm.active = frame_.active;
  tlm.conn_left = frame_.connLeft;
  tlm.conn_right = frame_.connRight;

  tlm.fault_bits = faultBits_;
  tlm.event_bits = eventBits_;

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

  // Own top-level armored payload (telemetry.proto's own Decision 3
  // resolution) -- same encode+armor sequence as Comms::sendReply(), reused
  // here via App::kArmoredBufSize/WireRuntime::base64Encode() rather than
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
