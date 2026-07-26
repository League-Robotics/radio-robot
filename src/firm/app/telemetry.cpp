// telemetry.cpp -- App::Telemetry implementation. See telemetry.h's file
// header for the module's boundary, its two send paths, and the flags-bit
// layout.
#include "app/telemetry.h"

#include <cstring>

#include "messages/wire.h"
#include "messages/wire_runtime.h"

namespace App {

// kAckRingDepth (telemetry.h) must match the generated msg::Telemetry::
// acks_[] array width (telemetry.proto's Telemetry.acks max_count) exactly
// -- a mismatch here would silently truncate (kAckRingDepth too big) or
// under-fill (too small) the wire array in emitPrimary() below. sizeof on
// a temporary's member is a valid, never-evaluated sizeof operand (the
// object is never constructed at runtime) -- the same "prove a size fact
// at compile time with no runtime cost" idiom this file's own generated
// siblings (layout_checks.h) already use for standard-layout checks.
//
// 124-008: acks_[] is now uint32_t[4] (packed corr_id<<4|err), not
// msg::AckEntry[4] -- msg::AckEntry is deleted (issue §B4).
static_assert(sizeof(msg::Telemetry{}.acks_) == static_cast<size_t>(kAckRingDepth) * sizeof(uint32_t),
              "App::kAckRingDepth (telemetry.h) must match telemetry.proto's Telemetry.acks (max_count)");

// Packed ack-ring word format (124-008, issue §B4): corr_id in the upper
// 28 bits, err (an ErrCode, 0-8) in the low 4 bits. err's real range fits
// comfortably in 4 bits (ERR_NOT_CONFIGURED=8 is the enum's own ceiling);
// corr_id needs 16 bits in practice (CommandEnvelope.corr_id/Move.id) and
// has 28 available.
constexpr uint32_t kAckErrBits = 4;
constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;

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
  pushAckRing(corrId, errCode);
}

void Telemetry::pushAckRing(uint32_t corrId, uint32_t errCode) {
  // Classic bounded circular buffer: the tail (next-write) slot is always
  // (ackRingHead_ + ackRingCount_) % kAckRingDepth. While there's spare
  // capacity, that slot is unused -- write it and grow the count. Once
  // full (count == depth), that SAME formula reduces to ackRingHead_
  // itself (the oldest entry's own slot) -- write the new value there,
  // evicting the oldest, then advance ackRingHead_ by one so the
  // just-written slot becomes the newest and the next slot over becomes
  // the new oldest.
  uint8_t tail;
  if (ackRingCount_ < kAckRingDepth) {
    tail = static_cast<uint8_t>((ackRingHead_ + ackRingCount_) % kAckRingDepth);
    ++ackRingCount_;
  } else {
    tail = ackRingHead_;
    ackRingHead_ = static_cast<uint8_t>((ackRingHead_ + 1) % kAckRingDepth);
  }
  // Packed word (124-008, issue §B4): corr_id<<4 | err.
  ackRing_[tail] = (corrId << kAckErrBits) | (errCode & kAckErrMask);
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
  // seq wraps at telemetry.proto's own declared (max)=127 (124-008, issue
  // §B5's size-accounting) -- kept genuinely small on the real value too,
  // not just the wire bound, so the two never drift: a caller reading the
  // declared bound sees the true worst case, not an optimistic one.
  tlm.seq = seq_;
  seq_ = (seq_ + 1) % 128u;
  tlm.mode = frame_.mode;

  // flags -- the single assembly point: the caller-staged bits (every
  // status/fault/event/presence bit RobotLoop already computed into
  // flags_ via setFlag()). 124-008 (issue §B4) deleted the single
  // "freshest ack" scalar slot and its own ack_fresh bit (bit 5) --
  // Telemetry no longer ORs in an internally-tracked bit here.
  tlm.flags = flags_;

  // Ack ring (120, ADDITIVE) -- serialize the ring's CURRENT contents,
  // oldest-to-newest (ackRingHead_'s own push/evict order), every
  // emitPrimary() call, exactly like every other Frame field: the last
  // staged snapshot, not a diff since the last send (app/DESIGN.md Sec 3).
  // A frame this call sends carries whatever the ring holds RIGHT NOW,
  // regardless of whether a new ack() landed since the last emit -- no
  // separate "is this new" bit applies (telemetry.proto's own
  // Telemetry.acks doc comment). 124-008: each entry is now a packed
  // uint32_t (corr_id<<4|err), not msg::AckEntry (deleted).
  tlm.acks_count = ackRingCount_;
  for (uint8_t i = 0; i < ackRingCount_; ++i) {
    const uint8_t idx = static_cast<uint8_t>((ackRingHead_ + i) % kAckRingDepth);
    tlm.acks_[i] = ackRing_[idx];
  }

  tlm.enc_left = frame_.encLeft;
  tlm.enc_right = frame_.encRight;
  tlm.otos = frame_.otos;
  tlm.pose = frame_.pose;
  tlm.twist = frame_.twist;
  tlm.line = frame_.line;
  tlm.color = frame_.color;

  // 123-004 (migrated from TelemetrySecondary -- see Frame's own doc
  // comment, telemetry.h): loop-timing diagnostics, now fresh every
  // primary frame instead of a ~5Hz secondary-frame sample.
  tlm.cycle_busy = frame_.cycleBusy;
  tlm.cycle_period = frame_.cyclePeriod;

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
  // cycle_busy/cycle_period (122-003) formerly lived here as an interim
  // placement -- MIGRATED to the primary frame (123-004, emitPrimary()
  // above) now that COBS+CRC restored primary-frame headroom; see
  // SecondaryFrame's own doc comment, telemetry.h.

  // Own top-level framed LINE -- same CRC-then-COBS composition (123-002
  // -- was "*B"+base64 pre-123), reused here via App::kMaxCrcPayloadBytes/
  // kFramedMaxBytes/WireRuntime's COBS+CRC primitives rather than
  // duplicated in a private helper, since Comms::sendReply() only accepts
  // a ReplyEnvelope (TelemetrySecondary is not one of its oneof arms).
  //
  // 124-005: rides the SAME "TLM:" command prefix/CRC-scope emitPrimary()
  // uses (Comms::sendReply() with body_kind=TLM) -- there is no separate
  // registry verb for the secondary frame (messages/commands.h's closed
  // set has exactly one telemetry-push verb), and the host already
  // disambiguates the two shapes structurally (tries ReplyEnvelope first,
  // falls back to TelemetrySecondary -- serial_conn.py's own
  // _handle_binary_reply() docstring), independent of the ASCII framing
  // layer. `kSecondaryCommand` is the SAME bytes for both the CRC-scope
  // input and the wire prefix actually written below, so the two can
  // never drift apart.
  static constexpr uint8_t kSecondaryCommand[] = {'T', 'L', 'M'};
  constexpr size_t kSecondaryCommandLen = sizeof(kSecondaryCommand);

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

  uint8_t combined[kMaxCrcPayloadBytes];
  std::memcpy(combined, rawBuf, n);
  size_t combinedLen = n;
  uint16_t crc = WireRuntime::crcInit();
  crc = WireRuntime::crcUpdate(crc, kSecondaryCommand, kSecondaryCommandLen);
  static constexpr uint8_t kCommandSeparator = ':';
  crc = WireRuntime::crcUpdate(crc, &kCommandSeparator, 1);
  crc = WireRuntime::crcUpdate(crc, rawBuf, n);
  if (!WireRuntime::encodeCrc16(crc, combined, sizeof(combined), &combinedLen)) {
    everEmittedSecondary_ = true;
    lastSecondaryEmit_ = now;
    return;  // same unreachable-in-practice sizing argument as above
  }

  uint8_t cobsOut[kFramedMaxBytes];
  size_t cobsLen = 0;
  if (!WireRuntime::cobsEncode(combined, combinedLen, cobsOut, sizeof(cobsOut), &cobsLen, kCobsDelimiter)) {
    everEmittedSecondary_ = true;
    lastSecondaryEmit_ = now;
    return;  // same unreachable-in-practice sizing argument as above
  }

  uint8_t line[kMaxLineBytes];
  if (kSecondaryCommandLen + 1 + cobsLen > sizeof(line)) {
    // Unreachable in practice -- kMaxLineBytes covers the worst case.
    everEmittedSecondary_ = true;
    lastSecondaryEmit_ = now;
    return;
  }
  std::memcpy(line, kSecondaryCommand, kSecondaryCommandLen);
  line[kSecondaryCommandLen] = ':';
  std::memcpy(line + kSecondaryCommandLen + 1, cobsOut, cobsLen);
  const uint16_t lineLen = static_cast<uint16_t>(kSecondaryCommandLen + 1 + cobsLen);

  // Broadcast on both transports, async/drop-on-full -- same discipline as
  // Comms::sendReply(): telemetry is always-on and must never stall the
  // loop on backpressure. The concrete transport appends the trailing
  // '\n' terminator itself.
  serialLink_.send(line, lineLen);
  radioLink_.send(line, lineLen);

  everEmittedSecondary_ = true;
  lastSecondaryEmit_ = now;
  ++secondaryEmitCount_;
}

}  // namespace App
