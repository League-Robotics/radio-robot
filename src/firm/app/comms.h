#pragma once

#include <cstddef>
#include "app/debug.h"
#include <cstdint>

#include "firm/types/robot_state.h"
#include "messages/commands.h"
#include "messages/envelope.h"
#include "messages/wire.h"

#ifndef HOST_BUILD
class SerialPort;
class Radio;
#endif

namespace App {

class Telemetry;

constexpr uint8_t kCobsDelimiter = 0x0A;

class Transport {
 public:
  virtual ~Transport() = default;

  virtual bool readLine(char* buf, uint16_t cap, uint16_t* outLen) = 0;

  virtual void send(const uint8_t* data, uint16_t len) = 0;

  virtual void sendReliable(const char* msg) = 0;
};

#ifndef HOST_BUILD

class SerialTransport : public Transport {
 public:
  explicit SerialTransport(SerialPort& serial);
  bool readLine(char* buf, uint16_t cap, uint16_t* outLen) override;
  void send(const uint8_t* data, uint16_t len) override;
  void sendReliable(const char* msg) override;

 private:
  SerialPort& serial_;
};

class RadioTransport : public Transport {
 public:
  explicit RadioTransport(Radio& radio);
  bool readLine(char* buf, uint16_t cap, uint16_t* outLen) override;
  void send(const uint8_t* data, uint16_t len) override;
  void sendReliable(const char* msg) override;

 private:
  Radio& radio_;
};

#endif

// kMaxEnvelopeBytes -- the larger of the two generated per-direction
// budgets (msg::wire::kCommandEnvelopeMaxEncodedSize (154, as of 132-015 --
// see that constant's own generated size-report comment, wire.h) /
// kReplyEnvelopeMaxEncodedSize (192)) -- one raw-byte scratch buffer,
// reused sequentially for an incoming decode or an outgoing encode (never
// overlapping within a single call). Computed by the constexpr expression
// itself so a future schema regeneration that changes either constant
// updates this one automatically.
//
// 132-013 (patch-surface retirement): kCommandEnvelopeMaxEncodedSize jumped
// from 55 to 234 -- not a regression, the expected cost of finally WIRING
// SetConfigGroup (robot_config.proto's own ~220 B `body`, allocated by
// ticket 001, unwired until now) into CommandEnvelope.cmd.config in place
// of the deleted ConfigDelta (whose curated per-target live-tuning
// messages never exceeded ~50 B).
//
// UPDATE, 132-015 (wire-budget emergency -- kFramedMaxBytes's own
// static_assert margin had fallen to 1 byte): robot_config.proto's
// SetConfigGroup.body/ConfigSnapshot.body `(max_count)` was re-sized from
// 220 to 140 (real largest group, Planner at 117 B, plus modest headroom --
// see that proto's own header comment for the rule and the stakeholder
// direction behind it). That drops kCommandEnvelopeMaxEncodedSize's worst
// arm (`config`) from 228 B to 148 B (total 234 -> 154) and
// kReplyEnvelopeMaxEncodedSize's worst arm from `cfg`=228 B to `tlm`=188 B
// (total 232 -> 192, `tlm` now the binding arm, not `cfg`) -- both
// directions shrink; `tlm` was already close behind `cfg` and is now the
// larger of the two ReplyEnvelope arms.
constexpr uint16_t kMaxEnvelopeBytes =
    (msg::wire::kCommandEnvelopeMaxEncodedSize > msg::wire::kReplyEnvelopeMaxEncodedSize)
        ? msg::wire::kCommandEnvelopeMaxEncodedSize
        : msg::wire::kReplyEnvelopeMaxEncodedSize;  // == 192

// kMaxCrcPayloadBytes -- kMaxEnvelopeBytes + 2 (the CRC-16 appended AFTER
// the schema payload, per the CRC-then-COBS composition -- see comms.cpp's
// sendReply()/decodeBinaryFrame() for the exact byte layout). This is the
// buffer the COBS encode/decode step itself operates on. The command
// prefix lives OUTSIDE the COBS region (see crcOverScope()'s own doc
// comment, comms.cpp), so it does not affect this constant.
constexpr uint16_t kMaxCrcPayloadBytes = kMaxEnvelopeBytes + 2;  // == 194

// kFramedMaxBytes -- worst-case COBS-encoded length of kMaxCrcPayloadBytes
// (194, down from 236 as of 132-015 -- see kMaxEnvelopeBytes's own doc
// comment above for the SetConfigGroup.body/ConfigSnapshot.body
// `(max_count)` shrink that caused this) zero-free bytes:
// cobsEncodedMaxLength(194) = 194 + 194/254 + 1 = 195
// (WireRuntime::cobsEncodedMaxLength()'s own documented formula). This is
// the size of the COBS-encoded region ALONE, because the ASCII command
// prefix is not part of this region (see kMaxLineBytes below for the
// buffer that DOES need room for the prefix).
//
// 132-015: re-picked at 200 (computed-195 + 5B headroom, the same margin
// convention as the "200 chosen pre-132-011" precedent this comment used
// to cite), down from the hand-picked 238 that had been left over from
// 132-013's config-arm wiring -- 238 was never wrong, just no longer
// tight once the `body` cap shrank, and a stale wide margin here is
// exactly the kind of drift this constant's own history warns about (it
// silently stayed at 238 through 132-011/132-013's own GROWTH, each of
// which re-picked it upward; nothing forces a re-pick on a SHRINK, so
// this ticket did it explicitly). The static_assert below is this
// constant's own safety net against the drift running the other way --
// it fires the moment either generated envelope constant grows past what
// this literal can cover, rather than silently overflowing a buffer.
constexpr uint16_t kFramedMaxBytes = 200;
static_assert(kFramedMaxBytes >= kMaxCrcPayloadBytes + kMaxCrcPayloadBytes / 254 + 1,
              "kFramedMaxBytes must cover cobsEncodedMaxLength(kMaxCrcPayloadBytes)");

constexpr size_t maxVerbNameLength() {
  size_t maxLen = 0;
  for (uint8_t i = 0; i < msg::kVerbCount; ++i) {
    size_t len = 0;
    for (const char* p = msg::kVerbTable[i].name; *p != '\0'; ++p) ++len;
    if (len > maxLen) maxLen = len;
  }
  return maxLen;
}

constexpr uint16_t kMaxCommandPrefixBytes = static_cast<uint16_t>(maxVerbNameLength() + 1);

// kMaxLineBytes -- Comms's own inbound/outbound scratch LINE buffer size:
// the longest possible `<COMMAND>':'<COBS+CRC bytes>` content a single
// Transport::readLine()/Transport::send() call ever needs to hold (the
// transport's own trailing `\n` is one further byte, appended by the
// transport itself, not counted here -- matches kFramedMaxBytes's own
// convention of excluding the delimiter).
//
// RESOLVED, 132-015 (was FLAGGED at 132-011/132-013): this constant is now
// 200 + kMaxCommandPrefixBytes (11, the `GET_CONFIG`/`CFG` verb pair's own
// 10-byte name driving this) = 211 -- 39 bytes below
// `Com::SerialPort::kTxBufferCapacity` (250, serial_port.h), the CODAL TX
// ring buffer's own conservative ceiling (physical max 254), up from the
// 1-byte margin (249 vs 250) this comment used to warn about. The fix was
// robot_config.proto's SetConfigGroup.body/ConfigSnapshot.body
// `(max_count)` shrink (220 -> 140, see that proto's own header comment)
// cascading through kMaxEnvelopeBytes -> kMaxCrcPayloadBytes ->
// kFramedMaxBytes (238 -> 200, re-picked, see that constant's own doc
// comment) -> this constant. This is not a buffer-overflow risk
// (`cobsOut`/`line` in comms.cpp are sized to these constants correctly)
// either before or after -- but the PRE-132-015 1-byte margin was a real,
// concrete headroom risk one layer down: `SerialPort::send()`'s own
// backpressure check (`kTxBufferCapacity - txBufferedSize() < frameLen`)
// meant a worst-case CFG reply competing with ANY other still-draining
// serial traffic (e.g. a telemetry frame not yet fully flushed) could be
// silently DROPPED under `Comms::sendReply()`'s async, drop-on-full
// send() policy, not merely delayed -- and telemetry is always-on and
// per-cycle, so that collision was likely, not merely possible. 39 bytes
// of margin does not make a drop impossible, but it is no longer a
// near-certainty on ordinary traffic.
constexpr uint16_t kMaxLineBytes = kFramedMaxBytes + kMaxCommandPrefixBytes;

enum class CmdStatus : uint8_t { kNone = 0, kDecoded = 1 };

struct Cmd {
  CmdStatus status = CmdStatus::kNone;
  msg::CommandEnvelope env;
};

constexpr uint8_t kCmdRingDepth = 12;

constexpr uint8_t kPumpMaxLines = 2 * kCmdRingDepth;

class Comms {
 public:
  struct Status {
    bool ready = false;
    bool active = false;
    bool wheelLeftConnected = false;
    bool wheelRightConnected = false;
    bool otosPresent = false;
    bool wedged = false;
    uint32_t flags = 0;

    uint8_t tlmMode = 1;
  };

  void setStatus(const Status& status) { status_ = status; }

  void updateStatus(const Types::RobotState& state, const Telemetry& tlm);

  enum class TlmAction : uint8_t { kNone, kFrame, kSetOff, kSetAuto, kSetOn, kUnrecognized };

  TlmAction takeTlmAction() {
    const TlmAction action = tlmAction_;
    tlmAction_ = TlmAction::kNone;
    return action;
  }

  // DbgAction -- the parsed effect of one inbound `DBG:<subcmd> ...` line
  // (system-test fault injection; ROBOT_DEBUG builds only -- without it,
  // inbound DBG falls through to the malformed count like any other
  // unhandled cleartext verb). Comms parses and STAGES exactly this;
  // RobotLoop::cycle() drains takeDbgAction() and applies it -- the same
  // stage/consume split TlmAction uses, for the same reason (Comms holds
  // no reference to the subsystems a fault targets).
  //
  // 133-003 added the four LIVE TUNING arms (kVmin/kGain/kASteady/kPos).
  // They are a different KIND of DBG verb from the fault-injection arms
  // above -- they change control parameters rather than inject a fault --
  // but they ride this same channel deliberately: they are RAM-only bench
  // knobs, and .claude/rules/configuration-discipline.md binds PRODUCTION
  // BOOT, not bench tuning ("you should be able to configure individual
  // items without the file -- we're going to do a sweep"). The permanent
  // home for a tuned value is still the robot JSON. Each applies through
  // an App::Drive setter and ECHOES "... applied" -- the echo is not
  // cosmetic: src/tests/bench/velocity_profile_gate.py refuses to report a
  // run until it sees one, so a verb that applied silently would produce a
  // measurement attributed to gains it never actually set.
  enum class DbgActionKind : uint8_t { kNone, kMark, kPing, kWedge, kClear,
                                       kVmin, kGain, kASteady, kPos,
                                       kUnrecognized };
  struct DbgAction {
    DbgActionKind kind = DbgActionKind::kNone;
    char text[64] = {};
    uint8_t port = 0;
    uint32_t duration = 0;  // [ms] kWedge auto-clear; 0 = latched
    // value/value2 -- the tuning arms' operands. One field pair rather
    // than four named ones: DbgAction is a ring element (kDbgRingDepth
    // copies live in Comms), the arms are mutually exclusive, and `kind`
    // already says which quantity `value` holds. The units differ per arm
    // and are therefore documented per arm, not in the field name.
    //   kVmin     value  [mm/s]    speed floor
    //   kASteady  value  [mm/s^2]  Stage C steady gate (Stage B stopped
    //                                gating on it at 133-002)
    //   kPos      value  [mm]      Stage B position-error clamp
    //   kGain     value / value2   [1] LEFT / RIGHT dutyPerSpeed multipliers
    // value2 is meaningful ONLY for kGain (the one two-argument verb);
    // every other arm leaves it at its default.
    float value = 0.0f;
    float value2 = 0.0f;
  };

  DbgAction takeDbgAction() {
    if (dbgCount_ == 0) return DbgAction{};
    const DbgAction action = dbgRing_[dbgHead_];
    dbgHead_ = (dbgHead_ + 1) % kDbgRingDepth;
    --dbgCount_;
    return action;
  }

  void sendTlmReply(TlmAction action);

  Comms(Transport& serialLink, Transport& radioLink, const char* banner, const char* idLine = "ID:unknown");

  void pump(uint32_t now);  // [ms]

  bool takeCommand(Cmd& out);

  uint8_t pendingCommandCount() const { return cmdCount_; }

  void sendReply(const msg::ReplyEnvelope& reply);

  void sendBanner();

  void sendReady();

#if defined(ROBOT_DEBUG) || defined(HOST_BUILD)
  void sendDebug(const char* line);
#endif

 private:
  void sendStatus(Transport& t);
  void sendHelp(Transport& t);

 public:

  uint32_t malformedCount() const { return malformedCount_; }

  uint32_t commandsDroppedCount() const { return commandsDroppedCount_; }

 private:
  bool pumpTransport(Transport& t, uint32_t now);  // [ms]

  void pushCommand(const Cmd& cmd);

  void dispatchLine(Transport& t, const char* line, uint16_t lineLen, Cmd& out, uint32_t now);  // [ms]

  void dispatchCleartext(msg::Verb verb, Transport& t, uint32_t now);  // [ms]

  void decodeBinaryFrame(const uint8_t* command, size_t commandLen, const uint8_t* data, uint16_t dataLen, Cmd& out);

  Transport& serialLink_;
  Transport& radioLink_;
  const char* banner_;
  const char* idLine_;

  Status status_{};

  TlmAction tlmAction_ = TlmAction::kNone;
  static constexpr uint8_t kDbgRingDepth = 4;
  DbgAction dbgRing_[kDbgRingDepth]{};
  uint8_t dbgHead_ = 0;
  uint8_t dbgCount_ = 0;
  void pushDbgAction(const DbgAction& action) {
    if (dbgCount_ >= kDbgRingDepth) return;
    dbgRing_[(dbgHead_ + dbgCount_) % kDbgRingDepth] = action;
    ++dbgCount_;
  }
  Transport* tlmReplyTransport_ = nullptr;
  uint32_t malformedCount_ = 0;
  uint32_t commandsDroppedCount_ = 0;

  Cmd cmdRing_[kCmdRingDepth]{};
  uint8_t cmdHead_ = 0;
  uint8_t cmdCount_ = 0;
};

}
