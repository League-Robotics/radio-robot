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

#endif  // HOST_BUILD

constexpr uint16_t kMaxEnvelopeBytes =
    (msg::wire::kCommandEnvelopeMaxEncodedSize > msg::wire::kReplyEnvelopeMaxEncodedSize)
        ? msg::wire::kCommandEnvelopeMaxEncodedSize
        : msg::wire::kReplyEnvelopeMaxEncodedSize;  // == 192

constexpr uint16_t kMaxCrcPayloadBytes = kMaxEnvelopeBytes + 2;  // == 194

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

constexpr uint16_t kMaxCommandPrefixBytes = static_cast<uint16_t>(maxVerbNameLength() + 1);  // name + ':'

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
    bool ready = false;             // boot() finished; Moves are accepted
    bool active = false;            // a Move is running
    bool wheelLeftConnected = false;
    bool wheelRightConnected = false;
    bool otosPresent = false;
    bool wedged = false;            // encoder stuck-position latch
    uint32_t flags = 0;             // the full telemetry flags word

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

  enum class DbgActionKind : uint8_t { kNone, kMark, kPing, kWedge, kClear,
                                       kVmin, kGain, kASteady, kPos, kOtos,
                                       kUnrecognized };
  struct DbgAction {
    DbgActionKind kind = DbgActionKind::kNone;
    char text[64] = {};   // kMark: the full original data ("mark leg1a")
    uint8_t port = 0;     // kWedge: 1 = left, 2 = right, 3 = both
    uint32_t duration = 0;  // [ms] kWedge auto-clear; 0 = latched
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
  DbgAction dbgRing_[kDbgRingDepth]{};  // staged by dispatchLine(); drained by RobotLoop
  uint8_t dbgHead_ = 0;
  uint8_t dbgCount_ = 0;
  void pushDbgAction(const DbgAction& action) {
    if (dbgCount_ >= kDbgRingDepth) return;  // drop-newest
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

}  // namespace App
