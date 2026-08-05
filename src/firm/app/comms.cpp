#include "app/comms.h"
#include "app/debug.h"
#include <cstdlib>

#include <cstdio>
#include <cstring>

#include "app/telemetry.h"
#include "messages/wire_runtime.h"
#include "types/version_generated.h"

#ifndef HOST_BUILD
#include "com/radio.h"
#include "com/serial_port.h"
#endif

namespace App {

#ifndef HOST_BUILD


SerialTransport::SerialTransport(SerialPort& serial) : serial_(serial) {}

bool SerialTransport::readLine(char* buf, uint16_t cap, uint16_t* outLen) {
  return serial_.readLine(buf, cap, outLen);
}

void SerialTransport::send(const uint8_t* data, uint16_t len) { serial_.send(data, len); }

void SerialTransport::sendReliable(const char* msg) { serial_.sendReliable(msg); }


RadioTransport::RadioTransport(Radio& radio) : radio_(radio) {}

bool RadioTransport::readLine(char* buf, uint16_t cap, uint16_t* outLen) {
  return radio_.poll(buf, cap, outLen);
}

void RadioTransport::send(const uint8_t* data, uint16_t len) { radio_.send(data, len); }

void RadioTransport::sendReliable(const char* msg) { radio_.send(reinterpret_cast<const uint8_t*>(msg),
                                                                  static_cast<uint16_t>(std::strlen(msg))); }

#endif

namespace {

uint16_t crcOverScope(const uint8_t* command, size_t commandLen, const uint8_t* payload, size_t payloadLen) {
  uint16_t crc = WireRuntime::crcInit();
  if (commandLen > 0) {
    crc = WireRuntime::crcUpdate(crc, command, commandLen);
    static constexpr uint8_t kCommandSeparator = ':';
    crc = WireRuntime::crcUpdate(crc, &kCommandSeparator, 1);
  }
  return WireRuntime::crcUpdate(crc, payload, payloadLen);
}

bool isRelayControlPlaneLine(const char* line, uint16_t lineLen) {
  if (lineLen == 0) return false;
  const char first = line[0];
  return first == '#' || first == '!' || first == '?';
}

const msg::VerbEntry* findVerb(const char* name, uint16_t nameLen) {
  for (uint8_t i = 0; i < msg::kVerbCount; ++i) {
    const msg::VerbEntry& entry = msg::kVerbTable[i];
    size_t entryLen = 0;
    while (entry.name[entryLen] != '\0') ++entryLen;
    if (entryLen == nameLen && std::memcmp(entry.name, name, nameLen) == 0) return &entry;
  }
  return nullptr;
}

const char* verbName(msg::Verb verb) {
  for (uint8_t i = 0; i < msg::kVerbCount; ++i) {
    if (msg::kVerbTable[i].verb == verb) return msg::kVerbTable[i].name;
  }
  return nullptr;
}

App::Comms::TlmAction classifyTlmArg(const uint8_t* data, uint16_t len) {
  if (len > 0 && data[len - 1] == '\r') --len;

  auto matches = [data, len](const char* token) {
    const size_t tokenLen = std::strlen(token);
    if (tokenLen != len) return false;
    for (size_t i = 0; i < tokenLen; ++i) {
      char a = static_cast<char>(data[i]);
      char b = token[i];
      if (a >= 'a' && a <= 'z') a = static_cast<char>(a - 'a' + 'A');
      if (b >= 'a' && b <= 'z') b = static_cast<char>(b - 'a' + 'A');
      if (a != b) return false;
    }
    return true;
  };

  if (matches("NOW")) return App::Comms::TlmAction::kFrame;
  if (matches("ON")) return App::Comms::TlmAction::kSetOn;
  if (matches("AUTO")) return App::Comms::TlmAction::kSetAuto;
  if (matches("OFF")) return App::Comms::TlmAction::kSetOff;
  return App::Comms::TlmAction::kUnrecognized;
}

// classifyDbgArg() -- tokenize one inbound `DBG:<subcmd> [args]` payload
// (ROBOT_DEBUG builds; see the dispatchLine() interception). Grammar:
//   mark <text>       echo request -- text preserved verbatim
//   ping              inbound-path self-test (replies DBG:pong)
//   wedge left|right|both [ms]   force the wedge latch; 0/omitted = latched
//   clear             clear every injected override AND restore every
//                     tuning value below to what boot installed
//
// 133-003 live-tuning arms (RAM-only; boot restores the robot JSON's own
// value through the generated boot config -- comms.h's DbgActionKind doc
// comment carries the configuration-discipline rationale):
//   vmin <mm/s>       App::Drive speed floor
//   asteady <mm/s^2>  Stage C's bias-adaptation steady gate (Stage B
//                     stopped gating on it at 133-002 -- its I term reads
//                     position now, so there is no accumulator to freeze)
//   pos <mm>          Stage B position-error clamp (133-002's setter)
//   gain <L> <R>      per-wheel multiplier on the BOOT-INSTALLED
//                     dutyPerSpeed pair; `gain 1 1` is the identity
//
// A missing operand, a non-numeric operand, or an out-of-domain value
// (negative for the three scalars; non-positive for either gain
// multiplier -- a zero or negative dutyPerSpeed is not a weak robot, it is
// an uncalibrated or backwards one) all fall through to kUnrecognized,
// which echoes "unrecognized dbg: <text>" rather than applying anything.
// The gate keys on "applied", so a rejected push can never be mistaken for
// a landed one.
#ifdef ROBOT_DEBUG
App::Comms::DbgAction classifyDbgArg(const uint8_t* data, uint16_t len) {
  App::Comms::DbgAction action;
  action.kind = App::Comms::DbgActionKind::kUnrecognized;
  if (data == nullptr || len == 0) return action;
  const size_t cap = sizeof(action.text) - 1;
  const size_t n = (len < cap) ? len : cap;
  std::memcpy(action.text, data, n);
  action.text[n] = '\0';

  char* saveptr = action.text;
  auto isSep = [](char c) {
    return c == ' ' || c == '\n' || c == '\r' || c == '\t';
  };
  auto nextToken = [&saveptr, &isSep]() -> char* {
    while (isSep(*saveptr)) ++saveptr;
    if (*saveptr == '\0') return nullptr;
    char* tok = saveptr;
    while (!isSep(*saveptr) && *saveptr != '\0') ++saveptr;
    if (*saveptr != '\0') { *saveptr = '\0'; ++saveptr; }
    return tok;
  };

  char verbatim[sizeof(action.text)];
  std::memcpy(verbatim, action.text, sizeof(verbatim));

  const char* sub = nextToken();
  if (sub == nullptr) return action;
  if (std::strcmp(sub, "mark") == 0) {
    action.kind = App::Comms::DbgActionKind::kMark;
    std::memcpy(action.text, verbatim, sizeof(action.text));
    return action;
  }
  if (std::strcmp(sub, "ping") == 0) {
    action.kind = App::Comms::DbgActionKind::kPing;
    return action;
  }
  if (std::strcmp(sub, "clear") == 0) {
    action.kind = App::Comms::DbgActionKind::kClear;
    return action;
  }
  // parseScalar -- one non-negative float operand, or false. Unlike the
  // bare std::strtof() the reference patch used, this REJECTS a token that
  // is not a number outright instead of letting strtof()'s own 0.0 return
  // stand in for it: `DBG:vmin oops` must not silently apply a zero floor
  // and then echo "applied", which is exactly the shape of failure the
  // echo contract exists to rule out.
  auto parseScalar = [&nextToken](float& out) -> bool {
    const char* token = nextToken();
    if (token == nullptr) return false;
    char* end = nullptr;
    const float parsed = std::strtof(token, &end);
    if (end == token || *end != '\0') return false;  // empty or trailing junk
    if (!(parsed >= 0.0f)) return false;             // negative, or NaN
    out = parsed;
    return true;
  };

  if (std::strcmp(sub, "vmin") == 0) {
    if (!parseScalar(action.value)) return action;
    action.kind = App::Comms::DbgActionKind::kVmin;
    return action;
  }
  if (std::strcmp(sub, "asteady") == 0) {
    if (!parseScalar(action.value)) return action;
    action.kind = App::Comms::DbgActionKind::kASteady;
    return action;
  }
  if (std::strcmp(sub, "pos") == 0) {
    if (!parseScalar(action.value)) return action;
    action.kind = App::Comms::DbgActionKind::kPos;
    return action;
  }
  if (std::strcmp(sub, "gain") == 0) {
    // BOTH multipliers are required. Defaulting the omitted one to 1.0
    // would be worse than rejecting: `DBG:gain 1.02` would then silently
    // RESET the right wheel to identity while reading as "I only touched
    // the left one".
    if (!parseScalar(action.value)) return action;
    if (!parseScalar(action.value2)) return action;
    if (action.value <= 0.0f || action.value2 <= 0.0f) return action;
    action.kind = App::Comms::DbgActionKind::kGain;
    return action;
  }
  if (std::strcmp(sub, "wedge") == 0) {
    const char* which = nextToken();
    if (which == nullptr) return action;
    if (std::strcmp(which, "left") == 0) action.port = 1;
    else if (std::strcmp(which, "right") == 0) action.port = 2;
    else if (std::strcmp(which, "both") == 0) action.port = 3;
    else return action;
    const char* ms = nextToken();
    if (ms != nullptr) action.duration = static_cast<uint32_t>(std::strtoul(ms, nullptr, 10));
    action.kind = App::Comms::DbgActionKind::kWedge;
    return action;
  }
  return action;
}
#endif

// bodyKindToVerb() -- ReplyEnvelope::BodyKind (envelope.proto) and
// msg::Verb (commands.proto) are two independently-generated enums over
// the SAME reply shapes (TLM/OK/ERR, plus CFG as of 132-011); this is the
// one seam that maps between them so sendReply() can derive its outbound
// command name from `reply.body_kind` alone -- no caller-supplied command
// string to drift out of sync with the envelope it names.
msg::Verb bodyKindToVerb(msg::ReplyEnvelope::BodyKind kind) {
  switch (kind) {
    case msg::ReplyEnvelope::BodyKind::TLM: return msg::Verb::TLM;
    case msg::ReplyEnvelope::BodyKind::OK: return msg::Verb::OK;
    case msg::ReplyEnvelope::BodyKind::ERR: return msg::Verb::ERR;
    case msg::ReplyEnvelope::BodyKind::CFG: return msg::Verb::CFG;
    case msg::ReplyEnvelope::BodyKind::NONE:
    default: return msg::Verb::VERB_UNSPECIFIED;
  }
}

}


Comms::Comms(Transport& serialLink, Transport& radioLink, const char* banner, const char* idLine)
    : serialLink_(serialLink), radioLink_(radioLink), banner_(banner), idLine_(idLine) {}

void Comms::pump(uint32_t now) {
  for (uint8_t consumed = 0; consumed < kPumpMaxLines; ++consumed) {
    if (pumpTransport(serialLink_, now)) continue;
    if (pumpTransport(radioLink_, now)) continue;
    return;
  }
}

bool Comms::pumpTransport(Transport& t, uint32_t now) {
  char line[kMaxLineBytes];
  uint16_t lineLen = 0;
  if (!t.readLine(line, sizeof(line), &lineLen)) return false;

  Cmd decoded;
  dispatchLine(t, line, lineLen, decoded, now);
  if (decoded.status == CmdStatus::kDecoded) pushCommand(decoded);
  return true;
}

void Comms::pushCommand(const Cmd& cmd) {
  if (cmdCount_ >= kCmdRingDepth) {
    ++commandsDroppedCount_;
    return;
  }
  const uint8_t slot = static_cast<uint8_t>((cmdHead_ + cmdCount_) % kCmdRingDepth);
  cmdRing_[slot] = cmd;
  ++cmdCount_;
}

bool Comms::takeCommand(Cmd& out) {
  if (cmdCount_ == 0) return false;
  out = cmdRing_[cmdHead_];
  cmdHead_ = static_cast<uint8_t>((cmdHead_ + 1) % kCmdRingDepth);
  --cmdCount_;
  return true;
}

void Comms::dispatchLine(Transport& t, const char* line, uint16_t lineLen, Cmd& out, uint32_t now) {
  if (isRelayControlPlaneLine(line, lineLen)) return;

  uint16_t colonPos = lineLen;
  for (uint16_t i = 0; i < lineLen; ++i) {
    if (line[i] == ':') {
      colonPos = i;
      break;
    }
  }

  const char* cmdPtr = line;
  uint16_t cmdLen = colonPos;
  const uint8_t* dataPtr = nullptr;
  uint16_t dataLen = 0;
  if (colonPos < lineLen) {
    dataPtr = reinterpret_cast<const uint8_t*>(line + colonPos + 1);
    dataLen = static_cast<uint16_t>(lineLen - colonPos - 1);
  } else {
    if (cmdLen > 0 && cmdPtr[cmdLen - 1] == '\r') --cmdLen;
  }

  const msg::VerbEntry* entry = findVerb(cmdPtr, cmdLen);
  if (entry == nullptr) {
    ++malformedCount_;
    return;
  }

  if (entry->verb == msg::Verb::TLM) {
    tlmReplyTransport_ = &t;
    tlmAction_ = (colonPos >= lineLen) ? TlmAction::kFrame : classifyTlmArg(dataPtr, dataLen);
    return;
  }

#ifdef ROBOT_DEBUG
  if (entry->verb == msg::Verb::DBG) {
    pushDbgAction(classifyDbgArg(dataPtr, dataLen));
    return;
  }
#endif

  if (entry->binary) {
    decodeBinaryFrame(reinterpret_cast<const uint8_t*>(cmdPtr), cmdLen, dataPtr, dataLen, out);
  } else {
    dispatchCleartext(entry->verb, t, now);
  }
}

void Comms::dispatchCleartext(msg::Verb verb, Transport& t, uint32_t now) {
  switch (verb) {
    case msg::Verb::HELLO:
      t.sendReliable(banner_);
      return;
    case msg::Verb::PING: {
      char pong[32];
      std::snprintf(pong, sizeof(pong), "PONG:t=%lu", static_cast<unsigned long>(now));
      t.sendReliable(pong);
      return;
    }
    case msg::Verb::ID:
      t.sendReliable(idLine_);
      return;
    case msg::Verb::VER:
      t.sendReliable("VER:" FIRMWARE_VERSION_STR);
      return;
    case msg::Verb::STATUS:
      sendStatus(t);
      return;
    case msg::Verb::HELP:
      sendHelp(t);
      return;
    default:
      ++malformedCount_;
      return;
  }
}

void Comms::decodeBinaryFrame(const uint8_t* command, size_t commandLen, const uint8_t* data, uint16_t dataLen,
                               Cmd& out) {
  uint8_t combined[kMaxCrcPayloadBytes];
  size_t combinedLen = 0;
  if (!WireRuntime::cobsDecode(data, dataLen, combined, sizeof(combined), &combinedLen, kCobsDelimiter)) {
    ++malformedCount_;
    return;
  }
  if (combinedLen < 2) {
    ++malformedCount_;
    return;
  }

  const size_t payloadLen = combinedLen - 2;
  size_t crcPos = payloadLen;
  uint16_t receivedCrc = 0;
  if (!WireRuntime::decodeCrc16(combined, combinedLen, &crcPos, &receivedCrc)) {
    ++malformedCount_;
    return;
  }
  const uint16_t expectedCrc = crcOverScope(command, commandLen, combined, payloadLen);
  if (expectedCrc != receivedCrc) {
    ++malformedCount_;
    return;
  }

  msg::CommandEnvelope decoded;
  const msg::wire::Result r =
      msg::wire::decode(decoded, combined, static_cast<uint16_t>(payloadLen));
  if (!r.ok) {
    ++malformedCount_;
    return;
  }

  out.status = CmdStatus::kDecoded;
  out.env = decoded;
}

void Comms::sendReply(const msg::ReplyEnvelope& reply) {
  const msg::Verb verb = bodyKindToVerb(reply.body_kind);
  const char* name = verbName(verb);
  if (name == nullptr) return;
  const size_t nameLen = std::strlen(name);

  uint8_t rawBuf[kMaxEnvelopeBytes];
  const uint16_t n = msg::wire::encode(reply, rawBuf, static_cast<uint16_t>(sizeof(rawBuf)));
  if (n == 0) {
    return;
  }

  uint8_t combined[kMaxCrcPayloadBytes];
  std::memcpy(combined, rawBuf, n);
  size_t combinedLen = n;
  const uint16_t crc = crcOverScope(reinterpret_cast<const uint8_t*>(name), nameLen, rawBuf, n);
  if (!WireRuntime::encodeCrc16(crc, combined, sizeof(combined), &combinedLen)) {
    return;
  }

  uint8_t cobsOut[kFramedMaxBytes];
  size_t cobsLen = 0;
  if (!WireRuntime::cobsEncode(combined, combinedLen, cobsOut, sizeof(cobsOut), &cobsLen, kCobsDelimiter)) {
    return;
  }

  uint8_t line[kMaxLineBytes];
  if (nameLen + 1 + cobsLen > sizeof(line)) {
    return;
  }
  std::memcpy(line, name, nameLen);
  line[nameLen] = ':';
  std::memcpy(line + nameLen + 1, cobsOut, cobsLen);
  const uint16_t lineLen = static_cast<uint16_t>(nameLen + 1 + cobsLen);

  serialLink_.send(line, lineLen);
  radioLink_.send(line, lineLen);
}

void Comms::sendBanner() {
  serialLink_.sendReliable(banner_);
  radioLink_.sendReliable(banner_);
}

void Comms::sendStatus(Transport& t) {
  const char* tlmStr = "auto";
  switch (status_.tlmMode) {
    case 0: tlmStr = "off"; break;
    case 1: tlmStr = "auto"; break;
    case 2: tlmStr = "on"; break;
    default: break;
  }

  char line[128];
  std::snprintf(line, sizeof(line),
                "STATUS:ready=%d:active=%d:connL=%d:connR=%d:otos=%d"
                ":wedge=%d:flags=0x%lx:tlm=%s",
                status_.ready ? 1 : 0, status_.active ? 1 : 0,
                status_.wheelLeftConnected ? 1 : 0,
                status_.wheelRightConnected ? 1 : 0,
                status_.otosPresent ? 1 : 0, status_.wedged ? 1 : 0,
                static_cast<unsigned long>(status_.flags), tlmStr);
  t.sendReliable(line);
}

void Comms::sendHelp(Transport& t) {
  char line[192];
  std::size_t n = 0;
  n += static_cast<std::size_t>(std::snprintf(line, sizeof(line), "HELP:"));
  for (std::size_t i = 0; i < msg::kVerbCount && n + 1 < sizeof(line); ++i) {
    const msg::VerbEntry& e = msg::kVerbTable[i];
    if (e.binary && e.verb != msg::Verb::TLM) continue;
    const char* token = (e.verb == msg::Verb::TLM) ? "TLM[:NOW|ON|AUTO|OFF]" : e.name;
    const int written = std::snprintf(line + n, sizeof(line) - n, "%s%s",
                                      n > 5 ? " " : "", token);
    if (written <= 0) break;
    n += static_cast<std::size_t>(written);
  }
  t.sendReliable(line);
}

void Comms::sendTlmReply(TlmAction action) {
  if (tlmReplyTransport_ == nullptr) return;
  switch (action) {
    case TlmAction::kSetOff:
    case TlmAction::kSetAuto:
    case TlmAction::kSetOn:
      sendStatus(*tlmReplyTransport_);
      break;
    case TlmAction::kUnrecognized:
      sendHelp(*tlmReplyTransport_);
      break;
    case TlmAction::kNone:
    case TlmAction::kFrame:
    default:
      break;
  }
}

void Comms::sendReady() {
  serialLink_.sendReliable("READY");
  radioLink_.sendReliable("READY");
}

#if defined(ROBOT_DEBUG) || defined(HOST_BUILD)
void Comms::sendDebug(const char* line) {
  char buf[210];
  std::snprintf(buf, sizeof(buf), "DBG:%s", line);
  serialLink_.sendReliable(buf);
  radioLink_.sendReliable(buf);
}
#endif

void Comms::updateStatus(const Types::RobotState& state, const Telemetry& tlm) {
  Status status;
  status.ready = state.health.ready;
  status.active = state.command.moveActive;
  status.wheelLeftConnected = state.wheelLeft.connected;
  status.wheelRightConnected = state.wheelRight.connected;
  status.otosPresent = state.otos.present;
  status.wedged = state.health.wedgeLatch;
  status.flags = tlm.flags();
  status.tlmMode = static_cast<uint8_t>(tlm.mode());
  status_ = status;
}

}
