// comms.cpp -- App::Comms implementation. See comms.h's file header for
// the module's boundary.
#include "app/comms.h"

#include <cstdio>
#include <cstring>

#include "messages/wire_runtime.h"
#include "types/version_generated.h"

#ifndef HOST_BUILD
#include "com/radio.h"
#include "com/serial_port.h"
#endif

namespace App {

#ifndef HOST_BUILD

// --- SerialTransport ---------------------------------------------------

SerialTransport::SerialTransport(SerialPort& serial) : serial_(serial) {}

bool SerialTransport::readLine(char* buf, uint16_t cap, uint16_t* outLen) {
  return serial_.readLine(buf, cap, outLen);
}

void SerialTransport::send(const uint8_t* data, uint16_t len) { serial_.send(data, len); }

void SerialTransport::sendReliable(const char* msg) { serial_.sendReliable(msg); }

// --- RadioTransport ------------------------------------------------------

RadioTransport::RadioTransport(Radio& radio) : radio_(radio) {}

bool RadioTransport::readLine(char* buf, uint16_t cap, uint16_t* outLen) {
  return radio_.poll(buf, cap, outLen);
}

void RadioTransport::send(const uint8_t* data, uint16_t len) { radio_.send(data, len); }

void RadioTransport::sendReliable(const char* msg) { radio_.send(reinterpret_cast<const uint8_t*>(msg),
                                                                  static_cast<uint16_t>(std::strlen(msg))); }

#endif  // HOST_BUILD

namespace {

// crcOverScope() -- the CRC-scope composition protocol v5 needs (124-003/
// 124-005, issue §3): `crc16(COMMAND ':' payload)` when a command name is
// given, `crc16(payload)` alone (byte-identical to protocol v4's CRC) when
// it is not. Threads WireRuntime's incremental crcInit()/crcUpdate() so the
// command bytes and the payload bytes -- two ranges that are never
// adjacent in memory -- are never concatenated into a scratch buffer just
// to hash them together. This composition (the ':' separator, "empty
// command means no scope extension") is protocol grammar, which is
// Comms's boundary, not WireRuntime's (see messages/DESIGN.md's
// three-layer split) -- WireRuntime only supplies the generic incremental
// primitive.
uint16_t crcOverScope(const uint8_t* command, size_t commandLen, const uint8_t* payload, size_t payloadLen) {
  uint16_t crc = WireRuntime::crcInit();
  if (commandLen > 0) {
    crc = WireRuntime::crcUpdate(crc, command, commandLen);
    static constexpr uint8_t kCommandSeparator = ':';
    crc = WireRuntime::crcUpdate(crc, &kCommandSeparator, 1);
  }
  return WireRuntime::crcUpdate(crc, payload, payloadLen);
}

// isRelayControlPlaneLine() -- 124-010 (relay-handshake-trips-comms-
// malformed.md): true if `line`'s first byte is one of the radio-relay
// dongle's own control-plane sigils -- '#' (a status/comment reply, e.g.
// "# entering data plane"), '!' (a dongle command, e.g. "!MODE RAW250"),
// or '?' (the dongle's own config query). host/robot_radio/io/
// serial_conn.py's `_relay_handshake()`/`_banner_classify()` are the only
// callers that ever write bytes shaped like this, always addressed to the
// RELAY's own control-plane parser, never intended for the robot. On a
// fresh relay connect a fragment of that handshake traffic can reach the
// robot's `radioLink_` before (or at the exact moment of) the dongle
// committing to transparent RAW250 pass-through -- see the ticket's
// closing notes for the root-cause analysis. No registered v5 verb name
// (messages/commands.h's `kVerbTable[]`) starts with any of these three
// bytes, so recognizing and dropping such a line here can never mask a
// genuine unrecognized command; it only tolerates exactly the shape of
// noise the relay's own control plane is known to emit. Mirrors the
// tolerance `host/robot_radio/io/serial_conn.py`'s own
// `_handle_wire_line()` already has for '#' lines -- this ticket extends
// the same idea to the robot's own firmware side, which had none.
bool isRelayControlPlaneLine(const char* line, uint16_t lineLen) {
  if (lineLen == 0) return false;
  const char first = line[0];
  return first == '#' || first == '!' || first == '?';
}

// findVerb() -- registry lookup (124-001's messages/commands.h::
// kVerbTable[]) by exact-length ASCII name match. The SOLE discriminator
// for how a wire line's data is read (issue §1) -- returns nullptr for
// any name not in the closed set, which callers count as malformed.
const msg::VerbEntry* findVerb(const char* name, uint16_t nameLen) {
  for (uint8_t i = 0; i < msg::kVerbCount; ++i) {
    const msg::VerbEntry& entry = msg::kVerbTable[i];
    size_t entryLen = 0;
    while (entry.name[entryLen] != '\0') ++entryLen;
    if (entryLen == nameLen && std::memcmp(entry.name, name, nameLen) == 0) return &entry;
  }
  return nullptr;
}

// verbName() -- the reverse lookup sendReply() needs: messages/commands.h's
// generated name string for a given Verb, or nullptr if the enumerator
// isn't in kVerbTable[] at all (VERB_UNSPECIFIED -- never a real wire verb,
// commands.proto's own doc comment).
const char* verbName(msg::Verb verb) {
  for (uint8_t i = 0; i < msg::kVerbCount; ++i) {
    if (msg::kVerbTable[i].verb == verb) return msg::kVerbTable[i].name;
  }
  return nullptr;
}

// classifyTlmArg() -- Part 4's case-insensitive cleartext token match for
// `TLM:<data>` (telemetry-emit-policy-rebuild-spec.md). A real binary
// telemetry frame only ever travels robot->host (this file's own
// dispatchLine() doc comment on the TLM verb), so an inbound `TLM:` line's
// data is ALWAYS one of these cleartext tokens, never a COBS+CRC binary
// body -- safe to strip a single trailing '\r' before comparing, the same
// allowance dispatchLine()'s no-colon branch already makes for a raw
// terminal's "\r\n" (issue Part 8 criterion 15, human-typed TLM:ON).
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

// bodyKindToVerb() -- ReplyEnvelope::BodyKind (envelope.proto) and
// msg::Verb (commands.proto) are two independently-generated enums over
// the SAME three reply shapes (TLM/OK/ERR); this is the one seam that
// maps between them so sendReply() can derive its outbound command name
// from `reply.body_kind` alone -- no caller-supplied command string to
// drift out of sync with the envelope it names.
msg::Verb bodyKindToVerb(msg::ReplyEnvelope::BodyKind kind) {
  switch (kind) {
    case msg::ReplyEnvelope::BodyKind::TLM: return msg::Verb::TLM;
    case msg::ReplyEnvelope::BodyKind::OK: return msg::Verb::OK;
    case msg::ReplyEnvelope::BodyKind::ERR: return msg::Verb::ERR;
    case msg::ReplyEnvelope::BodyKind::NONE:
    default: return msg::Verb::VERB_UNSPECIFIED;
  }
}

}  // namespace

// --- Comms -----------------------------------------------------------

Comms::Comms(Transport& serialLink, Transport& radioLink, const char* banner, const char* idLine)
    : serialLink_(serialLink), radioLink_(radioLink), banner_(banner), idLine_(idLine) {}

void Comms::pump(uint32_t now) {
  // Serial first, radio only when serial has nothing -- the same priority
  // the pre-ring pump() had; what changed is that having read one line is
  // no longer a reason to stop. Bounded by kPumpMaxLines so a flooding
  // host cannot hold the cycle (see that constant's own comment, comms.h).
  for (uint8_t consumed = 0; consumed < kPumpMaxLines; ++consumed) {
    if (pumpTransport(serialLink_, now)) continue;
    if (pumpTransport(radioLink_, now)) continue;
    return;  // both transports dry
  }
}

bool Comms::pumpTransport(Transport& t, uint32_t now) {
  char line[kMaxLineBytes];
  uint16_t lineLen = 0;
  if (!t.readLine(line, sizeof(line), &lineLen)) return false;

  // A LOCAL Cmd per line, pushed onto the ring only on decode success --
  // the same "never publish partial state" discipline decodeBinaryFrame()
  // already had against the old caller-supplied `out` (see comms.h).
  // Cleartext verbs and malformed lines leave status == kNone and push
  // nothing, while still counting as a consumed line so pump() keeps
  // draining past them.
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
  // 124-010 (relay-handshake-trips-comms-malformed.md): drop a leaked
  // relay control-plane line BEFORE the registry lookup, uncounted --
  // see isRelayControlPlaneLine()'s own doc comment above for why this
  // cannot mask a genuine malformed command.
  if (isRelayControlPlaneLine(line, lineLen)) return;

  // Uniform packet grammar (124-005, issue §1): `<COMMAND>[':' <data>]` --
  // the FIRST ':' ends the command; every later byte (including further
  // ':' bytes) is data. Whether that data is cleartext or binary is a
  // property of the COMMAND (the registry lookup below), decided BEFORE
  // any of the data's own bytes are inspected.
  uint16_t colonPos = lineLen;  // sentinel: no ':' found
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
    // No ':' at all -- a bare command candidate. Under this protocol a
    // colon-less line can only ever be attempting one of the no-data
    // cleartext verbs (HELLO/PING/ID/VER); every binary verb always
    // carries a ':'-prefixed COBS body, even an empty one. A single
    // trailing '\r' here is therefore always line-ending noise (a raw
    // terminal sending "\r\n"), never binary content -- safe to strip
    // BEFORE the registry lookup, unlike data after a ':' (issue §7:
    // "strip \r only for cleartext commands, which the parsed prefix
    // already identifies" -- for the no-':' shape the prefix IS the
    // whole line, so this is that same rule, not an exception to it).
    if (cmdLen > 0 && cmdPtr[cmdLen - 1] == '\r') --cmdLen;
  }

  const msg::VerbEntry* entry = findVerb(cmdPtr, cmdLen);
  if (entry == nullptr) {
    ++malformedCount_;
    return;
  }

  // TLM's whole inbound surface (Part 4, telemetry-emit-policy-rebuild-
  // spec.md) is intercepted HERE, before the registry's `binary` flag is
  // even consulted: a real binary telemetry frame only ever travels
  // robot->host, so an inbound, robot-ward `TLM`/`TLM:<data>` line is
  // NEVER a COBS+CRC frame to dearmor -- it is always one of the cleartext
  // control tokens below. Bare `TLM` (no ':', no body -- same property the
  // no-':' branch above relies on) and `TLM:NOW` are both a telemetry
  // REQUEST (kFrame): since telemetry went silent at rest there was no way
  // to ask "what is your state right now?" and get the full frame back;
  // STATUS answers in cleartext but carries only a handful of flags, not
  // pose/encoders/sensors. `TLM:ON`/`TLM:AUTO`/`TLM:OFF` (case-insensitive)
  // stage a mode change; anything else after the ':' stages kUnrecognized.
  // Comms only STAGES the action and remembers which transport asked --
  // Telemetry never parses wire text, and Comms holds no Telemetry& to
  // apply the mode itself, so RobotLoop::cycle() is the one that consumes
  // takeTlmAction() and calls tlm_.setMode()/emit()/sendTlmReply().
  if (entry->verb == msg::Verb::TLM) {
    tlmReplyTransport_ = &t;
    tlmAction_ = (colonPos >= lineLen) ? TlmAction::kFrame : classifyTlmArg(dataPtr, dataLen);
    return;
  }

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
      // t=<ms> is the robot's own clock at reply-formatting time (117,
      // SUC-056) -- activates the host's ClockSync (min-RTT offset + skew
      // fit). Integer formatting only: newlib-nano has no printf float
      // support, but `now` is already an integer, so this is a non-issue,
      // not a workaround. `PONG:t=<ms>` replaces pre-124 "OK pong t=<ms>"
      // (issue §4 -- the reply plane adopts the SAME grammar; a reply
      // verb echoing its command verb is unambiguous because link
      // direction is known, exactly like HELLO -> DEVICE:).
      char pong[32];
      std::snprintf(pong, sizeof(pong), "PONG:t=%lu", static_cast<unsigned long>(now));
      t.sendReliable(pong);
      return;
    }
    case msg::Verb::ID:
      // idLine_ -- caller-owned, preformatted "ID:<fields>" (architecture
      // Decision 4: configured-robot identity -- see comms.h's own
      // constructor doc comment).
      t.sendReliable(idLine_);
      return;
    case msg::Verb::VER:
      // Reads the existing generated build-version constant directly
      // (architecture Decision 4: zero new version-tracking
      // infrastructure) -- adjacent string-literal concatenation, no
      // runtime formatting at all.
      t.sendReliable("VER:" FIRMWARE_VERSION_STR);
      return;
    case msg::Verb::STATUS:
      sendStatus(t);
      return;
    case msg::Verb::HELP:
      sendHelp(t);
      return;
    default:
      // Any other cleartext verb arriving INBOUND (e.g. a stray
      // DEVICE/PONG -- those are reply-plane-only) has no command
      // handler -- count it exactly like an unrecognized command.
      ++malformedCount_;
      return;
  }
}

void Comms::decodeBinaryFrame(const uint8_t* command, size_t commandLen, const uint8_t* data, uint16_t dataLen,
                               Cmd& out) {
  // Reverse of sendReply()'s CRC-then-COBS composition (123-001 completion
  // notes, pinned exactly so firmware and host agree byte-for-byte): 1)
  // COBS-decode (delimiter kCobsDelimiter, 124-005) the received frame
  // bytes back into (schema payload + CRC) combined bytes, 2) split off
  // the trailing 2-byte CRC, 3) verify it against the leading payload
  // bytes (CRC-scoped over `command` too, per 124-003/124-005 -- see
  // crcOverScope() above), 4) only then hand the payload to
  // msg::wire::decode(). Every step fails cleanly (malformedCount_++,
  // out left untouched) on any malformed/corrupt input -- no partial state
  // ever reaches `out` (see pump()'s own doc comment). `data`==nullptr/
  // dataLen==0 (a binary verb with no ':' or an empty body) fails cleanly
  // at cobsDecode()'s own "empty input" rejection -- no special case
  // needed here.
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

  // Decode into a LOCAL temporary; only publish into `out` on success --
  // see comms.h's pump() doc comment. Comms's dearmor path never replies
  // synchronously (no sendError()/per-command ERR reply): a malformed
  // frame is silently counted (malformedCount_) and surfaced as a
  // Telemetry fault bit (App::kFaultCommsMalformed) instead.
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
  if (name == nullptr) return;  // BodyKind::NONE -- nothing to send
  const size_t nameLen = std::strlen(name);

  uint8_t rawBuf[kMaxEnvelopeBytes];
  const uint16_t n = msg::wire::encode(reply, rawBuf, static_cast<uint16_t>(sizeof(rawBuf)));
  if (n == 0) {
    // Unreachable in practice: kMaxEnvelopeBytes is sized from the SAME
    // generated kCommandEnvelopeMaxEncodedSize/kReplyEnvelopeMaxEncodedSize
    // constants encode() itself is budgeted against.
    return;
  }

  // CRC-then-COBS composition (123-001 completion notes -- NOT
  // COBS-then-append-CRC, which would risk emitting a literal delimiter
  // byte if the CRC bytes happen to contain one, breaking the delimiter
  // property): append the 2-byte CRC-16/CCITT-FALSE (little-endian) to
  // the schema-encoded payload, THEN COBS-encode (kCobsDelimiter,
  // 124-005) the combined bytes. The CRC itself is scoped over `name`
  // too (124-003/124-005 -- crcOverScope() above), so the wire layout
  // here is unchanged: the CRC still covers only `rawBuf`/`combined`'s
  // own bytes on the wire, and `name` never enters the COBS input.
  uint8_t combined[kMaxCrcPayloadBytes];
  std::memcpy(combined, rawBuf, n);
  size_t combinedLen = n;
  const uint16_t crc = crcOverScope(reinterpret_cast<const uint8_t*>(name), nameLen, rawBuf, n);
  if (!WireRuntime::encodeCrc16(crc, combined, sizeof(combined), &combinedLen)) {
    return;  // unreachable in practice -- combined is sized for exactly this
  }

  uint8_t cobsOut[kFramedMaxBytes];
  size_t cobsLen = 0;
  if (!WireRuntime::cobsEncode(combined, combinedLen, cobsOut, sizeof(cobsOut), &cobsLen, kCobsDelimiter)) {
    return;  // unreachable in practice -- cobsOut is sized to the worst case
  }

  // Build the full wire LINE content, `<NAME>':'<COBS bytes>` (124-005,
  // issue §1/§4 -- the reply plane uses the SAME grammar as the command
  // plane) -- the transport appends only the trailing '\n' terminator
  // itself (Transport::send()'s own doc comment, comms.h).
  uint8_t line[kMaxLineBytes];
  if (nameLen + 1 + cobsLen > sizeof(line)) {
    return;  // unreachable in practice -- kMaxLineBytes covers the worst case
  }
  std::memcpy(line, name, nameLen);
  line[nameLen] = ':';
  std::memcpy(line + nameLen + 1, cobsOut, cobsLen);
  const uint16_t lineLen = static_cast<uint16_t>(nameLen + 1 + cobsLen);

  // Broadcast on BOTH transports every call, via the async/drop-on-full
  // send() path (never sendReliable()) -- telemetry is always-on and must
  // never stall the loop on backpressure (primary+secondary frames go out
  // on both transports every cadence). The concrete transport appends the
  // trailing '\n' terminator itself.
  serialLink_.send(line, lineLen);
  radioLink_.send(line, lineLen);
}

void Comms::sendBanner() {
  // sendReliable(), not send(): one-off cleartext line (com/DESIGN.md §3).
  serialLink_.sendReliable(banner_);
  radioLink_.sendReliable(banner_);
}

// sendReady -- the unsolicited "the loop will now accept commands" line.
//
// Emitted once, from RobotLoop::boot()'s tail. A host cannot infer this from
// PING: comms_.pump() runs inside boot()'s probe loop (123-006), so PONG
// answers throughout a boot window in which every Move is correctly rejected
// with ERR_NOT_CONFIGURED (rejectDuringBoot, 125-001). See commands.proto's
// READY row for the measurement that motivated this.
// sendStatus -- the queryable state line, "STATUS:k=v:k=v...".
//
// Answers from the snapshot RobotLoop refreshes each cycle, so it costs one
// struct read and a format. Works with telemetry silent and without a binary
// codec, which is the point: a human on a serial terminal can ask.
//
// Booleans are 1/0 rather than yes/no -- shorter on a size-constrained line,
// and unambiguous to both a person and a parser. `flags` is hex because it
// is a bit field and every reader of it wants bits, not a decimal.
void Comms::sendStatus(Transport& t) {
  // tlm= (Part 4): the current TlmMode, spelled the same way the `TLM:`
  // command surface itself is spelled -- lowercase, since every other
  // STATUS field is lowercase and this one is meant to be read by both a
  // human at a terminal and a host parser. status_.tlmMode is the raw
  // App::TlmMode enum value (0/1/2) RobotLoop copies in each cycle
  // (Comms::Status's own doc comment) -- never derived here.
  const char* tlmStr = "auto";
  switch (status_.tlmMode) {
    case 0: tlmStr = "off"; break;
    case 1: tlmStr = "auto"; break;
    case 2: tlmStr = "on"; break;
    default: break;  // unreachable: only 0/1/2 are ever written
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

// sendHelp -- the cleartext verbs this build actually answers.
//
// WALKS kVerbTable rather than carrying a literal list, so it reports what
// the dispatcher will really accept. A hand-written list is a second source
// of truth and would drift the first time someone adds a verb -- which is
// exactly how the old help text died.
//
// Binary verbs are skipped: they need a COBS+CRC frame and cannot be typed,
// so listing them would advertise commands a human cannot use -- EXCEPT
// TLM (Part 4, telemetry-emit-policy-rebuild-spec.md): the registry still
// marks it `binary` (a real TLM frame travels robot->host as one), but an
// INBOUND TLM line is always this ticket's cleartext control surface
// (dispatchLine()'s own doc comment on the TLM verb) -- a human CAN type
// it, so it is listed here too, with its own argument grammar in place of
// the bare verb name.
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

// sendTlmReply -- see comms.h's own doc comment for the full contract
// (call AFTER setStatus() has the NEW mode). No pending-transport guard
// needed beyond the null check: tlmReplyTransport_ is only ever non-null
// after dispatchLine() staged a real TlmAction on the same call chain that
// produced the `action` argument here.
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
      break;  // kFrame's reply is the forced telemetry frame emit() sends
  }
}

void Comms::sendReady() {
  // Same one-off cleartext path as the banner. No data field: the verb IS
  // the whole message, so it parses as a bare no-data line under the v5
  // grammar, exactly like HELLO/PING inbound.
  serialLink_.sendReliable("READY");
  radioLink_.sendReliable("READY");
}

}  // namespace App
