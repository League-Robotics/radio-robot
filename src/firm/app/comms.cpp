// comms.cpp -- App::Comms implementation. See comms.h's file header for
// the module's boundary.
#include "app/comms.h"

#include <cstdio>
#include <cstring>

#include "messages/wire_runtime.h"

#ifndef HOST_BUILD
#include "com/radio.h"
#include "com/serial_port.h"
#endif

namespace App {

#ifndef HOST_BUILD

// --- SerialTransport ---------------------------------------------------

namespace {

// Maps a concrete transport's OWN frame-kind enum onto App::FrameKind --
// com/ has no dependency on app/ (com/DESIGN.md), so SerialPort/Radio each
// declare their own identical-shaped enum; this is the one seam (comms.cpp,
// which already depends on both) allowed to know both exist.
FrameKind toAppFrameKind(SerialPort::FrameKind k) {
  switch (k) {
    case SerialPort::FrameKind::kText: return FrameKind::kText;
    case SerialPort::FrameKind::kBinary: return FrameKind::kBinary;
    default: return FrameKind::kNone;
  }
}

FrameKind toAppFrameKind(Radio::FrameKind k) {
  switch (k) {
    case Radio::FrameKind::kText: return FrameKind::kText;
    case Radio::FrameKind::kBinary: return FrameKind::kBinary;
    default: return FrameKind::kNone;
  }
}

}  // namespace

SerialTransport::SerialTransport(SerialPort& serial) : serial_(serial) {}

FrameKind SerialTransport::readLine(char* buf, uint16_t cap, uint16_t* outLen) {
  return toAppFrameKind(serial_.readLine(buf, cap, outLen));
}

void SerialTransport::send(const uint8_t* data, uint16_t len) { serial_.send(data, len); }

void SerialTransport::sendReliable(const char* msg) { serial_.sendReliable(msg); }

// --- RadioTransport ------------------------------------------------------

RadioTransport::RadioTransport(Radio& radio) : radio_(radio) {}

FrameKind RadioTransport::readLine(char* buf, uint16_t cap, uint16_t* outLen) {
  return toAppFrameKind(radio_.poll(buf, cap, outLen));
}

void RadioTransport::send(const uint8_t* data, uint16_t len) { radio_.send(data, len); }

void RadioTransport::sendReliable(const char* msg) { radio_.sendText(msg); }

#endif  // HOST_BUILD

namespace {

// crcOverScope() -- the CRC-scope composition protocol v5 needs (124-003,
// issue §3): `crc16(COMMAND ':' payload)` when a command name is given,
// `crc16(payload)` alone (byte-identical to protocol v4's CRC) when it is
// not. Threads WireRuntime's incremental crcInit()/crcUpdate() so the
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

}  // namespace

// --- Comms -----------------------------------------------------------

Comms::Comms(Transport& serialLink, Transport& radioLink, const char* banner)
    : serialLink_(serialLink), radioLink_(radioLink), banner_(banner) {}

void Comms::pump(Cmd& out, uint32_t now) {
  out.status = CmdStatus::kNone;
  if (pumpTransport(serialLink_, out, now)) return;
  pumpTransport(radioLink_, out, now);
}

bool Comms::pumpTransport(Transport& t, Cmd& out, uint32_t now) {
  char line[kArmoredBufSize];
  uint16_t lineLen = 0;
  const FrameKind kind = t.readLine(line, sizeof(line), &lineLen);
  if (kind == FrameKind::kNone) return false;

  if (kind == FrameKind::kText) {
    // Text plane -- HELLO/PING replies. Every other text-plane line
    // (not "HELLO", not "PING") is unrecognized, not merely un-armored --
    // there is no more "*B..." prefix check now that armor is binary, so
    // ANY text line that isn't one of the two recognized commands counts
    // as malformed.
    if (std::strcmp(line, "HELLO") == 0) {
      t.sendReliable(banner_);
      return true;
    }
    if (std::strcmp(line, "PING") == 0) {
      // t=<ms> is the robot's own clock at reply-formatting time (117,
      // SUC-056) -- activates the host's ClockSync (min-RTT offset + skew
      // fit). Integer formatting only: newlib-nano has no printf float
      // support, but `now` is already an integer, so this is a non-issue,
      // not a workaround. Buffer sized generously for "OK pong t=" (10) +
      // a uint32_t's worst case (10 digits) + NUL.
      char pong[32];
      std::snprintf(pong, sizeof(pong), "OK pong t=%lu", static_cast<unsigned long>(now));
      t.sendReliable(pong);
      return true;
    }
    ++malformedCount_;
    return true;
  }

  // kind == FrameKind::kBinary
  decodeBinaryFrame(reinterpret_cast<const uint8_t*>(line), lineLen, out);
  return true;
}

void Comms::decodeBinaryFrame(const uint8_t* frame, uint16_t frameLen, Cmd& out, const uint8_t* command,
                               size_t commandLen) {
  // Reverse of sendReply()'s CRC-then-COBS composition (123-001 completion
  // notes, pinned exactly so firmware and host agree byte-for-byte): 1)
  // COBS-decode the received frame bytes back into (schema payload + CRC)
  // combined bytes, 2) split off the trailing 2-byte CRC, 3) verify it
  // against the leading payload bytes (CRC-scoped over `command` too, per
  // 124-003 -- see crcOverScope() above), 4) only then hand the payload to
  // msg::wire::decode(). Every step fails cleanly (malformedCount_++,
  // out left untouched) on any malformed/corrupt input -- no partial state
  // ever reaches `out` (see pump()'s own doc comment).
  uint8_t combined[kMaxCrcPayloadBytes];
  size_t combinedLen = 0;
  if (!WireRuntime::cobsDecode(frame, frameLen, combined, sizeof(combined), &combinedLen)) {
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

void Comms::sendReply(const msg::ReplyEnvelope& reply, const uint8_t* command, size_t commandLen) {
  uint8_t rawBuf[kMaxEnvelopeBytes];
  const uint16_t n = msg::wire::encode(reply, rawBuf, static_cast<uint16_t>(sizeof(rawBuf)));
  if (n == 0) {
    // Unreachable in practice: kMaxEnvelopeBytes is sized from the SAME
    // generated kCommandEnvelopeMaxEncodedSize/kReplyEnvelopeMaxEncodedSize
    // constants encode() itself is budgeted against.
    return;
  }

  // CRC-then-COBS composition (123-001 completion notes -- NOT
  // COBS-then-append-CRC, which would risk emitting a literal 0x00 if the
  // CRC bytes happen to contain one, breaking the delimiter property):
  // append the 2-byte CRC-16/CCITT-FALSE (little-endian) to the
  // schema-encoded payload, THEN COBS-encode the combined bytes. The CRC
  // itself is scoped over `command` too when one is given (124-003 --
  // crcOverScope() above), so the wire layout here is unchanged: the CRC
  // still covers only `rawBuf`/`combined`'s own bytes on the wire, and
  // `command`/`commandLen` never enter the COBS input.
  uint8_t combined[kMaxCrcPayloadBytes];
  std::memcpy(combined, rawBuf, n);
  size_t combinedLen = n;
  const uint16_t crc = crcOverScope(command, commandLen, rawBuf, n);
  if (!WireRuntime::encodeCrc16(crc, combined, sizeof(combined), &combinedLen)) {
    return;  // unreachable in practice -- combined is sized for exactly this
  }

  uint8_t framed[kFramedMaxBytes];
  size_t framedLen = 0;
  if (!WireRuntime::cobsEncode(combined, combinedLen, framed, sizeof(framed), &framedLen)) {
    return;  // unreachable in practice -- framed is sized to the worst case
  }

  // Broadcast on BOTH transports every call, via the async/drop-on-full
  // send() path (never sendReliable()) -- telemetry is always-on and must
  // never stall the loop on backpressure (primary+secondary frames go out
  // on both transports every cadence). The concrete transport appends the
  // trailing 0x00 delimiter itself.
  serialLink_.send(framed, static_cast<uint16_t>(framedLen));
  radioLink_.send(framed, static_cast<uint16_t>(framedLen));
}

void Comms::sendBanner() {
  // sendReliable(), not send(): one-off text-plane line (com/DESIGN.md §3).
  serialLink_.sendReliable(banner_);
  radioLink_.sendReliable(banner_);
}

}  // namespace App
