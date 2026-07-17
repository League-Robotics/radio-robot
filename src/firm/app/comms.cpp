// comms.cpp -- App::Comms implementation. See comms.h's file header for
// the module's boundary.
#include "app/comms.h"

#include <cstring>

#include "messages/wire_runtime.h"

#ifndef HOST_BUILD
#include "com/radio.h"
#include "com/serial_port.h"
#endif

namespace App {

#ifndef HOST_BUILD

// --- SerialTransport ---------------------------------------------------

SerialTransport::SerialTransport(SerialPort& serial) : serial_(serial) {}

bool SerialTransport::readLine(char* buf, uint16_t len) { return serial_.readLine(buf, len); }

void SerialTransport::send(const char* msg) { serial_.send(msg); }

void SerialTransport::sendReliable(const char* msg) { serial_.sendReliable(msg); }

// --- RadioTransport ------------------------------------------------------

RadioTransport::RadioTransport(Radio& radio) : radio_(radio) {}

bool RadioTransport::readLine(char* buf, uint16_t len) { return radio_.poll(buf, len); }

void RadioTransport::send(const char* msg) { radio_.send(msg); }

void RadioTransport::sendReliable(const char* msg) {
  // Radio has only one send path -- both Transport::send() and
  // sendReliable() delegate to the same Radio::send().
  radio_.send(msg);
}

#endif  // HOST_BUILD

// --- Comms -----------------------------------------------------------

Comms::Comms(Transport& serialLink, Transport& radioLink, const char* banner)
    : serialLink_(serialLink), radioLink_(radioLink), banner_(banner) {}

void Comms::pump(Cmd& out) {
  out.status = CmdStatus::kNone;
  if (pumpTransport(serialLink_, out)) return;
  pumpTransport(radioLink_, out);
}

bool Comms::pumpTransport(Transport& t, Cmd& out) {
  char line[kArmoredBufSize];
  if (!t.readLine(line, sizeof(line))) return false;

  // Text plane, checked BEFORE the '*' armor check -- HELLO/PING replies.
  if (std::strcmp(line, "HELLO") == 0) {
    t.sendReliable(banner_);
    return true;
  }
  if (std::strcmp(line, "PING") == 0) {
    t.sendReliable("OK pong");
    return true;
  }
  if (line[0] != '*') {
    // Not HELLO, not PING, not armored -- unrecognized text-plane line.
    ++malformedCount_;
    return true;
  }

  decodeArmoredLine(line, out);
  return true;
}

void Comms::decodeArmoredLine(const char* line, Cmd& out) {
  // Caller guarantees line[0] == '*'; line[1] != 'B' is still a real
  // possibility (a malformed/future-armor line) and must be rejected
  // cleanly, not assumed away.
  if (line[1] != 'B') {
    ++malformedCount_;
    return;
  }

  const char* b64 = line + 2;
  size_t b64Len = std::strlen(b64);
  while (b64Len > 0 && (b64[b64Len - 1] == '\r' || b64[b64Len - 1] == '\n' ||
                        b64[b64Len - 1] == ' ' || b64[b64Len - 1] == '\t')) {
    --b64Len;
  }

  uint8_t rawBuf[kMaxEnvelopeBytes];
  size_t rawLen = 0;
  if (!WireRuntime::base64Decode(b64, b64Len, rawBuf, sizeof(rawBuf), &rawLen)) {
    ++malformedCount_;
    return;
  }

  // Decode into a LOCAL temporary; only publish into `out` on success --
  // see comms.h's pump() doc comment. Comms's dearmor path never replies
  // synchronously (no sendError()/per-command ERR reply): a malformed
  // frame is silently counted (malformedCount_) and surfaced as a
  // Telemetry fault bit (App::kFaultCommsMalformed) instead.
  msg::CommandEnvelope decoded;
  const msg::wire::Result r = msg::wire::decode(decoded, rawBuf, static_cast<uint16_t>(rawLen));
  if (!r.ok) {
    ++malformedCount_;
    return;
  }

  out.status = CmdStatus::kDecoded;
  out.env = decoded;
}

void Comms::sendReply(const msg::ReplyEnvelope& reply) {
  uint8_t rawBuf[kMaxEnvelopeBytes];
  const uint16_t n = msg::wire::encode(reply, rawBuf, static_cast<uint16_t>(sizeof(rawBuf)));
  if (n == 0) {
    // Unreachable in practice: kMaxEnvelopeBytes is sized from the SAME
    // generated kCommandEnvelopeMaxEncodedSize/kReplyEnvelopeMaxEncodedSize
    // constants encode() itself is budgeted against.
    return;
  }

  char armored[kArmoredBufSize];
  armored[0] = '*';
  armored[1] = 'B';
  size_t b64Len = 0;
  if (!WireRuntime::base64Encode(rawBuf, n, armored + 2, sizeof(armored) - 3, &b64Len)) {
    return;  // same unreachable-in-practice sizing argument as above
  }
  armored[2 + b64Len] = '\0';

  // Broadcast on BOTH transports every call, via the async/drop-on-full
  // send() path (never sendReliable()) -- telemetry is always-on and must
  // never stall the loop on backpressure (primary+secondary frames go out
  // on both transports every cadence).
  serialLink_.send(armored);
  radioLink_.send(armored);
}

}  // namespace App
