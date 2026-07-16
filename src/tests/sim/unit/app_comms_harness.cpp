// app_comms_harness.cpp -- off-hardware acceptance harness for ticket
// 103-004 (SUC-004), App::Comms (src/firm/app/comms.{h,cpp}). Proves the
// "*B" armor/dearmor sequence transcribed from the deleted
// src/firm/commands/binary_channel.cpp (sprint 102's transcription note,
// clasi/sprints/done/102-single-loop-firmware-spikes-archive-and-delete-
// to-stub-p0-p2/notes/armor-wire-codec-transcription.md) round-trips a
// hand-built CommandEnvelope, rejects malformed input cleanly (no crash,
// no partial state), reproduces the HELLO/PING text-plane exception
// byte-identically to today's main.cpp stub, bounds pump() to at most one
// consumed line per call, and that sendReply() broadcasts an identical
// armored line on both transports.
//
// Mirrors wire_codec_harness.cpp's exact shape: hand-rolled
// beginScenario/fail/checkTrue/checkFalse/checkU64Eq assertion plumbing,
// PASS/FAIL printf, exit nonzero on failure. Compiled by test_app_comms.py
// with -DHOST_BUILD against comms.cpp, wire.cpp, wire_runtime.cpp (no
// MicroBit.h anywhere in this graph under HOST_BUILD -- comms.h/comms.cpp's
// SerialTransport/RadioTransport adapters are compiled out).
//
// FakeTransport (App::Transport) is TestSupport::FakeTransport
// (src/tests/sim/support/fake_transport.h, ticket 105-002) -- the ONE canonical
// scripted queue of inbound lines plus a log of every send()/sendReliable()
// call, so scenarios can feed input and assert exactly what got sent. This
// harness previously carried its own ad hoc copy; it now builds on the
// shared primitive (105-002's dedup mandate) with no scenario changes.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "app/comms.h"
#include "messages/envelope.h"
#include "messages/wire.h"
#include "messages/wire_runtime.h"
#include "support/fake_transport.h"

namespace {

using WireRuntime::WireType;

// --- Hand-rolled assertion plumbing (same tiny shape as
// wire_codec_harness.cpp). ---

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

void checkU64Eq(uint64_t actual, uint64_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %llu, got %llu", what.c_str(),
                  static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
    fail(buf);
  }
}

void checkFloatEq(float actual, float expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(), static_cast<double>(expected),
                  static_cast<double>(actual));
    fail(buf);
  }
}

void checkStrEq(const std::string& actual, const std::string& expected, const std::string& what) {
  if (actual != expected) {
    fail(what + " -- expected \"" + expected + "\", got \"" + actual + "\"");
  }
}

// --- Hand-rolled wire-byte builder (same as wire_codec_harness.cpp's own
// Buf/putVarintField/putFloatField/putMessageField -- composed with
// WireRuntime's own primitives, the SAME primitives wire.cpp's generated
// decode()/encode() are built on) -- lets this harness construct arbitrary
// CommandEnvelope payloads without depending on a generic encode(). ---

struct Buf {
  uint8_t data[256] = {};
  size_t len = 0;
};

bool putVarintField(Buf& b, uint32_t number, uint64_t v) {
  return WireRuntime::encodeTag(number, WireType::kVarint, b.data, sizeof(b.data), &b.len) &&
         WireRuntime::encodeVarint(v, b.data, sizeof(b.data), &b.len);
}

bool putFloatField(Buf& b, uint32_t number, float v) {
  return WireRuntime::encodeTag(number, WireType::kFixed32, b.data, sizeof(b.data), &b.len) &&
         WireRuntime::encodeFloat(v, b.data, sizeof(b.data), &b.len);
}

bool putBytesField(Buf& b, uint32_t number, const uint8_t* payload, size_t payloadLen) {
  if (!WireRuntime::encodeTag(number, WireType::kLengthDelimited, b.data, sizeof(b.data), &b.len)) return false;
  if (!WireRuntime::encodeVarint(payloadLen, b.data, sizeof(b.data), &b.len)) return false;
  if (b.len + payloadLen > sizeof(b.data)) return false;
  std::memcpy(b.data + b.len, payload, payloadLen);
  b.len += payloadLen;
  return true;
}

bool putMessageField(Buf& b, uint32_t number, const Buf& nested) {
  return putBytesField(b, number, nested.data, nested.len);
}

// Armor a raw CommandEnvelope/ReplyEnvelope byte buffer into a "*B<base64>"
// line -- the SAME sequence comms.cpp's own sendReply() performs, used here
// only to construct scenario INPUT (pump()'s inbound side), independent of
// Comms's own outbound path so the two directions are tested against each
// other, not tautologically.
std::string armor(const uint8_t* raw, size_t rawLen) {
  char b64[512] = {};
  size_t b64Len = 0;
  bool ok = WireRuntime::base64Encode(raw, rawLen, b64, sizeof(b64), &b64Len);
  if (!ok) return std::string();
  std::string out = "*B";
  out.append(b64, b64Len);
  return out;
}

using TestSupport::FakeTransport;

// ===========================================================================
// 1. Twist round-trip: hand-build a CommandEnvelope, armor it, feed it
//    through a FakeTransport's queued line, confirm pump() decodes it.
// ===========================================================================

void scenarioTwistRoundTrip() {
  beginScenario("pump(): armored twist CommandEnvelope decodes correctly");

  Buf twist;
  putFloatField(twist, 1, 150.0f);  // v_x
  putFloatField(twist, 2, -0.75f);  // omega
  putFloatField(twist, 3, 250.0f);  // duration
  Buf env;
  putVarintField(env, 1, 7);        // corr_id
  putMessageField(env, 19, twist);  // CommandEnvelope.cmd.twist, field 19

  std::string line = armor(env.data, env.len);
  checkTrue(!line.empty(), "armor() produced a non-empty line");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound(line.c_str());

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd);

  checkTrue(cmd.status == App::CmdStatus::kDecoded, "cmd.status == kDecoded");
  checkU64Eq(cmd.env.corr_id, 7, "corr_id round-trips");
  checkTrue(cmd.env.cmd_kind == msg::CommandEnvelope::CmdKind::TWIST, "cmd_kind == TWIST");
  checkFloatEq(cmd.env.cmd.twist.v_x, 150.0f, "twist.v_x round-trips");
  checkFloatEq(cmd.env.cmd.twist.omega, -0.75f, "twist.omega round-trips");
  checkFloatEq(cmd.env.cmd.twist.duration, 250.0f, "twist.duration round-trips");
  checkU64Eq(comms.malformedCount(), 0, "malformedCount stays 0 for a well-formed frame");
}

// ===========================================================================
// 2. Malformed input -- bad armor prefix, truncated base64, corrupt-but-
//    valid-base64 protobuf bytes -- each rejected cleanly: no crash,
//    out.status stays kNone, malformedCount() increments.
// ===========================================================================

void scenarioMalformedArmorPrefixRejected() {
  beginScenario("pump(): line[1] != 'B' rejected -- malformedCount increments, out untouched");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("*Xsomeunrecognizedarmor");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd);

  checkTrue(cmd.status == App::CmdStatus::kNone, "cmd.status stays kNone");
  checkU64Eq(comms.malformedCount(), 1, "malformedCount increments exactly once");
}

void scenarioMalformedTruncatedBase64Rejected() {
  beginScenario("pump(): truncated base64 (not a multiple of 4) rejected -- malformedCount increments");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("*BQQ");  // "QQ" -- 2 chars, not a multiple of 4

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd);

  checkTrue(cmd.status == App::CmdStatus::kNone, "cmd.status stays kNone");
  checkU64Eq(comms.malformedCount(), 1, "malformedCount increments exactly once");
}

void scenarioMalformedCorruptProtobufRejected() {
  beginScenario(
      "pump(): well-formed base64 but corrupt protobuf bytes (truncated wire tag) rejected -- malformedCount "
      "increments");

  // Mirrors wire_codec_harness.cpp's scenarioMalformedBufferRejected: a
  // well-formed corr_id field followed by a truncated varint continuation
  // byte with no following byte.
  Buf env;
  putVarintField(env, 1, 3);
  env.data[env.len++] = 0x80;

  std::string line = armor(env.data, env.len);
  checkTrue(!line.empty(), "armor() produced a non-empty line for the corrupt payload");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound(line.c_str());

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd);

  checkTrue(cmd.status == App::CmdStatus::kNone, "cmd.status stays kNone");
  checkU64Eq(comms.malformedCount(), 1, "malformedCount increments exactly once");
}

// ===========================================================================
// 3. Text plane -- HELLO -> banner via sendReliable(); PING -> "OK pong"
//    via sendReliable(). Byte-identical to today's main.cpp stub.
// ===========================================================================

void scenarioHelloRepliesWithBannerViaSendReliable() {
  beginScenario("pump(): HELLO replies with the banner via sendReliable()");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("HELLO");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd);

  checkTrue(cmd.status == App::CmdStatus::kNone, "HELLO never decodes a Cmd");
  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one sendReliable() call");
  if (!serialFake.sentReliable().empty()) {
    checkStrEq(serialFake.sentReliable()[0], banner, "sendReliable() carried the banner verbatim");
  }
  checkU64Eq(serialFake.sent().size(), 0, "no send() (async) call for a text-plane reply");
  checkU64Eq(comms.malformedCount(), 0, "HELLO does not count as malformed");
}

void scenarioPingRepliesOkPongViaSendReliable() {
  beginScenario("pump(): PING replies \"OK pong\" via sendReliable()");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("PING");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd);

  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one sendReliable() call");
  if (!serialFake.sentReliable().empty()) {
    checkStrEq(serialFake.sentReliable()[0], "OK pong", "sendReliable() carried \"OK pong\"");
  }
}

// ===========================================================================
// 4. pump() bounded to one line per call -- even when BOTH FakeTransports
//    have a line queued, only one is drained this call.
// ===========================================================================

void scenarioPumpBoundedToOneTransportPerCall() {
  beginScenario("pump(): drains at most one transport's line per call (serial checked first)");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("PING");
  radioFake.enqueueInbound("PING");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd);

  checkU64Eq(serialFake.inboundSize(), 0, "serial's queued line was drained this call");
  checkU64Eq(radioFake.inboundSize(), 1, "radio's queued line was NOT touched this call (serial had one)");
  checkU64Eq(serialFake.sentReliable().size(), 1, "serial received the PING reply");
  checkU64Eq(radioFake.sentReliable().size(), 0, "radio received no reply (never polled this call)");

  // A second pump() call now drains radio's queued line.
  App::Cmd cmd2;
  comms.pump(cmd2);
  checkU64Eq(radioFake.inboundSize(), 0, "radio's queued line is drained on the NEXT call");
  checkU64Eq(radioFake.sentReliable().size(), 1, "radio received the PING reply on the second call");
}

// ===========================================================================
// 5. sendReply() round-trip: both transports' send() logs capture the exact
//    same "*B<base64>" line, matching an independent re-encode of the same
//    ReplyEnvelope.
// ===========================================================================

void scenarioSendReplyBroadcastsIdenticalLineOnBothTransports() {
  beginScenario("sendReply(): identical \"*B<base64>\" line sent on both transports via send() (not sendReliable())");

  msg::ReplyEnvelope reply;
  reply.corr_id = 9;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::OK;
  reply.body.ok.q = 5;
  reply.body.ok.rem = 12.5f;
  reply.body.ok.t = 4242;

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  comms.sendReply(reply);

  checkU64Eq(serialFake.sent().size(), 1, "exactly one serial send() call");
  checkU64Eq(radioFake.sent().size(), 1, "exactly one radio send() call");
  checkU64Eq(serialFake.sentReliable().size(), 0, "sendReply() never uses sendReliable()");
  checkU64Eq(radioFake.sentReliable().size(), 0, "sendReply() never uses sendReliable()");

  if (!serialFake.sent().empty() && !radioFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], radioFake.sent()[0],
               "serial and radio received byte-identical armored lines");
  }

  // Independent re-encode, without going through Comms::sendReply() at
  // all -- proves the line is exactly what encode()+base64Encode() would
  // produce (round-trip proof without needing a generic ReplyEnvelope
  // decoder, per the ticket's own testing plan).
  uint8_t rawBuf[App::kMaxEnvelopeBytes];
  uint16_t n = msg::wire::encode(reply, rawBuf, sizeof(rawBuf));
  checkTrue(n > 0, "independent encode() succeeds");
  std::string expected = armor(rawBuf, n);
  checkTrue(!expected.empty(), "independent armor() succeeds");

  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expected, "sendReply()'s line matches an independent re-encode+armor");
  }
}

}  // namespace

int main() {
  scenarioTwistRoundTrip();
  scenarioMalformedArmorPrefixRejected();
  scenarioMalformedTruncatedBase64Rejected();
  scenarioMalformedCorruptProtobufRejected();
  scenarioHelloRepliesWithBannerViaSendReliable();
  scenarioPingRepliesOkPongViaSendReliable();
  scenarioPumpBoundedToOneTransportPerCall();
  scenarioSendReplyBroadcastsIdenticalLineOnBothTransports();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Comms scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Comms scenarios\n", g_failureCount);
  return 1;
}
