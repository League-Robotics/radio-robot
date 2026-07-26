// app_comms_harness.cpp -- off-hardware acceptance harness for ticket
// 103-004 (SUC-004), App::Comms (src/firm/app/comms.{h,cpp}). Proves the
// CRC-then-COBS binary armor/dearmor sequence (123-002 -- supersedes the
// "*B"+base64 armor transcribed from the deleted
// src/firm/commands/binary_channel.cpp, sprint 102's transcription note,
// clasi/sprints/done/102-single-loop-firmware-spikes-archive-and-delete-
// to-stub-p0-p2/notes/armor-wire-codec-transcription.md) round-trips a
// hand-built CommandEnvelope, rejects malformed input cleanly (no crash,
// no partial state -- including a genuine CRC-mismatch fault injection,
// SUC-002), reproduces the HELLO/PING text-plane exception (demuxed from
// the binary frames on the SAME byte stream) byte-identically to today's
// main.cpp stub, bounds pump() to at most one consumed frame per call, and
// that sendReply() broadcasts an identical framed body on both transports.
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

// armor() -- 124-005 (protocol v5 Part A, "framing grammar cutover"): builds
// the COMPLETE wire LINE, `<command>':'<COBS+CRC bytes>` (CRC-then-COBS,
// delimiter 0x0A) -- the SAME composition comms.cpp's own
// decodeBinaryFrame()/sendReply() perform, used here only to construct
// scenario INPUT (pump()'s inbound side), independent of Comms's own
// outbound path so the two directions are tested against each other, not
// tautologically. `command` is REQUIRED -- protocol v5 has no unscoped
// binary frame any more, every dispatched command needs a real registry
// verb name (messages/commands.h). The trailing '\n' terminator is a
// transport concern, not included here -- push the result via
// FakeTransport::enqueueInboundBinary() (an alias of enqueueInbound()
// since 124-005 -- see that class's own doc comment).
std::string armor(const uint8_t* raw, size_t rawLen, const char* command) {
  uint8_t combined[256];
  if (rawLen > sizeof(combined) - 2) return std::string();
  std::memcpy(combined, raw, rawLen);
  size_t combinedLen = rawLen;
  const size_t commandLen = std::strlen(command);
  uint16_t crc = WireRuntime::crcInit();
  crc = WireRuntime::crcUpdate(crc, reinterpret_cast<const uint8_t*>(command), commandLen);
  const uint8_t sep = ':';
  crc = WireRuntime::crcUpdate(crc, &sep, 1);
  crc = WireRuntime::crcUpdate(crc, raw, rawLen);
  if (!WireRuntime::encodeCrc16(crc, combined, sizeof(combined), &combinedLen)) return std::string();
  uint8_t framed[300];
  size_t framedLen = 0;
  if (!WireRuntime::cobsEncode(combined, combinedLen, framed, sizeof(framed), &framedLen, /*delimiter=*/0x0A)) {
    return std::string();
  }
  std::string line(command, commandLen);
  line += ':';
  line.append(reinterpret_cast<const char*>(framed), framedLen);
  return line;
}

using TestSupport::FakeTransport;

// ===========================================================================
// 1. Move round-trip (116-006, MOVE protocol cutover): hand-build a
//    CommandEnvelope{move: Move{twist, stop=time, timeout, replace, id}},
//    armor it, feed it through a FakeTransport's queued line, confirm
//    pump() decodes it. Supersedes the deleted scenarioTwistRoundTrip()
//    (arm 19, Twist, is reserved -- 116-001).
// ===========================================================================

void scenarioMoveRoundTrip() {
  beginScenario("pump(): armored MOVE CommandEnvelope decodes correctly");

  Buf moveTwist;
  putFloatField(moveTwist, 1, 150.0f);  // MoveTwist.v_x
  putFloatField(moveTwist, 3, -0.75f);  // MoveTwist.omega
  Buf move;
  putMessageField(move, 1, moveTwist);  // Move.velocity.twist, field 1
  putFloatField(move, 3, 250.0f);       // Move.stop.time, field 3
  putFloatField(move, 6, 5000.0f);      // Move.timeout, field 6
  putVarintField(move, 7, 1);           // Move.replace = true, field 7
  putVarintField(move, 8, 42);          // Move.id, field 8
  Buf env;
  putVarintField(env, 1, 7);       // corr_id
  putMessageField(env, 21, move);  // CommandEnvelope.cmd.move, field 21

  std::string line = armor(env.data, env.len, "MOVE");
  checkTrue(!line.empty(), "armor() produced a non-empty frame");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInboundBinary(line);

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kDecoded, "cmd.status == kDecoded");
  checkU64Eq(cmd.env.corr_id, 7, "corr_id round-trips");
  checkTrue(cmd.env.cmd_kind == msg::CommandEnvelope::CmdKind::MOVE, "cmd_kind == MOVE");
  checkTrue(cmd.env.cmd.move.velocity_kind == msg::Move::VelocityKind::TWIST,
            "move.velocity_kind == TWIST");
  checkFloatEq(cmd.env.cmd.move.velocity.twist.v_x, 150.0f, "move.twist.v_x round-trips");
  checkFloatEq(cmd.env.cmd.move.velocity.twist.omega, -0.75f, "move.twist.omega round-trips");
  checkTrue(cmd.env.cmd.move.stop_kind == msg::Move::StopKind::TIME, "move.stop_kind == TIME");
  checkFloatEq(cmd.env.cmd.move.stop.time, 250.0f, "move.stop.time round-trips");
  checkFloatEq(cmd.env.cmd.move.timeout, 5000.0f, "move.timeout round-trips");
  checkTrue(cmd.env.cmd.move.replace, "move.replace round-trips true");
  checkU64Eq(cmd.env.cmd.move.id, 42, "move.id round-trips");
  checkU64Eq(comms.malformedCount(), 0, "malformedCount stays 0 for a well-formed frame");
}

// ===========================================================================
// 2. Malformed input -- 123-002: an unrecognized text-plane line (there is
//    no more "*B" armor-prefix check -- armor is binary now), a malformed
//    COBS frame, a CRC mismatch (SUC-002's own fault-injection acceptance
//    criterion -- a corrupted frame is DETECTED, not silently mis-parsed),
//    and corrupt-but-well-framed protobuf bytes -- each rejected cleanly:
//    no crash, out.status stays kNone, malformedCount() increments.
// ===========================================================================

void scenarioMalformedUnrecognizedTextLineRejected() {
  beginScenario("pump(): unrecognized text-plane line (not HELLO/PING) rejected -- malformedCount increments");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("*Xsomeunrecognizedarmor");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kNone, "cmd.status stays kNone");
  checkU64Eq(comms.malformedCount(), 1, "malformedCount increments exactly once");
}

void scenarioMalformedCobsFrameRejected() {
  beginScenario("pump(): malformed COBS frame (truncated code byte) rejected -- malformedCount increments");

  // A code byte claiming a 10-byte block when only 2 bytes remain --
  // WireRuntime::cobsDecode()'s own "code claims more data bytes than
  // remain" malformed-input case (wire_runtime.cpp). 124-005: needs a
  // REAL registered command prefix to reach decodeBinaryFrame() at all
  // (an unprefixed/unrecognized line is now rejected one step earlier,
  // at the registry lookup -- scenarioMalformedUnrecognizedTextLineRejected
  // above already covers that path).
  const uint8_t badCobsBody[] = {0x0B, 0x01, 0x02};
  std::string badFrame = std::string("MOVE:") +
                          std::string(reinterpret_cast<const char*>(badCobsBody), sizeof(badCobsBody));

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInboundBinary(badFrame);

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kNone, "cmd.status stays kNone");
  checkU64Eq(comms.malformedCount(), 1, "malformedCount increments exactly once");
}

void scenarioMalformedCrcMismatchRejected() {
  beginScenario(
      "pump(): well-framed COBS but CRC mismatch (bit-flipped payload) rejected -- SUC-002 fault-injection, "
      "malformedCount increments");

  // A well-formed CommandEnvelope{corr_id=7, stop}, correctly CRC-then-COBS
  // framed by armor()'s own reverse-of-decodeBinaryFrame() composition,
  // then a SINGLE BIT FLIPPED in the framed bytes after the fact -- proves
  // the CRC (not just the COBS framing) is actually checked: the frame
  // still COBS-decodes cleanly (the flipped bit doesn't touch a code byte),
  // but the recomputed CRC no longer matches the trailing 2 CRC bytes.
  Buf env;
  putVarintField(env, 1, 7);  // corr_id
  putVarintField(env, 13, 0);  // CommandEnvelope.stop (empty message, field 13)

  std::string line = armor(env.data, env.len, "STOP");
  checkTrue(!line.empty(), "armor() produced a non-empty frame");
  checkTrue(line.size() > 2, "frame has enough bytes to flip one safely");
  // Flip a bit in a byte comfortably inside the frame body (not the first
  // COBS code byte, which -- for this short a payload -- would desync
  // framing entirely rather than exercise the CRC check specifically).
  line[line.size() - 2] ^= 0x01;

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInboundBinary(line);

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kNone, "cmd.status stays kNone -- corrupted frame never decodes");
  checkU64Eq(comms.malformedCount(), 1, "malformedCount increments exactly once (CRC mismatch detected, not mis-parsed)");
}

void scenarioMalformedCorruptProtobufRejected() {
  beginScenario(
      "pump(): well-framed COBS+CRC but corrupt protobuf bytes (truncated wire tag) rejected -- malformedCount "
      "increments");

  // Mirrors wire_codec_harness.cpp's scenarioMalformedBufferRejected: a
  // well-formed corr_id field followed by a truncated varint continuation
  // byte with no following byte. armor() computes a CORRECT CRC over these
  // (already-corrupt-at-the-protobuf-level) bytes, so this scenario
  // specifically exercises msg::wire::decode()'s own rejection -- NOT the
  // CRC check (that is scenarioMalformedCrcMismatchRejected() above).
  Buf env;
  putVarintField(env, 1, 3);
  env.data[env.len++] = 0x80;

  std::string line = armor(env.data, env.len, "MOVE");
  checkTrue(!line.empty(), "armor() produced a non-empty frame for the corrupt payload");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInboundBinary(line);

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kNone, "cmd.status stays kNone");
  checkU64Eq(comms.malformedCount(), 1, "malformedCount increments exactly once");
}

// ===========================================================================
// 3. Cleartext plane -- HELLO -> banner via sendReliable(); PING ->
//    "PONG:t=<ms>" via sendReliable() (124-005, issue §4 -- replaces
//    pre-124 "OK pong t=<ms>", the one behavioural break the reply-plane
//    grammar cutover makes).
// ===========================================================================

void scenarioHelloRepliesWithBannerViaSendReliable() {
  beginScenario("pump(): HELLO replies with the banner via sendReliable()");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("HELLO");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kNone, "HELLO never decodes a Cmd");
  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one sendReliable() call");
  if (!serialFake.sentReliable().empty()) {
    checkStrEq(serialFake.sentReliable()[0], banner, "sendReliable() carried the banner verbatim");
  }
  checkU64Eq(serialFake.sent().size(), 0, "no send() (async) call for a text-plane reply");
  checkU64Eq(comms.malformedCount(), 0, "HELLO does not count as malformed");
}

void scenarioPingRepliesOkPongViaSendReliable() {
  beginScenario("pump(): PING replies \"PONG:t=<now>\" via sendReliable() (117, SUC-056; 124-005, issue §4)");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("PING");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  // A specific, nonzero `now` -- proves the reply carries THIS call's own
  // argument, not a hardcoded/zero placeholder (117 ticket 001's own AC:
  // "t= followed by the now value passed into Comms::pump()/
  // pumpTransport() for that call").
  App::Cmd cmd;
  comms.pump(cmd, /*now=*/123456);

  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one sendReliable() call");
  if (!serialFake.sentReliable().empty()) {
    checkStrEq(serialFake.sentReliable()[0], "PONG:t=123456",
               "sendReliable() carried \"PONG:t=<now>\", now == pump()'s own argument");
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
  comms.pump(cmd, /*now=*/1000);

  checkU64Eq(serialFake.inboundSize(), 0, "serial's queued line was drained this call");
  checkU64Eq(radioFake.inboundSize(), 1, "radio's queued line was NOT touched this call (serial had one)");
  checkU64Eq(serialFake.sentReliable().size(), 1, "serial received the PING reply");
  checkU64Eq(radioFake.sentReliable().size(), 0, "radio received no reply (never polled this call)");
  if (!serialFake.sentReliable().empty()) {
    checkStrEq(serialFake.sentReliable()[0], "PONG:t=1000",
               "serial transport's PING reply also carries t=<now> (117, SUC-056; 124-005, issue §4)");
  }

  // A second pump() call now drains radio's queued line -- proves the SAME
  // t=<ms> reply shape on the OTHER transport, not just serial (117 ticket
  // 001's own AC: "Verified on both the serial and radio-relay
  // transports").
  App::Cmd cmd2;
  comms.pump(cmd2, /*now=*/2000);
  checkU64Eq(radioFake.inboundSize(), 0, "radio's queued line is drained on the NEXT call");
  checkU64Eq(radioFake.sentReliable().size(), 1, "radio received the PING reply on the second call");
  if (!radioFake.sentReliable().empty()) {
    checkStrEq(radioFake.sentReliable()[0], "PONG:t=2000",
               "radio transport's PING reply also carries t=<now> (117, SUC-056; 124-005, issue §4)");
  }
}

// ===========================================================================
// 5. sendReply() round-trip: both transports' send() logs capture the exact
//    same CRC-then-COBS frame body (123-002 -- was "*B<base64>" pre-123),
//    matching an independent re-encode of the same ReplyEnvelope.
// ===========================================================================

void scenarioSendReplyBroadcastsIdenticalLineOnBothTransports() {
  beginScenario("sendReply(): identical COBS+CRC frame sent on both transports via send() (not sendReliable())");

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
               "serial and radio received byte-identical COBS+CRC frames");
  }

  // Independent re-encode, without going through Comms::sendReply() at
  // all -- proves the LINE (command prefix + COBS+CRC bytes) is exactly
  // what encode()+CRC-then-COBS-over-"OK:" would produce (round-trip proof
  // without needing a generic ReplyEnvelope decoder, per the ticket's own
  // testing plan). "OK" because reply.body_kind == BodyKind::OK -- 124-005:
  // sendReply() derives the wire verb name from body_kind INTERNALLY, so
  // the independent re-encode here must use the same mapping to compare
  // like-for-like.
  uint8_t rawBuf[App::kMaxEnvelopeBytes];
  uint16_t n = msg::wire::encode(reply, rawBuf, sizeof(rawBuf));
  checkTrue(n > 0, "independent encode() succeeds");
  std::string expected = armor(rawBuf, n, "OK");
  checkTrue(!expected.empty(), "independent armor() succeeds");

  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expected, "sendReply()'s line matches an independent re-encode+armor");
  }
}

// ===========================================================================
// 6. sendReply()'s command name is derived from body_kind (124-005): OK and
//    ERR replies land on two DIFFERENT wire verbs ("OK:"/"ERR:"), hence two
//    different CRC scopes -- this is the reply-plane half of what 124-003
//    (issue §3) built and 124-005 wires live; the ID/VER/PONG half is
//    covered by scenario 3's PING scenario and app_robot_loop_harness.cpp's
//    own ID/VER scenarios.
// ===========================================================================

void scenarioSendReplyVerbNameTracksBodyKind() {
  beginScenario(
      "sendReply(): body_kind=OK and body_kind=ERR produce lines with two different wire-verb prefixes");

  msg::ReplyEnvelope okReply;
  okReply.corr_id = 9;
  okReply.body_kind = msg::ReplyEnvelope::BodyKind::OK;
  okReply.body.ok.q = 5;

  msg::ReplyEnvelope errReply;
  errReply.corr_id = 9;
  errReply.body_kind = msg::ReplyEnvelope::BodyKind::ERR;
  errReply.body.err.code = msg::ErrCode::ERR_BADARG;

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";

  FakeTransport serialFakeOk;
  FakeTransport radioFakeOk;
  App::Comms commsOk(serialFakeOk, radioFakeOk, banner);
  commsOk.sendReply(okReply);

  FakeTransport serialFakeErr;
  FakeTransport radioFakeErr;
  App::Comms commsErr(serialFakeErr, radioFakeErr, banner);
  commsErr.sendReply(errReply);

  checkU64Eq(serialFakeOk.sent().size(), 1, "OK reply produced exactly one line");
  checkU64Eq(serialFakeErr.sent().size(), 1, "ERR reply produced exactly one line");
  if (serialFakeOk.sent().empty() || serialFakeErr.sent().empty()) return;

  const std::string& okLine = serialFakeOk.sent()[0];
  const std::string& errLine = serialFakeErr.sent()[0];
  checkTrue(okLine.rfind("OK:", 0) == 0, "OK reply's line starts with the \"OK:\" verb prefix");
  checkTrue(errLine.rfind("ERR:", 0) == 0, "ERR reply's line starts with the \"ERR:\" verb prefix");
}

// ===========================================================================
// 7. CRC-scope mismatch (124-003/124-005, issue §3/§5): a frame CRC-scoped
//    under one command name fails verification when the wire line's own
//    parsed prefix names a DIFFERENT (same-length) registered verb -- the
//    "command byte mutated in transit" acceptance criterion, exercised via
//    dispatchLine()'s real parse (not a hand-injected scope argument --
//    that seam no longer exists on the public API, by design: the parsed
//    prefix IS the scope now).
// ===========================================================================

void scenarioDecodeBinaryFrameRejectsMismatchedCommandScope() {
  beginScenario(
      "pump(): a line armored under command MOVE fails CRC verification when its own wire prefix says STOP -- "
      "never dispatches");

  Buf env;
  putVarintField(env, 1, 11);  // corr_id
  putVarintField(env, 13, 0);  // CommandEnvelope.stop (empty message, field 13)

  std::string line = armor(env.data, env.len, "MOVE");
  checkTrue(!line.empty(), "armor() produced a non-empty frame");
  checkTrue(line.rfind("MOVE:", 0) == 0, "line starts with the MOVE: prefix");
  // Mutate the wire's OWN command prefix in place -- "MOVE" and "STOP" are
  // both real, same-length (4-byte) registered binary verbs (messages/
  // commands.h), so this line still passes the registry lookup and reaches
  // decodeBinaryFrame() -- but under a CRC that was computed over "MOVE:",
  // not "STOP:".
  line.replace(0, 4, "STOP");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInboundBinary(line);

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kNone, "cmd.status stays kNone -- scope-mismatched frame never decodes");
  checkU64Eq(comms.malformedCount(), 1, "malformedCount increments exactly once (CRC-scope mismatch detected)");
}

// ===========================================================================
// 8. ID/VER cleartext replies (124-005, issue §8 / sprint 124 architecture
//    Decision 4) -- new this ticket. HELLO/PING are covered above.
// ===========================================================================

void scenarioIdRepliesWithConfiguredIdentity() {
  beginScenario("pump(): ID replies with the configured idLine_ via sendReliable()");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("ID");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  static char idLine[] = "ID:differential:tovez_nocal:0.20260724.2";
  App::Comms comms(serialFake, radioFake, banner, idLine);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kNone, "ID never decodes a Cmd");
  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one sendReliable() call");
  if (!serialFake.sentReliable().empty()) {
    checkStrEq(serialFake.sentReliable()[0], idLine, "sendReliable() carried the configured idLine_ verbatim");
  }
  checkU64Eq(comms.malformedCount(), 0, "ID does not count as malformed");
}

void scenarioVerRepliesWithBuildVersion() {
  beginScenario("pump(): VER replies \"VER:<FIRMWARE_VERSION_STR>\" via sendReliable()");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("VER");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one sendReliable() call");
  if (!serialFake.sentReliable().empty()) {
    checkTrue(serialFake.sentReliable()[0].rfind("VER:", 0) == 0, "reply starts with the \"VER:\" verb prefix");
  }
  checkU64Eq(comms.malformedCount(), 0, "VER does not count as malformed");
}

// ===========================================================================
// 9. Grammar edge cases (124-005, issue §1/§6): a no-data verb with a stray
//    trailing ':' is handled gracefully (empty data, dispatched normally --
//    NOT malformed), and a truncated binary line (a ':' with no COBS body
//    at all) is counted malformed, not crashed.
// ===========================================================================

void scenarioStrayTrailingColonOnNoDataVerbHandledGracefully() {
  beginScenario("pump(): \"PING:\" (stray trailing ':', empty data) still replies PONG -- not malformed");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("PING:");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/777);

  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one sendReliable() call");
  if (!serialFake.sentReliable().empty()) {
    checkStrEq(serialFake.sentReliable()[0], "PONG:t=777",
               "\"PING:\"'s empty data after the colon is dispatched exactly like bare \"PING\"");
  }
  checkU64Eq(comms.malformedCount(), 0, "a stray trailing ':' on a no-data verb does not count as malformed");
}

void scenarioTruncatedBinaryLineCountsMalformedNotCrash() {
  beginScenario("pump(): \"MOVE:\" (colon, zero-length COBS body) counts malformed, does not crash");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInbound("MOVE:");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kNone, "cmd.status stays kNone -- no crash, no partial decode");
  checkU64Eq(comms.malformedCount(), 1, "malformedCount increments exactly once");
}

void scenarioDataContainingColonAndZeroRoundTripsCorrectly() {
  beginScenario(
      "pump(): a MOVE envelope carrying an unknown field whose bytes embed ':' and 0x00 still decodes -- the "
      "grammar's first-colon-ends-command rule is unambiguous even though binary data contains 0x3A/0x00");

  // An unknown field (99, length-delimited) whose payload is exactly the
  // bytes {':', 0x00, 'A'} -- msg::wire::decode() skips unrecognized field
  // numbers (forward-compat), so this proves the COLON and NUL bytes
  // embedded in the COBS-encoded data never get mistaken for the outer
  // grammar's own separator/terminator: only the FIRST ':' in the wire
  // LINE (right after "MOVE") ends the command; everything COBS-encoded
  // after it, including these bytes, is opaque data (issue §1).
  const uint8_t needle[] = {':', 0x00, 'A'};
  Buf move;
  putBytesField(move, 99, needle, sizeof(needle));
  Buf env;
  putVarintField(env, 1, 55);      // corr_id
  putMessageField(env, 21, move);  // CommandEnvelope.cmd.move, field 21

  std::string line = armor(env.data, env.len, "MOVE");
  checkTrue(!line.empty(), "armor() produced a non-empty frame");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInboundBinary(line);

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kDecoded,
            "decodes cleanly despite ':' and 0x00 embedded in the COBS-encoded data");
  checkU64Eq(cmd.env.corr_id, 55, "corr_id round-trips despite the embedded ':'/0x00 in a sibling field");
  checkU64Eq(comms.malformedCount(), 0, "not counted malformed");
}

}  // namespace

int main() {
  scenarioMoveRoundTrip();
  scenarioMalformedUnrecognizedTextLineRejected();
  scenarioMalformedCobsFrameRejected();
  scenarioMalformedCrcMismatchRejected();
  scenarioMalformedCorruptProtobufRejected();
  scenarioHelloRepliesWithBannerViaSendReliable();
  scenarioPingRepliesOkPongViaSendReliable();
  scenarioPumpBoundedToOneTransportPerCall();
  scenarioSendReplyBroadcastsIdenticalLineOnBothTransports();
  scenarioSendReplyVerbNameTracksBodyKind();
  scenarioDecodeBinaryFrameRejectsMismatchedCommandScope();
  scenarioIdRepliesWithConfiguredIdentity();
  scenarioVerRepliesWithBuildVersion();
  scenarioStrayTrailingColonOnNoDataVerbHandledGracefully();
  scenarioTruncatedBinaryLineCountsMalformedNotCrash();
  scenarioDataContainingColonAndZeroRoundTripsCorrectly();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Comms scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Comms scenarios\n", g_failureCount);
  return 1;
}
