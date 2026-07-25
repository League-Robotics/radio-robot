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

// Armor a raw CommandEnvelope/ReplyEnvelope byte buffer into a CRC-then-COBS
// frame body (123-002 -- was "*B<base64>" pre-123) -- the SAME composition
// comms.cpp's own decodeBinaryFrame()/sendReply() perform, used here only to
// construct scenario INPUT (pump()'s inbound side), independent of Comms's
// own outbound path so the two directions are tested against each other,
// not tautologically. The trailing 0x00 delimiter is a transport concern,
// not included here -- push the result via FakeTransport::
// enqueueInboundBinary(), not enqueueInbound() (that tags a frame kText).
std::string armor(const uint8_t* raw, size_t rawLen) {
  uint8_t combined[256];
  if (rawLen > sizeof(combined) - 2) return std::string();
  std::memcpy(combined, raw, rawLen);
  size_t combinedLen = rawLen;
  const uint16_t crc = WireRuntime::crcCompute(raw, rawLen);
  if (!WireRuntime::encodeCrc16(crc, combined, sizeof(combined), &combinedLen)) return std::string();
  uint8_t framed[300];
  size_t framedLen = 0;
  if (!WireRuntime::cobsEncode(combined, combinedLen, framed, sizeof(framed), &framedLen)) return std::string();
  return std::string(reinterpret_cast<const char*>(framed), framedLen);
}

// armorScoped() -- extends armor() with a CRC-scope command-name prefix
// (124-003, issue §3): `crc16(command ':' raw)` instead of `crc16(raw)`
// alone when `commandLen > 0`. An INDEPENDENT re-implementation of
// comms.cpp's own crcOverScope() (a private helper, not reachable from
// this translation unit), built the exact same way -- WireRuntime's
// incremental crcInit()/crcUpdate(), command and raw bytes never
// concatenated into one buffer -- so the two directions are tested
// against each other, not tautologically (same discipline armor() itself
// already follows for the unscoped case).
std::string armorScoped(const uint8_t* raw, size_t rawLen, const uint8_t* command, size_t commandLen) {
  uint8_t combined[256];
  if (rawLen > sizeof(combined) - 2) return std::string();
  std::memcpy(combined, raw, rawLen);
  size_t combinedLen = rawLen;
  uint16_t crc = WireRuntime::crcInit();
  if (commandLen > 0) {
    crc = WireRuntime::crcUpdate(crc, command, commandLen);
    const uint8_t sep = ':';
    crc = WireRuntime::crcUpdate(crc, &sep, 1);
  }
  crc = WireRuntime::crcUpdate(crc, raw, rawLen);
  if (!WireRuntime::encodeCrc16(crc, combined, sizeof(combined), &combinedLen)) return std::string();
  uint8_t framed[300];
  size_t framedLen = 0;
  if (!WireRuntime::cobsEncode(combined, combinedLen, framed, sizeof(framed), &framedLen)) return std::string();
  return std::string(reinterpret_cast<const char*>(framed), framedLen);
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

  std::string line = armor(env.data, env.len);
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
  // remain" malformed-input case (wire_runtime.cpp).
  const uint8_t badFrame[] = {0x0B, 0x01, 0x02};

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInboundBinary(badFrame, sizeof(badFrame));

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

  std::string line = armor(env.data, env.len);
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

  std::string line = armor(env.data, env.len);
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
  beginScenario("pump(): PING replies \"OK pong t=<now>\" via sendReliable() (117, SUC-056)");

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
    checkStrEq(serialFake.sentReliable()[0], "OK pong t=123456",
               "sendReliable() carried \"OK pong t=<now>\", now == pump()'s own argument");
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
    checkStrEq(serialFake.sentReliable()[0], "OK pong t=1000",
               "serial transport's PING reply also carries t=<now> (117, SUC-056)");
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
    checkStrEq(radioFake.sentReliable()[0], "OK pong t=2000",
               "radio transport's PING reply also carries t=<now> (117, SUC-056)");
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
  // all -- proves the frame is exactly what encode()+CRC-then-COBS would
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

// ===========================================================================
// 6. CRC-scope extension (124-003, issue §3): the CRC's input now covers
//    `COMMAND ':' payload`, not just `payload` -- sendReply()/
//    decodeBinaryFrame() take the command bytes as a SEPARATE argument
//    (default empty, byte-identical to protocol v4's CRC, since no CURRENT
//    call site has a real ASCII command prefix to pass yet -- that grammar
//    cutover is ticket 124-005).
// ===========================================================================

void scenarioSendReplyCrcScopeDiffersByCommand() {
  beginScenario(
      "sendReply(): identical reply payload under two different command names produces two different CRCs "
      "(124-003, issue §3)");

  msg::ReplyEnvelope reply;
  reply.corr_id = 9;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::OK;
  reply.body.ok.q = 5;
  reply.body.ok.rem = 12.5f;
  reply.body.ok.t = 4242;

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";

  const uint8_t commandA[] = {'M', 'O', 'V', 'E'};
  const uint8_t commandB[] = {'S', 'T', 'O', 'P'};

  FakeTransport serialFakeA;
  FakeTransport radioFakeA;
  App::Comms commsA(serialFakeA, radioFakeA, banner);
  commsA.sendReply(reply, commandA, sizeof(commandA));

  FakeTransport serialFakeB;
  FakeTransport radioFakeB;
  App::Comms commsB(serialFakeB, radioFakeB, banner);
  commsB.sendReply(reply, commandB, sizeof(commandB));

  checkU64Eq(serialFakeA.sent().size(), 1, "commandA send produced exactly one frame");
  checkU64Eq(serialFakeB.sent().size(), 1, "commandB send produced exactly one frame");
  if (serialFakeA.sent().empty() || serialFakeB.sent().empty()) return;

  const std::string& frameA = serialFakeA.sent()[0];
  const std::string& frameB = serialFakeB.sent()[0];
  checkTrue(frameA != frameB, "two different command names produce two different frames (the CRC differs)");

  // Decode both frames independently (plain COBS, delimiter 0x00 -- the
  // wire LAYOUT is unaffected by 124-003, only the CRC's input range) and
  // confirm the schema-encoded payload bytes are IDENTICAL between the two
  // (same reply content, same encode()) while the trailing 2-byte CRC
  // differs -- isolating the difference to the CRC exactly, not a
  // coincidental side effect of framing.
  uint8_t decodedA[300] = {};
  size_t decodedALen = 0;
  uint8_t decodedB[300] = {};
  size_t decodedBLen = 0;
  checkTrue(WireRuntime::cobsDecode(reinterpret_cast<const uint8_t*>(frameA.data()), frameA.size(), decodedA,
                                     sizeof(decodedA), &decodedALen),
            "frame A COBS-decodes cleanly");
  checkTrue(WireRuntime::cobsDecode(reinterpret_cast<const uint8_t*>(frameB.data()), frameB.size(), decodedB,
                                     sizeof(decodedB), &decodedBLen),
            "frame B COBS-decodes cleanly");
  checkTrue(decodedALen == decodedBLen && decodedALen >= 2, "both decode to the same combined length");
  if (decodedALen == decodedBLen && decodedALen >= 2) {
    const size_t payloadLen = decodedALen - 2;
    checkTrue(std::memcmp(decodedA, decodedB, payloadLen) == 0,
              "the schema-encoded payload bytes are identical between the two command names");
    checkTrue(std::memcmp(decodedA + payloadLen, decodedB + payloadLen, 2) != 0,
              "the trailing 2-byte CRC differs between the two command names");
  }
}

void scenarioDecodeBinaryFrameRejectsMismatchedCommandScope() {
  beginScenario(
      "pump(): frame CRC-scoped under a command name (124-003) fails verification when decoded under a "
      "DIFFERENT scope -- never dispatches");

  Buf env;
  putVarintField(env, 1, 11);  // corr_id
  putVarintField(env, 13, 0);  // CommandEnvelope.stop (empty message, field 13)

  const uint8_t command[] = {'M', 'O', 'V', 'E'};
  std::string line = armorScoped(env.data, env.len, command, sizeof(command));
  checkTrue(!line.empty(), "armorScoped() produced a non-empty frame");

  FakeTransport serialFake;
  FakeTransport radioFake;
  serialFake.enqueueInboundBinary(line);

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);

  // pump()'s CURRENT call site (pumpTransport(), comms.cpp) decodes with
  // decodeBinaryFrame()'s default EMPTY CRC scope -- the wire's own ASCII
  // command prefix is not parsed and threaded through yet (that is ticket
  // 124-005's grammar cutover; comms.h's decodeBinaryFrame() doc comment
  // says so explicitly). So a frame whose CRC was computed under a REAL
  // command scope must fail verification here -- a stand-in for "the
  // command byte was mutated/lost in transit" the issue's own AC names,
  // proven with the tooling this ticket actually lands rather than the
  // full wire-line parse 005 owns.
  App::Cmd cmd;
  comms.pump(cmd, /*now=*/0);

  checkTrue(cmd.status == App::CmdStatus::kNone, "cmd.status stays kNone -- scope-mismatched frame never decodes");
  checkU64Eq(comms.malformedCount(), 1, "malformedCount increments exactly once (CRC-scope mismatch detected)");
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
  scenarioSendReplyCrcScopeDiffersByCommand();
  scenarioDecodeBinaryFrameRejectsMismatchedCommandScope();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Comms scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Comms scenarios\n", g_failureCount);
  return 1;
}
