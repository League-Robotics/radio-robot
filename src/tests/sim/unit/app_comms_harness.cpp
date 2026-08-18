// app_comms_harness.cpp -- off-hardware acceptance harness for ticket
// 103-004 (SUC-004), Core::Comms (src/firm/app/comms.{h,cpp}). Proves the
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
// FakeTransport (Core::Transport) is TestSupport::FakeTransport
// (src/tests/sim/support/fake_transport.h, ticket 105-002) -- the ONE canonical
// scripted queue of inbound lines plus a log of every send()/sendReliable()
// call, so scenarios can feed input and assert exactly what got sent. This
// harness previously carried its own ad hoc copy; it now builds on the
// shared primitive (105-002's dedup mandate) with no scenario changes.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "core/comms.h"
#include "control/differential_drive.h"
#include "core/telemetry.h"
#include "host_fiber.h"
#include "hal/clock.h"
#include "hal/motor.h"
#include "firm/types/robot_state.h"
#include "messages/envelope.h"
#include "messages/wire.h"
#include "messages/wire_runtime.h"
#include "support/fake_transport.h"

// StubMotor/StubClock/StubSleeper/testDrive() -- Telemetry::update() takes
// a `const Control::DifferentialDrive&` for its own observability
// accessors; this harness never ticks/starts it, only constructs it.
// EXPLORATORY-KERNEL REWRITE (2026-08-15): applyTravelCalib() is gone
// (Hal::Motor no longer declares it -- the leaf is counts-native) and the
// kernel's constructor now takes a Hal::Clock&/Hal::Sleeper& instead of a
// trackWidth (the kernel has no trackWidth concept at all -- see
// differential_drive.h's own header).
class StubMotor : public Hal::Motor {
 public:
  void begin() override {}
  void requestSample() override {}
  void setDuty(float) override {}
  void setNeutral(Hal::Neutral) override {}
  // Records the emergency zero so a test can assert it actually
  // happened -- a mock that silently swallowed it would make the
  // sentinel's whole point untestable.
  void emergencyStop() override { ++emergencyStopCount; }
  int emergencyStopCount = 0;
  bool reconfigure(const Hal::MotorConfig&) override { return true; }
  void tick(uint64_t) override {}
  float position() const override { return 0.0f; }
  float velocity() const override { return 0.0f; }
  float appliedDuty() const override { return 0.0f; }
  bool connected() const override { return true; }
  uint64_t sampleTime() const override { return 0; }
  void resetPosition() override {}
  void rebaseline() override {}
};

class StubClock : public Hal::Clock {
 public:
  uint64_t nowMicros() const override { return 0; }
};

class StubSleeper : public Hal::Sleeper {
 public:
  void sleepMillis(uint32_t) override {}
  void yield() override {}
};

Control::DifferentialDrive& testDrive() {
  static StubMotor left;
  static StubMotor right;
  static StubClock clock;
  static StubSleeper sleeper;
  // The kernel takes its launcher at construction now. This harness
  // never calls start() -- FailingFiberLauncher aborts if it ever
  // does, which is the point: no fibers in a host test.
  static TestSim::FailingFiberLauncher fiberLauncher;
  static Control::DifferentialDrive drive(left, right, clock, sleeper, fiberLauncher);
  return drive;
}

// pumpOne() -- the pre-ring `Comms::pump(Cmd&, now)` shape, rebuilt on the
// ring API (command-ingestion-ring-buffered-comms-subsystem-routing-two-
// stops.md §1: pump() now DRAINS both transports into a ring and
// takeCommand() pops from it). These scenarios each feed exactly one line
// and want the one command it produced, so "pump then take" is the honest
// local equivalent -- not a claim that pump() still stops after one line.
Core::Cmd pumpOne(Core::Comms& comms, uint32_t now) {  // [ms]
  Core::Cmd cmd;
  comms.pump(now);
  comms.takeCommand(cmd);  // leaves cmd at status kNone when nothing decoded
  return cmd;
}

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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

  checkTrue(cmd.status == Core::CmdStatus::kDecoded, "cmd.status == kDecoded");
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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

  checkTrue(cmd.status == Core::CmdStatus::kNone, "cmd.status stays kNone");
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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

  checkTrue(cmd.status == Core::CmdStatus::kNone, "cmd.status stays kNone");
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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

  checkTrue(cmd.status == Core::CmdStatus::kNone, "cmd.status stays kNone -- corrupted frame never decodes");
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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

  checkTrue(cmd.status == Core::CmdStatus::kNone, "cmd.status stays kNone");
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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

  checkTrue(cmd.status == Core::CmdStatus::kNone, "HELLO never decodes a Cmd");
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
  Core::Comms comms(serialFake, radioFake, banner);

  // A specific, nonzero `now` -- proves the reply carries THIS call's own
  // argument, not a hardcoded/zero placeholder (117 ticket 001's own AC:
  // "t= followed by the now value passed into Comms::pump()/
  // pumpTransport() for that call").
  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/123456);

  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one sendReliable() call");
  if (!serialFake.sentReliable().empty()) {
    checkStrEq(serialFake.sentReliable()[0], "PONG:t=123456",
               "sendReliable() carried \"PONG:t=<now>\", now == pump()'s own argument");
  }
}

// ===========================================================================
// 4. QUARANTINED -- DEPRECATED-COMMAND-INGEST
//
// This is the canonical statement of the contract the command-ingestion
// rework REPLACED (command-ingestion-ring-buffered-comms-subsystem-routing-
// two-stops.md §1, "Deferred: the rest of the test suite"): pump() no
// longer stops after one transport, it DRAINS both into a command ring, so
// every assertion below now asserts the opposite of the truth. Its
// replacement -- a scenario asserting the drain contract, and the two
// PING-reply-per-transport checks it also carried -- is explicitly deferred
// to the later big-bang test pass this marker is the grep handle for.
// ===========================================================================
#if 0  // DEPRECATED-COMMAND-INGEST
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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/1000);

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
  Core::Cmd cmd2;
  cmd2 = pumpOne(comms, /*now=*/2000);
  checkU64Eq(radioFake.inboundSize(), 0, "radio's queued line is drained on the NEXT call");
  checkU64Eq(radioFake.sentReliable().size(), 1, "radio received the PING reply on the second call");
  if (!radioFake.sentReliable().empty()) {
    checkStrEq(radioFake.sentReliable()[0], "PONG:t=2000",
               "radio transport's PING reply also carries t=<now> (117, SUC-056; 124-005, issue §4)");
  }
}

#endif  // DEPRECATED-COMMAND-INGEST

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
  Core::Comms comms(serialFake, radioFake, banner);

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
  uint8_t rawBuf[Core::kMaxEnvelopeBytes];
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
  Core::Comms commsOk(serialFakeOk, radioFakeOk, banner);
  commsOk.sendReply(okReply);

  FakeTransport serialFakeErr;
  FakeTransport radioFakeErr;
  Core::Comms commsErr(serialFakeErr, radioFakeErr, banner);
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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

  checkTrue(cmd.status == Core::CmdStatus::kNone, "cmd.status stays kNone -- scope-mismatched frame never decodes");
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
  Core::Comms comms(serialFake, radioFake, banner, idLine);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

  checkTrue(cmd.status == Core::CmdStatus::kNone, "ID never decodes a Cmd");
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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/777);

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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

  checkTrue(cmd.status == Core::CmdStatus::kNone, "cmd.status stays kNone -- no crash, no partial decode");
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
  Core::Comms comms(serialFake, radioFake, banner);

  Core::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);

  checkTrue(cmd.status == Core::CmdStatus::kDecoded,
            "decodes cleanly despite ':' and 0x00 embedded in the COBS-encoded data");
  checkU64Eq(cmd.env.corr_id, 55, "corr_id round-trips despite the embedded ':'/0x00 in a sibling field");
  checkU64Eq(comms.malformedCount(), 0, "not counted malformed");
}

// ===========================================================================
// 10. Relay connect-handshake regression (124-010, ticket 010,
//     relay-handshake-trips-comms-malformed.md, SUC-008): reproduces the
//     issue's own isolated-test sequence off-hardware -- fresh Comms
//     ("clean-boot firmware"), a relay-shaped connect with ZERO
//     application commands, inspect malformedCount() (the host-side
//     observable is `fault_bits` bit 3 / kFaultCommsMalformed, which
//     RobotLoop derives 1:1 from this counter, robot_loop.cpp) across
//     several pump() drains (a "settle window") -- stays 0. Lines are
//     injected on radioFake specifically: a real relay leak lands on
//     the robot's OWN radio transport (radioLink_), never its silent,
//     unconnected USB serial (serialLink_) -- see comms.h's own
//     Comms::pump() doc comment ("serial + radio" transports) and
//     src/firm/main.cpp's wiring (RadioTransport around com/radio.h).
//     The exact relay-shaped lines below are the dongle's own
//     control-plane vocabulary verbatim (host/robot_radio/io/
//     serial_conn.py's `_banner_classify()`/`_relay_handshake()`):
//     "?" (channel/group/mode query), "!ECHO OFF", "!MODE RAW250", "!GO"
//     (the dongle commands proper), and "# entering data plane" (the
//     dongle's own status reply) -- covering both '!'/'?' (what the HOST
//     writes toward the dongle) and '#' (what the dongle writes back)
//     sigils in one scenario, matching the issue's "Direction" section
//     candidates.
// ===========================================================================

void scenarioRelayHandshakeChatterNeverCountsAsMalformed() {
  beginScenario(
      "pump(): leaked relay control-plane lines ('?'/'!.../'# entering data plane') on radioLink_ never "
      "count as malformed -- 124-010 isolated-repro sequence, multiple trials");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";

  // Three independent "fresh clean-boot" trials (124-010 AC: "reproduced
  // across multiple fresh-boot trials" -- a fresh Comms instance per
  // trial mirrors mbdeploy re-flashing before each isolated test run).
  for (int trial = 0; trial < 3; ++trial) {
    FakeTransport serialFake;
    FakeTransport radioFake;
    // The exact relay dongle handshake vocabulary, in the exact order
    // _banner_classify()/_relay_handshake() emit it -- zero application
    // commands anywhere in this queue (SUC-008's own precondition).
    radioFake.enqueueInbound("?channel:1 group:10 mode:RAW250");
    radioFake.enqueueInbound("!ECHO OFF");
    radioFake.enqueueInbound("!MODE RAW250");
    radioFake.enqueueInbound("!GO");
    radioFake.enqueueInbound("# entering data plane");

    Core::Comms comms(serialFake, radioFake, banner);

    // Drain every queued line (the "~1 s settle window" the issue's own
    // repro waits out before inspecting fault_bits). ONE pump() call now
    // drains both transports to empty (command-ingestion-ring-buffered-
    // comms-subsystem-routing-two-stops.md §1) -- the loop is kept so this
    // scenario still proves repeated pumping is idempotent on empty
    // transports, and that no relay-chatter line ever leaves a Cmd behind.
    for (int i = 0; i < 5; ++i) {
      Core::Cmd cmd = pumpOne(comms, /*now=*/static_cast<uint32_t>(i));
      checkTrue(cmd.status == Core::CmdStatus::kNone, "no relay chatter line ever decodes a Cmd");
    }

    checkU64Eq(radioFake.inboundSize(), 0, "every queued relay-chatter line was drained this trial");
    checkU64Eq(comms.malformedCount(), 0,
               "malformedCount stays 0 through a full relay-shaped connect handshake with zero application "
               "commands (trial " + std::to_string(trial) + ")");
  }
}

// ===========================================================================
// 11. The carve-out is narrow, not a general amnesty: a genuinely
//     unrecognized line that does NOT start with a relay sigil still
//     counts as malformed (guards against the fix over-broadening), AND a
//     real command dispatches normally immediately after relay chatter
//     (guards against the carve-out leaving any residual demux state that
//     would corrupt the FIRST real application command a host sends right
//     after connect).
// ===========================================================================

void scenarioRelayCarveOutIsNarrowAndDoesNotAffectSubsequentRealCommand() {
  beginScenario(
      "pump(): relay-sigil carve-out is narrow (non-sigil garbage still malformed) and leaves no residual "
      "state before the next real command");

  FakeTransport serialFake;
  FakeTransport radioFake;
  // A non-sigil-prefixed, still-unrecognized line -- must NOT be silently
  // tolerated by the new carve-out (it is not shaped like relay chatter).
  radioFake.enqueueInbound("GARBAGE");
  // Relay chatter, exactly as a leaked handshake fragment would arrive.
  radioFake.enqueueInbound("!MODE RAW250");
  // A real, registered no-data verb -- must still dispatch normally
  // right after the relay chatter, proving the carve-out (an early
  // `return` in dispatchLine(), never touching any parser/decoder state)
  // leaves nothing behind that could corrupt the next real line.
  radioFake.enqueueInbound("PING");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);

  // ONE pump() call consumes all three queued lines (command-ingestion-
  // ring-buffered-comms-subsystem-routing-two-stops.md §1: pump() drains
  // the transports rather than stopping after one line), so the whole
  // sequence -- garbage, relay chatter, real command -- is exercised in a
  // single call. That is a STRONGER form of this scenario's own question:
  // if the carve-out left residual parser state behind, draining all three
  // back-to-back inside one call is exactly where it would show.
  comms.pump(/*now=*/999);
  Core::Cmd cmd;
  checkTrue(!comms.takeCommand(cmd), "none of the three lines decodes into a command");
  checkU64Eq(comms.malformedCount(), 1,
             "exactly one malformed count: the non-sigil garbage line. Relay chatter is carved out, "
             "PING dispatches");
  checkU64Eq(radioFake.sentReliable().size(), 1, "PING replied normally right after relay chatter");
  if (!radioFake.sentReliable().empty()) {
    checkStrEq(radioFake.sentReliable()[0], "PONG:t=999", "PING's reply content is unaffected by prior chatter");
  }
}

// ===========================================================================
// 12. TLM: command surface (125-003, telemetry-emit-policy-rebuild-spec.md
//     Part 4): every row of the normative table, parsed by dispatchLine()
//     into a Comms::TlmAction, plus the STATUS/HELP reply shape
//     sendTlmReply() produces on the SAME transport the line arrived on.
//     Comms holds no Telemetry& (Part 4's own boundary), so these scenarios
//     prove the PARSE + REPLY halves at the Comms level only; RobotLoop's
//     own application of the mode change to Telemetry is
//     app_robot_loop_harness.cpp's job.
// ===========================================================================

void scenarioBareTlmAndTlmNowBothProduceKFrameNoModeChange() {
  beginScenario("pump(): bare \"TLM\" and \"TLM:NOW\" both parse to TlmAction::kFrame -- no mode change");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);

  serialFake.enqueueInbound("TLM");
  comms.pump(/*now=*/0);
  checkTrue(comms.takeTlmAction() == Core::Comms::TlmAction::kFrame, "bare TLM -> kFrame");

  serialFake.enqueueInbound("TLM:NOW");
  comms.pump(/*now=*/0);
  checkTrue(comms.takeTlmAction() == Core::Comms::TlmAction::kFrame, "TLM:NOW -> kFrame, same as bare TLM");

  Core::Cmd cmd;
  checkTrue(!comms.takeCommand(cmd), "neither line ever enters the command ring");
  checkU64Eq(comms.malformedCount(), 0, "neither line counts as malformed");
}

void scenarioTlmModeTokensCaseInsensitiveProduceCorrectAction() {
  beginScenario("pump(): TLM:ON/AUTO/OFF (any case) parse to the matching TlmAction, colon-spelled only");

  struct Row {
    const char* line;
    Core::Comms::TlmAction expected;
  };
  const Row rows[] = {
      {"TLM:ON", Core::Comms::TlmAction::kSetOn},     {"TLM:on", Core::Comms::TlmAction::kSetOn},
      {"TLM:On", Core::Comms::TlmAction::kSetOn},     {"TLM:AUTO", Core::Comms::TlmAction::kSetAuto},
      {"TLM:auto", Core::Comms::TlmAction::kSetAuto}, {"TLM:AuTo", Core::Comms::TlmAction::kSetAuto},
      {"TLM:OFF", Core::Comms::TlmAction::kSetOff},   {"TLM:off", Core::Comms::TlmAction::kSetOff},
  };

  for (const Row& row : rows) {
    FakeTransport serialFake;
    FakeTransport radioFake;
    static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
    Core::Comms comms(serialFake, radioFake, banner);
    serialFake.enqueueInbound(row.line);
    comms.pump(/*now=*/0);
    checkTrue(comms.takeTlmAction() == row.expected, std::string(row.line) + " parses to the expected TlmAction");
  }
}

void scenarioTlmGarbageArgProducesUnrecognizedNotMalformed() {
  beginScenario("pump(): \"TLM:<garbage>\" parses to kUnrecognized -- a valid line, not a malformed one");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);

  serialFake.enqueueInbound("TLM:BOGUS");
  comms.pump(/*now=*/0);
  checkTrue(comms.takeTlmAction() == Core::Comms::TlmAction::kUnrecognized, "TLM:BOGUS -> kUnrecognized");
  checkU64Eq(comms.malformedCount(), 0, "an unrecognized TLM: argument is not counted malformed");
}

void scenarioTlmWithSpaceNotColonFallsThroughToMalformed() {
  beginScenario(
      "pump(): \"TLM ON\" (space, not colon) is NOT special-cased -- falls through to the unrecognized-verb path");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);

  serialFake.enqueueInbound("TLM ON");
  comms.pump(/*now=*/0);
  checkTrue(comms.takeTlmAction() == Core::Comms::TlmAction::kNone, "\"TLM ON\" (no colon) never stages a TlmAction");
  checkU64Eq(comms.malformedCount(), 1,
             "\"TLM ON\" does not match the registered \"TLM\" verb name -- malformedCount increments");
}

void scenarioTlmModeChangeRepliesWithStatusLineOnOriginatingTransport() {
  beginScenario(
      "sendTlmReply(): TLM:ON replies with the STATUS line, carrying setStatus()'s tlm=, on the SAME "
      "transport the line arrived on (radio, not serial)");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);

  // Radio, not serial -- proves the reply transport is remembered per-line,
  // not hardcoded to serialLink_.
  radioFake.enqueueInbound("TLM:ON");
  comms.pump(/*now=*/0);
  const Core::Comms::TlmAction action = comms.takeTlmAction();
  checkTrue(action == Core::Comms::TlmAction::kSetOn, "setup: TLM:ON parses to kSetOn");

  // Mirrors RobotLoop::cycle()'s own ordering: apply the mode (not modeled
  // here -- Comms holds no Telemetry&), THEN refresh Comms::Status with the
  // NEW mode, THEN reply -- sendTlmReply() must be called in that order.
  Core::Comms::Status status;
  status.ready = true;
  status.tlmMode = 2;  // Core::TlmMode::kOn
  comms.setStatus(status);
  comms.sendTlmReply(action);

  checkU64Eq(radioFake.sentReliable().size(), 1, "exactly one reply, on radio");
  checkU64Eq(serialFake.sentReliable().size(), 0, "no reply on serial -- the line arrived on radio");
  if (!radioFake.sentReliable().empty()) {
    const std::string& line = radioFake.sentReliable()[0];
    checkTrue(line.rfind("STATUS:", 0) == 0, "TLM:ON's reply is a STATUS line");
    checkTrue(line.find("tlm=on") != std::string::npos, "STATUS line's tlm= field reports the NEW mode (on)");
  }
}

void scenarioTlmGarbageRepliesWithHelpLineListingTlmArgumentGrammar() {
  beginScenario("sendTlmReply(): TLM:<garbage> replies with the HELP line, listing TLM's own argument grammar");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);

  serialFake.enqueueInbound("TLM:BOGUS");
  comms.pump(/*now=*/0);
  const Core::Comms::TlmAction action = comms.takeTlmAction();
  comms.sendTlmReply(action);

  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one reply");
  if (!serialFake.sentReliable().empty()) {
    const std::string& line = serialFake.sentReliable()[0];
    checkTrue(line.rfind("HELP:", 0) == 0, "TLM:<garbage>'s reply is the HELP line");
    checkTrue(line.find("TLM[:NOW|ON|AUTO|OFF]") != std::string::npos,
              "HELP line lists TLM's own argument grammar, not the bare verb name");
  }
}

void scenarioBareTlmProducesNoSendTlmReplyOutputOfItsOwn() {
  beginScenario(
      "sendTlmReply(): kFrame (bare TLM / TLM:NOW) sends nothing itself -- the reply is the forced telemetry "
      "frame RobotLoop's own emit() sends separately");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);

  serialFake.enqueueInbound("TLM:NOW");
  comms.pump(/*now=*/0);
  const Core::Comms::TlmAction action = comms.takeTlmAction();
  comms.sendTlmReply(action);

  checkU64Eq(serialFake.sentReliable().size(), 0, "sendTlmReply(kFrame) sends no reliable reply of its own");
  checkU64Eq(serialFake.sent().size(), 0, "sendTlmReply(kFrame) sends no async reply either");
}

void scenarioStatusLineCarriesTlmFieldForEveryMode() {
  beginScenario("STATUS: tlm= field reflects status_.tlmMode for all three modes (off/auto/on)");

  struct Row {
    uint8_t tlmMode;
    const char* expected;
  };
  const Row rows[] = {
      {0, "tlm=off"},
      {1, "tlm=auto"},
      {2, "tlm=on"},
  };

  for (const Row& row : rows) {
    FakeTransport serialFake;
    FakeTransport radioFake;
    static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
    Core::Comms comms(serialFake, radioFake, banner);

    Core::Comms::Status status;
    status.tlmMode = row.tlmMode;
    comms.setStatus(status);

    serialFake.enqueueInbound("STATUS");
    Core::Cmd cmd = pumpOne(comms, /*now=*/0);
    checkTrue(cmd.status == Core::CmdStatus::kNone, "STATUS never decodes a Cmd");
    checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one STATUS reply");
    if (!serialFake.sentReliable().empty()) {
      checkTrue(serialFake.sentReliable()[0].find(row.expected) != std::string::npos,
                std::string("STATUS line contains ") + row.expected);
    }
  }
}

void scenarioStatusDefaultsTlmAutoBeforeAnySetStatusCall() {
  beginScenario(
      "STATUS: a freshly constructed Comms (before any setStatus() call) already reports tlm=auto, matching "
      "Telemetry::mode_'s own kAuto default");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);

  serialFake.enqueueInbound("STATUS");
  Core::Cmd cmd = pumpOne(comms, /*now=*/0);
  checkTrue(cmd.status == Core::CmdStatus::kNone, "STATUS never decodes a Cmd");
  if (!serialFake.sentReliable().empty()) {
    checkTrue(serialFake.sentReliable()[0].find("tlm=auto") != std::string::npos,
              "default STATUS (no setStatus() call yet) reports tlm=auto");
  }
}

void scenarioHelpLineListsTlmArgumentGrammar() {
  beginScenario("HELP: lists TLM[:NOW|ON|AUTO|OFF] (not the bare TLM verb name), alongside the other cleartext verbs");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);

  serialFake.enqueueInbound("HELP");
  Core::Cmd cmd = pumpOne(comms, /*now=*/0);
  checkTrue(cmd.status == Core::CmdStatus::kNone, "HELP never decodes a Cmd");
  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one HELP reply");
  if (!serialFake.sentReliable().empty()) {
    const std::string& line = serialFake.sentReliable()[0];
    checkTrue(line.rfind("HELP:", 0) == 0, "reply starts with the HELP: verb prefix");
    checkTrue(line.find("TLM[:NOW|ON|AUTO|OFF]") != std::string::npos, "HELP line lists TLM's argument grammar");
    checkTrue(line.find("STATUS") != std::string::npos, "HELP line still lists the pre-existing cleartext verbs too");
  }
}

// ===========================================================================
// 13. updateStatus() (128-012: absorbs RobotLoop::cycle()'s previous inline
//     field-by-field STATUS assembly). Drives it directly with a
//     synthesized Types::RobotState + a real Core::Telemetry (for the two
//     Telemetry-sourced fields, flags/tlmMode) -- no RobotLoop, no full
//     loop tick -- then reads the projection back off the wire via a
//     "STATUS" query, the same black-box observable every other STATUS
//     scenario in this file already uses (Comms exposes no direct status_
//     accessor -- sendStatus()'s own formatted line IS the public contract).
// ===========================================================================

void scenarioUpdateStatusProjectsAllEightFieldsFromSynthesizedState() {
  beginScenario("updateStatus(): all 8 STATUS fields -- including ready and the two Telemetry-sourced fields "
                "(flags/tlmMode) -- reflect a synthesized RobotState + Telemetry, not stale/default values");

  FakeTransport serialFake;
  FakeTransport radioFake;
  FakeTransport tlmSerialFake;
  FakeTransport tlmRadioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);
  // Telemetry's own Comms& is a SEPARATE instance from `comms` above --
  // updateStatus() only ever reads tlm.flags()/tlm.mode(), never anything
  // that would route a frame through this second Comms, so the two are
  // deliberately decoupled here (this scenario tests updateStatus()'s own
  // projection, not Telemetry's send path).
  Core::Comms tlmComms(tlmSerialFake, tlmRadioFake, banner);
  Core::Telemetry tlm(tlmComms);
  tlm.setMode(Core::TlmMode::kOn);

  Types::RobotState state;
  state.health.ready = true;
  state.command.moveActive = true;
  state.wheelLeft.connected = true;
  state.wheelRight.connected = false;  // deliberately the ONE false boolean --
                                        // proves the projection isn't just
                                        // echoing a single shared value
  state.otos.present = true;
  state.otos.connected = true;
  state.health.wedgeLatch = true;
  tlm.update(state, testDrive());  // derives flags_ from `state` -- kFlagActive | kFlagConnLeft |
                       // kFlagOtosPresent | kFlagOtosConnected | kFlagFaultWedgeLatch
                       // == 0x8f, given the fields set above and every other
                       // RobotState field left at its default

  comms.updateStatus(state, tlm);

  serialFake.enqueueInbound("STATUS");
  Core::Cmd cmd = pumpOne(comms, /*now=*/0);
  checkTrue(cmd.status == Core::CmdStatus::kNone, "STATUS never decodes a Cmd");
  checkU64Eq(serialFake.sentReliable().size(), 1, "exactly one STATUS reply");
  if (!serialFake.sentReliable().empty()) {
    const std::string& line = serialFake.sentReliable()[0];
    checkTrue(line.find("ready=1") != std::string::npos, "ready=1 -- state.health.ready");
    checkTrue(line.find("active=1") != std::string::npos, "active=1 -- state.command.moveActive");
    checkTrue(line.find("connL=1") != std::string::npos, "connL=1 -- state.wheelLeft.connected");
    checkTrue(line.find("connR=0") != std::string::npos, "connR=0 -- state.wheelRight.connected");
    checkTrue(line.find("otos=1") != std::string::npos, "otos=1 -- state.otos.present");
    checkTrue(line.find("wedge=1") != std::string::npos, "wedge=1 -- state.health.wedgeLatch");
    checkTrue(line.find("flags=0x8f") != std::string::npos,
              "flags=0x8f -- tlm.flags(), derived by Telemetry::update() from the same state");
    checkTrue(line.find("tlm=on") != std::string::npos, "tlm=on -- tlm.mode(), kOn set above");
  }
}

void scenarioUpdateStatusReadyFalseBeforeBoot() {
  beginScenario("updateStatus(): ready=0 when state.health.ready is still its boot-time default -- STATUS "
                "never hard-codes ready=1 itself");

  FakeTransport serialFake;
  FakeTransport radioFake;
  FakeTransport tlmSerialFake;
  FakeTransport tlmRadioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  Core::Comms comms(serialFake, radioFake, banner);
  Core::Comms tlmComms(tlmSerialFake, tlmRadioFake, banner);
  Core::Telemetry tlm(tlmComms);

  Types::RobotState state;  // health.ready defaults to false (robot_state.h)
  tlm.update(state, testDrive());
  comms.updateStatus(state, tlm);

  serialFake.enqueueInbound("STATUS");
  Core::Cmd cmd = pumpOne(comms, /*now=*/0);
  checkTrue(cmd.status == Core::CmdStatus::kNone, "STATUS never decodes a Cmd");
  if (!serialFake.sentReliable().empty()) {
    checkTrue(serialFake.sentReliable()[0].find("ready=0") != std::string::npos,
               "ready=0 -- a default-constructed RobotState (pre-boot()) reports not-ready");
  }
}

// --- 133-003: the four live-tuning DBG arms ------------------------------
//
// These exercise classifyDbgArg() through the REAL wire path (an inbound
// `DBG:` line fed to pump()), not by calling the file-local classifier
// directly -- the classifier is in an anonymous namespace in comms.cpp and,
// more to the point, "the operator typed this and the robot staged that" is
// the property worth proving. What the STAGED action then DOES lives in
// RobotLoop::applyDbgAction() and is out of this harness's scope.
//
// Note that this harness compiles with -DHOST_BUILD, which implies
// ROBOT_DEBUG (app/debug.h's own header), so the whole DBG surface is live
// here. That is also exactly why a shipped-image proof cannot live in this
// file -- see test_app_comms.py's own non-debug-build test.

// takeOneDbg -- feed one cleartext line and return the single DbgAction it
// staged (kNone when it staged nothing).
Core::Comms::DbgAction takeOneDbg(const char* line) {
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  FakeTransport serialFake;
  FakeTransport radioFake;
  Core::Comms comms(serialFake, radioFake, banner);
  serialFake.enqueueInbound(line);
  comms.pump(/*now=*/0);
  return comms.takeDbgAction();
}

// nearlyEqual -- the operands are decimal text parsed to float, so an exact
// compare would be testing strtof(), not the parser arm. 1e-4 is far tighter
// than any value a tuning session pushes and far looser than float noise.
bool nearlyEqual(float actual, float expected) {
  const float difference = actual - expected;
  return (difference < 1e-4f) && (difference > -1e-4f);
}

void scenarioDbgTuningVerbsStageTheirOwnArmWithTheParsedValue() {
  beginScenario("pump(): DBG:vmin/asteady/pos each stage their own arm carrying the parsed value");

  struct Row {
    const char* line;
    Core::Comms::DbgActionKind expected;
    float value;
  };
  const Row rows[] = {
      {"DBG:vmin 60", Core::Comms::DbgActionKind::kVmin, 60.0f},
      {"DBG:vmin 0", Core::Comms::DbgActionKind::kVmin, 0.0f},
      {"DBG:vmin 99.7", Core::Comms::DbgActionKind::kVmin, 99.7f},
      {"DBG:asteady 250", Core::Comms::DbgActionKind::kASteady, 250.0f},
      {"DBG:asteady 1200.5", Core::Comms::DbgActionKind::kASteady, 1200.5f},
      {"DBG:pos 5", Core::Comms::DbgActionKind::kPos, 5.0f},
      {"DBG:pos 0", Core::Comms::DbgActionKind::kPos, 0.0f},
  };

  for (const Row& row : rows) {
    const Core::Comms::DbgAction action = takeOneDbg(row.line);
    checkTrue(action.kind == row.expected,
              std::string(row.line) + " stages the expected arm");
    checkTrue(nearlyEqual(action.value, row.value),
              std::string(row.line) + " carries the parsed operand in `value`");
  }
}

void scenarioDbgGainStagesBothMultipliersIndependently() {
  beginScenario("pump(): DBG:gain <L> <R> stages kGain with the two multipliers in value/value2");

  const Core::Comms::DbgAction action = takeOneDbg("DBG:gain 1.02 0.98");
  checkTrue(action.kind == Core::Comms::DbgActionKind::kGain, "DBG:gain -> kGain");
  checkTrue(nearlyEqual(action.value, 1.02f), "left multiplier lands in value");
  checkTrue(nearlyEqual(action.value2, 0.98f), "right multiplier lands in value2");

  // The two operands must not be transposed or shared -- an L/R imbalance
  // trim applied to the wrong wheel doubles the imbalance instead of
  // cancelling it, and reads as a much worse robot rather than as a bug.
  const Core::Comms::DbgAction identity = takeOneDbg("DBG:gain 1 1");
  checkTrue(identity.kind == Core::Comms::DbgActionKind::kGain, "DBG:gain 1 1 -> kGain");
  checkTrue(nearlyEqual(identity.value, 1.0f) && nearlyEqual(identity.value2, 1.0f),
            "DBG:gain 1 1 is the identity push");
}

void scenarioDbgTuningVerbsRejectMalformedOperands() {
  beginScenario("pump(): a tuning verb with a missing, non-numeric, or out-of-domain "
                "operand stages kUnrecognized -- never a silently-zero apply");

  const char* rejected[] = {
      // Missing operand entirely.
      "DBG:vmin", "DBG:asteady", "DBG:pos", "DBG:gain",
      // Non-numeric: strtof() would return 0.0 here, which must NOT be
      // mistaken for a deliberate "0" push -- 0 is a meaningful value for
      // vmin (floor off) and pos (unclamped), so the difference between
      // "operator asked for zero" and "operator typo'd" is real.
      "DBG:vmin oops", "DBG:pos nan", "DBG:asteady --",
      // Trailing junk after a valid number: "60mm" is a unit an operator
      // might reasonably type, and it must be refused rather than
      // truncated to 60.
      "DBG:vmin 60mm", "DBG:pos 5.0.0",
      // Out of domain: negative for the scalars.
      "DBG:vmin -1", "DBG:asteady -0.5", "DBG:pos -10",
      // gain needs BOTH operands, and neither may be <= 0 (a zero or
      // negative dutyPerSpeed is an uncalibrated or backwards robot).
      "DBG:gain 1.02", "DBG:gain 0 1", "DBG:gain 1 0", "DBG:gain -1 -1",
      "DBG:gain 1.02 nope",
  };

  for (const char* line : rejected) {
    const Core::Comms::DbgAction action = takeOneDbg(line);
    checkTrue(action.kind == Core::Comms::DbgActionKind::kUnrecognized,
              std::string(line) + " -> kUnrecognized");
  }
}

void scenarioDbgTuningVerbsAreNotCountedMalformed() {
  beginScenario("pump(): a DBG tuning line is a recognized verb -- neither a good one "
                "nor a rejected one bumps malformedCount()");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  FakeTransport serialFake;
  FakeTransport radioFake;
  Core::Comms comms(serialFake, radioFake, banner);

  serialFake.enqueueInbound("DBG:vmin 60");
  serialFake.enqueueInbound("DBG:gain 1.02 0.98");
  serialFake.enqueueInbound("DBG:vmin garbage");
  comms.pump(/*now=*/0);

  // kFaultCommsMalformed is a LINK-quality fault bit. A mistyped tuning
  // value is an operator error, not a corrupted wire, and raising the fault
  // for it would make the bench gate's own "malformed stays clear" check
  // fail for a reason that has nothing to do with the link.
  checkU64Eq(comms.malformedCount(), 0,
             "three DBG tuning lines, one of them rejected, leave malformedCount() at 0");
  checkTrue(comms.takeDbgAction().kind == Core::Comms::DbgActionKind::kVmin,
            "the ring preserves order: first line out first");
  checkTrue(comms.takeDbgAction().kind == Core::Comms::DbgActionKind::kGain,
            "second line out second");
  checkTrue(comms.takeDbgAction().kind == Core::Comms::DbgActionKind::kUnrecognized,
            "the rejected line is still STAGED (so it can be echoed back), just not applied");
}

void scenarioDbgFaultInjectionArmsStillParseUnchanged() {
  beginScenario("pump(): 129-era mark/ping/wedge/clear arms are unchanged by the "
                "tuning arms sharing their parser");

  checkTrue(takeOneDbg("DBG:mark leg1a").kind == Core::Comms::DbgActionKind::kMark,
            "DBG:mark still -> kMark");
  checkTrue(takeOneDbg("DBG:ping").kind == Core::Comms::DbgActionKind::kPing,
            "DBG:ping still -> kPing");
  checkTrue(takeOneDbg("DBG:clear").kind == Core::Comms::DbgActionKind::kClear,
            "DBG:clear still -> kClear");

  const Core::Comms::DbgAction wedge = takeOneDbg("DBG:wedge left 500");
  checkTrue(wedge.kind == Core::Comms::DbgActionKind::kWedge, "DBG:wedge still -> kWedge");
  checkU64Eq(wedge.port, 1, "wedge port survives");
  checkU64Eq(wedge.duration, 500, "wedge duration survives");

  // kMark's verbatim-echo contract: `text` must still hold the WHOLE
  // original payload, not just the tokens the classifier consumed.
  checkTrue(std::string(takeOneDbg("DBG:mark leg1a").text) == "mark leg1a",
            "kMark still preserves the full original payload verbatim");
}

}  // namespace

// ---------------------------------------------------------------------------
// Transport COLLECTION (platform/hardware/hal reorganization). Core::Comms
// used to hold two named Transport& slots -- `serialLink_` and `radioLink_`
// -- which is what blocked a third transport (WiFi) without a structural
// change. It now holds a fixed-size array registered at composition time.
//
// Three things have to hold for that to be a safe swap:
//   1. The two-argument convenience constructor still registers serial
//      first, then radio, so pump()'s tie-break precedence is unchanged.
//   2. A THIRD transport really works -- it gets pumped and it gets
//      broadcasts.
//   3. Registration past kMaxTransports fails LOUDLY (returns false) rather
//      than silently dropping a link.
// ---------------------------------------------------------------------------

void scenarioTwoArgConstructorRegistersSerialThenRadio() {
  beginScenario("Comms(serial, radio): registers exactly two, serial first");
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  FakeTransport serialFake;
  FakeTransport radioFake;
  Core::Comms comms(serialFake, radioFake, banner);

  checkTrue(comms.transportCount() == 2, "two transports registered");

  // Both have a line ready. pump() offers transports in registration
  // order, so the FIRST-registered (serial) is consumed first -- the same
  // precedence the two-named-slot pump() had.
  serialFake.enqueueInbound("PING");
  radioFake.enqueueInbound("PING");
  comms.pump(0);

  // Both replies land (pump loops until every transport is dry), and each
  // reply goes back out the transport its command arrived on.
  checkTrue(!serialFake.sentReliable().empty(),
            "serial PING answered on the serial transport");
  checkTrue(!radioFake.sentReliable().empty(),
            "radio PING answered on the radio transport");
}

void scenarioThirdTransportIsPumpedAndBroadcastTo() {
  beginScenario("addTransport(): a THIRD transport is pumped and broadcast to");
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  FakeTransport serialFake;
  FakeTransport radioFake;
  FakeTransport thirdFake;  // stands in for a future WifiTransport
  Core::Comms comms(serialFake, radioFake, banner);

  checkTrue(comms.addTransport(thirdFake), "a third transport registers");
  checkTrue(comms.transportCount() == 3, "three transports registered");

  // Inbound: a command arriving ONLY on the third transport is seen, and
  // its reply goes back out that same transport.
  thirdFake.enqueueInbound("PING");
  comms.pump(0);
  checkTrue(!thirdFake.sentReliable().empty(),
            "a PING on the third transport is pumped and answered there");
  checkTrue(serialFake.sentReliable().empty(),
            "the reply did NOT leak onto the serial transport");

  // Outbound broadcast: the banner goes to every registered transport.
  // FakeTransport has no reset, so compare counts before and after.
  const size_t serialBefore = serialFake.sentReliable().size();
  const size_t radioBefore = radioFake.sentReliable().size();
  const size_t thirdBefore = thirdFake.sentReliable().size();
  comms.sendBanner();
  checkTrue(serialFake.sentReliable().size() == serialBefore + 1,
            "banner reaches serial");
  checkTrue(radioFake.sentReliable().size() == radioBefore + 1,
            "banner reaches radio");
  checkTrue(thirdFake.sentReliable().size() == thirdBefore + 1,
            "banner reaches the third transport too -- a broadcast is to ALL");
}

void scenarioRegistrationBeyondCapacityFailsLoudly() {
  beginScenario("addTransport(): registration past kMaxTransports returns false");
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  FakeTransport t0, t1, t2, t3, overflow;
  Core::Comms comms(t0, t1, banner);
  checkTrue(comms.addTransport(t2), "third registers");
  checkTrue(comms.addTransport(t3), "fourth registers (kMaxTransports == 4)");
  checkTrue(!comms.addTransport(overflow),
            "a fifth is REFUSED -- a silently dropped link is the failure "
            "mode this return value exists to prevent");
  checkTrue(comms.transportCount() == 4, "count stays at the ceiling");
}

int main() {
  scenarioTwoArgConstructorRegistersSerialThenRadio();
  scenarioThirdTransportIsPumpedAndBroadcastTo();
  scenarioRegistrationBeyondCapacityFailsLoudly();
  scenarioMoveRoundTrip();
  scenarioMalformedUnrecognizedTextLineRejected();
  scenarioMalformedCobsFrameRejected();
  scenarioMalformedCrcMismatchRejected();
  scenarioMalformedCorruptProtobufRejected();
  scenarioHelloRepliesWithBannerViaSendReliable();
  scenarioPingRepliesOkPongViaSendReliable();
  // scenarioPumpBoundedToOneTransportPerCall();  // DEPRECATED-COMMAND-INGEST -- see its own #if 0 block
  scenarioSendReplyBroadcastsIdenticalLineOnBothTransports();
  scenarioSendReplyVerbNameTracksBodyKind();
  scenarioDecodeBinaryFrameRejectsMismatchedCommandScope();
  scenarioIdRepliesWithConfiguredIdentity();
  scenarioVerRepliesWithBuildVersion();
  scenarioStrayTrailingColonOnNoDataVerbHandledGracefully();
  scenarioTruncatedBinaryLineCountsMalformedNotCrash();
  scenarioDataContainingColonAndZeroRoundTripsCorrectly();
  scenarioRelayHandshakeChatterNeverCountsAsMalformed();
  scenarioRelayCarveOutIsNarrowAndDoesNotAffectSubsequentRealCommand();
  scenarioBareTlmAndTlmNowBothProduceKFrameNoModeChange();
  scenarioTlmModeTokensCaseInsensitiveProduceCorrectAction();
  scenarioTlmGarbageArgProducesUnrecognizedNotMalformed();
  scenarioTlmWithSpaceNotColonFallsThroughToMalformed();
  scenarioTlmModeChangeRepliesWithStatusLineOnOriginatingTransport();
  scenarioTlmGarbageRepliesWithHelpLineListingTlmArgumentGrammar();
  scenarioBareTlmProducesNoSendTlmReplyOutputOfItsOwn();
  scenarioStatusLineCarriesTlmFieldForEveryMode();
  scenarioStatusDefaultsTlmAutoBeforeAnySetStatusCall();
  scenarioHelpLineListsTlmArgumentGrammar();
  scenarioUpdateStatusProjectsAllEightFieldsFromSynthesizedState();
  scenarioUpdateStatusReadyFalseBeforeBoot();
  scenarioDbgTuningVerbsStageTheirOwnArmWithTheParsedValue();
  scenarioDbgGainStagesBothMultipliersIndependently();
  scenarioDbgTuningVerbsRejectMalformedOperands();
  scenarioDbgTuningVerbsAreNotCountedMalformed();
  scenarioDbgFaultInjectionArmsStillParseUnchanged();

  if (g_failureCount == 0) {
    std::printf("OK: all Core::Comms scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Core::Comms scenarios\n", g_failureCount);
  return 1;
}
