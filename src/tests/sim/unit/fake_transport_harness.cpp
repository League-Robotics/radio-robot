// fake_transport_harness.cpp -- off-hardware acceptance harness for ticket
// 105-002 (SUC-019), TestSupport::FakeTransport
// (src/tests/sim/support/fake_transport.h). Proves the primitive ITSELF, in
// isolation from App::Comms/App::Telemetry: readLine() is non-blocking and
// returns false the instant the inbound FIFO is empty, enqueueInbound()/
// enqueueInboundBinary() lines drain in FIFO order at most one per
// readLine() call -- 124-005: no more per-entry FrameKind tag, since
// App::Transport itself no longer distinguishes text/binary at all (see
// comms.h's own file header) -- arbitrary (including realistic COBS+CRC-
// framed, 123-002/124-005) content survives the FIFO round trip
// byte-for-byte, and send()/sendReliable() are two genuinely separate
// captures.
//
// The Comms::pump()/Telemetry::emit() INTEGRATION proofs required by this
// ticket's own SUC-019 acceptance criteria (a real COBS+CRC-framed MOVE
// CommandEnvelope decoded via pump(), a real ReplyEnvelope/Telemetry frame
// captured via emit()) are NOT duplicated here -- they already exist as
// app_comms_harness.cpp's scenarioMoveRoundTrip() and
// app_telemetry_harness.cpp's scenarioPrimaryFrameAssemblyMatchesIndependent
// Encode(), both of which this ticket migrated onto this SAME canonical
// FakeTransport (see fake_transport.h's own header comment). Re-proving
// those round trips a third time here would be pure duplication of exactly
// what this ticket's own dedup mandate says not to do.
//
// Mirrors app_comms_harness.cpp's exact shape: hand-rolled
// beginScenario/fail/checkTrue/checkStrEq assertion plumbing, PASS/FAIL
// printf, exit nonzero on failure. Compiled by test_fake_transport.py with
// -DHOST_BUILD -- fake_transport.h itself pulls in app/comms.h only for the
// App::Transport base class, so no MicroBit.h anywhere in this graph.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "app/comms.h"
#include "support/fake_transport.h"

namespace {

// --- Hand-rolled assertion plumbing (see app_comms_harness.cpp). ---------

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

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkU64Eq(uint64_t actual, uint64_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %llu, got %llu", what.c_str(),
                  static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
    fail(buf);
  }
}

void checkStrEq(const std::string& actual, const std::string& expected, const std::string& what) {
  if (actual != expected) {
    fail(what + " -- expected \"" + expected + "\", got \"" + actual + "\"");
  }
}

// ===========================================================================
// 1. readLine() on a never-populated FakeTransport returns false
//    immediately -- never blocks, matches Transport::readLine()'s
//    documented non-blocking contract.
// ===========================================================================

void scenarioReadLineReturnsNoneWhenEmpty() {
  beginScenario("readLine(): returns false when the inbound FIFO is empty");

  TestSupport::FakeTransport fake;
  char buf[64] = {};
  uint16_t outLen = 0;
  bool got = fake.readLine(buf, sizeof(buf), &outLen);

  checkFalse(got, "readLine() on a never-populated FakeTransport");
  checkU64Eq(fake.inboundSize(), 0, "inboundSize() stays 0");
}

// ===========================================================================
// 2. enqueueInbound() lines drain in FIFO order, at most one per
//    readLine() call -- matches Comms::pump()'s own "at most one line per
//    call" contract. 124-005: no more per-entry kind tag (App::Transport
//    itself no longer distinguishes text/binary -- see comms.h's own file
//    header).
// ===========================================================================

void scenarioEnqueueInboundDrainsInFifoOrderOnePerCall() {
  beginScenario("enqueueInbound(): lines drain in FIFO order, one per readLine() call");

  TestSupport::FakeTransport fake;
  fake.enqueueInbound("first");
  fake.enqueueInbound("second");
  fake.enqueueInbound("third");
  checkU64Eq(fake.inboundSize(), 3, "inboundSize() reflects all three queued lines");

  char buf[64] = {};
  uint16_t outLen = 0;

  bool got = fake.readLine(buf, sizeof(buf), &outLen);
  checkTrue(got, "first readLine() call succeeds");
  checkStrEq(buf, "first", "first readLine() call returns the oldest queued line");
  checkU64Eq(fake.inboundSize(), 2, "inboundSize() drops to 2 after one drain");

  got = fake.readLine(buf, sizeof(buf), &outLen);
  checkTrue(got, "second readLine() call succeeds");
  checkStrEq(buf, "second", "second readLine() call returns the next-oldest queued line");

  got = fake.readLine(buf, sizeof(buf), &outLen);
  checkTrue(got, "third readLine() call succeeds");
  checkStrEq(buf, "third", "third readLine() call returns the last queued line");
  checkU64Eq(fake.inboundSize(), 0, "inboundSize() is 0 once every queued line is drained");

  got = fake.readLine(buf, sizeof(buf), &outLen);
  checkFalse(got, "a fourth readLine() call returns false -- queue exhausted");
}

// ===========================================================================
// 3. A realistic COBS+CRC-framed binary line (123-002/124-005 -- was "*B..."
//    base64 text pre-123), INCLUDING arbitrary non-ASCII/non-printable byte
//    values (a literal 0x00 included -- 124-005: COBS is keyed on 0x0A now,
//    not 0x00, so a real armored line may legitimately contain one),
//    survives the FIFO round trip byte-for-byte -- FakeTransport never
//    mutates or truncates the content a test enqueues (the ONLY thing later
//    tickets, e.g. 004's sim_api, can rely on to script real framed input).
// ===========================================================================

void scenarioBinaryFrameSurvivesRoundTripByteForByte() {
  beginScenario("enqueueInboundBinary()/readLine(): a binary line round-trips byte-for-byte");

  TestSupport::FakeTransport fake;
  // Deliberately includes bytes >= 0x80, a mid-range control byte, and a
  // literal 0x00 -- proves the FIFO is a genuine byte-buffer round trip
  // (explicit length, never strlen()-recovered), not a text-line
  // assumption in disguise.
  const uint8_t frame[] = {0x01, 0xAB, 0xCD, 0x00, 0x0D, 0xFF, 0x7F, 0x02};
  fake.enqueueInboundBinary(frame, sizeof(frame));
  checkU64Eq(fake.inboundSize(), 1, "inboundSize() reflects the one queued binary line");

  char buf[64] = {};
  uint16_t outLen = 0;
  bool got = fake.readLine(buf, sizeof(buf), &outLen);
  checkTrue(got, "readLine() succeeds for the binary line");
  checkU64Eq(outLen, sizeof(frame), "outLen reports the exact byte count, including the embedded 0x00");
  if (outLen == sizeof(frame)) {
    checkTrue(std::memcmp(buf, frame, sizeof(frame)) == 0, "the binary line comes back byte-for-byte unchanged");
  }
}

// ===========================================================================
// 4. send() and sendReliable() are two genuinely separate captures --
//    matches Transport::send()'s drop-on-full vs. sendReliable()'s
//    must-not-drop semantics being distinct call sites in production code
//    (Telemetry rides send(); HELLO/PING text-plane replies ride
//    sendReliable()). send() takes an explicit uint8_t*/len (123-002 --
//    was a NUL-terminated C string pre-123, since a COBS+CRC frame is not
//    a text string even though it happens to be 0x00-free).
// ===========================================================================

void scenarioSendAndSendReliableAreSeparateCaptures() {
  beginScenario("send()/sendReliable(): captured in two separate, non-interfering logs");

  TestSupport::FakeTransport fake;
  fake.send(reinterpret_cast<const uint8_t*>("async-one"), 9);
  fake.send(reinterpret_cast<const uint8_t*>("async-two"), 9);
  fake.sendReliable("reliable-one");

  checkU64Eq(fake.sent().size(), 2, "sent() captured both send() calls");
  checkU64Eq(fake.sentReliable().size(), 1, "sentReliable() captured exactly the one sendReliable() call");
  if (fake.sent().size() == 2) {
    checkStrEq(fake.sent()[0], "async-one", "sent()[0] is the first send() call, in order");
    checkStrEq(fake.sent()[1], "async-two", "sent()[1] is the second send() call, in order");
  }
  if (!fake.sentReliable().empty()) {
    checkStrEq(fake.sentReliable()[0], "reliable-one", "sentReliable()[0] is the sendReliable() call");
  }
}

}  // namespace

int main() {
  scenarioReadLineReturnsNoneWhenEmpty();
  scenarioEnqueueInboundDrainsInFifoOrderOnePerCall();
  scenarioBinaryFrameSurvivesRoundTripByteForByte();
  scenarioSendAndSendReliableAreSeparateCaptures();

  if (g_failureCount == 0) {
    std::printf("OK: all FakeTransport scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the FakeTransport scenarios\n", g_failureCount);
  return 1;
}
