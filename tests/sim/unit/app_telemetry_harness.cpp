// app_telemetry_harness.cpp -- off-hardware acceptance harness for ticket
// 103-005 (SUC-005), App::Telemetry (source/app/telemetry.{h,cpp}). Proves:
// primary-frame assembly (all listed fields) via Comms::sendReply(), the
// depth-3 ack ring's survive-a-dropped-frame property, fault_bits/
// event_bits bit-0 reflection (I2CBus safety net / Deadman expiry's real
// call sites -- see telemetry.h's own bit-layout comment), TelemetrySecondary's
// own independently-armored line never coinciding with a primary send in
// the same emit() call, and realized emission cadence for both frame types.
// Also proves (ticket 104-004, scenario 8): a malformed inbound frame
// pumped through App::Comms sets App::kFaultCommsMalformed (bit 3) in the
// telemetry frame that follows, once a caller mirrors
// Comms::malformedCount() via setFault() the way main.cpp's own loop does.
//
// Mirrors app_comms_harness.cpp's exact shape: hand-rolled
// beginScenario/fail/checkTrue/checkStrEq assertion plumbing, PASS/FAIL
// printf, exit nonzero on failure. Compiled by test_app_telemetry.py with
// -DHOST_BUILD against telemetry.cpp, comms.cpp, wire.cpp, wire_runtime.cpp
// (no MicroBit.h anywhere in this graph).
//
// No decode(ReplyEnvelope) codec exists (firmware only ever ENCODES a
// ReplyEnvelope -- envelope.proto's own doc comment) -- exactly like
// app_comms_harness.cpp's own sendReply() proof, frame CONTENTS are
// verified by independently building the expected msg::Telemetry/
// msg::TelemetrySecondary, encoding + armoring it via the SAME
// msg::wire::encode()/WireRuntime::base64Encode() primitives Telemetry
// itself is built on, and comparing the resulting line byte-for-byte
// against what the FakeTransport actually received.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "app/comms.h"
#include "app/telemetry.h"
#include "messages/envelope.h"
#include "messages/wire.h"
#include "messages/wire_runtime.h"
#include "support/fake_transport.h"

namespace {

// --- Hand-rolled assertion plumbing (see app_comms_harness.cpp) ----------

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

void checkStrEq(const std::string& actual, const std::string& expected, const std::string& what) {
  if (actual != expected) {
    fail(what + " -- expected \"" + expected + "\", got \"" + actual + "\"");
  }
}

void checkInRange(uint64_t actual, uint64_t lo, uint64_t hi, const std::string& what) {
  if (actual < lo || actual > hi) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected in [%llu, %llu], got %llu", what.c_str(),
                  static_cast<unsigned long long>(lo), static_cast<unsigned long long>(hi),
                  static_cast<unsigned long long>(actual));
    fail(buf);
  }
}

// --- armor() -- same "*B<base64>" sequence Comms::sendReply()/
// Telemetry::emitSecondary() themselves perform, used here only to build
// scenario EXPECTATIONS independently of Telemetry's own send path. ------

std::string armor(const uint8_t* raw, size_t rawLen) {
  char b64[512] = {};
  size_t b64Len = 0;
  bool ok = WireRuntime::base64Encode(raw, rawLen, b64, sizeof(b64), &b64Len);
  if (!ok) return std::string();
  std::string out = "*B";
  out.append(b64, b64Len);
  return out;
}

std::string armorReply(const msg::ReplyEnvelope& env) {
  uint8_t rawBuf[App::kMaxEnvelopeBytes];
  uint16_t n = msg::wire::encode(env, rawBuf, sizeof(rawBuf));
  if (n == 0) return std::string();
  return armor(rawBuf, n);
}

// --- FakeTransport is TestSupport::FakeTransport
// (tests/sim/support/fake_transport.h, ticket 105-002) -- the ONE canonical
// scripted queue of inbound lines plus a log of every send()/sendReliable()
// call. This harness previously carried its own ad hoc FakeTransport
// (readLine() hardcoded to return false, since Telemetry never reads) AND a
// second, separately-named QueueableFakeTransport variant (needed only by
// scenario 8 below, which drives Comms::pump() to produce a real
// App::Comms::malformedCount() > 0). Both collapse into this one shared
// class: its readLine() already returns false whenever nothing was ever
// enqueued (every scenario except #8), and #8 now calls enqueueInbound()
// on the same type. ---------------------------------------------------

using TestSupport::FakeTransport;

// ===========================================================================
// 1. Primary frame assembly: emit() with a fully-populated Frame builds
//    exactly the fields AC #1 lists and sends via Comms::sendReply() (both
//    transports, matching Comms's own broadcast discipline).
// ===========================================================================

void scenarioPrimaryFrameAssemblyMatchesIndependentEncode() {
  beginScenario("emit(): primary frame carries every AC-listed field, sent via Comms::sendReply()");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms, serialFake, radioFake);

  App::Telemetry::Frame frame;
  frame.mode = msg::DriveMode::VELOCITY;
  frame.hasEnc = true;
  frame.encLeft = 12.5f;
  frame.encRight = -3.25f;
  frame.hasVel = true;
  frame.velLeft = 100.0f;
  frame.velRight = -50.0f;
  frame.hasPose = true;
  frame.pose = {1.0f, 2.0f, 0.5f};
  frame.hasOtos = true;
  frame.otos = {1.1f, 2.2f, 0.6f};
  frame.otosConnected = true;
  frame.hasTwist = true;
  frame.twist = {150.0f, 0.0f, 0.75f};
  frame.active = true;
  frame.connLeft = true;
  frame.connRight = true;
  telemetry.setFrame(frame);

  telemetry.ack(7, msg::AckStatus::ACK_STATUS_OK, 0);

  telemetry.emit(1234);  // first call -- always sends primary (boot, no arming)

  checkU64Eq(serialFake.sent().size(), 1, "exactly one serial send() for the primary frame");
  checkU64Eq(radioFake.sent().size(), 1, "exactly one radio send() for the primary frame");
  checkU64Eq(serialFake.sentReliable().size(), 0, "primary frame never uses sendReliable()");

  msg::Telemetry expected;
  expected.acks_[0] = {7, msg::AckStatus::ACK_STATUS_OK, 0};
  expected.acks_count = 1;
  expected.now = 1234;
  expected.mode = msg::DriveMode::VELOCITY;
  expected.seq = 0;
  expected.has_enc = true;
  expected.enc_left = 12.5f;
  expected.enc_right = -3.25f;
  expected.has_vel = true;
  expected.vel_left = 100.0f;
  expected.vel_right = -50.0f;
  expected.has_pose = true;
  expected.pose = {1.0f, 2.0f, 0.5f};
  expected.has_otos = true;
  expected.otos = {1.1f, 2.2f, 0.6f};
  expected.otos_connected = true;
  expected.has_twist = true;
  expected.twist = {150.0f, 0.0f, 0.75f};
  expected.active = true;
  expected.conn_left = true;
  expected.conn_right = true;

  msg::ReplyEnvelope env;
  env.corr_id = 0;
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = expected;
  std::string expectedLine = armorReply(env);
  checkTrue(!expectedLine.empty(), "independent encode+armor of the expected frame succeeds");

  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expectedLine, "sent line matches an independent re-encode+armor");
  }
  if (!serialFake.sent().empty() && !radioFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], radioFake.sent()[0], "serial and radio received byte-identical lines");
  }
}

// ===========================================================================
// 2. Ack ring survives a dropped frame: push 4 sequential acks (ring
//    evicts the oldest), build 2 successive primary frames (the FIRST is
//    simulated "lost" -- its content is never inspected), confirm the
//    newest 3 acks (2,3,4) are present in the SECOND (the one "read").
// ===========================================================================

void scenarioAckRingSurvivesADroppedFrame() {
  beginScenario("ack ring: newest 3 acks survive into the frame read AFTER a simulated dropped frame");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms, serialFake, radioFake);

  telemetry.ack(1, msg::AckStatus::ACK_STATUS_OK, 0);
  telemetry.ack(2, msg::AckStatus::ACK_STATUS_OK, 0);
  telemetry.ack(3, msg::AckStatus::ACK_STATUS_ERR, 5);
  telemetry.ack(4, msg::AckStatus::ACK_STATUS_OK, 0);

  telemetry.emit(0);   // frame #1 -- the "lost" frame; not inspected below
  telemetry.emit(40);  // frame #2 -- the frame that IS read

  checkU64Eq(serialFake.sent().size(), 2, "two successive primary frames were sent");

  msg::Telemetry expected;
  expected.acks_[0] = {2, msg::AckStatus::ACK_STATUS_OK, 0};
  expected.acks_[1] = {3, msg::AckStatus::ACK_STATUS_ERR, 5};
  expected.acks_[2] = {4, msg::AckStatus::ACK_STATUS_OK, 0};
  expected.acks_count = 3;
  expected.now = 40;
  expected.seq = 1;  // frame #1 took seq 0

  msg::ReplyEnvelope env;
  env.corr_id = 0;
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = expected;
  std::string expectedLine = armorReply(env);
  checkTrue(!expectedLine.empty(), "independent encode+armor of the expected second frame succeeds");

  if (serialFake.sent().size() == 2) {
    checkStrEq(serialFake.sent()[1], expectedLine,
               "the SECOND (read) frame carries the newest 3 acks -- the first frame's own loss cost nothing");
  }
}

// ===========================================================================
// 3. Ack ring depth: pushing fewer than 3 acks reports acks_count
//    accurately (no phantom zero-valued entries claimed as real).
// ===========================================================================

void scenarioAckRingBelowCapacityReportsExactCount() {
  beginScenario("ack ring: fewer than 3 pushed acks -> acks_count reflects exactly how many, no phantom entries");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms, serialFake, radioFake);

  telemetry.ack(42, msg::AckStatus::ACK_STATUS_OK, 0);
  telemetry.emit(0);

  msg::Telemetry expected;
  expected.acks_[0] = {42, msg::AckStatus::ACK_STATUS_OK, 0};
  expected.acks_count = 1;
  expected.now = 0;
  expected.seq = 0;

  msg::ReplyEnvelope env;
  env.corr_id = 0;
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = expected;
  std::string expectedLine = armorReply(env);

  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expectedLine, "acks_count == 1, no phantom second/third entry emitted");
  }
}

// ===========================================================================
// 4. fault_bits/event_bits: the two AC-minimum bits (I2CBus safety-net
//    trip, Deadman expiry) flip when the value a real call site would
//    produce is fed in via setFault()/setEvent(), and clear again when the
//    condition clears -- level-set, not a sticky latch.
// ===========================================================================

void scenarioFaultAndEventBitsReflectRealCallSiteValues() {
  beginScenario("setFault(kFaultI2CSafetyNet, ...) / setEvent(kEventDeadmanExpired, ...) flip the wire bits");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms, serialFake, radioFake);

  // Simulates ticket 008's own future call site:
  // telemetry.setFault(App::kFaultI2CSafetyNet, i2cBus.clearanceSafetyNetCount() > 0);
  telemetry.setFault(App::kFaultI2CSafetyNet, true);
  // Simulates: telemetry.setEvent(App::kEventDeadmanExpired, deadman.expired());
  telemetry.setEvent(App::kEventDeadmanExpired, true);
  checkU64Eq(telemetry.faultBits(), App::kFaultI2CSafetyNet, "faultBits() reflects the I2C safety-net bit");
  checkU64Eq(telemetry.eventBits(), App::kEventDeadmanExpired, "eventBits() reflects the deadman-expired bit");

  telemetry.emit(0);

  msg::Telemetry expectedSet;
  expectedSet.now = 0;
  expectedSet.seq = 0;
  expectedSet.fault_bits = App::kFaultI2CSafetyNet;
  expectedSet.event_bits = App::kEventDeadmanExpired;
  msg::ReplyEnvelope envSet;
  envSet.corr_id = 0;
  envSet.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  envSet.body.tlm = expectedSet;
  std::string expectedSetLine = armorReply(envSet);
  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expectedSetLine, "first frame carries both bits set");
  }

  // Condition clears -- the caller mirrors that too, and the NEXT frame
  // clears the bit (level-set, no sticky latch invented by Telemetry).
  telemetry.setFault(App::kFaultI2CSafetyNet, false);
  telemetry.setEvent(App::kEventDeadmanExpired, false);
  checkU64Eq(telemetry.faultBits(), 0, "faultBits() clears once the caller reports the condition cleared");
  checkU64Eq(telemetry.eventBits(), 0, "eventBits() clears once the caller reports the condition cleared");

  telemetry.emit(40);

  msg::Telemetry expectedClear;
  expectedClear.now = 40;
  expectedClear.seq = 1;
  msg::ReplyEnvelope envClear;
  envClear.corr_id = 0;
  envClear.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  envClear.body.tlm = expectedClear;
  std::string expectedClearLine = armorReply(envClear);
  if (serialFake.sent().size() == 2) {
    checkStrEq(serialFake.sent()[1], expectedClearLine, "second frame carries both bits cleared");
  }
}

// ===========================================================================
// 5. TelemetrySecondary: rides its own independently-armored line (NOT a
//    ReplyEnvelope), never in the same emit() call as a primary send, and
//    does not delay the primary frame's own cadence.
// ===========================================================================

void scenarioSecondaryNeverCoincidesWithPrimaryAndDoesNotDelayIt() {
  beginScenario("emit(): TelemetrySecondary rides its own line, never the same call as primary, never delays it");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms, serialFake, radioFake);

  App::Telemetry::SecondaryFrame sec;
  sec.hasCmdVel = true;
  sec.cmdVelLeft = 90.0f;
  sec.cmdVelRight = -90.0f;
  sec.accLeft = 3.5f;
  sec.accRight = -1.5f;
  sec.glitchLeft = 2;
  sec.glitchRight = 0;
  sec.tsLeft = 100;
  sec.tsRight = 100;
  telemetry.setSecondaryFrame(sec);

  // Drive a fine-grained clock (7 ms/step, NOT a divisor of kPrimaryPeriod
  // or kSecondaryPeriod -- realistic quantization jitter, not a suspiciously
  // exact multiple) across several seconds, well above the loop's assumed
  // per-cycle rate (this file's own telemetry.h scheduling note).
  const uint32_t kStep = 7;
  const uint32_t kEndTime = 3000;
  uint32_t lastPrimarySeen = 0;
  bool sawPrimary = false;
  uint32_t maxPrimaryGap = 0;

  for (uint32_t now = 0; now <= kEndTime; now += kStep) {
    uint32_t beforePrimary = telemetry.primaryEmitCount();
    uint32_t beforeSecondary = telemetry.secondaryEmitCount();
    telemetry.emit(now);
    uint32_t primarySent = telemetry.primaryEmitCount() - beforePrimary;
    uint32_t secondarySent = telemetry.secondaryEmitCount() - beforeSecondary;

    checkTrue(!(primarySent > 0 && secondarySent > 0), "a single emit() call never sends both frame types");

    if (primarySent > 0) {
      if (sawPrimary) {
        uint32_t gap = now - lastPrimarySeen;
        if (gap > maxPrimaryGap) maxPrimaryGap = gap;
      }
      lastPrimarySeen = now;
      sawPrimary = true;
    }
  }

  checkTrue(telemetry.primaryEmitCount() > 0, "at least one primary frame was sent");
  checkTrue(telemetry.secondaryEmitCount() > 0, "at least one secondary frame was sent -- it is not starved");

  // Every send() call (both transports, both frame types) accounted for
  // exactly -- no call produced an untracked extra line.
  checkU64Eq(serialFake.sent().size(), telemetry.primaryEmitCount() + telemetry.secondaryEmitCount(),
             "serial send() log size == primary + secondary emit counts (no untracked sends)");

  // Primary cadence never stretched past kPrimaryPeriod + one scheduling
  // step by a competing secondary send -- "does not starve or delay the
  // primary frame's cadence" (AC #4).
  checkTrue(maxPrimaryGap <= App::kPrimaryPeriod + kStep,
            "no observed primary-to-primary gap exceeds kPrimaryPeriod by more than one scheduling step");

  // Expected counts, +/-1 for boundary quantization: primary roughly every
  // kPrimaryPeriod, secondary roughly every kSecondaryPeriod.
  // Quantized period: emit() only samples at kStep granularity, so the
  // REALIZED period is rounded up to the next kStep multiple (e.g. 40 ms
  // at a 7 ms step lands on 42 ms, not 40) -- expected counts are derived
  // from that realized period, not the nominal constant, to avoid a false
  // failure from the sampling granularity itself.
  auto quantizedPeriod = [](uint32_t period) { return ((period + kStep - 1) / kStep) * kStep; };
  uint64_t expectedPrimary = kEndTime / quantizedPeriod(App::kPrimaryPeriod);
  uint64_t expectedSecondary = kEndTime / quantizedPeriod(App::kSecondaryPeriod);
  checkInRange(telemetry.primaryEmitCount(), expectedPrimary - 2, expectedPrimary + 2,
               "primary emit count is close to kEndTime/(quantized kPrimaryPeriod)");
  checkInRange(telemetry.secondaryEmitCount(), expectedSecondary - 2, expectedSecondary + 2,
               "secondary emit count is close to kEndTime/(quantized kSecondaryPeriod)");

  // Confirm the FIRST secondary line sent is armored but is NOT a valid
  // ReplyEnvelope-shaped line by construction -- it independently
  // re-encodes to the exact bytes msg::wire::encode(TelemetrySecondary)
  // would produce, proving it rode its own top-level payload, not a `tlm`
  // oneof arm.
  // Verify the LAST secondary send specifically, using lastSecondaryEmit()
  // as its known timestamp -- exact enough to prove the encode path
  // without needing to track every intermediate secondary `now`.
  msg::TelemetrySecondary expectedSec;
  expectedSec.now = telemetry.lastSecondaryEmit();
  expectedSec.has_cmd_vel = true;
  expectedSec.cmd_vel_left = 90.0f;
  expectedSec.cmd_vel_right = -90.0f;
  expectedSec.acc_left = 3.5f;
  expectedSec.acc_right = -1.5f;
  expectedSec.glitch_left = 2;
  expectedSec.glitch_right = 0;
  expectedSec.ts_left = 100;
  expectedSec.ts_right = 100;

  uint8_t rawBuf[msg::wire::kTelemetrySecondaryMaxEncodedSize];
  uint16_t n = msg::wire::encode(expectedSec, rawBuf, sizeof(rawBuf));
  checkTrue(n > 0, "independent encode(TelemetrySecondary) succeeds");
  std::string expectedLine = armor(rawBuf, n);
  checkTrue(!expectedLine.empty(), "independent armor() of the secondary frame succeeds");

  bool found = false;
  for (const auto& line : serialFake.sent()) {
    if (line == expectedLine) {
      found = true;
      break;
    }
  }
  checkTrue(found, "the last secondary send's line matches an independent encode(TelemetrySecondary)+armor");
}

// ===========================================================================
// 6. Frame-size: a fully-populated primary frame (all has_* true, ring at
//    full depth 3) encodes at or under ticket 001's recorded 179 B
//    worst case for ReplyEnvelope{tlm} at ring depth 3.
// ===========================================================================

void scenarioFullyPopulatedPrimaryFrameFitsRecordedWorstCase() {
  beginScenario("a fully-populated primary frame's encoded size is <= ticket 001's recorded 179 B worst case");

  msg::Telemetry tlm;
  tlm.acks_[0] = {1, msg::AckStatus::ACK_STATUS_OK, 0};
  tlm.acks_[1] = {2, msg::AckStatus::ACK_STATUS_ERR, 7};
  tlm.acks_[2] = {3, msg::AckStatus::ACK_STATUS_OK, 0};
  tlm.acks_count = 3;
  tlm.now = 0xFFFFFFFFu;
  tlm.mode = msg::DriveMode::GO_TO;
  tlm.seq = 0xFFFFFFFFu;
  tlm.has_enc = true;
  tlm.enc_left = -1234.5f;
  tlm.enc_right = 6789.25f;
  tlm.has_vel = true;
  tlm.vel_left = -500.0f;
  tlm.vel_right = 500.0f;
  tlm.has_pose = true;
  tlm.pose = {1234.5f, -6789.25f, 3.14159f};
  tlm.has_otos = true;
  tlm.otos = {1234.5f, -6789.25f, 3.14159f};
  tlm.otos_connected = true;
  tlm.has_twist = true;
  tlm.twist = {150.0f, -0.5f, 0.75f};
  tlm.active = true;
  tlm.conn_left = true;
  tlm.conn_right = true;
  tlm.fault_bits = 0xFFFFFFFFu;
  tlm.event_bits = 0xFFFFFFFFu;

  msg::ReplyEnvelope env;
  env.corr_id = 0xFFFFFFFFu;
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = tlm;

  uint8_t rawBuf[App::kMaxEnvelopeBytes];
  uint16_t n = msg::wire::encode(env, rawBuf, sizeof(rawBuf));
  checkTrue(n > 0, "encode() succeeds for a fully-populated frame");
  checkTrue(n <= msg::wire::kReplyEnvelopeMaxEncodedSize,
            "encoded size fits ticket 001's recorded 179 B worst case for ReplyEnvelope{tlm}, ring depth 3");
  std::printf("  measured: fully-populated primary frame encodes to %u bytes (worst case %u, budget 186)\n",
              static_cast<unsigned>(n), static_cast<unsigned>(msg::wire::kReplyEnvelopeMaxEncodedSize));
}

// ===========================================================================
// 7. Measured cadence report -- this ticket's own acceptance criterion:
//    report the REAL realized cadence (not assumed to match spike-001's
//    pre-rewrite baseline) against the 25 Hz/40 ms target.
// ===========================================================================

void scenarioMeasuredCadenceReport() {
  beginScenario("measured emission cadence (both frame types) vs. the 25 Hz/40 ms target");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms, serialFake, radioFake);

  const uint32_t kStep = 3;  // [ms] fine-grained relative to kPrimaryPeriod=40/kSecondaryPeriod=200
  const uint32_t kEndTime = 10000;
  for (uint32_t now = 0; now <= kEndTime; now += kStep) {
    telemetry.emit(now);
  }

  double primaryHz = static_cast<double>(telemetry.primaryEmitCount()) / (static_cast<double>(kEndTime) / 1000.0);
  double secondaryHz = static_cast<double>(telemetry.secondaryEmitCount()) / (static_cast<double>(kEndTime) / 1000.0);
  std::printf("  measured: primary %.2f Hz (target 25 Hz/40 ms), secondary %.2f Hz (target ~5 Hz/200 ms) over %u ms\n",
              primaryHz, secondaryHz, static_cast<unsigned>(kEndTime));

  // Not required to HIT 25 Hz exactly (ticket's own acceptance criterion)
  // -- only sane and in the right neighborhood for a deterministic
  // scripted-clock host test.
  checkTrue(primaryHz > 15.0 && primaryHz < 35.0, "measured primary Hz is in a sane neighborhood of the 25 Hz target");
  checkTrue(secondaryHz > 2.0 && secondaryHz < 8.0, "measured secondary Hz is in a sane neighborhood of the ~5 Hz target");
}

// ===========================================================================
// 8. kFaultCommsMalformed (104-004, bit 3): a malformed/undecodable inbound
//    frame pumped through the SAME App::Comms instance Telemetry's own
//    Comms::sendReply() rides -- App::Comms::malformedCount() rising above
//    0 -- sets App::kFaultCommsMalformed in the NEXT telemetry frame, once
//    a caller mirrors it via setFault() exactly the way main.cpp's own loop
//    does (App::Telemetry::setFault(App::kFaultCommsMalformed,
//    comms.malformedCount() > 0)). Bit clears on a later frame if the
//    caller stops reporting it -- same level-set discipline as scenario 4.
// ===========================================================================

void scenarioMalformedFrameSetsCommsMalformedFaultBit() {
  beginScenario("malformed frame -> Comms::malformedCount() -> setFault(kFaultCommsMalformed) sets the wire bit");

  FakeTransport serialFake;
  FakeTransport radioFake;
  // Bad armor prefix -- same malformed input app_comms_harness.cpp's own
  // scenarioMalformedArmorPrefixRejected() uses to increment
  // malformedCount() by exactly 1.
  serialFake.enqueueInbound("*Xsomeunrecognizedarmor");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms, serialFake, radioFake);

  App::Cmd cmd;
  comms.pump(cmd);
  checkU64Eq(comms.malformedCount(), 1, "malformedCount() incremented by the malformed line");

  // Mirrors main.cpp's own call site exactly (source/main.cpp):
  // tlm.setFault(App::kFaultCommsMalformed, comms.malformedCount() > 0);
  telemetry.setFault(App::kFaultCommsMalformed, comms.malformedCount() > 0);
  checkU64Eq(telemetry.faultBits(), App::kFaultCommsMalformed, "faultBits() reflects kFaultCommsMalformed");

  telemetry.emit(0);

  msg::Telemetry expectedSet;
  expectedSet.now = 0;
  expectedSet.seq = 0;
  expectedSet.fault_bits = App::kFaultCommsMalformed;
  msg::ReplyEnvelope envSet;
  envSet.corr_id = 0;
  envSet.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  envSet.body.tlm = expectedSet;
  std::string expectedSetLine = armorReply(envSet);
  checkTrue(!expectedSetLine.empty(), "independent encode+armor of the expected frame succeeds");
  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expectedSetLine, "the frame AFTER the malformed pump() carries the bit set");
  }

  // No further malformed input arrives -- the caller mirrors that too
  // (malformedCount() is monotonic and never clears on its own, so a real
  // main.cpp call site would keep this bit latched; this half of the
  // scenario only proves Telemetry's own level-set discipline, matching
  // scenario 4's fault/event bit treatment).
  telemetry.setFault(App::kFaultCommsMalformed, false);
  checkU64Eq(telemetry.faultBits(), 0, "faultBits() clears once the caller reports the condition cleared");

  telemetry.emit(40);

  msg::Telemetry expectedClear;
  expectedClear.now = 40;
  expectedClear.seq = 1;
  msg::ReplyEnvelope envClear;
  envClear.corr_id = 0;
  envClear.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  envClear.body.tlm = expectedClear;
  std::string expectedClearLine = armorReply(envClear);
  if (serialFake.sent().size() == 2) {
    checkStrEq(serialFake.sent()[1], expectedClearLine, "second frame carries the bit cleared");
  }
}

}  // namespace

int main() {
  scenarioPrimaryFrameAssemblyMatchesIndependentEncode();
  scenarioAckRingSurvivesADroppedFrame();
  scenarioAckRingBelowCapacityReportsExactCount();
  scenarioFaultAndEventBitsReflectRealCallSiteValues();
  scenarioSecondaryNeverCoincidesWithPrimaryAndDoesNotDelayIt();
  scenarioFullyPopulatedPrimaryFrameFitsRecordedWorstCase();
  scenarioMeasuredCadenceReport();
  scenarioMalformedFrameSetsCommsMalformedFaultBit();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Telemetry scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Telemetry scenarios\n", g_failureCount);
  return 1;
}
