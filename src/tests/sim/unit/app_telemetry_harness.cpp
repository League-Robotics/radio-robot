// app_telemetry_harness.cpp -- off-hardware acceptance harness for ticket
// 103-005 (SUC-005), App::Telemetry (src/firm/app/telemetry.{h,cpp}). Proves:
// primary-frame assembly (all listed fields) via Comms::sendReply(), the
// single ack slot's overwrite-on-collision behavior and one-shot ack_fresh
// bit, the unified `flags` bit-string's status/fault/event bits (via
// setFlag() -- see telemetry.h's own bit-layout comment), TelemetrySecondary's
// own independently-armored line never coinciding with a primary send in
// the same emit() call, and realized emission cadence for both frame types.
// Also proves (originally ticket 104-004, scenario 8): a malformed inbound
// frame pumped through App::Comms sets App::kFlagFaultCommsMalformed in the
// telemetry frame that follows, once a caller mirrors
// Comms::malformedCount() via setFlag() the way RobotLoop::updateTlm() does.
//
// 115-005 (gut S1) rewrite: the depth-3 ack ring became a single ack slot
// (ack-depth-1 is a stakeholder-accepted tradeoff -- see telemetry.h's own
// ack()/kFlagAckFresh doc comments); the separate fault_bits/event_bits
// bitmasks and nine has_*/status bools folded into one `flags` word;
// Frame's flat enc_left/vel_left/bare-Pose2D-otos fields became timestamped
// EncoderReading/OtosReading objects; primary cadence changed from 40ms to
// 20ms. Every scenario below is updated for the new shapes; the old ack-RING
// scenario (push 4, evict oldest, newest-3-survive-a-dropped-frame) has no
// equivalent under depth-1 and is replaced by a single-slot-overwrite +
// one-shot-ack_fresh scenario.
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
// (src/tests/sim/support/fake_transport.h, ticket 105-002) -- the ONE canonical
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
//    transports, matching Comms's own broadcast discipline). Also proves
//    ack_fresh (flags bit 5) is set the SAME frame an ack() call precedes.
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
  frame.encLeft = {12.5f, 100.0f, 111};
  frame.encRight = {-3.25f, -50.0f, 111};
  frame.otos = {1.1f, 2.2f, 0.6f, 10.0f, -5.0f, 0.2f, 120};
  frame.otosPresent = true;
  frame.otosConnected = true;
  frame.pose = {1.0f, 2.0f, 0.5f};
  frame.twist = {150.0f, 0.0f, 0.75f};
  frame.line = 0x04030201u;
  frame.linePresent = true;
  frame.color = 0x0A090807u;
  frame.colorPresent = true;
  telemetry.setFrame(frame);

  // Frame.otosPresent/otosConnected/linePresent/colorPresent are STAGING
  // fields only -- Telemetry never reads them itself to derive flags bits
  // (see telemetry.h's own boundary comment: "callers... report
  // status/fault/event conditions via setFlag()"). RobotLoop::updateTlm()/
  // updateLineColor() are the real call sites that translate these into
  // setFlag() calls; this scenario mirrors that translation explicitly.
  telemetry.setFlag(App::kFlagOtosPresent, frame.otosPresent);
  telemetry.setFlag(App::kFlagOtosConnected, frame.otosConnected);
  telemetry.setFlag(App::kFlagLinePresent, frame.linePresent);
  telemetry.setFlag(App::kFlagColorPresent, frame.colorPresent);
  telemetry.setFlag(App::kFlagActive, true);
  telemetry.setFlag(App::kFlagConnLeft, true);
  telemetry.setFlag(App::kFlagConnRight, true);

  telemetry.ack(7, 0);

  telemetry.emit(1234);  // first call -- always sends primary (boot, no arming)

  checkU64Eq(serialFake.sent().size(), 1, "exactly one serial send() for the primary frame");
  checkU64Eq(radioFake.sent().size(), 1, "exactly one radio send() for the primary frame");
  checkU64Eq(serialFake.sentReliable().size(), 0, "primary frame never uses sendReliable()");

  msg::Telemetry expected;
  expected.now = 1234;
  expected.seq = 0;
  expected.mode = msg::DriveMode::VELOCITY;
  expected.flags = App::kFlagOtosPresent | App::kFlagOtosConnected | App::kFlagActive |
                    App::kFlagConnLeft | App::kFlagConnRight | App::kFlagAckFresh |
                    App::kFlagLinePresent | App::kFlagColorPresent;
  expected.ack_corr = 7;
  expected.ack_err = 0;
  expected.enc_left = {12.5f, 100.0f, 111};
  expected.enc_right = {-3.25f, -50.0f, 111};
  expected.otos = {1.1f, 2.2f, 0.6f, 10.0f, -5.0f, 0.2f, 120};
  expected.pose = {1.0f, 2.0f, 0.5f};
  expected.twist = {150.0f, 0.0f, 0.75f};
  expected.line = 0x04030201u;
  expected.color = 0x0A090807u;

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
// 2. Single ack slot (115-005: replaces the old depth-3 ring): a SECOND
//    ack() call before the next emit() overwrites the first -- only the
//    LATEST corr/err survives (ack-depth-1 tradeoff, stakeholder-accepted).
//    ack_fresh (flags bit 5) is a ONE-SHOT pulse: set on the frame right
//    after an ack() call, cleared again on the FOLLOWING frame if no new
//    ack() arrived in between.
// ===========================================================================

void scenarioSingleAckSlotOverwritesAndAckFreshIsOneShot() {
  beginScenario("ack(): single slot overwrites on a same-period collision; ack_fresh is a one-shot pulse");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms, serialFake, radioFake);

  // Two acks land before the next emit() -- only the second (corr=4) is
  // visible; corr=1's own ack is silently overwritten (the documented
  // tradeoff: wait_for_ack timeout+retry covers this on the host side).
  telemetry.ack(1, 0);
  telemetry.ack(4, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));

  telemetry.emit(0);   // frame #1 -- carries corr=4's ack, ack_fresh set
  telemetry.emit(40);  // frame #2 -- no ack() call in between, ack_fresh clears

  checkU64Eq(serialFake.sent().size(), 2, "two successive primary frames were sent");

  msg::Telemetry expectedFirst;
  expectedFirst.now = 0;
  expectedFirst.seq = 0;
  expectedFirst.flags = App::kFlagAckFresh;
  expectedFirst.ack_corr = 4;
  expectedFirst.ack_err = static_cast<uint32_t>(msg::ErrCode::ERR_BADARG);

  msg::ReplyEnvelope envFirst;
  envFirst.corr_id = 0;
  envFirst.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  envFirst.body.tlm = expectedFirst;
  std::string expectedFirstLine = armorReply(envFirst);
  checkTrue(!expectedFirstLine.empty(), "independent encode+armor of the expected first frame succeeds");

  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expectedFirstLine,
               "first frame carries only the LATEST ack (corr=4) -- corr=1 was overwritten, not queued");
  }

  msg::Telemetry expectedSecond;
  expectedSecond.now = 40;
  expectedSecond.seq = 1;
  expectedSecond.ack_corr = 4;  // ack_corr/ack_err values persist -- only ack_fresh clears
  expectedSecond.ack_err = static_cast<uint32_t>(msg::ErrCode::ERR_BADARG);

  msg::ReplyEnvelope envSecond;
  envSecond.corr_id = 0;
  envSecond.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  envSecond.body.tlm = expectedSecond;
  std::string expectedSecondLine = armorReply(envSecond);

  if (serialFake.sent().size() == 2) {
    checkStrEq(serialFake.sent()[1], expectedSecondLine,
               "second frame clears ack_fresh -- no new ack() call landed since the first frame");
  }
}

// ===========================================================================
// 3. flags: a representative status/fault/event bit set flips when the
//    value a real call site would produce is fed in via setFlag(), and
//    clears again when the condition clears -- level-set, not a sticky
//    latch.
// ===========================================================================

void scenarioFlagsReflectRealCallSiteValues() {
  beginScenario("setFlag(kFlagFaultI2CSafetyNet/kFlagEventDeadmanExpired, ...) flip the wire flags bits");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms, serialFake, radioFake);

  // Simulates RobotLoop::updateTlm()'s own call site:
  // tlm_.setFlag(App::kFlagFaultI2CSafetyNet, bus_.clearanceSafetyNetCount() > 0);
  telemetry.setFlag(App::kFlagFaultI2CSafetyNet, true);
  // Simulates RobotLoop::cycle()'s own deadman-expiry branch:
  // tlm_.setFlag(App::kFlagEventDeadmanExpired, deadman_.expired());
  telemetry.setFlag(App::kFlagEventDeadmanExpired, true);
  checkU64Eq(telemetry.flags(), App::kFlagFaultI2CSafetyNet | App::kFlagEventDeadmanExpired,
             "flags() reflects both bits set");

  telemetry.emit(0);

  msg::Telemetry expectedSet;
  expectedSet.now = 0;
  expectedSet.seq = 0;
  expectedSet.flags = App::kFlagFaultI2CSafetyNet | App::kFlagEventDeadmanExpired;
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
  telemetry.setFlag(App::kFlagFaultI2CSafetyNet, false);
  telemetry.setFlag(App::kFlagEventDeadmanExpired, false);
  checkU64Eq(telemetry.flags(), 0, "flags() clears once the caller reports the condition cleared");

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
// 4. TelemetrySecondary: rides its own independently-armored line (NOT a
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

  // Primary cadence never stretched past kPrimaryPeriod by more than TWO
  // scheduling steps: one step is the pre-existing sampling-granularity
  // slack (emit() only observes time at kStep resolution -- the same
  // quantizedPeriod() rationale below), the second is 106-002's own
  // documented tie-break cost (telemetry.h's emit() comment: "at most ONE
  // primary frame delayed by one loop cycle roughly once per
  // kSecondaryPeriod") -- this scenario's own long run (3000 ms) crosses
  // kSecondaryPeriod (200 ms) many times over, so it WILL observe that
  // occasional extra-step delay at least once; a bound of kPrimaryPeriod +
  // kStep (the pre-106-002 tolerance) is no longer correct, not because
  // primary cadence regressed without limit, but because the fix
  // deliberately trades one bounded step of primary jitter for
  // guaranteeing secondary a slot at all.
  checkTrue(maxPrimaryGap <= App::kPrimaryPeriod + 2 * kStep,
            "no observed primary-to-primary gap exceeds kPrimaryPeriod by more than the sampling-granularity "
            "step plus 106-002's own one-step tie-break cost");

  // Expected counts, +/-1 for boundary quantization: primary roughly every
  // kPrimaryPeriod, secondary roughly every kSecondaryPeriod.
  // Quantized period: emit() only samples at kStep granularity, so the
  // REALIZED period is rounded up to the next kStep multiple (e.g. 21 ms
  // at a 7 ms step for a 20ms nominal period) -- expected counts are
  // derived from that realized period, not the nominal constant, to avoid
  // a false failure from the sampling granularity itself.
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
// 5. Frame-size: a fully-populated primary frame encodes at or under the
//    rewritten frame's own recorded worst case (telemetry.proto's header
//    comment, sprint 115 ticket 003: 144 B standalone / 153 B as a
//    ReplyEnvelope.tlm arm -- smaller than the pre-rewrite 179 B while
//    carrying strictly more signal).
// ===========================================================================

void scenarioFullyPopulatedPrimaryFrameFitsRecordedWorstCase() {
  beginScenario("a fully-populated primary frame's encoded size is <= the rewritten frame's recorded 153 B worst case");

  msg::Telemetry tlm;
  tlm.now = 0xFFFFFFFFu;
  tlm.seq = 0xFFFFFFFFu;
  tlm.mode = msg::DriveMode::GO_TO;
  tlm.flags = 0xFFFFu;  // every declared bit (0-15) set -- max() = 65535
  tlm.ack_corr = 0xFFFFu;
  tlm.ack_err = 7u;
  tlm.enc_left = {-1234.5f, -500.0f, 0xFFFFFFFFu};
  tlm.enc_right = {6789.25f, 500.0f, 0xFFFFFFFFu};
  tlm.otos = {1234.5f, -6789.25f, 3.14159f, -500.0f, 500.0f, -3.14159f, 0xFFFFFFFFu};
  tlm.pose = {1234.5f, -6789.25f, 3.14159f};
  tlm.twist = {150.0f, -0.5f, 0.75f};
  tlm.line = 0xFFFFFFFFu;
  tlm.color = 0xFFFFFFFFu;

  msg::ReplyEnvelope env;
  env.corr_id = 0xFFFFFFFFu;
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = tlm;

  uint8_t rawBuf[App::kMaxEnvelopeBytes];
  uint16_t n = msg::wire::encode(env, rawBuf, sizeof(rawBuf));
  checkTrue(n > 0, "encode() succeeds for a fully-populated frame");
  checkTrue(n <= msg::wire::kReplyEnvelopeMaxEncodedSize,
            "encoded size fits the rewritten frame's recorded worst case for ReplyEnvelope{tlm}");
  std::printf("  measured: fully-populated primary frame encodes to %u bytes (worst case %u)\n",
              static_cast<unsigned>(n), static_cast<unsigned>(msg::wire::kReplyEnvelopeMaxEncodedSize));
}

// ===========================================================================
// 6. Measured cadence report -- this ticket's own acceptance criterion:
//    report the REAL realized cadence (not assumed) against the ~25 Hz/40 ms
//    target (118: kPrimaryPeriod follows robot_loop.cpp's kCycle back to
//    40ms/~25Hz, restored from the fictional 20ms/50Hz a zeroed
//    kSettle/kClear had been faking -- see
//    clasi/issues/restore-the-interleaved-request-settle-tick-loop-schedule.md).
// ===========================================================================

void scenarioMeasuredCadenceReport() {
  beginScenario("measured emission cadence (both frame types) vs. the ~25 Hz/40 ms target");

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
  std::printf("  measured: primary %.2f Hz (target ~25 Hz/40 ms), secondary %.2f Hz (target ~5 Hz/200 ms) over %u ms\n",
              primaryHz, secondaryHz, static_cast<unsigned>(kEndTime));

  // Not required to HIT 25 Hz exactly (ticket's own acceptance criterion)
  // -- only sane and in the right neighborhood for a deterministic
  // scripted-clock host test. Same proportional margin the pre-118 50 Hz
  // target used (+/-40%), rescaled to the new 25 Hz target.
  checkTrue(primaryHz > 15.0 && primaryHz < 35.0, "measured primary Hz is in a sane neighborhood of the 25 Hz target");
  checkTrue(secondaryHz > 2.0 && secondaryHz < 8.0, "measured secondary Hz is in a sane neighborhood of the ~5 Hz target");
}

// ===========================================================================
// 7. kFlagFaultCommsMalformed (originally 104-004, now flags bit 9): a
//    malformed/undecodable inbound frame pumped through the SAME App::Comms
//    instance Telemetry's own Comms::sendReply() rides --
//    App::Comms::malformedCount() rising above 0 -- sets
//    App::kFlagFaultCommsMalformed in the NEXT telemetry frame, once a
//    caller mirrors it via setFlag() exactly the way RobotLoop::updateTlm()
//    does. Bit clears on a later frame if the caller stops reporting it --
//    same level-set discipline as scenario 3.
// ===========================================================================

void scenarioMalformedFrameSetsCommsMalformedFlagBit() {
  beginScenario("malformed frame -> Comms::malformedCount() -> setFlag(kFlagFaultCommsMalformed) sets the wire bit");

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
  comms.pump(cmd, /*now=*/0);
  checkU64Eq(comms.malformedCount(), 1, "malformedCount() incremented by the malformed line");

  // Mirrors RobotLoop::updateTlm()'s own call site (src/firm/app/robot_loop.cpp):
  // tlm_.setFlag(App::kFlagFaultCommsMalformed, comms_.malformedCount() > 0);
  telemetry.setFlag(App::kFlagFaultCommsMalformed, comms.malformedCount() > 0);
  checkU64Eq(telemetry.flags(), App::kFlagFaultCommsMalformed, "flags() reflects kFlagFaultCommsMalformed");

  telemetry.emit(0);

  msg::Telemetry expectedSet;
  expectedSet.now = 0;
  expectedSet.seq = 0;
  expectedSet.flags = App::kFlagFaultCommsMalformed;
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
  // RobotLoop call site would keep this bit latched; this half of the
  // scenario only proves Telemetry's own level-set discipline, matching
  // scenario 3's flags-bit treatment).
  telemetry.setFlag(App::kFlagFaultCommsMalformed, false);
  checkU64Eq(telemetry.flags(), 0, "flags() clears once the caller reports the condition cleared");

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

// ===========================================================================
// 8. 106-002 fix (carried forward): secondary telemetry is NOT starved to 0
//    Hz when the caller's own per-call period exceeds kPrimaryPeriod -- the
//    ACTUAL bug (`clasi/issues/secondary-telemetry-starved-by-106-001-
//    cadence-retarget.md`): a real loop period ABOVE kPrimaryPeriod makes
//    primaryDue() true on EVERY call, and the pre-106-002 "primary always
//    wins a same-call tie" rule left secondary starved forever. 118 restores
//    kPrimaryPeriod to its genuine 40ms (was fictionally 20ms under the
//    5f5a2ba7 regression -- see
//    clasi/issues/restore-the-interleaved-request-settle-tick-loop-schedule.md)
//    -- a 52ms call period is still above it, just by a narrower margin than
//    the regressed 20ms case gave; the same regression shape still applies.
// ===========================================================================

void scenarioSecondaryNotStarvedWhenCallPeriodExceedsPrimaryPeriod() {
  beginScenario("emit(): secondary is not starved to 0 Hz when called at a fixed period > kPrimaryPeriod "
                "(52ms call period, above the 40ms primary target)");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms, serialFake, radioFake);

  // emit() called once per "cycle" at a fixed 52ms period (ABOVE
  // kPrimaryPeriod=40ms), so primaryDue() is true on EVERY call -- exactly
  // the condition that starved secondary to 0 Hz pre-106-002.
  const uint32_t kCallPeriod = 52;  // [ms]
  const int kCalls = 100;           // ~5.2s of simulated loop time
  uint32_t now = 0;
  uint32_t lastPrimarySeen = 0;
  bool sawPrimary = false;
  uint32_t maxPrimaryGap = 0;
  for (int i = 0; i < kCalls; ++i) {
    uint32_t beforePrimary = telemetry.primaryEmitCount();
    telemetry.emit(now);
    if (telemetry.primaryEmitCount() > beforePrimary) {
      if (sawPrimary) {
        uint32_t gap = now - lastPrimarySeen;
        if (gap > maxPrimaryGap) maxPrimaryGap = gap;
      }
      lastPrimarySeen = now;
      sawPrimary = true;
    }
    now += kCallPeriod;
  }

  checkTrue(telemetry.secondaryEmitCount() > 0,
            "secondary is NOT starved to 0 Hz -- at least one secondary frame sent over ~5s at a 52ms call period");

  double secondsElapsed = static_cast<double>(now) / 1000.0;
  double secondaryHz = static_cast<double>(telemetry.secondaryEmitCount()) / secondsElapsed;
  std::printf("  measured: secondary %.2f Hz (target ~5 Hz/200 ms) at a 52 ms emit() call period\n", secondaryHz);
  checkTrue(secondaryHz > 2.0 && secondaryHz < 8.0,
            "secondary rate is in a sane neighborhood of the ~5 Hz target even though the caller's own "
            "period exceeds kPrimaryPeriod");

  // The tie-break's own documented cost (telemetry.h's emit() comment): at
  // most one primary frame delayed by one call period, roughly once per
  // kSecondaryPeriod -- primary cadence stays well short of stalling.
  checkTrue(maxPrimaryGap <= App::kSecondaryPeriod,
            "no primary-to-primary gap approaches kSecondaryPeriod -- the tie-break costs at most an "
            "occasional single-cycle delay, not a stall");
}

}  // namespace

int main() {
  scenarioPrimaryFrameAssemblyMatchesIndependentEncode();
  scenarioSingleAckSlotOverwritesAndAckFreshIsOneShot();
  scenarioFlagsReflectRealCallSiteValues();
  scenarioSecondaryNeverCoincidesWithPrimaryAndDoesNotDelayIt();
  scenarioFullyPopulatedPrimaryFrameFitsRecordedWorstCase();
  scenarioMeasuredCadenceReport();
  scenarioMalformedFrameSetsCommsMalformedFlagBit();
  scenarioSecondaryNotStarvedWhenCallPeriodExceedsPrimaryPeriod();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Telemetry scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Telemetry scenarios\n", g_failureCount);
  return 1;
}
