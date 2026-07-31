// app_telemetry_harness.cpp -- off-hardware acceptance harness for
// App::Telemetry (src/firm/app/telemetry.{h,cpp}). Proves: the
// Types::RobotState -> wire msg::Telemetry projection via update()/emit()
// (124-009, issue §B1 -- "one struct, filled once, consumed three times"),
// the bounded ack ring's push/evict/persist behavior, flags derived from
// state (not scattered setFlag() calls -- update() is the ONE place a bit
// flips), setLiveFlag()'s narrow post-tick escape hatch, and realized
// primary-frame emission cadence.
//
// 124-009 rewrite (robot-state-blackboard-...md): Telemetry's public API
// changed shape entirely -- `Frame`/`SecondaryFrame`, `setFrame()`, the
// public `setFlag()`, `setSecondaryFrame()`, and every secondary-cadence
// accessor (`secondaryEmitCount()`/`lastSecondaryEmit()`) are GONE.
// `update(const Types::RobotState&)` replaces setFrame()+ten scattered
// setFlag() calls; `setLiveFlag()` replaces the two post-tick-only
// setFlag() calls (kFlagFaultMoveTimeout/kFlagFaultShapingDisabled); the
// constructor drops its Transport& parameters (those existed only for
// TelemetrySecondary's own independently-armored line). Every scenario
// below either follows RobotLoop's own new call shape (build a
// Types::RobotState, call update(), then emit()) or -- for
// TelemetrySecondary/ack-ring scenarios untouched by the state-object
// change -- keeps its prior shape with only the constructor signature
// updated.
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
// verified by independently building the expected msg::Telemetry, encoding
// + armoring it via the SAME msg::wire::encode()/WireRuntime primitives
// Telemetry itself is built on, and comparing the resulting line
// byte-for-byte against what the FakeTransport actually received.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "app/comms.h"
#include "app/telemetry.h"
#include "firm/types/robot_state.h"
#include "messages/envelope.h"
#include "messages/wire.h"
#include "messages/wire_runtime.h"
#include "support/fake_transport.h"

// pumpOne() -- the pre-ring `Comms::pump(Cmd&, now)` shape, rebuilt on the
// ring API (command-ingestion-ring-buffered-comms-subsystem-routing-two-
// stops.md §1: pump() now DRAINS both transports into a ring and
// takeCommand() pops from it). These scenarios each feed exactly one line
// and want the one command it produced, so "pump then take" is the honest
// local equivalent -- not a claim that pump() still stops after one line.
App::Cmd pumpOne(App::Comms& comms, uint32_t now) {  // [ms]
  App::Cmd cmd;
  comms.pump(now);
  comms.takeCommand(cmd);  // leaves cmd at status kNone when nothing decoded
  return cmd;
}

namespace {

// packAck() -- mirrors telemetry.cpp's own pushAckRing() packed-word format
// EXACTLY (124-008, issue §B4): corr_id<<4 | err. Used here only to build
// independent EXPECTED msg::Telemetry.acks_[] entries -- production code's
// own kAckErrBits/kAckErrMask live in telemetry.cpp (internal linkage), not
// reachable from this TU, so this is a second, deliberately-independent
// implementation of the same packing rule (same spirit as this file's own
// armor()/armorReply() re-implementing the production encode path).
constexpr uint32_t packAck(uint32_t corrId, uint32_t err) { return (corrId << 4) | err; }

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

// --- armor() -- 124-005 (protocol v5 Part A, "framing grammar cutover"):
// builds the COMPLETE wire LINE, `<command>':'<COBS+CRC bytes>` (CRC-then-
// COBS, delimiter 0x0A) Comms::sendReply() itself builds, used here only to
// construct scenario EXPECTATIONS independently of Telemetry's own send
// path. `command` is REQUIRED and, for every scenario in this file, "TLM"
// -- App::Telemetry never emits OK/ERR (envelope.proto's own doc comment:
// no current firmware call site). The trailing '\n' terminator is a
// transport concern, not included in this function's return value --
// matches what a FakeTransport::sent() capture holds. ------

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

std::string armorReply(const msg::ReplyEnvelope& env) {
  uint8_t rawBuf[App::kMaxEnvelopeBytes];
  uint16_t n = msg::wire::encode(env, rawBuf, sizeof(rawBuf));
  if (n == 0) return std::string();
  return armor(rawBuf, n, "TLM");
}

// --- FakeTransport is TestSupport::FakeTransport
// (src/tests/sim/support/fake_transport.h, ticket 105-002) -- the ONE canonical
// scripted queue of inbound lines plus a log of every send()/sendReliable()
// call. ---------------------------------------------------------------

using TestSupport::FakeTransport;

// ===========================================================================
// 1. RobotState -> wire projection (124-009, issue §B1): update(state) then
//    emit() builds exactly the fields AC #1 lists and sends via
//    Comms::sendReply() (both transports, matching Comms's own broadcast
//    discipline). Also proves the ack ring rides alongside every other
//    state-derived field, and that age (issue §B2/SUC-006) is computed as
//    now - sampleTime for each independently-timestamped reading.
// ===========================================================================

void scenarioUpdateProjectsWholeStateInOneCall() {
  beginScenario("update(state)+emit(): every AC-listed field is derived from RobotState in one call");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  const uint32_t kNow = 1234;

  Types::RobotState state;
  state.time.cycleStart = kNow;
  state.command.mode = Types::Mode::Velocity;
  state.command.moveActive = true;

  // Deliberately DIFFERENT sampleTime per reading (issue §B2/SUC-006: "two
  // fields carrying one value is worse than one field") -- ages below
  // (111/111/120) are chosen distinct from each other's SOURCE (left vs.
  // right vs. otos) but happen to share a value between the two wheels
  // here purely for arithmetic convenience; scenario 2 below (age
  // differential) is where left != right is the actual point under test.
  state.wheelLeft.position = 12.5f;
  state.wheelLeft.velocity = 100.0f;
  state.wheelLeft.sampleTime = kNow - 111;
  state.wheelLeft.connected = true;

  state.wheelRight.position = -3.25f;
  state.wheelRight.velocity = -50.0f;
  state.wheelRight.sampleTime = kNow - 111;
  state.wheelRight.connected = true;

  state.otos.present = true;
  state.otos.connected = true;
  state.otos.x = 1.1f;
  state.otos.y = 2.2f;
  state.otos.heading = 0.6f;
  state.otos.v_x = 10.0f;
  state.otos.v_y = -5.0f;
  state.otos.omega = 0.2f;
  state.otos.sampleTime = kNow - 120;

  state.pose.x = 1.0f;
  state.pose.y = 2.0f;
  state.pose.heading = 0.5f;
  state.pose.v_x = 150.0f;
  state.pose.v_y = 0.0f;
  state.pose.omega = 0.75f;

  state.perception.line = 0x04030201u;
  state.perception.lineFresh = true;
  state.perception.color = 0x0A090807u;
  state.perception.colorFresh = true;

  telemetry.update(state);
  telemetry.ack(7, 0);
  telemetry.emit(kNow);  // first call -- always sends primary (boot, no arming)
  // (120) ack(7, 0) above ALSO pushed onto the bounded ack ring -- see
  // this file's own scenario 4 below for the ring's push/evict/persist
  // behavior in isolation; here it's just one entry riding alongside
  // every other AC-listed field.

  checkU64Eq(serialFake.sent().size(), 1, "exactly one serial send() for the primary frame");
  checkU64Eq(radioFake.sent().size(), 1, "exactly one radio send() for the primary frame");
  checkU64Eq(serialFake.sentReliable().size(), 0, "primary frame never uses sendReliable()");

  msg::Telemetry expected;
  expected.now = kNow;
  expected.seq = 0;
  expected.mode = msg::DriveMode::VELOCITY;
  expected.flags = App::kFlagOtosPresent | App::kFlagOtosConnected | App::kFlagActive |
                    App::kFlagConnLeft | App::kFlagConnRight |
                    App::kFlagLinePresent | App::kFlagColorPresent;
  expected.enc_left = {msg::EncoderReading::packPosition(12.5f), msg::EncoderReading::packVelocity(100.0f),
                        /*age=*/111, /*position_epoch=*/0};
  expected.enc_right = {msg::EncoderReading::packPosition(-3.25f), msg::EncoderReading::packVelocity(-50.0f),
                         /*age=*/111, /*position_epoch=*/0};
  expected.otos = {msg::OtosReading::packX(1.1f),   msg::OtosReading::packY(2.2f),
                    msg::OtosReading::packHeading(0.6f), msg::OtosReading::packVX(10.0f),
                    msg::OtosReading::packVY(-5.0f), msg::OtosReading::packOmega(0.2f),
                    /*age=*/120};
  expected.pose = {msg::Pose2D::packX(1.0f), msg::Pose2D::packY(2.0f), msg::Pose2D::packH(0.5f)};
  expected.twist = {msg::BodyTwist3::packVX(150.0f), msg::BodyTwist3::packVY(0.0f),
                     msg::BodyTwist3::packOmega(0.75f)};
  expected.line = 0x04030201u;
  expected.color = 0x0A090807u;
  expected.acks_count = 1;
  expected.acks_[0] = packAck(7, 0);

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
// 2. Age differential (issue §B2/SUC-006): enc_left.age and enc_right.age
//    are computed INDEPENDENTLY from each wheel's own sampleTime -- feeding
//    genuinely different sampleTime values produces genuinely different
//    ages, never silently collapsed to a shared value. The REAL per-cycle
//    skew (driven by the actual kSettle+kClear-separated collect schedule)
//    is app_robot_loop_harness.cpp's own end-to-end scenario; this is the
//    Telemetry-layer unit proof that update()'s projection itself carries
//    whatever skew RobotState hands it, rather than re-deriving a single
//    shared age from `now` alone.
// ===========================================================================

void scenarioAgeIsComputedIndependentlyPerReading() {
  beginScenario("update(): enc_left.age/enc_right.age/otos.age are independent -- different sampleTime -> "
                "different age, never collapsed");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  const uint32_t kNow = 1000;

  Types::RobotState state;
  state.time.cycleStart = kNow;
  state.wheelLeft.sampleTime = kNow - 8;    // age 8
  state.wheelRight.sampleTime = kNow - 20;  // age 20 -- genuinely later collect, genuinely different
  state.otos.present = true;
  state.otos.sampleTime = kNow - 35;        // age 35 -- yet another distinct value

  telemetry.update(state);
  telemetry.emit(kNow);

  msg::Telemetry expected;
  expected.now = kNow;
  expected.seq = 0;
  expected.flags = App::kFlagOtosPresent;
  expected.enc_left.age = 8;
  expected.enc_right.age = 20;
  expected.otos.age = 35;

  msg::ReplyEnvelope env;
  env.corr_id = 0;
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = expected;
  std::string expectedLine = armorReply(env);
  checkTrue(!expectedLine.empty(), "independent encode+armor of the expected frame succeeds");
  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expectedLine,
               "enc_left.age=8, enc_right.age=20, otos.age=35 -- three distinct, real values, "
               "never equal to each other or to zero");
  }
}

// ===========================================================================
// 3. Ack ring (124-008, issue §B4): the single "freshest ack" scalar slot
//    (ack_corr/ack_err, flags bit 5 kFlagAckFresh) is DELETED -- ring
//    membership in `acks` already means "really acked," so two ack() calls
//    before the next emit() land BOTH entries in the ring (oldest first),
//    and the ring persists unchanged (no new push) across a later emit()
//    that follows. Unaffected by the state-object change -- ack() never
//    took a Frame/RobotState argument, before or after 124-009 -- update()
//    is never even called here.
// ===========================================================================

void scenarioAckRingCarriesEveryPushAndPersistsAcrossEmits() {
  beginScenario("ack(): every push lands in the ring (no single-slot overwrite); ring persists across an emit() "
                "with no new ack()");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  // Two acks land before the next emit() -- BOTH survive in the ring (no
  // single-slot overwrite any more, 124-008).
  telemetry.ack(1, 0);
  telemetry.ack(4, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));

  telemetry.emit(0);   // frame #1 -- carries both acks in the ring
  telemetry.emit(40);  // frame #2 -- no ack() call in between, ring unchanged

  checkU64Eq(serialFake.sent().size(), 2, "two successive primary frames were sent");

  msg::Telemetry expectedFirst;
  expectedFirst.now = 0;
  expectedFirst.seq = 0;
  // Ring (120): BOTH pushes survive, oldest (corr=1) first.
  expectedFirst.acks_count = 2;
  expectedFirst.acks_[0] = packAck(1, 0);
  expectedFirst.acks_[1] = packAck(4, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));

  msg::ReplyEnvelope envFirst;
  envFirst.corr_id = 0;
  envFirst.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  envFirst.body.tlm = expectedFirst;
  std::string expectedFirstLine = armorReply(envFirst);
  checkTrue(!expectedFirstLine.empty(), "independent encode+armor of the expected first frame succeeds");

  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expectedFirstLine,
               "first frame's ring carries BOTH pushes, oldest first");
  }

  msg::Telemetry expectedSecond;
  expectedSecond.now = 40;
  expectedSecond.seq = 1;
  // Ring (120): UNCHANGED from frame #1 -- no new ack() call landed, and
  // the ring (like every other Frame field) persists its last-staged
  // snapshot across an emit() that doesn't touch it, rather than clearing.
  expectedSecond.acks_count = 2;
  expectedSecond.acks_[0] = packAck(1, 0);
  expectedSecond.acks_[1] = packAck(4, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));

  msg::ReplyEnvelope envSecond;
  envSecond.corr_id = 0;
  envSecond.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  envSecond.body.tlm = expectedSecond;
  std::string expectedSecondLine = armorReply(envSecond);

  if (serialFake.sent().size() == 2) {
    checkStrEq(serialFake.sent()[1], expectedSecondLine,
               "second frame -- no new ack() call landed since the first frame -- the ack ring still carries "
               "both prior pushes, unchanged");
  }
}

// ===========================================================================
// 4. flags: derived FRESH from state on every update() call (124-009) --
//    re-deriving from a state whose health fields have cleared produces a
//    cleared flags() too, without any caller-side "un-set" call -- level-set
//    by re-derivation, not a sticky latch update() ever has to reason about.
// ===========================================================================

void scenarioFlagsAreFreshlyDerivedFromStateEveryUpdate() {
  beginScenario("update(state): flags() is freshly derived from state.health every call -- clears when the "
                "state clears, no caller-side un-set needed");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  // Simulates the wheel/health-section publish point in RobotLoop::cycle()
  // (state_.health.wedgeLatch = motorL_.wedged() || motorR_.wedged();
  //  state_.health.i2cSafetyNetCount = bus_.clearanceSafetyNetCount();).
  Types::RobotState state;
  state.time.cycleStart = 0;
  // Pin sampleTime to cycleStart on every update() below -- age = now -
  // sampleTime would otherwise drift to a nonzero, scenario-irrelevant
  // value as cycleStart advances (this scenario is about flags, not age).
  state.wheelLeft.sampleTime = state.time.cycleStart;
  state.wheelRight.sampleTime = state.time.cycleStart;
  state.health.i2cSafetyNetCount = 1;
  state.health.wedgeLatch = true;
  telemetry.update(state);
  checkU64Eq(telemetry.flags(), App::kFlagFaultI2CSafetyNet | App::kFlagFaultWedgeLatch,
             "flags() reflects both bits set immediately after update()");

  telemetry.emit(0);

  msg::Telemetry expectedSet;
  expectedSet.now = 0;
  expectedSet.seq = 0;
  expectedSet.flags = App::kFlagFaultI2CSafetyNet | App::kFlagFaultWedgeLatch;
  msg::ReplyEnvelope envSet;
  envSet.corr_id = 0;
  envSet.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  envSet.body.tlm = expectedSet;
  std::string expectedSetLine = armorReply(envSet);
  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expectedSetLine, "first frame carries both bits set");
  }

  // Condition clears in state -- the NEXT update() call re-derives flags()
  // to match, with no separate "un-set" call: update() OWNS these bits
  // completely on every call, unlike setLiveFlag()'s two exceptions below.
  state.time.cycleStart = 40;
  state.wheelLeft.sampleTime = state.time.cycleStart;
  state.wheelRight.sampleTime = state.time.cycleStart;
  state.health.i2cSafetyNetCount = 0;
  state.health.wedgeLatch = false;
  telemetry.update(state);
  checkU64Eq(telemetry.flags(), 0, "flags() clears once state reports the condition cleared, via re-derivation");

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
// 5. setLiveFlag() (124-009): the narrow escape hatch for
//    kFlagFaultMoveTimeout/kFlagFaultShapingDisabled, whose defining
//    condition (Motion::MoveQueue::tick()'s own outcome) is not known yet
//    at update()/emit() time -- see telemetry.h's own doc comment. Proves
//    setLiveFlag() mutates flags() live WITHOUT requiring another update()
//    call, and that a SUBSEQUENT update() call leaves a setLiveFlag()-owned
//    bit untouched (update() never re-derives bits 15/16 itself).
// ===========================================================================

void scenarioSetLiveFlagMutatesLiveFlagsIndependentlyOfUpdate() {
  beginScenario("setLiveFlag(): mutates flags() live without another update() call; a later update() call "
                "leaves it untouched");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  Types::RobotState state;
  state.time.cycleStart = 0;
  state.wheelLeft.sampleTime = state.time.cycleStart;
  state.wheelRight.sampleTime = state.time.cycleStart;
  telemetry.update(state);
  checkU64Eq(telemetry.flags(), 0, "flags() starts clear");

  // Simulates RobotLoop::cycle()'s own post-tick call site:
  // tlm_.setLiveFlag(kFlagFaultMoveTimeout, state_.health.moveTimeout);
  telemetry.setLiveFlag(App::kFlagFaultMoveTimeout, true);
  checkU64Eq(telemetry.flags(), App::kFlagFaultMoveTimeout,
             "setLiveFlag() sets the bit immediately, with no update()/emit() call in between");

  // A LATER update() call (the next cycle's own tlm_.update(state_)) must
  // NOT clobber this bit -- update() only ever mutates the bits it derives
  // from state (kFlagFaultMoveTimeout is not one of them).
  Types::RobotState nextState;
  nextState.time.cycleStart = 40;
  // Pin sampleTime to cycleStart, same reasoning as scenario 4 above -- this
  // scenario is about setLiveFlag(), not age.
  nextState.wheelLeft.sampleTime = nextState.time.cycleStart;
  nextState.wheelRight.sampleTime = nextState.time.cycleStart;
  telemetry.update(nextState);
  checkU64Eq(telemetry.flags(), App::kFlagFaultMoveTimeout,
             "a later update() call leaves the setLiveFlag()-owned bit untouched");

  telemetry.emit(40);
  msg::Telemetry expected;
  expected.now = 40;
  expected.seq = 0;
  expected.flags = App::kFlagFaultMoveTimeout;
  msg::ReplyEnvelope env;
  env.corr_id = 0;
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = expected;
  std::string expectedLine = armorReply(env);
  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expectedLine,
               "the frame after the later update() call still carries the setLiveFlag()-set bit");
  }

  telemetry.setLiveFlag(App::kFlagFaultMoveTimeout, false);
  checkU64Eq(telemetry.flags(), 0, "setLiveFlag() clears the bit live too, level-set not a sticky latch");
}

// ===========================================================================
// 6. Frame-size: a primary frame populated at EVERY field's own declared
//    (max)/(abs_max) bound (124-008, issue §B5's size-accounting table)
//    fits the regenerated worst case, kReplyEnvelopeMaxEncodedSize
//    (<= the envelope budget -- see the check's own comment for why the
//    old <=130B ticket-local figure was re-derived).
//    Several bounds this ticket adds (now/seq/the packed acks word/
//    ReplyEnvelope.corr_id) are SIZING bounds, not hard wire limits -- see
//    telemetry.proto's/envelope.proto's own doc comments -- so this
//    scenario, unlike a real robot's own output, uses the DECLARED bound
//    itself (not an arbitrary large value) to prove the budget the
//    generator computed is actually achievable, not merely asserted.
//    line/color are genuinely unbounded (any byte per channel can be 0xFF)
//    and use the full uint32 range. Operates on msg::wire::encode()
//    directly -- no App::Telemetry instance involved, unaffected by
//    124-009's API change.
// ===========================================================================

void scenarioFullyPopulatedPrimaryFrameFitsRecordedWorstCase() {
  beginScenario("a primary frame at every field's own declared bound fits the regenerated worst case");

  msg::Telemetry tlm;
  tlm.now = 2097151u;   // (max) -- sizing bound, not a hard wire limit
  tlm.seq = 127u;       // (max) -- sizing bound, not a hard wire limit
  tlm.mode = msg::DriveMode::GO_TO;
  tlm.flags = 262143u;  // (max) -- covers bit 16 (kFlagFaultShapingDisabled) and bit 17 (kFlagFaultPositionClamped)
  // EncoderReading/OtosReading/Pose2D/BodyTwist3: raw wire ints at their
  // own declared (abs_max) -- the RUNTIME engine's validateBounds()/
  // worst-case-size calculator both operate on the raw sint32 value, never
  // the scaled float (scale is a header-generation-time concept only --
  // options.proto's own (scale) doc comment).
  tlm.enc_left = {/*position=*/-32000, /*velocity=*/4000, /*age=*/255, /*position_epoch=*/127};
  tlm.enc_right = {/*position=*/32000, /*velocity=*/-4000, /*age=*/255, /*position_epoch=*/127};
  tlm.otos = {/*x=*/32000, /*y=*/-32000, /*heading=*/3142, /*v_x=*/-4000, /*v_y=*/4000, /*omega=*/-1000,
              /*age=*/255};
  tlm.pose = {/*x=*/-32000, /*y=*/32000, /*h=*/-3142};
  tlm.twist = {/*v_x=*/4000, /*v_y=*/-4000, /*omega=*/1000};
  tlm.line = 0xFFFFFFFFu;
  tlm.color = 0xFFFFFFFFu;
  // Full ring (120) at its own declared worst case per entry (124-008:
  // packed uint32, (max)=1048575 == (65535<<4)|15) -- proves the TRUE
  // worst case (not just one entry) still fits the budget below. This is
  // the engine's first real FieldKind::kRepeatedScalar use.
  tlm.acks_count = App::kAckRingDepth;
  for (uint8_t e = 0; e < App::kAckRingDepth; ++e) {
    tlm.acks_[e] = packAck(65535u, 15u);
  }
  tlm.cycle_busy = 200000u;
  tlm.cycle_period = 200000u;

  msg::ReplyEnvelope env;
  env.corr_id = 65535u;  // ReplyEnvelope.corr_id's own declared (max), 124-008
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = tlm;

  uint8_t rawBuf[App::kMaxEnvelopeBytes];
  uint16_t n = msg::wire::encode(env, rawBuf, sizeof(rawBuf));
  checkTrue(n > 0, "encode() succeeds for a fully-populated frame");
  checkTrue(n <= msg::wire::kReplyEnvelopeMaxEncodedSize,
            "encoded size fits the rewritten frame's recorded worst case for ReplyEnvelope{tlm}");
  // The <=130B figure was 124-008's OWN ticket-local target, not a system
  // limit -- the system limit is msg::wire::kEnvelopeBudgetBytes (240B),
  // which wire.h itself static_asserts. Re-derived here by the
  // command-ingestion rework (command-ingestion-ring-buffered-comms-
  // subsystem-routing-two-stops.md §1): raising kAckRingDepth 4 -> 12, so
  // that a burst which fits the new command ring also fits the ack ring,
  // costs 8 more packed ack words (~3B each) on the worst-case frame and
  // puts it at 155B. That is a deliberate, load-bearing trade -- acks the
  // firmware really pushed would otherwise be unobservable -- and it stays
  // comfortably inside the real budget, which is what this gate now
  // asserts.
  // 240 == the envelope budget wire.h itself static_asserts against (its
  // own kEnvelopeBudgetBytes narrative constant is documentation, not a
  // declared symbol -- the two static_asserts spell the number out).
  checkTrue(msg::wire::kReplyEnvelopeMaxEncodedSize <= 240,
            "kReplyEnvelopeMaxEncodedSize fits the 240-byte envelope budget");
  std::printf("  measured: fully-populated primary frame encodes to %u bytes (worst case %u)\n",
              static_cast<unsigned>(n), static_cast<unsigned>(msg::wire::kReplyEnvelopeMaxEncodedSize));
}

// ===========================================================================
// 7. Measured cadence report -- report the REAL realized primary cadence
//    (not assumed) against the ~25 Hz/40 ms target. 124-009: TelemetrySecondary
//    and its tie-break/alternation cadence machinery are GONE -- emit() is
//    now a plain "due since last send" gate with nothing to trade off
//    against, so this scenario measures ONLY primary cadence (no secondary
//    Hz, no tie-break-cost tolerance).
// ===========================================================================

void scenarioMeasuredCadenceReport() {
  beginScenario("measured primary emission cadence vs. the ~25 Hz/40 ms target");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  // A MOVING robot: the cadence target only applies when there is something
  // to say. Since 2026-07-29 emit() is gated on that (stakeholder directive:
  // no telemetry while parked, so a serial terminal stays typeable), so this
  // scenario has to put the robot in the state whose cadence it is measuring
  // -- an idle robot is now correctly near-silent, and measuring IT would be
  // measuring the wrong contract. No boot-boilerplate warm-up call of any
  // kind is needed or permitted (125-002): App::Telemetry has no arming
  // state at all any more -- a freshly constructed instance behaves
  // identically to one that has run for an hour, so kFlagActive alone is
  // enough to start the stream under the default kAuto mode.
  Types::RobotState moving;
  moving.command.moveActive = true;
  telemetry.update(moving);

  const uint32_t kStep = 3;  // [ms] fine-grained relative to kPrimaryPeriod=40
  const uint32_t kEndTime = 10000;
  for (uint32_t now = 0; now <= kEndTime; now += kStep) {
    telemetry.emit(now);
  }

  double primaryHz = static_cast<double>(telemetry.primaryEmitCount()) / (static_cast<double>(kEndTime) / 1000.0);
  std::printf("  measured: primary %.2f Hz while MOVING (target ~25 Hz/40 ms) over %u ms\n", primaryHz,
              static_cast<unsigned>(kEndTime));

  // Not required to HIT 25 Hz exactly (ticket's own acceptance criterion)
  // -- only sane and in the right neighborhood for a deterministic
  // scripted-clock host test.
  checkTrue(primaryHz > 15.0 && primaryHz < 35.0, "measured primary Hz is in a sane neighborhood of the 25 Hz target");

  // The other half of the contract: once the robot parks, the link goes
  // QUIET -- and (issue Part 1 item 6, report-on-change deleted outright)
  // there is no longer a "park transition" frame either: the stream simply
  // stops the moment activity's own window closes, it does not get one
  // more frame to announce the stop. This robot's own coasting velocity is
  // zero (never set), so the window closes immediately -- there is no
  // coast-down tail to observe here (see this file's own dedicated
  // coast-holdoff scenario below for that).
  const uint64_t whileMoving = telemetry.primaryEmitCount();
  Types::RobotState parked;  // moveActive false, zero velocity, same flags
  parked.time.cycleStart = kEndTime;
  telemetry.update(parked);

  uint32_t now = kEndTime + kStep;
  const uint32_t settleEnd = now + 5000;  // [ms] comfortably > kCoastHoldoff (2000ms)
  for (; now <= settleEnd; now += kStep) telemetry.emit(now);
  checkU64Eq(telemetry.primaryEmitCount(), whileMoving,
             "parked robot emits nothing further once activity ends -- no report-on-change "
             "transition frame, the stream simply stops");

  // Every send() call accounted for exactly -- no call produced an
  // untracked extra line (124-009: only ONE frame type exists, so this is
  // simply primaryEmitCount() now, not primary+secondary).
  checkU64Eq(serialFake.sent().size(), telemetry.primaryEmitCount(),
             "serial send() log size == primary emit count (no untracked sends, no second frame type)");
}

// ===========================================================================
// 8. kFlagFaultCommsMalformed (originally 104-004, now flags bit 9): a
//    malformed/undecodable inbound frame pumped through the SAME App::Comms
//    instance Telemetry's own Comms::sendReply() rides --
//    App::Comms::malformedCount() rising above 0 -- sets
//    App::kFlagFaultCommsMalformed in the NEXT telemetry frame, once
//    RobotLoop's own health-section publish mirrors it into
//    state.health.commsMalformedCount and update() derives the bit from
//    it. Bit clears on a later frame if the state clears -- same
//    fresh-derivation discipline as scenario 4.
// ===========================================================================

void scenarioMalformedFrameSetsCommsMalformedFlagBit() {
  beginScenario("malformed frame -> Comms::malformedCount() -> state.health.commsMalformedCount -> update() "
                "sets the wire bit");

  FakeTransport serialFake;
  FakeTransport radioFake;
  // Unrecognized text-plane line (not HELLO, not PING) -- 123-002 removed
  // the "*B" armor-prefix check entirely (armor is binary now), so any
  // non-HELLO/PING text line is malformed by construction; mirrors
  // app_comms_harness.cpp's own scenarioMalformedUnrecognizedTextLineRejected()
  // to increment malformedCount() by exactly 1.
  serialFake.enqueueInbound("*Xsomeunrecognizedarmor");

  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  App::Cmd cmd;
  cmd = pumpOne(comms, /*now=*/0);
  checkU64Eq(comms.malformedCount(), 1, "malformedCount() incremented by the malformed line");

  // Mirrors RobotLoop::cycle()'s own pace-block publish point
  // (src/firm/app/robot_loop.cpp):
  // state_.health.commsMalformedCount = comms_.malformedCount();
  Types::RobotState state;
  state.time.cycleStart = 0;
  state.wheelLeft.sampleTime = state.time.cycleStart;
  state.wheelRight.sampleTime = state.time.cycleStart;
  state.health.commsMalformedCount = comms.malformedCount();
  telemetry.update(state);
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

  // No further malformed input arrives -- a fresh state (commsMalformedCount
  // reset to 0, as RobotLoop would report if malformedCount() itself could
  // clear) re-derives the bit cleared, same discipline as scenario 4.
  Types::RobotState clearState;
  clearState.time.cycleStart = 40;
  clearState.wheelLeft.sampleTime = clearState.time.cycleStart;
  clearState.wheelRight.sampleTime = clearState.time.cycleStart;
  telemetry.update(clearState);
  checkU64Eq(telemetry.flags(), 0, "flags() clears once state reports the condition cleared");

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
// 9. Ack ring (120) push/evict/ordering, in isolation: pushing more than
//    kAckRingDepth acks before any emit() evicts the OLDEST first,
//    keeping only the most recent kAckRingDepth entries, oldest-to-newest
//    order preserved in the wire field -- the ticket's own rapid-fire
//    acceptance property, exercised at the Telemetry-unit level (the
//    bench's own hardware rapid-fire test, src/tests/bench/
//    move_protocol_bench.py, exercises the same property end-to-end over
//    the real link).
// ===========================================================================

void scenarioAckRingEvictsOldestPastDepthAndPreservesOrder() {
  beginScenario("ack(): pushing kAckRingDepth+1 acks evicts the OLDEST (corr=1), keeps the rest in "
                "oldest-to-newest order");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  // Depth is 12 since the command-ingestion rework (command-ingestion-ring-
  // buffered-comms-subsystem-routing-two-stops.md §1): the ring is sized to
  // App::kCmdRingDepth so a burst that fits the command ring also fits
  // here. The PROPERTY under test is unchanged -- push depth+1, watch the
  // oldest go -- so the scenario is driven off kAckRingDepth rather than a
  // hardcoded 4, and no longer needs updating when the depth moves again.
  constexpr uint32_t kDepth = App::kAckRingDepth;
  for (uint32_t i = 1; i <= kDepth + 1; ++i) telemetry.ack(i, 0);  // last push evicts corr=1

  telemetry.emit(0);

  msg::Telemetry expected;
  expected.now = 0;
  expected.seq = 0;
  expected.acks_count = static_cast<uint8_t>(kDepth);
  for (uint32_t i = 0; i < kDepth; ++i) expected.acks_[i] = packAck(i + 2, 0);

  msg::ReplyEnvelope env;
  env.corr_id = 0;
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = expected;
  std::string expectedLine = armorReply(env);
  checkTrue(!expectedLine.empty(), "independent encode+armor of the expected frame succeeds");
  if (!serialFake.sent().empty()) {
    checkStrEq(serialFake.sent()[0], expectedLine,
               "ring holds exactly the last kAckRingDepth pushes, oldest (1) evicted, oldest-to-newest wire order");
  }
}

// ===========================================================================
// 10. Ack ring (120) persists its snapshot across emit() calls with no new
//     ack() in between -- the same "last staged snapshot, not a diff"
//     invariant every other Frame field already has (app/DESIGN.md Sec 3),
//     extended to this new field. Also confirms a ring with FEWER than
//     kAckRingDepth entries reports the correct (small) acks_count, not a
//     padded/zero-filled kAckRingDepth.
// ===========================================================================

void scenarioAckRingPersistsAcrossEmitsBelowFullDepth() {
  beginScenario("ack(): a 2-entry ring (below kAckRingDepth) reports acks_count=2 and persists unchanged across a "
                "later emit() with no new ack()");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  telemetry.ack(10, 0);
  telemetry.ack(20, static_cast<uint32_t>(msg::ErrCode::ERR_FULL));

  telemetry.emit(0);    // frame #1
  telemetry.emit(100);  // frame #2 -- no new ack() call

  checkU64Eq(serialFake.sent().size(), 2, "two successive primary frames were sent");

  for (int f = 0; f < 2; ++f) {
    msg::Telemetry expected;
    expected.now = (f == 0) ? 0u : 100u;
    expected.seq = static_cast<uint32_t>(f);
    expected.acks_count = 2;
    expected.acks_[0] = packAck(10, 0);
    expected.acks_[1] = packAck(20, static_cast<uint32_t>(msg::ErrCode::ERR_FULL));

    msg::ReplyEnvelope env;
    env.corr_id = 0;
    env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
    env.body.tlm = expected;
    std::string expectedLine = armorReply(env);
    if (static_cast<size_t>(f) < serialFake.sent().size()) {
      checkStrEq(serialFake.sent()[static_cast<size_t>(f)], expectedLine,
                 f == 0 ? "frame #1: 2-entry ring, acks_count=2, not padded to depth 4"
                        : "frame #2: ring unchanged (no new ack()), still 2 entries");
    }
  }
}

// ===========================================================================
// 11. Smoke coverage for the new policy state itself (issue Part 3, ticket
//     125-002's own testing note: "at minimum a smoke test per new piece of
//     state"). mode_ defaults kAuto: proven here by construction with NO
//     setMode() call, so a parked robot stays silent (kOff/kAuto both do
//     that) while a MOVING robot streams (only kAuto/kOn do that) --
//     together the two halves pin the default to kAuto specifically. Also
//     doubles as Part 8 acceptance criterion #1 (fresh boot, silent host ->
//     primaryEmitCount()==0 after N cycles of parked idling) and this
//     ticket's own "no arming state" acceptance bar: NO warm-up call of any
//     kind precedes either half.
// ===========================================================================

void scenarioFreshConstructDefaultsAutoAndStaysSilentAtRest() {
  beginScenario("fresh construct: mode() defaults kAuto; a silent, parked robot emits nothing over many cycles "
                "(no arming state, no warm-up needed)");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  checkTrue(telemetry.mode() == App::TlmMode::kAuto, "mode() defaults kAuto immediately after construction");

  // No update() call at all -- proves the class needs no priming: a
  // never-updated instance is exactly as silent as one mid-session.
  for (uint32_t now = 0; now <= 2000; now += App::kPrimaryPeriod) telemetry.emit(now, /*force=*/false);
  checkU64Eq(telemetry.primaryEmitCount(), 0,
             "issue Part 8 #1: fresh construct + parked idling -> primaryEmitCount()==0, no boot-warm-up call "
             "of any kind preceded this");

  // A parked robot that HAS been update()d (connectivity/health bits set,
  // no motion) is equally silent -- the silence is a property of the
  // predicate, not of never having called update().
  Types::RobotState parked;
  parked.wheelLeft.connected = true;
  parked.wheelRight.connected = true;
  telemetry.update(parked);
  for (uint32_t now = 2040; now <= 4000; now += App::kPrimaryPeriod) telemetry.emit(now, /*force=*/false);
  checkU64Eq(telemetry.primaryEmitCount(), 0, "an update()d-but-parked robot is still silent in kAuto");
}

// ===========================================================================
// 12. Hand-spun-wheel / bogus-velocity case (issue Part 3's everMoved_
//     rationale; Part 8 acceptance criterion #5): nonzero staged wheel
//     velocity with kFlagActive never having been true -> everMoved_ stays
//     false forever -> the activity window can never open -> zero frames.
//     This is what makes power-on silence unconditional: a hand-spun wheel
//     on the stand (or a bogus first-sample read, belt-and-suspenders with
//     125-001's Devices::Motor::velocity() two-sample floor) cannot wake
//     the link.
// ===========================================================================

void scenarioHandSpunWheelBeforeFirstMoveNeverWakesTheLink() {
  beginScenario("everMoved_==false: nonzero wheel velocity with no Move ever active -> zero frames, ever");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  Types::RobotState spun;  // moveActive stays false -- never a real Move
  spun.wheelLeft.velocity = 250.0f;   // hand-spun / bogus first-sample reading
  spun.wheelRight.velocity = 250.0f;
  telemetry.update(spun);

  for (uint32_t now = 0; now <= 4000; now += App::kPrimaryPeriod) telemetry.emit(now, /*force=*/false);
  checkU64Eq(telemetry.primaryEmitCount(), 0,
             "issue Part 8 #5: nonzero wheel velocity with everMoved_==false -> zero frames");
}

// ===========================================================================
// 13. lastActivity_ refresh discipline (issue Part 3): kFlagActive alone
//     opens the window; once open, nonzero staged wheel velocity (coasting)
//     keeps refreshing it after kFlagActive drops (a STOP's deceleration
//     tail keeps streaming -- the "velocity frozen mid-decay" harness bug
//     this replaces stays fixed); once wheel velocity reaches zero, the
//     window stays open only until kCoastHoldoff elapses from the LAST
//     refresh, and a stationary-wheel reading can never reopen it once
//     closed (wheels alone never OPEN a window, only kFlagActive does).
// ===========================================================================

void scenarioActivityWindowRefreshesOnActiveAndCoastsThenClosesAfterHoldoff() {
  beginScenario("lastActivity_: kFlagActive opens the window; coasting wheel velocity keeps an open window "
                "alive; the window closes kCoastHoldoff after the last refresh and cannot be reopened by "
                "wheels alone");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  // A Move is active -- opens the window and latches everMoved_.
  Types::RobotState moving;
  moving.time.cycleStart = 0;
  moving.command.moveActive = true;
  moving.wheelLeft.velocity = 200.0f;
  moving.wheelRight.velocity = 200.0f;
  telemetry.update(moving);
  telemetry.emit(0);
  checkU64Eq(telemetry.primaryEmitCount(), 1, "moving: kFlagActive alone is enough to emit");

  // STOP: kFlagActive drops, but the wheels are still coasting (nonzero
  // staged velocity) and the window was already open -- must keep emitting.
  Types::RobotState coasting;
  coasting.time.cycleStart = 40;
  coasting.command.moveActive = false;
  coasting.wheelLeft.velocity = 50.0f;
  coasting.wheelRight.velocity = 50.0f;
  telemetry.update(coasting);
  telemetry.emit(40);
  checkU64Eq(telemetry.primaryEmitCount(), 2,
             "coast-down: nonzero staged velocity with an already-open window keeps the stream alive after "
             "kFlagActive drops");

  // Wheels reach zero -- no further refresh, but the window stays open
  // until kCoastHoldoff elapses from the LAST refresh (now=40 above).
  Types::RobotState stopped;
  stopped.time.cycleStart = 80;
  stopped.command.moveActive = false;
  stopped.wheelLeft.velocity = 0.0f;
  stopped.wheelRight.velocity = 0.0f;
  telemetry.update(stopped);
  telemetry.emit(80);
  checkU64Eq(telemetry.primaryEmitCount(), 3,
             "just past coast: still inside kCoastHoldoff of the last refresh (40), even though wheels are "
             "now at zero");

  // Well past kCoastHoldoff (2000ms) from the last refresh (40) -- the
  // window has closed, and a stationary-wheel reading cannot reopen it.
  telemetry.emit(40 + App::kCoastHoldoff + 100);
  checkU64Eq(telemetry.primaryEmitCount(), 3,
             "well past kCoastHoldoff: the link goes silent, and wheels alone (now reading zero) cannot "
             "reopen a closed window");
}

// ===========================================================================
// 14. setMode() (issue Part 3's public policy surface; ticket 003 owns the
//     wire parsing that calls it, this ticket owns the surface itself):
//     kOff suppresses unsolicited frames even while a Move is active
//     (only force/pendingAckDeliveries() still work); kOn streams even
//     while parked. Exercises the FULL three-way switch in emit() end to
//     end, not just the kAuto default every other scenario in this file
//     already covers.
// ===========================================================================

void scenarioSetModeControlsUnsolicitedStreamInOffAndOn() {
  beginScenario("setMode(): kOff suppresses unsolicited frames even while moving (force still works); kOn "
                "streams even while parked");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  telemetry.setMode(App::TlmMode::kOff);
  checkTrue(telemetry.mode() == App::TlmMode::kOff, "setMode(kOff) takes effect immediately");

  Types::RobotState moving;
  moving.time.cycleStart = 0;
  moving.command.moveActive = true;
  telemetry.update(moving);
  for (uint32_t now = 0; now <= 400; now += App::kPrimaryPeriod) telemetry.emit(now);
  checkU64Eq(telemetry.primaryEmitCount(), 0, "kOff: no unsolicited frames even though a Move is active");

  // Bare TLM/force still works in kOff -- reason 1, honored in every mode.
  telemetry.emit(440, /*force=*/true);
  checkU64Eq(telemetry.primaryEmitCount(), 1, "kOff: a forced request still gets exactly one frame");

  telemetry.setMode(App::TlmMode::kOn);
  Types::RobotState parked;
  parked.time.cycleStart = 440;
  telemetry.update(parked);
  const uint64_t beforeOn = telemetry.primaryEmitCount();
  for (uint32_t now = 480; now <= 800; now += App::kPrimaryPeriod) telemetry.emit(now);
  checkTrue(telemetry.primaryEmitCount() > beforeOn, "kOn: unsolicited frames stream even though the robot "
                                                      "is parked");
}

// ===========================================================================
// 15. THE regression (125-006, issue Part 8 acceptance criterion #2 --
//     explicitly called out as "the single most important test in the whole
//     sprint"): a PARKED robot with line AND color sensors delivering FRESH
//     readings that ALTERNATE every cycle -- exactly the pattern
//     RobotLoop::publishLineColor(tickedLine) produces in production
//     (robot_loop.cpp: "`tickedLine` says which leaf the loop just ticked;
//     the untouched leaf's fresh flag stays false" -- exactly one of
//     {line, color} fresh per cycle, never both, never neither) -- must
//     still emit ZERO frames. The pre-125-002 design could not pass this:
//     kFlagLinePresent/kFlagColorPresent toggling every cycle meant flags_
//     never held still for kBootStableCycles, so report-on-change never
//     armed -- and HAD it armed, that same toggling would have defeated
//     idle silence and streamed at 25 Hz forever (issue's own "Why"
//     paragraph). The rebuilt predicate (issue Part 3) never inspects a
//     Freshness bit at all, so this must pass now.
// ===========================================================================

void scenarioParkedRobotWithAlternatingFreshLineColorStaysSilent() {
  beginScenario("issue Part 8 #2 (THE regression): parked robot, line/color alternating fresh every cycle "
                "(exactly RobotLoop's own publishLineColor() pattern) -- primaryEmitCount() stays 0");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  bool sawLineBitSet = false;
  bool sawLineBitClear = false;
  bool sawColorBitSet = false;
  bool sawColorBitClear = false;

  bool tickedLine = true;  // mirrors robot_loop.cpp's own alternation flip-flop
  for (uint32_t now = 0; now <= 5000; now += App::kPrimaryPeriod) {
    Types::RobotState state;  // moveActive false, wheel velocities 0 -- genuinely parked, never moved
    state.time.cycleStart = now;
    state.wheelLeft.sampleTime = now;
    state.wheelRight.sampleTime = now;
    // Exactly one of {line, color} fresh THIS cycle, alternating -- the
    // real RobotLoop::publishLineColor(tickedLine) pattern, not a
    // simplified stand-in.
    state.perception.lineFresh = tickedLine;
    state.perception.colorFresh = !tickedLine;
    state.perception.line = tickedLine ? 0xAABBCCDDu : 0u;
    state.perception.color = !tickedLine ? 0x11223344u : 0u;
    telemetry.update(state);

    if (telemetry.flags() & App::kFlagLinePresent) {
      sawLineBitSet = true;
    } else {
      sawLineBitClear = true;
    }
    if (telemetry.flags() & App::kFlagColorPresent) {
      sawColorBitSet = true;
    } else {
      sawColorBitClear = true;
    }

    telemetry.emit(now, /*force=*/false);
    tickedLine = !tickedLine;
  }

  checkTrue(sawLineBitSet && sawLineBitClear,
            "sanity: kFlagLinePresent genuinely toggled across cycles -- this scenario really exercises "
            "the alternation, not a simplified stand-in");
  checkTrue(sawColorBitSet && sawColorBitClear, "sanity: kFlagColorPresent genuinely toggled across cycles");
  checkU64Eq(telemetry.primaryEmitCount(), 0,
             "issue Part 8 #2: a parked robot with fresh-alternating line/color sensors emits ZERO frames -- "
             "the structural defect the old design could not pass");
}

// ===========================================================================
// 16. issue Part 8 #3: a command to a parked robot -- its ack rides a frame
//     within 2x kPrimaryPeriod, exactly kAckRepeats frames carry it, then
//     the link falls silent again (pendingAckDeliveries() stops forcing
//     once every ring entry has been carried kAckRepeats times, and a
//     parked kAuto robot has no other reason to emit).
// ===========================================================================

void scenarioAckOnParkedRobotRidesExactlyKAckRepeatsFramesThenSilence() {
  beginScenario("issue Part 8 #3: an ack pushed to a parked robot rides exactly kAckRepeats frames, the "
                "first within 2x kPrimaryPeriod, then the link goes silent again");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  Types::RobotState parked;  // moveActive false, everMoved_ stays false -- purely at rest
  telemetry.update(parked);

  constexpr uint32_t kAckTime = 0;
  telemetry.ack(42, 0);

  uint32_t firstFrameTime = 0;
  bool sawFirstFrame = false;
  for (uint32_t now = kAckTime; now <= 5000; now += App::kPrimaryPeriod) {
    const uint64_t before = telemetry.primaryEmitCount();
    telemetry.emit(now, /*force=*/false);
    if (!sawFirstFrame && telemetry.primaryEmitCount() > before) {
      sawFirstFrame = true;
      firstFrameTime = now;
    }
  }

  checkTrue(sawFirstFrame, "the ack produced at least one frame");
  checkTrue(firstFrameTime <= kAckTime + 2 * App::kPrimaryPeriod,
            "the ack's first carrying frame arrives within 2x kPrimaryPeriod of the ack() call");
  checkU64Eq(telemetry.primaryEmitCount(), App::kAckRepeats,
             "exactly kAckRepeats frames carried the ack, then the parked/kAuto link fell silent again -- "
             "no further frames over 5s of continued cadence gating");
}

// ===========================================================================
// 17. issue Part 8 #6: bare TLM (force=true) on a parked robot produces
//     exactly one frame. TLM:NOW is structurally identical to bare TLM --
//     comms.cpp's dispatchLine() maps BOTH to the same TlmAction::kFrame
//     (app_comms_harness.cpp's own
//     scenarioBareTlmAndTlmNowBothProduceKFrameNoModeChange()), and
//     RobotLoop::cycle() has exactly ONE call site that turns kFrame into
//     tlm_.emit(now, force=true) -- there is no separate code path either
//     could diverge through, so proving force=true here proves both.
// ===========================================================================

void scenarioBareTlmForceOnParkedRobotProducesExactlyOneFrame() {
  beginScenario("issue Part 8 #6: bare TLM/TLM:NOW (force=true) on a parked robot produces exactly one frame");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  Types::RobotState parked;
  telemetry.update(parked);

  for (uint32_t now = 0; now <= 2000; now += App::kPrimaryPeriod) telemetry.emit(now, /*force=*/false);
  checkU64Eq(telemetry.primaryEmitCount(), 0, "setup: parked robot silent before the request");

  telemetry.emit(2040, /*force=*/true);
  checkU64Eq(telemetry.primaryEmitCount(), 1, "bare TLM/TLM:NOW: exactly one frame");

  for (uint32_t now = 2080; now <= 4000; now += App::kPrimaryPeriod) telemetry.emit(now, /*force=*/false);
  checkU64Eq(telemetry.primaryEmitCount(), 1, "no further unsolicited frames after the single requested one");
}

// ===========================================================================
// 18. issue Part 8 #7: TLM:OFF, then a Move runs -- ONLY ack frames appear
//     (enqueue + completion, kAckRepeats each), no stream, even though
//     kFlagActive is set the whole time the Move runs; a bare TLM request
//     still answers with one frame on top of the ack-carried total.
// ===========================================================================

void scenarioTlmOffThenMoveOnlyAckFramesNoStream() {
  beginScenario("issue Part 8 #7: TLM:OFF, then a Move runs -- ONLY ack frames appear (enqueue + completion, "
                "kAckRepeats each), no stream while moving; a bare TLM still answers with one frame");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  telemetry.setMode(App::TlmMode::kOff);

  // Enqueue ack -- mirrors RobotLoop's own handleMove(), which acks the
  // instant a Move is accepted, before it starts moving.
  telemetry.ack(/*corrId=*/100, 0);

  uint32_t now = 0;
  Types::RobotState moving;
  moving.command.moveActive = true;
  // Many cycles of the Move actually running -- kOff must produce NO
  // unsolicited frames despite kFlagActive being set every one of them,
  // only the pending enqueue ack's own bounded kAckRepeats burst.
  for (int i = 0; i < 20; ++i, now += App::kPrimaryPeriod) {
    moving.time.cycleStart = now;
    telemetry.update(moving);
    telemetry.emit(now, /*force=*/false);
  }
  checkU64Eq(telemetry.primaryEmitCount(), App::kAckRepeats,
             "kOff + moving: only the enqueue ack's own kAckRepeats burst, no stream");

  // Completion ack -- mirrors RobotLoop's own moveQueue completion
  // (tlm_.ack(moveId, 0)) call site.
  telemetry.ack(/*corrId=*/100, 0);
  Types::RobotState parked;  // the Move completed -- moveActive drops
  for (int i = 0; i < 20; ++i, now += App::kPrimaryPeriod) {
    parked.time.cycleStart = now;
    telemetry.update(parked);
    telemetry.emit(now, /*force=*/false);
  }
  checkU64Eq(telemetry.primaryEmitCount(), 2 * App::kAckRepeats,
             "kOff: the completion ack adds exactly ANOTHER kAckRepeats frames, still no unsolicited stream");

  // A bare TLM request still answers with one frame, on top of the
  // ack-carried total -- reason 1 (force) is honored in EVERY mode.
  telemetry.emit(now, /*force=*/true);
  checkU64Eq(telemetry.primaryEmitCount(), 2 * App::kAckRepeats + 1,
             "a bare TLM request in kOff still answers with exactly one frame");
}

// ===========================================================================
// 19. issue Part 8 #8: TLM:ON streams a parked robot at cadence; TLM:OFF
//     stops the stream within one kPrimaryPeriod (the very next cadence
//     tick after the mode change produces no frame); TLM:AUTO restores the
//     default mode-2 (silent-when-parked) behavior.
// ===========================================================================

void scenarioTlmOnThenOffStopsWithinOnePeriodThenAutoRestoresModeTwoBehavior() {
  beginScenario("issue Part 8 #8: TLM:ON streams a parked robot at cadence; TLM:OFF stops it within one "
                "kPrimaryPeriod; TLM:AUTO restores the default (silent-when-parked) behavior");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  Types::RobotState parked;
  telemetry.update(parked);

  telemetry.setMode(App::TlmMode::kOn);
  uint32_t now = 0;
  for (; now <= 800; now += App::kPrimaryPeriod) telemetry.emit(now, /*force=*/false);
  const uint64_t onCount = telemetry.primaryEmitCount();
  checkTrue(onCount > 15, "kOn: a parked robot streams at cadence (roughly one frame per kPrimaryPeriod)");

  telemetry.setMode(App::TlmMode::kOff);
  now += App::kPrimaryPeriod;
  telemetry.emit(now, /*force=*/false);
  checkU64Eq(telemetry.primaryEmitCount(), onCount,
             "kOff: the very next cadence tick after the mode change produces no frame -- the stream "
             "stopped within one kPrimaryPeriod");
  for (int i = 0; i < 10; ++i) {
    now += App::kPrimaryPeriod;
    telemetry.emit(now, /*force=*/false);
  }
  checkU64Eq(telemetry.primaryEmitCount(), onCount, "kOff: stays silent, parked");

  telemetry.setMode(App::TlmMode::kAuto);
  const uint64_t offCount = telemetry.primaryEmitCount();
  for (int i = 0; i < 20; ++i) {
    now += App::kPrimaryPeriod;
    telemetry.emit(now, /*force=*/false);
  }
  checkU64Eq(telemetry.primaryEmitCount(), offCount,
             "kAuto: restores mode-2 (silent-when-parked) behavior -- a parked robot stays silent under "
             "kAuto exactly like it did before it was ever switched kOn");
}

// ===========================================================================
// 20. issue Part 8 #10: mode is not persistent. Set kOn (or kOff), then a
//     FRESH Telemetry construction -- the only "reboot" semantics
//     App::Telemetry has, since mode_ is a plain member with no persistence
//     store at all (issue Part 4's explicit "no persistence" decision) --
//     reads back kAuto. See app_robot_loop_harness.cpp's own note (moved
//     125-006) for the wire-level ("TLM:ON" then simulated reboot) version
//     of the same property.
// ===========================================================================

void scenarioModeNotPersistedAcrossFreshConstruction() {
  beginScenario("issue Part 8 #10: mode is not persistent -- set kOn (or kOff), then a fresh Telemetry "
                "construction (the only 'reboot' semantics it has) reads back kAuto");

  FakeTransport serialFake1;
  FakeTransport radioFake1;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms1(serialFake1, radioFake1, banner);
  App::Telemetry telemetry1(comms1);
  telemetry1.setMode(App::TlmMode::kOn);
  checkTrue(telemetry1.mode() == App::TlmMode::kOn, "setup: kOn took effect on the first instance");

  // "Simulate reboot": a fresh Comms/Telemetry construction -- nothing
  // written to config, nothing carried over. This IS what a power cycle
  // does in production (mode_ resets to kAuto by construction every boot,
  // issue Part 3).
  FakeTransport serialFake2;
  FakeTransport radioFake2;
  App::Comms comms2(serialFake2, radioFake2, banner);
  App::Telemetry telemetry2(comms2);
  checkTrue(telemetry2.mode() == App::TlmMode::kAuto,
            "a fresh boot always starts kAuto, regardless of the prior session's kOn/kOff");

  telemetry1.setMode(App::TlmMode::kOff);
  FakeTransport serialFake3;
  FakeTransport radioFake3;
  App::Comms comms3(serialFake3, radioFake3, banner);
  App::Telemetry telemetry3(comms3);
  checkTrue(telemetry3.mode() == App::TlmMode::kAuto, "same for kOff: a fresh boot forgets it too");
}

// ===========================================================================
// 21. applyAction() (128-012: absorbs RobotLoop::cycle()'s previous inline
//     TlmAction switch AND the force-frame decision). Every arm of
//     Comms::TlmAction, driven directly against a fresh Telemetry -- no
//     Comms::takeTlmAction()/RobotLoop involved, matching this ticket's own
//     "synthesized action, no full loop tick needed" testing note.
// ===========================================================================

void scenarioApplyActionCoversEveryTlmActionArm() {
  beginScenario("applyAction(): every Comms::TlmAction arm sets the right mode (or none) and returns the "
                "correct force-frame answer");

  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:test:1234";
  App::Comms comms(serialFake, radioFake, banner);
  App::Telemetry telemetry(comms);

  checkTrue(telemetry.mode() == App::TlmMode::kAuto, "setup: fresh Telemetry defaults to kAuto");

  checkTrue(!telemetry.applyAction(App::Comms::TlmAction::kSetOff), "kSetOff: not a force-frame request");
  checkTrue(telemetry.mode() == App::TlmMode::kOff, "kSetOff -> mode() == kOff");

  checkTrue(!telemetry.applyAction(App::Comms::TlmAction::kSetOn), "kSetOn: not a force-frame request");
  checkTrue(telemetry.mode() == App::TlmMode::kOn, "kSetOn -> mode() == kOn");

  checkTrue(!telemetry.applyAction(App::Comms::TlmAction::kSetAuto), "kSetAuto: not a force-frame request");
  checkTrue(telemetry.mode() == App::TlmMode::kAuto, "kSetAuto -> mode() == kAuto");

  // kNone/kFrame/kUnrecognized never change mode -- set kOn first so a
  // spurious mode change would be visible, not masked by an already-matching
  // default.
  telemetry.setMode(App::TlmMode::kOn);
  checkTrue(!telemetry.applyAction(App::Comms::TlmAction::kNone), "kNone: not a force-frame request");
  checkTrue(telemetry.mode() == App::TlmMode::kOn, "kNone: mode unchanged");

  checkTrue(!telemetry.applyAction(App::Comms::TlmAction::kUnrecognized),
            "kUnrecognized: not a force-frame request");
  checkTrue(telemetry.mode() == App::TlmMode::kOn, "kUnrecognized: mode unchanged");

  // kFrame is the ONE arm that answers true -- a bare TLM/TLM:NOW line --
  // and it too leaves mode_ untouched (issue Part 3: the force decision and
  // the mode-change decision are orthogonal).
  checkTrue(telemetry.applyAction(App::Comms::TlmAction::kFrame), "kFrame: IS a force-frame request");
  checkTrue(telemetry.mode() == App::TlmMode::kOn, "kFrame: mode unchanged");

  // The returned bool is exactly what the next emit() call needs to force a
  // frame past the mode gate -- proven end to end here rather than just
  // asserting the bool in isolation, since that IS the contract RobotLoop
  // depends on (telemetry.h's own doc comment: "the returned bool is the
  // very next emit() call's own `force` argument").
  FakeTransport serialFake2;
  FakeTransport radioFake2;
  App::Comms comms2(serialFake2, radioFake2, banner);
  App::Telemetry telemetry2(comms2);
  telemetry2.setMode(App::TlmMode::kOff);
  Types::RobotState parked;
  parked.time.cycleStart = 0;
  telemetry2.update(parked);
  const bool force = telemetry2.applyAction(App::Comms::TlmAction::kFrame);
  telemetry2.emit(0, force);
  checkU64Eq(telemetry2.primaryEmitCount(), 1,
             "end-to-end: applyAction(kFrame)'s returned bool forces exactly one frame even in kOff");
}

}  // namespace

int main() {
  scenarioUpdateProjectsWholeStateInOneCall();
  scenarioAgeIsComputedIndependentlyPerReading();
  scenarioAckRingCarriesEveryPushAndPersistsAcrossEmits();
  scenarioFlagsAreFreshlyDerivedFromStateEveryUpdate();
  scenarioSetLiveFlagMutatesLiveFlagsIndependentlyOfUpdate();
  scenarioFullyPopulatedPrimaryFrameFitsRecordedWorstCase();
  scenarioMeasuredCadenceReport();
  scenarioMalformedFrameSetsCommsMalformedFlagBit();
  scenarioAckRingEvictsOldestPastDepthAndPreservesOrder();
  scenarioAckRingPersistsAcrossEmitsBelowFullDepth();
  scenarioFreshConstructDefaultsAutoAndStaysSilentAtRest();
  scenarioHandSpunWheelBeforeFirstMoveNeverWakesTheLink();
  scenarioActivityWindowRefreshesOnActiveAndCoastsThenClosesAfterHoldoff();
  scenarioSetModeControlsUnsolicitedStreamInOffAndOn();
  scenarioParkedRobotWithAlternatingFreshLineColorStaysSilent();
  scenarioAckOnParkedRobotRidesExactlyKAckRepeatsFramesThenSilence();
  scenarioBareTlmForceOnParkedRobotProducesExactlyOneFrame();
  scenarioTlmOffThenMoveOnlyAckFramesNoStream();
  scenarioTlmOnThenOffStopsWithinOnePeriodThenAutoRestoresModeTwoBehavior();
  scenarioModeNotPersistedAcrossFreshConstruction();
  scenarioApplyActionCoversEveryTlmActionArm();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Telemetry scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Telemetry scenarios\n", g_failureCount);
  return 1;
}
