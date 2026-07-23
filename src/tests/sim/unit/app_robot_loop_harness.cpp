// app_robot_loop_harness.cpp -- off-hardware acceptance harness for ticket
// 105-001 (SUC-018), App::RobotLoop (src/firm/app/robot_loop.{h,cpp}), the
// boot loop + main cycle body extracted from src/firm/main.cpp.
//
// Proves the extraction is genuinely host-buildable and runnable: constructs
// every leaf/app module RobotLoop needs over a TestSim::SimPlant (108-002),
// scripted deterministically via TestSim::ScriptedI2CHook (108-009), and a
// TestSim::SimClock/SimSleeper pair (tests/_infra/sim/sim_clock.cpp, the
// Devices::Clock/Sleeper host-test fakes -- sprint 108 ticket 010), a
// minimal App::Transport stub in place of real serial/radio, drives boot()
// to completion, then cycle() a few times, and asserts: no bus script
// under/over-run (proves cycle ordering matches what main.cpp's
// pre-extraction body actually issued), encoder-derived position()/
// velocity() reflect the scripted samples (proves the request/settle/collect
// timing survived the move), the Sleeper recorded a sleep for every
// runAndWait/pace point (proves markTime()/sleepUntil() now route through
// Devices::Clock/Sleeper, not system_timer_current_time()/uBit.sleep()), and
// Telemetry emitted real bytes on both the primary and (given enough
// cycles) secondary cadence. No MicroBit.h is included by this file,
// robot_loop.h, or robot_loop.cpp -- compiled with -DHOST_BUILD against the
// same headers the ARM build uses.
//
// Migrated by sprint 108 ticket 009 off the deleted src/firm/devices/
// i2c_bus_host.cpp scripted-FIFO Devices::I2CBus fake (ticket 001 reduced
// Devices::I2CBus to a pure interface and removed it) -- see
// devices_motor_harness.cpp's/scripted_i2c_hook.h's own header for the
// migration rationale. Every scenario below is otherwise UNCHANGED from the
// pre-migration harness -- only the bus/scripting plumbing moved. This
// harness needs exact, cycle-by-cycle register control (an exact
// write/read transaction budget across two motors + OTOS interleaved on one
// bus, a specific CONFIG-dispatch ack-ring fingerprint) that SimPlant's own
// live physics responses cannot give directly -- so it stays a small C++
// SimPlant-hook harness rather than a pure-Python SimHarness/SimLoop test
// (ticket 009's own documented fallback for scenarios needing
// host-unobservable, exact register-level control). Note ticket 108-003's
// TestSim::SimHarness/src/tests/sim/system/ now cover the SAME "whole loop end
// to end" ground with the REAL, live-responding SimPlant physics (no
// scripting) -- this harness's own value is the exact register-level
// transaction-budget/CONFIG-ack proof those system tests don't attempt.
//
// Mirrors app_preamble_harness.cpp's exact scripting/assertion-plumbing
// conventions (this codebase's established per-harness-file style).
//
// 115-005 (gut S1) update: App::Pilot/Motion::Executor/App::HeadingSource
// are deleted along with the rest of the motion stack -- RobotLoop's
// constructor drops the Pilot& parameter and gains
// Devices::ColorSensorLeaf&/Devices::LineSensorLeaf& (already-constructed
// leaves this harness's fixtures already build for Preamble, now ALSO
// wired directly into RobotLoop for its own rate-limited line/color
// polling). The CONFIG-dispatch scenario's PlannerConfigPatch injection is
// deleted (msg::PlannerConfigPatch no longer exists); its ack-content
// checks are rewritten against the new flat ack_corr/ack_err fields
// (single ack slot replaces the old depth-3 AckEntry ring -- see
// ackFingerprint()'s own updated comment below).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>

#include "app/comms.h"
#include "app/drive.h"
#include "app/move_queue.h"
#include "app/odometry.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/state_estimator.h"
#include "app/telemetry.h"
#include "config/persisted_tuning.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/line_sensor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "messages/wire_runtime.h"
#include "scripted_i2c_hook.h"
#include "sim_clock.h"
#include "sim_plant.h"
#include "support/bench_test_config.h"
#include "support/fake_transport.h"
#include "wire_test_codec.h"

namespace {

// --- Hand-rolled assertion plumbing (see app_preamble_harness.cpp) --------

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

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-3f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

// kCycleDtUs -- 118 ticket 003 (sim-cycle-must-match-firmware-period.md):
// single source of truth for every hand-rolled "tick the plant, advance the
// clock, cycle() the loop" step loop in this file (LiveFixture::step() below
// plus the standalone scenario functions that build their own local
// plant/clock/robotLoop instead of using LiveFixture). Derived from
// App::RobotLoop::kCycle (robot_loop.h), NOT an independently-hardcoded
// matching literal -- exactly the class of drift this ticket exists to
// close off (matches TestSim::SimHarness::kCycleDtUs's own derivation,
// sim_harness.h).
constexpr uint32_t kCycleDtUs = App::RobotLoop::kCycle * 1000;  // [us]

// --- Minimal App::Transport stub -- no real serial/radio, ever. readLine()
// always reports "nothing ready" (this harness does not need to inject
// commands to prove the extraction runs); send()/sendReliable() just record
// whether anything was ever sent, proving Telemetry's own emit() path is
// reachable end-to-end through Comms with no MicroBit.h dependency. -------
class NullTransport : public App::Transport {
 public:
  bool readLine(char*, uint16_t) override { return false; }
  void send(const char*) override { ++sendCount; }
  void sendReliable(const char*) override { ++sendCount; }

  int sendCount = 0;
};

// --- MockTuningStore (114-004) -- a trivial, no-flash Config::TuningStore
// double: records every save() call (count + last blob/version) without
// touching any hardware. Proves handleConfig()'s write-policy
// (persistTuningIfChanged()'s change-detection debounce) via a call-count
// assertion on this seam, per the ticket's own Testing section ("mock or
// count-based assertion on the persistence seam, not real flash") --
// Config::TuningStore is a plain C++ virtual base with zero hardware
// dependency (unlike Config::MicroBitTuningStore, its one real
// implementation), so it is directly mockable under HOST_BUILD even
// though the real ARM adapter is not exercised by any agent-run test. ---
class MockTuningStore : public Config::TuningStore {
 public:
  bool load(uint32_t*, Config::Blob*) override { return false; }  // "never written" -- not this test's concern

  void save(uint32_t version, const Config::Blob& blob) override {
    ++saveCount;
    lastVersion = version;
    lastBlob = blob;
  }

  void wipe() override { ++wipeCount; }

  int saveCount = 0;
  int wipeCount = 0;
  uint32_t lastVersion = 0;
  Config::Blob lastBlob{};
};

// --- Fixture helpers (mirrors app_preamble_harness.cpp) --------------------

constexpr uint16_t kMotorWireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
constexpr uint16_t kOtosWireAddr = static_cast<uint16_t>(Devices::kOtosDeviceAddr << 1);
constexpr uint16_t kColorAltWireAddr = static_cast<uint16_t>(Devices::kColorDeviceAddrAlt << 1);
constexpr uint16_t kLineWireAddr = static_cast<uint16_t>(Devices::kLineDeviceAddr << 1);

Devices::MotorConfig baseMotorConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  cfg.velFiltAlpha = 1.0f;  // no smoothing -- velocity() reflects the raw difference-quotient exactly
  return cfg;
}

void scriptEncoderCall(TestSim::ScriptedI2CHook& bus, int writeStatus, int readStatus) {
  bus.queueWrite(kMotorWireAddr, writeStatus);
  uint8_t data[4] = {0, 0, 0, 0};
  bus.queueRead(kMotorWireAddr, data, 4, readStatus);
}

// begin()'s hardReset(): 4 calls (3 median snapshots + 1 readback), all
// succeeding, all reading back raw=0 -- see app_preamble_harness.cpp's
// identical helper for the full derivation.
void scriptMotorBeginSuccess(TestSim::ScriptedI2CHook& bus) {
  for (int i = 0; i < 4; ++i) scriptEncoderCall(bus, /*writeStatus=*/0, /*readStatus=*/0);
}

void scriptOtosBeginSuccess(TestSim::ScriptedI2CHook& bus) {
  for (int i = 0; i < 7; ++i) bus.queueWrite(kOtosWireAddr, 0);
  uint8_t id[1] = {0x5F};
  bus.queueRead(kOtosWireAddr, id, 1, 0);
}

void scriptColorBeginSuccess(TestSim::ScriptedI2CHook& bus) {
  bus.queueWrite(kColorAltWireAddr, 0);  // writeReg8(0x81, 0xCA)
  bus.queueWrite(kColorAltWireAddr, 0);  // writeReg8(0x80, 0x17)
  bus.queueWrite(kColorAltWireAddr, 0);  // readReg16Alt(0xA4) lo-byte select
  bus.queueWrite(kColorAltWireAddr, 0);  // readReg16Alt(0xA4) hi-byte select
  uint8_t lo[1] = {0x34};
  uint8_t hi[1] = {0x12};
  bus.queueRead(kColorAltWireAddr, lo, 1, 0);
  bus.queueRead(kColorAltWireAddr, hi, 1, 0);
}

void scriptLineBeginSuccess(TestSim::ScriptedI2CHook& bus) {
  for (int ch = 0; ch < 4; ++ch) {
    bus.queueWrite(kLineWireAddr, 0);
    uint8_t data[1] = {100};
    bus.queueRead(kLineWireAddr, data, 1, 0);
  }
}

// One request(write)+collect(4-byte read) pair for a single motor, plus
// `extraDutyWrites` additional writes (0 or 1) IF this exact call is
// expected to also land a duty write this cycle.
//
// Unlike a single-device harness (e.g. devices_motor_harness.cpp's own
// scriptEncoderRequestCollect(), which over-provisions one "slack" write
// per cycle because an unconsumed entry simply rolls forward into that
// SAME motor's own next cycle, harmlessly), RobotLoop::cycle() interleaves
// TWO motors AND the OTOS burst read on the SAME global write/read FIFO
// (i2c_bus_host.cpp's own file-header: queueWrite()/queueRead() are each
// ONE queue shared across every address, matched by address only) -- an
// over-provisioned "maybe" write left unconsumed at a motor's own address
// does NOT roll forward harmlessly here; it gets wrongly popped by the
// NEXT real call to any address, including OTOS's own burst-read write,
// corrupting that leaf's errCount(). Exact counts are required.
//
// Empirically derived (via a standalone NezhaMotor+Drive-free probe, then
// confirmed against this exact scenario's own dumpRecent() log) and
// explained by RobotLoop::cycle()'s own real ordering: drive_.tick() (which
// calls both leaves' setVelocity(), transitioning mode_ from its
// pre-Drive-wiring default to Active) runs BETWEEN motorL_.tick() and
// motorR_.tick() within a SINGLE cycle() call (see robot_loop.cpp's own
// cycle() body). So each leaf's PID/duty dispatch activates ONE CYCLE
// APART: R's mode_ is already Active by the time motorR_.tick() runs on
// cycle 0 (drive_.tick() ran moments earlier, same cycle) -- R issues its
// OWN one-time "first write" (lastWrittenPct_'s -128 sentinel, pct=0) THAT
// cycle. L's motorL_.tick() runs BEFORE drive_.tick() has EVER executed on
// cycle 0 (mode_ still at its pre-Drive default, no dispatch at all) -- L's
// own one-time first write is deferred to cycle 1, once mode_ is Active
// from cycle 0's drive_.tick() call. Both gains are zero (baseMotorConfig()
// leaves velGains defaulted), so duty is deterministically 0 forever after
// each leaf's own first write -- write-on-change then skips every
// subsequent cycle for that leaf, matching this scenario's own 3/3/2
// write-count schedule below.
void scriptMotorCycle(TestSim::ScriptedI2CHook& bus, float positionMm, int extraDutyWrites) {
  bus.queueWrite(kMotorWireAddr, /*status=*/0);  // requestEncoder()'s 0x46 write
  for (int i = 0; i < extraDutyWrites; ++i) {
    bus.queueWrite(kMotorWireAddr, /*status=*/0);  // this leaf's own one-time first duty write
  }
  int32_t raw = static_cast<int32_t>(positionMm * 10.0f);
  uint8_t data[4] = {
      static_cast<uint8_t>(raw & 0xFF),
      static_cast<uint8_t>((raw >> 8) & 0xFF),
      static_cast<uint8_t>((raw >> 16) & 0xFF),
      static_cast<uint8_t>((raw >> 24) & 0xFF),
  };
  bus.queueRead(kMotorWireAddr, data, 4, /*status=*/0);
}

// One 1-write + 12-byte-read OTOS burst (Otos::readPositionVelocity()) --
// only needed on the FIRST cycle() call after begin() (hasRead_ starts
// false, so the very next tick() is unconditionally due); every later
// cycle() call in this harness runs at the SAME frozen fake-Clock value, so
// (nowUs - lastReadUs_) stays 0 < kReadPeriod and Otos::tick() performs zero
// bus traffic -- no further scripting needed for a "few cycles" smoke test.
void scriptOtosReadZeroPose(TestSim::ScriptedI2CHook& bus) {
  bus.queueWrite(kOtosWireAddr, /*status=*/0);
  uint8_t raw[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  bus.queueRead(kOtosWireAddr, raw, 12, /*status=*/0);
}

// --- Hand-rolled wire-byte builder (mirrors app_comms_harness.cpp's own
// Buf/putVarintField/putFloatField/putMessageField/armor -- lets this
// harness construct an arbitrary INBOUND CommandEnvelope{config: ...} line
// without depending on a generic encode(CommandEnvelope), which does not
// exist -- firmware only ever DECODES a CommandEnvelope). Used by the
// 106-002 CONFIG-dispatch scenario below to build the three ConfigDelta
// patch kinds and to search a captured OUTBOUND reply's raw bytes for a
// specific AckEntry{corr_id, status, err_code} sub-message (the same
// WireRuntime primitives wire.cpp's generated encode()/decode() are built
// on, so a byte-identical field encodes identically no matter which side
// produced it). ---------------------------------------------------------

using WireRuntime::WireType;

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

std::string armorLine(const uint8_t* raw, size_t rawLen) {
  char b64[512] = {};
  size_t b64Len = 0;
  bool ok = WireRuntime::base64Encode(raw, rawLen, b64, sizeof(b64), &b64Len);
  if (!ok) return std::string();
  std::string out = "*B";
  out.append(b64, b64Len);
  return out;
}

// De-armors a captured "*B<base64>" outbound line back into raw protobuf
// bytes -- the inverse of armorLine(), needed only because no
// decode(ReplyEnvelope) codec exists (app_telemetry_harness.cpp's own file
// header note): a substring search over these raw bytes is this harness's
// way of confirming a specific ack landed, without reconstructing the
// entire frame's other fields.
std::string rawBytesFromArmoredLine(const std::string& line) {
  if (line.size() < 2 || line[0] != '*' || line[1] != 'B') return std::string();
  uint8_t raw[App::kMaxEnvelopeBytes] = {};
  size_t rawLen = 0;
  bool ok = WireRuntime::base64Decode(line.c_str() + 2, line.size() - 2, raw, sizeof(raw), &rawLen);
  if (!ok) return std::string();
  return std::string(reinterpret_cast<const char*>(raw), rawLen);
}

bool containsSubBytes(const std::string& haystack, const Buf& needle) {
  if (needle.len == 0 || needle.len > haystack.size()) return false;
  return haystack.find(std::string(reinterpret_cast<const char*>(needle.data), needle.len)) != std::string::npos;
}

// 115-005: the depth-3 AckEntry ring is gone -- Telemetry's single ack slot
// rides directly as msg::Telemetry's own ack_corr (field 5)/ack_err (field
// 6) scalars, adjacent in field-number order with nothing else able to
// land between them (both are always-present, non-Opt<T> fields --
// telemetry.proto's own field list), so their two encoded tag+varint pairs,
// concatenated in that fixed order, are still a reliable, low-false-
// positive fingerprint of "ack_corr/ack_err currently hold this exact
// pair" wherever it appears in a raw decoded Telemetry frame -- the same
// technique the old AckEntry-submessage fingerprint used, just against two
// top-level scalars instead of one ring entry's three.
Buf ackFingerprint(uint32_t corrId, uint32_t errCode) {
  Buf b;
  putVarintField(b, 5, corrId);   // Telemetry.ack_corr
  putVarintField(b, 6, errCode);  // Telemetry.ack_err
  return b;
}

// ===========================================================================
// Boot resolves all 5 devices, then a few main cycles run to completion with
// no bus script under/over-run, encoder-derived position()/velocity()
// reflecting the scripted samples, and the Sleeper/Telemetry seams both
// exercised -- the ticket's own acceptance criterion ("steps it through
// boot + a few cycles, and confirms it runs to completion with no
// MicroBit.h dependency anywhere in the compiled translation units").
// ===========================================================================

void scenarioBootThenAFewCyclesRunToCompletion() {
  beginScenario("RobotLoop: boot() resolves all devices, cycle() runs a few passes cleanly");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  TestSim::SimClock clock;
  TestSim::SimSleeper sleeper;

  Devices::NezhaMotor motorL(plant, baseMotorConfig(1));
  Devices::NezhaMotor motorR(plant, baseMotorConfig(2));
  Devices::Otos otos(plant, Devices::OtosConfig{});
  Devices::ColorSensorLeaf color(plant, Devices::ColorConfig{});
  Devices::LineSensorLeaf line(plant, Devices::LineConfig{});

  NullTransport serialLink;
  NullTransport radioLink;
  App::Comms comms(serialLink, radioLink, "DEVICE:NEZHA2:robot:test:0");
  App::Telemetry tlm(comms, serialLink, radioLink);
  App::Drive drive(motorL, motorR, /*trackWidth=*/120.0f);
  App::Odometry odom(motorL, motorR, /*trackWidth=*/120.0f);
  // App::StateEstimator -- default-constructed (0/0/200ms weights). 118
  // ticket 004: QUARANTINED -- App::MoveQueue no longer depends on this
  // member (its own former anticipation-lead completion path is deleted,
  // move_queue.h's own file header); kept solely for robotLoop's own
  // consumption below.
  App::StateEstimator stateEstimator;
  App::MoveQueue moveQueue(drive, odom, clock);
  App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  // --- Drive Preamble to done() BEFORE constructing/calling RobotLoop's
  // own boot() -- see this file's own derivation (robot_loop.cpp's boot()
  // has no way to advance a HOST_BUILD fake Clock itself, exactly like the
  // real ARM Clock never self-advances except via the hardware timer; the
  // power-settle gate's "elapsed since Preamble's OWN first step() call"
  // check would otherwise never pass inside one un-interrupted boot() call
  // against a Clock that only moves when a test explicitly steps it). The
  // FIRST step() call below reproduces app_preamble_harness.cpp's own
  // "power-settle no-op" call at nowUs=0; advancing the clock to 50000
  // (>= Preamble's kPowerSettle) BEFORE robotLoop.boot() lets every
  // remaining slot resolve on its first attempt, all at that SAME frozen
  // timestamp -- exactly app_preamble_harness.cpp's own all-present
  // happy-path scenario, just driven through RobotLoop::boot() instead of
  // Preamble::step() directly. ---
  clock.setMicros(0);
  preamble.step();  // power-settle no-op; latches Preamble's own start time
  checkTrue(!preamble.done(), "not done after the power-settle no-op call");

  clock.setMicros(50000);
  scriptMotorBeginSuccess(bus);  // Left
  scriptMotorBeginSuccess(bus);  // Right
  scriptOtosBeginSuccess(bus);
  scriptColorBeginSuccess(bus);
  scriptLineBeginSuccess(bus);

  App::RobotLoop robotLoop(plant, motorL, motorR, otos, color, line, comms, tlm,
                            drive, odom, moveQueue, preamble, stateEstimator, clock, sleeper);

  int sleepsBeforeBoot = sleeper.sleepCount();
  robotLoop.boot();

  checkTrue(preamble.done(), "boot() returns once preamble.done() is true");
  checkTrue(preamble.leftConnected(), "leftConnected() true after boot()");
  checkTrue(preamble.rightConnected(), "rightConnected() true after boot()");
  checkTrue(preamble.otosConnected(), "otosConnected() true after boot()");
  checkTrue(sleeper.sleepCount() > sleepsBeforeBoot,
            "boot()'s own pacing sleep (kPreamblePace) went through Devices::Sleeper, "
            "not uBit.sleep()");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run: motor (boot)");
  checkUintEq(bus.errCount(Devices::kOtosDeviceAddr), 0, "no script under-run: otos (boot)");

  // --- A few main cycles. Encoder positions climb by 10mm per cycle on
  // both wheels (a plain forward roll) -- proves requestSample()/tick()'s
  // request->settle->collect timing survived the extraction unchanged, and
  // that Odometry/applyOtosSample still run every cycle in the same order
  // main.cpp's pre-extraction for(;;) body did. ---
  constexpr int kCycles = 3;
  for (int i = 0; i < kCycles; ++i) {
    float positionMm = static_cast<float>(i + 1) * 10.0f;
    // R gets its own one-time first duty write on cycle 0; L gets its own
    // one-time first duty write on cycle 1 (one cycle later than R) -- see
    // scriptMotorCycle()'s own header comment for the full derivation.
    scriptMotorCycle(bus, positionMm, /*extraDutyWrites=*/(i == 1) ? 1 : 0);  // Left
    scriptMotorCycle(bus, positionMm, /*extraDutyWrites=*/(i == 0) ? 1 : 0);  // Right
    if (i == 0) scriptOtosReadZeroPose(bus);  // Otos::tick()'s hasRead_==false on cycle 1 only

    int sleepsBefore = sleeper.sleepCount();
    robotLoop.cycle();
    checkTrue(sleeper.sleepCount() > sleepsBefore,
              "cycle() paces via Devices::Sleeper (three runAndWait blocks + final "
              "sleepUntil), not uBit.sleep()");
  }

  checkFloatEq(motorL.position(), 30.0f, "left position reflects the 3rd scripted cycle's sample");
  checkFloatEq(motorR.position(), 30.0f, "right position reflects the 3rd scripted cycle's sample");
  checkTrue(motorL.connected(), "left motor still connected after 3 cycles");
  checkTrue(motorR.connected(), "right motor still connected after 3 cycles");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run: motor (cycles)");
  checkUintEq(bus.errCount(Devices::kOtosDeviceAddr), 0, "no script under-run: otos (cycles)");

  checkTrue(tlm.primaryEmitCount() > 0, "Telemetry emitted at least one primary frame");
  checkTrue(serialLink.sendCount > 0, "Comms actually sent bytes over the (fake) transport");
}

// ===========================================================================
// 106-002/SUC-025 (115-005 rewrite): RobotLoop::cycle()'s CONFIG dispatch
// live-applies a MotorConfigPatch (both bound motors, acks OK) while
// DrivetrainConfigPatch continues acking ERR_UNIMPLEMENTED, unchanged.
// PlannerConfigPatch (109-008's un-stub) is GONE -- 115-005 (gut S1)
// deleted msg::PlannerConfigPatch and ConfigDelta's own PLANNER arm along
// with the rest of the motion stack; this scenario no longer injects one.
//
// A "quiet" fixture (robot never twisted, stays at encoder position 0 the
// whole run) so the ONLY thing that varies cycle to cycle is the ack slot --
// the motor "applies" half is proven directly by reading motorL/motorR's own
// gains() (owned by this test, passed into RobotLoop by reference, so no
// wire decoding is needed for that half at all); the drivetrain half is
// proven by a raw-byte substring search for the ack_corr/ack_err fingerprint
// in a captured outbound frame (no decode(ReplyEnvelope) codec exists --
// see rawBytesFromArmoredLine()'s own comment) -- deliberately NOT a
// full-frame byte-equality check (app_telemetry_harness.cpp's own
// technique), since this fixture makes no claim about the other frame
// fields' exact values.
//
// Single ack slot (115-005: replaces the old depth-3 AckEntry ring --
// ack-depth-1 is a stakeholder-accepted tradeoff): an ack pushed during
// cycle N's dispatch OVERWRITES whatever the slot held before, and is not
// visible in ANY emitted frame until cycle N+1's own emit (which runs
// BEFORE that cycle's own dispatch -- robot_loop.cpp's own cycle()
// ordering). Motor is dispatched FIRST and its own ack captured BEFORE
// drivetrain's dispatch overwrites the slot -- captureNextPrimaryLine()
// below runs a small BOUNDED number of extra quiet cycles if needed
// (106-002's own tie-break can occasionally divert a single call's emit()
// to the secondary frame instead -- telemetry.h's own emit() comment: "at
// most one primary frame delayed by one loop cycle roughly once per
// kSecondaryPeriod"), so this is deterministic regardless of exactly which
// cycle a tie lands on.
// ===========================================================================

void scenarioConfigMotorAppliesWhileDrivetrainStaysUnimplemented() {
  beginScenario("RobotLoop CONFIG: MotorConfigPatch live-applies + acks OK; "
                "DrivetrainConfigPatch stays ERR_UNIMPLEMENTED");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  TestSim::SimClock clock;
  TestSim::SimSleeper sleeper;

  Devices::NezhaMotor motorL(plant, baseMotorConfig(1));
  Devices::NezhaMotor motorR(plant, baseMotorConfig(2));
  Devices::Otos otos(plant, Devices::OtosConfig{});
  Devices::ColorSensorLeaf color(plant, Devices::ColorConfig{});
  Devices::LineSensorLeaf line(plant, Devices::LineConfig{});

  TestSupport::FakeTransport serialFake;
  TestSupport::FakeTransport radioFake;
  App::Comms comms(serialFake, radioFake, "DEVICE:NEZHA2:robot:test:0");
  App::Telemetry tlm(comms, serialFake, radioFake);
  App::Drive drive(motorL, motorR, /*trackWidth=*/120.0f);
  App::Odometry odom(motorL, motorR, /*trackWidth=*/120.0f);
  // App::StateEstimator -- default-constructed (0/0/200ms weights). 118
  // ticket 004: QUARANTINED -- App::MoveQueue no longer depends on this
  // member (its own former anticipation-lead completion path is deleted,
  // move_queue.h's own file header); kept solely for robotLoop's own
  // consumption below.
  App::StateEstimator stateEstimator;
  App::MoveQueue moveQueue(drive, odom, clock);
  App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  clock.setMicros(0);
  preamble.step();
  clock.setMicros(50000);
  scriptMotorBeginSuccess(bus);  // Left
  scriptMotorBeginSuccess(bus);  // Right
  scriptOtosBeginSuccess(bus);
  scriptColorBeginSuccess(bus);
  scriptLineBeginSuccess(bus);

  App::RobotLoop robotLoop(plant, motorL, motorR, otos, color, line, comms, tlm,
                            drive, odom, moveQueue, preamble, stateEstimator, clock, sleeper);
  robotLoop.boot();
  checkTrue(preamble.done(), "boot() completes against the FakeTransport-based fixture too");

  // Confirmed pre-patch baseline: baseMotorConfig() leaves velGains at
  // Devices::Gains{}'s all-zero default.
  checkFloatEq(motorL.gains().kp, 0.0f, "left motor starts at the constructed (zero) kp");
  checkFloatEq(motorR.gains().kp, 0.0f, "right motor starts at the constructed (zero) kp");

  const uint32_t kMotorCorrId = 87654;
  const uint32_t kDrivetrainCorrId = 87655;
  const uint32_t kErrUnimplemented = static_cast<uint32_t>(msg::ErrCode::ERR_UNIMPLEMENTED);

  // CONFIG{motor: side=LEFT, kp=0.02, ki=0.01} -- kff/i_max/kaw/travel_calib
  // deliberately absent, proving the merge-against-current-value path.
  Buf motorPatch;
  putVarintField(motorPatch, 1, 0);      // MotorConfigPatch.side = LEFT (0)
  putFloatField(motorPatch, 3, 0.02f);   // kp
  putFloatField(motorPatch, 4, 0.01f);   // ki
  Buf motorDelta;
  putMessageField(motorDelta, 2, motorPatch);  // ConfigDelta.motor, field 2
  Buf motorEnv;
  putVarintField(motorEnv, 1, kMotorCorrId);   // CommandEnvelope.corr_id
  putMessageField(motorEnv, 6, motorDelta);    // CommandEnvelope.config, field 6
  std::string motorLine = armorLine(motorEnv.data, motorEnv.len);
  checkTrue(!motorLine.empty(), "armor() of the CONFIG{motor} envelope succeeds");

  // CONFIG{drivetrain: trackwidth=130} -- a real (nonzero) field, not an
  // empty submessage, so this exercises the SAME shape a real host caller
  // would send.
  Buf drivetrainPatch;
  putFloatField(drivetrainPatch, 1, 130.0f);  // DrivetrainConfigPatch.trackwidth
  Buf drivetrainDelta;
  putMessageField(drivetrainDelta, 1, drivetrainPatch);  // ConfigDelta.drivetrain, field 1
  Buf drivetrainEnv;
  putVarintField(drivetrainEnv, 1, kDrivetrainCorrId);
  putMessageField(drivetrainEnv, 6, drivetrainDelta);
  std::string drivetrainLine = armorLine(drivetrainEnv.data, drivetrainEnv.len);
  checkTrue(!drivetrainLine.empty(), "armor() of the CONFIG{drivetrain} envelope succeeds");

  // The clock must ADVANCE past kPrimaryPeriod (20ms) each cycle so every
  // cycle()'s own emit() call actually sends a FRESH primary frame
  // reflecting that cycle's own ack-slot state (a frozen clock, as
  // scenarioBootThenAFewCyclesRunToCompletion above relies on, would let
  // Telemetry::emit() send its primary frame ONCE ever and never again --
  // fine for that scenario, which never inspects the ack slot's content,
  // but wrong here). Advancing past kPrimaryPeriod (20ms) also crosses
  // Otos::kReadPeriod (20ms), so every cycle needs a scripted OTOS burst
  // too. cycleIndex is a single running counter across the WHOLE scenario
  // (warm-up + dispatch + verification cycles) -- scriptMotorCycle()'s own
  // one-time first-duty-write quirk (R at global cycle 0, L at global cycle
  // 1) is keyed to it, matching scenarioBootThenAFewCyclesRunToCompletion's
  // own derivation; duty stays exactly 0 forever after that regardless of
  // the motor patch's own gain values, since target velocity is always 0
  // and encoder position never moves in this fixture (error is always 0).
  int cycleIndex = 0;
  uint64_t nowUs = 50000;
  auto runOneCycle = [&](const char* inject) {
    clock.setMicros(nowUs);
    scriptMotorCycle(bus, /*positionMm=*/0.0f, /*extraDutyWrites=*/(cycleIndex == 1) ? 1 : 0);  // Left
    scriptMotorCycle(bus, /*positionMm=*/0.0f, /*extraDutyWrites=*/(cycleIndex == 0) ? 1 : 0);  // Right
    scriptOtosReadZeroPose(bus);
    if (inject != nullptr) serialFake.enqueueInbound(inject);
    robotLoop.cycle();
    nowUs += 41000;  // > kPrimaryPeriod -- guarantees the NEXT cycle's primaryDue() is true
    ++cycleIndex;
  };
  // Runs quiet (no-injection) cycles, bounded, until a NEW primary frame is
  // observed -- absorbs 106-002's own occasional tie-break diversion to
  // secondary (see this scenario's own header comment).
  auto captureNextPrimaryLine = [&]() -> std::string {
    uint32_t before = tlm.primaryEmitCount();
    for (int attempt = 0; attempt < 5; ++attempt) {
      runOneCycle(nullptr);
      if (tlm.primaryEmitCount() > before) return serialFake.sent().back();
    }
    return std::string();
  };

  runOneCycle(nullptr);  // warm-up, cycleIndex 0 -- absorbs R's own one-time first duty write
  runOneCycle(nullptr);  // warm-up, cycleIndex 1 -- absorbs L's own one-time first duty write

  runOneCycle(motorLine.c_str());
  std::string afterMotorLine = captureNextPrimaryLine();

  runOneCycle(drivetrainLine.c_str());
  std::string afterDrivetrainLine = captureNextPrimaryLine();

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run: motor (config-dispatch cycles)");
  checkUintEq(bus.errCount(Devices::kOtosDeviceAddr), 0, "no script under-run: otos (config-dispatch cycles)");

  // --- "applies": both bound motors' PRESENT fields changed; ABSENT fields
  // (kff/iMax/kaw) stayed at their pre-patch value -- proves the merge, not
  // a blanket overwrite. ---
  checkFloatEq(motorL.gains().kp, 0.02f, "left motor kp reflects the applied patch");
  checkFloatEq(motorL.gains().ki, 0.01f, "left motor ki reflects the applied patch");
  checkFloatEq(motorL.gains().kff, 0.0f, "left motor kff (absent from the patch) stays at its prior value");
  checkFloatEq(motorR.gains().kp, 0.02f,
               "right motor kp ALSO reflects the applied patch -- kp/ki/kff/iMax/kaw apply to BOTH bound motors");
  checkFloatEq(motorR.gains().ki, 0.01f, "right motor ki also reflects the applied patch");

  // --- ack content. The motor patch's ack is a SUCCESS (err==0) ack --
  // proto3 implicit presence means encodeInto() OMITS a scalar field
  // holding its zero/default value entirely (findAck()'s own doc comment,
  // below, explains this in full), so ackFingerprint()/containsSubBytes()'s
  // raw-byte substring technique -- which synthesizes a literal "field
  // 6 (ack_err) == 0" byte pair that can never appear on the wire for a
  // genuine success ack -- is only valid for a NONZERO err (the drivetrain
  // check just below). The motor check instead decodes via the real
  // generated codec (TestSupport::decodeOutboundLine(), the same technique
  // findAck() uses) and compares the reconstructed scalar fields directly.
  // ---
  checkTrue(!afterMotorLine.empty(), "a primary frame was captured after the motor dispatch");
  TestSupport::DecodedLine motorDecoded = TestSupport::decodeOutboundLine(afterMotorLine);
  checkTrue(motorDecoded.kind == TestSupport::DecodedKind::kTelemetry,
            "the captured frame after the motor dispatch decodes as a Telemetry frame");
  checkUintEq(motorDecoded.telemetry.ack_corr, kMotorCorrId,
              "CONFIG{motor} acks against the motor patch's own corr_id");
  checkUintEq(motorDecoded.telemetry.ack_err, 0,
              "CONFIG{motor} acks OK (ack_err == 0, omitted from the wire by proto3 implicit presence)");

  checkTrue(!afterDrivetrainLine.empty(), "a primary frame was captured after the drivetrain dispatch");
  std::string drivetrainFrame = rawBytesFromArmoredLine(afterDrivetrainLine);
  checkTrue(!drivetrainFrame.empty(), "the captured frame de-armors to non-empty raw bytes");
  checkTrue(containsSubBytes(drivetrainFrame, ackFingerprint(kDrivetrainCorrId, kErrUnimplemented)),
            "CONFIG{drivetrain} still acks ERR_UNIMPLEMENTED");
  // Single ack slot: the drivetrain dispatch's own ack() call OVERWRITES
  // the shared corr/err pair outright (not just the freshness bit) -- so
  // the motor's own corr_id must no longer be the frame's ack_corr. Decoded
  // (not a byte-fingerprint absence check): a fingerprint search for the
  // motor's OWN success pair would trivially "pass" regardless of overwrite
  // state, since that exact byte pair never appears on the wire either way
  // (same proto3 implicit-presence reason as the motor check above) -- it
  // would not actually be testing the overwrite.
  TestSupport::DecodedLine drivetrainDecoded = TestSupport::decodeOutboundLine(afterDrivetrainLine);
  checkTrue(drivetrainDecoded.kind == TestSupport::DecodedKind::kTelemetry,
            "the captured frame after the drivetrain dispatch decodes as a Telemetry frame");
  checkTrue(drivetrainDecoded.telemetry.ack_corr != kMotorCorrId,
            "single ack slot: motor's own ack_corr no longer appears once drivetrain's dispatch overwrote "
            "the slot");
  checkUintEq(drivetrainDecoded.telemetry.ack_corr, kDrivetrainCorrId,
              "the slot now holds the drivetrain patch's own corr_id");
}

// ===========================================================================
// 114-004 (SUC-003): the persisted-tuning write policy. persistTuningIfChanged()
// must NOT write on every patch unconditionally (sprint.md Open Question 3) --
// it skips a save() call when the freshly-serialized snapshot is IDENTICAL to
// the last one actually written. Dispatches the SAME MotorConfigPatch bytes
// TWICE, through two separate cycle() calls, against a MockTuningStore, and
// asserts saveCount()==1 (mock/count-based assertion on the persistence seam,
// per this ticket's own Testing section -- not real flash).
// ===========================================================================

void scenarioConfigPersistWritePolicySkipsRedundantSave() {
  beginScenario("RobotLoop CONFIG: persisted-tuning write policy skips a redundant identical save()");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  TestSim::SimClock clock;
  TestSim::SimSleeper sleeper;

  Devices::NezhaMotor motorL(plant, baseMotorConfig(1));
  Devices::NezhaMotor motorR(plant, baseMotorConfig(2));
  Devices::Otos otos(plant, Devices::OtosConfig{});
  Devices::ColorSensorLeaf color(plant, Devices::ColorConfig{});
  Devices::LineSensorLeaf line(plant, Devices::LineConfig{});

  TestSupport::FakeTransport serialFake;
  TestSupport::FakeTransport radioFake;
  App::Comms comms(serialFake, radioFake, "DEVICE:NEZHA2:robot:test:0");
  App::Telemetry tlm(comms, serialFake, radioFake);
  App::Drive drive(motorL, motorR, /*trackWidth=*/120.0f);
  App::Odometry odom(motorL, motorR, /*trackWidth=*/120.0f);
  // App::StateEstimator -- default-constructed (0/0/200ms weights). 118
  // ticket 004: QUARANTINED -- App::MoveQueue no longer depends on this
  // member (its own former anticipation-lead completion path is deleted,
  // move_queue.h's own file header); kept solely for robotLoop's own
  // consumption below.
  App::StateEstimator stateEstimator;
  App::MoveQueue moveQueue(drive, odom, clock);
  App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  clock.setMicros(0);
  preamble.step();
  clock.setMicros(50000);
  scriptMotorBeginSuccess(bus);  // Left
  scriptMotorBeginSuccess(bus);  // Right
  scriptOtosBeginSuccess(bus);
  scriptColorBeginSuccess(bus);
  scriptLineBeginSuccess(bus);

  MockTuningStore mockStore;
  App::RobotLoop robotLoop(plant, motorL, motorR, otos, color, line, comms, tlm,
                            drive, odom, moveQueue, preamble, stateEstimator, clock, sleeper,
                            &mockStore);
  robotLoop.boot();
  checkTrue(preamble.done(), "boot() completes against the MockTuningStore-equipped fixture too");

  // The SAME patch bytes, encoded once and dispatched TWICE (below) -- only
  // the patch CONTENT (side=LEFT, kp=0.02, ki=0.01) matters for the write
  // policy's own blob comparison, so both dispatches reuse one envelope.
  Buf motorPatch;
  putVarintField(motorPatch, 1, 0);      // MotorConfigPatch.side = LEFT (0)
  putFloatField(motorPatch, 3, 0.02f);   // kp
  putFloatField(motorPatch, 4, 0.01f);   // ki
  Buf motorDelta;
  putMessageField(motorDelta, 2, motorPatch);  // ConfigDelta.motor, field 2
  Buf motorEnv;
  putVarintField(motorEnv, 1, /*corr_id=*/91234);
  putMessageField(motorEnv, 6, motorDelta);    // CommandEnvelope.config, field 6
  std::string motorLine = armorLine(motorEnv.data, motorEnv.len);
  checkTrue(!motorLine.empty(), "armor() of the CONFIG{motor} envelope succeeds");

  // Same cycle-index/extraDutyWrites schedule as
  // scenarioConfigMotorAppliesWhileDrivetrainStaysUnimplemented()
  // above (see that scenario's own scriptMotorCycle() derivation comment) --
  // this fixture never issues a TWIST/MOVE either, so drive_.tick()'s
  // one-time each-leaf "first write" lands on the identical cycles 0
  // (right)/1 (left), independent of which cycles inject a CONFIG line.
  uint64_t nowUs = 50000;
  for (int i = 0; i < 4; ++i) {
    clock.setMicros(nowUs);
    scriptMotorCycle(bus, /*positionMm=*/0.0f, /*extraDutyWrites=*/(i == 1) ? 1 : 0);  // Left
    scriptMotorCycle(bus, /*positionMm=*/0.0f, /*extraDutyWrites=*/(i == 0) ? 1 : 0);  // Right
    scriptOtosReadZeroPose(bus);

    if (i == 1 || i == 2) serialFake.enqueueInbound(motorLine.c_str());  // dispatch the SAME patch twice

    robotLoop.cycle();
    nowUs += 41000;
  }

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run: motor (persist-policy cycles)");
  checkUintEq(bus.errCount(Devices::kOtosDeviceAddr), 0, "no script under-run: otos (persist-policy cycles)");

  checkFloatEq(motorL.gains().kp, 0.02f, "left motor kp reflects the (twice-dispatched) patch");
  checkFloatEq(motorL.gains().ki, 0.01f, "left motor ki reflects the (twice-dispatched) patch");
  checkUintEq(static_cast<uint32_t>(mockStore.saveCount), 1,
              "persistTuningIfChanged() saves ONCE for the first patch, skips the byte-identical second one");
  checkUintEq(mockStore.lastVersion, Config::kConfigSchemaVersion,
              "the one save() call stamps the current schema version");
  checkUintEq(static_cast<uint32_t>(mockStore.wipeCount), 0,
              "a live CFG patch never calls wipe() -- that is boot-sequence-only (main.cpp)");
}

// ===========================================================================
// 116-006 (MOVE dispatch cutover, SUC-050/SUC-053/SUC-054/SUC-055) -- the
// new scenarios below exercise App::RobotLoop's MOVE dispatch/MoveQueue
// integration. Unlike the three scenarios above (which need a
// ScriptedI2CHook's EXACT bus-transaction budget), these run against a
// LIVE, UNSCRIPTED TestSim::SimPlant -- the same "always answers correctly,
// no pre-loaded script" property src/sim/sim_harness.h's own header
// documents (motorL_/motorR_'s baseMotorConfig() leaves velGains at
// Devices::Gains{}'s all-zero default, so commanded duty is always 0 and
// the plant's wheels never actually move -- exactly what a Kind::Distance
// stop condition that must NOT be satisfiable needs, and exactly what a
// Kind::Time stop condition does not care about). LiveFixture below
// mirrors sim_harness.h's own construction order/driveBootToDone() shape,
// hand-composed (not TestSim::SimHarness itself) so every module --
// including moveQueue_ and tlm_ -- stays directly reachable by the
// scenario, the same "own every collaborator, no wrapper" convention the
// three ScriptedI2CHook scenarios above already use.
// ===========================================================================

// driveLivePlantBootToDone -- mirrors sim_harness.h's own
// driveBootToDone() exactly: a live SimPlant resolves every device on its
// own first real transaction (no scripted budget to exhaust), so only
// color_/line_'s own retry-until-exhausted pacing governs how long this
// loop actually needs to run.
void driveLivePlantBootToDone(App::Preamble& preamble, TestSim::SimClock& clock) {
  clock.setMicros(0);
  preamble.step();
  clock.setMicros(50000);
  for (int i = 0; i < 200 && !preamble.done(); ++i) {
    preamble.step();
    clock.advanceMicros(50000);
  }
}

// LiveFixture -- the whole App:: graph, hand-composed against a live
// (unscripted) SimPlant, boot()ed by construction. `step(cycles)` mirrors
// sim_harness.h's own step(): tick the plant BEFORE the loop reads it,
// every cycle (that file's own header explains why this order is the one
// invariant that matters).
struct LiveFixture {
  TestSim::SimPlant plant;
  TestSim::SimClock clock;
  TestSim::SimSleeper sleeper;

  Devices::NezhaMotor motorL;
  Devices::NezhaMotor motorR;
  Devices::Otos otos;
  Devices::ColorSensorLeaf color;
  Devices::LineSensorLeaf line;

  TestSupport::FakeTransport serialFake;
  TestSupport::FakeTransport radioFake;

  App::Comms comms;
  App::Telemetry tlm;
  App::Drive drive;
  App::Odometry odom;
  // 117 ticket 003: default-constructed (encoder-only-v1 FusionWeights{}
  // default) -- directly reachable by every LiveFixture-based scenario
  // (unlike RobotLoop's own persistedTuning_/tuningStore_, which stay
  // private). 118 ticket 004: QUARANTINED -- App::MoveQueue no longer
  // depends on this member (its own former anticipation-lead completion
  // path is deleted, move_queue.h's own file header); kept solely for
  // robotLoop's own consumption below.
  App::StateEstimator stateEstimator;
  App::MoveQueue moveQueue;
  App::Preamble preamble;
  App::RobotLoop robotLoop;

  LiveFixture()
      : motorL(plant, baseMotorConfig(1)),
        motorR(plant, baseMotorConfig(2)),
        otos(plant, Devices::OtosConfig{}),
        color(plant, Devices::ColorConfig{}),
        line(plant, Devices::LineConfig{}),
        comms(serialFake, radioFake, "DEVICE:NEZHA2:robot:test:0"),
        tlm(comms, serialFake, radioFake),
        drive(motorL, motorR, /*trackWidth=*/120.0f),
        odom(motorL, motorR, /*trackWidth=*/120.0f),
        moveQueue(drive, odom, clock),
        preamble(motorL, motorR, otos, color, line, clock),
        robotLoop(plant, motorL, motorR, otos, color, line, comms, tlm, drive, odom, moveQueue,
                  preamble, stateEstimator, clock, sleeper) {
    driveLivePlantBootToDone(preamble, clock);
    robotLoop.boot();
  }

  void step(int cycles = 1) {
    for (int i = 0; i < cycles; ++i) {
      plant.tick(static_cast<float>(kCycleDtUs) / 1e6f);  // [s]
      clock.advanceMicros(kCycleDtUs);
      robotLoop.cycle();
    }
  }
};

// --- Hand-rolled CommandEnvelope{move: Move{...}} encoders (mirror this
// file's own Buf/putVarintField/putFloatField/putMessageField/armorLine
// helpers above -- no encode(CommandEnvelope) codec exists). Field numbers
// per envelope.proto (116-001): Move.velocity oneof (twist=1 ->
// MoveTwist{v_x=1, omega=3}), Move.stop oneof (time=3, distance=4),
// Move.timeout=6, Move.replace=7, Move.id=8; CommandEnvelope.move=21. ---

// includeVelocity/includeStop -- false OMITS the corresponding oneof field
// entirely (a genuinely malformed wire Move -- proto3 field ABSENCE, not a
// zero value, is what leaves velocity_kind/stop_kind at their NONE
// default), for the ERR_BADARG shape-validation scenario below.
std::string armorMoveTimeTwistCommand(bool includeVelocity, float v_x, float omega,
                                       bool includeStop, float stopTimeMs, float timeoutMs,
                                       bool replace, uint32_t id, uint32_t corrId) {
  Buf move;
  if (includeVelocity) {
    Buf moveTwist;
    putFloatField(moveTwist, 1, v_x);   // MoveTwist.v_x
    putFloatField(moveTwist, 3, omega); // MoveTwist.omega
    putMessageField(move, 1, moveTwist);  // Move.velocity.twist, field 1
  }
  if (includeStop) {
    putFloatField(move, 3, stopTimeMs);  // Move.stop.time, field 3
  }
  putFloatField(move, 6, timeoutMs);         // Move.timeout, field 6
  putVarintField(move, 7, replace ? 1 : 0);  // Move.replace, field 7
  putVarintField(move, 8, id);               // Move.id, field 8
  Buf env;
  putVarintField(env, 1, corrId);  // CommandEnvelope.corr_id
  putMessageField(env, 21, move);  // CommandEnvelope.cmd.move, field 21
  return armorLine(env.data, env.len);
}

// A DISTANCE-stop variant (SUC-054's own timeout scenario needs a stop
// condition the zero-gain motors below can never satisfy on their own).
std::string armorMoveDistanceTwistCommand(float v_x, float omega, float stopDistanceMm,
                                           float timeoutMs, bool replace, uint32_t id,
                                           uint32_t corrId) {
  Buf moveTwist;
  putFloatField(moveTwist, 1, v_x);
  putFloatField(moveTwist, 3, omega);
  Buf move;
  putMessageField(move, 1, moveTwist);       // Move.velocity.twist, field 1
  putFloatField(move, 4, stopDistanceMm);    // Move.stop.distance, field 4
  putFloatField(move, 6, timeoutMs);         // Move.timeout, field 6
  putVarintField(move, 7, replace ? 1 : 0);  // Move.replace, field 7
  putVarintField(move, 8, id);               // Move.id, field 8
  Buf env;
  putVarintField(env, 1, corrId);
  putMessageField(env, 21, move);
  return armorLine(env.data, env.len);
}

// CommandEnvelope{corr_id, config: ConfigDelta{motor: MotorConfigPatch{
// side=LEFT, kp}}} -- mirrors scenarioConfigMotorAppliesWhileDrivetrainStays
// Unimplemented()'s own inline construction above, factored into a
// standalone helper for SUC-055's reuse below.
std::string armorMotorConfigPatchCommand(float kp, uint32_t corrId) {
  Buf motorPatch;
  putVarintField(motorPatch, 1, 0);  // MotorConfigPatch.side = LEFT (0)
  putFloatField(motorPatch, 3, kp);  // kp
  Buf motorDelta;
  putMessageField(motorDelta, 2, motorPatch);  // ConfigDelta.motor, field 2
  Buf motorEnv;
  putVarintField(motorEnv, 1, corrId);
  putMessageField(motorEnv, 6, motorDelta);  // CommandEnvelope.config, field 6
  return armorLine(motorEnv.data, motorEnv.len);
}

// findAck -- mirrors config_gate_harness.cpp's own findAck() exactly:
// decodes every captured outbound line via TestSupport::decodeOutboundLine()
// (the REAL generated codec, not a hand-synthesized byte fingerprint) and
// looks for a Telemetry frame whose ack_corr matches `corrId` with the
// ack-fresh bit (flags bit 5) set. Deliberately NOT a raw-byte fingerprint
// search (this file's own ackFingerprint()/containsSubBytes() helpers,
// above, used only by the pre-existing CONFIG scenarios' OWN nonzero-err
// checks): a genuine err==0 (success) ack has its ack_err field OMITTED
// from the wire entirely (proto3 implicit presence -- wire.cpp's own
// encodeInto() skips a plain scalar field holding its zero/default value),
// so a synthesized ack_err==0 byte pair can never match real wire bytes --
// only the real decoder correctly reconstructs "absent means 0".
bool findAck(const std::deque<std::string>& lines, uint32_t corrId, uint32_t* errCode) {
  constexpr uint32_t kAckFreshBit = 1u << 5;  // App::kFlagAckFresh
  for (const auto& line : lines) {
    TestSupport::DecodedLine decoded = TestSupport::decodeOutboundLine(line);
    if (decoded.kind != TestSupport::DecodedKind::kTelemetry) continue;
    if ((decoded.telemetry.flags & kAckFreshBit) == 0) continue;
    if (decoded.telemetry.ack_corr == corrId) {
      *errCode = decoded.telemetry.ack_err;
      return true;
    }
  }
  return false;
}

// Steps fx one cycle at a time, up to maxCycles, checking after EACH step
// whether corrId's ack has appeared (fresh) -- absorbs BOTH "an ack pushed
// during cycle N's dispatch is not visible until cycle N+1's own emit()"
// (this file's own scenarioConfigMotorAppliesWhileDrivetrainStaysUnimplemented()
// header comment above) AND the primary/secondary tie-break's own
// occasional one-cycle emit slip (telemetry.h's emit() doc comment: "at
// most one primary frame delayed by one loop cycle roughly once per
// kSecondaryPeriod") without hand-deriving exact cycle parity for either.
// Returns true iff the ack was seen AND its err matched expectedErrCode.
bool stepUntilAckSeen(LiveFixture& fx, uint32_t corrId, uint32_t expectedErrCode, int maxCycles) {
  for (int i = 0; i < maxCycles; ++i) {
    fx.step(1);
    uint32_t errCode = 0;
    if (findAck(fx.serialFake.sent(), corrId, &errCode)) return errCode == expectedErrCode;
  }
  return false;
}

// ===========================================================================
// MOVE dispatch: config-gate refusal, ERR_BADARG shape validation,
// successful enqueue+ack (SUC-050).
// ===========================================================================

void scenarioMoveConfigGateRefusesWhenUnconfigured() {
  beginScenario("MOVE against an unconfigured RobotLoop: ERR_NOT_CONFIGURED, MoveQueue untouched "
                "(116-006)");

  LiveFixture fx;
  checkTrue(!fx.robotLoop.isConfigured(), "setup: RobotLoop starts unconfigured "
                                           "(markConfigured() never called)");

  const uint32_t kCorrId = 901;
  std::string moveLine = armorMoveTimeTwistCommand(/*includeVelocity=*/true, /*v_x=*/500.0f,
                                                     /*omega=*/0.0f, /*includeStop=*/true,
                                                     /*stopTimeMs=*/200.0f, /*timeoutMs=*/5000.0f,
                                                     /*replace=*/true, /*id=*/1, kCorrId);
  checkTrue(!moveLine.empty(), "armor() of the MOVE envelope succeeds");
  fx.serialFake.enqueueInbound(moveLine.c_str());

  checkTrue(stepUntilAckSeen(fx, kCorrId, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED), 10),
            "the MOVE's ack is ERR_NOT_CONFIGURED");
  checkTrue(!fx.moveQueue.active(), "MoveQueue stays empty -- handleMove() never reached enqueue()");
}

void scenarioMoveBadArgShapeValidation() {
  beginScenario("MOVE shape validation: missing velocity / missing stop / non-positive timeout "
                "all ack ERR_BADARG (116-006)");

  LiveFixture fx;
  fx.robotLoop.markConfigured();
  checkTrue(fx.robotLoop.isConfigured(), "setup: RobotLoop is configured for this scenario");

  const uint32_t kMissingVelocityCorrId = 911;
  const uint32_t kMissingStopCorrId = 912;
  const uint32_t kNonPositiveTimeoutCorrId = 913;
  const uint32_t kErrBadArg = static_cast<uint32_t>(msg::ErrCode::ERR_BADARG);

  fx.serialFake.enqueueInbound(
      armorMoveTimeTwistCommand(/*includeVelocity=*/false, 0.0f, 0.0f, /*includeStop=*/true, 200.0f,
                                 5000.0f, true, 1, kMissingVelocityCorrId)
          .c_str());
  checkTrue(stepUntilAckSeen(fx, kMissingVelocityCorrId, kErrBadArg, 10),
            "missing velocity variant acks ERR_BADARG");

  fx.serialFake.enqueueInbound(
      armorMoveTimeTwistCommand(/*includeVelocity=*/true, 500.0f, 0.0f, /*includeStop=*/false, 0.0f,
                                 5000.0f, true, 2, kMissingStopCorrId)
          .c_str());
  checkTrue(stepUntilAckSeen(fx, kMissingStopCorrId, kErrBadArg, 10),
            "missing stop variant acks ERR_BADARG");

  fx.serialFake.enqueueInbound(
      armorMoveTimeTwistCommand(/*includeVelocity=*/true, 500.0f, 0.0f, /*includeStop=*/true, 200.0f,
                                 /*timeoutMs=*/0.0f, true, 3, kNonPositiveTimeoutCorrId)
          .c_str());
  checkTrue(stepUntilAckSeen(fx, kNonPositiveTimeoutCorrId, kErrBadArg, 10),
            "non-positive timeout acks ERR_BADARG");

  checkTrue(!fx.moveQueue.active(), "MoveQueue stays empty across all three malformed Moves");
}

void scenarioMoveSuccessfulEnqueueAcksAndActivates() {
  beginScenario("well-formed MOVE: acks ERR_NONE, MoveQueue activates with matching moveId "
                "(116-006)");

  LiveFixture fx;
  fx.robotLoop.markConfigured();

  const uint32_t kCorrId = 921;
  const uint32_t kMoveId = 55;
  fx.serialFake.enqueueInbound(
      armorMoveTimeTwistCommand(true, 500.0f, 0.0f, true, 300.0f, 5000.0f, true, kMoveId, kCorrId)
          .c_str());
  fx.step(1);

  checkTrue(fx.moveQueue.active(), "MoveQueue activates immediately (queue was empty)");
  checkUintEq(fx.moveQueue.activeMoveId(), kMoveId, "activeMoveId() matches the injected Move.id");

  checkTrue(stepUntilAckSeen(fx, kCorrId, 0, 10), "the MOVE's enqueue ack is ERR_NONE (0)");
}

// ===========================================================================
// SUC-053: a Move that ends drains to stopped motors with zero further
// host traffic -- MoveQueue::tick() replaces the deleted deadman_.expired()
// branch at the exact same per-cycle, unconditional schedule position.
// ===========================================================================

void scenarioMoveEndDrainsWithNoFurtherHostTraffic() {
  beginScenario("SUC-053: a Move that ends via its own stop condition drains to stopped motors "
                "with zero further host traffic (no deadman to re-arm) (116-006)");

  LiveFixture fx;
  fx.robotLoop.markConfigured();

  const uint32_t kCorrId = 931;
  const uint32_t kMoveId = 61;
  // TIME stop at 150ms, generous 5s timeout -- ends via its OWN stop
  // condition, not the timeout backstop.
  fx.serialFake.enqueueInbound(
      armorMoveTimeTwistCommand(true, /*v_x=*/500.0f, /*omega=*/0.0f, true, /*stopTimeMs=*/150.0f,
                                 /*timeoutMs=*/5000.0f, true, kMoveId, kCorrId)
          .c_str());
  fx.step(1);
  checkTrue(fx.moveQueue.active(), "the Move activates immediately");

  bool ended = false;
  constexpr int kMaxCycles = 40;  // 40 * 50ms == 2s, comfortably past the 150ms stop threshold
  for (int i = 0; i < kMaxCycles && !ended; ++i) {
    fx.step(1);
    if (!fx.moveQueue.active()) ended = true;
  }
  checkTrue(ended, "the Move ends within a bounded number of cycles");

  // MoveQueue::tick() (this cycle's OWN 3rd runAndWait block) calls
  // Drive::stop(), which only STAGES a zero target on Drive itself --
  // Drive::tick() is the only method that ever calls
  // Devices::Motor::setVelocity() (drive.h's own doc comment), and it runs
  // at the TOP of cycle() (112-005's own reorder), BEFORE this same
  // cycle's dispatch block. So the actual zero duty write reaches the
  // motor leaves one cycle LATER -- exactly the same one-cycle lag the
  // deleted deadman-driven stop had, at this same schedule position; not
  // a regression here. One more cycle before checking velocityTarget().
  fx.step(1);
  checkFloatEq(fx.motorL.velocityTarget(), 0.0f, "left target velocity is zero once the Move ends");
  checkFloatEq(fx.motorR.velocityTarget(), 0.0f, "right target velocity is zero once the Move ends");

  // SUC-053's own rigor bar: zero FURTHER host traffic after the Move ends
  // -- no STOP, no new MOVE -- yet the motors stay at zero, cycle after
  // cycle, because moveQueue_.tick() runs unconditionally every cycle.
  for (int i = 0; i < 20; ++i) {
    fx.step(1);
    checkTrue(!fx.moveQueue.active(), "MoveQueue stays inactive with no further host traffic");
    checkFloatEq(fx.motorL.velocityTarget(), 0.0f,
                 "left target velocity stays zero, no host traffic needed");
    checkFloatEq(fx.motorR.velocityTarget(), 0.0f,
                 "right target velocity stays zero, no host traffic needed");
  }
}

// ===========================================================================
// SUC-054: a Move whose stop condition can never be reached ends via the
// required timeout backstop; kFlagFaultMoveTimeout is set live on that
// exact ending cycle.
// ===========================================================================

void scenarioMoveTimeoutSetsFaultFlagOnEndingCycle() {
  beginScenario("SUC-054: a Move whose stop condition can never be reached ends via timeout, "
                "kFlagFaultMoveTimeout set live on that exact ending cycle (116-006)");

  LiveFixture fx;
  fx.robotLoop.markConfigured();

  // baseMotorConfig() leaves velGains at Devices::Gains{}'s all-zero
  // default -- commanded duty is 0 regardless of target velocity, so the
  // plant's wheels never actually move and odom_.pathLength() stays 0
  // forever: a DISTANCE stop condition can never be satisfied on its own,
  // only the timeout backstop can end this Move.
  const uint32_t kCorrId = 941;
  const uint32_t kMoveId = 71;
  fx.serialFake.enqueueInbound(armorMoveDistanceTwistCommand(/*v_x=*/500.0f, /*omega=*/0.0f,
                                                              /*stopDistanceMm=*/50.0f,
                                                              /*timeoutMs=*/120.0f, /*replace=*/true,
                                                              kMoveId, kCorrId)
                                    .c_str());
  fx.step(1);
  checkTrue(fx.moveQueue.active(), "the Move activates immediately");

  bool ended = false;
  bool flagSetOnEndingCycle = false;
  constexpr int kMaxCycles = 40;  // 40 * 50ms == 2s, comfortably past the 120ms timeout
  for (int i = 0; i < kMaxCycles && !ended; ++i) {
    fx.step(1);
    if (!fx.moveQueue.active()) {
      ended = true;
      flagSetOnEndingCycle = (fx.tlm.flags() & App::kFlagFaultMoveTimeout) != 0;
    }
  }
  checkTrue(ended, "the Move ends within a bounded number of cycles (via timeout -- DISTANCE can "
                    "never be satisfied with zero-gain motors)");
  checkTrue(flagSetOnEndingCycle, "kFlagFaultMoveTimeout is set on the exact ending cycle");

  // Level-set (telemetry.h's own setFlag() contract): the flag clears
  // again the very next cycle, and the completion ack pushed during the
  // ending cycle's own dispatch is not visible on the wire until THIS
  // next cycle's own emit() call (cycle N+1's 2nd runAndWait block runs
  // BEFORE its own 3rd block, matching the CONFIG-dispatch scenario's own
  // "ack rides the next emitted frame" convention documented above).
  fx.step(1);
  checkTrue((fx.tlm.flags() & App::kFlagFaultMoveTimeout) == 0,
            "kFlagFaultMoveTimeout clears again the very next cycle");
  {
    uint32_t errCode = 1;  // any nonzero sentinel -- overwritten by findAck() on a match
    checkTrue(findAck(fx.serialFake.sent(), kMoveId, &errCode) && errCode == 0,
              "the Move's completion ack (ack_corr==Move.id, ack_err==0) reached the wire -- a "
              "timeout is signalled via the flags bit, not ack_err");
  }
}

// ===========================================================================
// SUC-055: a CONFIG patch injected mid-MOVE does not change the active
// Move's completion outcome. A/B comparison against a config-free
// baseline -- more robust than hand-deriving the exact expected cycle
// count, and directly proves "shifts nothing".
// ===========================================================================

void scenarioConfigMidMoveDoesNotChangeMoveCompletionOutcome() {
  beginScenario("SUC-055: a CONFIG patch injected mid-MOVE does not change the active Move's "
                "completion outcome (116-006)");

  constexpr float kStopTimeMs = 250.0f;
  constexpr float kTimeoutMs = 5000.0f;
  constexpr int kMaxCycles = 40;
  const uint32_t kMoveId = 81;

  LiveFixture baseline;
  baseline.robotLoop.markConfigured();
  baseline.serialFake.enqueueInbound(
      armorMoveTimeTwistCommand(true, 0.0f, 0.0f, true, kStopTimeMs, kTimeoutMs, true, kMoveId, 951)
          .c_str());
  baseline.step(1);
  checkTrue(baseline.moveQueue.active(), "baseline: the Move activates immediately");
  int baselineCyclesToEnd = 1;
  for (int i = 0; i < kMaxCycles && baseline.moveQueue.active(); ++i) {
    baseline.step(1);
    ++baselineCyclesToEnd;
  }
  checkTrue(!baseline.moveQueue.active(), "baseline: the Move ends within a bounded number of cycles");

  LiveFixture interfered;
  interfered.robotLoop.markConfigured();
  interfered.serialFake.enqueueInbound(
      armorMoveTimeTwistCommand(true, 0.0f, 0.0f, true, kStopTimeMs, kTimeoutMs, true, kMoveId, 953)
          .c_str());
  interfered.step(1);
  checkTrue(interfered.moveQueue.active(), "interfered: the Move activates immediately");
  checkFloatEq(interfered.motorL.gains().kp, 0.0f,
               "interfered: left motor starts at the constructed (zero) kp");

  // Mid-flight (well before the stop threshold): a CONFIG{motor} patch --
  // MoveQueue's own tick() has nothing to do with CONFIG dispatch
  // (handleConfig() is a fully separate branch of processMessage()'s own
  // switch), so this must not perturb the Move's own timing at all.
  interfered.serialFake.enqueueInbound(armorMotorConfigPatchCommand(/*kp=*/0.02f, /*corrId=*/954).c_str());
  interfered.step(1);
  checkFloatEq(interfered.motorL.gains().kp, 0.02f,
               "interfered: the CONFIG patch's own kp landed live, unaffected by the concurrently-"
               "active Move");

  int interferedCyclesToEnd = 2;
  for (int i = 0; i < kMaxCycles && interfered.moveQueue.active(); ++i) {
    interfered.step(1);
    ++interferedCyclesToEnd;
  }
  checkTrue(!interfered.moveQueue.active(),
            "interfered: the Move ends within a bounded number of cycles");

  checkUintEq(static_cast<uint32_t>(interferedCyclesToEnd), static_cast<uint32_t>(baselineCyclesToEnd),
              "SUC-055: the CONFIG patch injected mid-flight shifts nothing -- the Move ends at "
              "the SAME cycle count as the config-free baseline");
}

// ===========================================================================
// SUC-063 (118 ticket 002): the MOVE stop decision must read odometry
// integrated THIS cycle, not the previous cycle's. Uses a ScriptedI2CHook
// bus (not LiveFixture's live SimPlant) so wheel encoder positions -- and
// therefore odom_.pathLength()'s cycle-by-cycle growth -- are EXACTLY
// known, letting this scenario place a DISTANCE stop threshold exactly on
// the boundary a specific cycle's own odom_.integrate() call crosses.
// Velocity gains stay at baseMotorConfig()'s all-zero default (the same
// "duty stays exactly 0 regardless of target" posture every other
// ScriptedI2CHook scenario in this file relies on -- see
// scriptMotorCycle()'s own header comment), so the Move's own commanded
// v_x has zero effect on the scripted encoder schedule below; only the
// DISTANCE stop condition's own pathLength() comparison is under test.
//
// Before 118 ticket 002, MoveQueue::tick() ran from the R-settle block,
// BEFORE odom_.integrate() (trailing pace block) in the SAME cycle -- so a
// stop condition crossed by cycle N's own integrate() call would not be
// OBSERVED by tick() until cycle N+1's (then R-settle-positioned) call,
// one cycle late. After the relocation, tick() runs in the SAME pace
// block, immediately after integrate(), so the crossing is observed on
// cycle N itself. This scenario scripts a straight-line (equal L/R,
// headingDelta always 0) encoder ramp that lands the 30mm DISTANCE
// threshold exactly on cycle 3's own integrate() call, and asserts
// completion is visible by the END of cycle 3 -- not cycle 4, which is
// what the pre-relocation ordering would have needed.
// ===========================================================================

void scenarioMoveDistanceStopReadsThisCyclesOdometryNotLastCycles() {
  beginScenario("SUC-063: MOVE DISTANCE stop decision reads odometry integrated THIS cycle, "
                "not the previous cycle's (118 ticket 002)");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  TestSim::SimClock clock;
  TestSim::SimSleeper sleeper;

  Devices::NezhaMotor motorL(plant, baseMotorConfig(1));
  Devices::NezhaMotor motorR(plant, baseMotorConfig(2));
  Devices::Otos otos(plant, Devices::OtosConfig{});
  Devices::ColorSensorLeaf color(plant, Devices::ColorConfig{});
  Devices::LineSensorLeaf line(plant, Devices::LineConfig{});

  TestSupport::FakeTransport serialFake;
  TestSupport::FakeTransport radioFake;
  App::Comms comms(serialFake, radioFake, "DEVICE:NEZHA2:robot:test:0");
  App::Telemetry tlm(comms, serialFake, radioFake);
  App::Drive drive(motorL, motorR, /*trackWidth=*/120.0f);
  App::Odometry odom(motorL, motorR, /*trackWidth=*/120.0f);
  // Default-constructed StateEstimator (0/0/200ms weights) -- quarantined,
  // App::MoveQueue no longer depends on it (see the other constructions in
  // this file for the full note); kept solely for robotLoop's own
  // consumption below.
  App::StateEstimator stateEstimator;
  App::MoveQueue moveQueue(drive, odom, clock);
  App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  clock.setMicros(0);
  preamble.step();
  clock.setMicros(50000);
  scriptMotorBeginSuccess(bus);  // Left
  scriptMotorBeginSuccess(bus);  // Right
  scriptOtosBeginSuccess(bus);
  scriptColorBeginSuccess(bus);
  scriptLineBeginSuccess(bus);

  App::RobotLoop robotLoop(plant, motorL, motorR, otos, color, line, comms, tlm,
                            drive, odom, moveQueue, preamble, stateEstimator, clock, sleeper);
  robotLoop.boot();
  checkTrue(preamble.done(), "boot() completes against the ScriptedI2CHook-based fixture too");
  robotLoop.markConfigured();

  const uint32_t kCorrId = 971;
  const uint32_t kMoveId = 91;
  // 30mm DISTANCE stop, generous 5s timeout (never the actual trigger --
  // this scenario ends via its own stop condition, not the backstop).
  // Injected before cycle 0 runs -- decoded via comms_.pump() (L-settle)
  // and activated via processMessage()/handleMove() (R-settle) that SAME
  // cycle, capturing activationPathLength=0 (nothing integrated yet).
  serialFake.enqueueInbound(
      armorMoveDistanceTwistCommand(/*v_x=*/500.0f, /*omega=*/0.0f, /*stopDistanceMm=*/30.0f,
                                     /*timeoutMs=*/5000.0f, /*replace=*/true, kMoveId, kCorrId)
          .c_str());

  // Straight-line schedule: both wheels advance identically (headingDelta
  // stays 0 -- pure DISTANCE growth, no ANGLE interaction), 10mm/cycle:
  // POS(i) = i * 10mm for cycles 0..3, then flat while the ack propagates.
  // pathLength() growth per cycle (odom_.integrate()'s own delta =
  // POS(i) - POS(i-1)):
  //   cycle 0: 0 -> 0    (pathLength 0)
  //   cycle 1: 0 -> 10   (pathLength 10)
  //   cycle 2: 10 -> 20  (pathLength 20)
  //   cycle 3: 20 -> 30  (pathLength 30 -- the 30mm threshold is crossed
  //                       EXACTLY by cycle 3's own odom_.integrate() call)
  // Duty stays 0 every cycle regardless of the Move's own v_x (velGains
  // all-zero, same posture as every other ScriptedI2CHook scenario in this
  // file) -- extraDutyWrites follows the SAME global-cycle-indexed
  // first-write schedule scriptMotorCycle()'s own header comment derives
  // (R at cycle 0, L at cycle 1), independent of the Move injected above.
  uint64_t nowUs = 50000;
  for (int i = 0; i <= 3; ++i) {
    clock.setMicros(nowUs);
    float positionMm = static_cast<float>(i) * 10.0f;
    scriptMotorCycle(bus, positionMm, /*extraDutyWrites=*/(i == 1) ? 1 : 0);  // Left
    scriptMotorCycle(bus, positionMm, /*extraDutyWrites=*/(i == 0) ? 1 : 0);  // Right
    scriptOtosReadZeroPose(bus);
    robotLoop.cycle();
    nowUs += 41000;

    if (i == 2) {
      checkTrue(moveQueue.active(), "sanity: still active right after cycle 2 -- pathLength is "
                                     "20mm, below the 30mm threshold");
    }
  }

  checkTrue(!moveQueue.active(),
            "SUC-063: the Move has ended by the END of cycle 3 -- the SAME cycle "
            "odom_.integrate() first raises pathLength() to the 30mm threshold, not cycle 4 "
            "(one cycle later, the pre-118-ticket-002 R-settle-positioned tick() would have "
            "needed)");

  // Completion ack visibility: staged during cycle 3's OWN pace block,
  // which runs AFTER that cycle's own tlm_.emit() call (kClear block,
  // earlier in the same cycle) -- so it is not visible on the wire until
  // a LATER cycle's own emit(). A few bounded flat (no further motion)
  // cycles absorb both that one-cycle lag and the primary/secondary
  // tie-break's own occasional one-cycle slip (telemetry.h's emit() doc
  // comment) -- mirrors stepUntilAckSeen()'s own bounded-retry shape,
  // hand-rolled here since that helper is LiveFixture-specific.
  bool ackSeen = false;
  uint32_t errCode = 1;  // any nonzero sentinel -- overwritten by findAck() on a match
  for (int i = 0; i < 5 && !ackSeen; ++i) {
    clock.setMicros(nowUs);
    scriptMotorCycle(bus, /*positionMm=*/30.0f, /*extraDutyWrites=*/0);  // Left
    scriptMotorCycle(bus, /*positionMm=*/30.0f, /*extraDutyWrites=*/0);  // Right
    scriptOtosReadZeroPose(bus);
    robotLoop.cycle();
    nowUs += 41000;
    ackSeen = findAck(serialFake.sent(), kMoveId, &errCode);
  }
  checkTrue(ackSeen && errCode == 0,
            "the Move's completion ack (ack_corr==Move.id, ack_err==0) reaches the wire on a "
            "cycle AFTER the one it completed on -- 'ack rides the next frame', unchanged by "
            "this ticket");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run: motor (ordering cycles)");
  checkUintEq(bus.errCount(Devices::kOtosDeviceAddr), 0, "no script under-run: otos (ordering cycles)");
}

// ===========================================================================
// 117 ticket 003: RobotLoop::handleConfig()'s new ESTIMATOR branch --
// CONFIG{estimator} merges PRESENT fields onto stateEstimator_.weights()
// (partial-patch semantics, matching MotorConfigPatch/OtosConfigPatch),
// acks OK, and -- UNLIKE the MOTOR/OTOS branches -- NEVER touches
// persistedTuning_/calls tuningStore_->save() (Design Rationale Decision 4,
// this sprint's overlay design/design.md: a reboot always reverts to the
// baked Config::defaultEstimatorConfig() default, never a live-tuned
// value). Hand-composed against a LIVE (unscripted) SimPlant -- mirrors
// LiveFixture's own construction shape, plus a MockTuningStore (114-004's
// own no-flash Config::TuningStore double) so the "never persisted" half
// of this scenario's claim is a real save()-call-count assertion, not just
// an absence of an accessor (persistedTuning_ itself stays private).
// ===========================================================================

void scenarioConfigEstimatorAppliesPresentFieldMergeAndNeverPersists() {
  beginScenario("RobotLoop CONFIG: EstimatorConfigPatch present-field merge, acks OK, "
                "NEVER persisted (117 ticket 003)");

  TestSim::SimPlant plant;
  TestSim::SimClock clock;
  TestSim::SimSleeper sleeper;

  Devices::NezhaMotor motorL(plant, baseMotorConfig(1));
  Devices::NezhaMotor motorR(plant, baseMotorConfig(2));
  Devices::Otos otos(plant, Devices::OtosConfig{});
  Devices::ColorSensorLeaf color(plant, Devices::ColorConfig{});
  Devices::LineSensorLeaf line(plant, Devices::LineConfig{});

  TestSupport::FakeTransport serialFake;
  TestSupport::FakeTransport radioFake;
  App::Comms comms(serialFake, radioFake, "DEVICE:NEZHA2:robot:test:0");
  App::Telemetry tlm(comms, serialFake, radioFake);
  App::Drive drive(motorL, motorR, /*trackWidth=*/120.0f);
  App::Odometry odom(motorL, motorR, /*trackWidth=*/120.0f);
  // Turn-prediction campaign: stateEstimator constructed before moveQueue
  // -- see the earlier scenarios' own comment for why.
  App::StateEstimator stateEstimator;  // default weights (0.0/0.0/200ms)
  App::MoveQueue moveQueue(drive, odom, clock);
  App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  MockTuningStore mockStore;
  App::RobotLoop robotLoop(plant, motorL, motorR, otos, color, line, comms, tlm,
                            drive, odom, moveQueue, preamble, stateEstimator, clock,
                            sleeper, &mockStore);
  driveLivePlantBootToDone(preamble, clock);
  robotLoop.boot();

  checkFloatEq(stateEstimator.weights().headingOtos, 0.0f,
               "estimator starts at the constructed (default) headingOtos");
  checkFloatEq(stateEstimator.weights().omegaOtos, 0.0f,
               "estimator starts at the constructed (default) omegaOtos");
  checkUintEq(stateEstimator.weights().staleness, 200u,
              "estimator starts at the constructed (default) staleness");

  // CONFIG{estimator: weight_heading_otos=0.4, staleness_ms=80} --
  // weight_omega_otos deliberately absent, proving the merge-against-
  // current-value path (mirrors scenarioConfigMotorAppliesWhileDrivetrain
  // StaysUnimplemented()'s own kff-absent proof above).
  const uint32_t kCorrId = 96001;
  Buf estimatorPatch;
  putFloatField(estimatorPatch, 1, 0.4f);   // weight_heading_otos, field 1
  putFloatField(estimatorPatch, 3, 80.0f);  // staleness_ms, field 3
  Buf estimatorDelta;
  putMessageField(estimatorDelta, 6, estimatorPatch);  // ConfigDelta.estimator, field 6
  Buf estimatorEnv;
  putVarintField(estimatorEnv, 1, kCorrId);           // CommandEnvelope.corr_id
  putMessageField(estimatorEnv, 6, estimatorDelta);   // CommandEnvelope.config, field 6
  std::string estimatorLine = armorLine(estimatorEnv.data, estimatorEnv.len);
  checkTrue(!estimatorLine.empty(), "armor() of the CONFIG{estimator} envelope succeeds");

  serialFake.enqueueInbound(estimatorLine.c_str());

  // Steps a bounded number of live cycles until the ack (fresh, matching
  // corr_id) appears -- mirrors stepUntilAckSeen()'s own shape (that
  // helper takes a LiveFixture&, not this hand-composed fixture's own
  // objects, so this is a small local equivalent rather than a reuse).
  bool acked = false;
  uint32_t ackErr = 1;  // any nonzero sentinel -- overwritten by findAck() on a match
  for (int i = 0; i < 10 && !acked; ++i) {
    plant.tick(static_cast<float>(kCycleDtUs) / 1e6f);  // [s] matches LiveFixture::step()'s own kCycleDtUs
    clock.advanceMicros(kCycleDtUs);
    robotLoop.cycle();
    acked = findAck(serialFake.sent(), kCorrId, &ackErr);
  }
  checkTrue(acked, "CONFIG{estimator}'s ack (fresh, matching corr_id) appears within a bounded "
                    "number of cycles");
  checkUintEq(ackErr, 0, "CONFIG{estimator} acks OK (0)");

  checkFloatEq(stateEstimator.weights().headingOtos, 0.4f,
               "headingOtos reflects the applied patch");
  checkFloatEq(stateEstimator.weights().omegaOtos, 0.0f,
               "omegaOtos (absent from the patch) stays at its prior (default) value -- proves "
               "the merge, not a blanket overwrite");
  checkUintEq(stateEstimator.weights().staleness, 80u, "staleness reflects the applied patch");

  checkUintEq(static_cast<uint32_t>(mockStore.saveCount), 0,
              "EstimatorConfigPatch NEVER calls tuningStore_->save() -- deliberately not "
              "persisted (Design Rationale Decision 4, overlay design/design.md): a reboot "
              "always reverts to the baked Config::defaultEstimatorConfig() default");
}

// ===========================================================================
// 117 ticket 004: RobotLoop::cycle()'s trailing kPace block now calls
// stateEstimator_.update(frame_, nowUs) once per cycle, immediately after
// frame_.pose is staged. This scenario runs against a LIVE (unscripted)
// SimPlant with REAL, nonzero velocity gains
// (TestSupport::benchTestMotorConfig() -- the SAME realistic gains
// sim_api_harness.cpp's own scenarioTwistDrivesRealPlantRamp() uses; unlike
// this file's own baseMotorConfig() default, whose deliberately-zero gains
// never actually move the plant) and proves:
//   (1) after several cycles of commanded motion, StateEstimator's wheel
//       AND body peer estimates reach valid=true and track the commanded
//       twist in the expected direction/magnitude (AC #2's own call site,
//       exercised end to end);
//   (2) no regression in encoder-tracking-vs-commanded-speed accuracy
//       attributable to the estimator's addition to the schedule -- the
//       REAL motor's own encoder-derived velocity (Devices::Motor::
//       velocity(), independent of the estimator's own copy) still ramps
//       toward the commanded speed. Complements
//       scenarioVirtualCycleTimingDiagnostic() (sim_api_harness.cpp), which
//       proves the SAME "no regression" bar at the schedule-BUDGET level
//       (sleepCount/lastSleepMillis unchanged -- update() is pure
//       computation over already-staged data, no new I2C transaction, no
//       new sleep, see robot_loop.cpp's own kPace-block comment) -- this
//       scenario proves it at the PLANT-RESPONSE level instead;
//   (3) AC #6: ticket 003's handleConfig() ESTIMATOR branch and this
//       ticket's cycle()-call-site wiring reach the SAME stateEstimator
//       instance, not two -- a live EstimatorConfigPatch dispatched here
//       visibly mutates the SAME object update() has been refreshing every
//       cycle above.
// ===========================================================================

void scenarioStateEstimatorTracksCommandedMotionNoTrackingRegression() {
  beginScenario("117 ticket 004: StateEstimator tracks commanded motion after warm-up; real "
                "motor velocity still ramps toward commanded speed (no regression from the "
                "estimator's own addition to the schedule); ESTIMATOR branch reaches the same "
                "instance");

  TestSim::SimPlant plant;
  TestSim::SimClock clock;
  TestSim::SimSleeper sleeper;

  // Real, nonzero gains -- see this scenario's own header comment for why
  // baseMotorConfig() (this file's own zero-gain default, used by every
  // scenario above) will not do here.
  Devices::NezhaMotor motorL(plant, TestSupport::benchTestMotorConfig(1));
  Devices::NezhaMotor motorR(plant, TestSupport::benchTestMotorConfig(2));
  Devices::Otos otos(plant, Devices::OtosConfig{});
  Devices::ColorSensorLeaf color(plant, Devices::ColorConfig{});
  Devices::LineSensorLeaf line(plant, Devices::LineConfig{});

  TestSupport::FakeTransport serialFake;
  TestSupport::FakeTransport radioFake;
  App::Comms comms(serialFake, radioFake, "DEVICE:NEZHA2:robot:test:0");
  App::Telemetry tlm(comms, serialFake, radioFake);
  App::Drive drive(motorL, motorR, /*trackWidth=*/120.0f);
  App::Odometry odom(motorL, motorR, /*trackWidth=*/120.0f);
  // Default weights -- sourcing from Config::defaultEstimatorConfig() is
  // main.cpp's/sim_harness.h's own job (AC #1), not this unit test's
  // concern; this scenario only cares about update()'s own call site and
  // its effect on wheel/body validity/tracking. Constructed before
  // moveQueue (turn-prediction campaign) -- see the earlier scenarios'
  // own comment for why.
  App::StateEstimator stateEstimator;
  App::MoveQueue moveQueue(drive, odom, clock);
  App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  App::RobotLoop robotLoop(plant, motorL, motorR, otos, color, line, comms, tlm,
                            drive, odom, moveQueue, preamble, stateEstimator, clock,
                            sleeper);
  driveLivePlantBootToDone(preamble, clock);
  robotLoop.boot();
  robotLoop.markConfigured();

  checkTrue(!stateEstimator.wheelNow(App::Wheel::Left).valid,
            "left wheel peer starts invalid -- update() has not run yet (boot() never calls "
            "cycle())");
  checkTrue(!stateEstimator.whereAmI(0).valid,
            "body peer starts invalid -- update() has not run yet");

  constexpr float kCommandedVx = 300.0f;  // [mm/s]
  const uint32_t kMoveCorrId = 98001;
  const uint32_t kMoveId = 92;
  std::string moveLine = armorMoveTimeTwistCommand(
      /*includeVelocity=*/true, kCommandedVx, /*omega=*/0.0f, /*includeStop=*/true,
      /*stopTimeMs=*/5000.0f, /*timeoutMs=*/5000.0f, /*replace=*/true, kMoveId, kMoveCorrId);
  checkTrue(!moveLine.empty(), "armor() of the MOVE envelope succeeds");
  serialFake.enqueueInbound(moveLine.c_str());

  // ~1s of virtual ramp time -- comfortably >> the plant's default tau
  // (TestSim::kDefaultTau, 130ms, per sim_api_harness.cpp's own
  // scenarioTwistDrivesRealPlantRamp() derivation).
  constexpr int kCycles = 20;
  for (int i = 0; i < kCycles; ++i) {
    plant.tick(static_cast<float>(kCycleDtUs) / 1e6f);  // [s]
    clock.advanceMicros(kCycleDtUs);
    robotLoop.cycle();
  }

  uint32_t nowMs = static_cast<uint32_t>(clock.nowMicros() / 1000);

  App::WheelEstimate wheelL = stateEstimator.wheelNow(App::Wheel::Left);
  App::WheelEstimate wheelR = stateEstimator.wheelNow(App::Wheel::Right);
  checkTrue(wheelL.valid, "left wheel peer is valid after several cycles of motion");
  checkTrue(wheelR.valid, "right wheel peer is valid after several cycles of motion");
  checkTrue(wheelL.velocity > 100.0f,
            "left wheel peer's own velocity tracks the commanded forward motion (positive, "
            "well above zero)");
  checkTrue(wheelR.velocity > 100.0f,
            "right wheel peer's own velocity tracks the commanded forward motion (positive, "
            "well above zero)");

  App::BodyEstimate body = stateEstimator.whereAmI(nowMs);
  checkTrue(body.valid, "body peer is valid after several cycles of motion");
  checkTrue(body.v_x > 100.0f,
            "body peer's own v_x tracks the commanded forward twist (positive, well above "
            "zero)");
  checkFloatEq(body.v_y, 0.0f,
               "body peer's own v_y stays zero -- a straight twist commands no lateral motion",
               /*tol=*/1.0f);

  // (2) No tracking regression: the REAL encoder-derived velocity (not the
  // estimator's own copy of it) still ramps toward the commanded speed.
  checkTrue(motorL.velocity() > 100.0f,
            "left motor's REAL encoder-derived velocity ramped toward the commanded speed -- "
            "no regression from the estimator's own addition to the schedule");
  checkTrue(motorR.velocity() > 100.0f,
            "right motor's REAL encoder-derived velocity ramped toward the commanded speed -- "
            "no regression from the estimator's own addition to the schedule");

  // (3) AC #6: dispatch a live EstimatorConfigPatch and confirm THIS SAME
  // `stateEstimator` object -- the one update() has been refreshing every
  // cycle above -- picks it up, not a second, disconnected instance.
  const uint32_t kEstimatorCorrId = 98002;
  Buf estimatorPatch;
  putFloatField(estimatorPatch, 1, 0.4f);  // weight_heading_otos, field 1
  Buf estimatorDelta;
  putMessageField(estimatorDelta, 6, estimatorPatch);  // ConfigDelta.estimator, field 6
  Buf estimatorEnv;
  putVarintField(estimatorEnv, 1, kEstimatorCorrId);  // CommandEnvelope.corr_id
  putMessageField(estimatorEnv, 6, estimatorDelta);   // CommandEnvelope.config, field 6
  std::string estimatorLine = armorLine(estimatorEnv.data, estimatorEnv.len);
  checkTrue(!estimatorLine.empty(), "armor() of the CONFIG{estimator} envelope succeeds");
  serialFake.enqueueInbound(estimatorLine.c_str());

  plant.tick(static_cast<float>(kCycleDtUs) / 1e6f);  // [s]
  clock.advanceMicros(kCycleDtUs);
  robotLoop.cycle();

  checkFloatEq(stateEstimator.weights().headingOtos, 0.4f,
               "AC #6: handleConfig()'s ESTIMATOR branch (ticket 003) mutated the SAME "
               "stateEstimator instance this ticket's cycle()-call-site update() has been "
               "refreshing every cycle -- not a second, disconnected instance");
}

}  // namespace

int main() {
  scenarioBootThenAFewCyclesRunToCompletion();
  scenarioConfigMotorAppliesWhileDrivetrainStaysUnimplemented();
  scenarioConfigPersistWritePolicySkipsRedundantSave();

  scenarioMoveConfigGateRefusesWhenUnconfigured();
  scenarioMoveBadArgShapeValidation();
  scenarioMoveSuccessfulEnqueueAcksAndActivates();
  scenarioMoveEndDrainsWithNoFurtherHostTraffic();
  scenarioMoveTimeoutSetsFaultFlagOnEndingCycle();
  scenarioConfigMidMoveDoesNotChangeMoveCompletionOutcome();
  scenarioMoveDistanceStopReadsThisCyclesOdometryNotLastCycles();

  scenarioConfigEstimatorAppliesPresentFieldMergeAndNeverPersists();
  scenarioStateEstimatorTracksCommandedMotionNoTrackingRegression();

  if (g_failureCount == 0) {
    std::printf("OK: all App::RobotLoop scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::RobotLoop scenarios\n", g_failureCount);
  return 1;
}
