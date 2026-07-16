// app_robot_loop_harness.cpp -- off-hardware acceptance harness for ticket
// 105-001 (SUC-018), App::RobotLoop (source/app/robot_loop.{h,cpp}), the
// boot loop + main cycle body extracted from source/main.cpp.
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
// Migrated by sprint 108 ticket 009 off the deleted source/devices/
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
// TestSim::SimHarness/tests/sim/system/ now cover the SAME "whole loop end
// to end" ground with the REAL, live-responding SimPlant physics (no
// scripting) -- this harness's own value is the exact register-level
// transaction-budget/CONFIG-ack proof those system tests don't attempt.
//
// Mirrors app_preamble_harness.cpp's exact scripting/assertion-plumbing
// conventions (this codebase's established per-harness-file style).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "app/comms.h"
#include "app/deadman.h"
#include "app/drive.h"
#include "app/odometry.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/telemetry.h"
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
#include "support/fake_transport.h"

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

// The AckEntry{corr_id, status=ACK_STATUS_ERR(1), err_code=ERR_UNIMPLEMENTED(6)}
// sub-message's own 3 fields, concatenated -- AckEntry's fields (envelope.proto)
// are corr_id=1/status=2/err_code=3 (source/messages/wire.cpp's
// kFields_AckEntry), encoded in that fixed order with no other field
// interleaving inside one ring entry, so this exact byte run is a reliable,
// low-false-positive fingerprint of "this corr_id was acked ERR_UNIMPLEMENTED"
// wherever it appears in a raw decoded Telemetry frame.
Buf ackErrUnimplementedFingerprint(uint32_t corrId) {
  Buf b;
  putVarintField(b, 1, corrId);  // AckEntry.corr_id
  putVarintField(b, 2, 1);       // AckEntry.status = ACK_STATUS_ERR
  putVarintField(b, 3, 6);       // AckEntry.err_code = ERR_UNIMPLEMENTED
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
  App::Deadman deadman(clock);
  App::Drive drive(motorL, motorR, /*trackWidth=*/120.0f);
  App::Odometry odom(motorL, motorR, /*trackWidth=*/120.0f);
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

  App::RobotLoop robotLoop(plant, motorL, motorR, otos, comms, tlm, drive, odom,
                            deadman, preamble, clock, sleeper);

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
// 106-002/SUC-025: RobotLoop::cycle()'s CONFIG dispatch live-applies a
// MotorConfigPatch (both bound motors, acks OK) while DrivetrainConfigPatch/
// PlannerConfigPatch continue acking ERR_UNIMPLEMENTED, unchanged.
//
// A "quiet" fixture (robot never twisted, stays at encoder position 0 the
// whole run) so the ONLY thing that varies cycle to cycle is the ack ring --
// the "applies" half is proven directly by reading motorL/motorR's own
// gains() (owned by this test, passed into RobotLoop by reference, so no
// wire decoding is needed for that half at all); the "still
// ERR_UNIMPLEMENTED" half is proven by a raw-byte substring search for the
// AckEntry{corr_id, ERR, ERR_UNIMPLEMENTED} fingerprint in a captured
// outbound frame (no decode(ReplyEnvelope) codec exists -- see
// rawBytesFromArmoredLine()'s own comment) -- deliberately NOT a full-frame
// byte-equality check (app_telemetry_harness.cpp's own technique), since
// this fixture makes no claim about the other frame fields' exact values.
//
// Cycle bookkeeping (ring is FIFO depth 3, and an ack pushed during cycle
// N's dispatch is not visible in ANY emitted frame until cycle N+1's own
// emit, which runs BEFORE that cycle's own dispatch -- robot_loop.cpp's
// own cycle() ordering):
//   i=0,1  -- quiet warm-up (absorbs the documented one-time first-duty-
//             write quirk for both L (cycle 1) and R (cycle 0), matching
//             scenarioBootThenAFewCyclesRunToCompletion's own derivation)
//   i=2    -- inject CONFIG{motor}      -- dispatched+acked this cycle
//   i=3    -- inject CONFIG{drivetrain} -- dispatched+acked this cycle
//   i=4    -- inject CONFIG{planner}    -- dispatched+acked this cycle
//   i=5    -- no injection -- this cycle's own emit (block 2, before this
//             cycle's own dispatch) reflects the ring exactly as it stood
//             after i=4's dispatch: all three acks, none evicted (ring
//             depth 3, exactly 3 pushed).
// ===========================================================================

void scenarioConfigMotorPatchAppliesWhileDrivetrainPlannerStayUnimplemented() {
  beginScenario("RobotLoop CONFIG: MotorConfigPatch live-applies + acks OK; "
                "Drivetrain/PlannerConfigPatch stay ERR_UNIMPLEMENTED");

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
  App::Deadman deadman(clock);
  App::Drive drive(motorL, motorR, /*trackWidth=*/120.0f);
  App::Odometry odom(motorL, motorR, /*trackWidth=*/120.0f);
  App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  clock.setMicros(0);
  preamble.step();
  clock.setMicros(50000);
  scriptMotorBeginSuccess(bus);  // Left
  scriptMotorBeginSuccess(bus);  // Right
  scriptOtosBeginSuccess(bus);
  scriptColorBeginSuccess(bus);
  scriptLineBeginSuccess(bus);

  App::RobotLoop robotLoop(plant, motorL, motorR, otos, comms, tlm, drive, odom,
                            deadman, preamble, clock, sleeper);
  robotLoop.boot();
  checkTrue(preamble.done(), "boot() completes against the FakeTransport-based fixture too");

  // Confirmed pre-patch baseline: baseMotorConfig() leaves velGains at
  // Devices::Gains{}'s all-zero default.
  checkFloatEq(motorL.gains().kp, 0.0f, "left motor starts at the constructed (zero) kp");
  checkFloatEq(motorR.gains().kp, 0.0f, "right motor starts at the constructed (zero) kp");

  const uint32_t kMotorCorrId = 87654;
  const uint32_t kDrivetrainCorrId = 87655;
  const uint32_t kPlannerCorrId = 87656;

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

  // CONFIG{planner: min_speed=20} -- same rationale as drivetrain above.
  Buf plannerPatch;
  putFloatField(plannerPatch, 1, 20.0f);  // PlannerConfigPatch.min_speed
  Buf plannerDelta;
  putMessageField(plannerDelta, 3, plannerPatch);  // ConfigDelta.planner, field 3
  Buf plannerEnv;
  putVarintField(plannerEnv, 1, kPlannerCorrId);
  putMessageField(plannerEnv, 6, plannerDelta);
  std::string plannerLine = armorLine(plannerEnv.data, plannerEnv.len);
  checkTrue(!plannerLine.empty(), "armor() of the CONFIG{planner} envelope succeeds");

  // The clock must ADVANCE past kPrimaryPeriod (40ms) each cycle so every
  // cycle()'s own emit() call actually sends a FRESH primary frame
  // reflecting that cycle's own ring state (a frozen clock, as
  // scenarioBootThenAFewCyclesRunToCompletion above relies on, would let
  // Telemetry::emit() send its primary frame ONCE ever and never again --
  // fine for that scenario, which never inspects ring content, but wrong
  // here). Advancing past kPrimaryPeriod (40ms) also crosses Otos::
  // kReadPeriod (20ms), so every cycle needs a scripted OTOS burst too.
  //
  // 106-002's own tie-break fix means NOT every cycle's emit() is
  // necessarily the primary frame (a tie can resolve to secondary) -- track
  // tlm.primaryEmitCount() across each cycle() call and remember the line
  // for the LAST cycle it actually rose, rather than assuming
  // serialFake.sent().back() is always primary. A few extra "quiet" cycles
  // past the last dispatch (i=4) give the alternation room to land on
  // primary again after any tie.
  std::string lastPrimaryLine;
  uint32_t primaryCountSoFar = tlm.primaryEmitCount();
  uint64_t nowUs = 50000;
  for (int i = 0; i < 9; ++i) {
    clock.setMicros(nowUs);
    scriptMotorCycle(bus, /*positionMm=*/0.0f, /*extraDutyWrites=*/(i == 1) ? 1 : 0);  // Left
    scriptMotorCycle(bus, /*positionMm=*/0.0f, /*extraDutyWrites=*/(i == 0) ? 1 : 0);  // Right
    scriptOtosReadZeroPose(bus);

    if (i == 2) serialFake.enqueueInbound(motorLine.c_str());
    if (i == 3) serialFake.enqueueInbound(drivetrainLine.c_str());
    if (i == 4) serialFake.enqueueInbound(plannerLine.c_str());

    robotLoop.cycle();
    if (tlm.primaryEmitCount() > primaryCountSoFar) {
      lastPrimaryLine = serialFake.sent().back();
      primaryCountSoFar = tlm.primaryEmitCount();
    }
    nowUs += 41000;  // > kPrimaryPeriod -- guarantees the NEXT cycle's primaryDue() is true
  }

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

  // --- ack content, via raw-byte fingerprint search on the LAST primary
  // frame emitted (reflects the ring as it stood after all three dispatches
  // -- ring depth 3, exactly 3 pushed, none evicted). ---
  checkTrue(!lastPrimaryLine.empty(), "at least one primary frame was captured after the three dispatches");
  std::string lastFrame = rawBytesFromArmoredLine(lastPrimaryLine);
  checkTrue(!lastFrame.empty(), "the captured frame de-armors to non-empty raw bytes");

  checkTrue(containsSubBytes(lastFrame, ackErrUnimplementedFingerprint(kDrivetrainCorrId)),
            "CONFIG{drivetrain} still acks ERR_UNIMPLEMENTED (fingerprint found in the ring)");
  checkTrue(containsSubBytes(lastFrame, ackErrUnimplementedFingerprint(kPlannerCorrId)),
            "CONFIG{planner} still acks ERR_UNIMPLEMENTED (fingerprint found in the ring)");
  checkTrue(!containsSubBytes(lastFrame, ackErrUnimplementedFingerprint(kMotorCorrId)),
            "CONFIG{motor} does NOT ack ERR_UNIMPLEMENTED (no ERR/UNIMPLEMENTED fingerprint for its corr_id)");
}

}  // namespace

int main() {
  scenarioBootThenAFewCyclesRunToCompletion();
  scenarioConfigMotorPatchAppliesWhileDrivetrainPlannerStayUnimplemented();

  if (g_failureCount == 0) {
    std::printf("OK: all App::RobotLoop scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::RobotLoop scenarios\n", g_failureCount);
  return 1;
}
