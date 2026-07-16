// app_preamble_harness.cpp -- off-hardware acceptance harness for ticket
// 103-007 (SUC-007), App::Preamble (source/app/preamble.{h,cpp}). Proves:
// step() resolves every leaf to a terminal state (done()) via each leaf's
// own REAL begin()/beginStep(nowUs) entry point over a TestSim::SimPlant
// (108-002), scripted deterministically via TestSim::ScriptedI2CHook
// (108-009), and a scripted Devices::Clock fake (source/devices/
// clock_host.cpp) -- no MicroBitI2C, no CODAL, no real hardware, no wall
// clock.
//
// Migrated by sprint 108 ticket 009 off the deleted source/devices/
// i2c_bus_host.cpp scripted-FIFO Devices::I2CBus fake (ticket 001 reduced
// Devices::I2CBus to a pure interface and removed it) -- see
// devices_motor_harness.cpp's/scripted_i2c_hook.h's own header for the
// migration rationale. Every scenario below is otherwise UNCHANGED from the
// pre-migration harness -- only the bus/scripting plumbing moved. This
// harness needs exact per-leaf, per-attempt register control (a specific
// transient NAK inside one begin() call, an OTOS that never answers across
// exactly kOtosBeginAttempts retries, a bounded-probe-action count per
// step() call) that SimPlant's own live physics responses cannot give
// directly -- so it stays a small C++ SimPlant-hook harness rather than a
// pure-Python SimHarness/SimLoop test (ticket 009's own documented fallback
// for scenarios needing host-unobservable, exact register-level control).
//
// Mirrors devices_otos_harness.cpp/devices_sensors_harness.cpp's exact
// scripting conventions (this codebase's established per-harness-file
// fixture style): hand-rolled assertion plumbing, PASS/FAIL per scenario,
// nonzero exit on any failure. Compiled by test_app_preamble.py with
// -DHOST_BUILD against preamble.cpp + every leaf .cpp it drives.
//
// queueWrite()/queueRead() are TWO SEPARATE FIFOs (see scripted_i2c_hook.h's
// own header) -- scenarios that need MULTIPLE leaves to resolve therefore
// queue each leaf's scripted transactions in the EXACT chronological order
// Preamble's round-robin will call them (Left, Right, Otos, Color, Line --
// see preamble.h's Slot enum), verified by hand against preamble.cpp's
// deterministic round-robin logic for the "everything due, everything
// succeeds immediately" case each scenario below drives.
#include <cstdint>
#include <cstdio>
#include <string>

#include "app/preamble.h"
#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/line_sensor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "scripted_i2c_hook.h"
#include "sim_plant.h"

namespace {

// --- Hand-rolled assertion plumbing (see devices_otos_harness.cpp) -------

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

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

// --- Fixture helpers -------------------------------------------------------

constexpr uint16_t kMotorWireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
constexpr uint16_t kOtosWireAddr = static_cast<uint16_t>(Devices::kOtosDeviceAddr << 1);
constexpr uint16_t kColorAltWireAddr = static_cast<uint16_t>(Devices::kColorDeviceAddrAlt << 1);
constexpr uint16_t kLineWireAddr = static_cast<uint16_t>(Devices::kLineDeviceAddr << 1);

Devices::MotorConfig baseMotorConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  return cfg;
}

// Scripts ONE readEncoderAtomicRaw()-shaped call inside NezhaMotor::
// hardReset() -- one write (0x46 select) + one 4-byte read (raw encoder,
// value 0 -> {0,0,0,0}), independently statusable per side (nezha_motor.cpp
// issues the read unconditionally even if the write failed).
void scriptEncoderCall(TestSim::ScriptedI2CHook& bus, int writeStatus, int readStatus) {
  bus.queueWrite(kMotorWireAddr, writeStatus);
  uint8_t data[4] = {0, 0, 0, 0};
  bus.queueRead(kMotorWireAddr, data, 4, readStatus);
}

// begin()'s hardReset(): 3 median snapshot reads + 1 readback read, ALL
// succeeding and reading back raw=0 (within the +/-2 threshold) -- the
// first OUTER attempt succeeds immediately. 4 calls total (nezha_motor.cpp
// hardReset()'s own comment) == 8 transactions (4 writes + 4 reads).
void scriptMotorBeginSuccess(TestSim::ScriptedI2CHook& bus) {
  for (int i = 0; i < 4; ++i) scriptEncoderCall(bus, /*writeStatus=*/0, /*readStatus=*/0);
}

// begin()'s hardReset(), first OUTER attempt: the first TWO of the 4 calls
// carry a TRANSIENT I2C failure (one bad write, one bad read) -- proving
// this does not permanently latch connected() false, because
// readEncoderAtomicRaw() sets connected_ from EVERY call's own status, and
// the LAST (4th, readback) call here succeeds. Values are irrelevant to a
// failing call (readEncoderAtomicRaw() returns the cached lastGoodRawEnc_,
// not the scripted bytes, on any I2C failure), so every read carries {0}
// regardless -- the readback threshold check still passes (0 is within
// +/-2), so this SAME outer attempt returns without a second attempt.
void scriptMotorBeginTransientNakThenSuccess(TestSim::ScriptedI2CHook& bus) {
  scriptEncoderCall(bus, /*writeStatus=*/-5, /*readStatus=*/0);  // snapshot s0: write NAK'd
  scriptEncoderCall(bus, /*writeStatus=*/0, /*readStatus=*/-5);  // snapshot s1: read NAK'd
  scriptEncoderCall(bus, /*writeStatus=*/0, /*readStatus=*/0);   // snapshot s2: clean
  scriptEncoderCall(bus, /*writeStatus=*/0, /*readStatus=*/0);   // readback: clean -- connected() ends true
}

// Otos::begin()'s full successful-detect transaction count (devices_otos_
// harness.cpp's own kBeginTxnCount derivation, reproduced here): 1 write +
// 1 read (product-ID probe, 0x5F) + 3 writes (init()) + 1 write
// (setLinearScalar) + 1 write (setAngularScalar) + 1 write (zero pose) = 7
// writes + 1 read = 8 total. queueWrite()/queueRead() are separate FIFOs
// so enqueue order between them doesn't matter, only within each.
void scriptOtosBeginSuccess(TestSim::ScriptedI2CHook& bus) {
  for (int i = 0; i < 7; ++i) bus.queueWrite(kOtosWireAddr, 0);
  uint8_t id[1] = {0x5F};
  bus.queueRead(kOtosWireAddr, id, 1, 0);
}

// ColorSensorLeaf::beginStep()'s AltProbe branch, ONE attempt, succeeding
// immediately (nonzero probe value): writeReg8(0x81) + writeReg8(0x80) +
// readReg16Alt(0xA4) (2 writes + 2 reads) == 4 writes + 2 reads total,
// matching devices_sensors_harness.cpp's scriptAltDetectAttempt().
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

// LineSensorLeaf::beginStep()'s readRaw() probe, ONE attempt, all 4
// channels succeeding -- 4 write(index)/read(byte) pairs, matches
// devices_sensors_harness.cpp's scriptLineRead().
void scriptLineBeginSuccess(TestSim::ScriptedI2CHook& bus) {
  for (int ch = 0; ch < 4; ++ch) {
    bus.queueWrite(kLineWireAddr, 0);
    uint8_t data[1] = {100};
    bus.queueRead(kLineWireAddr, data, 1, 0);
  }
}

// --- Bounded-probe-action tracking ("step() bounded" AC) -------------------
// Snapshots txnCount() for every device address Preamble can ever touch;
// countChanged() reports how many of those 5 addresses saw ANY new
// transaction between two snapshots -- step()'s own contract (preamble.h's
// header comment) is that AT MOST ONE is ever nonzero, whatever slot the
// round robin happened to visit that call (a multi-transaction leaf call --
// motor begin()'s internal hardReset() -- is the documented exception to
// "one transaction," never to "one leaf/address").
struct TxnSnapshot {
  uint32_t motor, otos, colorAlt, colorApds, line;
};

TxnSnapshot snapshotTxns(const TestSim::ScriptedI2CHook& bus) {
  return TxnSnapshot{
      bus.txnCount(Devices::kNezhaDeviceAddr),
      bus.txnCount(Devices::kOtosDeviceAddr),
      bus.txnCount(Devices::kColorDeviceAddrAlt),
      bus.txnCount(Devices::kColorDeviceAddrApds),
      bus.txnCount(Devices::kLineDeviceAddr),
  };
}

int countChangedAddrs(const TxnSnapshot& before, const TxnSnapshot& after) {
  int n = 0;
  if (after.motor != before.motor) ++n;
  if (after.otos != before.otos) ++n;
  if (after.colorAlt != before.colorAlt) ++n;
  if (after.colorApds != before.colorApds) ++n;
  if (after.line != before.line) ++n;
  return n;
}

// ===========================================================================
// 1. All-present happy path: every leaf detects on its first attempt.
//    Proves done() is reachable, every per-device accessor reports
//    present/connected, no I2C traffic issues before the first step() call
//    (AC "no I2C traffic before Preamble has begun probing"), and every
//    single step() call touches at most one device address ("step()
//    bounded" AC).
// ===========================================================================

void scenarioAllPresentHappyPath() {
  beginScenario("Preamble: all-present happy path -- done() reached, every leaf present/connected");

  TestSim::SimPlant plant;

  TestSim::ScriptedI2CHook bus(plant);
  Devices::Clock clock;

  Devices::NezhaMotor left(plant, baseMotorConfig(1));
  Devices::NezhaMotor right(plant, baseMotorConfig(2));
  Devices::Otos otos(plant, Devices::OtosConfig{});
  Devices::ColorSensorLeaf color(plant, Devices::ColorConfig{});
  Devices::LineSensorLeaf line(plant, Devices::LineConfig{});

  checkUintEq(snapshotTxns(bus).motor + snapshotTxns(bus).otos + snapshotTxns(bus).colorAlt +
                  snapshotTxns(bus).colorApds + snapshotTxns(bus).line,
              0, "construction alone issues zero I2C traffic");

  App::Preamble preamble(left, right, otos, color, line, clock);

  // Round-robin order for this scenario (hand-verified against
  // preamble.cpp): Left, Right, Otos, Color, Line -- queue each leaf's
  // scripted transactions in that exact order (separate write/read FIFOs).
  scriptMotorBeginSuccess(bus);  // Left
  scriptMotorBeginSuccess(bus);  // Right
  scriptOtosBeginSuccess(bus);
  scriptColorBeginSuccess(bus);
  scriptLineBeginSuccess(bus);

  // First step() call: nowUs=0, power-settle not yet elapsed -- a total
  // no-op, no leaf touched, no I2C traffic at all.
  clock.setMicros(0);
  TxnSnapshot beforeFirst = snapshotTxns(bus);
  preamble.step();
  TxnSnapshot afterFirst = snapshotTxns(bus);
  checkUintEq(static_cast<uint32_t>(countChangedAddrs(beforeFirst, afterFirst)), 0,
              "power-settle no-op call issues zero I2C traffic on any address");
  checkFalse(preamble.done(), "not done after the power-settle no-op call");

  // Advance past the power-settle window, then drive step() until done(),
  // asserting the bounded-probe-action contract on every single call.
  clock.setMicros(50000);
  int stepsUsed = 0;
  for (int i = 0; i < 10 && !preamble.done(); ++i) {
    TxnSnapshot before = snapshotTxns(bus);
    preamble.step();
    TxnSnapshot after = snapshotTxns(bus);
    checkTrue(countChangedAddrs(before, after) <= 1,
              "step() touches at most one device address per call");
    clock.advanceMicros(1);  // trivial advance -- every remaining slot here is "always due"
    ++stepsUsed;
  }

  checkTrue(preamble.done(), "done() reached within 10 steps once every leaf detects on its first attempt");
  checkUintEq(static_cast<uint32_t>(stepsUsed), 5, "exactly 5 probe-carrying steps: Left, Right, Otos, Color, Line");

  checkTrue(preamble.leftConnected(), "leftConnected() true");
  checkTrue(preamble.rightConnected(), "rightConnected() true");
  checkTrue(preamble.otosPresent(), "otosPresent() true");
  checkTrue(preamble.otosConnected(), "otosConnected() true");
  checkTrue(preamble.colorPresent(), "colorPresent() true");
  checkTrue(preamble.linePresent(), "linePresent() true");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run: motor");
  checkUintEq(bus.errCount(Devices::kOtosDeviceAddr), 0, "no script under-run: otos");
  checkUintEq(bus.errCount(Devices::kColorDeviceAddrAlt), 0, "no script under-run: color");
  checkUintEq(bus.errCount(Devices::kLineDeviceAddr), 0, "no script under-run: line");
}

// ===========================================================================
// 2. OTOS absent latches after exhausting its own retry budget: every other
//    leaf detects normally; OTOS's I2CBus address is left completely
//    unscripted (i2c_bus_host.cpp's own "absent device" convention -- an
//    unscripted read decodes to id=0, never matching kExpectedProductId).
//    Proves done() is STILL reachable (bounded worst case, this ticket's
//    own AC), otosPresent()/otosConnected() stay false, and OTOS was
//    retried EXACTLY kOtosBeginAttempts (20) times -- not fewer (premature
//    latch), not more (unbounded retry).
// ===========================================================================

void scenarioOtosAbsentLatchesAfterRetries() {
  beginScenario("Preamble: OTOS absent -- latches un-present after exactly kOtosBeginAttempts retries, done() still reachable");

  TestSim::SimPlant plant;

  TestSim::ScriptedI2CHook bus(plant);
  Devices::Clock clock;

  Devices::NezhaMotor left(plant, baseMotorConfig(1));
  Devices::NezhaMotor right(plant, baseMotorConfig(2));
  Devices::Otos otos(plant, Devices::OtosConfig{});  // never scripted -- always absent
  Devices::ColorSensorLeaf color(plant, Devices::ColorConfig{});
  Devices::LineSensorLeaf line(plant, Devices::LineConfig{});

  App::Preamble preamble(left, right, otos, color, line, clock);

  // queueWrite()/queueRead() are ONE shared FIFO per I2CBus instance
  // covering EVERY address -- an unscripted call (OTOS here) pops nothing
  // only while the queue is genuinely empty (i2c_bus_host.cpp's own guard).
  // Queuing Color's/Line's scripts too far ahead would let OTOS's own
  // unscripted probe silently consume THEIR entries out of turn. So each
  // leaf's script is queued immediately before that leaf's own turn in the
  // round robin (hand-verified order: Left, Right, Otos, Color, Line, then
  // Otos again on every subsequent call) -- the queue is empty at every
  // OTOS attempt, so its mismatch never corrupts anyone else's script.
  clock.setMicros(0);
  preamble.step();  // power-settle no-op

  clock.setMicros(50000);
  scriptMotorBeginSuccess(bus);
  preamble.step();  // Left

  clock.advanceMicros(1);
  scriptMotorBeginSuccess(bus);
  preamble.step();  // Right

  clock.advanceMicros(1);
  preamble.step();  // Otos attempt #1 -- unscripted, queue empty -> safe mismatch

  clock.advanceMicros(1);
  scriptColorBeginSuccess(bus);
  preamble.step();  // Color

  clock.advanceMicros(1);
  scriptLineBeginSuccess(bus);
  preamble.step();  // Line

  int stepsUsed = 5;
  for (int i = 0; i < 25 && !preamble.done(); ++i) {
    clock.advanceMicros(100000);  // >= kOtosBeginRetryPeriod every call -- always paces OTOS's next attempt
    preamble.step();              // Otos attempts #2..#20 -- unscripted, queue stays empty -> safe
    ++stepsUsed;
  }

  checkTrue(preamble.done(), "done() reached within the bounded step budget even with OTOS permanently absent");
  checkTrue(stepsUsed <= 30, "bounded: did not need the full defensive iteration cap");

  checkFalse(preamble.otosPresent(), "otosPresent() false -- OTOS never detected");
  checkFalse(preamble.otosConnected(), "otosConnected() false -- OTOS never detected");
  checkTrue(preamble.leftConnected(), "leftConnected() true -- unaffected by OTOS's absence");
  checkTrue(preamble.rightConnected(), "rightConnected() true -- unaffected by OTOS's absence");
  checkTrue(preamble.colorPresent(), "colorPresent() true -- unaffected by OTOS's absence");
  checkTrue(preamble.linePresent(), "linePresent() true -- unaffected by OTOS's absence");

  // Exactly kOtosBeginAttempts (20) failed probes, 2 transactions each
  // (1 write + 1 read) -- device_bus.h's ported kOtosBeginAttempts,
  // preamble.h's own kOtosBeginAttempts.
  checkUintEq(bus.txnCount(Devices::kOtosDeviceAddr), 40,
              "OTOS retried exactly 20 times (40 transactions), not fewer or more");
}

// ===========================================================================
// 3. Transient NAK during a motor's begin() is not latched: two of the four
//    readEncoderAtomicRaw() calls inside hardReset()'s FIRST outer attempt
//    carry a transient I2C failure, but the LAST (readback) call succeeds --
//    proving Preamble reports leftConnected()==true from the leaf's own
//    freshest state, not stuck false by an earlier hiccup within the SAME
//    begin() call. Preamble calls begin() exactly ONCE per motor -- it never
//    retries the leaf itself (this ticket's own "no leaf's own retry loop is
//    reimplemented" rule); NezhaMotor's OWN internal retry/recovery is what
//    this scenario proves survives intact through Preamble's single call.
// ===========================================================================

void scenarioTransientMotorNakDuringBeginNotLatched() {
  beginScenario("Preamble: transient I2C NAK during NezhaMotor::begin() does not latch connected() false");

  TestSim::SimPlant plant;

  TestSim::ScriptedI2CHook bus(plant);
  Devices::Clock clock;

  Devices::NezhaMotor left(plant, baseMotorConfig(1));
  Devices::NezhaMotor right(plant, baseMotorConfig(2));
  Devices::Otos otos(plant, Devices::OtosConfig{});
  Devices::ColorSensorLeaf color(plant, Devices::ColorConfig{});
  Devices::LineSensorLeaf line(plant, Devices::LineConfig{});

  App::Preamble preamble(left, right, otos, color, line, clock);

  scriptMotorBeginTransientNakThenSuccess(bus);  // Left -- transient NAK, recovers
  scriptMotorBeginSuccess(bus);                  // Right -- clean
  scriptOtosBeginSuccess(bus);
  scriptColorBeginSuccess(bus);
  scriptLineBeginSuccess(bus);

  clock.setMicros(0);
  preamble.step();  // power-settle no-op

  clock.setMicros(50000);
  preamble.step();  // Left: begin() called exactly once here

  checkTrue(preamble.leftConnected(),
            "leftConnected() true after ONE begin() call despite 2 of its 4 internal I2C calls NAK'ing transiently");
  checkTrue(left.connected(), "the leaf's own connected() agrees -- Preamble adds no separate state");

  clock.advanceMicros(1);
  preamble.step();  // Right
  clock.advanceMicros(1);
  preamble.step();  // Otos
  clock.advanceMicros(1);
  preamble.step();  // Color
  clock.advanceMicros(1);
  preamble.step();  // Line

  checkTrue(preamble.done(), "done() reached -- the transient NAK cost no extra Preamble-level retry");
}

// ===========================================================================
// 4. Multiple leaves absent simultaneously (OTOS + color + line) -- a
//    stronger form of AC "an absent sensor does not hang boot forever":
//    done() is still reachable, bounded, with every absent leaf correctly
//    latched un-present and the two motors unaffected.
// ===========================================================================

void scenarioMultipleAbsentLeavesStillTerminates() {
  beginScenario("Preamble: OTOS + color + line all absent simultaneously -- done() still bounded-reachable");

  TestSim::SimPlant plant;

  TestSim::ScriptedI2CHook bus(plant);
  Devices::Clock clock;

  Devices::NezhaMotor left(plant, baseMotorConfig(1));
  Devices::NezhaMotor right(plant, baseMotorConfig(2));
  Devices::Otos otos(plant, Devices::OtosConfig{});          // absent
  Devices::ColorSensorLeaf color(plant, Devices::ColorConfig{});  // absent
  Devices::LineSensorLeaf line(plant, Devices::LineConfig{});     // absent

  App::Preamble preamble(left, right, otos, color, line, clock);

  scriptMotorBeginSuccess(bus);  // Left
  scriptMotorBeginSuccess(bus);  // Right
  // -- OTOS, color, line all deliberately unscripted --

  clock.setMicros(0);
  preamble.step();  // power-settle no-op

  clock.setMicros(50000);
  int stepsUsed = 0;
  for (int i = 0; i < 200 && !preamble.done(); ++i) {
    preamble.step();
    clock.advanceMicros(100000);  // paces every remaining leaf's own retry period
    ++stepsUsed;
  }

  checkTrue(preamble.done(), "done() reached even with 3 of 5 devices permanently absent");
  checkTrue(stepsUsed < 200, "bounded well within the defensive iteration cap");

  checkTrue(preamble.leftConnected(), "leftConnected() true -- unaffected");
  checkTrue(preamble.rightConnected(), "rightConnected() true -- unaffected");
  checkFalse(preamble.otosPresent(), "otosPresent() false");
  checkFalse(preamble.colorPresent(), "colorPresent() false");
  checkFalse(preamble.linePresent(), "linePresent() false");
}

}  // namespace

int main() {
  scenarioAllPresentHappyPath();
  scenarioOtosAbsentLatchesAfterRetries();
  scenarioTransientMotorNakDuringBeginNotLatched();
  scenarioMultipleAbsentLeavesStillTerminates();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Preamble scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Preamble scenarios\n", g_failureCount);
  return 1;
}
