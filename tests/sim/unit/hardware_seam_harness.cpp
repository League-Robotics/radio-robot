// hardware_seam_harness.cpp — off-hardware acceptance harness for ticket
// 081-002: proves Subsystems::Hardware (source/subsystems/hardware.h) is a
// REAL abstract seam, not just a declared one, by instantiating the one
// existing concrete owner, Subsystems::NezhaHardware, and driving
// begin()/motor()/tick()/both apply() overloads ENTIRELY through a
// Subsystems::Hardware* base pointer — before Subsystems::SimHardware
// (ticket 003) exists to provide a second implementation.
//
// This is a DIFFERENT proof from nezha_flipflop_harness.cpp, not a
// duplicate of it: that harness exercises NezhaHardware's own scheduling/
// distribution behavior in fine-grained detail (sequencing order, write-rate
// throttling, reversal-dwell timing, ...) through a `Subsystems::NezhaHardware&`.
// Every call in THIS file instead goes through `Subsystems::Hardware&`/
// `Subsystems::Hardware*` — never naming `Subsystems::NezhaHardware` again
// once the object is constructed — so a change that accidentally required
// some NezhaHardware-only API (breaking the abstract seam) would fail to
// COMPILE here, not just fail an assertion. Mirrors the existing harnesses'
// shape otherwise: hand-rolled assertions, PASS/FAIL per scenario, nonzero
// exit on any failure, same HOST_BUILD scripted I2CBus fake + same real
// source/hal/nezha/nezha_motor.cpp + source/subsystems/nezha_hardware.cpp
// compiled in (see test_hardware_seam.py).
#include <cstdint>
#include <cstdio>
#include <string>

#include "com/i2c_bus.h"
#include "messages/motor.h"
#include "runtime/queue.h"
#include "subsystems/hardware.h"
#include "subsystems/nezha_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (see nezha_flipflop_harness.cpp /
// dev_command_outbox_harness.cpp) ---

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
  if (!condition) fail(what + " — expected true, got false");
}

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " — expected false, got true");
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

// --- Fixture helpers (mirrors nezha_flipflop_harness.cpp's own) ---

constexpr uint16_t kAddr7 = 0x10;                                   // bare 7-bit (clear()'s convention)
constexpr uint16_t kWireAddr = static_cast<uint16_t>(kAddr7 << 1);  // 0x20 (write()/read()'s convention)

msg::MotorConfig g_defaultConfigs[Subsystems::Hardware::kPortCount];

void resetDefaultConfigs() {
  for (uint32_t i = 0; i < Subsystems::Hardware::kPortCount; ++i) {
    g_defaultConfigs[i] = msg::MotorConfig{};
    g_defaultConfigs[i].setPort(i + 1).setFwdSign(1).setTravelCalib(1.0f);
  }
}

// Pre-loads `count` identical (address, status=OK) writes and (address,
// dummy-4-byte, status=OK) reads — see nezha_flipflop_harness.cpp's own
// "Scripting model note" for why an over-sized, uniform pool is sufficient
// and self-detecting (errCount() catches any under-run).
void scriptGenerousPool(I2CBus& bus, int count) {
  static uint8_t canned[4] = {0, 0, 0, 0};
  for (int i = 0; i < count; ++i) {
    bus.scriptWrite(kWireAddr, /*status=*/0);
    bus.scriptRead(kWireAddr, canned, 4, /*status=*/0);
  }
}

msg::MotorCommand neutralCommand() {
  return msg::MotorCommand{}.setNeutral(msg::Neutral::COAST);
}

// --- Scenarios ------------------------------------------------------------

// 1. Construct the ONE existing concrete owner, but from this point on hold
//    it ONLY through the abstract Subsystems::Hardware* base pointer.
//    begin() dispatches through the base pointer to all four ports' real
//    NezhaMotor::begin()/hardReset() (bypassing the flip-flop scheduler
//    entirely, same as NezhaHardware::begin()'s own direct-call contract),
//    and motor() returns the Hal::Motor faceplate for every port — proving
//    both are real, callable virtual dispatch, not just declared.
void scenarioBeginAndMotorThroughBasePointer() {
  beginScenario("Subsystems::Hardware*: begin() + motor() dispatch through the abstract base pointer");
  resetDefaultConfigs();
  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware concreteHardware(bus, g_defaultConfigs);
  Subsystems::Hardware* hardware = &concreteHardware;   // NezhaHardware never named again below
  scriptGenerousPool(bus, 100);   // begin() drives all 4 ports' hardReset() directly, off-schedule

  hardware->begin();

  for (uint32_t port = 1; port <= Subsystems::Hardware::kPortCount; ++port) {
    char what[96];
    std::snprintf(what, sizeof(what), "port %u reports connected() after begin() through the base pointer",
                  static_cast<unsigned>(port));
    checkTrue(hardware->motor(port).connected(), what);
  }
  checkUintEq(bus.errCount(kAddr7), 0,
              "no script under-run across begin()'s direct per-port encoder priming");
}

// 2. tick() through the base pointer, idle schedule (no port ever
//    addressed): zero bus actions, exactly NezhaHardware's own decision-1
//    contract — driven entirely through Subsystems::Hardware::tick().
void scenarioTickThroughBasePointerIdleSchedule() {
  beginScenario("Subsystems::Hardware*: tick() idle schedule performs zero bus actions");
  resetDefaultConfigs();
  I2CBus::setClock(2000000);
  I2CBus bus;
  Subsystems::NezhaHardware concreteHardware(bus, g_defaultConfigs);
  Subsystems::Hardware* hardware = &concreteHardware;

  // 087-004: tick() gained a per-port motorIn[]/motorResetIn[] pair (never
  // posted to here -- a no-op consumption loop, see hardware.h's tick() doc
  // comment).
  Rt::Mailbox<msg::MotorCommand> motorIn[Subsystems::Hardware::kPortCount];
  bool motorResetIn[Subsystems::Hardware::kPortCount] = {false, false, false, false};

  for (uint32_t i = 0; i < 10; ++i) {
    hardware->tick(100 + i, motorIn, motorResetIn);
  }

  checkUintEq(bus.txnCount(kAddr7), 0, "10 idle Hardware::tick() calls performed zero I2C transactions");
}

// 3. apply(const Hal::CommandProcessorToHardwareCommand&) through the base
//    pointer: addresses port 1 (the `DEV M <n>` shape), and a subsequent
//    request/collect tick() pair (still entirely through the base pointer)
//    drives it to connected() — the unaddressed port 2 stays untouched.
void scenarioApplyCommandProcessorCommandThroughBasePointer() {
  beginScenario("Subsystems::Hardware*: apply(CommandProcessorToHardwareCommand) addresses and schedules a port");
  resetDefaultConfigs();
  I2CBus::setClock(3000000);
  I2CBus bus;
  Subsystems::NezhaHardware concreteHardware(bus, g_defaultConfigs);
  Subsystems::Hardware* hardware = &concreteHardware;
  scriptGenerousPool(bus, 20);

  Hal::CommandProcessorToHardwareCommand cmd;
  cmd.allPorts = false;
  cmd.count = 1;
  cmd.addressed[0].port = 1;
  cmd.addressed[0].command = neutralCommand();
  hardware->apply(cmd);

  // 087-004: never posted to in this scenario -- a no-op consumption loop.
  Rt::Mailbox<msg::MotorCommand> motorIn[Subsystems::Hardware::kPortCount];
  bool motorResetIn[Subsystems::Hardware::kPortCount] = {false, false, false, false};

  hardware->tick(1000, motorIn, motorResetIn);            // REQUEST_DUE
  I2CBus::advanceClock(4000);      // satisfy the request's own postClear
  hardware->tick(1010, motorIn, motorResetIn);            // COLLECT_DUE

  checkTrue(hardware->motor(1).connected(), "port 1 collected via base-pointer apply()/tick()");
  checkFalse(hardware->motor(2).connected(), "port 2 was never addressed — untouched");
  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 4. apply(const Hal::DrivetrainToHardwareCommand&) through the base
//    pointer: both wheels always addressed (never a broadcast) — the
//    `DEV DT` shape — and forwarded; the two unbound ports are never
//    touched.
void scenarioApplyDrivetrainCommandThroughBasePointer() {
  beginScenario("Subsystems::Hardware*: apply(DrivetrainToHardwareCommand) addresses both wheels");
  resetDefaultConfigs();
  I2CBus::setClock(4000000);
  I2CBus bus;
  Subsystems::NezhaHardware concreteHardware(bus, g_defaultConfigs);
  Subsystems::Hardware* hardware = &concreteHardware;
  scriptGenerousPool(bus, 40);

  Hal::DrivetrainToHardwareCommand dtCmd;
  dtCmd.wheel[0].port = 3;
  dtCmd.wheel[0].command = neutralCommand();
  dtCmd.wheel[1].port = 4;
  dtCmd.wheel[1].command = neutralCommand();
  hardware->apply(dtCmd);

  // 087-004: never posted to in this scenario -- a no-op consumption loop.
  Rt::Mailbox<msg::MotorCommand> motorIn[Subsystems::Hardware::kPortCount];
  bool motorResetIn[Subsystems::Hardware::kPortCount] = {false, false, false, false};

  for (int cycle = 0; cycle < 4; ++cycle) {
    hardware->tick(10 * static_cast<uint32_t>(cycle), motorIn, motorResetIn);
    I2CBus::advanceClock(4000);
    hardware->tick(10 * static_cast<uint32_t>(cycle) + 1, motorIn, motorResetIn);
  }

  checkTrue(hardware->motor(3).connected(), "left wheel (port 3) scheduled via base-pointer apply()/tick()");
  checkTrue(hardware->motor(4).connected(), "right wheel (port 4) scheduled via base-pointer apply()/tick()");
  checkFalse(hardware->motor(1).connected(), "port 1 (not a bound wheel) never touched");
  checkFalse(hardware->motor(2).connected(), "port 2 (not a bound wheel) never touched");
  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 5. (087-004) config()/state() through the base pointer -- a uniform,
//    port-indexed faceplate that does not require narrowing to a concrete
//    NezhaHardware& -- plus tick()'s motorIn[]/motorResetIn[] (Decision 2):
//    posting to one port's own Mailbox schedules that port ALONE (no
//    addressed-dispatch branch), and a motorResetIn flag is applied and
//    cleared, idempotently.
void scenarioConfigStateAndTickMotorInMotorResetInThroughBasePointer() {
  beginScenario(
      "Subsystems::Hardware*: config()/state() + tick()'s motorIn[]/motorResetIn[] "
      "through the base pointer");
  resetDefaultConfigs();
  I2CBus::setClock(5000000);
  I2CBus bus;
  Subsystems::NezhaHardware concreteHardware(bus, g_defaultConfigs);
  Subsystems::Hardware* hardware = &concreteHardware;
  scriptGenerousPool(bus, 60);

  for (uint32_t port = 1; port <= Subsystems::Hardware::kPortCount; ++port) {
    msg::MotorConfig readBack = hardware->config(port);
    checkTrue(readBack.port == g_defaultConfigs[port - 1].port,
              "config(port) returns the constructed config verbatim (port field)");
    checkTrue(readBack.fwd_sign == g_defaultConfigs[port - 1].fwd_sign,
              "config(port) returns the constructed config verbatim (fwd_sign)");
  }

  // Post independently to ports 1 and 2's own Mailbox -- Decision 2's
  // per-port independence, no addressed-dispatch branch -- plus a pending
  // reset on port 2, all consumed uniformly by tick().
  Rt::Mailbox<msg::MotorCommand> motorIn[Subsystems::Hardware::kPortCount];
  bool motorResetIn[Subsystems::Hardware::kPortCount] = {false, false, false, false};
  motorIn[0].post(neutralCommand());
  motorIn[1].post(neutralCommand());
  motorResetIn[1] = true;

  for (int cycle = 0; cycle < 4; ++cycle) {
    hardware->tick(10 * static_cast<uint32_t>(cycle), motorIn, motorResetIn);
    I2CBus::advanceClock(4000);
    hardware->tick(10 * static_cast<uint32_t>(cycle) + 1, motorIn, motorResetIn);
  }

  checkTrue(motorIn[0].empty(), "tick() drained motorIn[0]");
  checkTrue(motorIn[1].empty(), "tick() drained motorIn[1]");
  checkFalse(motorResetIn[1], "tick() cleared the consumed motorResetIn[1] flag");
  checkTrue(hardware->motor(1).connected(), "port 1 scheduled via motorIn[0] alone -- through the base pointer");
  checkTrue(hardware->motor(2).connected(), "port 2 scheduled via motorIn[1] -- through the base pointer");
  checkFalse(hardware->motor(3).connected(),
             "port 3 untouched -- neither motorIn[2] nor motorResetIn[2] was ever set");
  checkTrue(hardware->motor(2).softResetCount() >= 1,
            "port 2's motorResetIn flag really staged+applied a resetPosition() "
            "(observable via softResetCount, since port 2 was never at rest long "
            "enough for a hard reset)");

  msg::MotorState st = hardware->state(1);
  checkTrue(st.connected == hardware->motor(1).state().connected,
            "state(port) matches motor(port).state() exactly -- through the base pointer");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

}  // namespace

int main() {
  scenarioBeginAndMotorThroughBasePointer();
  scenarioTickThroughBasePointerIdleSchedule();
  scenarioApplyCommandProcessorCommandThroughBasePointer();
  scenarioApplyDrivetrainCommandThroughBasePointer();
  scenarioConfigStateAndTickMotorInMotorResetInThroughBasePointer();

  if (g_failureCount == 0) {
    std::printf("OK: Subsystems::Hardware abstract seam proven real via Subsystems::NezhaHardware\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Subsystems::Hardware seam scenarios\n", g_failureCount);
  return 1;
}
