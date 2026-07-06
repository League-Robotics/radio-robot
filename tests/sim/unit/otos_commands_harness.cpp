// otos_commands_harness.cpp — off-hardware acceptance harness for ticket
// 084-008: proves all seven OTOS verbs (OI/OZ/OR/OP/OV/OL/OA) reply
// "ERR nodev <verb>" against the REAL Subsystems::NezhaHardware (whose
// odometer() is still nullptr — no real-hardware OTOS driver this program,
// clasi/issues/nezha-hardware-otos-driver-for-new-source-tree.md), driven
// through the FULL otosCommands()/CommandProcessor dispatch path (not just
// a bare `odometer() == nullptr` check) — mirrors hardware_seam_harness.cpp's
// own "prove it through the real seam, not a mock" discipline. Every one of
// the seven verbs gets its own scenario, per this ticket's own acceptance
// criteria ("verified by an explicit test for every one of the seven, not
// just a subset").
//
// Same HOST_BUILD scripted I2CBus fake + real source/hal/nezha/nezha_motor.cpp
// + source/subsystems/nezha_hardware.cpp as hardware_seam_harness.cpp/
// test_hardware_seam.py (see that file's own doc comment for the recipe this
// mirrors), plus this ticket's own source/commands/otos_commands.cpp,
// source/commands/command_processor.cpp, and source/commands/arg_parse.cpp
// (needed for the full CommandProcessor dispatch table, not just the
// Subsystems::Hardware seam). No I2C bus scripting is required: none of the
// seven verbs ever reach the motor/I2C layer when odometer() is null (the
// nodev guard fires before any hardware access is attempted).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "com/i2c_bus.h"
#include "commands/command_processor.h"
#include "commands/otos_commands.h"
#include "messages/motor.h"
#include "subsystems/hardware.h"
#include "subsystems/nezha_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors hardware_seam_harness.cpp /
// nezha_flipflop_harness.cpp) ---

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

void checkEq(const std::string& actual, const std::string& expected, const std::string& what) {
  if (actual != expected) {
    fail(what + " — expected \"" + expected + "\", got \"" + actual + "\"");
  }
}

// --- Fixture helpers (mirrors hardware_seam_harness.cpp's own) ---

msg::MotorConfig g_defaultConfigs[Subsystems::Hardware::kPortCount];

void resetDefaultConfigs() {
  for (uint32_t i = 0; i < Subsystems::Hardware::kPortCount; ++i) {
    g_defaultConfigs[i] = msg::MotorConfig{};
    g_defaultConfigs[i].setPort(i + 1).setFwdSign(1).setTravelCalib(1.0f);
  }
}

// captureReply — stores exactly one reply line (every verb dispatched below
// replies exactly once, synchronously).
struct CaptureReply {
  char buf[128] = {};
};

void storeReply(const char* msg, void* ctx) {
  CaptureReply* c = static_cast<CaptureReply*>(ctx);
  std::snprintf(c->buf, sizeof(c->buf), "%s", msg);
}

// checkNodev — dispatches `line` through the real CommandProcessor and
// asserts the reply is exactly "ERR nodev <verb>".
void checkNodev(CommandProcessor& cmd, const char* line, const char* verb) {
  beginScenario(std::string("nodev: ") + line);
  CaptureReply reply;
  cmd.process(line, storeReply, &reply);
  char expected[32];
  std::snprintf(expected, sizeof(expected), "ERR nodev %s", verb);
  checkEq(reply.buf, expected, std::string("reply for '") + line + "'");
}

}  // namespace

int main() {
  resetDefaultConfigs();
  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware hardware(bus, g_defaultConfigs);

  OtosCommandState state;
  state.hardware = &hardware;   // odometer() inherits Subsystems::Hardware's
                                 // defaulted-nullptr override — never set here.

  CommandProcessor cmd(otosCommands(state));

  checkNodev(cmd, "OI", "oi");
  checkNodev(cmd, "OZ", "oz");
  checkNodev(cmd, "OR", "or");
  checkNodev(cmd, "OP", "op");
  checkNodev(cmd, "OV 0 0 0", "ov");
  checkNodev(cmd, "OL", "ol");
  checkNodev(cmd, "OA", "oa");

  if (g_failureCount == 0) {
    std::printf(
        "OK: all seven OTOS verbs reply ERR nodev against Subsystems::NezhaHardware\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the OTOS nodev scenarios\n", g_failureCount);
  return 1;
}
