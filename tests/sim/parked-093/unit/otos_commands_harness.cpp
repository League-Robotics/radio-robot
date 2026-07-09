// otos_commands_harness.cpp — off-hardware acceptance harness, originally
// for ticket 084-008 (proving all seven OTOS verbs replied "ERR nodev
// <verb>" against Subsystems::NezhaHardware, whose odometer() was nullptr —
// no real-hardware OTOS driver existed then). Ticket 086-006 gave
// NezhaHardware a real Hal::OtosOdometer member and an odometer() override
// that always returns its address — hardware.odometer() is NEVER nullptr
// for NezhaHardware any more, so otosReady()'s nodev guard (otos_commands.cpp,
// rewritten pointerless 087-006 — reads bb.otosPresent, a boot-time snapshot
// of hardware.odometer() != nullptr) can no longer fire against this owner.
// This harness now proves the opposite, current invariant: all seven verbs
// reach the real dispatch path and reply OK — driven through the FULL
// Rt::CommandRouter/Rt::Blackboard path (not just a bare
// `odometer() != nullptr` check), mirroring hardware_seam_harness.cpp's own
// "prove it through the real seam, not a mock" discipline.
//
// No I2C bus scripting is used: hardware.begin() is deliberately never
// called, so the real OtosOdometer's own product-ID probe never ran and it
// stays uninitialized (Hal::OtosOdometer::initialized_ == false) — its five
// primitive setters are then no-ops (see otos_odometer.h's own doc comment),
// and pose() returns the zero default. This is intentional: the point of
// this harness is the WIRE-DISPATCH-LEVEL guard (does bb.otosPresent see a
// non-null Odometer*), not real hardware I/O — that is otos_odometer_harness.cpp's
// job (086-006's own new host harness, a scripted-I2CBus proof of the leaf's
// register sequencing).
//
// Same HOST_BUILD scripted I2CBus fake + real source/hal/nezha/nezha_motor.cpp
// + source/subsystems/nezha_hardware.cpp + source/hal/otos/otos_odometer.cpp
// as hardware_seam_harness.cpp/test_hardware_seam.py, plus every command
// family Rt::CommandRouter's constructor registers (087-006: ONE unified
// table, liveness + all six families -- see command_router.cpp) and their
// transitive dependencies (subsystems/{drivetrain,pose_estimator,planner}.cpp,
// kinematics/body_kinematics.cpp, motion/{velocity_ramp,stop_condition}.cpp,
// estimation/ekf_tiny.cpp, telemetry/tlm_frame.cpp) -- CommandRouter's table
// is built unconditionally regardless of which family this harness actually
// dispatches through, so every family's own .cpp must link even though only
// otos_commands.cpp's own dispatch is exercised here.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "com/i2c_bus.h"
#include "runtime/blackboard.h"
#include "runtime/command_router.h"
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

// checkReply — dispatches `line` through the real Rt::CommandRouter (which
// unconditionally builds ALL SIX command families plus liveness, per
// command_router.cpp -- see this file's header) and asserts the reply is
// exactly `expected`.
void checkReply(Rt::CommandRouter& router, Rt::Blackboard& bb, const char* line,
                const char* expected) {
  beginScenario(std::string("dispatch: ") + line);
  CaptureReply reply;
  router.setReplyChannels(storeReply, &reply, storeReply, &reply);

  Subsystems::CommunicatorToCommandProcessorCommand cmd;
  cmd.returnPath = Subsystems::Channel::SERIAL;
  std::snprintf(cmd.line, sizeof(cmd.line), "%s", line);

  router.route(cmd, bb);
  checkEq(reply.buf, expected, std::string("reply for '") + line + "'");
}

}  // namespace

int main() {
  resetDefaultConfigs();
  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware hardware(bus, g_defaultConfigs);
  // hardware.begin() is deliberately never called -- see file header.

  Rt::Blackboard bb;
  // odometer() now (086-006) always returns the real, if uninitialized,
  // OtosOdometer -- bb.otosPresent is a boot-time snapshot of that fact
  // (087-006: otos_commands.cpp reads this instead of a Subsystems::Hardware*
  // -- see otos_commands.h's file header).
  bb.otosPresent = (hardware.odometer() != nullptr);

  Rt::CommandRouter router;

  checkReply(router, bb, "OI", "OK oi");
  checkReply(router, bb, "OZ", "OK oz");
  checkReply(router, bb, "OR", "OK or");
  checkReply(router, bb, "OP", "OK pos x=0 y=0 h=0");
  checkReply(router, bb, "OV 0 0 0", "OK setpos x=0 y=0 h=0");
  checkReply(router, bb, "OL", "OK linear scalar=0");
  checkReply(router, bb, "OA", "OK angular scalar=0");

  if (g_failureCount == 0) {
    std::printf(
        "OK: all seven OTOS verbs reply OK against the real (086-006) Subsystems::NezhaHardware\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the OTOS dispatch scenarios\n", g_failureCount);
  return 1;
}
