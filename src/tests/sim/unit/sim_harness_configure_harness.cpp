// sim_harness_configure_harness.cpp -- ticket 113-002's own acceptance
// proof: TestSim::SimHarness::configureMotor() is a purely ADDITIVE
// config-load surface (SUC-001/SUC-002/SUC-005), plus the configuration-
// completeness gate (isConfigured()) it drives.
//
// REWRITTEN by 115-006 (gut S1 sim lockstep): the original file (113-002)
// also covered SimHarness::configurePlanner()/plannerConfig() and the
// setYawRateMax() sim-only hook -- all deleted by this ticket, since
// Motion::Executor/App::Pilot/App::HeadingSource (115-002's motion-stack
// excision) no longer exist for any of them to configure. What survives:
// configureMotor()'s own per-port additive contract (readable back via
// motorConfig()) and the motor-only configuration-completeness gate
// (isConfigured(), now gated on BOTH configureMotor() calls alone -- see
// sim_harness.h's own maybeMarkConfigured() comment).
//
// Every scenario below constructs its OWN fresh TestSim::SimHarness -- this
// file never touches any of the ~40 pre-existing sim harness files, and
// never asserts against them either (that regression -- "no existing
// src/tests/sim/ test file's assertions change" -- is covered by the full
// `uv run python -m pytest` run this ticket's own Testing section calls
// for, not by anything here). What THIS file proves, specifically:
//
//   1. A default-constructed SimHarness (configureMotor() NEVER called)
//      observes Devices::MotorConfig{}'s own all-zero default for both
//      ports, and isConfigured() is false.
//   2. configureMotor() with values that differ from that (all-zero)
//      default takes effect per PORT and is readable back via
//      motorConfig(port) -- and configuring one port never touches the
//      other port's own record.
//   3. isConfigured() flips true only once BOTH ports have been configured
//      -- false after just one, true after both (the motor-only
//      configuration-completeness gate this ticket's own maybeMarkConfigured()
//      comment documents).
//
// Compiled by test_sim_harness_configure.py against the same full
// HOST_BUILD dependency graph the other post-gut sim/unit harnesses (e.g.
// test_app_robot_loop.py) compile -- SimHarness composes the real
// App::RobotLoop graph -- see sim_harness.h's own header.
#include <cmath>
#include <cstdio>
#include <string>

#include "sim_harness.h"

namespace {

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

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-4f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

}  // namespace

int main() {
  std::printf("=== TestSim::SimHarness::configureMotor() / isConfigured() (113-002, 115-006) ===\n\n");

  // --- Scenario 1: default construction, never configured -- observes
  //     Devices::MotorConfig{}'s own all-zero default for BOTH ports, and
  //     isConfigured() is false. ---
  {
    beginScenario("default-constructed SimHarness observes Devices::MotorConfig{}'s own "
                  "all-zero default (114-001: no self-configuration baseline anymore)");
    TestSim::SimHarness sim;
    sim.boot();

    checkFloatEq(sim.motorConfig(1).velFiltAlpha, 0.0f,
                 "left motorConfig starts at Devices::MotorConfig{}'s zero default");
    checkFloatEq(sim.motorConfig(2).velFiltAlpha, 0.0f,
                 "right motorConfig starts at Devices::MotorConfig{}'s zero default");
    checkFalse(sim.isConfigured(), "isConfigured() is false before any configureMotor() call");
  }

  // --- Scenario 2: configureMotor() with non-default values takes effect
  //     per port, readable via motorConfig(port); configuring one port
  //     never touches the other port's own record. ---
  {
    beginScenario("configureMotor() takes effect per port, readable via motorConfig(port)");
    TestSim::SimHarness sim;
    sim.boot();

    Devices::MotorConfig cfgL;
    cfgL.port = 1;
    cfgL.fwdSign = -1;
    cfgL.velFiltAlpha = 0.87f;
    sim.configureMotor(1, cfgL);

    checkFloatEq(sim.motorConfig(1).velFiltAlpha, 0.87f, "left velFiltAlpha took effect");
    checkTrue(sim.motorConfig(1).fwdSign == -1, "left fwdSign took effect");
    checkFloatEq(sim.motorConfig(2).velFiltAlpha, 0.0f,
                 "right motorConfig unaffected by configureMotor(1, ...)");

    Devices::MotorConfig cfgR;
    cfgR.port = 2;
    cfgR.fwdSign = 1;
    cfgR.velFiltAlpha = 0.42f;
    sim.configureMotor(2, cfgR);

    checkFloatEq(sim.motorConfig(2).velFiltAlpha, 0.42f, "right velFiltAlpha took effect");
    checkTrue(sim.motorConfig(2).fwdSign == 1, "right fwdSign took effect");
    checkFloatEq(sim.motorConfig(1).velFiltAlpha, 0.87f,
                 "left motorConfig unaffected by configureMotor(2, ...)");
  }

  // --- Scenario 3: isConfigured() flips true only once BOTH ports have
  //     landed a configureMotor() call -- the motor-only configuration-
  //     completeness gate (115-006: no planner half left to wait on). ---
  {
    beginScenario("isConfigured() gates on BOTH configureMotor() calls, not just one");
    TestSim::SimHarness sim;
    sim.boot();

    checkFalse(sim.isConfigured(), "isConfigured() false immediately after construction");

    Devices::MotorConfig cfgL;
    cfgL.port = 1;
    sim.configureMotor(1, cfgL);
    checkFalse(sim.isConfigured(), "isConfigured() still false after ONLY the left port is configured");

    Devices::MotorConfig cfgR;
    cfgR.port = 2;
    sim.configureMotor(2, cfgR);
    checkTrue(sim.isConfigured(), "isConfigured() true once BOTH ports are configured");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all SimHarness::configureMotor()/isConfigured() scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the SimHarness configure() scenarios\n", g_failureCount);
  return 1;
}
