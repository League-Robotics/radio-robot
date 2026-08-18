// sim_harness_configure_harness.cpp -- ticket 113-002's own acceptance
// proof: TestSim::SimHarness::configureMotor() is a purely ADDITIVE
// config-load surface (SUC-001/SUC-002/SUC-005), plus the configuration-
// completeness gate (isConfigured()) it drives.
//
// REWRITTEN by 115-006 (gut S1 sim lockstep): the original file (113-002)
// also covered SimHarness::configurePlanner()/plannerConfig() and the
// setYawRateMax() sim-only hook -- all deleted by this ticket, since
// Motion::Executor/Core::Pilot/Core::HeadingSource (115-002's motion-stack
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
//      observes Hal::MotorConfig{}'s own all-zero default for both
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
// Core::RobotLoop graph -- see sim_harness.h's own header.
#include <cmath>
#include <cstdio>
#include <string>

#include "config/boot_config.h"
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
  //     Hal::MotorConfig{}'s own all-zero default for BOTH ports, and
  //     isConfigured() is false. ---
  {
    beginScenario("default-constructed SimHarness observes Hal::MotorConfig{}'s own "
                  "all-zero default (114-001: no self-configuration baseline anymore)");
    TestSim::SimHarness sim;
    sim.boot();

    checkFloatEq(sim.motorConfig(1).slewRate, 0.0f,
                 "left motorConfig starts at Hal::MotorConfig{}'s zero default");
    checkFloatEq(sim.motorConfig(2).slewRate, 0.0f,
                 "right motorConfig starts at Hal::MotorConfig{}'s zero default");
    checkFalse(sim.isConfigured(), "isConfigured() is false before any configureMotor() call");
  }

  // --- Scenario 2: configureMotor() with non-default values takes effect
  //     per port, readable via motorConfig(port); configuring one port
  //     never touches the other port's own record. ---
  {
    beginScenario("configureMotor() takes effect per port, readable via motorConfig(port)");
    TestSim::SimHarness sim;
    sim.boot();

    // 125-003 discriminated on wheelTravelCalib; that field is DELETED with
    // the counts-native leaf (the kernel rework), so this scenario now
    // discriminates on slewRate. Any still-live nonzero field does equally
    // well -- the scenario is about per-port propagation, not about which
    // field carries it.
    Hal::MotorConfig cfgL;
    cfgL.port = 1;
    cfgL.fwdSign = -1;
    cfgL.slewRate = 0.87f;
    sim.configureMotor(1, cfgL);

    checkFloatEq(sim.motorConfig(1).slewRate, 0.87f, "left slewRate took effect");
    checkTrue(sim.motorConfig(1).fwdSign == -1, "left fwdSign took effect");
    checkFloatEq(sim.motorConfig(2).slewRate, 0.0f,
                 "right motorConfig unaffected by configureMotor(1, ...)");

    Hal::MotorConfig cfgR;
    cfgR.port = 2;
    cfgR.fwdSign = 1;
    cfgR.slewRate = 0.42f;
    sim.configureMotor(2, cfgR);

    checkFloatEq(sim.motorConfig(2).slewRate, 0.42f, "right slewRate took effect");
    checkTrue(sim.motorConfig(2).fwdSign == 1, "right fwdSign took effect");
    checkFloatEq(sim.motorConfig(1).slewRate, 0.87f,
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

    Hal::MotorConfig cfgL;
    cfgL.port = 1;
    sim.configureMotor(1, cfgL);
    checkFalse(sim.isConfigured(), "isConfigured() still false after ONLY the left port is configured");

    Hal::MotorConfig cfgR;
    cfgR.port = 2;
    sim.configureMotor(2, cfgR);
    checkTrue(sim.isConfigured(), "isConfigured() true once BOTH ports are configured");
  }

  // --- Scenario 4 (133-005): the sim's Core::DifferentialDrive is calibrated with an
  //     IDENTITY Stage A wheel correction, whatever data/robots/*.json
  //     currently holds.
  //
  //     This is a REGRESSION GUARD with a price tag. Core::DifferentialDrive's wheel
  //     correction linearizes a real gearbox (measured = gain*commanded +
  //     intercept); TestSim::WheelPlant is already linear, so installing
  //     a hardware fit against it cancels nothing and just bends every
  //     leg -- asymmetrically, since the left and right gains
  //     deliberately differ. That rule was enforced only by a comment
  //     asserting the bake "happened to hold identity." 132-019 fitted
  //     tovez's real gearbox and baked it (1.0 -> 0.9075 left / 0.8
  //     right), the assumption silently stopped being true, and
  //     src/tests/testgui/test_tour_closure_gate.py's TOUR_1/ideal worst
  //     per-turn error went 21.8deg -> 53.8deg with nothing failing that
  //     was not already failing. See Core::BootOverrides::wheelCorrection
  //     (app/boot_wiring.h) for the full account.
  //
  //     Three parts, because "the sim runs identity" alone would go
  //     vacuously green the moment a robot JSON returned to identity for
  //     unrelated reasons:
  //       a. the composed sim really is identity  (the invariant)
  //       b. loadBaked(nullptr) really does yield the FILE's values
  //          (the hardware path is not weakened -- a real robot must
  //          still boot its own fitted gains, which ticket 004 depends on)
  //       c. loadBaked(&identity) really does override them
  //          (the seam is load-bearing, whatever the bake holds today)
  {
    beginScenario("sim Drive is calibrated with an IDENTITY wheel correction regardless of "
                  "the baked robot JSON (133-005 regression guard)");
    TestSim::SimHarness sim;
    sim.boot();

    // (a) The invariant itself, read off the ONE configuration object the
    // whole install fans out from (Configurator::config()).
    const msg::Drive& live = sim.configurator().config().drive;
    checkFloatEq(live.wheel_gain_left_accel, 1.0f, "sim wheel_gain_left_accel is identity");
    checkFloatEq(live.wheel_gain_left_decel, 1.0f, "sim wheel_gain_left_decel is identity");
    checkFloatEq(live.wheel_gain_right_accel, 1.0f, "sim wheel_gain_right_accel is identity");
    checkFloatEq(live.wheel_gain_right_decel, 1.0f, "sim wheel_gain_right_decel is identity");
    checkFloatEq(live.wheel_intercept_left_accel, 0.0f, "sim wheel_intercept_left_accel is zero");
    checkFloatEq(live.wheel_intercept_left_decel, 0.0f, "sim wheel_intercept_left_decel is zero");
    checkFloatEq(live.wheel_intercept_right_accel, 0.0f, "sim wheel_intercept_right_accel is zero");
    checkFloatEq(live.wheel_intercept_right_decel, 0.0f, "sim wheel_intercept_right_decel is zero");

    // (b) No override == the file wins. This is the hardware path: main.cpp
    // passes no BootOverrides::wheelCorrection, so a real robot boots
    // whatever data/robots/<robot>.json was baked, unchanged.
    const msg::Drive baked = Config::defaultDriveGroup();
    sim.configurator().loadBaked(nullptr);
    const msg::Drive& afterBare = sim.configurator().config().drive;
    checkFloatEq(afterBare.wheel_gain_left_accel, baked.wheel_gain_left_accel,
                 "loadBaked(nullptr) yields the BAKED left gain (hardware path unchanged)");
    checkFloatEq(afterBare.wheel_gain_right_accel, baked.wheel_gain_right_accel,
                 "loadBaked(nullptr) yields the BAKED right gain (hardware path unchanged)");

    // (c) The override is load-bearing. Deliberately checked against a
    // NON-identity bake where one is present: if the active robot JSON is
    // itself at identity today, this still exercises the plumbing, and the
    // printed note says so rather than letting a vacuous pass look strong.
    const bool bakeIsIdentity = std::fabs(baked.wheel_gain_left_accel - 1.0f) < 1e-4f &&
                                std::fabs(baked.wheel_gain_right_accel - 1.0f) < 1e-4f;
    std::printf("  note: baked wheel gains are L=%g R=%g (%s)\n",
                static_cast<double>(baked.wheel_gain_left_accel),
                static_cast<double>(baked.wheel_gain_right_accel),
                bakeIsIdentity ? "identity today -- part (c) below is what keeps this "
                                 "scenario meaningful"
                               : "NON-identity -- part (a) above would fail without the "
                                 "override");

    const Config::WheelCorrection identity{};
    sim.configurator().loadBaked(&identity);
    const msg::Drive& afterOverride = sim.configurator().config().drive;
    checkFloatEq(afterOverride.wheel_gain_left_accel, 1.0f,
                 "loadBaked(&identity) overrides the baked left gain");
    checkFloatEq(afterOverride.wheel_gain_right_accel, 1.0f,
                 "loadBaked(&identity) overrides the baked right gain");
    checkFloatEq(afterOverride.wheel_intercept_right_decel, 0.0f,
                 "loadBaked(&identity) overrides the baked right decel intercept");

    // The override touches ONLY the wheel correction -- duty_per_speed and
    // crawl_pulse are the same Drive group and must still come from the file.
    checkFloatEq(afterOverride.duty_per_speed_left, baked.duty_per_speed_left,
                 "loadBaked(&identity) leaves duty_per_speed_left at its baked value");
    checkFloatEq(afterOverride.crawl_pulse, baked.crawl_pulse,
                 "loadBaked(&identity) leaves crawl_pulse at its baked value");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all SimHarness::configureMotor()/isConfigured() scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the SimHarness configure() scenarios\n", g_failureCount);
  return 1;
}
