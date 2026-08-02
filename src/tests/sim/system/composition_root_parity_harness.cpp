// composition_root_parity_harness.cpp -- 130-002's own acceptance proof
// (unify-sim-and-robot-composition-roots.md, SUC-003): sim and hardware
// construct IDENTICAL Motion::PlannerLimits/drive-calibration values by
// default -- any difference must be one of the THREE documented, explicit
// BootOverrides (trackWidth, controlPeriod/actuationDelay, otosConfig; see
// app/boot_wiring.h's own header) and nothing else.
//
// Cannot construct a REAL Devices::MicroBitI2CBus host-side (CODAL-only),
// so this harness does not build two full RobotGraphs and diff them
// end-to-end. Instead it proves the property that actually matters: the
// SAME boot_calibration.h functions (App::bootPlannerLimits(),
// App::effectiveTrackWidth(), Config::defaultDriveConfig()) are what BOTH
// composeRobot() call sites (src/firm/main.cpp with no overrides, and
// this harness computing the "hardware-equivalent" values directly) would
// read -- so comparing a freshly-booted TestSim::SimHarness's actual
// Motion::Planner::limits() against that SAME direct computation IS the
// parity check: every field must match except the three documented
// overrides, which must differ in EXACTLY the documented way.
#include <cmath>
#include <cstdio>
#include <string>

#include "app/boot_calibration.h"
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

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-4f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

void checkFloatNe(float actual, float notExpected, const std::string& what, float tol = 1e-4f) {
  if (std::fabs(actual - notExpected) <= tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "%s -- expected a value DIFFERENT from %g (documented override), got the "
                  "same value -- either the override stopped applying, or this is no longer "
                  "a genuine divergence",
                  static_cast<double>(notExpected), what.c_str());
    fail(buf);
  }
}

}  // namespace

int main() {
  std::printf("=== Composition-root parity: sim vs. hardware-equivalent PlannerLimits (130-002) ===\n\n");

  // The "hardware-equivalent" value: exactly what src/firm/main.cpp's own
  // composeRobot() call (no overrides) would install, computed via the
  // SAME boot_calibration.h functions, independent of any I2CBus.
  const msg::DrivetrainConfig drivetrainConfig = Config::defaultDrivetrainConfig();
  const float hwTrackWidth = App::effectiveTrackWidth(drivetrainConfig);
  const Motion::PlannerLimits hw = App::bootPlannerLimits(drivetrainConfig, hwTrackWidth);

  TestSim::SimHarness sim;
  const Motion::PlannerLimits& simLimits = sim.planner().limits();

  beginScenario("Every field NOT a documented override matches the hardware-equivalent value exactly");
  checkFloatEq(simLimits.vMax, hw.vMax, "vMax");
  checkFloatEq(simLimits.aMax, hw.aMax, "aMax");
  checkFloatEq(simLimits.aDecel, hw.aDecel, "aDecel");
  checkFloatEq(simLimits.omegaMax, hw.omegaMax, "omegaMax");
  checkFloatEq(simLimits.alphaMax, hw.alphaMax, "alphaMax");
  checkFloatEq(simLimits.alphaDecel, hw.alphaDecel, "alphaDecel");
  checkFloatEq(simLimits.jerkMax, hw.jerkMax, "jerkMax");
  checkFloatEq(simLimits.yawJerkMax, hw.yawJerkMax, "yawJerkMax");
  checkFloatEq(static_cast<float>(simLimits.requireSettle), static_cast<float>(hw.requireSettle),
               "requireSettle");
  checkFloatEq(simLimits.settleRestVelocity, hw.settleRestVelocity, "settleRestVelocity");
  checkFloatEq(simLimits.settleRestOmega, hw.settleRestOmega, "settleRestOmega");
  checkFloatEq(simLimits.settleWindow, hw.settleWindow, "settleWindow");
  checkFloatEq(simLimits.settleEpsilonLinear, hw.settleEpsilonLinear, "settleEpsilonLinear");
  checkFloatEq(simLimits.settleEpsilonAngular, hw.settleEpsilonAngular, "settleEpsilonAngular");
  checkFloatEq(simLimits.headingHoldGain, hw.headingHoldGain, "headingHoldGain");
  checkFloatEq(simLimits.velKff, hw.velKff, "velKff");
  checkFloatEq(simLimits.velKp, hw.velKp, "velKp");
  checkFloatEq(simLimits.velKi, hw.velKi, "velKi");
  checkFloatEq(simLimits.velIMax, hw.velIMax, "velIMax");
  checkFloatEq(simLimits.velKaff, hw.velKaff, "velKaff");
  checkFloatEq(simLimits.velIAccelGate, hw.velIAccelGate, "velIAccelGate");
  checkFloatEq(simLimits.dutyFloor, hw.dutyFloor, "dutyFloor");
  checkFloatEq(simLimits.trimKp, hw.trimKp, "trimKp");
  checkFloatEq(simLimits.trimKi, hw.trimKi, "trimKi");
  checkFloatEq(simLimits.trimIMax, hw.trimIMax, "trimIMax");
  checkFloatEq(simLimits.trimKaff, hw.trimKaff, "trimKaff");
  checkFloatEq(simLimits.trimMax, hw.trimMax, "trimMax");
  checkFloatEq(simLimits.decelPlanFraction, hw.decelPlanFraction, "decelPlanFraction");
  checkFloatEq(simLimits.velocityFilterWeight, hw.velocityFilterWeight, "velocityFilterWeight");

  beginScenario("trackWidth/controlPeriod/actuationDelay are the documented overrides -- "
                "genuinely different from the hardware-equivalent value, and equal to the "
                "documented sim value");
  checkFloatNe(hw.trackWidth, simLimits.trackWidth, "trackWidth diverges from hw (documented override)");
  checkFloatEq(simLimits.trackWidth, TestSim::kDefaultTrackWidth,
               "sim trackWidth equals TestSim::kDefaultTrackWidth (the sim's own fixture value)");
  checkFloatNe(hw.controlPeriod, simLimits.controlPeriod,
               "controlPeriod diverges from hw (documented override)");
  checkFloatEq(simLimits.controlPeriod, static_cast<float>(App::RobotLoop::kCycle),
               "sim controlPeriod equals App::RobotLoop::kCycle (the sim's own delivered cycle time)");
  checkFloatNe(hw.actuationDelay, simLimits.actuationDelay,
               "actuationDelay diverges from hw (documented override)");
  checkFloatEq(simLimits.actuationDelay, static_cast<float>(App::RobotLoop::kCycle),
               "sim actuationDelay equals App::RobotLoop::kCycle (the sim's own delivered cycle time)");

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: composition-root parity holds -- every non-overridden field matches, "
                "every override is exactly the documented one\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the composition-root parity check\n", g_failureCount);
  return 1;
}
