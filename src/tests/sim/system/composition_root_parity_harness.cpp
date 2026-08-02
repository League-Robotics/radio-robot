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

  // 130-009: PlannerLimits reshaped 34->18 fields under four sub-structs
  // (ceilings/plant/landing/tracking) -- every field check below just
  // grew its sub-struct prefix. The 16 fields cut by that ticket
  // (requireSettle/settleWindow, the M4 duty-stage gains velKff/velKp/
  // velKi/velIMax/velKaff/velIAccelGate/dutyFloor, and the dead
  // planner-side trim gains trimKp/trimKi/trimIMax/trimKaff/trimMax) have
  // no check here any more -- there is nothing left to compare.
  beginScenario("Every field NOT a documented override matches the hardware-equivalent value exactly");
  checkFloatEq(simLimits.ceilings.vMax, hw.ceilings.vMax, "ceilings.vMax");
  checkFloatEq(simLimits.ceilings.aMax, hw.ceilings.aMax, "ceilings.aMax");
  checkFloatEq(simLimits.ceilings.aDecel, hw.ceilings.aDecel, "ceilings.aDecel");
  checkFloatEq(simLimits.ceilings.omegaMax, hw.ceilings.omegaMax, "ceilings.omegaMax");
  checkFloatEq(simLimits.ceilings.alphaMax, hw.ceilings.alphaMax, "ceilings.alphaMax");
  checkFloatEq(simLimits.ceilings.alphaDecel, hw.ceilings.alphaDecel, "ceilings.alphaDecel");
  checkFloatEq(simLimits.ceilings.jerkMax, hw.ceilings.jerkMax, "ceilings.jerkMax");
  checkFloatEq(simLimits.ceilings.yawJerkMax, hw.ceilings.yawJerkMax, "ceilings.yawJerkMax");
  checkFloatEq(simLimits.landing.settleRestVelocity, hw.landing.settleRestVelocity,
               "landing.settleRestVelocity");
  checkFloatEq(simLimits.landing.settleRestOmega, hw.landing.settleRestOmega,
               "landing.settleRestOmega");
  checkFloatEq(simLimits.landing.settleEpsilonLinear, hw.landing.settleEpsilonLinear,
               "landing.settleEpsilonLinear");
  checkFloatEq(simLimits.landing.settleEpsilonAngular, hw.landing.settleEpsilonAngular,
               "landing.settleEpsilonAngular");
  checkFloatEq(simLimits.landing.decelPlanFraction, hw.landing.decelPlanFraction,
               "landing.decelPlanFraction");
  checkFloatEq(simLimits.tracking.headingHoldGain, hw.tracking.headingHoldGain,
               "tracking.headingHoldGain");
  checkFloatEq(simLimits.plant.velocityFilterWeight, hw.plant.velocityFilterWeight,
               "plant.velocityFilterWeight");

  beginScenario("trackWidth is the one documented override still genuinely different from "
                "the hardware-equivalent value, and equal to the documented sim value");
  checkFloatNe(hw.plant.trackWidth, simLimits.plant.trackWidth,
               "plant.trackWidth diverges from hw (documented override)");
  checkFloatEq(simLimits.plant.trackWidth, TestSim::kDefaultTrackWidth,
               "sim plant.trackWidth equals TestSim::kDefaultTrackWidth (the sim's own fixture value)");

  beginScenario("controlPeriod/actuationDelay: still a documented, explicit BootOverride "
                "structurally, but 130-007's one-50ms-period-everywhere change means the "
                "hardware-baked default and the sim's kCycle-derived value are now the SAME "
                "number by construction -- no longer a genuine divergence, and that IS the "
                "point (one period, not two)");
  checkFloatEq(hw.plant.controlPeriod, simLimits.plant.controlPeriod,
               "plant.controlPeriod: hw and sim now coincide (both honestly 50ms, 130-007)");
  checkFloatEq(simLimits.plant.controlPeriod, static_cast<float>(App::RobotLoop::kCycle),
               "sim plant.controlPeriod equals App::RobotLoop::kCycle (the sim's own delivered cycle time)");
  checkFloatEq(hw.plant.actuationDelay, simLimits.plant.actuationDelay,
               "plant.actuationDelay: hw and sim now coincide (both honestly 50ms, 130-007)");
  checkFloatEq(simLimits.plant.actuationDelay, static_cast<float>(App::RobotLoop::kCycle),
               "sim plant.actuationDelay equals App::RobotLoop::kCycle (the sim's own delivered cycle time)");

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: composition-root parity holds -- every non-overridden field matches, "
                "every override is exactly the documented one\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the composition-root parity check\n", g_failureCount);
  return 1;
}
