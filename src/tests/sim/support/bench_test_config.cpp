// bench_test_config.cpp -- see bench_test_config.h's own file header for
// scope and rationale (114-001). benchTestMotorConfig() below is byte-for-
// byte the deleted TestSim::SimHarness::makeMotorConfig(uint32_t) body
// (src/firm/platform/host/sim_harness.h) -- every field and every explanatory comment
// carried over verbatim; only the enclosing function name/namespace and the
// #include this file needs to reach TestSim::SimHarness's public
// configureMotor() surface (configureSimForBenchTest(), below) changed.
//
// 115-006 (gut S1 sim lockstep): benchTestPlannerConfig() DELETED --
// msg::PlannerConfig and SimHarness::configurePlanner() no longer exist.
// See bench_test_config.h's own header.
#include "bench_test_config.h"

#include "sim_harness.h"

namespace TestSupport {

Hal::MotorConfig benchTestMotorConfig(uint32_t port) {
  Hal::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  // NOTE: wheelTravelCalib is GONE from Hal::MotorConfig -- the leaf is
  // counts-native now (position()/velocity() report encoder counts, never
  // mm), so there is no mm scale left to push at it. Travel calibration
  // moved up to the application, which is also where the counts<->mm
  // conversion for the sim plant now lives: SimHarness derives the kernel's
  // fullDutyVelocity from the baked travel_calib itself.
  cfg.slewRate = 100.0f;
  // PARITY (stakeholder 2026-07-18; UPDATED sprint 114 ticket 003):
  // reversalDwell/outputDeadband are now REQUIRED plain floats -- Devices::
  // MotorConfig no longer has an Opt<float> "unset -> ship default"
  // substitution at all (gen_boot_config.py always emits real values now,
  // baked from data/robots/*.json's control.reversal_dwell_ms/
  // output_deadband). Set explicitly here to the historical ship-default
  // values (100ms / 0.03) so this test harness keeps byte-identical
  // write-shaping behavior to before this ticket -- the sim gets no special
  // write-shaping configuration of its own; the whole motor stack behaves
  // identically in both places, only the far side of the I2C bus differs.
  cfg.reversalDwell = 100.0f;    // [ms]
  cfg.outputDeadband = 0.03f;    // [-1,1] fraction
  return cfg;
}

// DELETED with the DifferentialDrive kernel rework: Gains/benchTestGains().
// Its one surviving consumer was the sim.drive().setDutyPerSpeed(kff, kff)
// call below, and that method is gone -- the kernel takes a single
// fullDutyVelocity [counts/s] instead of a per-wheel duty-per-speed scale,
// and TestSim::SimHarness already derives it from the baked travel_calib at
// construction. Zero callers left, so the struct and its factory go rather
// than linger as config that reaches nothing (configuration-discipline:
// "a value in the file that nothing consumes ... delete it, don't wire it").

void configureSimForBenchTest(TestSim::SimHarness& sim) {
  sim.configureMotor(1, benchTestMotorConfig(1));
  sim.configureMotor(2, benchTestMotorConfig(2));
  // The duty-per-speed push and the shaper-ceiling override that used to
  // follow are both gone with the motion stack:
  //
  //  - sim.drive().setDutyPerSpeed(kff, kff) -- superseded by the kernel's
  //    single fullDutyVelocity [counts/s], which SimHarness sets from the
  //    baked travel_calib (see sim_harness.h's own comment for why the sim
  //    plant's exact inverse belongs there and not here).
  //  - sim.planner().applyShaperLimits(1e6, ...) -- Motion::Planner is
  //    DELETED (the whole src/firm/motion/ tree). There is no shaper left
  //    to un-shape: the kernel takes a commanded (velocity, twist) and the
  //    application does its own profiling, so a harness that wants a step
  //    input simply commands one.
}

}  // namespace TestSupport
