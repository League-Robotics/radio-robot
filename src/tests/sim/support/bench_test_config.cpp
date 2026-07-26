// bench_test_config.cpp -- see bench_test_config.h's own file header for
// scope and rationale (114-001). benchTestMotorConfig() below is byte-for-
// byte the deleted TestSim::SimHarness::makeMotorConfig(uint32_t) body
// (src/sim/sim_harness.h) -- every field and every explanatory comment
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

Devices::MotorConfig benchTestMotorConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  // mm per encoder count (== mm/motor-degree, 360/rev). Must be the RECIPROCAL
  // of sim_plant.cpp's kEncoderCountsPerMm so counts*travelCalib round-trips
  // to true mm (1.4187 * 0.704871 == 1.0). The GUI overrides this at connect
  // with the geometry-derived ml/mr push (~0.70486), which agrees.
  cfg.wheelTravelCalib = 0.704871f;
  cfg.velFiltAlpha = 1.0f;
  cfg.slewRate = 100.0f;
  // Velocity feedforward so the sim tracks the COMMANDED velocity (like the
  // real robot's calibrated gains do), instead of under-tracking ~17% on
  // pure-P and undershooting every drive/turn. kff = 1/kDefaultDutyVelMax:
  // duty = target/500 -> plant velocity = 500*duty = target (open-loop
  // exact), with kp trimming transients/disturbance.
  cfg.velGains.kff = 1.0f / TestSim::kDefaultDutyVelMax;  // 0.002 duty per mm/s
  // 114-006 (SUC-006 precondition): matches data/robots/tovez_nocal.json's
  // shipped control.vel_kp=0.002 -- this field used to hardcode 0.003 (the
  // pre-113 value the sim silently ran before config-as-truth), exactly the
  // class of divergence bench_test_config.h's own header warns against.
  // kff above already tracks the commanded velocity open-loop-exact on its
  // own (duty = target/500 -> plant velocity = target); kp is a small
  // closed-loop trim on top of that -- still needed (kp=0 lands 90deg turns
  // ~30deg off + faults, per the original finding below), just a smaller
  // trim at 0.002 than the stale 0.003 was.
  cfg.velGains.kp = 0.002f;   // feedback trim -- needed for turn accuracy
                             // (kp=0 lands 90deg turns ~30deg off + faults)
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

void configureSimForBenchTest(TestSim::SimHarness& sim) {
  sim.configureMotor(1, benchTestMotorConfig(1));
  sim.configureMotor(2, benchTestMotorConfig(2));
}

}  // namespace TestSupport
