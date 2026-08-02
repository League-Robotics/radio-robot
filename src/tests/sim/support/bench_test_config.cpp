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

// benchTestGains -- 125-003: relocated from the pre-125-003
// benchTestMotorConfig()'s own velGains/velFiltAlpha (see bench_test_config.h's
// own header). Velocity feedforward so the sim tracks the COMMANDED
// velocity (like the real robot's calibrated gains do), instead of
// under-tracking ~17% on pure-P and undershooting every drive/turn. kff =
// 1/kDefaultDutyVelMax: duty = target/500 -> plant velocity = 500*duty =
// target (open-loop exact), with kp trimming transients/disturbance.
// 114-006 (SUC-006 precondition): kp matches data/robots/tovez_nocal.json's
// shipped control.vel_kp=0.002 -- kff above already tracks the commanded
// velocity open-loop-exact on its own; kp is a small closed-loop trim on
// top of that -- still needed (kp=0 lands 90deg turns ~30deg off + faults).
Gains benchTestGains() {
  Gains gains;
  gains.kff = 1.0f / TestSim::kDefaultDutyVelMax;  // 0.002 duty per mm/s
  gains.kp = 0.002f;   // feedback trim -- needed for turn accuracy
                       // (kp=0 lands 90deg turns ~30deg off + faults)
  return gains;
}

void configureSimForBenchTest(TestSim::SimHarness& sim) {
  sim.configureMotor(1, benchTestMotorConfig(1));
  sim.configureMotor(2, benchTestMotorConfig(2));
  // 130-007 REMOVAL NOTE: this used to also push benchTestGains() onto
  // Motion::Planner's own M4 duty stage (sim.planner().applyVelGains(...))
  // -- that stage is deleted outright (WheelPid/stageDuty(), reversing
  // sprint 128 Decision 2's PARK now that App::Drive's own controller is
  // the proven, shipped law); the call site is gone with it. Only the
  // duty-per-speed scale below survives -- see its own comment.
  {
    const Gains g = benchTestGains();
    // Open-loop wheel drive: kff IS the duty-per-speed scale for the sim
    // plant (1/kDefaultDutyVelMax).
    sim.drive().setDutyPerSpeed(g.kff, g.kff);
  }
  // SIM OVERRIDE (130-002, unify-sim-and-robot-composition-roots.md): the
  // sim now boots its shaper ceilings through the SAME composeRobot() path
  // hardware does (App::RobotGraph), which bakes the REAL robot JSON's
  // measured ramp/jerk ceilings (aMax 300 mm/s^2, alphaMax 6 rad/s^2, etc)
  // -- replacing the OLD sim-only TestSim::simPlannerLimits() literals,
  // which were "effectively UNSHAPED" (aMax 1e6) on purpose: every
  // sim/system harness that calls configureSimForBenchTest() tests
  // queue/stop-condition/protocol mechanics (chaining, replace, ESTOP/STOP,
  // ERR_FULL, ...) against a FIXED, hand-counted cycle budget that assumes
  // a commanded velocity lands in the wheel's target in ONE step -- not
  // motion-shaping fidelity, which has its own dedicated coverage in
  // src/motion/planner/tests/. Restore the historical unshaped ceilings
  // here, explicitly, so those fixed-cycle-count assertions keep meaning
  // what they always meant -- never let the real shaping in via silence.
  sim.planner().applyShaperLimits(/*aMax=*/1.0e6f, /*aDecel=*/1.0e6f, /*alphaMax=*/1.0e5f,
                                  /*alphaDecel=*/1.0e5f, /*jerkMax=*/0.0f, /*yawJerkMax=*/0.0f);
  // 130-005 REMOVAL NOTE: this used to also zero Motion::WheelTrim's
  // velocity-domain trim gains here, explicitly, so driveTargetVelLeft()/
  // Right() (state.wheelLeft/Right.cmdVelocity) kept landing EXACTLY on the
  // profiled target for these harnesses' fixed-tolerance "target is
  // committed" checks -- a live trim used to perturb that field by a small,
  // transient, CORRECT correction. Motion::WheelTrim is deleted outright
  // (wheel-speed-controller-moves-into-drive.md Phase 3): Motion::Planner
  // now publishes cmdVelocity bit-for-bit as the profiled command, always,
  // with no correction of any kind ever added to it -- App::Drive's own
  // Stage B/C controller corrects duty, a layer downstream of this field,
  // so there is nothing left here to zero.
}

}  // namespace TestSupport
