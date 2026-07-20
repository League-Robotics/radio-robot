// bench_test_config.cpp -- see bench_test_config.h's own file header for
// scope and rationale (114-001). The two value-producing functions below are
// byte-for-byte the deleted TestSim::SimHarness::makeMotorConfig(uint32_t)/
// makeExecutorConfig() bodies (src/sim/sim_harness.h) -- every field and
// every explanatory comment carried over verbatim; only the enclosing
// function name/namespace and the #include this file needs to reach
// TestSim::SimHarness's public configurePlanner()/configureMotor() surface
// (configureSimForBenchTest(), below) changed.
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
  cfg.velGains.kp = 0.003f;   // feedback trim -- needed for turn accuracy
                             // (kp=0 lands 90deg turns ~30deg off + faults)
  // PARITY (stakeholder 2026-07-18): reversalDwell/outputDeadband are
  // deliberately left UNSET -- exactly what the production boot config
  // bakes (gen_boot_config.py leaves both .has == false on purpose), so
  // NezhaMotor's ctor substitutes the SAME ship defaults (100ms / 0.03)
  // in the sim as on the robot. The sim gets no special write-shaping
  // configuration of its own -- the whole motor stack behaves
  // identically in both places; only the far side of the I2C bus
  // differs.
  return cfg;
}

msg::PlannerConfig benchTestPlannerConfig() {
  msg::PlannerConfig cfg;
  cfg.a_max = 800.0f;         // [mm/s^2]
  cfg.a_decel = 1000.0f;      // [mm/s^2]
  cfg.v_body_max = 600.0f;    // [mm/s]
  cfg.yaw_rate_max = 4.0f;    // [rad/s]
  cfg.yaw_acc_max = 20.0f;    // [rad/s^2]
  cfg.j_max = 8000.0f;        // [mm/s^3]
  cfg.yaw_jerk_max = 80.0f;   // [rad/s^3]
  // 109-005: heading PD cascade gain + dwell-completion gate. kp=6.0
  // matches data/robots/tovez.json's own bench-proven sprint-098 value
  // (see .clasi/knowledge/heading-loop-solves-turn-accuracy.md); the
  // dwell tolerance/rate/hold match this same file's own
  // planner.proto/gen_boot_config.py default derivation (0.5deg/1deg-per-
  // s/150ms).
  cfg.heading_kp = 2.5f;                     // [1/s]
  cfg.heading_kd = 0.0f;                     // dimensionless
  // Dwell tolerance 1.5deg (was 0.5deg) + min_speed 20mm/s (2026-07-18,
  // terminal stiction/deadband floor): with the write shaping honestly ON
  // (parity), the smallest wheel command that moves the plant is
  // ~outputDeadband/kff ~= 15mm/s -- the PD stalls below that, so
  // App::Pilot floors its terminal output at min_speed (pilot.cpp) and
  // the dwell tolerance must sit ABOVE where a floored approach can stop
  // (floor rate x plant decay ~= 1.3deg). Matches gen_boot_config.py's
  // own updated defaults -- same values both places.
  cfg.heading_dwell_tol = 3.0f * 3.14159265f / 180.0f;   // [rad]
  cfg.heading_dwell_rate = 1.0f * 3.14159265f / 180.0f;  // [rad/s]
  cfg.arrive_dwell = 0.15f;                  // [s]
  cfg.min_speed = 16.0f;                     // [mm/s] Pilot heading-PD floor: just above the ~15mm/s deadband cut; coast quantum = floor x (tau 0.13s + a write cycle) ~= 2.6deg < the 3deg dwell tol
  // 109-010: lead-compensation defaults. heading_lead_bias defaults to
  // -0.05 -- NOT 0.0 -- matching gen_boot_config.py's own shipped
  // HEADING_LEAD_BIAS_DEFAULT (see that constant's own doc comment for
  // the full characterization writeup): a genuinely UNCOMPENSATED raw
  // age lead (bias=0.0) was found, DURING this ticket's own work, to
  // actively FAULT pre-existing sim system tests that construct a
  // SimHarness/SimLoop and never call setLeadCompensation() at all
  // (test_sim_transport_tour1.py, heading_source_harness.cpp) -- this
  // class's own OTOS burst-read omega used to always read 0 (TestSim::
  // OtosPlant's own pre-109-010 stub), so the projection was silently
  // inert everywhere until this ticket's own OtosPlant::omega() fix
  // made it real; -0.05 (this harness's own 50ms kCycleDtUs, exactly
  // canceled) restores the pre-109-010 NO-OP behavior as the harness's
  // own default, matching the shipped firmware default's own posture.
  // plan_lead/terminal_lead default to 0.0 (genuine no-ops, unaffected
  // by the omega fix). setLeadCompensation() below overrides all three
  // for a test that wants to sweep them.
  cfg.heading_lead_bias = -0.05f;  // [s]
  cfg.plan_lead = 0.20f;           // [s] ~2 staging cycles + plant tau -- eliminates the terminal PD reversal (2026-07-18 sweep; matches gen_boot_config.py PLAN_LEAD_DEFAULT)
  cfg.terminal_lead = 0.0f;        // [s]
  // 112-002: App::Drive's own model feedforward gain -- matches
  // gen_boot_config.py's shipped ACTUATION_LAG_DEFAULT/Motion::kDeadTime's
  // own bench-derived value (120-140ms), so this harness's own
  // drive_.configure(cfg) call (constructor, setLeadCompensation(),
  // setYawRateMax()) actually exercises the feedforward path a real robot
  // boots with, not a silent 0.0f no-op.
  cfg.actuation_lag = 0.0f;      // [s]
  // 112-003: App::Pilot's own bounded linear position-feedback trim gain.
  // 112-004 UPDATE: no longer left at 0.0 -- Motion::Executor's own
  // unified completion rule reads `distance_tol` LIVE now (|sErr| <
  // distance_tol, the linear half of `done = t >= duration+margin AND
  // |sErr| < distance_tol AND |thetaErr| < heading_dwell_tol`), which
  // makes the trim's own closed-loop convergence load-bearing for
  // completion for the first time -- a plant with NO trim at all settles
  // several mm short of target (a real, measured lag-induced undershoot,
  // not a bug) and can never satisfy the completion rule's own tolerance
  // test, timing out instead of completing. 8.0 matches
  // gen_boot_config.py's own DISTANCE_KP_DEFAULT (112-004's own
  // empirically-swept, closed-loop-stable value -- see that constant's
  // own comment and pilot.cpp's own trim-gating comment for the full
  // derivation); every PRE-EXISTING sim scenario that never queried
  // completion timing precisely is unaffected either way, and
  // setDistanceKp() below still lets a test override this per-scenario.
  cfg.distance_kp = 2.5f;          // [1/s]
  // 112-004: same load-bearing-for-completion reasoning as distance_kp
  // above -- unlike distance_kp, 0.0 here is not even a directionally
  // safe fallback: a strict `<` against a 0 tolerance can never be
  // satisfied (the same reason heading_dwell_tol above is never left at
  // 0 either), so every DISTANCE-mode (kArc) scenario through THIS
  // harness needs a real value or its "not carrying" completion branch
  // can only ever reach kTimeout, never kDone. 3.0mm matches
  // gen_boot_config.py's own DISTANCE_TOL_DEFAULT and the value
  // Motion::kDistanceSettleEpsilonMm used to hardcode before this ticket
  // wired the live field in.
  cfg.distance_tol = 6.0f;         // [mm]
  // 113-001: App::Pilot's own two-stage model-reference feedback plant-lag
  // time constants (pilot.h's modelTauLin_/modelTauAng_) -- explicit here
  // so this harness's default construction path observes byte-for-byte
  // identical Pilot behavior to before this ticket. A msg::PlannerConfig{}
  // with these fields left at their zero-value default would silently
  // change modelTauLin_/modelTauAng_ to 0.0, turning tick()'s alphaLin/
  // alphaAng first-order-lag-toward-the-reference into an instant,
  // unfiltered reference -- a real behavior change for every existing sim
  // scenario/characterization test, not a no-op (SUC-005). Matches
  // pilot.h's own prior hardcoded member initializers and
  // gen_boot_config.py's MODEL_TAU_LIN_DEFAULT/MODEL_TAU_ANG_DEFAULT.
  cfg.model_tau_lin = 0.10f;       // [s]
  cfg.model_tau_ang = 0.08f;       // [s]
  return cfg;
}

void configureSimForBenchTest(TestSim::SimHarness& sim) {
  sim.configurePlanner(benchTestPlannerConfig());
  sim.configureMotor(1, benchTestMotorConfig(1));
  sim.configureMotor(2, benchTestMotorConfig(2));
}

}  // namespace TestSupport
