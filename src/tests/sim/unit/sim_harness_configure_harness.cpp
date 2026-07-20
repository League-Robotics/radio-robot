// sim_harness_configure_harness.cpp -- ticket 113-002's own acceptance
// proof: TestSim::SimHarness::configurePlanner()/configureMotor() are a
// purely ADDITIVE config-load surface (SUC-001/SUC-002/SUC-005).
//
// Every scenario below constructs its OWN fresh TestSim::SimHarness -- this
// file never touches any of the ~40 pre-existing sim harness files, and
// never asserts against them either (that regression -- "no existing
// src/tests/sim/ test file's assertions change" -- is covered by the full
// `uv run python -m pytest` run this ticket's own Testing section calls
// for, not by anything here). What THIS file proves, specifically:
//
//   1. A default-constructed SimHarness (configurePlanner()/configureMotor()
//      NEVER called) observes literally makeExecutorConfig()'s own baseline
//      values -- the regression pin for "the additive surface starts from
//      an unmodified default."
//   2. configurePlanner() with values that differ from that baseline takes
//      effect and is readable back via plannerConfig() (Pilot::
//      plannerConfig()'s full-struct copy -- see sim_harness.h's own
//      comment on why that accessor is a reliable readback for every
//      PlannerConfig field, not just the ones Pilot's own arithmetic
//      reads).
//   3. configureMotor() with values that differ from Devices::MotorConfig{}'s
//      zero defaults takes effect per PORT and is readable back via
//      motorConfig(port) -- and configuring one port never touches the
//      other port's own last-configured record.
//   4. The regression this ticket's own acceptance criteria call out by
//      name: a caller that calls configurePlanner() with a full config and
//      THEN calls setYawRateMax() (one of the three pre-existing sim-only
//      hooks) does NOT silently lose every other field configurePlanner()
//      set back to makeExecutorConfig()'s stand-in defaults -- only
//      yaw_rate_max (and setYawRateMax()'s own remembered
//      lead-compensation/distance_kp overrides) actually change.
//
// Compiled by test_sim_harness_configure.py against the same full
// HOST_BUILD dependency graph test_pilot_distance_trim.py/
// test_heading_source.py already compile (SimHarness composes the real
// App::RobotLoop graph -- see sim_harness.h's own header).
#include <cmath>
#include <cstdio>
#include <string>

#include "messages/planner.h"
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

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-4f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// makeExecutorConfig()'s own literal baseline values (sim_harness.h) --
// duplicated here deliberately (a test should never depend on an ambient
// default silently tracking a production one -- same posture
// pilot_distance_trim_harness.cpp's own kDistanceKp comment takes) so
// scenario 1 below is a genuine regression pin, not a tautology.
constexpr float kBaselineAMax = 800.0f;          // [mm/s^2]
constexpr float kBaselineHeadingKp = 2.5f;       // [1/s]
constexpr float kBaselineModelTauLin = 0.10f;    // [s]
constexpr float kBaselineModelTauAng = 0.08f;    // [s]
constexpr float kBaselineYawRateMax = 4.0f;      // [rad/s]

}  // namespace

int main() {
  std::printf("=== TestSim::SimHarness::configurePlanner()/configureMotor() (113-002) ===\n\n");

  // --- Scenario 1: default construction, never configured -- observes
  //     makeExecutorConfig()'s own baseline (additive-surface regression
  //     pin) ---
  {
    beginScenario("default-constructed SimHarness observes makeExecutorConfig()'s own baseline");
    TestSim::SimHarness sim;
    sim.boot();

    checkFloatEq(sim.plannerConfig().a_max, kBaselineAMax,
                 "a_max matches makeExecutorConfig()'s baseline before any configurePlanner() call");
    checkFloatEq(sim.plannerConfig().heading_kp, kBaselineHeadingKp,
                 "heading_kp matches makeExecutorConfig()'s baseline before any configurePlanner() call");
    checkFloatEq(sim.plannerConfig().model_tau_lin, kBaselineModelTauLin,
                 "model_tau_lin matches makeExecutorConfig()'s baseline before any configurePlanner() call");
    checkFloatEq(sim.plannerConfig().model_tau_ang, kBaselineModelTauAng,
                 "model_tau_ang matches makeExecutorConfig()'s baseline before any configurePlanner() call");
  }

  // --- Scenario 2: configurePlanner() with values differing from the
  //     baseline takes effect, readable via plannerConfig() ---
  {
    beginScenario("configurePlanner() with non-baseline values takes effect, readable via plannerConfig()");
    TestSim::SimHarness sim;
    sim.boot();

    msg::PlannerConfig cfg;
    cfg.a_max = 1234.0f;                                            // [mm/s^2]
    cfg.a_decel = 999.0f;                                           // [mm/s^2]
    cfg.v_body_max = 777.0f;                                        // [mm/s]
    cfg.yaw_rate_max = 6.5f;                                        // [rad/s]
    cfg.yaw_acc_max = 33.0f;                                        // [rad/s^2]
    cfg.j_max = 5000.0f;                                            // [mm/s^3]
    cfg.yaw_jerk_max = 40.0f;                                       // [rad/s^3]
    cfg.min_speed = 25.0f;                                          // [mm/s]
    cfg.heading_kp = 7.25f;                                         // [1/s]
    cfg.heading_kd = 0.15f;                                         // dimensionless
    cfg.arrive_dwell = 0.30f;                                       // [s]
    cfg.heading_source = msg::HeadingSourceMode::HEADING_SOURCE_FORCE_ENCODER;
    cfg.heading_dwell_tol = 0.05f;                                  // [rad]
    cfg.heading_dwell_rate = 0.02f;                                 // [rad/s]
    cfg.heading_lead_bias = -0.11f;                                 // [s]
    cfg.plan_lead = 0.33f;                                          // [s]
    cfg.terminal_lead = 0.07f;                                      // [s]
    cfg.actuation_lag = 0.125f;                                     // [s]
    cfg.distance_kp = 9.5f;                                         // [1/s]
    cfg.distance_tol = 4.0f;                                        // [mm]
    cfg.model_tau_lin = 0.33f;                                      // [s]
    cfg.model_tau_ang = 0.44f;                                      // [s]

    sim.configurePlanner(cfg);

    const msg::PlannerConfig& got = sim.plannerConfig();
    checkFloatEq(got.a_max, 1234.0f, "a_max took effect");
    checkFloatEq(got.a_decel, 999.0f, "a_decel took effect");
    checkFloatEq(got.v_body_max, 777.0f, "v_body_max took effect");
    checkFloatEq(got.yaw_rate_max, 6.5f, "yaw_rate_max took effect");
    checkFloatEq(got.yaw_acc_max, 33.0f, "yaw_acc_max took effect");
    checkFloatEq(got.j_max, 5000.0f, "j_max took effect");
    checkFloatEq(got.yaw_jerk_max, 40.0f, "yaw_jerk_max took effect");
    checkFloatEq(got.min_speed, 25.0f, "min_speed took effect");
    checkFloatEq(got.heading_kp, 7.25f, "heading_kp took effect");
    checkFloatEq(got.heading_kd, 0.15f, "heading_kd took effect");
    checkFloatEq(got.arrive_dwell, 0.30f, "arrive_dwell took effect");
    checkTrue(got.heading_source == msg::HeadingSourceMode::HEADING_SOURCE_FORCE_ENCODER,
              "heading_source took effect");
    checkFloatEq(got.heading_dwell_tol, 0.05f, "heading_dwell_tol took effect");
    checkFloatEq(got.heading_dwell_rate, 0.02f, "heading_dwell_rate took effect");
    checkFloatEq(got.heading_lead_bias, -0.11f, "heading_lead_bias took effect");
    checkFloatEq(got.plan_lead, 0.33f, "plan_lead took effect");
    checkFloatEq(got.terminal_lead, 0.07f, "terminal_lead took effect");
    checkFloatEq(got.actuation_lag, 0.125f, "actuation_lag took effect");
    checkFloatEq(got.distance_kp, 9.5f, "distance_kp took effect");
    checkFloatEq(got.distance_tol, 4.0f, "distance_tol took effect");
    checkFloatEq(got.model_tau_lin, 0.33f, "model_tau_lin took effect (113-001 field)");
    checkFloatEq(got.model_tau_ang, 0.44f, "model_tau_ang took effect (113-001 field)");
  }

  // --- Scenario 3: configureMotor() with non-default values takes effect
  //     per port, readable via motorConfig(port); configuring one port
  //     never touches the other port's own record ---
  {
    beginScenario("configureMotor() takes effect per port, readable via motorConfig(port)");
    TestSim::SimHarness sim;
    sim.boot();

    checkFloatEq(sim.motorConfig(1).velFiltAlpha, 0.0f,
                 "left motorConfig starts at Devices::MotorConfig{}'s zero default");
    checkFloatEq(sim.motorConfig(2).velFiltAlpha, 0.0f,
                 "right motorConfig starts at Devices::MotorConfig{}'s zero default");

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

  // --- Scenario 4: the regression this ticket's own acceptance criteria
  //     name explicitly -- configurePlanner() THEN setYawRateMax() must not
  //     lose every other field configurePlanner() set. ---
  {
    beginScenario("configurePlanner() then setYawRateMax(): other fields survive (no silent reset)");
    TestSim::SimHarness sim;
    sim.boot();

    msg::PlannerConfig cfg;
    cfg.a_max = 4242.0f;             // [mm/s^2] -- distinctive, NOT makeExecutorConfig()'s 800
    cfg.heading_kp = 11.0f;          // [1/s] -- distinctive, NOT makeExecutorConfig()'s 2.5
    cfg.model_tau_lin = 0.55f;       // [s] -- distinctive, NOT makeExecutorConfig()'s 0.10
    cfg.model_tau_ang = 0.66f;       // [s] -- distinctive, NOT makeExecutorConfig()'s 0.08
    cfg.yaw_rate_max = kBaselineYawRateMax;  // unchanged from baseline for this scenario
    sim.configurePlanner(cfg);

    checkFloatEq(sim.plannerConfig().a_max, 4242.0f, "setup: a_max holds configurePlanner()'s value before setYawRateMax()");

    sim.setYawRateMax(1.5f);  // only yaw_rate_max is meant to change

    checkFloatEq(sim.plannerConfig().yaw_rate_max, 1.5f, "setYawRateMax() itself took effect");
    checkFloatEq(sim.plannerConfig().a_max, 4242.0f,
                 "a_max SURVIVES setYawRateMax() -- not silently reset to makeExecutorConfig()'s 800");
    checkFloatEq(sim.plannerConfig().heading_kp, 11.0f,
                 "heading_kp SURVIVES setYawRateMax() -- not silently reset to makeExecutorConfig()'s 2.5");
    checkFloatEq(sim.plannerConfig().model_tau_lin, 0.55f,
                 "model_tau_lin SURVIVES setYawRateMax() -- not silently reset to makeExecutorConfig()'s 0.10");
    checkFloatEq(sim.plannerConfig().model_tau_ang, 0.66f,
                 "model_tau_ang SURVIVES setYawRateMax() -- not silently reset to makeExecutorConfig()'s 0.08");
  }

  // --- Scenario 5: setYawRateMax() alone (configurePlanner() NEVER called)
  //     keeps its PRE-EXISTING behavior -- rebuilds from
  //     makeExecutorConfig(), exactly as before this ticket. ---
  {
    beginScenario("setYawRateMax() alone (no configurePlanner() call): unchanged pre-existing behavior");
    TestSim::SimHarness sim;
    sim.boot();

    sim.setYawRateMax(7.0f);

    checkFloatEq(sim.plannerConfig().yaw_rate_max, 7.0f, "setYawRateMax() itself took effect");
    checkFloatEq(sim.plannerConfig().a_max, kBaselineAMax,
                 "a_max still makeExecutorConfig()'s baseline -- setYawRateMax() rebuilt from it, "
                 "not from any configurePlanner() call (there was none)");
    checkFloatEq(sim.plannerConfig().heading_kp, kBaselineHeadingKp,
                 "heading_kp still makeExecutorConfig()'s baseline for the same reason");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all SimHarness::configurePlanner()/configureMotor() scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the SimHarness configure() scenarios\n", g_failureCount);
  return 1;
}
