// configurator_loadbaked_harness.cpp -- ticket 132-006's own smoke test
// (the-configuration-object.md, sprint 132 "configuration discipline"):
// Core::Configurator now owns the one Config::Robot instance --
// RobotGraph's constructor calls configurator_.loadBaked() +
// configurator_.install() in place of the old RobotGraph::Resolved +
// install*Calibration() sequence (see boot_wiring.{h,cpp}/configurator.
// {h,cpp}). This is deliberately NOT composition_root_parity_harness.cpp
// (that harness is not required to pass yet at this ticket -- byte-
// identical boot is a ticket-018 concern) -- it is the "lighter smoke
// test" the ticket's own Testing section calls for:
//
//   1. The sim composition root (TestSim::SimHarness -> Core::composeRobot())
//      constructs and boots without crashing, now that Configurator's own
//      constructor body work (loadBaked()/install()) runs as part of that
//      construction.
//   2. A basic WHEELS command still drives a nonzero commanded wheel
//      target -- proof the graph install() wired is still LIVE, not just
//      non-crashing.
//   3. Configurator::config() reflects the SAME baked values the active
//      robot JSON produces -- read directly via the SAME generated
//      Config::default*Group() functions (config/boot_config.h, 132-005)
//      loadBaked() itself calls, one representative field per group (the
//      "representative sample across all 7 groups" ticket 005's own New
//      Tests entry used for the same generated functions) -- mirroring
//      composition_root_parity_harness.cpp's own "compare against the SAME
//      generation the composition root used" pattern rather than
//      re-parsing tovez.json independently. ONE documented exception,
//      133-005: drive.wheel_gain_*/wheel_intercept_*, which this harness's
//      SimHarness deliberately overrides to identity (a real gearbox
//      linearization has nothing to linearize in a linear plant) -- see
//      that assertion's own comment below, and Core::BootOverrides::
//      wheelCorrection (app/boot_wiring.h) for the regression that made
//      the override explicit.
//   4. (132-007) RobotLoop::configure(const Config::Robot&) -- the one
//      configure() entry point boot_wiring.cpp calls directly rather than
//      through Configurator::install() (see that method's own doc comment,
//      configurator.h) -- installs rotation calibration matching
//      config().geometry, read back via the new rotationGainPos()/
//      rotationOffsetPos()/rotationGainNeg()/rotationOffsetNeg() getters
//      (robot_loop.h). The other five 132-007 configure() entry points
//      (Drive::configure(), Core::configurePlanner()/configureMotor()/
//      configureOtos()) are covered by
//      src/tests/sim/unit/configure_entry_points_harness.cpp instead,
//      which needs no full composition root.
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/system harness's own shape.
// Run by test_configurator_loadbaked.py.
#include <cmath>
#include <cstdio>
#include <string>

#include "bench_test_config.h"
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
  std::printf(
      "=== Configurator::loadBaked()/install() smoke test (132-006) ===\n\n");

  beginScenario("The sim composition root constructs and boots without crashing");
  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  checkTrue(sim.booted(), "sim.booted() is true after boot()");

  beginScenario("A basic WHEELS command still drives real duty onto both wheels");
  constexpr float kSpeed = 100.0f;    // [mm/s]
  constexpr float kDuration = 500.0f;  // [ms]
  sim.injectWheels(kSpeed, kSpeed, kDuration, /*id=*/1, /*corrId=*/1);
  sim.step(5);
  // driveTargetVelLeft()/Right() are GONE with RobotState::Wheel::cmdVelocity
  // -- the actuation boundary moved inside Control::DifferentialDrive, whose
  // Command mailbox is private by design. The kernel's published Output is
  // the observable now, and applied duty is a STRONGER assertion than the old
  // staged target: it says the command survived the whole pipeline (lease,
  // gates, Stage A, PID, shaping) and reached the wire, not merely that a
  // blackboard field was written.
  const Control::DifferentialDrive::Output out = sim.drive().output();
  checkTrue(out.appliedDutyLeft > 0.0f,
            "left wheel has nonzero applied duty after a WHEELS command");
  checkTrue(out.appliedDutyRight > 0.0f,
            "right wheel has nonzero applied duty after a WHEELS command");

  beginScenario(
      "Configurator::config() reflects the same baked values as the "
      "active robot JSON's generated defaults, across all 8 groups");
  const Config::Robot& config = sim.configurator().config();

  const msg::Geometry hwGeometry = Config::defaultGeometryGroup();
  checkFloatEq(config.geometry.trackwidth, hwGeometry.trackwidth, "geometry.trackwidth");
  checkFloatEq(config.geometry.rotational_slip, hwGeometry.rotational_slip,
               "geometry.rotational_slip");

  const msg::Motors hwMotors = Config::defaultMotorsGroup();
  checkFloatEq(config.motors.travel_calib_left, hwMotors.travel_calib_left,
               "motors.travel_calib_left");
  checkFloatEq(config.motors.output_deadband, hwMotors.output_deadband,
               "motors.output_deadband");

  const msg::Drive hwDrive = Config::defaultDriveGroup();
  checkFloatEq(config.drive.duty_per_speed_left, hwDrive.duty_per_speed_left,
               "drive.duty_per_speed_left");
  // 133-005: drive.wheel_gain_*/wheel_intercept_* are the ONE documented
  // exception to this scenario's bake-parity rule, and deliberately so.
  // This harness composes a TestSim::SimHarness, which declares an
  // identity wheel correction through Core::BootOverrides::wheelCorrection
  // -- a real gearbox linearization has nothing to linearize in a linear
  // plant, so the sim must NOT inherit the file's fitted gains. Asserting
  // bake parity on them here would be asserting the very defect that cost
  // sprint 132 a doubled turn error (see BootOverrides::wheelCorrection's
  // own doc comment, app/boot_wiring.h).
  //
  // The check is not dropped, only MOVED to where it is true: the
  // hardware path's "no override == the file wins" parity, and the sim
  // path's identity invariant, are both asserted by
  // src/tests/sim/unit/sim_harness_configure_harness.cpp's own scenario 4.
  // crawl_pulse below keeps a second, unexempted Drive field in this
  // scenario so the group is still covered here.
  checkFloatEq(config.drive.crawl_pulse, hwDrive.crawl_pulse, "drive.crawl_pulse");
  checkFloatEq(config.drive.wheel_gain_left_accel, 1.0f,
               "drive.wheel_gain_left_accel is the sim's identity override, NOT the bake");

  const msg::WheelControl hwWheelControl = Config::defaultWheelControlGroup();
  checkFloatEq(config.wheelControl.pid_kp, hwWheelControl.pid_kp, "wheelControl.pid_kp");

  const msg::Planner hwPlanner = Config::defaultPlannerGroup();
  checkFloatEq(config.planner.v_max, hwPlanner.v_max, "planner.v_max");
  checkFloatEq(config.planner.heading_hold_gain, hwPlanner.heading_hold_gain,
               "planner.heading_hold_gain");

  // 132-017 split: the shaper ceilings live on plannerShaper now, not
  // planner (the boot-only remainder).
  const msg::PlannerShaper hwPlannerShaper = Config::defaultPlannerShaperGroup();
  checkFloatEq(config.plannerShaper.a_max, hwPlannerShaper.a_max, "plannerShaper.a_max");

  const msg::Otos hwOtos = Config::defaultOtosGroup();
  checkFloatEq(config.otos.linear_scale, hwOtos.linear_scale, "otos.linear_scale");

  const msg::Estimator hwEstimator = Config::defaultEstimatorGroup();
  checkFloatEq(config.estimator.weight_heading_otos, hwEstimator.weight_heading_otos,
               "estimator.weight_heading_otos");

  beginScenario(
      "132-007: RobotLoop::configure(const Config::Robot&) -- called directly from "
      "boot_wiring.cpp's constructor, right after loadBaked() -- installs rotation "
      "calibration matching config().geometry, converted through the SAME derived "
      "methods (rotationOffsetPos()/rotationOffsetNeg(), config/robot.h) that replaced "
      "the deleted installRotationCalibration() free function");
  constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
  checkFloatEq(sim.robotLoop().rotationGainPos(), config.geometry.rotation_gain_pos,
               "robotLoop().rotationGainPos()");
  checkFloatEq(sim.robotLoop().rotationOffsetPos(), config.geometry.rotation_offset * kDegToRad,
               "robotLoop().rotationOffsetPos()");
  checkFloatEq(sim.robotLoop().rotationGainNeg(), config.geometry.rotation_gain_neg,
               "robotLoop().rotationGainNeg()");
  checkFloatEq(sim.robotLoop().rotationOffsetNeg(), config.geometry.rotation_offset_neg * kDegToRad,
               "robotLoop().rotationOffsetNeg()");

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf(
        "OK: sim composition root constructs/boots/drives with "
        "Configurator::loadBaked()+install(), config() reflects the "
        "baked robot config across all 7 groups, and RobotLoop::configure() "
        "(132-007) installs rotation calibration matching it\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Configurator::loadBaked() smoke test\n",
              g_failureCount);
  return 1;
}
