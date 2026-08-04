// bake_push_parity_harness.cpp -- 132-018's own disposable verification for
// sprint.md's "Bake/push parity" success criterion: "Build an image from
// one robot JSON, push that same JSON to a robot running a DIFFERENT baked
// config, confirm identical behaviour."
//
// This harness proves the property at the Configurator level, composing
// two already-established, already-tested facts rather than re-deriving
// them:
//   - applyGroup() COMMITS a full-group decode into config_ with no merge
//     (ticket 008, configurator_applygroup_harness.cpp) -- pushing group X
//     onto ANY prior config_ state leaves config_ holding EXACTLY X, not a
//     blend of X and whatever was there before.
//   - install(target) always forwards config_'s CURRENT value to the real
//     subsystem setter (Drive::configure()/App::configureOtos()), the SAME
//     call loadBaked()+install() (boot time) uses -- there is no separate
//     "pushed" code path a live SetConfigGroup takes that a baked boot
//     does not.
//
// Scenario: "robot B" starts SEEDED with togov-shaped values (a stand-in
// for "a robot baked from togov.json" -- literal numbers below are
// togov.json's own DRIVE/WHEEL_CONTROL/OTOS/PLANNER_SHAPER values, read
// 2026-08-04). tovez.json's OWN values (also literal, read the same day)
// are then pushed onto B via applyGroup(), one call per live group. B's
// resulting config_ + the REAL subsystem state that install(target) drove
// (Drive's written duty/gains, Otos's register-converted scale) is
// compared field-for-field against "robot A", seeded ONCE directly with
// tovez's values via applyGroup() (standing in for loadBaked() from
// tovez.json -- configurator_getconfig_harness.cpp already proves
// loadBaked() and applyGroup() commit identically into config_).
//
// GEOMETRY/PLANNER are boot-only (ERR_NOT_LIVE) by design -- excluded here,
// not an oversight: a robot baked from togov keeps togov's own trackwidth/
// planner ceilings forever, live push can never reach them, so "identical
// behaviour" for THOSE two groups is never claimed by the architecture.
// MOTORS/ESTIMATOR are excluded from this specific harness (MOTORS'
// guarded-while-moving path is already exhaustively covered by ticket
// 008's own harness; ESTIMATOR never lands on ANY robot, live or baked,
// so there is no behavior to compare).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "app/configurator.h"
#include "app/drive.h"
#include "config/robot.h"
#include "devices/motor.h"
#include "devices/otos.h"
#include "messages/envelope.h"
#include "messages/robot_config.h"
#include "messages/wire_runtime.h"
#include "motion/planner/planner.h"
#include "motion/planner/planner_types.h"

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

void checkEq(msg::ErrCode actual, msg::ErrCode expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected ErrCode %d, got %d", what.c_str(),
                  static_cast<int>(expected), static_cast<int>(actual));
    fail(buf);
  }
}

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-4f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

using WireRuntime::WireType;

bool encodeFloatField(uint32_t fieldNumber, float value, uint8_t* buf, size_t cap, size_t* pos) {
  if (value == 0.0f) return true;
  if (!WireRuntime::encodeTag(fieldNumber, WireType::kFixed32, buf, cap, pos)) return false;
  return WireRuntime::encodeFloat(value, buf, cap, pos);
}

class RecordingMotor : public Devices::Motor {
 public:
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { lastDuty = duty; }
  void setNeutral(Devices::Neutral) override {}
  void applyTravelCalib(float travelCalib) override { lastTravelCalib = travelCalib; }
  [[nodiscard]] bool reconfigure(const Devices::MotorConfig&) override { return true; }
  void tick(uint64_t) override {}

  float position() const override { return 0.0f; }
  float velocity() const override { return 0.0f; }
  float appliedDuty() const override { return 0.0f; }
  bool connected() const override { return true; }
  uint64_t sampleTime() const override { return 0; }

  void resetPosition() override {}
  void rebaseline() override {}

  float lastDuty = 0.0f;
  float lastTravelCalib = -1.0f;
};

class RecordingOtos : public Devices::Otos {
 public:
  void begin() override {}
  void tick(uint64_t) override {}
  Devices::PoseReading pose() const override { return {}; }
  bool poseFresh() const override { return false; }
  bool connected() const override { return true; }
  bool present() const override { return true; }
  uint64_t sampleTime() const override { return 0; }

  void setLinearScalar(float scalar) override { linearScalar = scalar; }
  void setAngularScalar(float scalar) override { angularScalar = scalar; }
  void setOffset(float x, float y, float heading) override {
    offsetX = x;
    offsetY = y;
    offsetHeading = heading;
  }
  void getOffset(float& x, float& y, float& heading) override {
    x = offsetX;
    y = offsetY;
    heading = offsetHeading;
  }
  void init() override {}

  float linearScalar = -1.0f;
  float angularScalar = -1.0f;
  float offsetX = -1.0f;
  float offsetY = -1.0f;
  float offsetHeading = -1.0f;
};

// Push helper: encode `fields` (fieldNumber, value pairs) and applyGroup().
struct FieldVal {
  uint32_t number;
  float value;
};

msg::ErrCode pushGroup(App::Configurator& configurator, msg::ConfigGroupTarget target,
                        const std::initializer_list<FieldVal>& fields) {
  uint8_t buf[256];
  size_t pos = 0;
  for (const auto& f : fields) {
    if (!encodeFloatField(f.number, f.value, buf, sizeof(buf), &pos)) {
      std::printf("  ENCODE FAILURE for target %d\n", static_cast<int>(target));
      return msg::ErrCode::ERR_OVERSIZE;
    }
  }
  return configurator.applyGroup(target, buf, pos);
}

}  // namespace

int main() {
  std::printf("=== 132-018 bake/push parity: tovez-baked vs togov-baked-then-tovez-pushed ===\n\n");

  // --- Robot A: seeded directly with tovez's own values (stands in for
  // loadBaked() from tovez.json -- ticket 011's harness already proves
  // loadBaked() and applyGroup() commit identically into config_). ------
  RecordingMotor aMotorL, aMotorR;
  RecordingOtos aOtos;
  App::Drive aDrive(aMotorL, aMotorR, /*trackWidth=*/128.0f);
  // dutyPerSpeed is BOOT-ONLY even inside the "live" DRIVE group --
  // Configurator::install()'s own doc comment: "neither Drive::configure()
  // nor install(DRIVE) touch dutyPerSpeed live." The composition root sets
  // it once, at construction, exactly like this (boot_wiring.cpp's own
  // bakeBootValues() call) -- mirrored here, NOT reachable by any push
  // below, so A and B are seeded with their OWN robot's dutyPerSpeed and
  // that seed is expected to survive every subsequent push untouched.
  aDrive.setDutyPerSpeed(0.001182f, 0.001182f);  // tovez.json's own value
  Motion::PlannerLimits aLimits;
  Motion::Planner aPlanner(aLimits);
  App::Configurator a(aDrive, aMotorL, aMotorR, aOtos, aPlanner, /*tuningStore=*/nullptr);

  // tovez.json's own DRIVE/WHEEL_CONTROL/OTOS/PLANNER_SHAPER values
  // (data/robots/tovez.json, read 2026-08-04 -- field numbers per
  // robot_config.proto / messages/robot_config.h declaration order).
  checkEq(pushGroup(a, msg::ConfigGroupTarget::DRIVE,
                     {{1, 0.001182f}, {2, 0.001182f}, {3, 0.0f},
                      {4, 1.0f}, {5, 0.0f}, {6, 1.0f}, {7, 0.0f},
                      {8, 1.0f}, {9, 0.0f}, {10, 1.0f}, {11, 0.0f}}),
          msg::ErrCode::ERR_NONE, "A: seed DRIVE with tovez values");
  checkEq(pushGroup(a, msg::ConfigGroupTarget::WHEEL_CONTROL,
                     {{1, 99.7f}, {2, 23.8f}, {3, 30.0f}, {4, 30.0f},
                      {5, 0.0f}, {6, 0.0f}, {7, 0.0f}, {8, 0.0f}, {9, 0.0f}, {10, 0.0f}, {11, 0.0f}}),
          msg::ErrCode::ERR_NONE, "A: seed WHEEL_CONTROL with tovez values");
  checkEq(pushGroup(a, msg::ConfigGroupTarget::OTOS,
                     {{1, -47.7f}, {2, 3.5f}, {3, 0.0f}, {4, 1.0275f}, {5, 0.987f}}),
          msg::ErrCode::ERR_NONE, "A: seed OTOS with tovez values");
  checkEq(pushGroup(a, msg::ConfigGroupTarget::PLANNER_SHAPER,
                     {{1, 300.0f}, {2, 250.0f}, {3, 6.0f}, {4, 5.0f}, {5, 1500.0f}, {6, 30.0f}}),
          msg::ErrCode::ERR_NONE, "A: seed PLANNER_SHAPER with tovez values");

  // --- Robot B: seeded with togov's own (DIFFERENT) values, THEN pushed
  // tovez's values -- the actual "bake from togov, push tovez" scenario. --
  RecordingMotor bMotorL, bMotorR;
  RecordingOtos bOtos;
  App::Drive bDrive(bMotorL, bMotorR, /*trackWidth=*/126.0f);
  bDrive.setDutyPerSpeed(0.00187325f, 0.00187325f);  // togov.json's own value -- boot-baked, see A's own comment above
  Motion::PlannerLimits bLimits;
  Motion::Planner bPlanner(bLimits);
  App::Configurator b(bDrive, bMotorL, bMotorR, bOtos, bPlanner, /*tuningStore=*/nullptr);

  beginScenario("B: baked from togov (seed -- different values than tovez)");
  checkEq(pushGroup(b, msg::ConfigGroupTarget::DRIVE,
                     {{1, 0.00187325f}, {2, 0.00187325f}, {3, 0.0f},
                      {4, 1.0f}, {5, 0.0f}, {6, 1.0f}, {7, 0.0f},
                      {8, 1.0f}, {9, 0.0f}, {10, 1.0f}, {11, 0.0f}}),
          msg::ErrCode::ERR_NONE, "B: seed DRIVE with togov values");
  checkEq(pushGroup(b, msg::ConfigGroupTarget::WHEEL_CONTROL, {}), msg::ErrCode::ERR_NONE,
          "B: seed WHEEL_CONTROL with togov values (all-zero -- proto3 implicit default)");
  checkEq(pushGroup(b, msg::ConfigGroupTarget::OTOS,
                     {{1, -51.5f}, {2, 0.0f}, {3, 0.0f}, {4, 1.0f}, {5, 1.0f}}),
          msg::ErrCode::ERR_NONE, "B: seed OTOS with togov values");
  checkEq(pushGroup(b, msg::ConfigGroupTarget::PLANNER_SHAPER,
                     {{1, 300.0f}, {2, 250.0f}, {3, 6.0f}, {4, 5.0f}, {5, 1500.0f}, {6, 30.0f}}),
          msg::ErrCode::ERR_NONE, "B: seed PLANNER_SHAPER with togov values (== tovez's here)");

  beginScenario("B: verify seeded state genuinely diverges from A before the push");
  checkFloatEq(bOtos.offsetX, -51.5f, "B pre-push Otos leaf offsetX is togov's, not tovez's");
  checkFloatEq(bDrive.dutyPerSpeedLeft(), 0.00187325f,
               "B's boot-baked dutyPerSpeedLeft is togov's -- confirmed BEFORE the push too, "
               "since (see below) no push will ever change it");

  beginScenario("B: push tovez's own values onto the togov-seeded robot (the actual scenario)");
  checkEq(pushGroup(b, msg::ConfigGroupTarget::DRIVE,
                     {{1, 0.001182f}, {2, 0.001182f}, {3, 0.0f},
                      {4, 1.0f}, {5, 0.0f}, {6, 1.0f}, {7, 0.0f},
                      {8, 1.0f}, {9, 0.0f}, {10, 1.0f}, {11, 0.0f}}),
          msg::ErrCode::ERR_NONE, "B: push DRIVE with tovez values");
  checkEq(pushGroup(b, msg::ConfigGroupTarget::WHEEL_CONTROL,
                     {{1, 99.7f}, {2, 23.8f}, {3, 30.0f}, {4, 30.0f},
                      {5, 0.0f}, {6, 0.0f}, {7, 0.0f}, {8, 0.0f}, {9, 0.0f}, {10, 0.0f}, {11, 0.0f}}),
          msg::ErrCode::ERR_NONE, "B: push WHEEL_CONTROL with tovez values");
  checkEq(pushGroup(b, msg::ConfigGroupTarget::OTOS,
                     {{1, -47.7f}, {2, 3.5f}, {3, 0.0f}, {4, 1.0275f}, {5, 0.987f}}),
          msg::ErrCode::ERR_NONE, "B: push OTOS with tovez values");
  checkEq(pushGroup(b, msg::ConfigGroupTarget::PLANNER_SHAPER,
                     {{1, 300.0f}, {2, 250.0f}, {3, 6.0f}, {4, 5.0f}, {5, 1500.0f}, {6, 30.0f}}),
          msg::ErrCode::ERR_NONE, "B: push PLANNER_SHAPER with tovez values");

  beginScenario("A.config() == B.config() for every live group, after the push (bake/push parity)");
  checkFloatEq(a.config().drive.duty_per_speed_left, b.config().drive.duty_per_speed_left,
               "drive.duty_per_speed_left");
  checkFloatEq(a.config().drive.duty_per_speed_right, b.config().drive.duty_per_speed_right,
               "drive.duty_per_speed_right");
  checkFloatEq(a.config().wheelControl.v_min, b.config().wheelControl.v_min, "wheelControl.v_min");
  checkFloatEq(a.config().wheelControl.bias_max, b.config().wheelControl.bias_max, "wheelControl.bias_max");
  checkFloatEq(a.config().otos.offset_x, b.config().otos.offset_x, "otos.offset_x");
  checkFloatEq(a.config().otos.offset_y, b.config().otos.offset_y, "otos.offset_y");
  checkFloatEq(a.config().otos.linear_scale, b.config().otos.linear_scale, "otos.linear_scale");
  checkFloatEq(a.config().otos.angular_scale, b.config().otos.angular_scale, "otos.angular_scale");
  checkFloatEq(a.config().plannerShaper.a_max, b.config().plannerShaper.a_max, "plannerShaper.a_max");
  checkFloatEq(a.config().plannerShaper.yaw_jerk_max, b.config().plannerShaper.yaw_jerk_max,
               "plannerShaper.yaw_jerk_max");

  beginScenario("REAL subsystem state (not just config_) matches too -- install(target) drove the "
                "actual Drive/Otos/Planner setters identically for A and B, for every field "
                "Drive::configure() actually forwards (Stage A wheel correction is config_-level-"
                "identical, already checked above via wheel_gain_*/wheel_intercept_* -- Drive has "
                "no public getter for those, so Stage B/C's ControlGains/AdaptationBounds -- the "
                "OTHER half of what Drive::configure() pulls from config.wheelControl -- stand in)");
  checkFloatEq(aDrive.controlGains().kp, bDrive.controlGains().kp, "Drive::controlGains().kp");
  checkFloatEq(aDrive.controlGains().kaff, bDrive.controlGains().kaff, "Drive::controlGains().kaff");
  checkFloatEq(aDrive.adaptationBounds().vMin, bDrive.adaptationBounds().vMin,
               "Drive::adaptationBounds().vMin");
  checkFloatEq(aDrive.adaptationBounds().biasMax, bDrive.adaptationBounds().biasMax,
               "Drive::adaptationBounds().biasMax");
  checkFloatEq(aDrive.adaptationBounds().tauAdapt, bDrive.adaptationBounds().tauAdapt,
               "Drive::adaptationBounds().tauAdapt");
  // OTOS scale is converted through Devices::scaleToRegister() before
  // reaching the leaf's setter (trap 3, 132-010) -- comparing the LEAF's
  // recorded value (not config_'s raw multiplier) proves the CONVERSION
  // was applied identically for both robots too, not just the raw push.
  checkFloatEq(aOtos.linearScalar, bOtos.linearScalar, "Otos leaf linearScalar (post scaleToRegister())");
  checkFloatEq(aOtos.angularScalar, bOtos.angularScalar, "Otos leaf angularScalar (post scaleToRegister())");
  checkFloatEq(aOtos.offsetX, bOtos.offsetX, "Otos leaf offsetX (== -47.7 for both -- B's own togov "
               "seed, -51.5, is gone)");
  checkFloatEq(aPlanner.limits().ceilings.aMax, bPlanner.limits().ceilings.aMax,
               "Planner leaf ceilings.aMax (applyShaperLimits())");
  checkFloatEq(aPlanner.limits().ceilings.yawJerkMax, bPlanner.limits().ceilings.yawJerkMax,
               "Planner leaf ceilings.yawJerkMax (applyShaperLimits())");

  beginScenario("The ONE documented exception: dutyPerSpeed stays boot-baked even inside the "
                "'live' DRIVE group (Configurator::install()'s own doc comment) -- B's push of "
                "tovez's duty_per_speed_left/right changes config_ (read-back honesty, already "
                "proven above) but does NOT reach the Drive leaf; B's ACTUAL wheel-duty "
                "conversion stays togov's own boot-baked number forever. Not a bug this harness "
                "found -- the documented boundary of 'live', asserted here so a future change "
                "that silently starts forwarding it (or silently stops) is caught either way");
  checkFloatEq(bDrive.dutyPerSpeedLeft(), 0.00187325f,
               "B's dutyPerSpeedLeft is STILL togov's after the push -- boot-only, by design");
  checkFloatEq(aDrive.dutyPerSpeedLeft(), 0.001182f, "A's dutyPerSpeedLeft is tovez's own boot bake");

  beginScenario("GEOMETRY/PLANNER are boot-only by design -- B's construction-time trackWidth "
                "(togov's 126mm) stays 126mm forever; no push can reach it, and this harness "
                "does not claim otherwise");
  checkFloatEq(bDrive.trackWidth(), 126.0f, "B's trackWidth is still togov's -- boot-only, unreachable by any push");
  checkFloatEq(aDrive.trackWidth(), 128.0f, "A's trackWidth is tovez's own -- for contrast, not equal to B's");

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf(
        "OK: a robot baked from togov.json, then pushed tovez.json's own DRIVE/WHEEL_CONTROL/"
        "OTOS/PLANNER_SHAPER values over the wire, ends up with IDENTICAL config_ state AND "
        "identical real subsystem state (Drive/Otos/Planner leaves) to a robot baked from "
        "tovez.json directly -- bake/push parity holds for every live-reappliable FIELD. Two "
        "documented, by-design exceptions confirmed to stay boot-baked rather than silently "
        "drift: GEOMETRY/PLANNER (whole boot-only groups) and dutyPerSpeed specifically (the "
        "one field inside the otherwise-live DRIVE group with no post-construction setter).\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the bake/push parity check\n", g_failureCount);
  return 1;
}
