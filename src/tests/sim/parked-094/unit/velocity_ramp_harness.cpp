// PARKED (sprint 094, ticket 094-002): Motion::VelocityRamp was DELETED
// outright (src/firm/motion/velocity_ramp.{h,cpp} no longer exist -- see
// src/tests/sim/parked-094/README.md). Kept as a historical record only; a
// revival must port this coverage onto Motion::JerkTrajectory, not
// resurrect VelocityRamp -- clasi/issues/restore-goto-pursuit-with-pose-
// estimator.md.
//
// velocity_ramp_harness.cpp -- off-hardware acceptance harness for ticket
// 084-001 (SUC-001/SUC-002/SUC-003): exercises Motion::VelocityRamp
// (src/firm/motion/velocity_ramp.{h,cpp}) in isolation against plain
// msg::PlannerConfig fixtures -- no fakes needed, pure profiler math.
//
// Mirrors drivetrain_harness.cpp's shape exactly: #includes only
// motion/velocity_ramp.h + messages/planner.h (both dependency-free -- no
// MicroBit.h, no I2CBus), links against motion/velocity_ramp.cpp, compiles
// with the plain system C++ compiler -- no CMake, no ARM toolchain.
// Hand-rolled assertions, prints PASS/FAIL, exits nonzero on any failure.
// Run by test_velocity_ramp.py, which compiles and runs this binary via
// subprocess.

#include <cmath>
#include <cstdio>
#include <string>

#include "messages/planner.h"
#include "motion/velocity_ramp.h"

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

void checkFloatNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g (+/- %g), got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(tol),
                  static_cast<double>(actual));
    fail(buf);
  }
}

msg::PlannerConfig trapezoidConfig() {
  msg::PlannerConfig cfg;
  cfg.a_max = 500.0f;       // [mm/s^2]
  cfg.a_decel = 500.0f;     // [mm/s^2]
  cfg.v_body_max = 300.0f;  // [mm/s]
  cfg.yaw_rate_max = 4.0f;  // [rad/s]
  cfg.yaw_acc_max = 8.0f;   // [rad/s^2]
  cfg.j_max = 0.0f;         // trapezoid (no jerk limit)
  cfg.yaw_jerk_max = 0.0f;  // trapezoid (no jerk limit)
  return cfg;
}

// 1. A fresh ramp targeting a nonzero (v, omega) is not at target, and
// converges to it within a bounded number of ticks under the trapezoid
// profile (j_max == 0).
void scenarioTrapezoidConverges() {
  beginScenario("trapezoid: converges to target under accel limits");
  Motion::VelocityRamp ramp;
  ramp.configure(trapezoidConfig());
  ramp.setTarget(200.0f, 2.0f);

  checkFalse(ramp.atTarget(), "not at target immediately after setTarget()");

  bool stillRamping = true;
  int ticks = 0;
  const int kMaxTicks = 200;  // 200 * 0.05s = 10s -- generous upper bound
  while (stillRamping && ticks < kMaxTicks) {
    stillRamping = ramp.advance(0.05f);
    ++ticks;
  }

  checkTrue(ticks < kMaxTicks, "converged before the generous tick budget ran out");
  checkTrue(ramp.atTarget(), "atTarget() true once advance() stops reporting still-ramping");
  checkFloatNear(ramp.currentV(), 200.0f, 0.5f, "currentV() converged to the commanded speed");
  checkFloatNear(ramp.currentOmega(), 2.0f, 0.001f, "currentOmega() converged to the commanded rate");
}

// 2. Convergence is gradual, not instantaneous: after one small tick the
// ramp has moved TOWARD the target but has not leapt straight to it (proves
// the accel limit is actually being applied, not just clamped-and-set).
void scenarioTrapezoidRampsGradually() {
  beginScenario("trapezoid: one tick moves partway, not all the way, to target");
  Motion::VelocityRamp ramp;
  ramp.configure(trapezoidConfig());
  ramp.setTarget(200.0f, 0.0f);

  ramp.advance(0.05f);  // dv_max = a_max * dt = 500 * 0.05 = 25 mm/s

  checkFloatNear(ramp.currentV(), 25.0f, 1e-3f, "one tick advances by exactly a_max*dt");
  checkFalse(ramp.atTarget(), "still short of the 200 mm/s target after one small tick");
}

// 3. atTarget() clamps the reported target against v_body_max/yaw_rate_max
// before comparing -- a target beyond the configured ceiling converges to
// the CLAMPED value, not the raw commanded one.
void scenarioClampedTargetConverges() {
  beginScenario("target beyond v_body_max/yaw_rate_max converges to the clamped value");
  Motion::VelocityRamp ramp;
  ramp.configure(trapezoidConfig());  // v_body_max = 300, yaw_rate_max = 4.0
  ramp.setTarget(500.0f, 10.0f);      // both beyond the configured ceiling

  bool stillRamping = true;
  int ticks = 0;
  while (stillRamping && ticks < 200) {
    stillRamping = ramp.advance(0.05f);
    ++ticks;
  }

  checkFloatNear(ramp.currentV(), 300.0f, 0.5f, "currentV() converges to v_body_max, not 500");
  checkFloatNear(ramp.currentOmega(), 4.0f, 0.001f, "currentOmega() converges to yaw_rate_max, not 10");
}

// 4. S-curve path (j_max > 0): the live acceleration itself ramps up under
// the jerk bound, so early ticks advance LESS than the pure-trapezoid case
// would -- proving the jerk-limited branch actually engages, not just the
// trapezoid path regardless of j_max.
void scenarioSCurveRampsMoreGraduallyThanTrapezoid() {
  beginScenario("S-curve (j_max > 0): early ticks advance less than the trapezoid case");
  msg::PlannerConfig cfg = trapezoidConfig();
  cfg.j_max = 2000.0f;  // [mm/s^3] finite jerk limit

  Motion::VelocityRamp ramp;
  ramp.configure(cfg);
  ramp.setTarget(200.0f, 0.0f);
  ramp.advance(0.05f);

  // Trapezoid (no jerk limit) would reach 25 mm/s in one 0.05s tick (see
  // scenario 2). The S-curve path must advance LESS in that same tick,
  // since a_live itself starts at 0 and only approaches a_max under the
  // jerk bound (jerkStep = j_max * dt = 2000 * 0.05 = 100 mm/s^2 this tick).
  checkTrue(ramp.currentV() < 25.0f,
            "S-curve's first-tick speed is strictly less than the trapezoid's");
  checkTrue(ramp.currentV() > 0.0f, "S-curve still makes forward progress on the first tick");

  bool stillRamping = true;
  int ticks = 1;
  while (stillRamping && ticks < 200) {
    stillRamping = ramp.advance(0.05f);
    ++ticks;
  }
  checkTrue(ramp.atTarget(), "S-curve still converges to target given enough ticks");
  checkFloatNear(ramp.currentV(), 200.0f, 0.5f, "S-curve converges to the commanded speed");
}

// 5. reset() zeroes everything (current AND target), so atTarget() is
// trivially true and currentV()/currentOmega() report 0.
void scenarioResetZeroesEverything() {
  beginScenario("reset() zeroes current and target state");
  Motion::VelocityRamp ramp;
  ramp.configure(trapezoidConfig());
  ramp.setTarget(200.0f, 2.0f);
  ramp.advance(0.05f);
  checkTrue(ramp.currentV() > 0.0f, "precondition: ramp made some progress");

  ramp.reset();

  checkFloatNear(ramp.currentV(), 0.0f, 1e-6f, "reset() zeroes currentV()");
  checkFloatNear(ramp.currentOmega(), 0.0f, 1e-6f, "reset() zeroes currentOmega()");
  checkTrue(ramp.atTarget(), "reset() also zeroes the target, so atTarget() is trivially true");
}

// 6. seedCurrent() sets live state directly (no ramp step) -- the very next
// advance() call ramps from the SEEDED value, not from zero.
void scenarioSeedCurrentSkipsRampFromZero() {
  beginScenario("seedCurrent() sets live state with no ramp step");
  Motion::VelocityRamp ramp;
  ramp.configure(trapezoidConfig());
  ramp.seedCurrent(150.0f, 1.0f);

  checkFloatNear(ramp.currentV(), 150.0f, 1e-6f, "seedCurrent() takes effect immediately");
  checkFloatNear(ramp.currentOmega(), 1.0f, 1e-6f, "seedCurrent() takes effect immediately");

  // Target still at (0,0) (never set) -- one tick approaches it from the
  // seeded value, proving advance() ramps FROM the seed, not from zero.
  ramp.setTarget(0.0f, 0.0f);
  ramp.advance(0.05f);  // dv_max = a_decel * dt = 500 * 0.05 = 25 mm/s
  checkFloatNear(ramp.currentV(), 125.0f, 1e-3f, "ramps down from the seeded 150, not from 0");
}

// 7. advance() with dt <= 0 is a no-op (returns !atTarget(), state
// unchanged) -- matches the ported source's documented contract.
void scenarioNonPositiveDtIsNoOp() {
  beginScenario("advance(dt <= 0) is a no-op");
  Motion::VelocityRamp ramp;
  ramp.configure(trapezoidConfig());
  ramp.setTarget(200.0f, 0.0f);

  bool result = ramp.advance(0.0f);
  checkTrue(result, "advance(0) returns !atTarget() (still short of target)");
  checkFloatNear(ramp.currentV(), 0.0f, 1e-6f, "advance(0) does not change currentV()");

  result = ramp.advance(-1.0f);
  checkTrue(result, "advance(negative) also returns !atTarget()");
  checkFloatNear(ramp.currentV(), 0.0f, 1e-6f, "advance(negative) does not change currentV()");
}

}  // namespace

int main() {
  scenarioTrapezoidConverges();
  scenarioTrapezoidRampsGradually();
  scenarioClampedTargetConverges();
  scenarioSCurveRampsMoreGraduallyThanTrapezoid();
  scenarioResetZeroesEverything();
  scenarioSeedCurrentSkipsRampFromZero();
  scenarioNonPositiveDtIsNoOp();

  if (g_failureCount == 0) {
    std::printf("OK: all VelocityRamp scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the VelocityRamp scenarios\n", g_failureCount);
  return 1;
}
