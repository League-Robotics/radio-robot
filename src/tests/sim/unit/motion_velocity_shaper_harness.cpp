// motion_velocity_shaper_harness.cpp -- off-hardware acceptance proof for
// Motion::VelocityShaper (src/firm/motion/velocity_shaper.{h,cpp}),
// decel-into-the-goal campaign (follow-on to
// clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md).
//
// Compiles ONLY against velocity_shaper.cpp -- no App::/Devices:: fakes of
// any kind, mirroring motion_stop_condition_harness.cpp's own "the module
// takes every reading as a plain parameter" compile-boundary proof (see
// velocity_shaper.h's own file header: zero dependency on App::MoveQueue,
// Motion::StopCondition, or any msg::* wire type).
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "motion/velocity_shaper.h"

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

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-3f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

void checkLe(float actual, float bound, const std::string& what) {
  if (actual > bound) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected <= %g, got %g", what.c_str(),
                  static_cast<double>(bound), static_cast<double>(actual));
    fail(buf);
  }
}

// ===========================================================================
// 1. Accel clamp: from a standstill, one tick's step is exactly aMax*dt --
//    never a jump straight to cruise.
// ===========================================================================

void scenarioAccelClampFromStandstill() {
  beginScenario("VelocityShaper::next(): accel clamp -- one tick from 0 steps by exactly aMax*dt");

  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/300.0f, /*currentSpeed=*/0.0f,
                                             /*remaining=*/10000.0f, /*dt=*/0.02f,
                                             /*aMax=*/500.0f, /*aDecel=*/800.0f);
  checkFloatEq(next, 10.0f, "next == aMax*dt (500*0.02) -- remaining/cruise both far from binding");
}

// ===========================================================================
// 2. Accel clamp never overshoots cruise -- a huge aMax*dt still lands
//    exactly ON cruiseSpeed, not past it.
// ===========================================================================

void scenarioAccelClampNeverOvershootsCruise() {
  beginScenario("VelocityShaper::next(): accel clamp never overshoots cruiseSpeed even with a huge aMax*dt");

  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/100.0f, /*currentSpeed=*/90.0f,
                                             /*remaining=*/10000.0f, /*dt=*/1.0f,
                                             /*aMax=*/100000.0f, /*aDecel=*/800.0f);
  checkFloatEq(next, 100.0f, "next lands exactly on cruiseSpeed, never past it");
}

// ===========================================================================
// 3. Once at cruise with remaining still large, next tick holds cruise
//    (steady state).
// ===========================================================================

void scenarioHoldsCruiseAtSteadyState() {
  beginScenario("VelocityShaper::next(): holds cruiseSpeed once reached, remaining still large");

  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/300.0f, /*currentSpeed=*/300.0f,
                                             /*remaining=*/10000.0f, /*dt=*/0.02f,
                                             /*aMax=*/500.0f, /*aDecel=*/800.0f);
  checkFloatEq(next, 300.0f, "next holds cruiseSpeed -- no drift once at steady state");
}

// ===========================================================================
// 4. Decel taper curve: at cruise, with remaining small enough that
//    sqrt(2*aDecel*remaining) < cruise, next is CAPPED to that sqrt value,
//    not cruise -- the "decelerate into the goal" curve itself.
// ===========================================================================

void scenarioDecelTaperCapsBelowCruiseNearGoal() {
  beginScenario("VelocityShaper::next(): decel taper caps next below cruiseSpeed as remaining shrinks");

  // sqrt(2*800*10) = sqrt(16000) ~= 126.49 -- well under the 300 cruise.
  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/300.0f, /*currentSpeed=*/300.0f,
                                             /*remaining=*/10.0f, /*dt=*/0.02f,
                                             /*aMax=*/500.0f, /*aDecel=*/800.0f);
  checkFloatEq(next, std::sqrt(2.0f * 800.0f * 10.0f), "next == sqrt(2*aDecel*remaining), the decel-taper ceiling");
  checkLe(next, 300.0f, "sanity: taper ceiling is below cruiseSpeed near the goal");
}

// ===========================================================================
// 5. The taper curve monotonically shrinks as remaining shrinks -- the
//    stakeholder's own "speed drops as you approach the target" property,
//    sampled at several remaining values.
// ===========================================================================

void scenarioTaperMonotonicallyDecreasesWithRemaining() {
  beginScenario("VelocityShaper::next(): taper ceiling shrinks monotonically as remaining shrinks");

  float remainders[] = {200.0f, 100.0f, 50.0f, 10.0f, 1.0f};
  float previous = 1.0e9f;
  for (float remaining : remainders) {
    float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/300.0f, /*currentSpeed=*/300.0f,
                                               remaining, /*dt=*/0.02f,
                                               /*aMax=*/500.0f, /*aDecel=*/800.0f);
    if (next > previous + 1e-3f) {
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "next (%g) at remaining=%g exceeds the previous (larger-remaining) next (%g)",
                    static_cast<double>(next), static_cast<double>(remaining),
                    static_cast<double>(previous));
      fail(buf);
    }
    previous = next;
  }
}

// ===========================================================================
// 6. Zero/negative remaining: the taper ceiling is exactly 0 -- the goal
//    itself commands a full stop, regardless of current/cruise.
// ===========================================================================

void scenarioZeroRemainingCapsToZero() {
  beginScenario("VelocityShaper::next(): remaining == 0 caps next to exactly 0");

  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/300.0f, /*currentSpeed=*/300.0f,
                                             /*remaining=*/0.0f, /*dt=*/0.02f,
                                             /*aMax=*/500.0f, /*aDecel=*/800.0f);
  checkFloatEq(next, 0.0f, "remaining == 0 -- decel-taper ceiling is 0");
}

void scenarioNegativeRemainingClampsToZeroSameAsZero() {
  beginScenario("VelocityShaper::next(): negative remaining clamps to 0, same result as remaining == 0");

  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/300.0f, /*currentSpeed=*/300.0f,
                                             /*remaining=*/-5.0f, /*dt=*/0.02f,
                                             /*aMax=*/500.0f, /*aDecel=*/800.0f);
  checkFloatEq(next, 0.0f, "negative remaining clamps to 0 -- same malformed-input posture as stop_condition.cpp");
}

// ===========================================================================
// 7. Sign handling: a negative cruiseSpeed (e.g. a clockwise turn) tapers
//    toward 0 the SAME way a positive one does, mirrored in sign.
// ===========================================================================

void scenarioNegativeCruiseTapersSymmetrically() {
  beginScenario("VelocityShaper::next(): negative cruiseSpeed (e.g. CW turn) tapers symmetrically");

  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/-2.0f, /*currentSpeed=*/-2.0f,
                                             /*remaining=*/0.1f /*[rad]*/, /*dt=*/0.02f,
                                             /*aMax=*/8.0f, /*aDecel=*/8.0f);
  float expectedMag = std::sqrt(2.0f * 8.0f * 0.1f);
  checkFloatEq(next, -expectedMag, "negative cruise -- next carries cruiseSpeed's own sign");
}

void scenarioNegativeCruiseRampsUpFromZeroNegatively() {
  beginScenario("VelocityShaper::next(): negative cruiseSpeed ramps up (more negative) from 0, accel-clamped");

  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/-2.0f, /*currentSpeed=*/0.0f,
                                             /*remaining=*/1000.0f, /*dt=*/0.02f,
                                             /*aMax=*/8.0f, /*aDecel=*/8.0f);
  checkFloatEq(next, -0.16f, "next == -aMax*dt (8*0.02) -- accel clamp respects cruise's negative sign");
}

// ===========================================================================
// 8. current beyond cruise (e.g. a live config lowered the ceiling
//    mid-Move) ramps DOWN toward the new cruise, at the accel-magnitude
//    rate -- never an instantaneous drop.
// ===========================================================================

void scenarioCurrentAboveCruiseRampsDownGracefully() {
  beginScenario("VelocityShaper::next(): current above cruiseSpeed ramps down at the accel rate, not instantly");

  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/50.0f, /*currentSpeed=*/300.0f,
                                             /*remaining=*/10000.0f, /*dt=*/0.02f,
                                             /*aMax=*/500.0f, /*aDecel=*/800.0f);
  checkFloatEq(next, 290.0f, "next == current - aMax*dt (300-10) -- ramps down toward the lower cruise");
}

// ===========================================================================
// 9. remaining == +infinity (Kind::Time -- no decel taper, MoveQueue's own
//    documented "pass +infinity" contract): the decel clamp never binds,
//    only the accel ramp/cruise hold apply.
// ===========================================================================

void scenarioInfiniteRemainingDisablesDecelTaper() {
  beginScenario("VelocityShaper::next(): remaining == +infinity disables the decel taper entirely (Kind::Time)");

  float inf = std::numeric_limits<float>::infinity();
  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/300.0f, /*currentSpeed=*/300.0f,
                                             /*remaining=*/inf, /*dt=*/0.02f,
                                             /*aMax=*/500.0f, /*aDecel=*/800.0f);
  checkFloatEq(next, 300.0f, "holds cruiseSpeed under infinite remaining -- taper never binds");

  float rampNext = Motion::VelocityShaper::next(/*cruiseSpeed=*/300.0f, /*currentSpeed=*/0.0f,
                                                 /*remaining=*/inf, /*dt=*/0.02f,
                                                 /*aMax=*/500.0f, /*aDecel=*/800.0f);
  checkFloatEq(rampNext, 10.0f, "accel ramp still applies under infinite remaining -- only the taper is disabled");
}

// ===========================================================================
// 10. dt == 0 -- no accel-side movement, decel taper still evaluated.
// ===========================================================================

void scenarioZeroDtNoAccelMovement() {
  beginScenario("VelocityShaper::next(): dt == 0 -- no accel-side movement this tick");

  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/300.0f, /*currentSpeed=*/100.0f,
                                             /*remaining=*/10000.0f, /*dt=*/0.0f,
                                             /*aMax=*/500.0f, /*aDecel=*/800.0f);
  checkFloatEq(next, 100.0f, "dt == 0 -- candidate stays at currentSpeed (accel step is 0)");
}

// ===========================================================================
// 11. Negative aMax/aDecel (malformed input) are treated as their own
//     magnitude, matching stop_condition.cpp's own defense-in-depth
//     posture for malformed scalar inputs.
// ===========================================================================

void scenarioNegativeAccelMagnitudesTreatedAsMagnitude() {
  beginScenario("VelocityShaper::next(): negative aMax/aDecel are treated as |aMax|/|aDecel|");

  float next = Motion::VelocityShaper::next(/*cruiseSpeed=*/300.0f, /*currentSpeed=*/0.0f,
                                             /*remaining=*/10000.0f, /*dt=*/0.02f,
                                             /*aMax=*/-500.0f, /*aDecel=*/800.0f);
  checkFloatEq(next, 10.0f, "negative aMax behaves identically to +500 (magnitude, not signed rate)");
}

}  // namespace

int main() {
  scenarioAccelClampFromStandstill();
  scenarioAccelClampNeverOvershootsCruise();
  scenarioHoldsCruiseAtSteadyState();
  scenarioDecelTaperCapsBelowCruiseNearGoal();
  scenarioTaperMonotonicallyDecreasesWithRemaining();
  scenarioZeroRemainingCapsToZero();
  scenarioNegativeRemainingClampsToZeroSameAsZero();
  scenarioNegativeCruiseTapersSymmetrically();
  scenarioNegativeCruiseRampsUpFromZeroNegatively();
  scenarioCurrentAboveCruiseRampsDownGracefully();
  scenarioInfiniteRemainingDisablesDecelTaper();
  scenarioZeroDtNoAccelMovement();
  scenarioNegativeAccelMagnitudesTreatedAsMagnitude();

  if (g_failureCount == 0) {
    std::printf("OK: all Motion::VelocityShaper scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Motion::VelocityShaper scenarios\n", g_failureCount);
  return 1;
}
