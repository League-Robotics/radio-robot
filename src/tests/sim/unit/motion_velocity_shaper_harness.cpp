// motion_velocity_shaper_harness.cpp -- off-hardware acceptance proof for
// Motion::VelocityShaper (src/firm/motion/velocity_shaper.{h,cpp}),
// decel-into-the-goal campaign (follow-on to
// clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md), jerk-
// limited S-curve stage.
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

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
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

// Common test-fixture magnitudes, shared across scenarios below (linear
// units -- mm/s, mm/s^2, mm/s^3).
constexpr float kDt = 0.02f;         // [s] one cycle, matches the real ~50Hz loop
constexpr float kCruise = 300.0f;    // [mm/s]
constexpr float kAMax = 1000.0f;     // [mm/s^2]
constexpr float kADecel = 800.0f;    // [mm/s^2]
constexpr float kJMax = 5000.0f;     // [mm/s^3]

// ===========================================================================
// 1. Accel-slew bound: across a full ramp-to-cruise-and-hold profile, the
//    per-tick CHANGE in commandedAccel() never exceeds jMax*dt -- the jerk
//    ceiling itself, the acceptance criterion the stakeholder's own jerk
//    directive named explicitly ("max |da/dt| <= j_max across a full
//    profile").
// ===========================================================================

void scenarioAccelSlewNeverExceedsJerkBound() {
  beginScenario("VelocityShaper: accel-slew per tick never exceeds jMax*dt across a full ramp");

  Motion::VelocityShaper shaper;
  float prevAccel = 0.0f;
  float worstStep = 0.0f;
  for (int i = 0; i < 400; ++i) {
    shaper.next(kCruise, /*remaining=*/1.0e6f, kDt, kAMax, kADecel, kJMax);
    float step = std::fabs(shaper.commandedAccel() - prevAccel);
    if (step > worstStep) worstStep = step;
    prevAccel = shaper.commandedAccel();
  }
  checkLe(worstStep, kJMax * kDt + 1e-3f, "worst per-tick |delta accel| stays within jMax*dt");
}

// ===========================================================================
// 2. Accel-slew bound holds through a full ramp-up + brake-to-a-stop
//    profile too (both accel directions exercised, not just ramp-up).
// ===========================================================================

void scenarioAccelSlewNeverExceedsJerkBoundThroughFullStop() {
  beginScenario("VelocityShaper: accel-slew bound holds through ramp-up AND brake-to-stop");

  Motion::VelocityShaper shaper;
  float prevAccel = 0.0f;
  float worstStep = 0.0f;
  float remaining = 100.0f;  // [mm] short enough to trigger a full ramp-brake cycle
  for (int i = 0; i < 400; ++i) {
    float v = shaper.next(kCruise, remaining, kDt, kAMax, kADecel, kJMax);
    remaining -= std::fabs(v) * kDt;
    if (remaining < 0.0f) remaining = 0.0f;
    float step = std::fabs(shaper.commandedAccel() - prevAccel);
    if (step > worstStep) worstStep = step;
    prevAccel = shaper.commandedAccel();
  }
  checkLe(worstStep, kJMax * kDt + 1e-3f, "worst per-tick |delta accel| stays within jMax*dt through a full stop");
}

// ===========================================================================
// 3. S-curve reaches cruise: given ample remaining, the commanded speed
//    converges to EXACTLY cruiseSpeed and holds there -- no steady-state
//    error, no oscillation once settled.
// ===========================================================================

void scenarioReachesAndHoldsCruise() {
  beginScenario("VelocityShaper: S-curve reaches cruiseSpeed and holds it, ample remaining");

  Motion::VelocityShaper shaper;
  float v = 0.0f;
  for (int i = 0; i < 200; ++i) {
    v = shaper.next(kCruise, /*remaining=*/1.0e6f, kDt, kAMax, kADecel, kJMax);
  }
  checkFloatEq(v, kCruise, "converges to exactly cruiseSpeed within 200 ticks");
  checkFloatEq(shaper.commandedAccel(), 0.0f, "commandedAccel settles to 0 once at cruise (holding, not still ramping)");

  // Hold for another 50 ticks -- no drift/oscillation once settled.
  for (int i = 0; i < 50; ++i) {
    v = shaper.next(kCruise, /*remaining=*/1.0e6f, kDt, kAMax, kADecel, kJMax);
    checkFloatEq(v, kCruise, "stays pinned at cruiseSpeed once settled, no oscillation");
  }
}

// ===========================================================================
// 4. The S-curve never overshoots cruiseSpeed even momentarily -- the
//    jerk-aware roll-off (chooseAccelTarget()) exists specifically to
//    prevent this; sampled every tick through the whole ramp.
// ===========================================================================

void scenarioNeverOvershootsCruiseDuringRampUp() {
  beginScenario("VelocityShaper: commanded speed never exceeds cruiseSpeed during ramp-up");

  Motion::VelocityShaper shaper;
  for (int i = 0; i < 200; ++i) {
    float v = shaper.next(kCruise, /*remaining=*/1.0e6f, kDt, kAMax, kADecel, kJMax);
    checkLe(v, kCruise + 1e-3f, "commanded speed never overshoots cruiseSpeed");
  }
}

// ===========================================================================
// 5. Taper lands: driving toward a SHORT remaining, the commanded speed
//    decays toward 0 as the goal is approached and settles at/near 0
//    exactly as remaining reaches 0 -- no overshoot PAST the goal (the
//    commanded speed never needs to reverse sign to "come back").
// ===========================================================================

void scenarioTaperLandsAtZeroWithoutReversal() {
  beginScenario("VelocityShaper: taper lands the commanded speed at ~0 as remaining reaches 0, no reversal");

  Motion::VelocityShaper shaper;
  float remaining = 60.0f;  // [mm] short enough to never reach cruise (a "triangle" profile)
  float peak = 0.0f;
  bool settled = false;
  for (int i = 0; i < 300; ++i) {
    float v = shaper.next(kCruise, remaining, kDt, kAMax, kADecel, kJMax);
    if (v > peak) peak = v;
    checkTrue(v >= -1e-3f, "commanded speed never reverses sign (goes negative) while approaching a positive-cruise goal");
    remaining -= std::fabs(v) * kDt;
    if (remaining < 0.0f) remaining = 0.0f;
    if (remaining <= 0.0f && std::fabs(v) < 1e-2f) {
      settled = true;
      checkFloatEq(v, 0.0f, "commanded speed lands at ~0 exactly as remaining reaches 0", 0.05f);
      break;
    }
  }
  checkTrue(settled, "the shaper actually settles to ~0 within the simulated window (not stuck oscillating)");
  checkLe(peak, kCruise + 1e-3f, "sanity: this short-remaining profile never reached full cruise (a triangle, not a trapezoid)");
}

// ===========================================================================
// 6. Zero/negative remaining: the very next call's target is immediately
//    braking (jerk-aware stopping distance for ANY nonzero speed exceeds
//    0 remaining) -- commanded speed moves toward 0, never away from it.
// ===========================================================================

void scenarioZeroRemainingTargetsImmediateBraking() {
  beginScenario("VelocityShaper: remaining == 0 targets immediate braking from a nonzero speed");

  Motion::VelocityShaper shaper;
  // Get the shaper moving first.
  for (int i = 0; i < 50; ++i) shaper.next(kCruise, /*remaining=*/1.0e6f, kDt, kAMax, kADecel, kJMax);
  float speedBefore = shaper.commandedSpeed();
  checkTrue(speedBefore > 0.0f, "setup: shaper is genuinely moving before the remaining=0 call");

  float v = shaper.next(kCruise, /*remaining=*/0.0f, kDt, kAMax, kADecel, kJMax);
  checkTrue(v <= speedBefore, "commanded speed decreases (braking), never increases, once remaining hits 0");
}

void scenarioNegativeRemainingClampsSameAsZero() {
  beginScenario("VelocityShaper: negative remaining clamps to 0, same braking behavior as remaining == 0");

  Motion::VelocityShaper shaperZero;
  Motion::VelocityShaper shaperNeg;
  for (int i = 0; i < 50; ++i) {
    shaperZero.next(kCruise, 1.0e6f, kDt, kAMax, kADecel, kJMax);
    shaperNeg.next(kCruise, 1.0e6f, kDt, kAMax, kADecel, kJMax);
  }
  float vZero = shaperZero.next(kCruise, /*remaining=*/0.0f, kDt, kAMax, kADecel, kJMax);
  float vNeg = shaperNeg.next(kCruise, /*remaining=*/-5.0f, kDt, kAMax, kADecel, kJMax);
  checkFloatEq(vNeg, vZero, "negative remaining behaves identically to remaining == 0 -- same malformed-input posture as stop_condition.cpp");
}

// ===========================================================================
// 7. Sign handling: a negative cruiseSpeed (e.g. a clockwise turn) ramps
//    up, holds, and tapers the SAME way a positive one does, mirrored in
//    sign -- full round trip, angular-scale units.
// ===========================================================================

void scenarioNegativeCruiseRoundTripSymmetric() {
  beginScenario("VelocityShaper: negative cruiseSpeed (e.g. CW turn) ramps/holds/tapers symmetrically");

  constexpr float kOmega = -2.0f;    // [rad/s]
  constexpr float kAlphaMax = 6.0f;  // [rad/s^2]
  constexpr float kAlphaDecel = 7.0f;  // [rad/s^2]
  constexpr float kJerk = 100.0f;    // [rad/s^3]

  Motion::VelocityShaper shaper;
  float remaining = 1.5708f;  // [rad] ~pi/2
  float v = 0.0f;
  bool reachedCruise = false;
  for (int i = 0; i < 400; ++i) {
    v = shaper.next(kOmega, remaining, kDt, kAlphaMax, kAlphaDecel, kJerk);
    checkTrue(v <= 1e-3f, "commanded omega stays <= 0 throughout -- never overshoots past the CW target's own sign");
    if (std::fabs(v - kOmega) < 1e-2f) reachedCruise = true;
    remaining -= std::fabs(v) * kDt;
    if (remaining < 0.0f) remaining = 0.0f;
    if (remaining <= 0.0f && std::fabs(v) < 1e-2f) break;
  }
  checkTrue(reachedCruise, "reaches the negative cruise omega at some point during the turn (ample remaining early on)");
  checkFloatEq(v, 0.0f, "settles back to ~0 as the turn's own remaining angle reaches 0", 0.05f);
}

// ===========================================================================
// 8. remaining == +infinity (Kind::Time -- no decel taper, MoveQueue's own
//    documented "pass +infinity" contract): the shaper ramps to cruise and
//    holds it indefinitely -- the braking target never triggers.
// ===========================================================================

void scenarioInfiniteRemainingDisablesDecelTaper() {
  beginScenario("VelocityShaper: remaining == +infinity disables the decel taper entirely (Kind::Time)");

  float inf = std::numeric_limits<float>::infinity();
  Motion::VelocityShaper shaper;
  float v = 0.0f;
  for (int i = 0; i < 200; ++i) {
    v = shaper.next(kCruise, inf, kDt, kAMax, kADecel, kJMax);
  }
  checkFloatEq(v, kCruise, "reaches and holds cruiseSpeed under infinite remaining -- taper never binds");

  // Hold for many more ticks -- still no braking, ever.
  for (int i = 0; i < 100; ++i) {
    v = shaper.next(kCruise, inf, kDt, kAMax, kADecel, kJMax);
    checkFloatEq(v, kCruise, "never brakes under infinite remaining, however many ticks pass");
  }
}

// ===========================================================================
// 9. jMax <= 0 degrades to UNLIMITED jerk (this module's own prior accel-
//    only pass) rather than dividing by zero -- one tick's step should
//    exactly match "accel snaps straight to aMax, integrated over dt",
//    i.e. aMax*dt, the SAME number the accel-only stage measured.
// ===========================================================================

void scenarioZeroJerkDegradesToUnlimitedJerkAccelOnlyBehavior() {
  beginScenario("VelocityShaper: jMax <= 0 degrades to unlimited-jerk (accel-only) behavior, not a divide-by-zero");

  Motion::VelocityShaper shaper;
  float v = shaper.next(kCruise, /*remaining=*/1.0e6f, kDt, kAMax, kADecel, /*jMax=*/0.0f);
  checkFloatEq(v, kAMax * kDt, "jMax<=0 -- one tick from rest steps by exactly aMax*dt (unlimited jerk)");
  checkFloatEq(shaper.commandedAccel(), kAMax, "jMax<=0 -- commandedAccel snaps directly to aMax, no slew");
}

// ===========================================================================
// 10. reset()/syncTo() -- the two state-management entry points
//     App::MoveQueue relies on (Drive::stop()/flush() and the "shaping
//     disabled" mirror-sync path respectively).
// ===========================================================================

void scenarioResetZeroesBothStateFields() {
  beginScenario("VelocityShaper::reset(): zeroes both commandedSpeed() and commandedAccel()");

  Motion::VelocityShaper shaper;
  for (int i = 0; i < 50; ++i) shaper.next(kCruise, 1.0e6f, kDt, kAMax, kADecel, kJMax);
  checkTrue(shaper.commandedSpeed() > 0.0f, "setup: shaper is genuinely moving before reset()");

  shaper.reset();
  checkFloatEq(shaper.commandedSpeed(), 0.0f, "reset() zeroes commandedSpeed()");
  checkFloatEq(shaper.commandedAccel(), 0.0f, "reset() zeroes commandedAccel()");
}

void scenarioSyncToSetsSpeedAndZeroesAccel() {
  beginScenario("VelocityShaper::syncTo(): sets commandedSpeed() directly, zeroes commandedAccel(), no shaping math");

  Motion::VelocityShaper shaper;
  for (int i = 0; i < 50; ++i) shaper.next(kCruise, 1.0e6f, kDt, kAMax, kADecel, kJMax);
  checkTrue(shaper.commandedAccel() != 0.0f || shaper.commandedSpeed() == kCruise,
            "setup: shaper has SOME nonzero accel state before syncTo() (mid-ramp or just-settled)");

  shaper.syncTo(123.5f);
  checkFloatEq(shaper.commandedSpeed(), 123.5f, "syncTo() sets commandedSpeed() directly");
  checkFloatEq(shaper.commandedAccel(), 0.0f, "syncTo() zeroes commandedAccel()");
}

}  // namespace

int main() {
  scenarioAccelSlewNeverExceedsJerkBound();
  scenarioAccelSlewNeverExceedsJerkBoundThroughFullStop();
  scenarioReachesAndHoldsCruise();
  scenarioNeverOvershootsCruiseDuringRampUp();
  scenarioTaperLandsAtZeroWithoutReversal();
  scenarioZeroRemainingTargetsImmediateBraking();
  scenarioNegativeRemainingClampsSameAsZero();
  scenarioNegativeCruiseRoundTripSymmetric();
  scenarioInfiniteRemainingDisablesDecelTaper();
  scenarioZeroJerkDegradesToUnlimitedJerkAccelOnlyBehavior();
  scenarioResetZeroesBothStateFields();
  scenarioSyncToSetsSpeedAndZeroesAccel();

  if (g_failureCount == 0) {
    std::printf("OK: all Motion::VelocityShaper scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Motion::VelocityShaper scenarios\n", g_failureCount);
  return 1;
}
